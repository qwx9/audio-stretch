#include <u.h>
#include <libc.h>
#include <thread.h>
#include "stretch.h"

enum{
	Nrate = 44100,
	Nchan = 2,
	Sampsz = Nchan * 2,
};

void
usage(void)
{
	fprint(2, "usage: %s [OPTIONS] [FILE]\n"
		"-r F  stretch ratio, [0.25,4.0] default 1.0\n"
		"-g F  gap/silence stretch ratio (if different)\n"
		"-u N  upper freq period limit [40,44100[ default 333 Hz\n"
		"-l N  lower freq period limit, [20,[ default 55 Hz\n"
		"-b N  audio buffer/window length, [1,100] default 25 ms\n"
		"-t N  gap/silence threshold (dB re FS, default -40)\n"
		"-c	cycle through all ratios, starting higher\n"
		"-C	cycle through all ratios, starting lower\n"
		"-d	force dual instance even for shallow ratios\n"
		"-s	scale rate to preserve duration (not pitch)\n"
		"-f	fast pitch detection (default >= 32 kHz)\n"
		"-n	normal pitch detection (default < 32 kHz)\n",
		argv0);
	exits("usage");
}

double
rms_level_dB (int16_t *audio, int samples, int channels)
{
	double rms_sum = 0.0;
	int i;

	if (channels == 1)
		for (i = 0; i < samples; ++i)
			rms_sum += (double) audio [i] * audio [i];
	else
		for (i = 0; i < samples; ++i) {
			double average = (audio [i * 2] + audio [i * 2 + 1]) / 2.0;
			rms_sum += average * average;
		}

	return log10 (rms_sum / samples / (32768.0 * 32767.0 * 0.5)) * 10.0;
}

void
threadmain(int argc, char **argv)
{
	int n, m, fd, ofd, pfd[2], cycle, dual, fast, normal, doscale, lf, uf, wsz, flags, maxnsamp, silence, nibuf, min_period, max_period, non_silence_frames, silence_frames, used_silence_frames, max_generated_stretch, max_generated_flush, samples_to_stretch, consecutive_silence_frames, verbose;
	char *r;
	s16int *prebuf, *ibuf, *obuf;
	u32int insamp, outsamp;
	float max_ratio;
	double level, ratio, gap, smin;
	StretchHandle S;

	fd = 0;
	ofd = 1;
	cycle = 0;	/* cycle ratio, direction, 1: fw, 2: back */
	dual = 0;	/* force dual instance */
	fast = 0;	/* force fast pitch detection */
	normal = 0;	/* force normal pitch detection */
	doscale = 0;	/* scale rate to preserve duration */
	lf = 55;	/* freq lower bound */
	uf = 333;	/* freq upper bound */
	gap = 0.0;	/* gap/silence stretch ratio */
	ratio = 1.0;	/* stretch ratio */
	wsz = 25;	/* window size */
	smin = -40;	/* gap/silence threshold */
	verbose = 0;
	ARGBEGIN{
	case 'C': cycle = 2; break;
	case 'c': cycle = 1; break;
	case 'd': dual = 1; break;
	case 'f': fast = 1; break;
	case 'g': gap = strtod(EARGF(usage()), nil); if(gap < 0.25 || gap > 4.0) usage(); break;
	case 'l': lf = strtol(EARGF(usage()), nil, 10); if(lf < 20) usage(); break;
	case 'n': normal = 1; break;
	case 'r': ratio = strtod(EARGF(usage()), nil); if(ratio < 0.25 || ratio > 4.0) usage(); break;
	case 's': doscale = 1; break;
	case 't': smin = strtod(EARGF(usage()), nil); if(smin < -70 || smin > -10) usage(); break;
	case 'u': uf = strtol(EARGF(usage()), nil, 10); if(uf < 40) usage(); break;
	case 'v': verbose = 1; break;
	case 'w': wsz = strtol(EARGF(usage()), nil, 10); if(wsz < 1 || wsz > 100) usage(); break;
	case 'h':
	default: usage(); break;
	}ARGEND
	if(*argv != nil && (fd = open(*argv, OREAD)) < 0)
		usage();

	insamp = 0;
	outsamp = 0;
	min_period = Nrate / uf;
	max_period = Nrate / lf;
	flags = 0;
	silence = gap != 0.0 && gap != ratio && !cycle;
	nibuf = Nrate * (wsz / 1000.0);
	max_ratio = ratio;
	if(dual || ratio < 0.5 || ratio > 2.0 ||
		(silence && (gap < 0.5 || gap > 2.0)))
		flags |= STRETCH_DUAL_FLAG;
	if((fast || Nrate >= 32000) && !normal)
		flags |= STRETCH_FAST_FLAG;
	if(verbose){
		fprint(2, "file sample rate is %d Hz (%s), buffer size is %d samples\n",
			Nrate, Nchan == 2 ? "stereo" : "mono", nibuf);
		fprint(2, "stretch period range = %d to %d, %d channels, %s, %s\n",
			min_period, max_period, 2, (flags & STRETCH_FAST_FLAG) ? "fast mode" : "normal mode",
			(flags & STRETCH_DUAL_FLAG) ? "dual instance" : "single instance");
	}
	if((S = stretch_init(min_period, max_period, Nchan, flags)) == NULL)
		sysfatal("initialization failed");
	if(cycle)
		max_ratio = (flags & STRETCH_DUAL_FLAG) ? 4.0 : 2.0;
	else if(silence && gap > max_ratio)
		max_ratio = gap;
	maxnsamp = stretch_output_capacity(S, nibuf, max_ratio);
	obuf = prebuf = nil;
	if((ibuf = malloc(nibuf * Sampsz)) == nil
	|| (obuf = malloc(maxnsamp * Sampsz)) == nil
	|| silence && (prebuf = malloc(nibuf * Sampsz)) == nil)
		sysfatal("malloc: %r");
	non_silence_frames = 0,
	silence_frames = 0,
	used_silence_frames = 0;
	max_generated_stretch = 0,
	max_generated_flush = 0;
	samples_to_stretch = 0,
	consecutive_silence_frames = 1;

	if(doscale){
		if(pipe(pfd) < 0)
			sysfatal("pipe: %r");
		switch(rfork(RFPROC|RFFDG)){
		case -1:
			sysfatal("rfork: %r");
		case 0:
			close(0);
			close(pfd[1]);
			dup(pfd[0], 0);
			if((r = smprint("s16c2r%d", (u32int)(Nrate * ratio + 0.5))) == nil)
				sysfatal("smprint: %r");
			execl("/bin/audio/pcmconv", "pcmconv", "-i", r, "-o", "s16c2r44100", nil);
			sysfatal("execl: %r");
		default:
			close(1);
			close(pfd[0]);
			ofd = pfd[1];
		}
	}
	for(;;){
		n = read(fd, silence ? prebuf : ibuf, Sampsz * nibuf);
		n /= Sampsz;
		if(n < 0)
			sysfatal("read: %r");
		if(!silence && n == 0)
			break;
		insamp += n;
		if(silence){
			if(n != 0){
				level = rms_level_dB(prebuf, n, Nchan);
				if(level > smin){
					consecutive_silence_frames = 0;
					non_silence_frames++;
				}
				else{
					consecutive_silence_frames++;
					silence_frames++;
				}
			}
		}
		else
			samples_to_stretch = n;
		if(cycle){
			if(flags & STRETCH_DUAL_FLAG)
				ratio = (sin((double) outsamp / Nrate / 2.0) *(cycle & 1 ? 1.875 : -1.875)) + 2.125;
			else
				ratio = (sin((double) outsamp / Nrate) * (cycle & 1 ? 0.75 : -0.75)) + 1.25;
		}
		if(samples_to_stretch){
			if(consecutive_silence_frames >= 3){
				m = stretch_samples(S, ibuf, samples_to_stretch, obuf, gap);
				used_silence_frames++;
			}else
				m = stretch_samples(S, ibuf, samples_to_stretch, obuf, ratio);
			if(m){
				if(m > max_generated_stretch)
					max_generated_stretch = m;
				write(ofd, obuf, Sampsz * m);
				outsamp += m;
				if(m > maxnsamp)
					sysfatal("sample generation overflow");
			}
		}
		if(silence){
			if(n){
				memcpy(ibuf, prebuf, n * Sampsz);
				samples_to_stretch = n;
			}else
				break;
		}
	}
	for(;;){
		if((n = stretch_flush(S, obuf)) == 0)
			break;
		if(n > max_generated_flush)
			max_generated_flush = n;
		if(n > maxnsamp)
			sysfatal("flush overflow");
		write(ofd, obuf, Sampsz * n);
		outsamp += n / Sampsz;
	}
    if(insamp && verbose){
        fprint(2, "done, %ud samples --> %ud samples (ratio = %.3f)\n",
            insamp, outsamp, (double)outsamp / insamp);
        if(doscale)
            fprint(2, "sample rate changed from %d Hz to %ud Hz\n",
                Nrate, (u32int)(Nrate * ratio + 0.5));
        fprint(2, "max expected samples = %d, actually seen = %d stretch, %d flush\n",
            maxnsamp, max_generated_stretch, max_generated_flush);
        if(silence_frames || non_silence_frames) {
            int total_frames = silence_frames + non_silence_frames;
            fprint(2, "%d silence frames detected (%.2f%%), %d actually used (%.2f%%)\n",
                silence_frames, silence_frames * 100.0 / total_frames,
                used_silence_frames, used_silence_frames * 100.0 / total_frames); 
        }
    }
	stretch_deinit(S);
	exits(nil);
}
