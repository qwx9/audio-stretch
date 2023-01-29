</$objtype/mkfile
TARG=stretch
BIN=/$objtype/bin/audio

OFILES=\
	this.$O\
	stretch.$O\

HFILES=\
	stretch.h\

default:V: all

</sys/src/cmd/mkone

CFLAGS=$CFLAGS -p -I/sys/include/npe -D__plan9__ -D__${objtype}__\
	-D_POSIX_SOURCE
