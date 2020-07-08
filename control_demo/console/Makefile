BINARIES     = control_demo_console

INCLUDES     = -I../..
FLAME_LIBS   = -L../../real_time_support -L../../utility -lrealtime -lutility -lncurses -lm
CFLAGS       = -g -O2

LIBDEPENDS   = ../../real_time_support/librealtime.a ../../utility/libutility.a

# The default entry builds all the programs
default: $(BINARIES)

################################################################
# default rules

%.o : %.c
	$(CC) -c $< $(CFLAGS) ${INCLUDES}

% : %.o $(LIBDEPENDS)
	$(CC) -o $@ $< ${FLAME_LIBS}

% : %.c $(LIBDEPENDS)
	$(CC) -o $@ $< $(CFLAGS) ${INCLUDES} ${FLAME_LIBS}

clean:
	-rm *.o $(BINARIES)

################################################################
# automatic dependencies
depend:
	makedepend -I.. -fMakefile -Y -- *.c 

# DO NOT DELETE

