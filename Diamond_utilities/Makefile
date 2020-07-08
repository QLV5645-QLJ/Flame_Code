# This directory contains programs linked against the Diamond System library.

BINARIES     = calibrate_DMM16AT test_DMM16AT test_AthenaDAQ

INSTALLED_BINARIES=$(BINARIES:%=installed-files/%)

INCLUDES     = -I.. -I/usr/local/lib/dscud5
FLAME_LIBS   = -L/usr/local/lib/dscud5 -ldscud5 -lpthread -lm
CFLAGS       = -g3 -O2

# The default entry builds all the programs
default: $(BINARIES)

# This copies every modified program to the target, keeping a cached copy 
# locally for the time stamp.
install: installed-files $(INSTALLED_BINARIES)

installed-files/% : % 
	( scp $^ root@robot:/usr/local/bin && cp $^ $@ )

# create the cache directory if it doesn't exist
installed-files:
	mkdir $@

################################################################
test_AthenaDAQ: test_AthenaDAQ.o
	$(CC) -o $@ $< ${FLAME_LIBS} -L../real_time_support -lrealtime -L../utility -lutility

################################################################
# default rules

%.o : %.c
	$(CC) -c $< $(CFLAGS) ${INCLUDES}

% : %.o
	$(CC) -o $@ $< ${FLAME_LIBS}

% : %.c
	$(CC) -o $@ $< $(CFLAGS) ${INCLUDES} ${FLAME_LIBS}

clean:
	-rm *.o $(BINARIES)

dist-clean: clean
	-rm -r installed-files

################################################################
# automatic dependencies
depend:
	makedepend -I.. -fMakefile -Y -- *.c 

# DO NOT DELETE
