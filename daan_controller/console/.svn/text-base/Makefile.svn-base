BINARIES     = Flame_console

INCLUDES     = -I../..
FLAME_LIBS   = -L../../real_time_support -L../../utility -lrealtime -lutility -lncurses -lm
CFLAGS       = -g3 -O0

LIBDEPENDS   = ../../real_time_support/librealtime.a ../../utility/libutility.a

# The default entry builds all the programs
default: $(BINARIES)

################################################################
# default rules

%.o : %.cpp
	g++ -c $< $(CFLAGS) ${INCLUDES}

% : %.o $(LIBDEPENDS)
	g++ -o $@ $< ${FLAME_LIBS}

% : %.cpp $(LIBDEPENDS)
	g++ -o $@ $< $(CFLAGS) ${INCLUDES} ${FLAME_LIBS}

clean:
	-rm *.o $(BINARIES)

################################################################
# automatic dependencies
depend:
	makedepend -I.. -fMakefile -Y -- *.cpp

# DO NOT DELETE

