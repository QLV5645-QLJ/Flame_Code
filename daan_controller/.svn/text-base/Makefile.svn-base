BINARIES     = Flame_core Flame_core_helper
FILES        = 

INSTALLED_FILES=$(BINARIES:%=installed-files/%) $(FILES:%=installed-files/%)

RTAI_INCLUDES = -I/usr/realtime/include
RTAI_LIBS     = -L/usr/realtime/lib/ -llxrt -lpthread -lm

# uncomment this two variables to enable memory allocation debugging
# DEBUG_LIBS= -ldmalloc
# DEBUG_CFLAGS= -DUSE_DMALLOC

INCLUDES     = -I.. -I. ${RTAI_INCLUDES}

FLAME_LIBS   = ${DEBUG_LIBS} -L../real_time_support -L../hardware_drivers -L../utility -L../xsens \
				-lflameio -lrealtime -lutility -lxsens

CFLAGS       = ${DEBUG_CFLAGS} -g3 -O2

LIBDEPENDS   = ../real_time_support/librealtime.a ../hardware_drivers/libflameio.a ../utility/libutility.a

################################################################
# The default entry builds all the programs.
# The host-side console is built in a subdirectory.

default:
	make $(BINARIES)
	( cd console ; make )

################################################################
# Specify all the components of the real-time control program.

CONTROLLER_OBJS = \
	Flame_core.o \
	CtrlFlame.o \
	CtrlExercise.o \
	CtrlStanding.o \
	CtrlWalking.o \
	CtrlJoints.o \
	FlameJoints.o \
	polynomials.o \
	LookupTables.o \
	StateMachines.o \

# The non-real-time Flame_core program uses a C++ library, so it needs the C++ linker:
Flame_core: $(CONTROLLER_OBJS) $(LIBDEPENDS)
	g++ -o $@ $(CFLAGS) $(CONTROLLER_OBJS)  ${FLAME_LIBS} ${RTAI_LIBS}


Flame_core_helper: Flame_core_helper.o $(LIBDEPENDS)
	g++ -o $@ $< $(CFLAGS) ${FLAME_LIBS} ${RTAI_LIBS}

################################################################
# This copies every modified program to the target, keeping a cached copy 
# locally for the time stamp.
install: installed-files $(INSTALLED_FILES)

installed-files/% : % 
	( scp $^ root@robot:/usr/local/bin && cp $^ $@ )

installed-files/PARAMS : PARAMS
	( scp $^ root@robot:/root && cp $^ $@ )

# create the cache directory if it doesn't exist
installed-files:
	mkdir $@

################################################################
# auto-generated source
#system.h sysvars.h: makevars.scm
#	guile -e main -s makevars.scm

# a few manual dependencies to overcome limitations of makedepend
Flame_core.o: local_protocol_version.h

# A time stamp to help ensure the RT and UI are in sync.  This
# requires the GNU version of 'date'.
local_protocol_version.h: message_format.h
	echo -n "#define LOCAL_MESSAGE_PROTOCOL_VERSION " > local_protocol_version.h
	date +%s >>local_protocol_version.h

dist-clean: clean
	-rm -r local_protocol_version.h installed-files

################################################################
# default rules

%.o : %.cpp
	g++ -c $< $(CFLAGS) ${INCLUDES}

% : %.o $(LIBDEPENDS)
	g++ -o $@ $< $(CFLAGS) ${FLAME_LIBS} ${RTAI_LIBS}

% : %.cpp $(LIBDEPENDS)
	g++ -o $@ $< $(CFLAGS) ${INCLUDES} ${FLAME_LIBS} ${RTAI_LIBS}

clean:
	-rm *.o $(BINARIES)
	( cd console ; make clean )

################################################################
# automatic dependencies

DEPENDS = Makefile-dependencies

depend:
	makedepend -f$(DEPENDS) -I.. -I. -Y -- *.cpp *.h 

include $(DEPENDS)
