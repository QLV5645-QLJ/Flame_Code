# $Id: Makefile,v 1.4 2005/12/14 10:18:44 garthz Exp $
# Makefile : for the interface routines to the real time system interfaces
# Copyright (c) 2001-2005 Garth Zeglin. Provided under the terms of the
# GNU General Public License as included in the top level directory.

LIBOBJS =	RTAI_user_space_realtime.o POSIX_soft_realtime.o \
		messaging.o RTAI_mailbox_messaging.o UDP_messaging.o

default: librealtime.a

librealtime.a: $(LIBOBJS)
	ar cru $@ $^

INCLUDES = -I.. -I/usr/realtime/include
CFLAGS = -g3 -O2 


################################################################
# A time stamp to help ensure the RT and UI are in sync.  This
# requires the GNU version of 'date'.
protocol_version.h: messages.h messaging.cpp
	echo -n "#define MESSAGE_PROTOCOL_VERSION " > protocol_version.h
	date +%s >>protocol_version.h

../real_time_support/protocol_version.h: protocol_version.h

################################################################

%.o : %.cpp
	g++ -c $< $(CFLAGS) ${INCLUDES}

clean:
	-rm *.o librealtime.a

dist-clean: clean
	-rm protocol_version.h

################################################################
# automatic dependencies
depend:
	makedepend -I.. -fMakefile -Y -- *.cpp 

# DO NOT DELETE

messaging.o: ../real_time_support/messaging.h ../real_time_support/messages.h
messaging.o: ../real_time_support/protocol_version.h ../utility/utility.h
POSIX_soft_realtime.o: ../utility/utility.h
RTAI_user_space_realtime.o: ../utility/utility.h
RTAI_user_space_realtime.o: ../real_time_support/messaging.h
RTAI_user_space_realtime.o: ../real_time_support/messages.h
