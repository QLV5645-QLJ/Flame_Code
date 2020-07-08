BINARIES =	DMM16AT_test \
		sensor_console \
		AthenaDAQ_test \
		FlameIO_test \
		Flame_off \
		Flame_wakeup \

# these are probably obsolete
#   rt_analog_output test_user_space_realtime test_mailbox_messaging
#   test_sensor_logging 
#		next_filename \
#		test_saving
# 		UDP_messaging_test \

INSTALLED_BINARIES=$(BINARIES:%=installed-files/%)

RTAI_INCLUDES = -I/usr/realtime/include
RTAI_LIBS     = -L/usr/realtime/lib/ -llxrt -lpthread -lm

INCLUDES     = -I.. ${RTAI_INCLUDES}
FLAME_LIBS   = -L../real_time_support -L../hardware_drivers -L../utility -lflameio -lrealtime -lutility
CFLAGS       = -g3 -O2

LIBDEPENDS   = ../real_time_support/librealtime.a ../hardware_drivers/libflameio.a ../utility/libutility.a

# The default entry builds all the programs
default: $(BINARIES)

# This copies every modified program to the target, keeping a cached copy 
# locally for the time stamp.
install: installed-files $(INSTALLED_BINARIES)

installed-files/% : % 
	(scp $^ root@robot:/usr/local/bin  &&  cp $^ $@ )

# create the cache directory if it doesn't exist
installed-files:
	mkdir $@

################################################################

# the default rules take care of most of these compilations, but
# the following require individual entries for special options:

sensor_console: sensor_console.o
	g++ -o $@ $^ ${FLAME_LIBS} ${RTAI_LIBS} -lncurses

UDP_messaging_test: UDP_messaging_test.o ${LIBDEPENDS}
	g++ -o $@ $< ${FLAME_LIBS}

test_saving: test_saving.o ${LIBDEPENDS}
	g++ -o $@ $< ${FLAME_LIBS}

################################################################
# default rules

%.o : %.cpp
	g++ -c $< $(CFLAGS) ${INCLUDES}

% : %.o $(LIBDEPENDS)
	g++ -o $@ $< ${FLAME_LIBS} ${RTAI_LIBS}

% : %.cpp $(LIBDEPENDS)
	g++ -o $@ $< $(CFLAGS) ${INCLUDES} ${FLAME_LIBS} ${RTAI_LIBS}

clean:
	-rm *.o $(BINARIES)

dist-clean: clean
	-rm -r installed-files

################################################################
# automatic dependencies
depend:
	makedepend -I.. -fMakefile -Y -- *.cpp

# DO NOT DELETE

AthenaDAQ_test.o: ../hardware_drivers/AthenaDAQ.h
AthenaDAQ_test.o: ../hardware_drivers/IO_permissions.h
AthenaDAQ_test.o: ../real_time_support/POSIX_soft_realtime.h
AthenaDAQ_test.o: ../utility/utility.h
DMM16AT_test.o: ../hardware_drivers/FlameIO.h ../hardware_drivers/AthenaDAQ.h
DMM16AT_test.o: ../hardware_drivers/DMM16AT.h
DMM16AT_test.o: ../hardware_drivers/Mesanet_4I36.h
DMM16AT_test.o: ../hardware_drivers/FlameIO_defs.h
DMM16AT_test.o: ../hardware_drivers/IO_permissions.h
DMM16AT_test.o: ../real_time_support/POSIX_soft_realtime.h
DMM16AT_test.o: ../utility/utility.h
FlameIO_test.o: ../hardware_drivers/FlameIO.h ../hardware_drivers/AthenaDAQ.h
FlameIO_test.o: ../hardware_drivers/DMM16AT.h
FlameIO_test.o: ../hardware_drivers/Mesanet_4I36.h
FlameIO_test.o: ../hardware_drivers/FlameIO_defs.h ../utility/utility.h
Flame_off.o: ../hardware_drivers/FlameIO.h ../hardware_drivers/AthenaDAQ.h
Flame_off.o: ../hardware_drivers/DMM16AT.h ../hardware_drivers/Mesanet_4I36.h
Flame_off.o: ../hardware_drivers/FlameIO_defs.h ../utility/utility.h
Flame_wakeup.o: ../hardware_drivers/FlameIO.h ../hardware_drivers/AthenaDAQ.h
Flame_wakeup.o: ../hardware_drivers/DMM16AT.h
Flame_wakeup.o: ../hardware_drivers/Mesanet_4I36.h
Flame_wakeup.o: ../hardware_drivers/FlameIO_defs.h ../utility/utility.h
next_filename.o: ../utility/utility.h
rt_analog_output.o: ../real_time_support/RTAI_user_space_realtime.h
rt_analog_output.o: ../real_time_support/messages.h
rt_analog_output.o: ../hardware_drivers/FlameIO.h
rt_analog_output.o: ../hardware_drivers/AthenaDAQ.h
rt_analog_output.o: ../hardware_drivers/DMM16AT.h
rt_analog_output.o: ../hardware_drivers/Mesanet_4I36.h
rt_analog_output.o: ../hardware_drivers/FlameIO_defs.h
sensor_console.o: ../real_time_support/RTAI_user_space_realtime.h
sensor_console.o: ../real_time_support/RTAI_mailbox_messaging.h
sensor_console.o: ../real_time_support/messaging.h
sensor_console.o: ../real_time_support/messages.h
sensor_console.o: ../real_time_support/protocol_version.h
sensor_console.o: ../utility/utility.h ../hardware_drivers/DMM16AT.h
sensor_console.o: ../hardware_drivers/AthenaDAQ.h
sensor_console.o: ../hardware_drivers/Mesanet_4I36.h
sensor_console.o: ../hardware_drivers/IO_permissions.h
test_mailbox_messaging.o: ../real_time_support/RTAI_user_space_realtime.h
test_mailbox_messaging.o: ../real_time_support/RTAI_mailbox_messaging.h
test_mailbox_messaging.o: ../real_time_support/messaging.h
test_mailbox_messaging.o: ../real_time_support/messages.h
test_mailbox_messaging.o: ../real_time_support/protocol_version.h
test_mailbox_messaging.o: ../utility/utility.h
test_saving.o: ../utility/utility.h ../utility/system_state_var.h
test_sensor_logging.o: ../real_time_support/RTAI_user_space_realtime.h
test_sensor_logging.o: ../real_time_support/RTAI_mailbox_messaging.h
test_sensor_logging.o: ../real_time_support/messaging.h
test_sensor_logging.o: ../real_time_support/messages.h
test_sensor_logging.o: ../real_time_support/protocol_version.h
test_sensor_logging.o: ../utility/utility.h ../hardware_drivers/FlameIO.h
test_sensor_logging.o: ../hardware_drivers/AthenaDAQ.h
test_sensor_logging.o: ../hardware_drivers/DMM16AT.h
test_sensor_logging.o: ../hardware_drivers/Mesanet_4I36.h
test_sensor_logging.o: ../hardware_drivers/FlameIO_defs.h
test_sensor_logging.o: ../hardware_drivers/IO_permissions.h
test_user_space_realtime.o: ../real_time_support/RTAI_user_space_realtime.h
test_user_space_realtime.o: ../real_time_support/messages.h
test_user_space_realtime.o: ../utility/utility.h
UDP_messaging_test.o: ../real_time_support/UDP_messaging.h
UDP_messaging_test.o: ../real_time_support/messaging.h
UDP_messaging_test.o: ../real_time_support/messages.h
UDP_messaging_test.o: ../real_time_support/protocol_version.h
UDP_messaging_test.o: ../utility/utility.h
