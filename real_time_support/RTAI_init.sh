#! /bin/sh

# This is intended to be installed in /etc/init.d for use as a startup/shutdown script.

# exit immediately on failure of any command
set -e

# define a function to load the RTAI modules
load_RTAI() {

    if [ -d '/proc/rtai' ]; 
    then
	echo "RTAI already appears to be loaded."
    else
	echo "Loading RTAI modules with insmod.."
	insmod /usr/realtime/modules/rtai_hal.o
	insmod /usr/realtime/modules/rtai_up.o
	insmod /usr/realtime/modules/rtai_lxrt.o
	insmod /usr/realtime/modules/rtai_sem.o
	insmod /usr/realtime/modules/rtai_mbx.o
	insmod /usr/realtime/modules/rtai_msg.o
    fi
}
# define a function to unload the RTAI modules
unload_RTAI() {

    if [ ! -d '/proc/rtai' ]; 
    then
	echo "RTAI does not appear to be loaded."
    else
	echo "Removing RTAI modules from kernel with rmmod."
	rmmod rtai_msg
	rmmod rtai_mbx
	rmmod rtai_sem
	rmmod rtai_lxrt
	rmmod rtai_up
	rmmod rtai_hal
    fi
}

# select action in the standard init script way
case "$1" in
  start)
	load_RTAI
	;;

  stop)
	unload_RTAI
	;;

  reload|force-reload|restart)
	unload_RTAI
	load_RTAI
	;;

  *)
	echo "Usage: /etc/init.d/RTAI_init.sh {start|stop|reload|force-reload|restart}"
	exit 1
esac

exit 0
