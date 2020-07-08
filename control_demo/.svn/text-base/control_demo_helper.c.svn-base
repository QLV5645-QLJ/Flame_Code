// $Id: control_demo_helper.c,v 1.3 2005/12/16 16:14:22 garthz Exp $
// control_demo_helper : a non-realtime auxiliary for the demonstration controller for the Flame biped robot
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
// This used to be a separate non-real-time RTAI thread within
// the control_demo.c real-time program.  After chasing down
// unexpected bugs in memory allocation I decided it was simpler
// if this was clearly a separate process.  The bugs were mostly
// likely in my own code, but this reduces the number of
// potential interactions.

#include <stdio.h>
#include <asm/io.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include <stdarg.h>
#include <signal.h>
#include <sys/types.h>
#include <regex.h>

#include <rtai_lxrt.h>

#include <utility/utility.h>
#include <utility/dataset.h>
#include <utility/record.h>
#include <utility/system_state_var.h>

#include <real_time_support/RTAI_user_space_realtime.h>
#include <real_time_support/RTAI_mailbox_messaging.h>
#include <real_time_support/UDP_messaging.h>
#include <real_time_support/protocol_version.h>

// support for the IMU
#include <xsens/xsens.h>

// definitions for the big structures of control state
#include <hardware_drivers/FlameIO_defs.h>

// application header files
#include "system.h"
#include "local_protocol_version.h"
#include "message_format.h"

/****************************************************************/
#define NAME "helper"

// the monitor task is the RTAI task for the non-real-time thread
static struct realtime_task *monitor_task = NULL;
static int monitor_thread_started = 0;

/****************************************************************/
// local copies of the controller state
static FlameIO_state_t s;        // the global hardware state structure (i.e. blackboard)
static FlameIO_params_t params;  // the hardware parameters structure 
static controller_state_t c;     // the controller state blackboard

// communications channels
static message_port *udp_port = NULL;
static message_port *mailbox_to_rt = NULL;
static message_port *mailbox_from_rt = NULL;

// flag if the IMU is available
static int xsens_IMU_available = 0;

/****************************************************************/
// Provide a local version of errprintf to override the one in
// the utility library.
void errprintf(char *format, ...)
{
  va_list args;
  char nowstr[26];
  time_t now = time(NULL);
  va_start( args, format);
  strncpy( nowstr, ctime(&now)+11, 8);   // ctime format: "Wed Jun 30 21:49:08 1993\n"
  nowstr[8] = 0;
  fprintf(stderr, "%s error %s: ", NAME, nowstr);
  vfprintf(stderr, format, args);
  fflush(stderr);

  va_end(args);
}

/****************************************************************/
// Provide a local version of logprintf to override the one in
// the utility library.
void logprintf(char *format, ...)
{
  va_list args;

  va_start(args, format);
  fprintf(stdout, "%s: ", NAME);
  vfprintf(stdout, format, args);
  fflush(stderr);

  va_end(args);
}

/****************************************************************/
volatile static int keep_running = 1;

// User break handler for the main thread.
static void main_thread_break_handler(int sig)
{
  logprintf("received break signal, quitting.\n");
  keep_running = 0;
}

static void install_break_handler(void)
{
  // set up a break handler to detect user reset
  struct sigaction break_sigaction;
  break_sigaction.sa_handler = main_thread_break_handler;
  sigemptyset( &break_sigaction.sa_mask );
  break_sigaction.sa_flags   = SA_ONESHOT;

  if ( sigaction( SIGHUP, &break_sigaction, NULL ) ) {
    errprintf("unable to install SIGHUP handler: %s\n", strerror( errno ) );
  }

  if ( sigaction( SIGINT, &break_sigaction, NULL ) ) {
    errprintf("unable to install SIGINT handler: %s\n", strerror( errno ) );
  }

  if ( sigaction( SIGQUIT, &break_sigaction, NULL ) ) {
    errprintf("unable to install SIGQUIT handler: %s\n", strerror( errno ) );
  }
}
/****************************************************************/
// Bounce packets from a RTAI message queue to a UDP port.  The
// real time control thread cannot use Linux system calls, so it
// must send messages through a message port to this
// non-real-time thread, which can then echo them over the
// network.
static void run_event_loop(void)
{
  // Enter an event loop to receive and send packets.
  logprintf( "entering event loop.\n" );

  while ( keep_running ) {
    union message_t msg;

    if ( message_receive( mailbox_from_rt, &msg ) ) {

      // for now, just bounce messages from the RT to the UI
      send_message( udp_port, &msg );

      // do some limited interpretation of the packets
      if ( msg.header.type == FLAME_MESSAGE ) {
	switch ( msg.header.subtype ) {

	case MSG_SHUTDOWN:	
	  // logprintf("received SHUTDOWN, but will continue to run.\n");

	  logprintf("received SHUTDOWN, exiting.\n");
	  keep_running = 0;
	  break;

	case MSG_SENSOR_DATA:
	  {
	    static int warning_given = -1;
	    struct sensor_message_t *m = (struct sensor_message_t *) &msg; 
	    if ( m->local_protocol_version != LOCAL_MESSAGE_PROTOCOL_VERSION && m->local_protocol_version != warning_given ) {
	      logprintf( "Warning, protocol version %d doesn't match %d; this message will not be repeated.\n",
			 m->local_protocol_version, LOCAL_MESSAGE_PROTOCOL_VERSION );
	      warning_given = m->local_protocol_version;
	    }
	  }
	      
	  // keep_running = 0;
	  break;

	case MSG_PRINT:
	  // this will print on the local console, which isn't usually visible
	  printf("RT: ");
	  fwrite( &msg.print.data, 1, msg.header.length - sizeof(msg.header), stdout);
	  break;

	}
      }

    } else { 
      // no more data in queue from real time process, so poll
      // the IMU, which will block until the serial packet is
      // complete

      if ( xsens_IMU_available ) {
	xsens_IMU_data_t data;
	struct imu_data_message_t msg;

	int err = poll_xsens_IMU( &data );
	if ( err ) {
	  printf("Error from xsens IMU, detaching from it.\n");
	  xsens_IMU_available = 0;
	}
	
	// transmit the contents of the IMU packet to the real time process
	msg.hdr.type     = FLAME_MESSAGE;
	msg.hdr.subtype  = MSG_IMU_DATA;
	msg.hdr.length   = sizeof( msg );

	msg.samples = data.samples;
	msg.yaw     = data.yaw;
	msg.pitch   = data.pitch;
	msg.roll    = data.roll;

	send_message( mailbox_to_rt, (union message_t *) &msg );

	if ( (data.samples % 100) == 0 )  
	  printf("Samples: %d\tRoll: %6.1f\tPitch:%6.1f\tYaw:%6.1f\n",
		 data.samples, data.roll, data.pitch, data.yaw );

      } else {
	// if the IMU is not available, just sleep a little to
	// let the queue fill from the real time process
	
	delay_microseconds( 10000 );
      }
    }
  }
}

/****************************************************************/
int main(int argc, char **argv)
{
  int i;

  printf("control demo helper for Flame.\n");
  printf("This uses the RTAI real time system, which must already be loaded.\n");
  printf("This is a non-real time auxiliary for the real time controller.\n");

  if (!RTAI_is_ready()) {
    errprintf( "didn't find RTAI module, quitting.\n" );
    exit(1);
  }

  install_break_handler();

  // Open the default UDP socket.  The default port numbers are defined in messages.h.
  logprintf("opening UDP socket.\n");
  udp_port = init_UDP_message_port( message_port_alloc(), 
				    REALTIME_HOST_NAME, REALTIME_UDP_PORT,
				    DISPLAY_HOST_NAME, DISPLAY_UDP_PORT, 
				    UDP_MESSAGE_PORT_INPUT | UDP_MESSAGE_PORT_OUTPUT | UDP_MESSAGE_EXCLUSIVE_INPUT
				    );

  if ( !MESSAGE_PORT_READY( udp_port ) ) {
    errprintf("unable to create UDP message port %s:%d.\n", REALTIME_HOST_NAME, REALTIME_UDP_PORT );
    goto fail;
  }

  // Initialize the real time interface.
  logprintf( "initializing RTAI real time interface.\n");
  monitor_task = create_RTAI_user_space_task( DISPLAY_PROCESS_NAME );
  if ( monitor_task == NULL ) {
    errprintf("failed to initialize RTAI interface.\n");
    goto fail;
  }

  // Create the mailbox communications queues to interact with the control task
  logprintf("Monitor thread opening mailboxes.\n");
  mailbox_from_rt = init_RTAI_mailbox_message_port( message_port_alloc(), REALTIME_OUTPUT_MAILBOX_NAME );
  mailbox_to_rt   = init_RTAI_mailbox_message_port( message_port_alloc(), REALTIME_INPUT_MAILBOX_NAME );

  if ( mailbox_to_rt == NULL || mailbox_from_rt == NULL ) {
    errprintf( "unable to create mailbox message port.\n");
    goto fail;
  }

  // Make sure the inbox has nothing left from other programs or a previous run.
  {
    int count = 0;
    union message_t msg;
    while ( message_receive( mailbox_from_rt, &msg ) ) count++;
    if ( count > 0 ) logprintf( "flushed %d messages from input port.\n", count);
  }

  // Try opening the IMU; the function returns false on success.
  xsens_IMU_available = !open_xsens_IMU();

  // start working
  run_event_loop();

  // It exited, now clean up.
  logprintf("event loop exited.\n");

  message_port_dealloc( mailbox_from_rt );
  message_port_dealloc( mailbox_to_rt );
  shutdown_RTAI_user_space_task( monitor_task );
  message_port_dealloc( udp_port );

  // final farewell
  logprintf("done.\n");  
  exit(0);

  // a variety of failures end here
 fail:
   if ( udp_port != NULL )         message_port_dealloc( udp_port );
   if ( mailbox_to_rt != NULL )    message_port_dealloc( mailbox_to_rt );
   if ( mailbox_from_rt != NULL )  message_port_dealloc( mailbox_from_rt );
   if ( monitor_task != NULL )    shutdown_RTAI_user_space_task( monitor_task );
   monitor_task = NULL;
   logprintf("quitting %d %d.\n");  

   exit(1);
}

