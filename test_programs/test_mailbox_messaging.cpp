// $Id: test_mailbox_messaging.c,v 1.4 2005/12/12 17:30:59 garthz Exp $
// test_mailbox_messaging.c : test program for RTAI mailboxes
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
// This forks into real time and non-real-time threads and
// communicates between them.

#include <stdio.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <errno.h>

#include <real_time_support/RTAI_user_space_realtime.h>
#include <real_time_support/RTAI_mailbox_messaging.h>
#include <real_time_support/protocol_version.h>
#include <utility/utility.h>

/****************************************************************/
// This function will be called at regular intervals.
// No system calls are allowed inside this function. 
// Returns true if the periodic calls should continue.

static message_port *rt_out_port = NULL;
static message_port *rt_in_port = NULL;

// opaque handle for the real time control thread task
static struct realtime_task *control_task = NULL;

static int realtime_thread( long long timestamp, void *userdata )
{
  static int count = 0;
  
  count++;
  if ((count%1000) == 0) {
    send_print(rt_out_port, "real time task second interval tick.");
  }

  // check for other messages
  {
    union message_t msg;
    if ( message_receive( rt_in_port, &msg ) ) {
      if (msg.header.type == FLAME_MESSAGE) {
	switch (msg.header.subtype) {  	// select based on subtype

	case MSG_PING:
	  send_pong( rt_out_port );
	  break;

	case MSG_SHUTDOWN: // immediately quit
	  return 0;
	  break;
	  
	default:
	  break;
	}
      }
    }
  }

  if (count < 5000) return 1;
  else {
    send_signal ( rt_out_port, MSG_SHUTDOWN );
    return 0;
  }
}

// This is the child branch of the fork().
static void begin_realtime_program(void)
{
  logprintf("test_mailbox_messaging real time task starting.\n");

  control_task = create_RTAI_user_space_task( REALTIME_PROCESS_NAME );
  if ( control_task == NULL ) {
    printf("Failed to initialize RTAI interface.\n");
    return;
  }

  rt_out_port = init_RTAI_mailbox_message_port( message_port_alloc(), REALTIME_OUTPUT_MAILBOX_NAME );
  rt_in_port  = init_RTAI_mailbox_message_port( message_port_alloc(),  REALTIME_INPUT_MAILBOX_NAME );

  if ( rt_out_port == NULL ) {
    errprintf("Real time task unable to create message port.\n");
    shutdown_RTAI_user_space_task( control_task );
    return;
  }
  
  logprintf("starting real time thread, it should run for five seconds...\n");
  send_print(rt_out_port, "real time task beginnning.\n");

  run_RTAI_user_space_realtime_periodic_thread( control_task, realtime_thread, 1000000 /* nanoseconds */, NULL );

  logprintf("Real time thread exited.\n");
  
  message_port_dealloc( rt_out_port );
  message_port_dealloc( rt_in_port );
  shutdown_RTAI_user_space_task( control_task );

  logprintf("test_mailbox_messaging real time task done.\n");
}

/****************************************************************/
// this is the parent to run as a monitoring process
static void begin_monitor_program(void)
{
  int keep_running = 1;
  message_port *port_from_rt, *port_to_rt;

  control_task = create_RTAI_user_space_task( DISPLAY_PROCESS_NAME );
  if ( control_task == NULL ) {
    errprintf("Monitor process failed to initialize RTAI interface.\n");
    return;
  }

  port_from_rt = init_RTAI_mailbox_message_port( message_port_alloc(), REALTIME_OUTPUT_MAILBOX_NAME );
  port_to_rt   = init_RTAI_mailbox_message_port( message_port_alloc(), REALTIME_INPUT_MAILBOX_NAME );

  if ( port_from_rt == NULL || port_to_rt == NULL ) {
    errprintf("Monitor process unable to create message port.\n");
    shutdown_RTAI_user_space_task( control_task );
    return;
  }

  // begin message receiving loop
  while ( keep_running ) {
    union message_t msg;
    int ready;

    ready = message_receive( port_from_rt, &msg );
    if ( !ready ) {
      logprintf("monitor process: no message, sleeping.\n"); 
      delay_microseconds( 500000 );
      send_signal( port_to_rt, MSG_PING );
      continue; 

    } else {
      logprintf("Received message: type 0x%04x subtype %d length %d.\n", 
		msg.header.type, msg.header.subtype, msg.header.length );

      // drop any packets without the correct type
      if (msg.header.type != FLAME_MESSAGE) {
	errprintf( "  Error: major message type not correct.\n");

      } else {
	// select based on subtype
	switch (msg.header.subtype) {

	case MSG_PONG:
	  logprintf("  Received PONG with version code %d (%s).\n", msg.pong.version, 
		    (msg.pong.version == MESSAGE_PROTOCOL_VERSION) ? "correct" : "wrong!");
	  break;

	case MSG_PING:
	  logprintf("  Received PING, sending PONG reply.\n");
	  send_pong( port_to_rt );
	  break;

	case MSG_SHUTDOWN:
	  logprintf("  Received SHUTDOWN from real time thread.\n");
	  keep_running = 0;
	  break;

	case MSG_RESET:
	  logprintf("  Received RESET from real time thread, which doesn't make sense.\n");
	  break;

	case MSG_PRINT:
	  printf("  Received PRINT message from real time thread:\n    ");
	  fwrite( &msg.print.data, 1, msg.header.length - sizeof(msg.header), stdout);
	  printf("\n");
	  break;

	default:
	  logprintf("  Unhandled message type.\n");
	  break;
	}
      }
    }
  }
  message_port_dealloc( port_from_rt );
  message_port_dealloc( port_to_rt );

  logprintf("Monitor process shutting down master task.\n");
  shutdown_RTAI_user_space_task( control_task );
  logprintf("Monitor process done.\n");

}

/****************************************************************/
int main(int argc, char **argv) 
{
  pid_t parent;
  logprintf("test_mailbox_messaging starting up.\n");

  // First split into two processes, one to run real time, and one to monitor it.

  parent = fork();
  // parent = (pid_t) 1; // don't split

  if (parent == -1) {
    errprintf("unable to fork: %s", strerror(errno) );
    exit(1);

  } else if (parent == 0) {
    // this is the child process
    begin_realtime_program();
    exit(0);
  }

  // this is the parent process
  delay_microseconds( 200000 );  // give the other time to start
  begin_monitor_program();

  // wait for the child to end
  logprintf("waiting for child to finish.\n");
  { 
    int status; wait(&status);
  }

  logprintf("test_mailbox_messaging done.\n");
  exit(0);
}

