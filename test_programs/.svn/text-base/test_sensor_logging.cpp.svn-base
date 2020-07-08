// $Id: test_sensor_logging.c,v 1.6 2005/12/12 17:30:59 garthz Exp $
// test_sensor_logging.c : log sensor data to disk using a real time thread and non-real-time monitor
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
#include <rtai_lxrt.h>

#include <real_time_support/RTAI_user_space_realtime.h>
#include <real_time_support/RTAI_mailbox_messaging.h>
#include <real_time_support/protocol_version.h>
#include <utility/utility.h>
#include <hardware_drivers/FlameIO.h>
#include <hardware_drivers/DMM16AT.h>
#include <hardware_drivers/IO_permissions.h>

/****************************************************************/
#define SAMPLING_RATE 500      // Hz
#define TIMER_PERIOD (1000000000 / SAMPLING_RATE )  // nsec

// opaque handle for the real time control thread task
static struct realtime_task *control_task = NULL;

/****************************************************************/
// define a special purpose message type

#define MSG_SENSOR_DATA (MSG_LASTMESSAGENUM + 1)
struct sensor_message_t {
  struct _msg_hdr header;
  int serial_number;
  unsigned int sensor_processing_duration;
  unsigned int total_cycle_duration;
  short analog[8];
};

/****************************************************************/
// This function will be called at regular intervals.
// No system calls are allowed inside this function. 
// Returns true if the periodic calls should continue.

static message_port *rt_out_port = NULL;
static message_port *rt_in_port = NULL;
static DMM16AT *dmm = NULL;
static int next_serial = 0;

static int realtime_thread( long long timestamp, void *userdata )
{
  RTIME start_of_cycle, end_of_sensor_reading, end_of_cycle;
  static unsigned int last_cycle_duration = 0, last_sensor_duration = 0;
  static int count = 0;

  start_of_cycle = rt_get_cpu_time_ns();

  if ( dmm != NULL ) {
    struct sensor_message_t msg;
    int channel;

    DMM16AT_read_all_analog_inputs( dmm );

    end_of_sensor_reading = rt_get_cpu_time_ns();

    // format a data packet
    msg.header.type     = FLAME_MESSAGE;
    msg.header.subtype  = MSG_SENSOR_DATA;
    msg.header.length   = sizeof( struct sensor_message_t );
    msg.serial_number   = next_serial++;

    for ( channel = 0; channel < 8; channel++ ) msg.analog[channel] = dmm->analog_inputs[ channel ];

    msg.sensor_processing_duration = last_sensor_duration;
    msg.total_cycle_duration       = last_cycle_duration;
    send_message ( rt_out_port, (union message_t *) &msg);
  }

  // check for messages
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
  end_of_cycle = rt_get_cpu_time_ns();
  last_sensor_duration = (unsigned int) (end_of_sensor_reading - start_of_cycle);
  last_cycle_duration = (unsigned int) (end_of_cycle - start_of_cycle);

  if (++count < 5000) return 1;
  else {
    send_signal ( rt_out_port, MSG_SHUTDOWN );
    return 0;
  }
}

// This is the child branch of the fork().
static void begin_realtime_program(void)
{
  logprintf("test_sensor_logging real time task starting.\n");

  // open up the hardware
  enable_IO_port_access();
  logprintf( "Opening up DMM-16-AT.\n");
  dmm = DMM16AT_init ( DMM16AT_alloc() );

  if ( dmm == NULL ) {
    errprintf( "Unable to open hardware: %s", strerror( errno) );
    exit(1);
  }

  // start up the real time system
  control_task = create_RTAI_user_space_task( REALTIME_PROCESS_NAME );
  if ( control_task == NULL ) {
    printf("Failed to initialize RTAI interface.\n");
    return;
  }

  rt_out_port = init_RTAI_mailbox_message_port( message_port_alloc(), REALTIME_OUTPUT_MAILBOX_NAME );
  rt_in_port  = init_RTAI_mailbox_message_port( message_port_alloc(), REALTIME_INPUT_MAILBOX_NAME );

  if ( rt_out_port == NULL ) {
    errprintf("Real time task unable to create message port.\n");
    shutdown_RTAI_user_space_task( control_task );
    return;
  }
  
  logprintf("starting real time thread, it should run for five seconds...\n");
  send_print(rt_out_port, "real time task beginnning.\n");

  run_RTAI_user_space_realtime_periodic_thread( control_task, realtime_thread, TIMER_PERIOD /* nanoseconds */, NULL );

  logprintf("Real time thread exited.\n");
  
  message_port_dealloc( rt_out_port );
  message_port_dealloc( rt_in_port );
  shutdown_RTAI_user_space_task( control_task );

  logprintf( "Shutting down DMM-16-AT.\n");
  DMM16AT_dealloc( dmm );

  logprintf("test_sensor_logging real time task done.\n");
}

/****************************************************************/
// this is the parent to run as a monitoring process
static void begin_monitor_program(void)
{
  FILE *record_file = NULL;
  int keep_running = 1;
  message_port *port_from_rt, *port_to_rt;

  control_task = create_RTAI_user_space_task( DISPLAY_PROCESS_NAME );
  if ( control_task == NULL ) {
    errprintf("Monitor process failed to initialize RTAI interface.\n");
    return;
  }

  port_from_rt = init_RTAI_mailbox_message_port( message_port_alloc(), REALTIME_OUTPUT_MAILBOX_NAME );
  port_to_rt   = init_RTAI_mailbox_message_port( message_port_alloc(), REALTIME_INPUT_MAILBOX_NAME );

  record_file = fopen( "/tmp/test_sensor_logging.data", "w");

  if ( port_from_rt == NULL || port_to_rt == NULL ) {
    errprintf("Monitor process unable to create message port.\n");
    shutdown_RTAI_user_space_task( control_task );
    return;
  }

  // send initial ping to elicit a response
  send_signal( port_to_rt, MSG_PING );

  // begin message receiving loop
  while ( keep_running ) {
    union message_t msg;
    int ready;

    ready = message_receive( port_from_rt, &msg );
    if ( !ready ) {
      // logprintf("monitor process: no message, sleeping.\n"); 
      delay_microseconds( 5000 );
      continue; 

    } else {
      // logprintf("Received message: type 0x%04x subtype %d length %d.\n", msg.header.type, msg.header.subtype, msg.header.length );

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

	case MSG_SENSOR_DATA:
	  {
	    struct sensor_message_t *s = (struct sensor_message_t *) &msg;
	    int c;
	    fprintf(record_file, "%d: %d %d  ", 
		    s->serial_number, s->sensor_processing_duration, 
		    s->total_cycle_duration);

	    for (c = 0; c < 8; c++) fprintf(record_file, "%d ", s->analog[c]);
	    fprintf(record_file, "\n");
	  }
	  break;

	default:
	  logprintf("  Received unhandled message type.\n");
	  break;
	}
      }
    }
  }
  message_port_dealloc( port_from_rt );
  message_port_dealloc( port_to_rt );
  fclose(record_file);

  logprintf("Monitor process shutting down master task.\n");
  shutdown_RTAI_user_space_task( control_task );
  logprintf("Monitor process done.\n");

}

/****************************************************************/
int main(int argc, char **argv) 
{
  pid_t parent;
  logprintf( "test_sensor_logging starting up.\n" );

  // First split into two processes, one to run real time, and one to monitor it.

  parent = fork();

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

  logprintf("test_sensor_logging done.\n");
  exit(0);
}

