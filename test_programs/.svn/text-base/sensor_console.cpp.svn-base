// $Id: sensor_console.c,v 1.10 2005/12/15 16:01:20 garthz Exp $
// sensor_console.c : simple display program to print out sensor data streamed from a real time process
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
// This forks into real time and non-real-time threads and
// communicates between them.  This uses the hardware drivers
// directly, not the FlameIO composite device.

#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#define _GNU_SOURCE
#include <string.h>

#include <curses.h>
#include <rtai_lxrt.h>

#include <real_time_support/RTAI_user_space_realtime.h>
#include <real_time_support/RTAI_mailbox_messaging.h>
#include <real_time_support/protocol_version.h>
#include <utility/utility.h>

#include <hardware_drivers/DMM16AT.h>
#include <hardware_drivers/AthenaDAQ.h>
#include <hardware_drivers/Mesanet_4I36.h>

#include <hardware_drivers/IO_permissions.h>

/****************************************************************/
#define SAMPLING_RATE 1000      // Hz
#define TIMER_PERIOD (1000000000 / SAMPLING_RATE )  // nsec

/****************************************************************/
// Structure used to capture the hardware state.
struct hardware_state_t {
  short analog[32];
  int encoder[16];
  unsigned char front_panel_sw;
  unsigned char motor_faults;
};

// These are defined globally, but remember that this program
// forks into separate processes, so these variables in the real
// time loop are not the same memory as in the monitor process.

static message_port *rt_out_port = NULL;
static message_port *rt_in_port = NULL;

// Opaque handle for the real time tasks.  Since this program
// actually forks instead of using pthreads, only one of these
// will be in use at a time.
static struct realtime_task *control_task = NULL;
static struct realtime_task *display_task = NULL;

/****************************************************************/
// Define a special purpose message type to communicate between
// the two processes.

#define MSG_SENSOR_DATA (MSG_LASTMESSAGENUM + 1)
struct sensor_message_t {
  struct _msg_hdr header;
  unsigned int serial_number;
  unsigned int sensor_processing_duration;
  unsigned int total_cycle_duration;

  struct hardware_state_t raw;
};

/****************************************************************/
// Real time program

// Globals for the real time process.

// Hardware driver handles.
static DMM16AT *dmm = NULL;
static AthenaDAQ *daq = NULL;
static Mesanet_4I36 *mesa1 = NULL;
static Mesanet_4I36 *mesa2 = NULL;

/****************************************************************/
// This function will be called at regular intervals.
// No system calls are allowed inside this function. 
// Returns true if the periodic calls should continue.


static int realtime_thread( long long timestamp, void *userdata )
{
  static unsigned int last_cycle_duration = 0, last_sensor_duration = 0;
  static unsigned next_serial = 0;

  RTIME start_of_cycle, end_of_sensor_reading, end_of_cycle;
  struct sensor_message_t sens;
  int channel;
  union message_t msg;

  start_of_cycle = (RTIME) timestamp;

  // Read all the sensor inputs.
  memset( &sens, 0, sizeof(sens) );
  if ( dmm != NULL)    DMM16AT_read_all_analog_inputs( dmm );
  if ( dmm != NULL)    sens.raw.motor_faults = DMM16AT_read_digital_input_byte ( dmm );
  if ( daq != NULL)    AthenaDAQ_read_all_analog_inputs( daq );
  if ( daq != NULL)    sens.raw.front_panel_sw = AthenaDAQ_read_digital_input_byte( daq, 2 );
  if ( mesa1 != NULL ) Mesanet_4I36_read_all_counters( mesa1 );
  if ( mesa2 != NULL ) Mesanet_4I36_read_all_counters( mesa2 );

  end_of_sensor_reading = rt_get_cpu_time_ns();

  // Format a data packet with all the raw sensor values.
  sens.header.type     = FLAME_MESSAGE;
  sens.header.subtype  = MSG_SENSOR_DATA;
  sens.header.length   = sizeof( struct sensor_message_t );
  sens.serial_number   = next_serial++;

  if ( daq != NULL)   for ( channel =  0; channel < 16; channel++ ) sens.raw.analog[channel]  = daq->analog_inputs[ channel ];
  if ( dmm != NULL)   for ( channel = 16; channel < 32; channel++ ) sens.raw.analog[channel]  = dmm->analog_inputs[ channel-16 ];
  if ( mesa1 != NULL) for ( channel =  0; channel <  8; channel++ ) sens.raw.encoder[channel] = mesa1->count[ channel ];
  if ( mesa2 != NULL) for ( channel =  8; channel < 16; channel++ ) sens.raw.encoder[channel] = mesa2->count[ channel-8 ];

  // Add timing information.
  sens.sensor_processing_duration = last_sensor_duration;
  sens.total_cycle_duration       = last_cycle_duration;
  send_message ( rt_out_port, (union message_t *) &sens);

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

  end_of_cycle = rt_get_cpu_time_ns();
  last_sensor_duration = (unsigned int) (end_of_sensor_reading - start_of_cycle);
  last_cycle_duration = (unsigned int) (end_of_cycle - start_of_cycle);

  // always ask for more iterations by default
  return 1;
}

// This is the child branch of the fork().
static void begin_realtime_program(void)
{
  logprintf("test_sensor_logging real time task starting.\n");

  // open up the hardware
  enable_IO_port_access();
  logprintf( "Opening up hardware devices.\n");

  // Each of these lines may be commented out to not use a particular device.
  dmm = DMM16AT_init ( DMM16AT_alloc() );
  daq = AthenaDAQ_init( AthenaDAQ_alloc() );
  mesa1 = Mesanet_4I36_init_with_address ( Mesanet_4I36_alloc(), 0x220 );
  mesa2 = Mesanet_4I36_init_with_address ( Mesanet_4I36_alloc(), 0x230 );

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

  // Make sure the inbox has nothing left from other programs or a previous run.
  {
    int count = 0;
    union message_t msg;
    while ( message_receive( rt_in_port, &msg ) ) count++;
    if ( count > 0 ) logprintf("RT: Flushed %d messages from input port.\n", count);
  }
  
  logprintf("starting real time thread...\n");

  if ( !DMM16AT_is_ready( dmm ) )        send_print( rt_out_port, "DMM16AT not opened." );
  if ( !AthenaDAQ_is_ready( daq ) )      send_print( rt_out_port, "Athena DAQ not opened." );
  if ( !Mesanet_4I36_is_ready( mesa1 ) ) send_print( rt_out_port, "Mesa 1 board not opened." );
  if ( !Mesanet_4I36_is_ready( mesa2 ) ) send_print( rt_out_port, "Mesa 2 board not opened." );

  send_print(rt_out_port, "real time task beginning.");

  run_RTAI_user_space_realtime_periodic_thread( control_task, realtime_thread, TIMER_PERIOD /* nanoseconds */, NULL );

  send_print(rt_out_port, "real time task exited.\n");
  // logprintf("Real time thread exited.\n");
  
  message_port_dealloc( rt_out_port );
  message_port_dealloc( rt_in_port );
  shutdown_RTAI_user_space_task( control_task );

  // logprintf( "Shutting down DMM-16-AT.\n");
  DMM16AT_dealloc( dmm );
  AthenaDAQ_dealloc( daq );
  Mesanet_4I36_dealloc ( mesa1 );
  Mesanet_4I36_dealloc ( mesa2 );

  // logprintf("test_sensor_logging real time task done.\n");
}
/****************************************************************/
// Monitor program
/****************************************************************/
// Define a state structure for the monitor program.  This is
// defined globally, but remember that this process forks into
// separate processes, so this structure in the real time loop is
// not the same memory at the one in the monitor process.

static struct {
  struct hardware_state_t raw;

  int running;      // true until it is time to quit
  float sensor_load;  // cpu usage percent
  float rt_load;
  unsigned cycle_count;
  unsigned messages;
  unsigned missed_messages;
  float t;
} s;

// Curses based console display. (Curses is the standard Unix library 
// for full screen text interfaces.)

#define TOP_OF_CONSOLE_AREA 18
#define BOT_OF_CONSOLE_AREA 24

static void start_curses(void)
{
  initscr();
  raw();
  noecho();
  // nonl(); 
  // intrflush(stdscr, FALSE); 
  // keypad(stdscr, TRUE);

  // the following is not part of curses, it's just a normal system call
  fcntl( fileno(stdin), F_SETFL, O_NONBLOCK );

  // create a software scrolling region for the scrolling console display
  setscrreg( TOP_OF_CONSOLE_AREA, BOT_OF_CONSOLE_AREA);
  scrollok(stdscr, 1);
  mvaddstr( TOP_OF_CONSOLE_AREA-1, 0, 
	    "============================== Message Area ===================================");
}
static void stop_curses(void)
{
  echo();
  cbreak();
  endwin();

  // the following is not part of curses, it's just a normal system call
  fcntl(fileno(stdin), F_SETFL, 0);   // restore blocking modes
}
static void console_printf( char *format, ...)
{
  va_list args;
  char *message;

  va_start(args, format);
  vasprintf( &message, format, args);
  mvaddstr( BOT_OF_CONSOLE_AREA, 0, message);
  free( message );
  fflush(stderr);

  va_end(args);
}
// A safe function for printing to a fixed width area on a curses display.
static void mvnprintf( int y, int x, int width, char *format, ...)
{
  va_list args;
  char *message;
  va_start(args, format);
  vasprintf( &message, format, args);
  mvaddnstr( y, x, message, width );
  free( message );
  fflush(stderr);
  va_end(args);
}
// Redraw the whole display.
static void update_screen(void)
{
  static int frame = 0;
  int c;
  mvaddstr( 0, 0, "Analog Inputs");
  for ( c = 0; c < 32; c++ ) {
    mvnprintf( 1 + (c / 8), 10*(c % 8), 9, "%d        ", s.raw.analog[c]);
  }

  mvaddstr( 6, 0, "Encoder Inputs");
  for ( c = 0; c < 16; c++ ) {
    mvnprintf( 7 + (c / 8), 10*(c % 8), 9, "%d        ", s.raw.encoder[c]);
  }

  mvaddstr( 10, 0, "Switches: ");
  for ( c = 0; c < 8; c++ ) {
    addch( (s.raw.front_panel_sw & ( 1 << c )) ? '1' : '0');
    addch( ' ' );
  }

  mvaddstr( 10, 40, "Faults: ");
  for ( c = 0; c < 8; c++ ) {
    addch( (s.raw.motor_faults & ( 1 << c )) ? '1' : '0');
    addch( ' ' );
  }

#define LOADL 12
  mvnprintf( LOADL,   0,  19, "sensor load: %3.2f%%     ", s.sensor_load );
  mvnprintf( LOADL,  20, 19, "realtm load: %3.2f%%     ", s.rt_load );
  mvnprintf( LOADL+1, 0,  19, "Cycle #: %d  ", s.cycle_count );
  mvnprintf( LOADL+1, 20, 19, "Messages #: %d  ", s.messages);
  mvnprintf( LOADL+1, 40, 19, "Missed #: %d  ", s.missed_messages);
  mvnprintf( LOADL+1, 60, 19, "Frame #: %d  ", frame++);

  refresh(); // update the actual display
}
static void check_keyboard(void)
{
  int k = getc(stdin);
  
  // assume all errors indicate the getc would block, meaning no key has been pressed
  if ( k == -1) return ;

  switch(k) {
  case 'p':
    console_printf("Sending PING to real time process.\n");
    send_signal( rt_in_port, MSG_PING );
    break;

  case 'q':
  case 'Q':
  case 27:    // ESC

    // send a quit message to the real time process
    send_signal ( rt_in_port, MSG_SHUTDOWN );
    s.running = 0;
    break;

  default:
    console_printf("keypress not understood.\n");
    break;
  }
}

/****************************************************************/
static void
process_message_from_rt( union message_t *msg )
{
  // drop any packets without the correct type
  if (msg->header.type != FLAME_MESSAGE) {
    console_printf( "Error: major message type not correct.\n");

  } else {
    // select based on subtype
    switch (msg->header.subtype) {

    case MSG_PONG:
      console_printf("Received PONG with version code %d (%s).\n", msg->pong.version, 
		(msg->pong.version == MESSAGE_PROTOCOL_VERSION) ? "correct" : "wrong!");
      break;

    case MSG_PING:
      console_printf("Received PING, sending PONG reply.\n");
      send_pong( rt_in_port );
      break;

    case MSG_SHUTDOWN:
      console_printf("Received SHUTDOWN from real time thread.\n");
      s.running = 0;
      break;

    case MSG_RESET:
      console_printf("Received RESET from real time thread, which doesn't make sense.\n");
      break;

    case MSG_PRINT:
      {
	// Create a null terminated string to print.
	char *str = (char *) strndup( &msg->print.data, msg->header.length - sizeof(msg->header));
	console_printf("RT: %s\n", str);
	free(str);
      }
      break;

    case MSG_SENSOR_DATA:
      {
	static unsigned next_packet_serial = 0;
	// copy this into our version of the state structure
	struct sensor_message_t *m = (struct sensor_message_t *) msg;

	// Copy all the raw sensor values in one block.
	memcpy( &s.raw, &m->raw, sizeof( s.raw ) );

	// Rescale the timing values to percentages.
	s.sensor_load = (100.0 * m->sensor_processing_duration) / TIMER_PERIOD;
	s.rt_load     = (100.0 * m->total_cycle_duration) / TIMER_PERIOD;
	s.cycle_count = m->serial_number;

	// Watch for dropped packets.  The count actually keeps
	// track of missed blocks of packets; if the serial
	// number increases by more than one, it will still just
	// increment missed_messages and reset the local counter.
	if ( m->serial_number != next_packet_serial ) s.missed_messages++;
	next_packet_serial = m->serial_number + 1;
      }
      break;

    default:
      console_printf("Received unhandled message subtype %d.\n", msg->header.subtype );
      break;
    }
  }
}

/****************************************************************/
// this is the parent to run as a monitoring process
static void begin_monitor_program(void)
{
  s.running = 1;
  s.messages = 0;

  display_task = create_RTAI_user_space_task( DISPLAY_PROCESS_NAME );
  if ( display_task == NULL ) {
    errprintf("Monitor process failed to initialize RTAI interface.\n");
    return;
  }

  rt_out_port = init_RTAI_mailbox_message_port( message_port_alloc(), REALTIME_OUTPUT_MAILBOX_NAME );
  rt_in_port  = init_RTAI_mailbox_message_port( message_port_alloc(), REALTIME_INPUT_MAILBOX_NAME );

  if ( rt_out_port == NULL || rt_in_port == NULL ) {
    errprintf("Monitor process unable to create message port.\n");
    shutdown_RTAI_user_space_task( display_task );
    return;
  }

  // start up interface
  start_curses();

  // Make sure the inbox has nothing left from other programs or a previous run.
  {
    int count = 0;
    union message_t msg;
    while ( message_receive( rt_out_port, &msg ) ) count++;
    if ( count > 0 ) console_printf("Flushed %d messages from input port.\n", count);
  }

  // send initial ping to elicit a response
  send_signal( rt_in_port, MSG_PING );

  // begin message receiving loop
  while ( s.running ) {
    union message_t msg;

    // check for commands from the user
    check_keyboard();

    // sleep a little to allow other processes to run
    delay_microseconds( 50000 );

    // process all data from the real time process
    while ( message_receive( rt_out_port, &msg ) ) {
      process_message_from_rt( &msg );
      s.messages++;
    }

    // update the display
    update_screen();
  }

  // shutdown the text display
  stop_curses();

  message_port_dealloc( rt_out_port );
  message_port_dealloc( rt_in_port );

  logprintf("Monitor process shutting down master task.\n");
  shutdown_RTAI_user_space_task( display_task );
  logprintf("Monitor process done.\n");
}

/****************************************************************/
int main(int argc, char **argv) 
{
  pid_t parent;
  logprintf( "sensor_console starting up.\n" );

  if (!RTAI_is_ready()) {
    errprintf("Didn't find RTAI module, quitting.\n");
    exit(1);
  }

  // First split into two processes, one to run real time, and one to monitor it.

  parent = fork();

  if (parent == -1) {
    errprintf("unable to fork: %s", strerror(errno) );
    exit(1);

  } else if (parent == 0) {      // this is the child process
    begin_realtime_program();
    exit(0);
  }

  // this is the parent process
  delay_microseconds( 1000000 );  // give the other time to start
  begin_monitor_program();

  // wait for the child to end
  logprintf("waiting for child to finish.\n");
  { int status; wait(&status); }

  logprintf("sensor_console done.\n");
  exit(0);
}

