// $Id: control_demo_console.c,v 1.5 2005/12/16 16:14:23 garthz Exp $
// control_demo_console.c : simple display program to run on a host and display sensor data received via UDP
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

// This runs as an ordinary user process (not real-time) on a
// non-real-time host and displays sensor data received via UDP
// packets.  It has a curses-based text display.

#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#define _GNU_SOURCE
#include <string.h>

#include <curses.h>

#include <real_time_support/UDP_messaging.h>
#include <real_time_support/protocol_version.h>

#include <utility/utility.h>

// application headers
#include <hardware_drivers/FlameIO_defs.h>
#include "../FlameJoints.h"
#include "../message_format.h"
#include "../local_protocol_version.h"

/****************************************************************/
// Local copies of the state of the robot.

static FlameIO_state_t s;        // the global hardware state structure (i.e. blackboard)

// Local global state.

static int messages = 0;
static int missed_messages = 0;
static int cycle_count = 0;
static int running = 1;

/****************************************************************/
// UDP message ports for communicating with the robot.
static message_port *rt_out_port = NULL;
static message_port *rt_in_port = NULL;

/****************************************************************/
// Curses based console display. (Curses is the standard Unix library 
// for full screen text interfaces.)

// Define display column positions
#define COLWIDTH 12
#define MINWIDTH 75

// Define display row positions.
#define HIPX 1
#define LHIPY 3
#define LKNEE 4
#define LANKLEY 5
#define LANKLEX 6
#define LHIPYMOT 7
#define LKNEEMOT 8
#define LANKLEYMOT 9
#define RHIPY 11
#define RKNEE 12
#define RANKLEY 13
#define RANKLEX 14
#define RHIPYMOT 15
#define RKNEEMOT 16
#define RANKLEYMOT 17

#define FOOTSWITCHES 19

#define IMU                (FOOTSWITCHES + 3)

#define FLAGS        (IMU + 5)
#define VOLTAGES     (FLAGS+1)
#define LOADL        (VOLTAGES+1)
#define TOP_OF_CONSOLE_AREA (LOADL + 3)
#define BOT_OF_CONSOLE_AREA (LINES-1)  // LINES is a ncurses global variable
#define MINHEIGHT (TOP_OF_CONSOLE_AREA + 8)

static int start_curses(void)
{
  initscr();
  if ( LINES < MINHEIGHT || COLS < MINWIDTH ) {
    endwin();
    printf("Can't open display, it is too small.\n");
    return -1;
  }
    
  raw();
  noecho();
  // nonl(); 
  // intrflush(stdscr, FALSE); 
  // keypad(stdscr, TRUE);

  // the following is not part of curses, it's just a normal system call
  fcntl( fileno(stdin), F_SETFL, O_NONBLOCK );

  // create a software scrolling region for the scrolling console display
  setscrreg( TOP_OF_CONSOLE_AREA, BOT_OF_CONSOLE_AREA );
  scrollok(stdscr, 1);
  mvaddstr( TOP_OF_CONSOLE_AREA-1, 0, 
	    "============================== Message Area ===================================");
  return 0;
}
static void stop_curses(void)
{
  echo();
  cbreak();
  endwin();

  // the following is not part of curses, it's just a normal system call
  fcntl(fileno(stdin), F_SETFL, 0);   // restore blocking modes
}
/****************************************************************/
static void console_printf( char *format, ...)
{
  va_list args;
  char *message;

  va_start(args, format);
  vasprintf( &message, format, args);
  mvaddstr( BOT_OF_CONSOLE_AREA, 0, message);
  free( message );
  // fflush(stderr);

  va_end(args);
}
/****************************************************************/
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
/****************************************************************/
// Redraw a name/value pair in two adjacent columns
static inline void 
print_float_value( int row, int col, char *name, float value )
{
  mvaddstr( row, col*COLWIDTH, name);
  mvnprintf( row, (col+1)*COLWIDTH, COLWIDTH-1, "%f   ", value);
}
static inline void 
print_int_value( int row, int col, char *name, int value )
{
  mvaddstr( row, col*COLWIDTH, name);
  mvnprintf( row, (col+1)*COLWIDTH, COLWIDTH-1, "%d   ", value);
}

/****************************************************************/
// Redraw the whole display.
static void update_screen(void)
{
  static int frame = 0;
  int bit;

  mvaddstr( 0, 1*COLWIDTH, "q");
  mvaddstr( 0, 2*COLWIDTH, "qd");
  mvaddstr( 0, 3*COLWIDTH, "tau");
  mvaddstr( 0, 4*COLWIDTH, "imon");

  mvaddstr( HIPX ,   0, "hipx"     );

  mvaddstr( LHIPY,   0, "l.hipy"   );
  mvaddstr( LKNEE,   0, "l.knee"   );
  mvaddstr( LANKLEY, 0, "l.ankley" );
  mvaddstr( LANKLEX, 0, "l.anklex" );

  mvaddstr( LHIPYMOT,   0, "l.hipymot"   );
  mvaddstr( LKNEEMOT,   0, "l.kneemot"   );
  mvaddstr( LANKLEYMOT, 0, "l.ankleymot" );

  mvaddstr( RHIPY,   0, "r.hipy"   );
  mvaddstr( RKNEE,   0, "r.knee"   );
  mvaddstr( RANKLEY, 0, "r.ankley" );
  mvaddstr( RANKLEX, 0, "r.anklex" );

  mvaddstr( RHIPYMOT,   0, "r.hipymot"   );
  mvaddstr( RKNEEMOT,   0, "r.kneemot"   );
  mvaddstr( RANKLEYMOT, 0, "r.ankleymot" );

  mvaddstr( FOOTSWITCHES, 1*COLWIDTH, "l.back" );
  mvaddstr( FOOTSWITCHES, 2*COLWIDTH, "l.front" );
  mvaddstr( FOOTSWITCHES, 3*COLWIDTH, "r.back" );
  mvaddstr( FOOTSWITCHES, 4*COLWIDTH, "r.front" );
  mvaddstr( FOOTSWITCHES+1, 0, "foot sw" );

  mvaddstr( IMU, 0, "IMU YPR:");

  mvnprintf( HIPX ,   1*COLWIDTH, COLWIDTH-1, "%f    ", s.hipx.q     );
  mvnprintf( LHIPY,   1*COLWIDTH, COLWIDTH-1, "%f    ", s.l().hipy.q   );
  mvnprintf( LKNEE,   1*COLWIDTH, COLWIDTH-1, "%f    ", s.l().knee.q   );
  mvnprintf( LANKLEY, 1*COLWIDTH, COLWIDTH-1, "%f    ", s.l().ankley.q );
  mvnprintf( LANKLEX, 1*COLWIDTH, COLWIDTH-1, "%f    ", s.l().anklex.q );
  mvnprintf( RHIPY,   1*COLWIDTH, COLWIDTH-1, "%f    ", s.r().hipy.q   );
  mvnprintf( RKNEE,   1*COLWIDTH, COLWIDTH-1, "%f    ", s.r().knee.q   );
  mvnprintf( RANKLEY, 1*COLWIDTH, COLWIDTH-1, "%f    ", s.r().ankley.q );
  mvnprintf( RANKLEX, 1*COLWIDTH, COLWIDTH-1, "%f    ", s.r().anklex.q );

  mvnprintf( LHIPYMOT,   1*COLWIDTH, COLWIDTH-1, "%f    ", s.l().hipymot.q   );
  mvnprintf( LKNEEMOT,   1*COLWIDTH, COLWIDTH-1, "%f    ", s.l().kneemot.q   );
  mvnprintf( LANKLEYMOT, 1*COLWIDTH, COLWIDTH-1, "%f    ", s.l().ankleymot.q );
  mvnprintf( RHIPYMOT,   1*COLWIDTH, COLWIDTH-1, "%f    ", s.r().hipymot.q   );
  mvnprintf( RKNEEMOT,   1*COLWIDTH, COLWIDTH-1, "%f    ", s.r().kneemot.q   );
  mvnprintf( RANKLEYMOT, 1*COLWIDTH, COLWIDTH-1, "%f    ", s.r().ankleymot.q );

  mvnprintf( HIPX ,   2*COLWIDTH, COLWIDTH-1, "%f    ", s.hipx.qd     );
  mvnprintf( LHIPY,   2*COLWIDTH, COLWIDTH-1, "%f    ", s.l().hipy.qd   );
  mvnprintf( LKNEE,   2*COLWIDTH, COLWIDTH-1, "%f    ", s.l().knee.qd   );
  mvnprintf( LANKLEY, 2*COLWIDTH, COLWIDTH-1, "%f    ", s.l().ankley.qd );
  mvnprintf( LANKLEX, 2*COLWIDTH, COLWIDTH-1, "%f    ", s.l().anklex.qd );
  mvnprintf( RHIPY,   2*COLWIDTH, COLWIDTH-1, "%f    ", s.r().hipy.qd   );
  mvnprintf( RKNEE,   2*COLWIDTH, COLWIDTH-1, "%f    ", s.r().knee.qd   );
  mvnprintf( RANKLEY, 2*COLWIDTH, COLWIDTH-1, "%f    ", s.r().ankley.qd );
  mvnprintf( RANKLEX, 2*COLWIDTH, COLWIDTH-1, "%f    ", s.r().anklex.qd );

  mvnprintf( LHIPYMOT,   2*COLWIDTH, COLWIDTH-1, "%f    ", s.l().hipymot.qd   );
  mvnprintf( LKNEEMOT,   2*COLWIDTH, COLWIDTH-1, "%f    ", s.l().kneemot.qd   );
  mvnprintf( LANKLEYMOT, 2*COLWIDTH, COLWIDTH-1, "%f    ", s.l().ankleymot.qd );
  mvnprintf( RHIPYMOT,   2*COLWIDTH, COLWIDTH-1, "%f    ", s.r().hipymot.qd   );
  mvnprintf( RKNEEMOT,   2*COLWIDTH, COLWIDTH-1, "%f    ", s.r().kneemot.qd   );
  mvnprintf( RANKLEYMOT, 2*COLWIDTH, COLWIDTH-1, "%f    ", s.r().ankleymot.qd );

  mvnprintf( HIPX ,   3*COLWIDTH, COLWIDTH-1, "%f    ", s.hipx.tau     );
  mvnprintf( LHIPY,   3*COLWIDTH, COLWIDTH-1, "%f    ", s.l().hipy.tau   );
  mvnprintf( LKNEE,   3*COLWIDTH, COLWIDTH-1, "%f    ", s.l().knee.tau   );
  mvnprintf( LANKLEY, 3*COLWIDTH, COLWIDTH-1, "%f    ", s.l().ankley.tau );
  mvnprintf( RHIPY,   3*COLWIDTH, COLWIDTH-1, "%f    ", s.r().hipy.tau   );
  mvnprintf( RKNEE,   3*COLWIDTH, COLWIDTH-1, "%f    ", s.r().knee.tau   );
  mvnprintf( RANKLEY, 3*COLWIDTH, COLWIDTH-1, "%f    ", s.r().ankley.tau );

  mvnprintf( HIPX ,   4*COLWIDTH, COLWIDTH-1, "%f    ", s.hipx.imon     );
  mvnprintf( LHIPY,   4*COLWIDTH, COLWIDTH-1, "%f    ", s.l().hipy.imon   );
  mvnprintf( LKNEE,   4*COLWIDTH, COLWIDTH-1, "%f    ", s.l().knee.imon   );
  mvnprintf( LANKLEY, 4*COLWIDTH, COLWIDTH-1, "%f    ", s.l().ankley.imon );
  mvnprintf( RHIPY,   4*COLWIDTH, COLWIDTH-1, "%f    ", s.r().hipy.imon   );
  mvnprintf( RKNEE,   4*COLWIDTH, COLWIDTH-1, "%f    ", s.r().knee.imon   );
  mvnprintf( RANKLEY, 4*COLWIDTH, COLWIDTH-1, "%f    ", s.r().ankley.imon );

  mvnprintf( FOOTSWITCHES+1, 1*COLWIDTH, COLWIDTH-1, "%d   ", s.l().foot.back.state );
  mvnprintf( FOOTSWITCHES+1, 2*COLWIDTH, COLWIDTH-1, "%d   ", s.l().foot.front.state );
  mvnprintf( FOOTSWITCHES+1, 3*COLWIDTH, COLWIDTH-1, "%d   ", s.r().foot.back.state );
  mvnprintf( FOOTSWITCHES+1, 4*COLWIDTH, COLWIDTH-1, "%d   ", s.r().foot.front.state );

  mvnprintf( IMU, 1*COLWIDTH, COLWIDTH-1, "%f   ", s.imu.yaw);
  mvnprintf( IMU, 2*COLWIDTH, COLWIDTH-1, "%f   ", s.imu.pitch);
  mvnprintf( IMU, 3*COLWIDTH, COLWIDTH-1, "%f   ", s.imu.roll);

  /*  // a bunch of output for debugging the garthz controller
   *print_float_value( GARTHZ, 0, "pitch", c.garthz.pitch );
   *print_float_value( GARTHZ, 2, "sw_leg_ang", c.garthz.sw_leg_ang );
   *print_float_value( GARTHZ, 4, "st_leg_ang", c.garthz.st_leg_ang );
   *print_float_value( GARTHZ, 6, "rel_d_leg", c.garthz.rel_d_leg_ang );
   *
   *print_float_value( GARTHZ+1, 0, "inx", c.garthz.inx);
   *print_float_value( GARTHZ+1, 2, "st_leg", c.garthz.st_leg );
   */
  mvaddstr( FLAGS, 0, "Switches: ");
  for ( bit = 0; bit < 8; bit++ ) {
    addch( (s.front_panel_sw & ( 1 << bit )) ? '1' : '0');
    addch( ' ' );
  }

  mvaddstr( FLAGS, 30, "Faults: ");
  for ( bit = 0; bit < 8; bit++ ) {
    addch( (s.motor_faults & ( 1 << bit )) ? '1' : '0');
    addch( ' ' );
  }
//  mvnprintf( FLAGS, 60, 19, "mode: %d  ", c.mode );


  mvnprintf( VOLTAGES,   0,  19, "unsw mot: %3.2f V   ", s.battery.mot_un );
  mvnprintf( VOLTAGES,   20, 19, "swit mot: %3.2f V   ", s.battery.mot_sw );
  mvnprintf( VOLTAGES,   40, 19, "unsw cmp: %3.2f V   ", s.battery.com_un );
  mvnprintf( VOLTAGES,   60, 19, "swit cmp: %3.2f V   ", s.battery.com_sw );

  mvnprintf( LOADL,   0,  19, "t: %f     ", s.t );
  mvnprintf( LOADL,  20,  19, "sensor load: %3.2f%%     ", 100 * (s.timing.sensor_processing / 0.001) );
  mvnprintf( LOADL,  40,  19, "realtm load: %3.2f%%     ", 100 * (s.timing.total_cycle / 0.001) );

  mvnprintf( LOADL+1,  0, 19, "Cycle #: %d  ", cycle_count );
  mvnprintf( LOADL+1, 20, 19, "Messages #: %d  ", messages);
  mvnprintf( LOADL+1, 40, 19, "Missed #: %d  ", missed_messages);
  mvnprintf( LOADL+1, 60, 19, "Frame #: %d  ", frame++);

  // leave the cursor in the message area
  move( BOT_OF_CONSOLE_AREA, 0 );

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
  case 3:     // Control-C
  case 4:     // Control-D
  case 27:    // ESC

    // send a quit message to the real time process
    send_signal ( rt_in_port, MSG_SHUTDOWN );
    running = 0;
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

      // No need to actually shut down, this can just keep
      // displaying the last data until the next controller
      // starts sending.
      // running = 0;

      break;

    case MSG_RESET:
      console_printf("Received RESET from real time thread, which doesn't make sense.\n");
      break;

    case MSG_PRINT:
      {
	// Create a null terminated string to print.
	char *str = (char *) strndup( &msg->print.data, msg->header.length - sizeof(msg->header));
	console_printf("RT: %s", str);
	free(str);
      }
      break;

    case MSG_SENSOR_DATA:
      {
	static unsigned next_packet_serial = 0;
	// copy this into our version of the state structure
	struct sensor_message_t *m = (struct sensor_message_t *) msg;

	if ( m->local_protocol_version != LOCAL_MESSAGE_PROTOCOL_VERSION ) 
	  console_printf("Received incorrect local protocol version %d, should be %d.\n", 
			 m->local_protocol_version, LOCAL_MESSAGE_PROTOCOL_VERSION);

	memcpy( &s, &m->state, sizeof( s ) );       // copy all hardware state data from packet
	//memcpy( &joints, &m->joints, sizeof( joints ) );     // copy all controller state data from packet

	// Watch for dropped packets.  The count actually keeps
	// track of missed blocks of packets; if the serial
	// number increases by more than one, it will still just
	// increment missed_messages and reset the local counter.
	cycle_count = m->serial_number;

	if ( m->serial_number != next_packet_serial ) missed_messages++;
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
int main(int argc, char **argv) 
{
  logprintf( "Flame_console starting up.\n" );

  // Open the default UDP socket.  The default port numbers are defined in messages.h.
  rt_in_port = init_UDP_message_port( message_port_alloc(), 
				      DISPLAY_HOST_NAME, DISPLAY_UDP_PORT, 
				      REALTIME_HOST_NAME, REALTIME_UDP_PORT,
				      UDP_MESSAGE_PORT_INPUT | UDP_MESSAGE_PORT_OUTPUT | UDP_MESSAGE_EXCLUSIVE_INPUT
				      );

  // The same port is also used for output.
  rt_out_port = rt_in_port;

  if ( !MESSAGE_PORT_READY( rt_in_port ) ) {
    errprintf("%s unable to create UDP message port %s:%d\n", argv[0], DISPLAY_HOST_NAME, DISPLAY_UDP_PORT);
    goto fail;
  }

  // start up interface
  if (start_curses()) goto fail;

  // print a banner
  console_printf( "%s running. You can press q or ESC to exit.\n", argv[0]);

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
  while ( running ) {
    union message_t msg;

    // check for commands from the user
    check_keyboard();

    // sleep a little to allow other processes to run
    delay_microseconds( 50000 );

    // process all data from the real time process
    while ( message_receive( rt_out_port, &msg ) ) {
      process_message_from_rt( &msg );
      messages++;
    }

    // update the display
    update_screen();
  }

  // shutdown the text display
  stop_curses();
  logprintf("%s done.\n", argv[0]);
  message_port_dealloc( rt_in_port );
  exit(0);

 fail:
  logprintf("%s quitting.\n", argv[0]);
  if ( rt_in_port != NULL ) message_port_dealloc( rt_in_port );
  exit(1);
}
