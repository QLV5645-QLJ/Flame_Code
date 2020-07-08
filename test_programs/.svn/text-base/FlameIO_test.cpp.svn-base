// $Id: FlameIO_test.c,v 1.5 2005/12/14 17:31:01 garthz Exp $
// FlameIO_test.c : test program for operating the PC/104 hardware without the real time system
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
#include <stdio.h>
#include <stdlib.h>
#include <asm/io.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include <stdarg.h>
#include <signal.h>
#include <sched.h>
#include <sys/resource.h>

#include <hardware_drivers/FlameIO.h>
#include <utility/utility.h>

/****************************************************************/
static FlameIO_state_t s;        // the global state structure (i.e. blackboard)
static FlameIO_params_t params;  // the hardware parameters structure 
static FlameIO io;               // a static area to hold the driver data

/****************************************************************/
volatile static int keep_running = 1;

// User break handler for the main thread.
static void main_thread_break_handler(int sig)
{
  errprintf("\n\nFlameIO_test main thread received break signal, quitting.\n");
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
    errprintf("FlameIO_test main thread unable to install SIGHUP handler: %s\n", strerror( errno ) );
  }

  if ( sigaction( SIGINT, &break_sigaction, NULL ) ) {
    errprintf("FlameIO_test main thread unable to install SIGINT handler: %s\n", strerror( errno ) );
  }

  if ( sigaction( SIGQUIT, &break_sigaction, NULL ) ) {
    errprintf("FlameIO_test main thread unable to install SIGQUIT handler: %s\n", strerror( errno ) );
  }
}

/****************************************************************/
int main(int argc, char **argv)
{
  int i;

  printf("Flame I/O test.\n");
  printf("This is meant to be run on the PC/104 stack for the Flame Biped.\n");
  printf("It does not use the RTAI real time system.\n");

  install_break_handler();

  logprintf( "Opening up all Flame I/O hardware.\n");

  FlameIO_init ( &io );

  // Test a partial hardware configuration.
  // AthenaDAQ_close    ( &io.daq );
  // Mesanet_4I36_close ( &io.mesa1 );
  // Mesanet_4I36_close ( &io.mesa2 );
  // DMM16AT_close      ( &io.dmm );

  if (!FlameIO_is_ready( &io )) {
    logprintf("Warning: not all Flame I/O hardware is open.\n");
  }

  // Initialize our offset and scale structures.
  FlameIO_initialize_default_parameters( &params );

  // A battery of possible test code

  /****************************************************************/
#if 0
  printf("Press the first console pushbutton to quit.\n");
  do {
    FlameIO_read_all_sensors( &io, &s );
    printf("computer battery: %f   ", s.battery.com_un );
    printf("computer input:   %f   ", s.battery.com_sw );
    // printf("   motor battery: %f   ", s.battery.mot_un );
    // printf("   motor input:   %f   ", s.battery.mot_sw );
    printf("\n");
    delay_microseconds(100000);
  } while ( (s.front_panel_sw & 1) );
#endif

  /****************************************************************/
#if 0
  printf("LED blink test.\n");
  {
    int led = 0;
    for (i = 0; i < 24; i++) {
      printf( "turning front panel LED %d ON.\n", led);
      FlameIO_set_front_panel_LEDS( &io, 1<<led );
      delay_microseconds( 300000 );
      if (++led > 5) led = 0;
    }
  }
#endif
  /****************************************************************/

#if 0
  printf("Foot switch test.\n");
  printf("Press the first console pushbutton to quit.\n");
  do {
    FlameIO_read_all_sensors( &io, &s );
    printf("%f %f %f %f\n", s.foot.l.front, s.foot.l.back, s.foot.r.front, s.foot.r.back );
    delay_microseconds(100000);
  } while ( (s.front_panel_sw & 1) );
#endif

  /****************************************************************/

#if 0
  printf("DMM16 test.\n");
  printf("Press the first console pushbutton to quit.\n");
  do {
    FlameIO_read_all_sensors( &io, &s );
    printf("%f %f %f %f\n", s.imon.hipx, s.imon.l.hipy, s.imon.l.knee, s.imon.l.ankley);
    delay_microseconds(100000);
  } while ( (s.front_panel_sw & 1) );
#endif

  /****************************************************************/

#if 1
  printf("D/A converter test.\n");
  printf("This will generate a full-range sine wave with a different frequency for each driver control output.\n");
  printf("  taumax values:\n");
  printf("    params.taumax.hipx = %f\n", params.hipx.taumax );
  printf("    params.taumax.l.hipy = %f\n", params.l().hipy.taumax );
  printf("    params.taumax.l.knee = %f\n", params.l().knee.taumax );
  printf("    params.taumax.l.ankley = %f\n", params.l().ankley.taumax );
  printf("    params.taumax.r.hipy = %f\n", params.r().hipy.taumax );
  printf("    params.taumax.r.knee = %f\n", params.r().knee.taumax );
  printf("    params.taumax.r.ankley = %f\n", params.r().ankley.taumax );
  printf("\n");
  // printf("Press the first console pushbutton to quit.\n");
  printf("Press ^C to quit.\n");
  s.front_panel_sw = 0xffff;

  {
    int count = 0;
    double phase = 0.0;
    int fd;
    int data;
#if 0
    fd = open( "/dev/null", O_WRONLY);

    if ( -1 == setpriority( PRIO_PROCESS, 0, 2 ) ){  // lower the process priority
      errprintf("Unable to set lower priority: %s\n", strerror(errno));
    }
#endif
    do {
      FlameIO_read_all_sensors( &io, &params, &s );

      s.hipx.tau     = params.hipx.taumax   	 * sin( phase );
      s.l().hipy.tau   = params.l().hipy.taumax 	 * sin( 1.1 * phase );
      s.l().knee.tau   = params.l().knee.taumax 	 * sin( 1.2 * phase );
      s.l().ankley.tau = params.l().ankley.taumax * sin( 1.3 * phase );
      s.r().hipy.tau   = params.r().hipy.taumax   * sin( 1.4 * phase );
      s.r().knee.tau   = params.r().knee.taumax   * sin( 1.5 * phase );
      s.r().ankley.tau = params.r().ankley.taumax * sin( 1.6 * phase );
      s.powered      = 0xffff;

      FlameIO_write_torque_commands( &io, &params, &s );
      FlameIO_set_front_panel_LEDS( &io, (count++ >> 5));
      phase += 0.01 * M_PI; 
      
      // For some reason, on this system this process will never
      // be interrupted if there is no system call; all the aI/O
      // is direct to ports, so there are no system calls, and
      // for some reason the scheduler is not ever time slicing it.

      // if (kbhit()) break;
      // write( fd, &data, 1 );

    } while ( keep_running && s.front_panel_sw & 1 );
  }
#endif

  /****************************************************************/
#if 0
  printf("Mesa 4I36 test.\n");
  printf("Press the first console pushbutton to quit.\n");
  do {
    static int count = 0;
    FlameIO_read_all_sensors( &io, &params, &s );
    printf("%f %f %f %f\n", s.q.hipx, s.q.l.hipy, s.q.l.knee, s.q.l.ankley);
    delay_microseconds(100000);
    FlameIO_set_front_panel_LEDS( &io, count++ );
  } while ( (s.front_panel_sw & 1) );
#endif

  /****************************************************************/


  printf( "Shutting down Flame I/O.\n");
  FlameIO_close( &io );

#if 0
  printf( "Pausing....\n");  
  delay_microseconds( 1000000 );
#endif

  printf("\nFlame I/O test done.\n");  

  return 0;
}

