// $Id: DMM16AT_test.c,v 1.6 2005/12/14 08:32:07 garthz Exp $
// DMM16AT_test.c : test program for operating the PC/104 hardware without the real time system
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
#include <math.h>
#include <string.h>
#include <pthread.h>
#include <stdarg.h>
#include <signal.h>

#include <hardware_drivers/FlameIO.h>
#include <hardware_drivers/DMM16AT.h>
#include <hardware_drivers/IO_permissions.h>
#include <real_time_support/POSIX_soft_realtime.h>
#include <utility/utility.h>

/****************************************************************/
volatile static int keep_running = 1;

// User break handler for the main thread.
static void main_thread_break_handler(int sig)
{
  errprintf("\n\nDMM16AT_test main thread received break signal, quitting.\n");
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
    errprintf("DMM16AT_test main thread unable to install SIGHUP handler: %s\n", strerror( errno ) );
  }

  if ( sigaction( SIGINT, &break_sigaction, NULL ) ) {
    errprintf("DMM16AT_test main thread unable to install SIGINT handler: %s\n", strerror( errno ) );
  }

  if ( sigaction( SIGQUIT, &break_sigaction, NULL ) ) {
    errprintf("DMM16AT_test main thread unable to install SIGQUIT handler: %s\n", strerror( errno ) );
  }
}

/****************************************************************/
#define FREQ 50.0
#define SAMPLINGRATE 1000.0

static void dmm16at_analog_output_test( DMM16AT *dmm )
{
  double phase = 0.0;
  short value;
  int i;

  printf( "Beginning D/A write test.\n");
  initialize_POSIX_soft_realtime();

  while ( keep_running ) {
    value = DMM16AT_MAX_ANALOG_OUTPUT * sin(phase);

    phase += (FREQ / SAMPLINGRATE ) * 2.0*M_PI;

    DMM16AT_write_analog_output( dmm, 0, value );
    delay_microseconds( (unsigned long) (1000000 / SAMPLINGRATE) );

  }
}
#undef FREQ
#undef SAMPLINGRATE

/****************************************************************/
#define SAMPLINGRATE 200.0

static void dmm16at_digital_output_test( DMM16AT *dmm )
{
  int i;

  printf( "Beginning digital I/O write test.\n");
  initialize_POSIX_soft_realtime();

  while ( keep_running ) {

    DMM16AT_write_digital_output_byte( dmm, i & 0xff );
    delay_microseconds( (unsigned long) (1000000 / SAMPLINGRATE) );

    if ( !(i & 255) ) {
      putchar('*'); 
      fflush(stdout);
    }
  }
}
#undef SAMPLINGRATE

/****************************************************************/
static void dmm16at_digital_input_test( DMM16AT *dmm )
{
  int i;

  printf( "Beginning digital I/O read test.\n");
  printf( "Press control-C to quit.\n");

  while ( keep_running ) {
    unsigned char value, mask = 1;
    int b;
    char string[9];

    value = DMM16AT_read_digital_input_byte( dmm );

    // Create a binary string, since printf can't do it.
    for ( b=0; b < 8; b++, mask <<= 1 ) string[7-b] = ( value & mask ) ? '1' : '0';
    string[8] = 0;
    printf(" digital inputs: %s\r", string);
  }
}

/****************************************************************/
static void dmm16at_analog_input_test( DMM16AT *dmm )
{
  int i;

  printf( "Beginning analog I/O read test.\n");
  printf( "Press control-C to quit.\n");

  while ( keep_running ) {
    int channel;

    DMM16AT_read_all_analog_inputs( dmm );

    for ( channel = 0; channel < 8; channel++ ) {
      printf("%04x ", dmm->analog_inputs[ channel ] );
    }
    printf("\r");
  }
}

/****************************************************************/
int main(int argc, char **argv)
{
  int action = 0;
  DMM16AT *dmm;

  printf("DMM16AT I/O test.\n");
  printf("This is meant to be run on the PC/104 stack for the Flame Biped.\n");
  printf("It does not use the RTAI real time system.\n");
  printf("usage: %s [<test-number>]\n", argv[0]);

  enable_IO_port_access();
  install_break_handler();

  logprintf( "Opening up DMM-16-AT.\n");
  dmm = DMM16AT_init ( DMM16AT_alloc() );

  if ( dmm == NULL ) {
    errprintf( "Unable to open hardware: %s", strerror( errno) );
    exit(1);
  }

  if ( argc > 1 ) {
    action = atoi(argv[1]);
  }

  switch( action ) {
  case 0:   dmm16at_analog_output_test( dmm ); break;
  case 1:   dmm16at_analog_input_test( dmm ); break;
  case 2:   dmm16at_digital_input_test( dmm ); break;
  case 3:   dmm16at_digital_output_test( dmm ); break;
  }

  printf( "Shutting down DMM-16-AT.\n");
  DMM16AT_dealloc( dmm );

  printf("\nDMM16AT I/O test done.\n");  
}
