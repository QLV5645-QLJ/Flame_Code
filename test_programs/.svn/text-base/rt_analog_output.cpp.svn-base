// $Id: rt_analog_output.c,v 1.7 2005/12/12 17:30:59 garthz Exp $
// rt_analog_output.c : test program for running a single analog output using RTAI timing
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
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

#include <real_time_support/RTAI_user_space_realtime.h>
#include <real_time_support/messages.h>
#include <hardware_drivers/FlameIO.h>
#include <hardware_drivers/DMM16AT.h>

/****************************************************************/
#define FREQ 50.0                        // Hz  
#define SAMPLINGRATE 1000                // Hz
#define PERIOD 1000000000/SAMPLINGRATE   // in nanoseconds
#define RUNTIME 20                       // in seconds

// opaque handle for the real time control thread task
static struct realtime_task *control_task = NULL;

/****************************************************************/
static int realtime_thread( long long timestamp, void *userdata );

static DMM16AT *dmm;

int main(int argc, char **argv) 
{


  printf("rt_analog_output starting up.\n");

  control_task = create_RTAI_user_space_task( REALTIME_PROCESS_NAME );
  if ( control_task == NULL ) {
    printf("Failed to initialize RTAI interface.\n");
    exit(1);
  }

  // Initialize hardware interface.
  enable_IO_port_access();

  printf( "Opening up DMM-16-AT.\n");
  dmm = DMM16AT_init( DMM16AT_alloc() ) ;


  printf("starting real time thread, it should run for %d seconds...\n", RUNTIME);
  run_RTAI_user_space_realtime_periodic_thread( control_task, realtime_thread, PERIOD /* nanoseconds */, NULL );

  printf("Real time thread exited.\n");
  
  shutdown_RTAI_user_space_task( control_task );
  DMM16AT_dealloc( dmm );

  printf("rt_analog_output done.\n");
}


/****************************************************************/
// This function will be called at regular intervals.
// No system calls are allowed inside this function. 
// Returns true if the periodic calls should continue.

static int realtime_thread( long long timestamp, void *userdata )
{
  static int count = 0;
  static double phase = 0.0;
  short value;

  count++;

  value = (short) (DMM16AT_MAX_ANALOG_OUTPUT * sin(phase));
  phase += (FREQ / SAMPLINGRATE ) * 2.0*M_PI;
  DMM16AT_write_analog_output( dmm, 0, value );

  return (count < RUNTIME*SAMPLINGRATE);
}

