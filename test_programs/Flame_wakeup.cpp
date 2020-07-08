// Flame_wakeup.c : power on initialization program
//
// Copyright (c) 2005-2006 Garth Zeglin. Provided under the terms
// of the GNU General Public License as included in the top level
// directory.
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

#include <hardware_drivers/FlameIO.h>
#include <utility/utility.h>

/****************************************************************/
static FlameIO_state_t s;        // the global state structure (i.e. blackboard)
static FlameIO_params_t params;  // the hardware parameters structure 
static FlameIO io;               // a static area to hold the driver data

/****************************************************************/
int main(int argc, char **argv)
{
  int i;

  printf("Flame waking up.\n");
  printf("This is meant to be run on the PC/104 stack for the Flame Biped.\n");
  printf("It does not use the RTAI real time system.\n");
  logprintf( "Opening up all Flame I/O hardware.\n");

  FlameIO_init ( &io );

  // Initialize our offset and scale structures.
  FlameIO_initialize_default_parameters( &params );

  // Initialize the motor power relay to be driven off.
  FlameIO_write_power_control_outputs( &io, FLAME_MOTOR_POWER_OFF );

  // Read voltages.
  FlameIO_read_all_sensors( &io, &params, &s );
  printf("computer battery: %f\n", s.battery.com_un );
  printf("computer input:   %f\n", s.battery.com_sw );
  printf("   motor battery: %f\n", s.battery.mot_un );
  printf("   motor input:   %f\n", s.battery.mot_sw );

  // Flash all the lights a few times.
#define INTERVAL_IN_USEC 25000
  
  for (i = 0; i < 24; i++) {
    FlameIO_set_front_panel_LEDS(  &io, 1 << (i % 6));
    FlameIO_set_motor_driver_LEDS( &io, 1 << (i % 4));
    FlameIO_set_body_LEDS(         &io, 1 << (i % 3));
    delay_microseconds( INTERVAL_IN_USEC );
  }
  FlameIO_set_front_panel_LEDS(  &io, 0 );
  FlameIO_set_motor_driver_LEDS( &io, 0 );
  FlameIO_set_body_LEDS(         &io, 0 );

  printf("wakeup done.\n");
}

