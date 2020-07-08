// $Id: Flame_off.c,v 1.1 2005/12/07 15:27:25 garthz Exp $
// Flame_off.c : power shutdown test program
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
#include <utility/utility.h>

/****************************************************************/
static FlameIO_state_t s;        // the global state structure (i.e. blackboard)
static FlameIO_params_t params;  // the hardware parameters structure 
static FlameIO io;               // a static area to hold the driver data

/****************************************************************/
int main(int argc, char **argv)
{
  int i;

  printf("Turning off Flame power.\n");
  printf("This is meant to be run on the PC/104 stack for the Flame Biped.\n");
  printf("It does not use the RTAI real time system.\n");
  logprintf( "Opening up all Flame I/O hardware.\n");

  FlameIO_init ( &io );

  // Initialize our offset and scale structures.
  FlameIO_initialize_default_parameters( &params );

  // Read voltages.
  FlameIO_read_all_sensors( &io, &params, &s );
  printf("computer battery: %f\n", s.battery.com_un );
  printf("computer input:   %f\n", s.battery.com_sw );
  printf("   motor battery: %f\n", s.battery.mot_un );
  printf("   motor input:   %f\n", s.battery.mot_sw );

  // Make disk safe.
  logprintf("Flushing discs.\n");
  system("sync");

  logprintf("Remounting root read-only.\n");
  system("mount /dev/hda1 / -o ro -o remount");

  // Read voltages.
  FlameIO_read_all_sensors( &io, &params, &s );
  printf("computer battery: %f\n", s.battery.com_un );
  printf("computer input:   %f\n", s.battery.com_sw );
  printf("   motor battery: %f\n", s.battery.mot_un );
  printf("   motor input:   %f\n", s.battery.mot_sw );

  // Turn off power relay.
  {
    int i;
    printf( "Power off countdown: ");
    for (i = 0; i < 6; i++) {
      printf( ". "); fflush(stdout);
      FlameIO_set_front_panel_LEDS( &io, 1 << i);
      delay_microseconds( 500000 );
    }
    FlameIO_set_front_panel_LEDS( &io, 0xff );
  }

  printf( "Shutting off Flame power.\n");

  FlameIO_write_power_control_outputs( &io, FLAME_SELF_POWER_OFF | FLAME_MOTOR_POWER_OFF );

  printf( "Busywaiting....\n");
  while(1);

}

