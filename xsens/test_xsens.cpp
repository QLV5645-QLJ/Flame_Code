// test_xsens.c : test of C to C++ glue for the xsens IMU interface

// Copyright (c) 2006 Garth Zeglin. Provided under the terms of
// the GNU General Public License as included in the top level
// directory.

#include <stdio.h>
#include <stdlib.h>
#include "xsens.h"

int main( int argc, char **argv ) 
{
  xsens_IMU_data_t data;
  int err;
  int i;

  err = open_xsens_IMU();
  
  if ( err ) {
    printf("Unable to open IMU.\n");
    exit(1);
  }

  for (i = 0; i < 1000; i++) {
    err = poll_xsens_IMU( &data );
    
    if (err) {
      printf("Error polling IMU, quitting.\n");
      break;
    }

    printf("Samples: %d\tRoll: %6.1f\tPitch:%6.1f\tYaw:%6.1f\n",
	   data.samples, data.roll, data.pitch, data.yaw );
  }
  close_xsens_IMU();
}
