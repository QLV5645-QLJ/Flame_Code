// $Id: calibrate_DMM16AT.c,v 1.1 2005/12/07 15:27:47 garthz Exp $
// calibrate_DMM16AT.c : set up a Diamond Systems DMM16AT board using the Diamond Systems driver
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>

// This is for the Diamond System driver.
#include <dscud.h>



int main ( int argc, char **argv )
{
  int result;
  ERRPARAMS errparams;

  printf("Diamond Systems DMM16AT D/A calibration program.\n");
  printf("Warning: this should NOT be run with anything sensitive attached to outputs.\n");
  printf("This does not use RTAI, but must be run as root.\n");

  // Try opening the driver.
  printf("Opening Diamond Systems driver.\n");
  result = dscInit(DSC_VERSION);
  if ( result != DE_NONE ) {
    dscGetLastError( &errparams );
    printf("dscInit failed: %s (%s)\n", dscGetErrorString(result), errparams.errstring);
    return result;
  }	

  // DSC_DMM16AT
  printf("Opening DMM16AT board.\n");
  {
    DSCCB dsccb;
    DSCB dscb;
    DSCDACALPARAMS dacalparams;

    memset(&dsccb, 0, sizeof(DSCCB));
    dsccb.base_address = 0x300;
    dsccb.int_level = 7;

    result = dscInitBoard( DSC_DMM16AT, &dsccb, &dscb);
    if ( result != DE_NONE ) {
      dscGetLastError( &errparams );
      printf("dscInitBoard failed: %s (%s)\n", dscGetErrorString(result), errparams.errstring);
      dscFreeBoard(dscb);
      return result;
    }	

    memset(&dacalparams, 0, sizeof(DSCDACALPARAMS));
    dacalparams.darange = 10;
    dacalparams.polarity = FALSE;   // bipolar
    dacalparams.fixed = FALSE;      // programmable range

    printf("Calibrating D/A.  Please wait.\n", argv[0]);

    result = dscDAAutoCal( dscb, &dacalparams );

    if ( result != DE_NONE ) {
      dscGetLastError(&errparams);
      printf("dscDAAutoCal failed: %s (%s)\n", dscGetErrorString(result), errparams.errstring);
      dscFreeBoard(dscb);
      return result;
    }

    printf("Verifying calibration.\n" );

    result = dscDACalVerify(dscb, &dacalparams);
    if ( result != DE_NONE ) {
      dscGetLastError(&errparams);
      printf("dscDACalVerify failed: %s (%s)\n", dscGetErrorString(result), errparams.errstring);
      dscFreeBoard(dscb);
      return result;
    }

    printf( "range: %s,  polarity: %s,  range: %f\n",
	    ( dacalparams.fixed ) ? "fixed" : "programmable", 
	    (dacalparams.polarity) ? "unipolar" : "bipolar", 
	    dacalparams.darange );

    printf("D/A Offset Error: %.4f D/A code, D/A Gain Error: %.4f D/A code\n", dacalparams.offset, dacalparams.gain);

    printf("%s done, closing driver.\n", argv[0]);
    dscFreeBoard(dscb);
  }
}
