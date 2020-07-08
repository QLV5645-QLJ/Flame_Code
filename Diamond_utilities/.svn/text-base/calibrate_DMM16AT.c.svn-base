// $Id: calibrate_DMM16AT.c,v 1.1 2005/12/07 15:27:47 garthz Exp $
// calibrate_DMM16AT.c : set up a Diamond Systems DMM16AT board using the Diamond Systems driver
//
// Version 2, Eelko van Breda, 2008,  Also calibrate AD converters
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

  printf("Diamond Systems DMM16AT D/A and A/D calibration program.\n");
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
    DSCADCALPARAMS adcalparams;

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
// initialize and set D/A parameters
    memset(&dacalparams, 0, sizeof(DSCDACALPARAMS));
    dacalparams.darange = 10;
    dacalparams.polarity = FALSE;   // bipolar
    dacalparams.fixed = FALSE;      // programmable range

// initialize and set A/D parameters
    memset(&adcalparams, 0, sizeof(DSCADCALPARAMS));
    adcalparams.adrange = 255; 		//calibrate all ad modes
    adcalparams.boot_adrange = 8;   // set power-up A/D mode to +/- 10V
    
// calibrate D/A    
    printf("Calibrating D/A.  Please wait.\n", argv[0]);
    result = dscDAAutoCal( dscb, &dacalparams );

    if ( result != DE_NONE ) {
      dscGetLastError(&errparams);
      printf("dscDAAutoCal failed: %s (%s)\n", dscGetErrorString(result), errparams.errstring);
      dscFreeBoard(dscb);
      return result;
    }

    printf("Verifying D/A calibration.\n" );
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

    printf("D/A Offset Error: %.4f D/A code, D/A Gain Error: %.4f D/A code\n\n\n", dacalparams.offset, dacalparams.gain);

// calibrate A/D
/*
    printf("Checking current A/D calibration.\n" );
    result = dscADCalVerify(dscb, &adcalparams);
    if ( result != DE_NONE ) {
      dscGetLastError(&errparams);
      printf("dscADCalVerify failed: %s (%s)\n", dscGetErrorString(result), errparams.errstring);
      dscFreeBoard(dscb);
      return result;
    }
    printf("A/D Offset Error: %.4f A/D code, A/D Gain Error: %.4f A/D code\n", adcalparams.ad_offset, adcalparams.ad_gain);
   
  */  
    printf("Calibrating A/D.  Please wait.\n", argv[0]);
    result = dscADAutoCal( dscb, &adcalparams );
     if ( result != DE_NONE ) {
      dscGetLastError(&errparams);
      printf("dscADAutoCal failed: %s (%s)\n", dscGetErrorString(result), errparams.errstring);
      dscFreeBoard(dscb);
      return result;
    }

    printf("Verifying new A/D calibration.\n" );
    result = dscADCalVerify(dscb, &adcalparams);
    if ( result != DE_NONE ) {
      dscGetLastError(&errparams);
      printf("dscADCalVerify failed: %s (%s)\n", dscGetErrorString(result), errparams.errstring);
      dscFreeBoard(dscb);
      return result;
    }
    printf("A/D Offset Error: %.4f A/D code, A/D Gain Error: %.4f A/D code\n", adcalparams.ad_offset, adcalparams.ad_gain);
   
    
    
    printf("%s done, closing driver.\n", argv[0]);
    dscFreeBoard(dscb);
  }
}
