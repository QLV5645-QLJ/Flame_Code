// $Id: test_DMM16AT.c,v 1.2 2005/12/14 08:33:40 garthz Exp $
// test_DMM16AT.c : simple test output from a Diamond Systems DMM16AT board using the Diamond Systems driver
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <signal.h>
#include <errno.h>

// This is for the Diamond System driver.
#include <dscud.h>


/****************************************************************/
volatile static int keep_running = 1;

// User break handler for the main thread.
static void main_thread_break_handler(int sig)
{
  printf("\n\nDMM16AT_test main thread received break signal, quitting.\n");
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
    printf("DMM16AT_test main thread unable to install SIGHUP handler: %s\n", strerror( errno ) );
  }

  if ( sigaction( SIGINT, &break_sigaction, NULL ) ) {
    printf("DMM16AT_test main thread unable to install SIGINT handler: %s\n", strerror( errno ) );
  }

  if ( sigaction( SIGQUIT, &break_sigaction, NULL ) ) {
    printf("DMM16AT_test main thread unable to install SIGQUIT handler: %s\n", strerror( errno ) );
  }
}

/****************************************************************/
int main ( int argc, char **argv )
{
  int result;
  ERRPARAMS errparams;

  printf("Diamond Systems DMM16AT D/A output test program.\n");
  printf("Warning: this should NOT be run with anything sensitive attached to outputs.\n");
  printf("This does not use RTAI, but must be run as root.\n");

  install_break_handler();

  // Try opening the driver.
  printf("Opening Diamond Systems driver.\n");
  result = dscInit(DSC_VERSION);
  if ( result != DE_NONE ) {
    dscGetLastError( &errparams );
    printf("dscInit failed: %s (%s)\n", dscGetErrorString(result), errparams.errstring);
    return result;
  }	

  // DSC_DMM16AT
  {
    DSCCB dsccb;
    DSCB dscb;
    DSCDASETTINGS dasettings;

    memset(&dsccb, 0, sizeof(DSCCB));
    dsccb.base_address = 0x300;
    dsccb.int_level = 7;

    printf("Opening DMM16AT board.\n");
    result = dscInitBoard( DSC_DMM16AT, &dsccb, &dscb);
    if ( result != DE_NONE ) {
      dscGetLastError( &errparams );
      printf("dscInitBoard failed: %s (%s)\n", dscGetErrorString(result), errparams.errstring);
      dscFreeBoard(dscb);
      return result;
    }	

    // configure the D/A settings
#if 0
    memset( &dasettings, 0, sizeof( DSCDASETTINGS ) );
    dasettings.polarity = BIPOLAR;
    dasettings.load_cal = TRUE;
    dasettings.range    = 10.0;

    printf("Configuring DMM16AT D/A converters.\n");
    result = dscDASetSettings( dscb, &dasettings );
    if ( result != DE_NONE ) {
      dscGetLastError(&errparams);
      printf("dscDASetSettings failed: %s (%s)\n", dscGetErrorString(result), errparams.errstring);
      dscFreeBoard(dscb);
      return result;
    }

    printf( "polarity: %s,  load calibration: %s,  range: %f\n",
	    ( dasettings.polarity) ? "unipolar" : "bipolar", 
	    ( dasettings.load_cal ) ? "yes" : "no",
	    dasettings.range );

#endif
    printf("generating sine wave on D/A output.\n");
    {
      int i = 0;
      double phase = 0.0;

      while ( keep_running ) {
	DSCDACODE value = (DSCDACODE) (2047 + 2047*sin(phase));

	result = dscDAConvert( dscb, 0 /*channel*/,  value );
	phase += 0.01 * M_PI; 

	if ( result != DE_NONE ) {
	  dscGetLastError(&errparams);
	  printf("dscDAConvert failed: %s (%s)\n", dscGetErrorString(result), errparams.errstring);
	  dscFreeBoard(dscb);
	  return result;
	}
      }
    }

    printf("%s done, closing driver.\n", argv[0]);
    dscFreeBoard(dscb);
  }
}
