// xsense_glue.cpp : C-compatible interface to xsens IMU C++ driver code
//
// originally derived from the xsense sample code "MTComm Example.cpp"

// modifications Copyright (c) 2006 Garth Zeglin. Provided under
// the terms of the GNU General Public License as included in the
// top level directory.

// Original file copyright:
//  This file is part of the Xsens SDK Code Samples.
//
//  Copyright (C) Xsens Technologies B.V., 2005-2006.  All rights reserved.

//  This source code is intended only as a supplement to Xsens
//  Development Tools and/or documentation.  See these other
//  materials for detailed information regarding Xsens code samples.

//  THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
//  KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//  PARTICULAR PURPOSE.

//****************************************************************/

#include <sys/ioctl.h>
#include <math.h>
#include "MTComm.h"

// Declare the interface functions to have C linkage.
//extern "C" {
#include "xsens.h"
//};

static CMTComm mtcomm;
static char deviceName[]   = "/dev/ttyS0";
static int outputMode      = OUTPUTMODE_ORIENT | OUTPUTMODE_CALIB;//orient and calib//(2 << 1); // orientation data
// static int outputSettings  = OUTPUTSETTINGS_ORIENTMODE_EULER | OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
static int outputSettings  = OUTPUTSETTINGS_ORIENTMODE_MATRIX | OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
static unsigned short numDevices;

static unsigned char PARAM_FilterSetting;
static float VALUE_FilterSetting;
static bool STORE_FilterSetting;


//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized global MTComm class
static bool doMtSettings(void) 
{
  unsigned long tmpOutputMode, tmpOutputSettings;
  unsigned short tmpDataLength;

  // Put MTi/MTx in Config State
  if(mtcomm.writeMessage(MID_GOTOCONFIG) != MTRV_OK){
    printf("No device connected\n");
    return false;
  }

  // Get current settings and check if Xbus Master is connected
  if (mtcomm.getDeviceMode( &numDevices ) != MTRV_OK) {
    if (numDevices == 1)
      printf("MTi / MTx has not been detected\nCould not get device mode\n");
    else
      printf("Not just MTi / MTx connected to Xbus\nCould not get all device modes\n");
    return false;
  }
	
  // Check if Xbus Master is connected
  mtcomm.getMode(tmpOutputMode, tmpOutputSettings, tmpDataLength, BID_MASTER);
  if (tmpOutputMode == OUTPUTMODE_XM)	{
    // If Xbus Master is connected, attached Motion Trackers should not send sample counter
    outputSettings &= 0xFFFFFFFF - OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
  }
	
  // Set output mode and output settings for each attached MTi/MTx
  for (int i = 0; i < numDevices; i++) {
    if (mtcomm.setDeviceMode(outputMode, outputSettings, BID_MT + i) != MTRV_OK) 
    {
      printf("Could not set (all) device mode(s)\n");
      return false;
    }
    // disable magneto sensor
/*    if (mtcomm.setSetting(MID_SETFILTERSETTINGS, PARAM_FilterSetting = 1, VALUE_FilterSetting = 0.0, STORE_FilterSetting = 0, BID_MT + i) != MTRV_OK) 
    {
          printf("Could not disable (all) magneto sensor(s)\n");
          return false;
    }
*/
/*    // set filter value
    if (mtcomm.setSetting(MID_SETFILTERSETTINGS, PARAM_FilterSetting = 0, VALUE_FilterSetting = 1.0, STORE_FilterSetting = 0, BID_MT + i) != MTRV_OK) 
    {
          printf("Could not set filter value\n");
          return false;
    }
*/
  }

  // Put MTi/MTx in Measurement State
  mtcomm.writeMessage(MID_GOTOMEASUREMENT);

  return true;
}

//////////////////////////////////////////////////////////////////////////
// initialize the IMU on the default serial port
// returns true on error
// this function will be called from C and has C linkage

int 
open_xsens_IMU ( void )
{
  // Open and initialize serial port
  if (mtcomm.openPort(deviceName) != MTRV_OK) {
    fprintf(stderr, "init_xsens_IMU: cannot open serial port %s\n", deviceName);
    return -1;  // MTRV_INPUTCANNOTBEOPENED
  }	

  if( doMtSettings() == false ) {
    return -1;  // MTRV_UNEXPECTEDMSG;
  }
  
  return 0; // no error
}

int
poll_xsens_IMU( xsens_IMU_data_t *result )
{
  unsigned char data[MAXMSGLEN];
  short datalen;
  float gdata[6] = {0};
  float fdata[18] = {0};
  unsigned short samplecounter;

  int err = mtcomm.readDataMessage(data, datalen);
  
  if ( err == MTRV_OK ) {
    mtcomm.getValue(VALUE_SAMPLECNT, samplecounter, data, BID_MASTER);
    mtcomm.getValue(VALUE_CALIB_GYR, gdata, data, BID_MT);
    mtcomm.getValue(VALUE_ORIENT_MATRIX, fdata, data, BID_MT);

    result->samples = samplecounter;
    result->roll    = atan((-fdata[7]*fdata[0]+fdata[6]*fdata[1])/(-fdata[7]*fdata[3]+fdata[6]*fdata[4]));//fdata[0];
//    result->pitch   = copysign(atan(sqrt((fdata[0]*fdata[0]+fdata[1]*fdata[1])/(fdata[6]*fdata[6]+fdata[7]*fdata[7]))),
//    						fdata[0]*fdata[6]+fdata[1]*fdata[7]);
    result->pitch   = atan((-fdata[4]*fdata[0]+fdata[3]*fdata[1])/(-fdata[4]*fdata[6]+fdata[3]*fdata[7]));
    result->yaw     = atan((-fdata[2]*fdata[7]+fdata[1]*fdata[8])/(-fdata[2]*fdata[4]+fdata[1]*fdata[5]));//fdata[2];
    
    result->rolld	= gdata[2];
    result->pitchd  = -gdata[1];
    return 0;

  } else { 
    return -1;
  }
}

int
reset_orientation_xsens_IMU( void )
{
	// reset orientation
	  mtcomm.writeMessage(MID_RESETORIENTATION, RESETORIENTATION_HEADING, LEN_RESETORIENTATION, BID_MT);
	  return 0;
}

int
close_xsens_IMU( void )
{
  // When done, close the serial port
  mtcomm.close();
  return 0;
}
