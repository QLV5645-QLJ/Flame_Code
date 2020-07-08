// $Id: message_format.h,v 1.2 2005/12/15 17:16:46 garthz Exp $
// message_format.h : special purpose application message format
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

// Define a special purpose message type for just this application.

#ifndef MESSAGE_FORMAT_H_INCLUDED
#define MESSAGE_FORMAT_H_INCLUDED

#include <real_time_support/messages.h>
#include <hardware_drivers/FlameIO_defs.h>
#include "FlameJoints.h"
//#include "globals.h"

// Locally defined message structures, used to communicate
// between the real time process and non-real time processes.
// These will only be interpreted correctly within this
// application.  The structure must be similar in layout to the
// message_t union.

#define MSG_SENSOR_DATA (MSG_LASTMESSAGENUM + 1)
#define MSG_IMU_DATA    (MSG_SENSOR_DATA+1)      // IMU data from helper process to real time

// The state structures must have been previously declared by including system.h 
// and real_time_support/FlameIO_defs.h

struct sensor_message_t {
  struct _msg_hdr header;
  int serial_number;
  int local_protocol_version;

  // a complete state snapshot
  //controller_state_t control;
  CFlameJoints joints;
  FlameIO_state_t state;

};

// Definitions for the control.mode variable.
enum { MODE_INIT = 0,
       MODE_STARTUP,
       MODE_IDLE,
       MODE_BEGIN_SHUTDOWN,
       MODE_SHUTDOWN,
       MODE_POWER_UP_DRIVERS,
       MODE_POWER_DN_DRIVERS,
       MODE_DEMO,
       MODE_STANDING,
       MODE_EXERCISE,
       MODE_START_WALKING,
       MODE_WALKING,

       MODE_NO_MODE    // tag for end of list
};

// data from the inertial measurement unit
struct imu_data_message_t {
  struct _msg_hdr hdr;
  int samples;
  float yaw, pitch, roll, rolld, pitchd;
};


#endif // MESSAGE_FORMAT_H_INCLUDED
