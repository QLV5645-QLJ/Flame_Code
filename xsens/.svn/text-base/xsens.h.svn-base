// xsens.h : convenient C-compatible interface to xsens IMU C++ driver code

// Copyright (c) 2006 Garth Zeglin. Provided under the terms of
// the GNU General Public License as included in the top level
// directory.

#ifndef __XSENS_H_INCLUDED__
#define __XSENS_H_INCLUDED__


typedef struct {
  int samples;
  float yaw, pitch, roll, pitchd, rolld;
} xsens_IMU_data_t;

// Each of these functions returns true on error and 0 on success.
extern int open_xsens_IMU ( void );
extern int poll_xsens_IMU( xsens_IMU_data_t *result );
extern int reset_orientation_xsens_IMU( void );
extern int close_xsens_IMU( void );

#endif // __XSENS_H_INCLUDED__
