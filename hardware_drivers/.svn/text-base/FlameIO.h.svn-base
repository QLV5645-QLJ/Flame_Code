// $Id: FlameIO.h,v 1.8 2005/12/16 16:13:20 garthz Exp $ 
// FlameIO.h : composite driver for all the analog and digital I/O on the Flame biped robot
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#ifndef FLAMEIO_H_INCLUDED
#define FLAMEIO_H_INCLUDED

#include <hardware_drivers/AthenaDAQ.h>
#include <hardware_drivers/DMM16AT.h>
#include <hardware_drivers/Mesanet_4I36.h>
#include <hardware_drivers/FlameIO_defs.h>

/****************************************************************/
// Define a device structure to hold all hardware configuration
// and state information.  This is not written as an opaque
// structure, but is subject to change, so it is preferable to
// use access methods rather than modify the fields directly.

typedef struct {

  AthenaDAQ    daq;      // the interface hardware on the Athena CPU board
  DMM16AT      dmm;      // the DMM16AT I/O card
  Mesanet_4I36 mesa1;    // a Mesa encoder counter card
  Mesanet_4I36 mesa2;    // a Mesa encoder counter card

  // driver status flags
  unsigned int initialized      :1;      // true if this data structure is valid
  unsigned int closed           :1;      // true if hardware has been closed
  
} FlameIO;

/****************************************************************/
// The object creation and destruction methods loosely follow the Objective-C style.

// Create an empty FlameIO object.
extern FlameIO *FlameIO_alloc(void);

// Initialize a FlameIO device.  The structure may be statically allocated.
extern FlameIO *FlameIO_init( FlameIO *io );

// Shut down all FlameIO devices.  The driver structure may be
// statically allocated, i.e., this doesn't deallocate any memory.
extern void FlameIO_close( FlameIO *io );

// De-allocate a dynamically allocated FlameIO object, stopping the
// subsidiary devices if still active.
extern void FlameIO_dealloc( FlameIO *io );

// Test if all devices initialized properly.
extern int FlameIO_is_ready( FlameIO *io );

/****************************************************************/
// Convenient I/O operations.

// Each of the LED write functions writes a set of bits starting with the LSB.
// There are six front panel LEDs.
extern void FlameIO_set_front_panel_LEDS( FlameIO *io, unsigned char leds );

// There are three body LEDs.
extern void FlameIO_set_body_LEDS( FlameIO *io, unsigned char leds );

// There are four motor driver board LEDS.
#define FLAME_MLED_LEFT0 1
#define FLAME_MLED_LEFT1 2
#define FLAME_MLED_RIGHT0 4
#define FLAME_MLED_RIGHT1 8
extern void FlameIO_set_motor_driver_LEDS( FlameIO *io, unsigned char leds );

// Low-level control of power control outputs according to the following flags:
#define FLAME_SELF_POWER_OFF  1
#define FLAME_MOTOR_POWER_ON  2
#define FLAME_MOTOR_POWER_OFF 4
extern void FlameIO_write_power_control_outputs( FlameIO *io, unsigned char flags );

/****************************************************************/
// Bit masks for the front panel switches and LEDS.

// The numbering of the switches goes left to right.  This is opposite
// the bit position order because the front panel board was installed
// upside-down.

#define PUSHBUTTON0	8           // the leftmost
#define PUSHBUTTON1	4
#define PUSHBUTTON2	2
#define PUSHBUTTON3	1	    // the rightmost

#define TOGGLE0		128	    // the leftmost
#define TOGGLE1		64
#define TOGGLE2		32
#define TOGGLE3		16	    // the rightmost 

#define LEDLEFT		32	    // the leftmost
#define LED0		16          // matching the switch position
#define LED1		8           // matching the switch position
#define LED2		4           // matching the switch position
#define LED3		2           // matching the switch position
#define LEDRIGHT	1	    // the rightmost

#define NUMPANELLEDS    6

// Macros to hide the details of the switch polarity.  The pushbuttons
// are 1 when inactive, the toggle switches are 0 when up and 1 when
// down. (This toggle value is opposite the original design intent,
// since the front panel board was installed upside-down.)

#define FLAME_PUSHBUTTON_PRESSED(bits, mask) (!((bits) & (mask)))
#define FLAME_TOGGLE_UP(bits, mask)          (!((bits) & (mask)))

/****************************************************************/
// Initialize general I/O state structure values.
extern void FlameIO_initialize_default_state( FlameIO_state_t *s );

// Initialize the I/O state structure offsets and scaling factors.
extern void FlameIO_initialize_default_parameters( FlameIO_params_t *p );

// Read all inputs and perform conversion to real world units.
extern void FlameIO_read_all_sensors( FlameIO *io, FlameIO_params_t *p, FlameIO_state_t *s);

/****************************************************************/
// Update the torque command outputs from the tau state
// variables.  This will send a zero command signal for disabled
// channels, but does not enable or disable the drivers.
extern void FlameIO_write_torque_commands( FlameIO *io, FlameIO_params_t *p, FlameIO_state_t *s );

// Enable or disable the motor drivers. A 1 in each position in
// the flags enables one driver (i.e., raises the /INHIBIT line,
// de-asserting inhibit)
extern void FlameIO_enable_motor_drivers( FlameIO *io, unsigned char flags );

// Assignments of motor axis to motor driver channel.  This must 
// match the actual robot driver assignment.

#define HIPX_DRIVER    3	// top left driver (as seen from rear of machine)
#define LKNEE_DRIVER   2
#define LHIPY_DRIVER   1
#define LANKY_DRIVER   0	// bottom left driver

#define UNUSED_DRIVER  7	// top right driver (as seen from rear of machine)
#define RHIPY_DRIVER   6
#define RKNEE_DRIVER   5
#define RANKY_DRIVER   4	// bottom right driver

// Bit masks to define motor channels in the powered and fault bitfields.
#define FLAME_HIPX_MOTOR      ( 1 << HIPX_DRIVER  )
#define FLAME_LHIPY_MOTOR     ( 1 << LHIPY_DRIVER )
#define FLAME_LKNEE_MOTOR     ( 1 << LKNEE_DRIVER )
#define FLAME_LANKLEY_MOTOR   ( 1 << LANKY_DRIVER )
#define FLAME_RHIPY_MOTOR     ( 1 << RHIPY_DRIVER )
#define FLAME_RKNEE_MOTOR     ( 1 << RKNEE_DRIVER )
#define FLAME_RANKLEY_MOTOR   ( 1 << RANKY_DRIVER )

#define FLAME_ALL_MOTORS      ( 0x7f )

// Current setup of Flame PC/104 hardware. These aren't really
// public values, but it is better to keep them with the other
// hardware configuration parameters.
#define DMM16ATBASE    0x300    // set via jumpers on J6
#define MESA1BASE      0x220
#define MESA2BASE      0x230

// Power supply voltage thresholds
#define FLAME_MIN_MOTOR_VOLTAGE      27.0
#define FLAME_STABLE_MOTOR_VOLTAGE   29.0

// Default scaling for each joint sensor, in radians/count.
#define TWOPI (2.0*M_PI)
#define MOTOR_CPR           (500*4)     // counts per rev for the HEDS 5xxx-A11 motor encoders
#define JOINT_CPR           (7500*4)    // counts per rev for the joint encoders
#define HIPX_MOT_RATIO      1.0         // gear ratio of the motor itself
#define HIPY_MOT_RATIO      79 	        // gear ratio of the motor itself
#define KNEE_MOT_RATIO      51 	        // gear ratio of the motor itself
#define ANKLEY_MOT_RATIO    51          // gear ratio of the motor itself
#define ANKLEY_JOINT_RATIO  4.375//=70.0/16.0//old:46.5/16     // lever ratio of the ankle joint, distance of ankle lever / radius ankle drive pulley
#define HIPY_JOINT_RATIO    2.0         // ratio of hip pulley
#define KNEE_JOINT_RATIO    1.0         // ratio of knee pulleys, which are identical

// The hipx axis involves a linkage, so it will eventually need some kind of calibrated function.
#define HIPX_JOINT_RATIO   416     // (hipx motor angle) / (joint angle)

#define Z12A8_IMON_SCALE 4.0    // motor driver output amps/volt
// scale factor for the Maxon RE35 90W motor with 30V winding
#define MAXON_RE35_TORQUE_CONSTANT 0.0389    // Newton-meter / Ampere
// this assumes the drivers put out the maximum transient current at the maximum rated input command
#define Z12A8_MAX_CURRENT  12  // amperes

#endif // FLAMEIO_H_INCLUDED

