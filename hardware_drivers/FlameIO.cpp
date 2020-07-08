// $Id: FlameIO.c,v 1.7 2005/12/16 16:13:20 garthz Exp $
/// FlameIO.c : composite driver for all the analog and digital I/O on the Flame biped robot
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <asm/io.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <signal.h>

#include "FlameIO.h"
#include "IO_permissions.h"

// Define logprintf, errprintf.
#include <utility/utility.h>

/****************************************************************/
// Define symbols for the encoder channel assignment.  Mesa1 is 0-7 and Mesa2 is 8-15.

#define MESA1_ENCODERS     0
#define MESA2_ENCODERS     8  
#define ENCODER_CHANNELS   16


#define HIPX_ENCODER   	   4 
#define LHIPY_ENCODER      5 
#define LKNEE_ENCODER      6 
#define LANKLEY_ENCODER    7 
#define LANKLEX_ENCODER    12
#define LHIPYMOT_ENCODER   13
#define LKNEEMOT_ENCODER   14
#define LANKLEYMOT_ENCODER 15
#define UNUSED_ENCODER     0 
#define RHIPY_ENCODER      1 
#define RKNEE_ENCODER      2 
#define RANKLEY_ENCODER    3 
#define RANKLEX_ENCODER    8 
#define RHIPYMOT_ENCODER   9 
#define RKNEEMOT_ENCODER   10
#define RANKLEYMOT_ENCODER 11


/****************************************************************/
// Create an empty FlameIO object.
FlameIO *
FlameIO_alloc(void)
{
  FlameIO *io = (FlameIO *) calloc (1, sizeof( FlameIO ) );
}

/****************************************************************/
// Auxiliary routine to configure the index pulse mode for an encoder channel.
static void 
configure_index_pulse_mode( FlameIO *io, int channel, int type )
{
  if ( channel < MESA2_ENCODERS ) {
    Mesanet_4I36_configure_index( &io->mesa1, channel, type );
  } else {
    Mesanet_4I36_configure_index( &io->mesa2, channel - MESA2_ENCODERS, type );
  }
}

  /****************************************************************/
// Initialize a FlameIO device.  The structure may be statically allocated.
FlameIO *
FlameIO_init( FlameIO *io )
{
  // This only works if root, try enabling I/O permission.
  enable_IO_port_access();

  // Initialize all the drivers.
  AthenaDAQ_init ( &io->daq );
  DMM16AT_init_with_address ( &io->dmm, DMM16ATBASE ) ;
  Mesanet_4I36_init_with_address( &io->mesa1, MESA1BASE );
  Mesanet_4I36_init_with_address( &io->mesa2, MESA2BASE );

  io->initialized = 1;
  io->closed = 0;


  // Configure the hardware for the specific I/O setup on the Flame biped robot.

  // Make sure PORT A and PORT B will start in a reasonable state
  AthenaDAQ_write_digital_output_byte( &io->daq, ATHENADAQ_PORTA, 0);
  AthenaDAQ_write_digital_output_byte( &io->daq, ATHENADAQ_PORTB, 0);

  // Port C is used to read the front panel switches, Ports A and B will now be outputs.
  AthenaDAQ_configure_digital_direction( &io->daq, ATHENADAQ_PORTCL_INPUT | ATHENADAQ_PORTCH_INPUT );

  // The DMM16 digital output controls the /INHIBIT lines on the motor drivers; 0 == not energized.
  DMM16AT_write_digital_output_byte( &io->dmm, 0 );

  // Configure the index pulse interpretation on the encoder
  // channels.  The configure_index_pulse_mode function is in
  // this file.

  // A first cut is to configure all the motor axes to
  // RESET_ONCE, to pick up the relative zero and then keep track
  // as it spins around, and all the joint axes to RESET_ALWAYS,
  // since they never go a complete revolution, and the index
  // pulse can then always indicate a fixed angular position.

  configure_index_pulse_mode( io, LHIPY_ENCODER     , MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, LKNEE_ENCODER     , MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, LANKLEY_ENCODER   , MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, LANKLEX_ENCODER   , MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, RHIPY_ENCODER     , MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, RKNEE_ENCODER     , MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, RANKLEY_ENCODER   , MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, RANKLEX_ENCODER   , MESANET_IDX_MODE_RESET_ONCE );

  configure_index_pulse_mode( io, HIPX_ENCODER      , MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, LHIPYMOT_ENCODER  , MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, LKNEEMOT_ENCODER  , MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, LANKLEYMOT_ENCODER, MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, RHIPYMOT_ENCODER  , MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, RKNEEMOT_ENCODER  , MESANET_IDX_MODE_RESET_ONCE );
  configure_index_pulse_mode( io, RANKLEYMOT_ENCODER, MESANET_IDX_MODE_RESET_ONCE );

  return io;
}

/****************************************************************/
// Shut down all FlameIO devices.  The driver structure may be
// statically allocated, i.e., this doesn't deallocate any memory.

void 
FlameIO_close( FlameIO *io )
{
  if ( io != NULL && io->initialized ) { 
    io->closed = 1;

    AthenaDAQ_close(    &io->daq );
    DMM16AT_close(      &io->dmm );
    Mesanet_4I36_close( &io->mesa1 );
    Mesanet_4I36_close( &io->mesa2 );

  }  
}

/****************************************************************/
// De-allocate a dynamically allocated FlameIO object, stopping the
// subsidiary devices if still active.
void 
FlameIO_dealloc( FlameIO *io )
{
  if ( io != NULL ) { 
    if ( io->initialized && !io->closed ) FlameIO_close( io );

    AthenaDAQ_dealloc(    &io->daq );
    DMM16AT_dealloc(      &io->dmm );
    Mesanet_4I36_dealloc( &io->mesa1 );
    Mesanet_4I36_dealloc( &io->mesa2 );

    free ( io );
  }  
}
/****************************************************************/
// A macro to test whether the driver object state indicates the
// hardware is in an initialized state.
#define IO_READY(driver_object) (((driver_object) != NULL) && \
				 ((driver_object)->initialized) && \
				 !((driver_object)->closed))

// Test if all devices initialized properly.
int 
FlameIO_is_ready( FlameIO *io )
{
  return ( IO_READY( io )
	   && DMM16AT_is_ready( &io->dmm ) 
	   && AthenaDAQ_is_ready( &io->daq)
	   && Mesanet_4I36_is_ready( &io->mesa1 )
	   && Mesanet_4I36_is_ready( &io->mesa1 )
	   );
}
	   

/****************************************************************/
// LED control operations.

// There are six front panel LEDs.
void 
FlameIO_set_front_panel_LEDS( FlameIO *io, unsigned char leds )
{
  if ( IO_READY(io) ) {

    // The front panel leds are spread over PORT A and PORT B
    
    AthenaDAQ_write_digital_output_bits( &io->daq, ATHENADAQ_PORTA, 0xf8, leds << 3 );
    AthenaDAQ_write_digital_output_bits( &io->daq, ATHENADAQ_PORTB, 0x01, leds >> 5 );
  }
}


// There are three body LEDs.
void 
FlameIO_set_body_LEDS( FlameIO *io, unsigned char leds )
{
  if ( IO_READY(io) ) {
    AthenaDAQ_write_digital_output_bits( &io->daq, ATHENADAQ_PORTB, 0x0e, leds << 1 );
  }
}


// There are four motor driver board LEDS.
void 
FlameIO_set_motor_driver_LEDS( FlameIO *io, unsigned char leds )
{
  if ( IO_READY(io) ) {
    AthenaDAQ_write_digital_output_bits( &io->daq, ATHENADAQ_PORTB, 0xf0, leds << 4 );
  }
}

/****************************************************************/
// Power Control.

// For now just provide a way to flips the control bits, although
// what is actually required is to do something smart, like
// monitor motor voltages, but that requires integration into a
// continuing control process.

void 
FlameIO_write_power_control_outputs( FlameIO *io, unsigned char flags )
{
  if ( IO_READY(io) ) {
    AthenaDAQ_write_digital_output_bits( &io->daq, ATHENADAQ_PORTA, 0x07, flags );
  }
}

/****************************************************************/
// Motor Driver Control.  This writes the flags directly to the
// output byte, which is fine since the motor flags are defined
// in the header file to match the actual robot driver
// assignment.

void 
FlameIO_enable_motor_drivers( FlameIO *io, unsigned char flags )
{
  if ( IO_READY(io) ) {
    DMM16AT_write_digital_output_byte( &io->dmm, flags );
  }
}

/****************************************************************/
// Initialize general I/O state structure values.
void
FlameIO_initialize_default_state( FlameIO_state_t *s )
{
  s->powered = 0;

  // zero out all the torque values
  s->hipx.tau     = 0.0; 
  s->l().hipy.tau   = 0.0; 
  s->l().knee.tau   = 0.0; 
  s->l().ankley.tau = 0.0; 
  s->r().hipy.tau   = 0.0; 
  s->r().knee.tau   = 0.0; 
  s->r().ankley.tau = 0.0; 

  s->l().foot.front.input    = 5.0;
  s->l().foot.back.input     = 5.0;
  s->r().foot.front.input    = 5.0;
  s->r().foot.back.input     = 5.0;

  s->l().foot.front.count    = 0.0;
  s->l().foot.back.count     = 0.0;
  s->r().foot.front.count    = 0.0;
  s->r().foot.back.count     = 0.0;

  s->l().foot.front.state    = 0.0;
  s->l().foot.back.state     = 0.0;
  s->r().foot.front.state    = 0.0;
  s->r().foot.back.state     = 0.0;

  // strictly speaking, these four should be in FlameIO_params_t
  s->l().foot.front.threshold    = 2.5;
  s->l().foot.back.threshold     = 2.5;
  s->r().foot.front.threshold    = 2.5;
  s->r().foot.back.threshold     = 2.5;

}

// Initialize the I/O state structure offsets and scaling factors.
void
FlameIO_initialize_default_parameters( FlameIO_params_t *p )
{
  // default offset for each joint sensor, in radians
  p->hipx.q.offset        = 0.0 ;    // no index pulse

  p->l().hipy.q.offset      = -0.055;
  p->l().knee.q.offset      =  0.137;
  p->l().ankley.q.offset    = -0.008;//-0.196;//-0.208;//-0.370;
  p->l().anklex.q.offset    = -0.004;//0.053;

  p->l().hipymot.q.offset   = 0.0;    // multiple rotations
  p->l().kneemot.q.offset   = 0.0;
  p->l().ankleymot.q.offset = 0.0;

  p->r().hipy.q.offset      = -0.093;
  p->r().knee.q.offset      =  0.232;
  p->r().ankley.q.offset    = -0.025;//-0.005;//-0.156;//-0.205;//-0.278;
  p->r().anklex.q.offset    = -0.001;//-0.133;

  p->r().hipymot.q.offset   = 0.0;    // multiple rotations
  p->r().kneemot.q.offset   = 0.0;
  p->r().ankleymot.q.offset = 0.0;

  p->battery.com_un.offset = 0.0;     // offset in volts
  p->battery.com_sw.offset = 0.0;
  p->battery.mot_un.offset = 0.0;
  p->battery.mot_sw.offset = 0.0;

  p->l().foot.front.offset   = 0.0;
  p->l().foot.back.offset    = 0.0;
  p->r().foot.front.offset   = 0.0;
  p->r().foot.back.offset    = 0.0;

  p->hipx.imon.offset      = 0.0;     // offset in amps
  p->l().hipy.imon.offset    = 0.0;
  p->l().knee.imon.offset    = 0.0;
  p->l().ankley.imon.offset  = 0.0;
  p->r().hipy.imon.offset    = 0.0;
  p->r().knee.imon.offset    = 0.0;
  p->r().ankley.imon.offset  = 0.0;


  p->hipx.q.scale        = ( TWOPI ) / ( MOTOR_CPR * HIPX_JOINT_RATIO );
  p->l().hipy.q.scale      = ( TWOPI ) / ( JOINT_CPR );               // (radians/rev) * (rev/count) = radians/count
  p->l().knee.q.scale      = ( TWOPI ) / ( JOINT_CPR );
  p->l().ankley.q.scale    = ( TWOPI ) / ( JOINT_CPR );
  p->l().anklex.q.scale    = -( TWOPI ) / ( JOINT_CPR );
  p->l().hipymot.q.scale   = -( TWOPI ) / ( MOTOR_CPR * HIPY_MOT_RATIO * HIPY_JOINT_RATIO);
  p->l().kneemot.q.scale   = ( TWOPI ) / ( MOTOR_CPR * KNEE_MOT_RATIO * KNEE_JOINT_RATIO);
  p->l().ankleymot.q.scale = ( TWOPI ) / ( MOTOR_CPR * ANKLEY_MOT_RATIO * ANKLEY_JOINT_RATIO);

  p->r().hipy.q.scale      = -( TWOPI ) / ( JOINT_CPR );               // (radians/rev) * (rev/count) = radians/count
  p->r().knee.q.scale      = -( TWOPI ) / ( JOINT_CPR );
  p->r().ankley.q.scale    = -( TWOPI ) / ( JOINT_CPR );
  p->r().anklex.q.scale    = -( TWOPI ) / ( JOINT_CPR );
  p->r().hipymot.q.scale   =  ( TWOPI ) / ( MOTOR_CPR * HIPY_MOT_RATIO * HIPY_JOINT_RATIO);
  p->r().kneemot.q.scale   = -( TWOPI ) / ( MOTOR_CPR * KNEE_MOT_RATIO * KNEE_JOINT_RATIO);
  p->r().ankleymot.q.scale = -( TWOPI ) / ( MOTOR_CPR * ANKLEY_MOT_RATIO * ANKLEY_JOINT_RATIO);

  // Define the resistive dividers in the battery voltage circuits
#define BATT_COM_UN_VOLTAGE_DIVIDER ( 100.0 / ( 100.0 + 100.0 ))
#define BATT_COM_SW_VOLTAGE_DIVIDER ( 100.0 / ( 100.0 + 100.0 ))

  // The original divider resistors for the 30V motor voltages
  // were too large; there were large offset errors in the
  // measurement.  I guess I was optimistic about the input
  // impedance of the A/D converter multipler on the Athena
  // board. - GJZ

  // #define BATT_MOT_UN_VOLTAGE_DIVIDER ( 100.0 / ( 100.0 + 267.0 ))
  // #define BATT_MOT_SW_VOLTAGE_DIVIDER ( 100.0 / ( 100.0 + 267.0 ))
#define BATT_MOT_UN_VOLTAGE_DIVIDER ( 10.0 / ( 10.0 + 47.0 ))
#define BATT_MOT_SW_VOLTAGE_DIVIDER ( 10.0 / ( 10.0 + 47.0 ))

  // All the A/D converters are configured for bipolar +/- 10V inputs.
#define FULL_SCALE 10.0

  p->battery.com_un.scale = ( FULL_SCALE ) / ( ATHENADAQ_MAX_ANALOG_INPUT * BATT_COM_UN_VOLTAGE_DIVIDER );
  p->battery.com_sw.scale = ( FULL_SCALE ) / ( ATHENADAQ_MAX_ANALOG_INPUT * BATT_COM_SW_VOLTAGE_DIVIDER );
  p->battery.mot_un.scale = ( FULL_SCALE ) / ( ATHENADAQ_MAX_ANALOG_INPUT * BATT_MOT_UN_VOLTAGE_DIVIDER );
  p->battery.mot_sw.scale = ( FULL_SCALE ) / ( ATHENADAQ_MAX_ANALOG_INPUT * BATT_MOT_SW_VOLTAGE_DIVIDER );

  // The foot sensors are just reported in volts.
  p->l().foot.front.scale   = ( FULL_SCALE ) / ( ATHENADAQ_MAX_ANALOG_INPUT );
  p->l().foot.back.scale    = ( FULL_SCALE ) / ( ATHENADAQ_MAX_ANALOG_INPUT );
  p->r().foot.front.scale   = ( FULL_SCALE ) / ( ATHENADAQ_MAX_ANALOG_INPUT );
  p->r().foot.back.scale    = ( FULL_SCALE ) / ( ATHENADAQ_MAX_ANALOG_INPUT );
  // scale factor for the Maxon RE35 90W motor with 30V winding
  // currently current is translated to (expected) torque
  p->hipx.imon.scale      = -( FULL_SCALE * Z12A8_IMON_SCALE ) / ( DMM16AT_MAX_ANALOG_INPUT ) * MAXON_RE35_TORQUE_CONSTANT * HIPX_JOINT_RATIO;
  p->l().hipy.imon.scale    = ( FULL_SCALE * Z12A8_IMON_SCALE ) / ( DMM16AT_MAX_ANALOG_INPUT ) * MAXON_RE35_TORQUE_CONSTANT * HIPY_MOT_RATIO   * HIPY_JOINT_RATIO;
  p->l().knee.imon.scale    = -( FULL_SCALE * Z12A8_IMON_SCALE ) / ( DMM16AT_MAX_ANALOG_INPUT ) * MAXON_RE35_TORQUE_CONSTANT * KNEE_MOT_RATIO   * KNEE_JOINT_RATIO;
  p->l().ankley.imon.scale  = -( FULL_SCALE * Z12A8_IMON_SCALE ) / ( DMM16AT_MAX_ANALOG_INPUT ) * MAXON_RE35_TORQUE_CONSTANT * ANKLEY_MOT_RATIO * ANKLEY_JOINT_RATIO;
  p->r().hipy.imon.scale    = -( FULL_SCALE * Z12A8_IMON_SCALE ) / ( DMM16AT_MAX_ANALOG_INPUT ) * MAXON_RE35_TORQUE_CONSTANT * HIPY_MOT_RATIO   * HIPY_JOINT_RATIO;
  p->r().knee.imon.scale    = ( FULL_SCALE * Z12A8_IMON_SCALE ) / ( DMM16AT_MAX_ANALOG_INPUT ) * MAXON_RE35_TORQUE_CONSTANT * KNEE_MOT_RATIO   * KNEE_JOINT_RATIO;
  p->r().ankley.imon.scale  = ( FULL_SCALE * Z12A8_IMON_SCALE ) / ( DMM16AT_MAX_ANALOG_INPUT ) * MAXON_RE35_TORQUE_CONSTANT * ANKLEY_MOT_RATIO * ANKLEY_JOINT_RATIO;

  // this assumes the drivers put out the maximum transient current at the maximum rated input command
  // this just sets the taumax to the full hardware limit
  p->hipx.taumax     = Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * HIPX_JOINT_RATIO;
  p->l().hipy.taumax   = Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * HIPY_MOT_RATIO   * HIPY_JOINT_RATIO;
  p->l().knee.taumax   = Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * KNEE_MOT_RATIO   * KNEE_JOINT_RATIO;
  p->l().ankley.taumax = Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * ANKLEY_MOT_RATIO * ANKLEY_JOINT_RATIO;
  p->r().hipy.taumax   = Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * HIPY_MOT_RATIO   * HIPY_JOINT_RATIO;
  p->r().knee.taumax   = Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * KNEE_MOT_RATIO   * KNEE_JOINT_RATIO;
  p->r().ankley.taumax = Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * ANKLEY_MOT_RATIO * ANKLEY_JOINT_RATIO;

  // this is scaling factor from Newton-meter to a normalized unit-range [-1, 1] torque command
  p->hipx.tau.scale     =  1.0 / ( Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * HIPX_JOINT_RATIO);
  p->l().hipy.tau.scale   = -1.0 / ( Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * HIPY_MOT_RATIO   * HIPY_JOINT_RATIO);  
  p->l().knee.tau.scale   =  1.0 / ( Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * KNEE_MOT_RATIO   * KNEE_JOINT_RATIO);  
  p->l().ankley.tau.scale =  1.0 / ( Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * ANKLEY_MOT_RATIO * ANKLEY_JOINT_RATIO);
  p->r().hipy.tau.scale   =  1.0 / ( Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * HIPY_MOT_RATIO   * HIPY_JOINT_RATIO);  
  p->r().knee.tau.scale   = -1.0 / ( Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * KNEE_MOT_RATIO   * KNEE_JOINT_RATIO);  
  p->r().ankley.tau.scale = -1.0 / ( Z12A8_MAX_CURRENT * MAXON_RE35_TORQUE_CONSTANT * ANKLEY_MOT_RATIO * ANKLEY_JOINT_RATIO);

  // default offsets
  p->hipx.tau.offset     = 0.0;
  p->l().hipy.tau.offset   = 0.0;
  p->l().knee.tau.offset   = 0.0;
  p->l().ankley.tau.offset = 0.0;
  p->r().hipy.tau.offset   = 0.0;
  p->r().knee.tau.offset   = 0.0;
  p->r().ankley.tau.offset = 0.0;


}
/****************************************************************/
// Apply the foot switch debounce logic.  The switches are
// currently normally-open switches, so the input is pulled up to
// the maximum when there is no ground contact.
static inline void
update_foot_switch_logic( flame_foot_switch_t *sw )
{
  char onfloor = sw->input < sw->threshold;

  if (onfloor && (sw->count > 0 ))
    sw->count++;
  else if (!onfloor && (sw->count <= 0))
    sw->count--;
  else if (!onfloor && (sw->count >= 1))
    sw->count = 0;
  else if (onfloor  && (sw->count <= 0 ))
    sw->count = 1;

  // create switch relay
  sw->state = sw->state | ( sw->count > 3 );
  sw->state = sw->state & !( sw->count < -3 );
}

/*{
 * if ( sw->input < sw->threshold ) sw->count++;
 * else sw->count = 0;
 * sw->state = ( sw->count > 5 ); 
 *}
 */

/****************************************************************/
void
FlameIO_read_all_sensors( FlameIO *io, FlameIO_params_t *p, FlameIO_state_t *s )
{
  int counts[ ENCODER_CHANNELS ];

  if (!IO_READY(io)) return;
  
  // Immediately trigger both A/D circuits.
  AthenaDAQ_start_analog_input_scan( &io->daq );
  DMM16AT_start_analog_input_scan( &io->dmm );

  // Read the user switches.
  s->front_panel_sw = AthenaDAQ_read_digital_input_byte( &io->daq, ATHENADAQ_PORTC );

  // Read the motor driver fault outputs.
  s->motor_faults   = DMM16AT_read_digital_input_byte( &io->dmm );

  // Read the encoders.
  Mesanet_4I36_read_all_counters( &io->mesa1 );
  Mesanet_4I36_read_all_counters( &io->mesa2 );

  // Apply the joint encoder calibration.  First copy all the
  // values from each encoder counter board to a common array
  // just to simplify assigning the values.

  memcpy( &counts[0], io->mesa1.count, 8 * sizeof( int ));
  memcpy( &counts[8], io->mesa2.count, 8 * sizeof( int ));

  // Now apply scale and offset.
  s->hipx.q             = ( counts[ HIPX_ENCODER       ] * p->hipx.q.scale        ) + p->hipx.q.offset;
  s->l().hipy.q           = ( counts[ LHIPY_ENCODER      ] * p->l().hipy.q.scale      ) + p->l().hipy.q.offset;
  s->l().knee.q           = ( counts[ LKNEE_ENCODER      ] * p->l().knee.q.scale      ) + p->l().knee.q.offset;
  s->l().ankley.q         = ( counts[ LANKLEY_ENCODER    ] * p->l().ankley.q.scale    ) + p->l().ankley.q.offset;
  s->l().anklex.q         = ( counts[ LANKLEX_ENCODER    ] * p->l().anklex.q.scale    ) + p->l().anklex.q.offset;
  s->l().hipymot.q        = ( counts[ LHIPYMOT_ENCODER   ] * p->l().hipymot.q.scale   ) + p->l().hipymot.q.offset;
  s->l().kneemot.q        = ( counts[ LKNEEMOT_ENCODER   ] * p->l().kneemot.q.scale   ) + p->l().kneemot.q.offset;
  s->l().ankleymot.q      = ( counts[ LANKLEYMOT_ENCODER ] * p->l().ankleymot.q.scale ) + p->l().ankleymot.q.offset;
  // unused             = ( counts[ UNUSED_ENCODER     ] );
  s->r().hipy.q           = ( counts[ RHIPY_ENCODER      ] * p->r().hipy.q.scale      ) + p->r().hipy.q.offset;
  s->r().knee.q           = ( counts[ RKNEE_ENCODER      ] * p->r().knee.q.scale      ) + p->r().knee.q.offset;
  s->r().ankley.q         = ( counts[ RANKLEY_ENCODER    ] * p->r().ankley.q.scale    ) + p->r().ankley.q.offset;
  s->r().anklex.q         = ( counts[ RANKLEX_ENCODER    ] * p->r().anklex.q.scale    ) + p->r().anklex.q.offset;
  s->r().hipymot.q        = ( counts[ RHIPYMOT_ENCODER   ] * p->r().hipymot.q.scale   ) + p->r().hipymot.q.offset;
  s->r().kneemot.q        = ( counts[ RKNEEMOT_ENCODER   ] * p->r().kneemot.q.scale   ) + p->r().kneemot.q.offset;
  s->r().ankleymot.q      = ( counts[ RANKLEYMOT_ENCODER ] * p->r().ankleymot.q.scale ) + p->r().ankleymot.q.offset;


  // These might busywait if the conversions are not yet complete.
  AthenaDAQ_finish_analog_input_scan( &io->daq );

  // Apply the analog input calibration for the first bank of A/D channels.

  // battery voltages
  s->battery.com_un        = ( io->daq.analog_inputs[ 0 ] * p->battery.com_un.scale ) + p->battery.com_un.offset;
  s->battery.com_sw        = ( io->daq.analog_inputs[ 1 ] * p->battery.com_sw.scale ) + p->battery.com_sw.offset;
  s->battery.mot_un        = ( io->daq.analog_inputs[ 2 ] * p->battery.mot_un.scale ) + p->battery.mot_un.offset;
  s->battery.mot_sw        = ( io->daq.analog_inputs[ 3 ] * p->battery.mot_sw.scale ) + p->battery.mot_sw.offset;

  // foot sensors
  s->l().foot.front.input    = ( io->daq.analog_inputs[ 14 ] * p->l().foot.front.scale ) + p->l().foot.front.offset;
  s->l().foot.back.input     = ( io->daq.analog_inputs[ 13 ] * p->l().foot.back.scale  ) + p->l().foot.back.offset;
  s->r().foot.front.input    = ( io->daq.analog_inputs[ 11 ] * p->r().foot.front.scale ) + p->r().foot.front.offset;
  s->r().foot.back.input     = ( io->daq.analog_inputs[ 10 ] * p->r().foot.back.scale  ) + p->r().foot.back.offset;


  // Apply the foot switch debounce logic; this uses a simple
  // inline function which appears above.
  update_foot_switch_logic( &s->l().foot.back );
  update_foot_switch_logic( &s->l().foot.front );
  update_foot_switch_logic( &s->r().foot.back );
  update_foot_switch_logic( &s->r().foot.front );

  // This could busywait if the conversions are not yet complete.
  DMM16AT_finish_analog_input_scan( &io->dmm );

  // Apply the analog input calibration for the second bank of A/D channels.

  // motor currents
  s->hipx.imon             = ( io->dmm.analog_inputs[ HIPX_DRIVER  ] * p->hipx.imon.scale        ) + p->hipx.imon.offset;
  s->l().hipy.imon           = ( io->dmm.analog_inputs[ LHIPY_DRIVER ] * p->l().hipy.imon.scale      ) + p->l().hipy.imon.offset;
  s->l().knee.imon           = ( io->dmm.analog_inputs[ LKNEE_DRIVER ] * p->l().knee.imon.scale      ) + p->l().knee.imon.offset;
  s->l().ankley.imon         = ( io->dmm.analog_inputs[ LANKY_DRIVER ] * p->l().ankley.imon.scale    ) + p->l().ankley.imon.offset;
  s->r().hipy.imon           = ( io->dmm.analog_inputs[ RHIPY_DRIVER ] * p->r().hipy.imon.scale      ) + p->r().hipy.imon.offset;
  s->r().knee.imon           = ( io->dmm.analog_inputs[ RKNEE_DRIVER ] * p->r().knee.imon.scale      ) + p->r().knee.imon.offset;
  s->r().ankley.imon         = ( io->dmm.analog_inputs[ RANKY_DRIVER ] * p->r().ankley.imon.scale    ) + p->r().ankley.imon.offset;

}
/****************************************************************/
// Update the torque command outputs.
void FlameIO_write_torque_commands( FlameIO *io, FlameIO_params_t *p, FlameIO_state_t *s )
{
  int i;
  float unitvalue[8];
  short dacvalue[8];

  if (!IO_READY(io)) return;

  // Apply a soft torque limit, which may be lower than the hardware command maximums.
#define APPLY_TAU_LIMIT(axis)  if ( s->axis.tau < -p->axis.taumax ) s->axis.tau = -p->axis.taumax; else if ( s->axis.tau > p->axis.taumax ) s->axis.tau = p->axis.taumax

  APPLY_TAU_LIMIT( hipx );
  APPLY_TAU_LIMIT( l().hipy );
  APPLY_TAU_LIMIT( l().knee );
  APPLY_TAU_LIMIT( l().ankley );
  APPLY_TAU_LIMIT( r().hipy );
  APPLY_TAU_LIMIT( r().knee );
  APPLY_TAU_LIMIT( r().ankley );

  // Offset, and scale the torque commands.

  unitvalue[ HIPX_DRIVER  ]  =  ( s->hipx.tau     - p->hipx.tau.offset     )  *  p->hipx.tau.scale     ;
  unitvalue[ LHIPY_DRIVER ]  =  ( s->l().hipy.tau   - p->l().hipy.tau.offset   )  *	 p->l().hipy.tau.scale   ;
  unitvalue[ LKNEE_DRIVER ]  =  ( s->l().knee.tau   - p->l().knee.tau.offset   )  *	 p->l().knee.tau.scale   ;
  unitvalue[ LANKY_DRIVER ]  =  ( s->l().ankley.tau - p->l().ankley.tau.offset )  *	 p->l().ankley.tau.scale ;
  unitvalue[ RHIPY_DRIVER ]  =  ( s->r().hipy.tau   - p->r().hipy.tau.offset   )  *	 p->r().hipy.tau.scale   ;
  unitvalue[ RKNEE_DRIVER ]  =  ( s->r().knee.tau   - p->r().knee.tau.offset   )  *	 p->r().knee.tau.scale   ;
  unitvalue[ RANKY_DRIVER ]  =  ( s->r().ankley.tau - p->r().ankley.tau.offset )  *	 p->r().ankley.tau.scale ;
  unitvalue[ UNUSED_DRIVER ] = 0.0;

  // Clamp all unit values
  for (i = 0; i < 8; i++ ) {
    if ( unitvalue[i] > 1.0) unitvalue[i] = 1.0;
    else if ( unitvalue[i] < -1.0) unitvalue[i] = -1.0;
  }

  // Convert from unit [-1, 1] torque commands to integer values for the hardware.
  for (i = 0; i < 4; i++ ) dacvalue[i] = (short) (ATHENADAQ_MAX_ANALOG_OUTPUT * unitvalue[i]);
  for (i = 4; i < 8; i++ ) dacvalue[i] = (short) (DMM16AT_MAX_ANALOG_OUTPUT * unitvalue[i]);

  // Zero any analog output values for disabled channels.
  if ( !(s->powered & FLAME_HIPX_MOTOR    ) )  dacvalue[ HIPX_DRIVER  ] = 0;
  if ( !(s->powered & FLAME_LHIPY_MOTOR   ) )  dacvalue[ LHIPY_DRIVER ] = 0;
  if ( !(s->powered & FLAME_LKNEE_MOTOR   ) )  dacvalue[ LKNEE_DRIVER ] = 0; 
  if ( !(s->powered & FLAME_LANKLEY_MOTOR ) )  dacvalue[ LANKY_DRIVER ] = 0; 
  if ( !(s->powered & FLAME_RHIPY_MOTOR   ) )  dacvalue[ RHIPY_DRIVER ] = 0; 
  if ( !(s->powered & FLAME_RKNEE_MOTOR   ) )  dacvalue[ RKNEE_DRIVER ] = 0; 
  if ( !(s->powered & FLAME_RANKLEY_MOTOR ) )  dacvalue[ RANKY_DRIVER ] = 0; 

  // Write out integer values to the hardware.
  AthenaDAQ_write_analog_output( &io->daq, 0, dacvalue[0]);
  DMM16AT_write_all_analog_outputs( &io->dmm, dacvalue[4], dacvalue[5], dacvalue[6], dacvalue[7] );

  // Each of these calls takes 20-30 microseconds since they must
  // busywait on the completion of the previous update.
  AthenaDAQ_write_analog_output( &io->daq, 1, dacvalue[1]);
  AthenaDAQ_write_analog_output( &io->daq, 2, dacvalue[2]);
  AthenaDAQ_write_analog_output( &io->daq, 3, dacvalue[3]);

}
