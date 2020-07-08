// $Id: DMM16AT.h,v 1.5 2005/12/07 15:25:38 garthz Exp $
// DMM16AT.h : driver for the Diamond Systems DMM-16-AT PC/104 analog/digital I/O card.
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
/****************************************************************/
#ifndef DMM16AT_H_INCLUDED
#define DMM16AT_H_INCLUDED

// Define a device structure to hold configuration and state
// information.  This is not written as an opaque structure, but
// is subject to change, so it is preferable to use access
// methods rather than modify the fields directly.

typedef struct {

  // The board can be installed at a range of port addresses
  // between 0x220 to 0x3e0.  The default is 0x300.
  short base_address;

  // The state of the digital outputs, saved for efficient
  // modification of a single bit.
  unsigned char digital_output_state;

  // The A/D channel register value, which defines the range of A/D channels read.
  // This must have a different value for differential or single-ended mode.
  unsigned char channel_range;

  // The A/D converter mode, which reflects the input gains, polarity, conversion rate.
  unsigned char ad_mode;

  // The most recently read A/D values.
  short analog_inputs[16];

  // The minimum and maximum values represented by 0x8000 and 0x7fff; i.e, by a signed 
  // short, ranging from -32768 to 32767.
  float min_analog_input, max_analog_input;

  // driver status flags
  unsigned int initialized      :1;      // true if this data structure is valid
  unsigned int closed           :1;      // true if hardware has been closed
  unsigned int scan_in_progress :1;      // true if analog conversion is in progress

} DMM16AT;

/****************************************************************/
// The object creation and destruction methods loosely follow the Objective-C style.

// Create an empty DMM16AT object.
extern DMM16AT *DMM16AT_alloc(void);                          

// Initialize a DMM16AT device.  The structure may be statically allocated.
extern DMM16AT *DMM16AT_init( DMM16AT *dmm );
extern DMM16AT *DMM16AT_init_with_address( DMM16AT *dmm, short base_address ); 

// Shut down a DMM16AT device.   The structure may be statically allocated.
extern void DMM16AT_close( DMM16AT *dmm ); 

// De-allocate an allocated DMM16AT object, stopping the device if still active.
extern void DMM16AT_dealloc( DMM16AT *dmm );

// Test if the device initialized properly.
extern int DMM16AT_is_ready( DMM16AT *dmm );

/****************************************************************/
// I/O operations

// Output a byte of data to the digital output port
extern void DMM16AT_write_digital_output_byte( DMM16AT *dmm, unsigned char byte );

// Read a byte of data from the digital input port
extern unsigned char DMM16AT_read_digital_input_byte( DMM16AT *dmm );

// perform D/A conversions on one or all D/A channels
#define DMM16AT_MAX_ANALOG_OUTPUT  2047
#define DMM16AT_MIN_ANALOG_OUTPUT -2048
extern void DMM16AT_write_analog_output( DMM16AT *dmm, unsigned channel, short value );
extern void DMM16AT_write_all_analog_outputs( DMM16AT *dmm, short value0, short value1, short value2, short value3 );

// perform A/D conversions on all A/D channels
extern void DMM16AT_read_all_analog_inputs( DMM16AT *dmm );
#define DMM16AT_MAX_ANALOG_INPUT  32767         // 16 bit converter
#define DMM16AT_MIN_ANALOG_INPUT  -32768

// separately begin and end a scan, to allow other processing while polling the conversion progress
extern void DMM16AT_start_analog_input_scan( DMM16AT *dmm );
extern int DMM16AT_is_analog_scan_done( DMM16AT *dmm );
extern void DMM16AT_finish_analog_input_scan( DMM16AT *dmm );

#endif // DMM16AT_H_INCLUDED
