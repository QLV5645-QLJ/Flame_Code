// $Id: AthenaDAQ.h,v 1.4 2005/12/07 15:25:38 garthz Exp $
// AthenaDAQ.h : driver for the Diamond Systems Athena Digital Analog Data Acquisition hardware
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
/****************************************************************/

// Define a device structure to hold configuration and state
// information.  This is not written as an opaque structure, but
// is subject to change, so it is preferable to use access
// methods rather than modify the fields directly.

typedef struct {

  // The DAQ hardware can be configured for different port addresses
  // by the BIOS.  The default is 0x280.
  short base_address;

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

} AthenaDAQ;

/****************************************************************/
// The object creation and destruction methods loosely follow the Objective-C style.

// Create an empty AthenaDAQ object.
extern AthenaDAQ *AthenaDAQ_alloc(void);                          

// Initialize a AthenaDAQ device.  The structure may be statically allocated.
extern AthenaDAQ *AthenaDAQ_init( AthenaDAQ *daq );
extern AthenaDAQ *AthenaDAQ_init_with_address( AthenaDAQ *daq, short base_address ); 

// Shut down a AthenaDAQ device.   The structure may be statically allocated.
extern void AthenaDAQ_close( AthenaDAQ *daq ); 

// De-allocate an allocated AthenaDAQ object, stopping the device if still active.
extern void AthenaDAQ_dealloc( AthenaDAQ *daq );

// Test if the device initialized properly.
extern int AthenaDAQ_is_ready( AthenaDAQ *daq );

/****************************************************************/
// I/O operations

// Output a byte of data to the digital output port.  The port indicees are as follows:
#define ATHENADAQ_PORTA   0
#define ATHENADAQ_PORTB   1
#define ATHENADAQ_PORTC   2
extern void AthenaDAQ_write_digital_output_byte( AthenaDAQ *daq, unsigned port, unsigned char byte );

// Write individual bits in a port.  The mask bits determine
// which port bits are updated from the corresponding bits in
// 'bits'.
extern void AthenaDAQ_write_digital_output_bits( AthenaDAQ *daq, unsigned port, unsigned char mask, unsigned char bits );

// Read a byte of data from the digital input port
extern unsigned char AthenaDAQ_read_digital_input_byte( AthenaDAQ *daq, unsigned port );

// Configure the digital I/O directions.  The flag bits are as follows:
#define ATHENADAQ_PORTA_INPUT   0x10
#define ATHENADAQ_PORTB_INPUT   0x02
#define ATHENADAQ_PORTCL_INPUT  0x01
#define ATHENADAQ_PORTCH_INPUT  0x08
extern void AthenaDAQ_configure_digital_direction( AthenaDAQ *daq, unsigned flags );


// Write a new value to a D/A converter channel.  It takes 20
// microseconds to update a single channel due to the DAC serial
// interface.  For this reason, just a single channel update function
// is provided on the expecation that the user will intersperse
// individual channel updates with other I/O operations to avoid
// excessive busy-waiting.
extern void AthenaDAQ_write_analog_output( AthenaDAQ *daq, unsigned channel, short value );
#define ATHENADAQ_MAX_ANALOG_OUTPUT  2047
#define ATHENADAQ_MIN_ANALOG_OUTPUT -2048

// perform A/D conversions on all A/D channels
extern void AthenaDAQ_read_all_analog_inputs( AthenaDAQ *daq );
#define ATHENADAQ_MAX_ANALOG_INPUT  32767         // 16 bit converter
#define ATHENADAQ_MIN_ANALOG_INPUT  -32768

// separately begin and end a scan, to allow other processing while polling the conversion progress
extern void AthenaDAQ_start_analog_input_scan( AthenaDAQ *daq );
extern int AthenaDAQ_is_analog_scan_done( AthenaDAQ *daq );
extern void AthenaDAQ_finish_analog_input_scan( AthenaDAQ *daq );

