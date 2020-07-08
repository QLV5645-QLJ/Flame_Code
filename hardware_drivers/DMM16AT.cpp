// $Id: DMM16AT.c,v 1.4 2005/12/07 15:25:38 garthz Exp $
// DMM16AT.c : driver for the Diamond Systems DMM-16-AT PC/104 analog/digital I/O card.
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

//==========================================================
#include "DMM16AT.h"

// I/O Map of the DMM-16-AT.  All addresses are port addresses relative to base I/O.

// Write functions.
#define START_AD    0
#define DA_LSB      1
#define AD_CHANNEL  2
#define DIGOUT      3
#define DA0_MSB     4
#define DA1_MSB     5
#define DA2_MSB     6
#define DA3_MSB     7
#define CLEARINT    8
#define INTCONTROL  9
#define FIFOCTL     10
#define ANALOGCFG   11

// page 0 for addresses 12-15
#define CTR0DATA    12
#define CTR1DATA    13
#define CTR2DATA    14
#define CTRCONTROL  15

// page 1 for addresses 12-15
#define EEPROMDATA  12
#define EEPROMADDR  13
#define CALIBCTL    14
#define EEPROMKEY   15
// End of write function addresses.

// Read functions, wherever different from the write function.
#define AD_LSB      0
#define AD_MSB      1
#define DIGIN       3
#define DA0_UPDATE  4
#define DA1_UPDATE  5
#define DA2_UPDATE  6
#define DA3_UPDATE  7
#define STATUS      8
#define FIFOSTATUS  10

// page 1 for addresses 12-15
#define CALIBSTATUS 14
#define FPGAVERSION 15

// Define the acceptable FPGA version
#define FPGAVERSIONCODE 0x40

// Loop counter for timeouts on busywaits.
#define LOOPTIMEOUT     100000

// Define masks for Calibration Status Register
#define CALIBSTATUS_TDBUSY 0x40         // indicates TrimDAC transfer is in progress

/****************************************************************/
// A macro to test whether it is safe to access the hardware.

#define DMM_READY(driver_object) (((driver_object) != NULL) && \
				  ((driver_object)->initialized) && \
				  !((driver_object)->closed))

/****************************************************************/

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

// Define the I/O port assembler macros.  This depends on holding
// iopl(3) permissions.  Note that the order of arguments to the
// Linux outb macro is outb (data, address).
#include <asm/io.h>


// Define logprintf, errprintf.
#include <utility/utility.h>

/****************************************************************/

// Create an empty DMM16AT object.
DMM16AT *DMM16AT_alloc(void)
{
  DMM16AT *dmm = (DMM16AT *) calloc (1, sizeof( DMM16AT ) );
}

// Initialize a DMM16AT device.  The structure may be statically allocated.
DMM16AT *DMM16AT_init( DMM16AT *dmm )
{
  return DMM16AT_init_with_address( dmm, 0x300 ); // default base address
}

DMM16AT *DMM16AT_init_with_address( DMM16AT *dmm, short base_address )
{
  unsigned char fpga_rev;

  if ( dmm != NULL ) { 
    dmm->base_address = base_address;
    dmm->initialized = 1;
    dmm->closed = 0;
    dmm->digital_output_state = 0;

    dmm->channel_range = 0xf0;   // default is all channels in single-ended mode
    dmm->scan_in_progress = 0;

    dmm->ad_mode       = 0x18;   // default is +/- 10V input range, 5.3uS sampling interval   
    dmm->min_analog_input = -10.0; 
    dmm->max_analog_input =  10.0; 

    // Check FPGA rev. Select page 1 to read the upper registers.
    outb( 0x40, base_address + FIFOCTL);

    // read the FPGA version number
    fpga_rev = inb( base_address + FPGAVERSION );

    logprintf("DMM16AT driver: found FPGA revision code 0x%02x.\n", fpga_rev );

    if (fpga_rev != FPGAVERSIONCODE) {
      errprintf("DMM16AT_init: error, invalid FPGA revision 0x%02x.\n", fpga_rev );
      dmm->initialized = 0;
    } else {
      // perform hardware initialization.

      // Set up the D/A converter TrimDACs for +/- 10V output.
      // This assumes the board has been configured for
      // programmable output range, i.e., that on J5 the jumper is
      // installed in the P position but not the 5 position.

    }
  }
  return dmm;
}

/****************************************************************/
// Shut down a DMM16AT device.  The driver structure may be statically allocated.
void DMM16AT_close( DMM16AT *dmm )
{
  if ( dmm != NULL ) { 
    dmm->closed = 1;

    // perform hardware shutdown
  }  
}

/****************************************************************/
// De-allocate an allocated DMM16AT object, stopping the device if still active.
void DMM16AT_dealloc( DMM16AT *dmm )
{
  if ( dmm != NULL ) { 
    if ( dmm->initialized && !dmm->closed ) DMM16AT_close( dmm );

    free ( dmm );
  }  
}

// Test if the device initialized properly.
int
DMM16AT_is_ready( DMM16AT *dmm )
{
  return DMM_READY( dmm );
}

/****************************************************************/
void DMM16AT_write_digital_output_byte( DMM16AT *dmm, unsigned char byte )
{
  if ( DMM_READY(dmm) ) {
    dmm->digital_output_state = byte;          // save DIO output byte value for future output bit manipulation
    outb( byte, dmm->base_address + DIGOUT );  // write DIO output byte
  }
}

unsigned char DMM16AT_read_digital_input_byte( DMM16AT *dmm )
{
  if ( DMM_READY(dmm) ) {
    return inb( dmm->base_address + DIGIN );  // read DIO input byte
  } else return 0;
}

/****************************************************************/
void DMM16AT_write_analog_output( DMM16AT *dmm, unsigned channel, short value )
{
  if ( DMM_READY(dmm) && channel < 3) {

    // Convert from a signed integer to a unsigned 12 bit integer.
    value += 2048;

    // write out bottom 8 bits
    outb( value & 0xff, dmm->base_address + DA_LSB );  

    // write out the top 8 bits to a specific channel register
    outb( (value >> 8) & 0x0f, dmm->base_address + channel + DA0_MSB ); 

    // read any channel's MSB to update all D/A converter outputs
    inb( dmm->base_address + DA0_UPDATE );
  }
}

// Efficient update of all channels
void DMM16AT_write_all_analog_outputs( DMM16AT *dmm, short value0, short value1, short value2, short value3 )
{
  if ( DMM_READY(dmm) ) {

    // Convert from signed integers to unsigned 12 bit integers.
    value0 += 2048;
    value1 += 2048;
    value2 += 2048;
    value3 += 2048;

    // write out bottom 8 bits
    outb( value0 & 0xff, dmm->base_address + DA_LSB );  

    // write out the top 8 bits to a specific channel register
    outb( (value0 >> 8) & 0x0f, dmm->base_address + DA0_MSB ); 

    // same for remainder of channels
    outb( value1 & 0xff, dmm->base_address + DA_LSB );  
    outb( (value1 >> 8) & 0x0f, dmm->base_address + DA1_MSB ); 

    outb( value2 & 0xff, dmm->base_address + DA_LSB );  
    outb( (value2 >> 8) & 0x0f, dmm->base_address + DA2_MSB ); 

    outb( value3 & 0xff, dmm->base_address + DA_LSB );  
    outb( (value3 >> 8) & 0x0f, dmm->base_address + DA3_MSB ); 

    // read any channel's MSB to update all D/A converter outputs
    inb( dmm->base_address + DA0_UPDATE );
  }
}

/****************************************************************/
void DMM16AT_start_analog_input_scan( DMM16AT *dmm )
{
  unsigned timeout;
  if ( DMM_READY(dmm) && !dmm->scan_in_progress ) {
    
    outb( 0x80, dmm->base_address + FIFOCTL );                   // reset the FIFO by writing the FIFORST bit
    outb( dmm->channel_range, dmm->base_address + AD_CHANNEL );  // configure the scan range
    outb( dmm->ad_mode, dmm->base_address + ANALOGCFG );         // configure the A/D gains
    
    // wait for WAIT bit to indicate analog input circuit has settled
    timeout = 0;
    while( inb( dmm->base_address + FIFOCTL) & 0x80) {
      if (++timeout > LOOPTIMEOUT) {
	errprintf("DMM16AT_start_analog_input_scan: analog gain settling wait timed out.\n");
	return;
      }
    }

    outb( 0x10, dmm->base_address + FIFOCTL );                   // set SCANEN to enable scan
    outb( 0x00, dmm->base_address + START_AD );                  // trigger A/D scan

    dmm->scan_in_progress = 1;
  }
}

int DMM16AT_is_analog_scan_done( DMM16AT *dmm )
{
  return ( DMM_READY(dmm) 
	   && dmm->scan_in_progress 
	   && !( inb( dmm->base_address + STATUS ) & 0x80) );
}

void DMM16AT_finish_analog_input_scan( DMM16AT *dmm )
{
  int channel, low, high;
  unsigned timeout;

  if ( DMM_READY(dmm) && dmm->scan_in_progress ) {

    // determine number of channels read
    low  = dmm->channel_range & 0x0f;
    high = ( dmm->channel_range >> 4) & 0x0f;
    
    // wait for A/D converter scan to finish (bit STS goes low)
    timeout = 0;
    while( inb( dmm->base_address + STATUS ) & 0x80) {
      if (++timeout > LOOPTIMEOUT) {
	errprintf("DMM16AT_finish_analog_input_scan: A/D scan complete wait timed out.\n");
	dmm->scan_in_progress = 0;    
	return;
      }
    }

    // read FIFO data
    for ( channel = low; channel <= high; channel++ ) {
      int lsb = inb( dmm->base_address + AD_LSB );
      int msb = inb( dmm->base_address + AD_MSB );
      dmm->analog_inputs[ channel ] = lsb | (msb<<8);
    }
    dmm->scan_in_progress = 0;    
  }
}

void DMM16AT_read_all_analog_inputs( DMM16AT *dmm )
{
  DMM16AT_start_analog_input_scan( dmm );
  DMM16AT_finish_analog_input_scan( dmm );
}

