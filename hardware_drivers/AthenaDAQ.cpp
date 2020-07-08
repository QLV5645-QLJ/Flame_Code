// $Id: AthenaDAQ.c,v 1.5 2005/12/14 08:28:02 garthz Exp $
// AthenaDAQ.c : driver for the Diamond Systems Athena Digital Analog Data Acquisition hardware
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

//==========================================================
#include "AthenaDAQ.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

// Define the I/O port assembler macros.  This depends on holding
// iopl(3) permissions.  Note that the order of arguments to the
// Linux outb macro is outb (data, address).
#include <asm/io.h>

// Define logprintf, errprintf.
#include <utility/utility.h>

//==========================================================
// The J13 jumpers allow the following configuration choices:
//   single-ended  or  differential  A/D
//   unipolar      or  bipolar       A/D
//   unipolar      or  bipolar       D/A
//
// The default is single-ended bipolar A/D, and bipolar D/A, which is
// correct for the Flame robot:
//   J13-1 installed
//   J13-2 installed
//   J13-8 installed

//==========================================================
// I/O Map of the Athena system hardware
#define WATCHDOG_SERIAL_FPGA_CONTROL    0x25f
#define FPGAEN_MASK      0x20

//==========================================================
// I/O Map of the Athena DAQ hardware.  All addresses are port
// addresses relative to base I/O address.

// Write functions.
#define COMMAND     0  	// 0  command register
                       	// 1  not used
#define ADCHANNEL   2  	// 2  A/D channel register
#define ADGAINSCAN  3  	// 3  A/D gain and scan settings
#define INTDMACTR   4  	// 4  interrupt / DMA / counter control
#define FIFOTHRESH  5  	// 5  FIFO threshold
#define DALSB       6  	// 6  D/A LSB
#define DAMSBCHAN   7  	// 7  D/A MSB + channel no.
#define DIOPORTA    8  	// 8  Digital I/O Port A output
#define DIOPORTB    9  	// 9  Digital I/O Port B output
#define DIOPORTC    10  // 10 Digital I/O Port C output
#define DIODIR      11  // 11 Digital I/O direction control
#define CTRDAT0     12  // 12 Counter/timer D7-0
#define CTRDAT1     13  // 13 Counter/timer D15-8
#define CTRDAT2     14  // 14 Counter/timer D23-16
#define CTRCTL      15  // 15 Counter/timer control register

// Read functions, wherever different from the write function.
#define ADLSB       0   // 0  A/D LSB
#define ADSTATUS    3   // 3  Analog Input Status
#define ADMSB       1   // 1  A/D MSB
#define FIFODEPTH   6   // 6  FIFO current depth 
#define INTADCHREAD 7   // 7  Interrupt and A/D channel readback
#define FPGAVERSION 15  // 15 FPGA Revision code

// DIODIR register masks
#define DIODIR_DIOCTR 0x80       // Should be true when the upper half of port C is configured for digital I/O.

// COMMAND register masks
#define COMMAND_STRTAD 	0x80     // Start an A/D conversion
#define COMMAND_RSTBRD 	0x40     // Reset the board excluding the D/A.
#define COMMAND_RSTDA  	0x20     // Reset the analog outputs.
#define COMMAND_RSTFIFO 0x10     // Reset the FIFO depth to 0 (clear the FIFO).
#define COMMAND_CLRDMA  0x08     // Reset the DMA interrupt request flip flop.
#define COMMAND_CLRT    0x04     // Reset the timer interrupt request flip flop.
#define COMMAND_CLRD    0x02     // Reset the digital I/O interrupt request flip flop.
#define COMMAND_CLRA    0x01     // Reset the analog interrupt request flip flop.

// Analog Input Status masks
#define ADSTATUS_STS    0x80     // A/D state, 1 indicates A/D conversion or scan is in progress
#define ADSTATUS_SD     0x40     // 1 indicates single ended mode, 0 indicates differential
#define ADSTATUS_WAIT   0x20     // WAIT, indicates A/D gain circuit is settling
#define ADSTATUS_DACBSY 0x10     // indicates the DAC is busy updating
#define ADSTATUS_OVF    0x08     // indicates FIFO has overflowed
#define ADSTATUS_SCANEN 0x04
#define ADSTATUS_G1     0x02
#define ADSTATUS_G0     0x01

// Loop counter for timeouts on busywaits.
#define LOOPTIMEOUT     100000

/****************************************************************/
// A macro to test whether the driver object state indicates the
// hardware is in an initialized state.
#define DAQ_READY(driver_object) (((driver_object) != NULL) && \
				  ((driver_object)->initialized) && \
				  !((driver_object)->closed))

/****************************************************************/
// Create an empty AthenaDAQ object.
AthenaDAQ *
AthenaDAQ_alloc(void)
{
  AthenaDAQ *daq = (AthenaDAQ *) calloc (1, sizeof( AthenaDAQ ) );
}

// Initialize a AthenaDAQ device.  The structure may be statically allocated.
AthenaDAQ *
AthenaDAQ_init( AthenaDAQ *daq )
{
  return AthenaDAQ_init_with_address( daq, 0x280 ); // default base address
}

AthenaDAQ *
AthenaDAQ_init_with_address( AthenaDAQ *daq, short base_address )
{
  unsigned char fpga_rev;

  if ( daq != NULL ) { 
    daq->base_address = base_address;
    daq->initialized = 1;
    daq->closed = 0;

    // The following is applied to the ADCHANNEL register.  The
    // default is to scan all channels, assuming the board is
    // configued for single-ended mode.
    daq->channel_range = 0xf0;

    // A flag to keep track of the FIFO state to support separate 
    // initiate and finish operations for A/D conversion.
    daq->scan_in_progress = 0;

    // The following value is applied to the ADGAINSCAN register.
    // This default value has scan enabled, with +/- 10V input range,
    // assuming bipolar mode is set on the jumpers.
    daq->ad_mode           = 0x04;   

    // If in the future code is added to configure the analog gain, it
    // can indicate the range with these variables:
    daq->min_analog_input = -10.0; 
    daq->max_analog_input =  10.0; 

    // Perform hardware initialization.

    // Check if the FPGA chip select is enabled, taken care of by
    // the BIOS setting.  It doesn't take any action, assuming
    // that the BIOS setting can be changed, and also to keep
    // this driver compatible with the Prometheus hardware.
    {
      int fpgaen = inb( WATCHDOG_SERIAL_FPGA_CONTROL );
      if (! (fpgaen  & FPGAEN_MASK) ) {
	errprintf("AthenaDAQ_init: error, FPGA appears not to be enabled\n");
	// outb( fpgaen | FPGAEN_MASK, WATCHDOG_SERIAL_FPGA_CONTROL );
      }
    }

    // Check FPGA revision
    fpga_rev = inb( base_address + FPGAVERSION );
    logprintf("AthenaDAQ driver: found FPGA revision code 0x%02x.\n", fpga_rev );

    // This is the code observed on the board we purchased for Flame.  There is 
    // no documentation on what it should be or what it means.
    if ( fpga_rev != 0x23 ) {
      errprintf("AthenaDAQ_init: error, invalid FPGA revision 0x%02x.\n", fpga_rev );
      daq->initialized = 0;

    } else {

      // Reset the board including the analog outputs.
      outb( COMMAND_RSTBRD | COMMAND_RSTDA, base_address + COMMAND );

    }
  }
  return daq;
}

/****************************************************************/
// Shut down a AthenaDAQ device.  The driver structure may be
// statically allocated, i.e., this doesn't deallocate any memory.

void 
AthenaDAQ_close( AthenaDAQ *daq )
{
  if ( daq != NULL ) { 
    daq->closed = 1;

    // Perform hardware shutdown.
    // This might reset the D/A converters.
  }  
}

/****************************************************************/
// De-allocate a dynamically allocated AthenaDAQ object, stopping the
// device if still active.
void 
AthenaDAQ_dealloc( AthenaDAQ *daq )
{
  if ( daq != NULL ) { 
    if ( daq->initialized && !daq->closed ) AthenaDAQ_close( daq );
    free ( daq );
  }  
}

// Test if the device initialized properly.
int AthenaDAQ_is_ready( AthenaDAQ *daq )
{
  return DAQ_READY( daq );
}

/****************************************************************/
void 
AthenaDAQ_write_digital_output_byte( AthenaDAQ *daq, unsigned port, unsigned char byte )
{
  if ( DAQ_READY(daq) && port < 3) {
    outb( byte, daq->base_address + port + DIOPORTA );  // write DIO output byte to one of three registers
  }
}
void 
AthenaDAQ_write_digital_output_bits( AthenaDAQ *daq, unsigned port, unsigned char mask, unsigned char bits )
{
  if ( DAQ_READY(daq) && port < 3) {
    unsigned char value = inb( daq->base_address + port + DIOPORTA );  // read existing values

    // mask selects which bits of of value to change; the first term clears them, the second writes them
    value = (value & ~mask) | ( bits & mask );

    // write it back out
    outb( value, daq->base_address + port + DIOPORTA );  // write DIO output byte to one of three registers
  }
}

unsigned char 
AthenaDAQ_read_digital_input_byte( AthenaDAQ *daq, unsigned port )
{
  if ( DAQ_READY(daq) & port < 3) {
    return inb( daq->base_address + port + DIOPORTA );  // read DIO input byte
  } else return 0;
}

void
AthenaDAQ_configure_digital_direction( AthenaDAQ *daq, unsigned flags )
{
  if ( DAQ_READY(daq) ) {  
    outb( (flags & 0xff) | DIODIR_DIOCTR, daq->base_address + DIODIR );
  }
}
/****************************************************************/
// Write a new value to a D/A converter channel.  It takes 20
// microseconds to update a single channel due to the DAC serial
// interface.  For this reason, just a single channel update function
// is provided on the expecation that the user will intersperse
// individual channel updates with other I/O operations to avoid
// excessive busy-waiting.
void 
AthenaDAQ_write_analog_output( AthenaDAQ *daq, unsigned channel, short value )
{
  unsigned timeout;
  if ( DAQ_READY(daq) && channel < 4) {

    // Clamp the input value to a signed 12 bit value.
    if (value > ATHENADAQ_MAX_ANALOG_OUTPUT) value = ATHENADAQ_MAX_ANALOG_OUTPUT;
    else if (value < ATHENADAQ_MIN_ANALOG_OUTPUT) value = ATHENADAQ_MIN_ANALOG_OUTPUT;

    // Convert from a signed integer to an unsigned 12 bit integer.
    value += 2048;

    // Wait for previous serial data transfer to DAC to finish.  This 
    // could take up to 20 microseconds.
    timeout = 0;

    while ( inb_p(daq->base_address + ADSTATUS) & ADSTATUS_DACBSY ) {
      if (++timeout > LOOPTIMEOUT) {
	errprintf("AthenaDAQ_write_analog_output: D/A ready wait timed out.\n");
	return;
      }
    }

    // Dangerous! disable interrupts.  This is possible because of iopl().  The Athena
    // manual claims it is a write to DAMSBCHAN which updates the converter, but 
    // I was observing glitches when interrupts were enabled, presumably because
    // the LSB register was getting written and then an interrupt was occurring before
    // MSBCHAN was written.

    asm ("cli");  

    // Write out bottom 8 bits.
    outb_p( value & 0xff, daq->base_address + DALSB );  

    // Write out the top 8 bits plus the channel selector.
    outb_p( ((value >> 8) & 0x0f) | (channel << 6), daq->base_address + DAMSBCHAN ); 

    asm ("sti");  // re-enable interrupts.
  }
}

/****************************************************************/
void AthenaDAQ_start_analog_input_scan( AthenaDAQ *daq )
{
  unsigned timeout;
  if ( DAQ_READY(daq) && !daq->scan_in_progress ) {

    outb( daq->ad_mode, daq->base_address + ADGAINSCAN );         // configure the A/D gains and enable scan  
    outb( daq->channel_range, daq->base_address + ADCHANNEL );    // configure the scan range  
    outb( COMMAND_RSTFIFO, daq->base_address + COMMAND );         // reset the FIFO by writing the FIFORST bit
    
    // Wait for WAIT bit to indicate analog input circuit has settled.  This takes
    // 10 microseconds.
    timeout = 0;    
    while( inb( daq->base_address + ADSTATUS) & ADSTATUS_WAIT ) {
      if (++timeout > LOOPTIMEOUT) {
	errprintf("AthenaDAQ_start_analog_input_scan: analog gain settling wait timed out.\n");
	return;
      }
    }

    outb( COMMAND_STRTAD, daq->base_address + COMMAND );         // trigger scan

    daq->scan_in_progress = 1;
  }
}

int AthenaDAQ_is_analog_scan_done( AthenaDAQ *daq )
{
  return ( DAQ_READY(daq) 
	   && daq->scan_in_progress 
	   && !( inb( daq->base_address + ADSTATUS ) & ADSTATUS_STS) );
}

// Read the FIFO depth to test how the A/D conversion is progressing.
int AthenaDAQ_analog_input_FIFO_status( AthenaDAQ *daq )
{
  if ( DAQ_READY(daq) ) {
    return inb( daq->base_address + FIFODEPTH );
  } else {
    return 0;
  }
}

void AthenaDAQ_finish_analog_input_scan( AthenaDAQ *daq )
{
  int channel, low, high;
  unsigned timeout;

  if ( DAQ_READY(daq) && daq->scan_in_progress ) {

    // determine number of channels read
    low  = daq->channel_range & 0x0f;
    high = ( daq->channel_range >> 4) & 0x0f;
    
    // wait for A/D converter scan to finish (bit STS goes low)
    timeout = 0;
    while( inb( daq->base_address + ADSTATUS ) & ADSTATUS_STS ) {
      if (++timeout > LOOPTIMEOUT) {
	errprintf("AthenaDAQ_finish_analog_input_scan: A/D scan complete wait timed out.\n");
	daq->scan_in_progress = 0;    
	return;
      }
    }

    // read FIFO data
    for ( channel = low; channel <= high; channel++ ) {
      int lsb = inb( daq->base_address + ADLSB );
      int msb = inb( daq->base_address + ADMSB );
      daq->analog_inputs[ channel ] = lsb | (msb<<8);
    }
    daq->scan_in_progress = 0;    
  }
}

void AthenaDAQ_read_all_analog_inputs( AthenaDAQ *daq )
{
  AthenaDAQ_start_analog_input_scan( daq );
  AthenaDAQ_finish_analog_input_scan( daq );
}

