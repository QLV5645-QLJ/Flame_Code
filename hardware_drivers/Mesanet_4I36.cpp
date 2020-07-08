// $Id: Mesanet_4I36.c,v 1.6 2005/12/14 17:30:35 garthz Exp $
// Mesanet_4I36.c : driver for the Mesanet 4I36 PC/104 quadrature encoder counter board
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#include <stdio.h>
#include <stdlib.h>
#include <asm/io.h>
#include <unistd.h>

#include <hardware_drivers/Mesanet_4I36.h>

// Define logprintf, errprintf.
#include <utility/utility.h>

// Register map, all relative to base address.
#define Index       0x00        // Index Register
#define CounterLow  0x02        // Counter Low Register (indexed)
#define CounterHigh 0x04        // Counter High Register (indexed)
#define CounterCont 0x06        // Counter Control Register (indexed)
#define PortA       0x08        // Port A Data Register
#define PortADDR    0x0A        // Port A Data Direction Register
#define PortB       0x0C        // Port B Data Register
#define PortBDDR    0x0E        // Port B Data Direction Register

// Index register bits (bits 0 to 2 are counter channel select bits)
#define IDXConfigEna   0x8000      //  must be set to enable EEPROM re-config 
#define IDXAutoInc     0x4000      //  if set, reading counter high increments index 
#define IDXGlobalClr   0x2000      //  if set clears all counters 
#define IDXGlobalHold  0x1000      //  if set holds all counters 
#define ChSelMask      0x0007

// Counter control register bits (per counter) 
#define GlobalClear    0x2000      //  same as in index reg 
#define GlobalHold     0x1000      //  same as in index reg 
#define AutoCount      0x800       //  if set, counter counts up at 48 MHz
#define CounterMode    0x400       //  if set counter is in up down mode (1X), otherwise quadrature mode (4X) 
#define QuadFilter     0x200       //  use slow input digital filter if set. this limits input frequency to about 1.5 MHz,
                                   //    and quad count rate to about 6 MHz 
#define LocalHold      0x100       //  holds count when set 
#define IndexGate      0x80        //  if set, quada and quadb must be low to detect index
#define ClearOnce      0x40        //  if set, ClearOnIndex is cleared once index is detected 
#define ClearOnIndex   0x20        //  if set, index edge will clear counter 
#define IndexPolarity  0x10        //  sets polarity of active index edge 
#define LatchOnRead    0x08        //  if set counter is latched on read from host 
#define IndexClear     0x04        //  read = real time index signal, if set on write, clears counter 
#define QuadB          0x02        //  read only - real time quadb input
#define QuadA          0x01        //  read only - real time quada input

#define RefRate      48000000      //  the reference count rate when a counter is used as a timer

/****************************************************************/
// Functions for addressing a particular Mesa board.  The use of
// outw_p instead of outw and inw_p insted of inw is critical;
// the _p versions pause until I/O is complete.  Without forcing
// the CPU to complete the I/O operation, the driver will
// sometimes freeze the machine.  This has been observed running
// under RTAI; it's not clear if it happens running as a normal
// Linux process.

static inline void writeportw( Mesanet_4I36 *m, int address, int value)
{
  outw_p( value, m->base_address + address );
}

static inline unsigned short readportw( Mesanet_4I36 *m, int address) 
{
  return inw_p( m->base_address + address );
}

/****************************************************************/
// Create an empty Mesanet_4I36 object.
Mesanet_4I36 *Mesanet_4I36_alloc(void)
{
  Mesanet_4I36 *board = (Mesanet_4I36 *) calloc (1, sizeof( Mesanet_4I36 ) );
}

// Initialize a Mesanet_4I36 device.  The structure may be statically allocated.
Mesanet_4I36 *Mesanet_4I36_init( Mesanet_4I36 *board )
{
  return Mesanet_4I36_init_with_address( board, 0x220 ); // default base address
}

static int base_address_valid( short address )
{
  return ( address == 0x220 || address == 0x230 || address == 0x240 || address == 0x250 );
}
Mesanet_4I36 *Mesanet_4I36_init_with_address( Mesanet_4I36 *board, short base_address )
{
  int value;
  int c;
  if ( board != NULL && base_address_valid( base_address ))  { 
    board->base_address = base_address;
    board->initialized = 1;
    board->closed = 0;

    // Perform hardware initialization.

    // In general, the counters are not cleared at startup so that the
    // correct encoder count may be maintained across program
    // invocations.  This requires that the index pulses occur after
    // powerup before the values are used.

    // writeportw( m, Index, IDXGlobalClr );  // clears all counters
    // writeportw( m, Index, 0 );             // normal operation

    // Attempt to detect the presence of a board.

#define READBACK (IDXGlobalHold + 5)
    writeportw( board, Index, READBACK );
    value = readportw( board, Index );
    writeportw( board, Index, 0 );   // restore normal state

    if ( value != READBACK ) {
      errprintf( "Mesanet_4I36_init( 0x%x ): error, index register readback failed: read 0x%x, expected 0x%x\n", 
		 board->base_address, value, READBACK );
      board->initialized = 0;

    } else {
      logprintf( "Mesa 4I36 driver for board 0x%x passed read back test\n", board->base_address, value);

      // Configure each counter channel to reset on index pulses.
      for (c = 0; c < 8; c++) {
	writeportw( board,  Index, c );
	writeportw( board,  CounterCont, LatchOnRead | QuadFilter | ClearOnIndex );
	board->count[c] = 0;
      }
    }
  }
  return board;
}

/****************************************************************/
// Shut down a Mesanet_4I36 device.  The driver structure may be statically allocated.
void Mesanet_4I36_close( Mesanet_4I36 *board )
{
  if ( board != NULL ) { 
    if (board->initialized && !board->closed) {

      // perform hardware shutdown

      board->closed = 1;
    }
  }  
}

/****************************************************************/
// De-allocate an allocated Mesanet_4I36 object, stopping the device if still active.
void Mesanet_4I36_dealloc( Mesanet_4I36 *board )
{
  if ( board != NULL ) { 
    if ( board->initialized && !board->closed ) Mesanet_4I36_close( board );

    free ( board );
  }  
}

// Test if the device initialized properly.
int Mesanet_4I36_is_ready( Mesanet_4I36 *m )
{
  return (m != NULL && m->initialized && !m->closed);
}

/****************************************************************/
void Mesanet_4I36_configure_index( Mesanet_4I36 *m, unsigned channel, int index_mode)
{
  if ( m != NULL && m->initialized && !m->closed && channel < 8 ) {
    writeportw( m,  Index, channel );
    switch (index_mode) {

    case MESANET_IDX_MODE_NO_INDEX:       // ignore the index pulse input
      writeportw( m,  CounterCont, LatchOnRead | QuadFilter );
      break;

    case MESANET_IDX_MODE_RESET_ONCE:     // reset the counter on the first index pulse
      writeportw( m,  CounterCont, LatchOnRead | QuadFilter | ClearOnIndex | ClearOnce );
      break;

    case MESANET_IDX_MODE_RESET_ALWAYS:
    default: // reset the counter on every index pulse
      writeportw( m,  CounterCont, LatchOnRead | QuadFilter | ClearOnIndex );      
      break;
    }
  }
}

/****************************************************************/
// Reset a particular channel.
void Mesanet_4I36_clear_counter( Mesanet_4I36 *m, unsigned channel )
{
  if ( m != NULL && m->initialized && !m->closed && channel < 8 ) {
    short int oldccr;
    writeportw( m,  Index, channel );
    oldccr = readportw( m,  CounterCont );
    writeportw( m,  CounterCont, oldccr | IndexClear );
    m->count[ channel ] = 0;
  }
}

/****************************************************************/
void Mesanet_4I36_clear_all_counters( Mesanet_4I36 *m )
{
  if ( m != NULL && m->initialized && !m->closed ) {
    writeportw( m, Index, IDXGlobalClr );  // clears all counters
    writeportw( m, Index, 0 );             // normal operation
  }
}

/****************************************************************/
void Mesanet_4I36_read_all_counters( Mesanet_4I36 *m )
{
  if ( m != NULL && m->initialized && !m->closed ) {
    short int counter;

    // Enable the auto-increment feature and select register 0.
    writeportw( m, Index, IDXAutoInc );

    // Latch all counters.
    writeportw( m, CounterHigh, 0 );

    // Read each counter; the index will automatically advance
    // after each high word is read.
    for (counter = 0; counter < 8; counter++ ) {

      unsigned short LowWord, HighWord;
      int value;

      asm ("cli");  // disable interrupts
      LowWord  = readportw( m,  CounterLow );
      HighWord = readportw( m,  CounterHigh );
      asm ("sti");  // re-enable interrupts.
      value = LowWord | (HighWord << 16);
      m->count[ counter ] = value;
    }
  }
}
