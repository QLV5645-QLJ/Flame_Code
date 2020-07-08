// $Id: Mesanet_4I36.h,v 1.4 2005/12/09 09:05:55 garthz Exp $
// Mesanet_4I36.h : driver for the Mesanet 4I36 PC/104 quadrature encoder counter board
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#ifndef MESANET_4I36_H_INCLUDED
#define MESANET_4I36_H_INCLUDED

/****************************************************************/
// Define a device structure to hold configuration and state
// information.  This is not written as an opaque structure, but
// is subject to change, so it is preferable to use access
// methods rather than modify the fields directly.

typedef struct {
  // Possible base addresses: 0x220 0x230 0x240 0x250.
  unsigned short base_address;

  // The last read count values.  These are signed integers,
  // which is more convenient than unsigned when the reset zeros
  // the counter.

  // N.B.: When the index pulse resets the counter it goes to
  // zero.  If the encoder happens to be moving in a negative
  // direction, the counter will then have negative values.  If
  // moving in a positive direction, it will have positive
  // values.  So simply turning on the index pulse does not
  // guarantee a one to one relationship between count and
  // position.

  int count[8];                          

  // driver status flags
  unsigned int initialized      :1;      // true if this data structure is valid
  unsigned int closed           :1;      // true if hardware has been closed

} Mesanet_4I36;

/****************************************************************/
// The object creation and destruction methods loosely follow the Objective-C style.

// Create an empty Mesanet_4I36 object.
extern Mesanet_4I36 *Mesanet_4I36_alloc(void);                          

// Initialize a Mesanet_4I36 device.  The structure may be statically allocated.
extern Mesanet_4I36 *Mesanet_4I36_init( Mesanet_4I36 *board );
extern Mesanet_4I36 *Mesanet_4I36_init_with_address( Mesanet_4I36 *board, short base_address ); 

// Shut down a Mesanet_4I36 device.   The structure may be statically allocated.
extern void Mesanet_4I36_close( Mesanet_4I36 *board ); 

// De-allocate an allocated Mesanet_4I36 object, stopping the device if still active.
extern void Mesanet_4I36_dealloc( Mesanet_4I36 *board );

// Test if the device initialized properly.
extern int Mesanet_4I36_is_ready( Mesanet_4I36 *board );

/****************************************************************/
// Configure the index pulse operation for a given channel.
#define MESANET_IDX_MODE_NO_INDEX     0
#define MESANET_IDX_MODE_RESET_ONCE   1  // reset just once on a index, suitable for motor axes 
#define MESANET_IDX_MODE_RESET_ALWAYS 2  // (default) reset on every index, suitable for joint axes 
extern void Mesanet_4I36_configure_index( Mesanet_4I36 *board, unsigned channel, int index_mode);

// Reset all channels.
extern void Mesanet_4I36_clear_all_counters( Mesanet_4I36 *m );

// Reset a particular channel.
extern void Mesanet_4I36_clear_counter( Mesanet_4I36 *board, unsigned channel );

// I/O operations
extern void Mesanet_4I36_read_all_counters( Mesanet_4I36 *board );

#endif // MESANET_4I36_H_INCLUDED
