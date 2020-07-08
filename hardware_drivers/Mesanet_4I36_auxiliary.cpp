// $Id: Mesanet_4I36_auxiliary.c,v 1.1 2005/11/21 14:21:24 garthz Exp $
// Mesanet_4I36_auxiliary.c : additional driver code for the Mesanet 4I36 PC/104 quadrature encoder counter board
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

/****************************************************************/
static unsigned ReadCounter( Mesanet_4I36 *m,  int ch ) 
{
  writeportw( m,  Index, ch & ChSelMask);
  {
    unsigned short LowWord  = readportw( m,  CounterLow );
    unsigned short HighWord = readportw( m,  CounterHigh );
    unsigned int value = LowWord | (HighWord << 16);
    m->count[ ch & ChSelMask ] = value;
    return value;
  }
}
/****************************************************************/
static void ClearCounter( Mesanet_4I36 *m,  int ch )
{
  short int oldccr;

  writeportw( m,  Index, ch & ChSelMask );
  oldccr = readportw( m,  CounterCont );
  writeportw( m,  CounterCont, oldccr | IndexClear );
  m->count[ ch & ChSelMask ] = 0;
}
/****************************************************************/
static void FilterOn( Mesanet_4I36 *m,  int ch )
{
  short int oldccr;
  writeportw( m,  Index, ch & ChSelMask );
  oldccr = readportw( m,  CounterCont );
  writeportw( m,  CounterCont, (oldccr & ~IndexClear) | QuadFilter);
}
/****************************************************************/
static void FilterOff( Mesanet_4I36 *m,  int ch )
{
  short int oldccr;
  writeportw( m,  Index, ch & ChSelMask );
  oldccr = readportw( m,  CounterCont );
  writeportw( m,  CounterCont, (oldccr & ~IndexClear) | ~QuadFilter);
}
/****************************************************************/
// latch an individual counter
// only works if LatchOnRead not set on this counter

static void LatchCounter( Mesanet_4I36 *m,  int ch ) 
{
  writeportw( m,  Index, ch & ChSelMask );
  writeportw( m,  CounterLow, 0 );
}

/****************************************************************/
// latch all counters that do not have LatchOnRead set
static void LatchAllCounters( Mesanet_4I36 *m )
{
  writeportw( m,  CounterHigh, 0 );
}
