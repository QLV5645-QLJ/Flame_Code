// demo.c : Flame biped demonstration controller.

// Copyright (C) 2001-2006 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.

#include <math.h>
#include <hardware_drivers/FlameIO.h>
#include <hardware_drivers/FlameIO_defs.h>
#include "system.h"
#include "globals.h"
#include "joint.h"
#include "polynomials.h"

/****************************************************************/
// define symbols for the state machine
enum {
  S_SLEEP = 0,
  S_DEMO,
  S_NOSTATE
};

/****************************************************************/
// Advance the state machine to a new state.
inline static void
transition( int newstate )
{
  if (newstate >= 0 && newstate < S_NOSTATE) {
    c.demo.tstart  = c.t;
    c.demo.inx     = -newstate; // negative to signal the state is not yet initialized
  }
}

// measure elapsed time in the current state
inline static double
elapsed ( void )
{
  return ( c.t - c.demo.tstart );
}

/****************************************************************/
// initialize this controller state
void init_demo_controller(void)
{
  c.demo.amplitude = 0.1;
  c.demo.frequency = 0.2;

  transition( S_SLEEP );
}

/****************************************************************/
// control polling function
void update_demo_controller(void)
{
  int state_init;    // true if state index is being used for the first time, used to signal transition code

  // If we are running, then compute control outputs depending upon the mode and state index.
  state_init = (c.demo.inx < 0);        // signal state initialization code
  if ( state_init ) c.demo.inx = -c.demo.inx;

  // This is the switch-case that drives the state machine.  For every 
  // state, select the control action and check for state transitions.

  switch ( c.demo.inx ) {

  case S_SLEEP:
    s.powered = 0;

    if ( state_init ) {
      logprintf("Push button 1 to advance to DEMO controller\n");
    }
    // transition to standing state
       if ( FLAME_PUSHBUTTON_PRESSED( s.front_panel_sw, PUSHBUTTON1 )) {
	 transition( S_DEMO );}
    break;

  case S_DEMO:
    if ( state_init ) {
      logprintf("demo controller entering DEMO state\n");
      
      // capture the initial configuration at the beginning; all motions will be relative to this one
      s.powered = FLAME_ALL_MOTORS;
      
      c.q_start.hipx     = s.q.hipx     ;     
      c.q_start.l.ankley = s.q.l.ankley ; 
      c.q_start.l.knee   = s.q.l.knee   ;
      c.q_start.l.hipy   = s.q.l.hipy   ;
      c.q_start.r.ankley = s.q.r.ankley ;
      c.q_start.r.knee   = s.q.r.knee   ;
      c.q_start.r.hipy   = s.q.r.hipy   ;
      // Generate test sine wave trajectories on all axes.  Some
      // axes are biased to only move through positive
      // displacements from the starting configuration.
    }
    c.joint.hipx.q_d     = c.q_start.hipx     + 0.8 * c.demo.amplitude * (1.6 + sin ( c.demo.frequency * 2.0 * M_PI * c.t ));
    c.joint.l.ankley.q_d = c.q_start.l.ankley + 0.5 * c.demo.amplitude * (1.0 + sin ( c.demo.frequency * 2.0 * M_PI * c.t ));  
    c.joint.l.knee.q_d   = c.q_start.l.knee   + 0.5 * c.demo.amplitude * (1.0 + sin ( c.demo.frequency * 2.0 * M_PI * c.t ));  
    c.joint.l.hipy.q_d   = c.q_start.l.hipy   + 0.5 * c.demo.amplitude * (0.0 + sin ( c.demo.frequency * 2.0 * M_PI * c.t ));  
    c.joint.r.ankley.q_d = c.q_start.r.ankley + 0.5 * c.demo.amplitude * (1.0 + sin ( c.demo.frequency * 2.0 * M_PI * c.t ));
    c.joint.r.knee.q_d   = c.q_start.r.knee   + 0.5 * c.demo.amplitude * (1.0 - sin ( c.demo.frequency * 2.0 * M_PI * c.t ));  
    c.joint.r.hipy.q_d   = c.q_start.r.hipy   + 0.5 * c.demo.amplitude * (0.0 - sin ( c.demo.frequency * 2.0 * M_PI * c.t )); 
    
    
    // PD servos
    s.tau.hipx     =(c.joint.hipx.kp     *(c.joint.hipx.q_d     - s.q.hipx     )) - (c.joint.hipx.kd     * s.qd.hipx );
    s.tau.l.hipy   =(c.joint.l.hipy.kp   *(c.joint.l.hipy.q_d   - s.q.l.hipymot  )) - (c.joint.l.hipy.kd   * s.qd.l.hipymot );
    s.tau.l.knee   =(c.joint.l.knee.kp   *(c.joint.l.knee.q_d   - s.q.l.kneemot  )) - (c.joint.l.knee.kd   * s.qd.l.kneemot );
    s.tau.l.ankley =(c.joint.l.ankley.kp *(c.joint.l.ankley.q_d - s.q.l.ankleymot)) - (c.joint.l.ankley.kd * s.qd.l.ankleymot);
    s.tau.r.hipy   =(c.joint.r.hipy.kp   *(c.joint.r.hipy.q_d   - s.q.r.hipymot  )) - (c.joint.r.hipy.kd   * s.qd.r.hipymot );
    s.tau.r.knee   =(c.joint.r.knee.kp   *(c.joint.r.knee.q_d   - s.q.r.kneemot  )) - (c.joint.r.knee.kd   * s.qd.r.kneemot );
    s.tau.r.ankley =(c.joint.r.ankley.kp *(c.joint.r.ankley.q_d - s.q.r.ankleymot)) - (c.joint.r.ankley.kd * s.qd.r.ankleymot);
  
    break;
  }
}
