// standing.c : Flame biped stand-in-place controller.

// Copyright (C) 2001-2006 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.

#include <hardware_drivers/FlameIO.h>
#include <hardware_drivers/FlameIO_defs.h>
#include "system.h"
#include "globals.h"
#include "joint.h"
#include "polynomials.h"
#include "math.h"

/****************************************************************/
// define symbols for the state machine
enum {
  S_SLEEP = 0,
  S_STAND,
  S_STAND_CONTROL,
  S_NOSTANDSTATE
};

// define symbols for the determination of position or torque controller
enum {
  S_POSITION = 0,
  S_TORQUE,
  S_UNKNOWN,
};

#define LBODY 0.175        // length body mass above hipx: estimated
#define LHIP  0.069        // width of hip (anklex to hipx)
#define LLEG  0.647        // length total leg (anklex hipx)!

/****************************************************************/
// Advance the state machine to a new state.
inline static void transition( int newstate )
{
  if (newstate >= 0 && newstate < S_NOSTANDSTATE) {
    c.standing.tstart  = c.t;
    c.standing.inx     = -newstate; // negative to signal the state is not yet initialized
  }
}
// measure elapsed time in the current state
inline static double elapsed ( void )
{
  return ( c.t - c.standing.tstart );
}

/****************************************************************/
// initialize this controller state
void init_standing_controller(void)
{
  transition( S_SLEEP );
}
/****************************************************************/
static inline void xmass( void ) 
{
  c.joint.hipx.data = LLEG*sin(s.q.r.anklex) + LHIP*cos(s.q.r.anklex) + LBODY*sin(s.q.r.anklex); // -s.q.hipx
}
/****************************************************************/
// determine which controller to use
inline static char determine_controller(void) 
{
  char controller_choice = S_UNKNOWN;
  if ((s.foot.l.back.count > 30 || s.foot.l.front.count > 30 ) && (s.foot.r.front.count > 30 || s.foot.r.back.count > 30))
       controller_choice = S_POSITION;
  
  if ((s.foot.l.back.count <-30 && s.foot.l.front.count <-30) || (s.foot.r.front.count < -30 && s.foot.r.back.count < -30))
      controller_choice = S_TORQUE; 
  
  return ( controller_choice );
}

/****************************************************************/
// control polling function
void update_standing_controller(void)
{
  int state_init;    // true if state index is being used for the first time, used to signal transition code

  // If we are running, then compute control outputs depending upon the mode and state index.
  state_init = (c.standing.inx < 0);        // signal state initialization code
  if ( state_init ) c.standing.inx = -c.standing.inx;

  // This is the switch-case that drives the state machine.  For every 
  // state, select the control action and check for state transitions.

  switch (c.standing.inx) {

  case S_SLEEP:
    // make all joints limp by default
    set_joint_LIMP_mode( &c.joint.hipx     );
    set_joint_LIMP_mode( &c.joint.l.hipy   );
    set_joint_LIMP_mode( &c.joint.l.knee   );
    set_joint_LIMP_mode_ankle( &c.joint.l.ankley );
    set_joint_LIMP_mode( &c.joint.r.hipy   );
    set_joint_LIMP_mode( &c.joint.r.knee   );
    set_joint_LIMP_mode_ankle( &c.joint.r.ankley );

    update_joint_controllers();
    s.powered = 0;

    // immediately transition to standing state
    transition( S_STAND );
    break;

  case S_STAND:
    if ( state_init ) {
      logprintf("standing controller initializing STAND state\nPut Flame on ground to go to strong controller.\n");

      // The gains that are used to find the index pulses are not the correct gains for the standing controller.
      // these should change a little bit, especially for the ankle motors
      c.joint.r.ankley.kp = 5.0;
      c.joint.l.ankley.kp = 5.0;
      c.joint.r.ankley.kd = 0.1;
      c.joint.l.ankley.kd = 0.1;
      c.joint.l.knee.kp = 40;
      c.joint.r.knee.kp = 40;
      c.joint.l.knee.kd = 0.2;
      c.joint.r.knee.kd = 0.2;
      c.joint.l.hipy.kp = 80;
      c.joint.r.hipy.kp = 80;
      c.joint.l.hipy.kd = 0.5;
      c.joint.r.hipy.kd = 0.5;
      c.joint.hipx.qref = s.q.hipx;

      // initialize all joints to fixed position controllers
      set_joint_QDPD_mode( &c.joint.l.hipy,   HIPY_POS,  0.3,   s.q.l.hipy);    
      set_joint_QDPD_mode( &c.joint.l.knee,   KNEE_POS,  0.3,   s.q.l.knee);
      set_joint_QDPD_mode( &c.joint.r.hipy,   HIPY_POS,  0.3,   s.q.r.hipy);  
      set_joint_QDPD_mode( &c.joint.r.knee,   KNEE_POS,  0.3,   s.q.r.knee);
      s.powered = FLAME_ALL_MOTORS;
    }

    c.joint.hipx.q_d = compute_quintic_spline( 0.5*elapsed(), c.joint.hipx.qref, 0.0, 0.0, HIPX_POS, 0.0, 0.0);
    c.joint.l.ankley.q_d = compute_quintic_spline( 0.5*elapsed(), s.q.l.ankley, 0.0, 0.0, ANKLE_POS, 0.0, 0.0);
    c.joint.r.ankley.q_d = compute_quintic_spline( 0.5*elapsed(), s.q.r.ankley, 0.0, 0.0, ANKLE_POS, 0.0, 0.0);
    set_joint_PD_mode( &c.joint.hipx ); 
    set_joint_position_PD_mode_ankle( &c.joint.l.ankley ); 
    set_joint_position_PD_mode_ankle( &c.joint.r.ankley ); 

    // on every tick, just run the controllers
    update_joint_controllers();

    // If Flame is placed on ground, go to the normal PD controller
    if ( s.foot.l.back.state && s.foot.r.back.state ) {
       transition ( S_STAND_CONTROL );
    }
    break;

  case S_STAND_CONTROL:
    if ( state_init ){
      logprintf("STAND PD CONTROL state\nPress button 1 to go start walking, Press button 2 to increase the Kpp\n");
    
      c.joint.r.ankley.kp = 4.0; // Torque control gains
      c.joint.l.ankley.kp = 4.0;
      c.joint.r.ankley.kd = 0.1;  // Torque control gains
      c.joint.l.ankley.kd = 0.1;
      c.joint.l.ankley.tau_d = 4.0; // about the amount of torque delivered by the front springs when feet at 0 rad.
      c.joint.r.ankley.tau_d = 4.0; 

      c.joint.hipx.q_d     =   HIPX_POS;
      c.joint.l.knee.q_d   =   KNEE_POS;
      c.joint.l.ankley.q_d =   ANKLE_POS;  
      c.joint.l.hipy.q_d   =   HIPY_POS;
      c.joint.r.knee.q_d   =   KNEE_POS;
      c.joint.r.ankley.q_d =   ANKLE_POS;  
      c.joint.r.hipy.q_d   =   HIPY_POS;

      // The PD controllers of the series elastic actuators need the velocity change of the desired angle:
      c.joint.l.ankley.qd_d = 0.0;
      c.joint.r.ankley.qd_d = 0.0;

      set_joint_PD_mode( &c.joint.hipx );
      set_joint_PD_mode( &c.joint.l.hipy);
      set_joint_PD_mode( &c.joint.l.knee);
      set_joint_PD_mode( &c.joint.r.hipy);
      set_joint_PD_mode( &c.joint.r.knee);
    }
    
    // This part is to increase k or kpp incrementally during standing. This is useful to find to 
    // best gains inmediately in one run.
    /*
     *{
     * static int debounce = 0;
     * if ( FLAME_PUSHBUTTON_PRESSED( s.front_panel_sw, PUSHBUTTON2 )) {
     *	c.LEDS |= LED2;
     *	debounce++;
     *	if ( debounce == 5 ) {
     *	  c.joint.l.ankley.kp += 4;
     *	  c.joint.r.ankley.kp += 4;
     *     logprintf("K = %f\n",c.joint.l.ankley.k);
     *	  //c.joint.l.ankley.kpp += 10;
     *	  //c.joint.r.ankley.kpp += 10;
     *	  //logprintf("Kpp = %f\n",c.joint.l.ankley.kpp);
     *	}
     * } else {
     *	debounce = 0;
     *	c.LEDS &= ~LED2;
     * }
     *}
     */

    // if both feet are on the ground, a position controller can be used. This is not possible when the 
    // feet are not on the ground because the gains are way to high for the small mass of the feet.
    if ( determine_controller() == S_POSITION ) { 
      // position controller
      c.joint.l.ankley.kp  = 16.0;
      c.joint.r.ankley.kp  = 16.0;
      c.joint.l.ankley.kd  = 0.1;
      c.joint.r.ankley.kd  = 0.1;
      c.joint.l.ankley.kpp = 200; 
      c.joint.r.ankley.kpp = 200; 
      c.joint.l.ankley.kdd = 2.0; 
      c.joint.r.ankley.kdd = 2.0; 

      set_joint_position_PD_mode_ankle( &c.joint.l.ankley );
      set_joint_position_PD_mode_ankle( &c.joint.r.ankley );
    }

    else if ( determine_controller() == S_TORQUE ){
      // Torque controller
      c.joint.l.ankley.kp  = 16.0;
      c.joint.r.ankley.kp  = 16.0;
      c.joint.l.ankley.kd  = 0.1;
      c.joint.r.ankley.kd  = 0.1;
      c.joint.l.ankley.tau_d = 4.0;
      c.joint.r.ankley.tau_d = 4.0;

      set_joint_torque_PD_mode_ankle( &c.joint.l.ankley );
      set_joint_torque_PD_mode_ankle( &c.joint.r.ankley );
    }

    xmass();
    update_joint_controllers();

  break;
  }
}
