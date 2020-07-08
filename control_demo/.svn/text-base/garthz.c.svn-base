// garthz.c : Flame biped robot walking controller.
//
// Copyright (C) 2001-2006 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
// This is an ad-hoc heuristic controller based on a state machine,
// descended from similar controllers for the walking machines in the
// CMU Atkeson lab.
//
// This was developed on a simulator which is resembles Flame, but
// which was written before the design was complete.  It has been
// modified for the correct morphology, but not yet successfully
// tested.

/****************************************************************/
#include <math.h>

#include <hardware_drivers/FlameIO.h>
#include <hardware_drivers/FlameIO_defs.h>
#include "system.h"
#include "globals.h"
#include "joint.h"
#include "polynomials.h"

#define RADIAN(d) ((d)*(M_PI/180.0)) 

#ifndef NULL
#define NULL ((void *) 0)
#endif

#define ST_LEG_LEFT 0

/****************************************************************/
// Define the controller state indices.
enum {
  S_BEGIN = 0,
  S_LAUNCH,

  S_SWING,
  S_TOEOFF,

  S_CRASH = 1000,
  S_NOSTATE        // must be the last symbol
};

/****************************************************************/
// These pointers are set to substructures of s so that control
// can be written more abstractly in terms of swing and stance
// leg, as they swapped back and forth.

static leg_dof_t *st_q,   *sw_q; 
static leg_dof_t *st_qd,  *sw_qd;
static leg_tau_t *st_tau, *sw_tau;

// Pointers for joint controller parameters.
static joint_param_t *sw_ankley_joint;
static joint_param_t *st_ankley_joint;
static joint_param_t *sw_knee_joint;
static joint_param_t *st_knee_joint;
static joint_param_t *sw_hipy_joint;
static joint_param_t *st_hipy_joint;

// Foot switch values.
static int sw_back_switch, st_back_switch;       
static int sw_front_switch, st_front_switch;       

/****************************************************************/
// Advance the state machine to a new state.  The state indices are defined in symbols.h.
inline static void
transition( int newstate )
{
  if (newstate >= 0 && newstate < S_NOSTATE) {
    c.garthz.tstart  = c.t;
    c.garthz.inx     = -newstate; // negative to signal the state is not yet initialized
  }
}
// measure elapsed time in the current state
inline static double
elapsed ( void )
{
  return ( c.t - c.garthz.tstart );
}

/****************************************************************/
// initialize this controller state
void init_garthz_controller( void )
{
  c.garthz.tstart = 0.0;
  c.garthz.motors =  FLAME_ALL_MOTORS;
  c.garthz.st_leg = ST_LEG_LEFT;  // initial assumed stance leg is the left leg
  transition( S_SWING );
  strcpy ( c.garthz.inxname, "init");

  // computed values
  c.garthz.sw_leg_ang = 0;
  c.garthz.st_leg_ang = 0;
  c.garthz.rel_ang_COM = 0;
  c.garthz.rel_omega_COM = 0;
  c.garthz.rel_d_leg_ang = 0;
  c.garthz.t0_swing = 0;
  c.garthz.rel0_ang_COM = 0;

  // controller parameters
  c.garthz.q_d_pitch = 0.0;
  c.garthz.tmin_swing = 0.1;
  c.garthz.q_d_hipx = 0.0;
  c.garthz.qd_d_hipx = 1.0;
  c.garthz.k_hipx = 200;
  c.garthz.b_hipx = 10;
  c.garthz.k_yd_hipx = 0.0;
  c.garthz.k_roll_hipx = 0.0;
  c.garthz.sw_k_hipy = 50;
  c.garthz.sw_b_hipy = 1;
  c.garthz.st_k_hipy = 50;
  c.garthz.st_b_hipy = 1;
  c.garthz.t_hipy_extend = 0.2;
  c.garthz.ext_rel_ang_COM = -0.35;
  c.garthz.ret_rel_omega_COM = 0.80;
  c.garthz.ret_max_ang_COM = -0.32;
  c.garthz.sw_q_d_knee = 1.35;
  c.garthz.sw_qd_d_knee = 10.0;
  c.garthz.sw_k_knee = 20;
  c.garthz.sw_b_knee = 1;
  c.garthz.st_q_d_knee = 0.0;
  c.garthz.st_qd_d_knee = 4.0;
  c.garthz.st_k_knee = 40;
  c.garthz.st_b_knee = 1;
  c.garthz.t_knee_retract = 0.13;
  c.garthz.t_knee_bent = 0.05;
  c.garthz.t_knee_extend = 0.2;
  c.garthz.st_q_d_ankley = -0.14;
  c.garthz.st_qd_d_ankley = 0.20;
  c.garthz.st_k_ankley = 100;
  c.garthz.st_b_ankley = 5;
  c.garthz.sw_q_d_ankley = -0.09;
  c.garthz.sw_qd_d_ankley = 0.3;
  c.garthz.sw_k_ankley = 100;
  c.garthz.sw_b_ankley = 5;
  c.garthz.t_ankley_bent = 0.260;
  c.garthz.to_q_d_ankley = -0.34;
  c.garthz.to_k_ankley = 100;
  c.garthz.to_b_ankley = 5;
  c.garthz.to_ff_tau_ankley = 0;
  c.garthz.tmin_toeoff = 0.0;
  c.garthz.tmax_toeoff = 0.12;
}
/****************************************************************/
static void
set_stance_swing_pointers(void)
{
  // Select which leg is the stance leg and which is the swing
  // leg.  This makes the walking code simpler since everything
  // can be phrased in terms of "swing" and "stance" even as the
  // roles of left and right keep switching.

  if ( c.garthz.st_leg == ST_LEG_LEFT ) {
    st_q    = &s.q.l;        // left leg stance
    st_qd   = &s.qd.l;
    st_tau  = &s.tau.l;

    st_back_switch  = s.foot.l.back.state;
    st_front_switch = s.foot.l.front.state;

    st_ankley_joint = &c.joint.l.ankley;
    st_knee_joint   = &c.joint.l.knee;
    st_hipy_joint   = &c.joint.l.hipy;

    sw_q     = &s.q.r;        // right leg swing
    sw_qd    = &s.qd.r;
    sw_tau   = &s.tau.r;

    sw_back_switch  = s.foot.r.back.state;
    sw_front_switch = s.foot.r.front.state;

    sw_ankley_joint = &c.joint.r.ankley;
    sw_knee_joint   = &c.joint.r.knee;
    sw_hipy_joint   = &c.joint.r.hipy;

  } else { 
    st_q    = &s.q.r;        // right leg stance
    st_qd   = &s.qd.r;
    st_tau  = &s.tau.r;

    st_back_switch  = s.foot.r.back.state;
    st_front_switch = s.foot.r.front.state;

    st_ankley_joint = &c.joint.r.ankley;
    st_knee_joint   = &c.joint.r.knee;
    st_hipy_joint   = &c.joint.r.hipy;

    sw_q     = &s.q.l;       // left leg swing
    sw_qd    = &s.qd.l;
    sw_tau   = &s.tau.l;

    sw_back_switch  = s.foot.l.back.state;
    sw_front_switch = s.foot.l.front.state;

    sw_ankley_joint = &c.joint.l.ankley;
    sw_knee_joint   = &c.joint.l.knee;
    sw_hipy_joint   = &c.joint.l.hipy;
  }
}
/****************************************************************/
static double inline swing_hipy_torque(void)
{
  // Servo swing leg COM angle to fixed offset from stance leg COM,
  // and apply damping in terms of relative COM angular velocity, which
  // will decouple the knee motion.

  return ( ( -c.garthz.sw_k_hipy * (c.garthz.rel_ang_COM - c.garthz.rel_d_leg_ang))
	   - (c.garthz.sw_b_hipy * c.garthz.rel_omega_COM )
	   );
  
}

/****************************************************************/
// Compute the desired relative leg COM angle as a function of time.
static double 
relative_leg_COM_trajectory(void)
{
  garthz_con_t *p = &c.garthz;      // control parameters for this controller

  // find the current time along the trajectory
  double t = ( c.t - p->t0_swing ) / p->t_hipy_extend;
  if ( t < 0.0 ) t = 0.0;

  if ( t < 1.0) {
    return compute_quintic_spline( t, 
				   p->rel0_ang_COM,    	    // the recorded initial angle
				   0.0, 0.0,           	    // initial zero velocity and acceleration
				   p->ext_rel_ang_COM, 	    // the relative leg angle at extension
				   p->ret_rel_omega_COM,    // the retraction velocity at that point
				   0.0 );
  } else {
    double angle = p->ext_rel_ang_COM +  p->ret_rel_omega_COM * ( t - 1.0 );
    if ( angle > p->ret_max_ang_COM ) angle = p->ret_max_ang_COM;
    return angle;
  }
}

/****************************************************************/
// Compute the knee bend trajectory as a function of time.
static double 
knee_trajectory( void )
{
  garthz_con_t *p = &c.garthz;      // control parameters for this controller

  // find the current time along the trajectory
  double dt =  c.t - p->t0_swing;
  if ( dt < p->t_knee_retract ) {
    return compute_quintic_spline_pp( dt / p->t_knee_retract, p->st_q_d_knee, p->sw_q_d_knee );
  }

  dt -= p->t_knee_retract;
  if ( dt < p->t_knee_bent ) {
    return p->sw_q_d_knee;
  }

  dt -= p->t_knee_bent;
  return compute_quintic_spline_pp( dt / p->t_knee_extend, p->sw_q_d_knee, p->st_q_d_knee );
}

/****************************************************************/
// control polling function
void update_garthz_controller( void )
{
  int state_init;       // true if state index is being used for the first time, used to signal transition code
  garthz_con_t *p = &c.garthz;      // control parameters for this controller

  // If we are running, then compute control outputs depending upon the mode and state index.
  state_init = (p->inx < 0);        // signal state initialization code
  if ( state_init ) p->inx = -p->inx;

  // configure swing and stance for current leg modes
  set_stance_swing_pointers();

  // estimate body pitch, assuming the stance foot is flat
  c.garthz.pitch = -st_q->ankley - st_q->knee - st_q->hipy;

  // compute the overall hip-foot angle, assuming equal hip and shin length
  c.garthz.sw_leg_ang = c.garthz.pitch + sw_q->hipy + 0.5*sw_q->knee;
  c.garthz.st_leg_ang = c.garthz.pitch + st_q->hipy + 0.5*st_q->knee;

  // compute some very rough approximations of leg COM angular velocity and position
  if ( c.garthz.st_leg == ST_LEG_LEFT ) {
    c.garthz.rel_omega_COM = s.qd.r.hipy - s.qd.l.hipy;
    c.garthz.rel_ang_COM   = s.q.r.hipy - s.q.l.hipy;
  } else {
    c.garthz.rel_omega_COM = s.qd.l.hipy - s.qd.r.hipy;
    c.garthz.rel_ang_COM   = s.q.l.hipy - s.q.r.hipy;
  }

  // safety check, either falling or being hoisted
  // if ( p->inx != S_CRASH && (s.q.z < 0.5) ) transition ( S_CRASH );

  // This is the switch-case that drives the walking the state machine.  For every 
  // state, select the control action and check for state transitions.

  switch (p->inx) {

  case S_BEGIN:       
    // make all joints limp by default
    set_joint_LIMP_mode( &c.joint.hipx );
    set_joint_LIMP_mode( &c.joint.l.hipy );
    set_joint_LIMP_mode( &c.joint.l.knee );
    set_joint_LIMP_mode( &c.joint.l.ankley);
    set_joint_LIMP_mode( &c.joint.r.hipy );
    set_joint_LIMP_mode( &c.joint.r.knee );
    set_joint_LIMP_mode( &c.joint.r.ankley);
    update_joint_controllers();

    // always assume starting on the left leg
    c.garthz.st_leg = ST_LEG_LEFT;
    transition( S_LAUNCH ); // enter the cycle at the beginning of swing
    break;


  case S_LAUNCH:
    if ( state_init ) {
      logprintf("Entered LAUNCH. Press button2 to begin walking.\n");

      // Initialize a rigid set of controllers to help the manual
      // launch.  Remember that the global gain ramp is taking
      // place while these are running.

      // initialize all joints to fixed position controllers

      p->rel0_ang_COM = 0.3;

      set_joint_QDPD_mode( &c.joint.hipx,  p->q_d_hipx, p->qd_d_hipx, s.q.hipx, p->k_hipx, p->b_hipx );

      set_joint_QDPD_mode( &c.joint.l.hipy, -0.20, 0.5, s.q.l.hipy, p->st_k_hipy, p->st_b_hipy );
      set_joint_QDPD_mode( &c.joint.r.hipy, c.joint.l.hipy.q_d + p->rel0_ang_COM, 0.5, s.q.r.hipy, p->sw_k_hipy, p->sw_b_hipy );

      set_joint_QDPD_mode( &c.joint.l.knee, p->st_q_d_knee, p->st_qd_d_knee, s.q.l.knee, p->st_k_knee, p->st_b_knee );
      set_joint_QDPD_mode( &c.joint.r.knee, p->sw_q_d_knee, p->sw_qd_d_knee, s.q.r.knee, p->sw_k_knee, p->sw_b_knee );

      set_joint_QDPD_mode( &c.joint.l.ankley, p->st_q_d_ankley,  p->st_qd_d_ankley,  s.q.l.ankley, p->st_k_ankley, p->st_b_ankley );
      set_joint_QDPD_mode( &c.joint.r.ankley, p->sw_q_d_ankley,  p->sw_qd_d_ankley,  s.q.r.ankley, p->sw_k_ankley, p->sw_b_ankley );
    }

    // on every tick, just run the controllers
    update_joint_controllers();

    if ( st_front_switch &&
	 st_back_switch &&
	 !sw_front_switch &&
	 !sw_back_switch &&
	 FLAME_PUSHBUTTON_PRESSED( s.front_panel_sw, PUSHBUTTON2 )) {

      transition( S_SWING ); // enter the cycle at the beginning of swing
    }
    break;

    // ================ normal walking cycle ====================================

  case S_SWING:
    strcpy ( c.garthz.inxname, "swing");

    if ( state_init ) {
      logprintf("Entering SWING, st_leg = %s\n", (p->st_leg == ST_LEG_LEFT) ? "LEFT" : "RIGHT" );

      // hold leg roll at fixed angle
      set_joint_QDPD_mode( &c.joint.hipx,  p->q_d_hipx, p->qd_d_hipx, s.q.hipx, p->k_hipx, p->b_hipx );

      // turn off hipy joint controllers, swing hipy torque will be computed explicitly
      set_joint_OFF_mode( sw_hipy_joint );
      set_joint_OFF_mode( st_hipy_joint );

      // the knees are fully controlled through a set of positions
      set_joint_QDPD_mode( sw_knee_joint, p->sw_q_d_knee, p->sw_qd_d_knee, sw_q->knee, p->sw_k_knee, p->sw_b_knee );
      set_joint_QDPD_mode( st_knee_joint, p->st_q_d_knee, p->st_qd_d_knee, st_q->knee, p->st_k_knee, p->st_b_knee );

      // retract swing ankle a bit
      set_joint_QDPD_mode( sw_ankley_joint, p->sw_q_d_ankley, p->sw_qd_d_ankley, sw_q->ankley, p->sw_k_ankley, p->sw_b_ankley );

      // hold stance ankle at fixed angle
      set_joint_QDPD_mode( st_ankley_joint, p->st_q_d_ankley, p->st_qd_d_ankley, st_q->ankley, p->st_k_ankley, p->st_b_ankley );
    }

    // servo swing leg COM angle to fixed offset from stance leg COM
    p->rel_d_leg_ang = relative_leg_COM_trajectory();
    sw_tau->hipy = swing_hipy_torque();
      
    // use stance leg to servo body pitch to vertical, and add a feedforward torque for the swing leg
    st_tau->hipy = ( (p->st_k_hipy * (c.garthz.pitch - p->q_d_pitch)) - (p->st_b_hipy * st_qd->hipy) - sw_tau->hipy );

    // compute desired knee angle
    sw_knee_joint->q_d = knee_trajectory();

    // compute ankley trajectory
    if ( (c.t - p->t0_swing) > p->t_ankley_bent ) {

      // extend swing ankle again
      sw_ankley_joint->q_d = p->st_q_d_ankley;
    }

    // compute all the individual joints
    update_joint_controllers();

    // Transition once the swing foot touches down.
    if ( (elapsed() > p->tmin_swing)      // if the minimum time has passed
	 && st_back_switch                // and the stance foot is still firmly placed
	 && st_front_switch
	 && sw_back_switch                // and the swing leg foot has made contact
	 ) {
      transition( S_TOEOFF );             // then begin the toeoff

      p->st_leg = 1 - p->st_leg;  // switch roles of stance and swing legs
    }
    break;

    // Double support phase.  The legs have changed roles, the swing leg is now in the rear.
  case S_TOEOFF:
    strcpy ( c.garthz.inxname, "toe off");

    if ( state_init ) {
      logprintf("Entering TOEOFF, st_leg = %s\n", (p->st_leg == ST_LEG_LEFT) ? "LEFT" : "RIGHT" );
      // capture the time of the beginning of the stride
      p->t0_swing = c.t;

      // capture the initial leg separation
      p->rel0_ang_COM = p->rel_ang_COM;

      // The swing leg is now the rear leg.  The swing knee will retain stance gains for the 
      // duration of toe-off.

      // The stance leg is now the front leg; this should primarily be a gain change.
      set_joint_QDPD_mode( st_knee_joint, p->st_q_d_knee, p->st_qd_d_knee, st_q->knee, p->st_k_knee, p->st_b_knee );

      // The front ankle should now switch from the swing gains to the stance gains.
      st_ankley_joint->k   = p->st_k_ankley;
      st_ankley_joint->b   = p->st_b_ankley;
      st_ankley_joint->q_d = p->st_q_d_ankley;

      // Toe off the rear ankle.
      sw_ankley_joint->q_d    = p->to_q_d_ankley;
      sw_ankley_joint->k      = p->to_k_ankley;
      sw_ankley_joint->b      = p->to_b_ankley;
      sw_ankley_joint->ff_tau = p->to_ff_tau_ankley;
    }
    // servo swing leg COM angle to fixed offset from stance leg COM
    p->rel_d_leg_ang = relative_leg_COM_trajectory();
    sw_tau->hipy = swing_hipy_torque();

    // compute desired knee angle
    sw_knee_joint->q_d = knee_trajectory();

    // use stance leg to servo body pitch to vertical, and add a feedforward to torque for the swing leg
    st_tau->hipy = ( (p->st_k_hipy * (c.garthz.pitch - p->q_d_pitch)) - (p->st_b_hipy * st_qd->hipy) - sw_tau->hipy );

    // compute all the individual joints
    update_joint_controllers();

    if ( elapsed() > p->tmin_toeoff     // if the minimum time has passed
	 && !sw_back_switch             // and the rear foot heel has lifted
	 && st_back_switch              // and both front foot switches are in contact
	 && st_front_switch
	 && ( !sw_front_switch ||              // and either the rear toe has lifted
	      sw_q->ankley > p->to_q_d_ankley  // or the rear ankle has extended to the limit
	      // || elapsed() > p->tmax_toeoff    // or the maximum time has passed
	      )
	 ) transition( S_SWING );              // begin swinging the rear leg forward
    break;

    // =================== end of normal walking cycle =======================

  case S_CRASH:
    strcpy ( c.garthz.inxname, "crash");
    // Just keep recording data to get post-crash trajectory.

    c.joint.hipx.mode = JOINT_OFF;

    c.joint.l.hipy.mode = JOINT_OFF;
    c.joint.r.hipy.mode = JOINT_OFF;
    c.joint.l.knee.mode = JOINT_OFF;
    c.joint.r.knee.mode = JOINT_OFF;
    c.joint.l.ankley.mode = JOINT_OFF;
    c.joint.r.ankley.mode = JOINT_OFF;

    s.tau.hipx   = 0.0;

    s.tau.l.hipy   = 0.0;
    s.tau.l.knee   = 0.0;
    s.tau.l.ankley = 0.0;

    s.tau.r.hipy   = 0.0;
    s.tau.r.knee   = 0.0;
    s.tau.r.ankley = 0.0;

    break;
  }

  // finish up with some common code

  // normally this enables all motors, but individual ones can be disabled for testing
  if (p->inx != S_CRASH)   s.powered = p->motors;   
  else                     s.powered = 0;              // if crashed, disable motors
}
