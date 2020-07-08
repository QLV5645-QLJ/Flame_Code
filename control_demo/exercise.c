// exercise.c - move the robot through a calibration sequence to find the index pulses.

// This sequence assumes that the joint controllers are operating
// purely using the motor encoders, and does a motion on each set
// of axes relative to the initial configuration.  In each case
// the motor encoder will reset within a single rotation, and
// perhaps cause a small step in the error.  The joint encoder
// should reset at some point during the motion.  In the final
// state the joint values will be correct, and the motor offsets
// can then be adjusted.

#include <math.h>
#include <hardware_drivers/FlameIO.h>
#include <hardware_drivers/FlameIO_defs.h>
#include "system.h"
#include "globals.h"
#include "joint.h"
#include "polynomials.h"

#define HIPX_POS_EX     0.1
#define HIPY_POS_EX     0.35
#define HIPY_NEG_EX     0.35
#define KNEE_POS_EX     0.9
#define ANKLE_T         4.0

/****************************************************************/
// define symbols for the state machine
enum {
  S_BEGIN = 0,
  S_HIPX,
  S_HIPX_BACK,
  S_EXERCISE_LEFTLEG_UP,
  S_EXERCISE_RIGHTLEG_UP,
  S_EXERCISE_LEGS_DOWN,
  S_END,
  S_NOSTATE
};

/****************************************************************/
// Advance the state machine to a new state.
inline static void
transition( int newstate )
{
  if (newstate >= 0 && newstate < S_NOSTATE) {
    c.exercise.tstart  = c.t;
    c.exercise.inx     = -newstate; // negative to signal the state is not yet initialized
  }
}
// measure elapsed time in the current state
inline static double
elapsed ( void )
{
  return ( c.t - c.exercise.tstart );
}


/****************************************************************/
// initialize this controller state
void init_exercise_controller(void)
{
  transition( S_BEGIN );
}

/****************************************************************/
// control polling function
void update_exercise_controller(void)
{
  int state_init;    // true if state index is being used for the first time, used to signal transition code

  // If we are running, then compute control outputs depending upon the mode and state index.
  state_init = (c.exercise.inx < 0);        // signal state initialization code
  if ( state_init ) c.exercise.inx = -c.exercise.inx;

  // This is the switch-case that drives the state machine.  For every 
  // state, select the control action and check for state transitions.

  switch (c.exercise.inx) {

  case S_BEGIN:
    // make all joints limp by default
    set_joint_LIMP_mode( &c.joint.hipx );
    set_joint_LIMP_mode( &c.joint.l.hipy );
    set_joint_LIMP_mode( &c.joint.l.knee );
    set_joint_LIMP_mode_ankle( &c.joint.l.ankley);
    set_joint_LIMP_mode( &c.joint.r.hipy );
    set_joint_LIMP_mode( &c.joint.r.knee );
    set_joint_LIMP_mode_ankle( &c.joint.r.ankley);

    // set all motor encoders to zero. These will obtain the correct angles when pushbutton 1 
    // is pressed at the end of the exercise mode.
    params.offset.q.l.hipymot    -= s.q.l.hipymot;
    params.offset.q.l.kneemot    -= s.q.l.kneemot;
    params.offset.q.l.ankleymot  -= s.q.l.ankleymot;
    params.offset.q.r.hipymot    -= s.q.r.hipymot;
    params.offset.q.r.kneemot    -= s.q.r.kneemot;
    params.offset.q.r.ankleymot  -= s.q.r.ankleymot;

    update_joint_controllers_exercise();
    s.powered = FLAME_ALL_MOTORS;
    
    c.joint.l.hipy.kp = 80.0;
    c.joint.l.hipy.kd = 0.2;
    c.joint.r.hipy.kp = 80.0;
    c.joint.r.hipy.kd = 0.2;
    
    c.joint.l.knee.kp = 25.0;
    c.joint.l.knee.kd = 0.2;
    c.joint.r.knee.kp = 25.0;
    c.joint.r.knee.kd = 0.2;

    // capture the initial configuration at the beginning of a case
    // Do it before every new case so when segments move meanwhile, the 
    // exercise program doesn't apply suddenly huge torques.
    c.q_start.hipx     = s.q.hipx;
     transition( S_HIPX );
    break;
    
  case S_HIPX:
    // This is the mode where the hipx moves first to fin the index pulse.
    // After initiation stiffen hipx
    c.joint.hipx.q_d = compute_quintic_spline( 0.5*elapsed(), c.q_start.hipx, 0.0, 0.0, \
					       c.q_start.hipx + HIPX_POS_EX,  0.0, 0.0  );
    c.joint.hipx.kd = 0.0;
    set_joint_PD_mode( &c.joint.hipx);
    
    update_joint_controllers_exercise();
    
    if ( 0.5*elapsed() >= 1.0 ) {
    transition ( S_HIPX_BACK );
  }
  break;
  
  case S_HIPX_BACK:
      // This is the mode where hipx goes back.   
      c.joint.hipx.q_d = compute_quintic_spline( 0.5*elapsed(), c.q_start.hipx + HIPX_POS_EX, 0.0, 0.0, \
						 c.q_start.hipx + 0.08, 0.0, 0.0);
      set_joint_PD_mode( &c.joint.hipx);
      update_joint_controllers_exercise();
   
      if ( 0.5*elapsed() >= 1.0 ) {
        transition( S_EXERCISE_LEFTLEG_UP );
      }
      break;

    case S_EXERCISE_LEFTLEG_UP:
    if ( state_init ) {
      c.q_start.l.hipy   = s.q.l.hipymot;
      c.q_start.l.knee   = s.q.l.kneemot;
      c.q_start.r.hipy   = s.q.r.hipymot;
      c.joint.l.hipy.kd = 0.0;
      c.joint.l.knee.kd = 0.0;
      c.joint.r.hipy.kd = 0.0;
      c.joint.l.ankley.kp = 1.0;
      c.joint.l.ankley.kd = 0.03;
    }
    // Move the left leg up and bended (knee + ankle). Move the right leg back.
    c.joint.l.hipy.q_d= compute_quintic_spline(0.5*elapsed(),c.q_start.l.hipy,0.0,0.0,c.q_start.l.hipy-HIPY_NEG_EX,0.0,0.0);
    c.joint.l.knee.q_d= compute_quintic_spline(0.5*elapsed(),c.q_start.l.knee,0.0,0.0,c.q_start.l.knee+KNEE_POS_EX,0.0,0.0);
    c.joint.l.ankley.tau_d = compute_quintic_spline( 0.5*elapsed(), 0.0, 0.0, 0.0, ANKLE_T, 0.0, 0.0 );
    c.joint.r.hipy.q_d= compute_quintic_spline(0.5*elapsed(),c.q_start.r.hipy,0.0,0.0,c.q_start.r.hipy+HIPY_POS_EX,0.0,0.0);
    set_joint_PD_mode( &c.joint.l.hipy );
    set_joint_PD_mode( &c.joint.l.knee );
    set_joint_torque_PD_mode_ankle( &c.joint.l.ankley );
    set_joint_PD_mode( &c.joint.r.hipy);
    
    update_joint_controllers_exercise();
    
    if (  0.5*elapsed() >= 1.0 ) { // if elapsed time of slowest motion is bigger then 1, then switch to the next case
      transition( S_EXERCISE_RIGHTLEG_UP );
    }  
    break;

  case S_EXERCISE_RIGHTLEG_UP:
    if ( state_init ) {
      c.q_start.r.knee   = s.q.r.kneemot;
      c.q_start.r.ankley = s.q.r.ankleymot;
      c.joint.r.hipy.kd = 0.0;
      c.joint.r.knee.kd = 0.0;
      c.joint.r.ankley.kp = 1.0;
      c.joint.r.ankley.kd = 0.03;
    }
      // Left leg down and right leg up
      c.joint.l.hipy.q_d = compute_quintic_spline( 0.5*elapsed(), c.q_start.l.hipy - HIPY_NEG_EX, 0.0, 0.0, \
						   c.q_start.l.hipy + HIPY_POS_EX, 0.0, 0.0);
      c.joint.l.knee.q_d = compute_quintic_spline( 0.5*elapsed(), c.q_start.l.knee + KNEE_POS_EX, 0.0, 0.0, \
						   c.q_start.l.knee, 0.0, 0.0);
      c.joint.l.ankley.tau_d = compute_quintic_spline( 0.5*elapsed(), ANKLE_T, 0.0, 0.0, 0.0, 0.0, 0.0 );
      c.joint.r.hipy.q_d = compute_quintic_spline( 0.5*elapsed(), c.q_start.r.hipy + HIPY_POS_EX, 0.0, 0.0, \
						   c.q_start.r.hipy - HIPY_NEG_EX, 0.0, 0.0);
      c.joint.r.knee.q_d = compute_quintic_spline( 0.5*elapsed(), c.q_start.r.knee, 0.0, 0.0, \
						   c.q_start.r.knee + KNEE_POS_EX, 0.0, 0.0);
      c.joint.r.ankley.tau_d = compute_quintic_spline( 0.5*elapsed(), 0.0, 0.0, 0.0, ANKLE_T, 0.0, 0.0  );

      set_joint_PD_mode( &c.joint.l.hipy);
      set_joint_PD_mode( &c.joint.l.knee);
      set_joint_torque_PD_mode_ankle( &c.joint.l.ankley );
      set_joint_PD_mode( &c.joint.r.hipy);
      set_joint_PD_mode( &c.joint.r.knee);
      set_joint_torque_PD_mode_ankle( &c.joint.r.ankley );

      update_joint_controllers_exercise();

      if (  0.5*elapsed() >= 1.0 ) { // if elapsed time of the slowest motion is bigger then 1, switch to the next case
        transition( S_EXERCISE_LEGS_DOWN );
      }  
      break;

   case S_EXERCISE_LEGS_DOWN:
      if ( state_init ) {
       set_joint_LIMP_mode( &c.joint.l.knee );
       set_joint_LIMP_mode_ankle( &c.joint.l.ankley );
      }

     c.joint.l.hipy.q_d = compute_quintic_spline(0.5*elapsed(),c.q_start.l.hipy+HIPY_POS_EX,0.0,0.0,c.q_start.l.hipy,0.0,0.0);
     c.joint.r.hipy.q_d = compute_quintic_spline(0.5*elapsed(),c.q_start.r.hipy-HIPY_NEG_EX,0.0,0.0,c.q_start.r.hipy,0.0,0.0);
     c.joint.r.knee.q_d = compute_quintic_spline(0.5*elapsed(),c.q_start.r.knee+KNEE_POS_EX,0.0,0.0,c.q_start.r.knee,0.0,0.0);
     c.joint.r.ankley.tau_d = compute_quintic_spline( 0.5*elapsed(), ANKLE_T, 0.0, 0.0, 0.0, 0.0, 0.0 );

     set_joint_PD_mode( &c.joint.r.hipy);
     set_joint_PD_mode( &c.joint.r.knee);
     set_joint_torque_PD_mode_ankle( &c.joint.r.ankley);
     set_joint_PD_mode( &c.joint.l.hipy);

     update_joint_controllers_exercise();

     if ( 0.5*elapsed() >= 1.0 ) {   // time of slowest motion is reached, switch to the next case
       transition( S_END );
       }
     break;

  case S_END:
    if ( state_init) {
       set_joint_LIMP_mode( &c.joint.l.hipy );
       set_joint_LIMP_mode_ankle( &c.joint.l.ankley );
       set_joint_LIMP_mode( &c.joint.l.knee );
       set_joint_LIMP_mode_ankle( &c.joint.r.ankley );
       set_joint_LIMP_mode( &c.joint.r.knee );
       set_joint_LIMP_mode( &c.joint.r.hipy );
       // keep hipx active

      logprintf("END OF EXERCISE!\n Hold the hipx in the '0' position.\nPress button 1 \
 to recalibrate and advance to standing mode...\n");
    }

    update_joint_controllers();     
    s.powered = 0;

    break;
  }
}
