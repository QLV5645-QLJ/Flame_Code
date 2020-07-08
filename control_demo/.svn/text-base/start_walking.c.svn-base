// Start_walking.c : Flame biped robot initiate walking controller.
//
// This file assumes that Flame is in a balancing position and wants 
// to start walking. It runs through one stride! Then the normal walking 
// controller starts. The difference is that the first step is 
// significantly different than the normal walking steps.
// The initiation can be seen as two parts; placing the mass above
// the right leg (cases launch and launch2). Then the left leg is 
// placed forward.
//
// Thijs Mandersloot - TUDelft - january 2007
/****************************************************************/
#include <math.h>
#include <hardware_drivers/FlameIO.h>
#include <hardware_drivers/FlameIO_defs.h>
#include "system.h"
#include "globals.h"
#include "joint.h"
#include "polynomials.h"

/****************************************************************/
// Define the controller state indices.
enum {
  START_BEGIN = 0,
  START_BEGIN_WAIT,
  START_LAUNCH,
  START_LAUNCH_2,
  START_STANCE_RIGHT_LEG,
  START_UNKNOWN        // must be the last symbol
};

#define LBODY 0.175        // length body mass above hipx: estimated
#define LHIP  0.069        // width of hip (anklex to hipx)
#define LLEG  0.647        // length total leg (anklex hipx)!
#define XTOE  0.107

#define CP 200 // number of positive counts before we are sure that that switch is on the ground
#define CN -60 // number of negative counts before we are sure that that switch is on the ground

#define LIFT_TORQUE       15.6*9.81/2*XTOE + 4   // The torque that is needed to lift half the weight of flame.
#define ANKLEX_POS       -0.06                   // the first desired sideways angle around right foot 
#define ANKLEX_POS_FINAL -0.06                   // the final desired sideways angle around right foot 
#define T_ANKLE_BOOST1    4.0                    // boost to lift up the weight launch_2
#define T_ANKLE_BOOST2    2.0                    // boost to lift up left leg to get clearance from ground.
/****************************************************************/
// Advance the state machine to a new state.  The state indices are defined in symbols.h.
inline static void transition( int newstate )
{
  if (newstate >= 0 && newstate < START_UNKNOWN) {
    c.walking.tstart  = c.t;
    c.walking.inx     = -newstate; // negative to signal the state is not yet initialized
  }
}
// measure elapsed time in the current state
inline static double elapsed ( void )
{
  return ( c.t - c.walking.tstart );
}
/****************************************************************/
// initialize this controller state
void init_start_walking_controller( void )
{ // THESE PARAMS ARE ALSO USED FOR WALKING.C
  c.walking.tstart       = 0.0;
  c.walking.inx          = START_BEGIN;
  c.walking.knee_inx     = 0;
  c.walking.t1           = 0.0;
  c.walking.t2           = 0.0;

  c.walking.rel_hipangle_q    = 0.0;
  c.walking.rel_hipangle_qd   = 0.0;
  c.walking.rel_hipangle_q_d  = 0.0;
  c.walking.rel_hipangle_qd_d = 0.0;
  c.walking.rel_hipangle_prev = 0.0;

  // controller parameters
  c.walking.hipx.q_d            = 0.1;
  c.walking.hipx.qd_d           = 0.0;
  c.walking.hipx.ymass          = 0.0;

  c.walking.swing.hipy.q_d        =  0.0;
  c.walking.swing.hipy.q_prev     =  0.0;
  c.walking.swing.hipy.qd_d       =  0.0;
  c.walking.swing.hipy.qd_prev    =  0.0;
  c.walking.swing.hipy.tau_d      =  0.0;
  c.walking.swing.hipy.tau_prev   =  0.0;
  c.walking.swing.hipy.taud_prev  =  0.0;
  c.walking.stance.hipy.q_d       =  0.0;
  c.walking.stance.hipy.q_prev    =  0.0;
  c.walking.stance.hipy.qd_d      =  0.0;
  c.walking.stance.hipy.qd_prev   =  0.0;
  c.walking.stance.hipy.tau_d     =  0.0;
  c.walking.stance.hipy.tau_prev  =  0.0;
  c.walking.stance.hipy.taud_prev =  0.0;

  c.walking.swing.knee.q_d        = 0.0;
  c.walking.swing.knee.q_prev     = 0.0;
  c.walking.swing.knee.qd_d       = 0.0;
  c.walking.swing.knee.qd_prev    = 0.0;
  c.walking.swing.knee.tau_d      = 0.0;
  c.walking.swing.knee.tau_prev   = 0.0;
  c.walking.swing.knee.taud_prev  = 0.0;
  c.walking.stance.knee.q_d       = 0.0;
  c.walking.stance.knee.qd_d      = 0.0;
  c.walking.stance.knee.tau_d     = 0.0; 
  c.walking.stance.knee.tau_prev  = 0.0;
  c.walking.stance.knee.taud_prev = 0.0;

  c.walking.stance.ankley.q_d       = 0.0;
  c.walking.stance.ankley.qd_d      = 0.0;
  c.walking.stance.ankley.tau_d     = 0.0;
  c.walking.stance.ankley.tau_prev  = 0.0;
  c.walking.stance.ankley.taud_prev = 0.0;
  c.walking.swing.ankley.q_d        = 0.0;
  c.walking.swing.ankley.qd_d       = 0.0;
  c.walking.swing.ankley.tau_d      = 0.0;
  c.walking.swing.ankley.tau_prev   = 0.0;
  c.walking.swing.ankley.taud_prev  = 0.0;

  c.walking.l.anklex.q_d         = 0.0;
  c.walking.l.anklex.qd_d        = 0.0;
  c.walking.l.anklex.kmp         = 300.0;
  c.walking.l.anklex.kmd         = 0.0; // The damping should be active (kmd > 0), but therefore velocity estimation should work better
  c.walking.l.anklex.yfoot       = 0.0;
  c.walking.l.anklex.tau_extra   = 0.0;
  c.walking.l.anklex.q_prev      = 0.0;
  c.walking.l.anklex.tau_prev    = 0.0;

  c.walking.r.anklex.q_d         = 0.0;
  c.walking.r.anklex.qd_d        = 0.0;
  c.walking.r.anklex.kmp         = 300.0;
  c.walking.r.anklex.kmd         = 0.0; // The damping should be active (kmd > 0), but therefore velocity estimation should work better
  c.walking.r.anklex.yfoot       = 0.0;
  c.walking.r.anklex.tau_extra   = 0.0;
  c.walking.r.anklex.q_prev      = 0.0;
  c.walking.r.anklex.tau_prev    = 0.0;
}

/****************************************************************/
static inline void ymass( void ) 
{
  c.walking.hipx.ymass = LLEG*sin(s.q.r.anklex) + LHIP*cos(s.q.r.anklex) + LBODY*sin(s.q.r.anklex); // -s.q.hipx
}
/****************************************************************/
static inline void yfoot( void ) 
{
  // This initializes the state variables to calculate the trajectories and torques
  c.walking.r.anklex.yfoot   = LLEG*sin(s.q.r.anklex) + LHIP*cos(s.q.r.anklex) + LLEG*sin(2*s.q.hipx - s.q.r.anklex);
  c.walking.l.anklex.yfoot   = -(LLEG*sin(s.q.l.anklex) + LHIP*cos(s.q.l.anklex) + LLEG*sin(2*s.q.hipx - s.q.l.anklex));
}
/****************************************************************/
static inline void determine_ankle_torques( void ) 
{
  float tau_d_left = 0.0;
  float tau_d_right = 0.0;
  joint_param2_t *jl = &c.joint.l.ankley;
  joint_param2_t *jr = &c.joint.r.ankley;
  walking_con_t *p = &c.walking;

  jl->tau_d = p->r.anklex.tau_extra + \
    (XTOE/ p->r.anklex.yfoot) * (s.q.r.anklex - p->r.anklex.q_d) * p->r.anklex.kmp + \
    (XTOE/ p->r.anklex.yfoot) * (s.qd.r.anklex - p->r.anklex.qd_d) * p->r.anklex.kmd + \
    (jr->q_d - s.q.r.ankley)*jl->kpp + (jl->qd_d - s.qd.r.ankley)*jl->kdd;
  if ( jl->tau_d < 0.0 ) { jl->tau_d = 0.0; }

  jr->tau_d = 2*(jr->q_d - s.q.r.ankley)*jr->kpp + (jr->qd_d - s.qd.r.ankley)*jr->kdd - jl->tau_d;
  if ( jr->tau_d < 0.0 ) { jr->tau_d = 0.0; }

}
/****************************************************************/
void update_start_walking_controller(void)
{
  // If we are running, then compute control outputs depending upon the mode and state index.
  walking_con_t *p        = &c.walking; 
  joint_param2_t *jl      = &c.joint.l.ankley;
  joint_param2_t *jr      = &c.joint.r.ankley;

  int state_init; // True if state init is used for the first time.

  if (p->inx == START_BEGIN)
    transition( START_BEGIN_WAIT );
  
  state_init = (p->inx < 0);        // signal state initialization code
  if ( state_init ) p->inx = -p->inx;


  switch (c.walking.inx) {

  case ( START_BEGIN ):
    break;

  case ( START_BEGIN_WAIT ):
    if (state_init) {
      logprintf("Begin by waiting 2 seconds\n");
      c.LEDS &= ~LED1;
    }
    if ( elapsed() > 2.0 ) {
      transition( START_LAUNCH );
    }
    update_joint_controllers;
    break;
  
  case ( START_LAUNCH ):
    if (state_init) {
      logprintf("Begin start walking\n");

      jl->kp  = 16.0;
      jr->kp  = 16.0;
      jl->kd  = 0.1;
      jr->kd  = 0.1;
      jl->kpp = 200;
      jr->kpp = 200;
      jl->kdd = 0.5;
      jr->kdd = 0.5;

      // These values are copied from the standing state.
      c.joint.hipx.q_d     =   HIPX_POS;
      c.joint.l.knee.q_d   =   KNEE_POS-0.01;
      c.joint.l.ankley.q_d =   ANKLE_POS;  
      c.joint.l.hipy.q_d   =   HIPY_POS;
      c.joint.r.knee.q_d   =   KNEE_POS-0.01;
      c.joint.r.ankley.q_d =   ANKLE_POS;  
      c.joint.r.hipy.q_d   =   HIPY_POS;

      c.joint.l.knee.kp   = 100;
      c.joint.l.knee.kd   = 1.0;
      c.joint.l.hipy.kp   = 60;
      c.joint.l.hipy.kd   = 0.1;
      c.joint.r.knee.kp   = 100;
      c.joint.r.knee.kp   = 1.0;
      c.joint.r.hipy.kp   = 60;
      c.joint.r.hipy.kd   = 0.1;

      set_joint_PD_mode( &c.joint.hipx  );
      set_joint_PD_mode( &c.joint.l.hipy);
      set_joint_PD_mode( &c.joint.l.knee);
      set_joint_PD_mode( &c.joint.r.hipy);
      set_joint_PD_mode( &c.joint.r.knee);
      set_joint_torque_PD_mode_ankle_ff_tau( &c.joint.l.ankley );
      set_joint_torque_PD_mode_ankle_ff_tau( &c.joint.r.ankley );

      p->l.anklex.tau_prev    = jl->tau_d;
    }  
    p->r.anklex.q_d = s.q.r.anklex;

    { 
      // Make the left ankley torque by our own:
      float tau_d_left  = 0.0;
      float tau_d_right = 0.0;
      float dtau_load   = 0.0;

      jr->q_dprev = jr->q_d;

      if ( elapsed() < 1.0 ) { // Change the angles for sagittal controller of the left foot for those from the right foot 
	tau_d_left  = (1.0 - elapsed()) * (jr->q_d - s.q.l.ankley)*jl->kpp + \
	                     elapsed() * (jr->q_d - s.q.r.ankley)*jl->kpp +	\
	              (1.0 - elapsed()) * (jr->qd_d - s.qd.l.ankley)*jl->kdd + \
	                     elapsed() * (jr->qd_d - s.qd.r.ankley)*jl->kdd;
	tau_d_right = (jr->q_d - s.q.r.ankley)*jr->kpp + (jr->qd_d - s.qd.r.ankley)*jr->kdd;
      }
      else {
	tau_d_left = (jr->q_d - s.q.r.ankley)*jl->kpp + (jr->qd_d - s.qd.r.ankley)*jl->kdd + p->r.anklex.tau_extra;
	tau_d_right = (jr->q_d - s.q.r.ankley)*jr->kpp + (jr->qd_d - s.qd.r.ankley)*jr->kdd - p->r.anklex.tau_extra;
	
	if ( p->r.anklex.tau_extra < LIFT_TORQUE ) { 
	  p->r.anklex.tau_extra += 0.003;
	  jr->q_d -= 0.000004;
	}
	if ( tau_d_right < 6.0 ) {
	  jr->q_d += 0.000004;
	  p->r.anklex.tau_extra -= 0.002;

	}
	if ( jr->kpp < 350 ){
	  jr->kpp += 0.04;
	  jl->kpp -= 0.04;
	}
	if (( tau_d_right > 10.0 ) && (jr->q_d < 0.0)) {
	  jr->q_d += 0.000004;
	}
      }
      jr->qd_d = (jr->q_d - jr->q_dprev)/c.dt;

      // determine the torques that should go to the motor
      jl->tau_d = 15.0/16.0*jl->tau_dprev + 1.0/16.0*tau_d_left;
      if ( jl->tau_d < -1.0 ) { jl->tau_d = -1.0; }
      if ( jl->tau_d > LIFT_TORQUE) { jl->tau_d = LIFT_TORQUE; }

      jr->tau_d = 15.0/16.0*jr->tau_dprev + 1.0/16.0*tau_d_right;
      if ( jr->tau_d < -1.0 ) { jr->tau_d = -1.0; }
      if ( jr->tau_d > LIFT_TORQUE) { jr->tau_d = LIFT_TORQUE; }
    }

    update_joint_controllers();

    // Wait 10 second to go to the next stage.
    if ( elapsed() > 10) {
      transition( START_LAUNCH_2 );
    }
    break;

  case ( START_LAUNCH_2 ):
    if (state_init) {
      // An own torque has to be created for the ankles so disable the update_controller() for the left ankle
      logprintf("Lifting mass to one side... Press button 1 to start making steps\n");
      yfoot();

      jl->kpp               = 100;
      jr->kpp               = 460;                 // This is high to control the sagital motion at the end of the lift, but
      jr->q_d               = ANKLE_POS -0.055;    // therefore the set point must be placed more forward for the beginning of the
      jl->q_d               = ANKLE_POS -0.055;    // lift.  See thesis for more explaination.
      p->r.anklex.tau_prev  = jl->tau_d;
      p->l.anklex.tau_prev  = jr->tau_d;
      p->swing.ankley.q_d   = jr->q_d;
      p->r.anklex.q_prev    = s.q.r.anklex;
      p->l.anklex.q_prev    = s.q.r.anklex;
      p->r.anklex.q_d       = s.q.r.anklex;
      p->l.anklex.tau_extra = p->r.anklex.tau_extra;
    }

    // some seconds turbo boost to lift the weight
    if ( 0.3*elapsed() < 1.0 ) jl->turbo_boost = 1;
    else jl->turbo_boost = 0;

    p->r.anklex.tau_extra = s.q.r.anklex / p->r.anklex.q_prev * LIFT_TORQUE;
    if ( p->r.anklex.tau_extra < 0.0) p->r.anklex.tau_extra = 0.0;      
    
    // Desired sideways trajectory: let it lead the real angle by 0.05 radians.
    if ((s.q.r.anklex - p->r.anklex.q_d < 0.05) && (p->r.anklex.q_d > ANKLEX_POS )) { 
      p->r.anklex.q_d -= 0.00012;
      // Misschien moet hier nog een stukje dat de jr->q_d weer terug naar normaal brengt....
    }
    p->r.anklex.qd_d = 0.0;   // This is ugly programming, but now the velocity gain for the sideways controller is zero anyway.
    determine_ankle_torques();
    update_joint_controllers();
    
//     if (( FLAME_PUSHBUTTON_PRESSED( s.front_panel_sw, PUSHBUTTON1 ))&& ( elapsed() > 10.0 )) {
//       transition( START_STANCE_RIGHT_LEG );
//     } 
//     
//     if ( FLAME_TOGGLE_UP( s.front_panel_sw, TOGGLE1 ) && elapsed() > 10.0) {
//       transition( START_STANCE_RIGHT_LEG );
//     }
    break;

  case ( START_STANCE_RIGHT_LEG ): // THIS IS ONLY PROGRAMMED ONCE. IT DOES NOT WORK, AND DIFFERS FROM THE WALKING PROGRAM 'STYLE', for instance: The knee_inx parameter is removed and can not be used for instance. This should be fixed
    if (state_init){
      logprintf("Stance leg is RIGHT leg\n");

      c.joint.l.knee.kp   = 40;
      c.joint.l.knee.kd   = 0.2;
      c.joint.l.hipy.kp   = 80;
      c.joint.l.hipy.kd   = 1.0;
      c.joint.r.knee.kp   = 120;
      c.joint.r.knee.kd   = 4.0;
      c.joint.r.hipy.kp   = 120;
      c.joint.r.hipy.kd   = 2.0;
      c.joint.l.ankley.kp  = 4.0;
      c.joint.r.ankley.kp  = 16.0;
      c.joint.l.ankley.kd  = 0.1;
      c.joint.r.ankley.kd  = 0.1;

      c.joint.hipx.q_d           = 0.1;

      p->stance.ankley.tau_d     = 6.0;
      p->stance.ankley.tau_prev  = jr->tau_d;
      p->stance.ankley.taud_prev = jr->taud_d;
      p->swing.ankley.taud_prev  = jl->taud_d; 
      p->swing.ankley.tau_prev   = jl->tau_d;
      p->swing.ankley.tau_d      = 3.0;
      p->swing.knee.q_prev       = c.joint.l.knee.q_d;    
      p->swing.knee.q_d          = 0.18;    

      p->swing.hipy.q_prev       = c.joint.l.hipy.q_d;    
      p->swing.hipy.q_d          = c.joint.l.hipy.q_d - 0.15;
      p->swing.hipy.qd_d         = 0.0;
      p->swing.hipy.qd_prev      = c.joint.l.hipy.qd_d;

      p->stance.hipy.q_prev      = c.joint.r.hipy.q_d; 
      p->stance.hipy.q_d         = c.joint.r.hipy.q_d + c.joint.l.hipy.q_d - p->swing.hipy.q_d; 
      p->stance.hipy.qd_prev     = c.joint.r.hipy.qd_d; 
      p->stance.hipy.qd_d        = 0.0; 

      p->t1                      = 0.0;
      p->t2                      = 0.0;  
    }

    // push left ankle down a little bit to get sideways momentum
    if ( 6.0*elapsed() < 1.0 ) {
      jl->turbo_boost = 1;
      jl->tau_d = compute_quintic_spline( 6.0*elapsed(), p->swing.ankley.tau_prev, p->swing.ankley.taud_prev, 0.0, \
					  p->swing.ankley.tau_prev + T_ANKLE_BOOST2, 0.0, 0.0 );
      p->t1 =  c.t - c.walking.tstart;
    }
    else {
      jl->turbo_boost = 0;   
      // Swing left leg forward
      c.joint.l.hipy.q_dprev = c.joint.l.hipy.q_d; 
      c.joint.l.hipy.q_d = compute_quintic_spline( 1.8*(elapsed()-p->t1), p->swing.hipy.q_prev, 0.0, 0.0, \
						   p->swing.hipy.q_d, p->swing.hipy.qd_d, 0.0);
      c.joint.l.hipy.qd_d = (c.joint.l.hipy.q_d - c.joint.l.hipy.q_dprev) / c.dt;
      
      if ( p->knee_inx == 0 ) {
	c.joint.l.knee.q_dprev = c.joint.l.knee.q_d;      
	c.joint.l.knee.q_d = compute_quintic_spline( 1.8*(elapsed()-p->t1),p->swing.knee.q_prev, 0.0,0.0,\
						     p->swing.knee.q_d, 0.0,0.0);
	c.joint.l.knee.qd_d = (c.joint.l.knee.q_d - c.joint.l.knee.q_dprev) / c.dt;
	p->t2               = c.t - c.walking.tstart;
	
	if (s.q.l.hipy < s.q.r.hipy - 0.05 ) {
	  p->swing.knee.q_d     = KNEE_POS;
	  p->knee_inx           = 1;
	  p->t2                 = c.t - c.walking.tstart;
	  p->swing.knee.q_prev  = c.joint.l.knee.q_d;
	  p->swing.knee.qd_prev = c.joint.l.knee.qd_d; // track the velocity to calculate the next spline smoothly
	}    
      }
      
      else {
	c.joint.l.knee.q_dprev = c.joint.l.knee.q_d; 
	c.joint.l.knee.q_d = compute_quintic_spline( 2*(elapsed()-p->t2),p->swing.knee.q_prev, p->swing.knee.qd_prev, 0.0, \
						     p->swing.knee.q_d, 0.0, 0.0);
	c.joint.l.knee.qd_d = (c.joint.l.knee.q_d - c.joint.l.knee.q_dprev) / c.dt;
      }
      
      jl->tau_d = compute_quintic_spline( 4.0*(elapsed()-p->t1), p->swing.ankley.tau_prev + T_ANKLE_BOOST2, 0.0, 0.0, \
					  p->swing.ankley.tau_d, 0.0, 0.0 );
      jr->tau_d = compute_quintic_spline( 1.6*(elapsed()-p->t1), p->stance.ankley.tau_prev, p->stance.ankley.taud_prev, 0.0, \
					  p->stance.ankley.tau_d, 0.0, 0.0 );
      
      c.joint.r.hipy.q_dprev = c.joint.r.hipy.q_d;     
      c.joint.r.hipy.q_d = compute_quintic_spline( 1.4*(elapsed()-p->t1), p->stance.hipy.q_prev, 0.0, 0.0, \
						   p->stance.hipy.q_d, p->stance.hipy.qd_d, 0.0);
      c.joint.r.hipy.qd_d = (c.joint.r.hipy.q_d - c.joint.r.hipy.q_dprev) / c.dt;
    }

    update_joint_controllers();
    break;
  } // End of read-out case machine
}

