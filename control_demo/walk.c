// walk.c : Flame biped robot walk controller.
//
// This file assumes that Flame has made a first step and inmediately  
// has to walk. The easiest thing to do is to walk very fast so sideways 
// dynamics are less important. However, they can not be neglected.
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
  WALK_BEGIN = 0,
  WALK_GET_READY,
  WALK_INITIATE,
  WALK_EARLY_SWING,
  WALK_LATE_SWING,
  WALK_TOE_OFF,
  WALK_NO_STANCE_LEG,
  WALK_CRASH = 1000,
  WALK_UNKNOWN        // must be the last symbol
};
#define STANCE_LEG_RIGHT 0

#define LBODY 0.175        // length body mass above hipx: estimated
#define LHIP  0.069        // width of hip (anklex to hipx)
#define LLEG  0.647        // length total leg (anklex hipx)!
#define XTOE  0.107

#define CP     20 // number of positive counts before we are sure that that switch is on the ground
#define CPFAST 10 // when heel-strike is expected we don't have to wait long
#define CN    -50 // number of negative counts before we are sure that that switch is on the ground

#define LIFT_TORQUE       15.6*9.81/2*XTOE+2   // The torque that is needed to lift half the weight of flame.
#define T_ANKLE_BOOST     5.0
#define TIME_TOE_OFF      0.0//0.12
#define HIPY_SWING        0.5
#define LEANING           0.06
#define HIPY_TIME         2.2//2.8
#define TIME_STRETCH_KNEE 3.0
#define KNEE_SWING        0.8
#define SWING_KNEE_KP     20
#define SWING_KNEE_KD     0.6
#define TORQUE_EXTRA      0.0
#define K_ANKLEY          40.0 //150

/****************************************************************/
// Advance the state machine to a new state.  The state indices are defined in symbols.h.
inline static void transition( int newstate )
{
  if (newstate >= 0 && newstate < WALK_UNKNOWN) {
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
static inline void ymass( void ) // not really important for control
{
  c.walking.hipx.ymass = LLEG*sin(s.q.r.anklex) + LHIP*cos(s.q.r.anklex) + LBODY*sin(s.q.r.anklex); // -s.q.hipx
}
/****************************************************************/
static inline void yfoot( void ) // not important yet, maybe usefull for lateral controller
{
  // This initializes the state variables to calculate the trajectories and torques
  c.walking.r.anklex.yfoot   = LLEG*sin(s.q.r.anklex) + LHIP*cos(s.q.r.anklex) + LLEG*sin(2*s.q.hipx - s.q.r.anklex);
  c.walking.l.anklex.yfoot   = -(LLEG*sin(s.q.l.anklex) + LHIP*cos(s.q.l.anklex) + LLEG*sin(2*s.q.hipx - s.q.l.anklex));
}

/****************************************************************/
// Implement a bisecting controller.
static inline void control_torso_bisecting( joint_param_t *swinghipy, joint_param_t *stancehipy, float qswing, float qdswing, float qstance, float qdstance ) 
{
  swinghipy->tau_d = -swinghipy->kp_bi * ( qswing + qstance + LEANING ) - swinghipy->kd_bi * ( qdswing + qdstance );
  stancehipy->tau_d = -stancehipy->kp_bi * ( qswing + qstance + LEANING ) - stancehipy->kd_bi * ( qdswing + qdstance );
}

/****************************************************************/

// These pointers are set to substructures of s so that control
// can be written more abstractly in terms of swing and stance
// leg, as they swapped back and forth.

static leg_dof_t *stance_q,   *swing_q; 
static leg_dof_t *stance_qd,  *swing_qd;
static leg_tau_t *stance_tau, *swing_tau;

// Pointers for joint controller parameters.
static joint_param2_t *swing_ankley_joint;
static joint_param2_t *stance_ankley_joint;
static joint_param_t *swing_knee_joint;
static joint_param_t *stance_knee_joint;
static joint_param_t *swing_hipy_joint;
static joint_param_t *stance_hipy_joint;

// Foot switch values.
static int swing_back_switch, stance_back_switch;       
static int swing_front_switch, stance_front_switch; 

/****************************************************************/
static void set_stance_swing_pointers(void)
{
  // Select which leg is the stance leg and which is the swing
  // leg.  This makes the walking code simpler since everything
  // can be phrased in terms of "swing" and "stance" even as the
  // roles of left and right keep switching.

  if ( c.walking.stance_leg == STANCE_LEG_RIGHT ) {    
    stance_q    = &s.q.r;        // right leg stance
    stance_qd   = &s.qd.r;
    stance_tau  = &s.tau.r;

    stance_ankley_joint = &c.joint.r.ankley;
    stance_knee_joint   = &c.joint.r.knee;
    stance_hipy_joint   = &c.joint.r.hipy;

    swing_q     = &s.q.l;       // left leg swing
    swing_qd    = &s.qd.l;
    swing_tau   = &s.tau.l;

    stance_back_switch  = s.foot.r.back.count;
    stance_front_switch = s.foot.r.front.count;
    swing_back_switch   = s.foot.l.back.count;
    swing_front_switch  = s.foot.l.front.count;

    swing_ankley_joint = &c.joint.l.ankley;
    swing_knee_joint   = &c.joint.l.knee;
    swing_hipy_joint   = &c.joint.l.hipy;
   
  } else { 
    stance_q    = &s.q.l;        // left leg stance
    stance_qd   = &s.qd.l;
    stance_tau  = &s.tau.l;

    stance_ankley_joint = &c.joint.l.ankley;
    stance_knee_joint   = &c.joint.l.knee;
    stance_hipy_joint   = &c.joint.l.hipy;

    swing_q     = &s.q.r;        // right leg swing
    swing_qd    = &s.qd.r;
    swing_tau   = &s.tau.r;

    stance_back_switch  = s.foot.l.back.count;
    stance_front_switch = s.foot.l.front.count; 
    swing_back_switch   = s.foot.r.back.count;
    swing_front_switch  = s.foot.r.front.count;

    swing_ankley_joint = &c.joint.r.ankley;
    swing_knee_joint   = &c.joint.r.knee;
    swing_hipy_joint   = &c.joint.r.hipy;
  }
}
/****************************************************************/
// Create trajectory for the hips
static inline void hipy_actuation( void ) 
{  
  walking_con_t *p        = &c.walking;

  p->rel_hipangle_q   = stance_q->hipy  - swing_q->hipy;
  p->rel_hipangle_qd  = stance_qd->hipy - swing_qd->hipy;
  p->rel_hipangle_q_d = compute_quintic_spline( HIPY_TIME*(elapsed()+p->t2+p->t1), p->rel_hipangle_prev, \
						0.0, 0.0, HIPY_SWING, p->rel_hipangle_qd_d, 0.0);

  // use bisecting mechanism to determine the angle of other leg
  control_torso_bisecting( swing_hipy_joint, stance_hipy_joint, swing_q->hipy, swing_qd->hipy,\
			   stance_q->hipy, stance_qd->hipy ); 

  // Directly determine the torque to the motor drivers (s.tau.hip). This doesn't go through joint.c so the mode 
  // used for the hip joints is the OFF-mode. Joint.c does not rewrite the s.tau.hip then.
  swing_tau->hipy =  swing_hipy_joint->kp * (p->rel_hipangle_q  - p->rel_hipangle_q_d ) + \
                     swing_hipy_joint->kd * (p->rel_hipangle_qd - p->rel_hipangle_qd_d) + swing_hipy_joint->tau_d;   

  stance_tau->hipy = -swing_tau->hipy + stance_hipy_joint->tau_d;
}
/****************************************************************/
// Create trajectory for the hips
static inline void ankley_actuation( void ) 
{  
  walking_con_t *p        = &c.walking;

  // when the foot is not completely on the ground (0.3 seconds of state early swing)
  if ( p->inx == 5 ) { // This is toe off, but is actually not used (only 0.001 sec)
    stance_ankley_joint->tau_d = 4.0;
    swing_ankley_joint->tau_d  = 4.0;
  } 
  else if ( p->inx == 4 && elapsed() > 0.1 ) { // then the swing ankle must be streched a bit to land shallow
    swing_ankley_joint->q_d    = 0.1;
    stance_ankley_joint->tau_d = 4.0 - K_ANKLEY * (stance_q->ankley + 0.02);
  }  
  else if ( p->inx == 3 && elapsed() < 0.1 ) { // the swing ankle creates the ground clearance. The new stance
    // ankle just makes sure that the foot goes to the ground completely.
    swing_ankley_joint->q_d    = -0.4;
    stance_ankley_joint->tau_d = 2.5;
  }
  else { // so from inx = 3 and > 0.1 sec till inx = 4 and < 0.1 sec
    swing_ankley_joint->q_d    = -0.4;
    stance_ankley_joint->tau_d = 4.0 - K_ANKLEY * (stance_q->ankley + 0.02);
  }
  
  if ( stance_ankley_joint->tau_d < 0.0 ) stance_ankley_joint->tau_d = 0.0;
  if ( swing_ankley_joint->tau_d < 0.0 ) swing_ankley_joint->tau_d = 0.0;

}
/****************************************************************/
void update_walking_controller(void)
{
  // If we are running, then compute control outputs depending upon the mode and state index.
  walking_con_t *p        = &c.walking;
  joint_param2_t *jl      = &c.joint.l.ankley;
  joint_param2_t *jr      = &c.joint.r.ankley;

  int state_init; // True if state init is used for the first time.

  state_init = (p->inx < 0);        // signal state initialization code
  if ( state_init ) p->inx = -p->inx;

  if ( p->inx == WALK_BEGIN ) transition( WALK_GET_READY );

  // configure swing and stance for current leg modes
  set_stance_swing_pointers();


  switch (c.walking.inx) {

  case ( WALK_BEGIN ):
    break;

  case ( WALK_GET_READY ):
    if (state_init) {
      logprintf("Begin start walking, press button 2 to swing left leg\n");

      jl->kp  = 16.0;
      jr->kp  = 16.0;
      jl->kd  = 0.1;
      jr->kd  = 0.1;
      jl->kpp = 160;
      jr->kpp = 160;
      jl->kdd = 2.0;
      jr->kdd = 2.0;
      jl->tau_d = 4.0;
      jr->tau_d = 4.0;

      // These values are copied from the standing state.
      c.joint.hipx.q_d     =   HIPX_POS;
      c.joint.l.knee.q_d   =   KNEE_POS;
      c.joint.l.ankley.q_d =   ANKLE_POS;  
      c.joint.l.hipy.q_d   =   HIPY_POS;
      c.joint.r.knee.q_d   =   KNEE_POS;
      c.joint.r.ankley.q_d =   ANKLE_POS;  
      c.joint.r.hipy.q_d   =   HIPY_POS;

      c.joint.l.knee.kp     = 35;
      c.joint.r.knee.kp     = 100;
      c.joint.l.knee.kd     = 2.0;
      c.joint.r.knee.kd     = 4.0;
      c.joint.l.hipy.kp     = 80;
      c.joint.r.hipy.kp     = 0.0;
      c.joint.l.hipy.kd     = 0.2;
      c.joint.r.hipy.kd     = 0.2;
      c.joint.r.hipy.kp_bi  = 100.0;
      c.joint.r.hipy.kd_bi  = 1.0;
      c.joint.l.hipy.kp_bi  = 100.0;
      c.joint.l.hipy.kd_bi  = 1.0;

      set_joint_PD_mode( &c.joint.hipx  );
      set_joint_PD_mode( &c.joint.l.hipy);
      set_joint_PD_mode( &c.joint.l.knee);
      set_joint_PD_mode( &c.joint.r.hipy);
      set_joint_PD_mode( &c.joint.r.knee);
      set_joint_torque_PD_mode_ankle( &c.joint.l.ankley );
      set_joint_torque_PD_mode_ankle( &c.joint.r.ankley );

      p->l.anklex.tau_prev    = jl->tau_d;
      p->r.anklex.tau_extra   = 0.0;

      p->stance_leg = 0;
      p->t2         = 0.0;
    }  
    p->r.anklex.q_d = s.q.r.anklex;

    if ( stance_back_switch > CP &&
	 swing_back_switch > CP ){
      jr->tau_d = 4.0 - K_ANKLEY * s.q.r.ankley;
      jl->tau_d = 4.0 - K_ANKLEY * s.q.l.ankley;
    } else {
      jr->tau_d = 4.0;
      jl->tau_d = 4.0;
    }
    update_joint_controllers();
    if (// p->r.anklex.tau_extra > LIFT_TORQUE - 0.1 &&  commented to get immedeately to walk but with help
	// jl->tau_d > LIFT_TORQUE - 0.1 &&
	// jr->tau_d > 6.0 &&
	FLAME_PUSHBUTTON_PRESSED( s.front_panel_sw, PUSHBUTTON2 )) {
      c.LEDS &= ~LED3;
      transition( WALK_INITIATE );
    }


    break;

  case ( WALK_INITIATE ): // Bring the swing leg forward from midstance
    if ( state_init ) {
      set_joint_OFF_mode( swing_hipy_joint );
      set_joint_OFF_mode( stance_hipy_joint );

      swing_hipy_joint->kp            = 40; //90;
      swing_hipy_joint->kd            = 5.0;//3.0;
      swing_hipy_joint->turbo_boost   = 0;
      swing_hipy_joint->kp_bi         = 0.0; // no bisect controller implemented on swing leg
      swing_hipy_joint->kd_bi         = 0.0;
      stance_hipy_joint->kp           = 0.0; // The bisecting mechanism will do the controlling
      stance_hipy_joint->kd           = 0.0; // 4.0;
      stance_hipy_joint->turbo_boost  = 1;
      stance_hipy_joint->kp_bi        = 150.0;
      stance_hipy_joint->kd_bi        = 4.0;
      p->rel_hipangle_prev       = stance_q->hipy - swing_q->hipy;
      p->rel_hipangle_qd_d       = 0.0;
      p->t1                      = 0.0;
      p->t2                      = 0.0;
      p->swing.knee.q_prev       = swing_knee_joint->q_d;    
      p->swing.knee.q_d          = 0.5*KNEE_SWING;    
      p->swing.knee.qd_prev      = swing_knee_joint->qd_d;
    }

    if ( stance_front_switch < CN &&
	 stance_back_switch < CN &&
	 swing_front_switch < CN &&
	 swing_back_switch < CN ) {
      transition( WALK_NO_STANCE_LEG );
    }

    // the control bisecting can not be implemented as LEANING will applied at once. This is not good. Therefore
    // hiypaction() can not be used.
    p->rel_hipangle_q   = stance_q->hipy  - swing_q->hipy;
    p->rel_hipangle_qd  = stance_qd->hipy - swing_qd->hipy;
    p->rel_hipangle_q_d = compute_quintic_spline( HIPY_TIME*(elapsed()+p->t2+p->t1), p->rel_hipangle_prev, \
						  0.0, 0.0, HIPY_SWING, p->rel_hipangle_qd_d, 0.0);
    
    // use bisecting controller, but build up the leaning angle gently.
    if (elapsed() < 1.0) 
      stance_hipy_joint->tau_d = -stance_hipy_joint->kp_bi * ( swing_q->hipy + stance_q->hipy + elapsed()*LEANING ) - \
                                  stance_hipy_joint->kd_bi * ( swing_qd->hipy + stance_qd->hipy );
    else
      stance_hipy_joint->tau_d = -stance_hipy_joint->kp_bi * ( swing_q->hipy + stance_q->hipy + LEANING ) - \
	                          stance_hipy_joint->kd_bi * ( swing_qd->hipy + stance_qd->hipy );
    
    // Send out torques:
    swing_tau->hipy =  swing_hipy_joint->kp * (p->rel_hipangle_q  - p->rel_hipangle_q_d ) + \
                       swing_hipy_joint->kd * (p->rel_hipangle_qd - p->rel_hipangle_qd_d) + swing_hipy_joint->tau_d;   
    
    stance_tau->hipy = -swing_tau->hipy + stance_hipy_joint->tau_d;
    
    // Lower leg --- first swing lower leg backwards, by the natural dynamics, then stretch by increasing the stiffness.
    if (elapsed() < 1.0) {
      swing_knee_joint->kp = elapsed()*SWING_KNEE_KP;
      swing_knee_joint->kd = elapsed()*SWING_KNEE_KD;
    }    
    else {
      swing_knee_joint->kp = SWING_KNEE_KP;
      swing_knee_joint->kd = SWING_KNEE_KD;
    }

    // And prepare the ankle for heel strike
    swing_ankley_joint->tau_d = 5.5;

    update_joint_controllers();

    //    if ( 3.0*elapsed() > 1.0 ) transition( WALK_EARLY_SWING );
    if ( swing_back_switch > CPFAST ) transition( WALK_TOE_OFF );
    break;

  case ( WALK_EARLY_SWING ):
    if (state_init){
      logprintf("Entering SWING, stance_leg = %s\n", (p->stance_leg == STANCE_LEG_RIGHT) ? "RIGHT" : "LEFT" );
      
      set_joint_OFF_mode( swing_hipy_joint );
      set_joint_PD_mode( &c.joint.l.knee   );
      set_joint_position_PD_mode_ankle( swing_ankley_joint );
      set_joint_OFF_mode( stance_hipy_joint );
      set_joint_PD_mode( &c.joint.r.knee   );
      set_joint_torque_PD_mode_ankle( stance_ankley_joint ); 
      
      swing_knee_joint->kp            = SWING_KNEE_KP;
      swing_knee_joint->kd            = SWING_KNEE_KD;
      stance_knee_joint->kp           = 70;
      stance_knee_joint->kd           = 3.0;

      swing_hipy_joint->kp            = 40; //90;
      swing_hipy_joint->kd            = 5.0;//3.0;
      swing_hipy_joint->turbo_boost   = 0;
      swing_hipy_joint->kp_bi         = 0.0; // no bisect controller implemented on swing leg
      swing_hipy_joint->kd_bi         = 0.0;
      stance_hipy_joint->kp           = 0.0; // The bisecting mechanism will do the controlling
      stance_hipy_joint->kd           = 0.0; // 4.0;
      stance_hipy_joint->turbo_boost  = 1;
      stance_hipy_joint->kp_bi        = 150.0;
      stance_hipy_joint->kd_bi        = 4.0;

      swing_ankley_joint->kpp         = 120.0;
      stance_ankley_joint->kpp        = 120.0;
      swing_ankley_joint->kp          = 4.0;
      stance_ankley_joint->kp         = 16.0;
      swing_ankley_joint->kd          = 0.1;
      stance_ankley_joint->kd         = 0.1;
      swing_ankley_joint->turbo_boost = 0;
      stance_ankley_joint->turbo_boost = 1; 
      
      p->stance.ankley.tau_prev  = stance_ankley_joint->tau_d;
      p->stance.ankley.taud_prev = stance_ankley_joint->taud_d;
      p->swing.ankley.taud_prev  = swing_ankley_joint->taud_d; 
      p->swing.ankley.tau_prev   = swing_ankley_joint->tau_d;
      
      p->stance.ankley.tau_d     = 2.0;//1.0;
      p->swing.ankley.tau_d      = 4.0;
      p->swing.knee.q_prev       = swing_knee_joint->q_d;    
      p->swing.knee.q_d          = KNEE_SWING;    
      p->swing.knee.qd_prev      = swing_knee_joint->qd_d;

      p->swing.hipy.q_prev       = swing_hipy_joint->q_d;    
      p->swing.hipy.q_d          = -HIPY_SWING;
      p->swing.hipy.qd_d         = 0.0;
      p->swing.hipy.qd_prev      = -0.0;// This is to make trajectory of swing hyperbolic instead of 
      // sine like. Therefore after heel-strike, the leg doesn;t wait till move, it wants to go immedeately 
      // towards the end goal. Using swing_hipy_joint->qd_d; would give +/- 0.0.
      p->rel_hipangle_prev       = stance_q->hipy - swing_q->hipy;
      p->rel_hipangle_qd_d       = 0.0;

      p->stance.hipy.q_prev      = stance_hipy_joint->q_d; 
      p->stance.hipy.q_d         = -p->swing.hipy.q_d; 
      p->stance.hipy.qd_prev     = stance_hipy_joint->qd_d; 
      p->stance.hipy.qd_d        = 0.0; // swing leg retraction..

      p->t1                      = 0.0;
    }

    // swing hip + bisecting mechanism
    hipy_actuation();

    // Lower leg --- first swing lower leg backwards
    swing_knee_joint->q_dprev = swing_knee_joint->q_d;      
    swing_knee_joint->q_d = compute_quintic_spline( 2.2*elapsed(),p->swing.knee.q_prev, p->swing.knee.qd_prev, 0.0, \
						    p->swing.knee.q_d, 0.0, 0.0);
   if ( swing_knee_joint->q_d > KNEE_SWING ) swing_knee_joint->q_d = KNEE_SWING; // to prevent overshoot
    swing_knee_joint->qd_d = (swing_knee_joint->q_d - swing_knee_joint->q_dprev) / c.dt;
    
    // control the ankles
    ankley_actuation();
    
    update_joint_controllers();

    // when the swing hipy is beyond a certain angle, transition to late_swing
    if (swing_q->hipy < (-0.05-LEANING)){
      // track time so that in late_swing the hipy can follow its original trajectory started in early swing.
      p->t1 = c.t - c.walking.tstart;
      transition( WALK_LATE_SWING );
    }  
    
    // when picked up, swing legs to zero position again
    if ( stance_front_switch < CN &&
	 stance_back_switch < CN &&
	 swing_front_switch < CN &&
	 swing_back_switch < CN ) {
      transition( WALK_NO_STANCE_LEG );
    }

    break;

  case ( WALK_LATE_SWING ):
    if (state_init) {
      p->swing.knee.q_d      = KNEE_POS;
      p->stance.ankley.tau_d = 4.0;
      p->swing.ankley.tau_d  = 4.0;
      p->swing.knee.q_prev   = swing_knee_joint->q_d;
      p->swing.knee.qd_prev  = swing_knee_joint->qd_d; // track the velocity to calculate the next spline smoothly
    } 

    // swing hip + bisecting mechanism
    hipy_actuation();

    // start extending the lower leg
    swing_knee_joint->q_dprev = swing_knee_joint->q_d; 
    swing_knee_joint->q_d = compute_quintic_spline( TIME_STRETCH_KNEE*elapsed(),p->swing.knee.q_prev, 0.0, 0.0, \
						    p->swing.knee.q_d, 0.0, 0.0);
    if ( swing_knee_joint->q_d < KNEE_POS ) swing_knee_joint->q_d = KNEE_POS; // to prevent overshoot
    swing_knee_joint->qd_d = (swing_knee_joint->q_d - swing_knee_joint->q_dprev) / c.dt;

    // control the ankles
    ankley_actuation();

    update_joint_controllers();

    // when picked up, swing legs to zero position again
    if ( stance_front_switch < CN &&
	 stance_back_switch < CN &&
	 swing_front_switch < CN &&
	 swing_back_switch < CN ) {
      transition( WALK_NO_STANCE_LEG );
    }
    // When swing.back switch is down, then TOE-OFF.
    if ( swing_back_switch > CPFAST ) transition( WALK_TOE_OFF );

    break;

  case ( WALK_TOE_OFF ):
    if (state_init) {
      set_joint_torque_PD_mode_ankle( swing_ankley_joint );

      p->stance.ankley.tau_prev  = stance_ankley_joint->tau_d;
      p->stance.ankley.taud_prev = stance_ankley_joint->taud_d;
      p->swing.ankley.taud_prev  = swing_ankley_joint->taud_d; 
      p->swing.ankley.tau_prev   = swing_ankley_joint->tau_d;

      // stance leg will become air-born so knee stiffness of stance leg must go down and vice versa.
      stance_knee_joint->kp           = SWING_KNEE_KP;
      stance_knee_joint->kd           = SWING_KNEE_KD;
      swing_knee_joint->kp            = 80;
      swing_knee_joint->kd            = 2.0;
    
      swing_ankley_joint->kp          = 4.0;
      stance_ankley_joint->kp         = 16.0;
      swing_ankley_joint->turbo_boost = 0;
  
      p->swing.ankley.tau_d           = 4.0;

      p->stance.knee.q_prev           = KNEE_POS;    
      p->stance.knee.q_d              = KNEE_SWING;
      p->stance.knee.qd_prev          = 0.0;
      p->swing.hipy.q_prev            = swing_hipy_joint->q_d;    
      p->swing.hipy.q_d               = -HIPY_SWING;
      p->swing.hipy.qd_prev           = 0.0; //swing_hipy_joint->qd_d;
      p->stance.hipy.q_prev           = stance_hipy_joint->q_d; 
      p->stance.hipy.q_d              = -p->swing.hipy.q_d; 
      p->stance.hipy.qd_prev          = 0.0; //stance_hipy_joint->qd_d; 
      p->t1                           = 0.0;
      p->t2                           = 0.0;
    }

    // swing hip + bisecting mechanism
    hipy_actuation();

    // control the ankles
    ankley_actuation();

    // To hold the same trajectory of the hipy, SWING phase must base trajectory on what already was changed 
    // for q_d during this toe-off   
    // p->t2 =  c.t - p->tstart;


    // hold knees in same position
    swing_knee_joint->q_dprev = swing_knee_joint->q_d; 
    swing_knee_joint->q_d = KNEE_POS;
    swing_knee_joint->qd_d = (swing_knee_joint->q_d - swing_knee_joint->q_dprev) / c.dt;
    
    stance_knee_joint->q_dprev = stance_knee_joint->q_d;      
    stance_knee_joint->q_d = KNEE_POS;//compute_quintic_spline( 1.6*elapsed(),p->stance.knee.q_prev, 0.0,0.0,p->stance.knee.q_d, 0.0,0.0);
    stance_knee_joint->qd_d = (stance_knee_joint->q_d - stance_knee_joint->q_dprev) / c.dt;
    

    update_joint_controllers();
    
    // When stance foot is not on the ground anymore, switch stance leg and go to swing
    if ( elapsed() > TIME_TOE_OFF ) {
      p->stance_leg = 1 - p->stance_leg;  // switch roles of stance and swing legs
      transition( WALK_EARLY_SWING );
    }

    // when picked up, swing legs to zero position again
    if ( stance_front_switch < CN &&
	 stance_back_switch < CN &&
	 swing_front_switch < CN &&
	 swing_back_switch < CN ) {
      transition( WALK_NO_STANCE_LEG );
    }
    break;
    
  case ( WALK_NO_STANCE_LEG ):
   if (state_init) { 
     logprintf("I'm hanging in the air\n");
     p->swing.knee.q_prev  = swing_knee_joint->q_d;
     p->swing.knee.q_d     = KNEE_POS;    
     p->stance.knee.q_prev = stance_knee_joint->q_d;
     p->stance.knee.q_d    = KNEE_POS;
     p->swing.hipy.q_prev  = swing_hipy_joint->q_d;
     p->swing.hipy.q_d     = HIPY_POS;    
     p->stance.hipy.q_prev = stance_hipy_joint->q_d;
     p->stance.hipy.q_d    = HIPY_POS;
     
     swing_knee_joint->kp   = 20;
     stance_knee_joint->kp  = 20;
     swing_knee_joint->kd   = 1.0;
     stance_knee_joint->kd  = 1.0;

     swing_hipy_joint->kp   = 10.0; // 30;
     stance_hipy_joint->kp  = 10.0; // bisecting... // 30;
     swing_hipy_joint->kd   = 0.1; // 0.2;
     stance_hipy_joint->kd  = 0.1; // 0.2;

     swing_hipy_joint->tau_d  = 0.0; // 30;
     stance_hipy_joint->tau_d = 0.0; // bisecting... // 30;

     
     // Position controller
     jl->kp  = 4.0;
     jr->kp  = 4.0;
     jl->kd  = 0.1;
     jr->kd  = 0.1;
     jl->q_d = 0.0;
     jr->q_d = 0.0;
     swing_ankley_joint->kpp         = 100.0;
     stance_ankley_joint->kpp        = 100.0;
     swing_ankley_joint->turbo_boost = 0;
     stance_ankley_joint->turbo_boost = 0;

     set_joint_PD_mode( &c.joint.l.hipy );
     set_joint_PD_mode( &c.joint.r.hipy );
     set_joint_position_PD_mode_ankle( &c.joint.l.ankley );
     set_joint_position_PD_mode_ankle( &c.joint.r.ankley );
    
     c.LEDS |= LED3;
   }

    // hipx trajectory
    c.joint.hipx.q_d = HIPX_POS;

    // knee trajectory
    swing_knee_joint->q_dprev = swing_knee_joint->q_d; 
    swing_knee_joint->q_d = compute_quintic_spline( 2*elapsed(),p->swing.knee.q_prev, 0.0, 0.0, \
						 KNEE_POS, 0.0, 0.0);
    swing_knee_joint->qd_d = (swing_knee_joint->q_d - swing_knee_joint->q_dprev) / c.dt;
    
    stance_knee_joint->q_dprev = stance_knee_joint->q_d; 
    stance_knee_joint->q_d = compute_quintic_spline( 2*elapsed(),p->stance.knee.q_prev, 0.0, 0.0, \
						 KNEE_POS, 0.0, 0.0);
    stance_knee_joint->qd_d = (stance_knee_joint->q_d - stance_knee_joint->q_dprev) / c.dt;
    
    // hipy 
    swing_hipy_joint->q_dprev = swing_hipy_joint->q_d; 
    swing_hipy_joint->q_d = compute_quintic_spline( 2*elapsed(),p->swing.hipy.q_prev, 0.0, 0.0, \
						 HIPY_POS, 0.0, 0.0);
    swing_hipy_joint->qd_d = (swing_hipy_joint->q_d - swing_hipy_joint->q_dprev) / c.dt;

    stance_hipy_joint->q_dprev = stance_hipy_joint->q_d; 
    stance_hipy_joint->q_d = compute_quintic_spline( 2*elapsed(),p->stance.hipy.q_prev, 0.0, 0.0, \
						 HIPY_POS, 0.0, 0.0);
    stance_hipy_joint->qd_d = (stance_hipy_joint->q_d - stance_hipy_joint->q_dprev) / c.dt;
   
    update_joint_controllers();

  // When both feet are down 
    if (( s.foot.r.back.count > CP || s.foot.r.front.count > CP ) &&
        ( s.foot.l.back.count > CP || s.foot.l.front.count > CP ))    
      transition( WALK_GET_READY );
    break;
    
  case ( WALK_CRASH ):
   if (state_init) { 
     logprintf("CRASH!!\n");
   }
    s.powered = 0;
    break;
  } // End of read-out case machine
}

