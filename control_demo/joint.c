// joint.c : convenient biped robot individual axis controllers

// Copyright (C) 2001-2006 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#include <math.h>
#include "joint.h"

#define RADIAN(d) ((d)*(M_PI/180.0)) 

/****************************************************************/
// initialize this controller state
void init_joint_controllers( void )
{
  c.joint.hipx.mode      = 0;
  c.joint.hipx.q_d       = 0.0;
  c.joint.hipx.q_dprev   = 0.0;
  c.joint.hipx.tau_d     = 0.0;
  c.joint.hipx.tau_dprev = 0.0;
  c.joint.hipx.taud_d    = 0.0;
  c.joint.hipx.qd_d      = 0.0;
  c.joint.hipx.kp        = 400.0;
  c.joint.hipx.kd        = 2.0;
  c.joint.hipx.qref      = 0.0;
  c.joint.hipx.data      = 0.0;
  c.joint.hipx.kp_bi     = 0.0;// This is an artifact of makevars.scm, where this param is created. This param is obsolete
  c.joint.hipx.kd_bi     = 0.0;// This is an artifact of makevars.scm, where this param is created. This param is obsolete
  c.joint.hipx.turbo_boost = 0;

  c.joint.l.hipy.mode         = 0;
  c.joint.l.hipy.q_d          = 0.0;
  c.joint.l.hipy.q_dprev      = 0.0;
  c.joint.l.hipy.tau_d        = 0.0;
  c.joint.l.hipy.tau_dprev    = 0.0;
  c.joint.l.hipy.taud_d       = 0.0;
  c.joint.l.hipy.qd_d         = 0.0;
  c.joint.l.hipy.kp           = 40.0;
  c.joint.l.hipy.kd           = 0.05;
  c.joint.l.hipy.qref         = 0.0;
  c.joint.l.hipy.data         = 0.0;
  c.joint.l.hipy.kp_bi        = 50.0;
  c.joint.l.hipy.kd_bi        = 1.0;
  c.joint.l.hipy.turbo_boost  = 0;

  c.joint.l.knee.mode      = 0;
  c.joint.l.knee.q_d       = 0.0;
  c.joint.l.knee.q_dprev   = 0.0;
  c.joint.l.knee.tau_d     = 0.0;
  c.joint.l.knee.tau_dprev = 0.0;
  c.joint.l.knee.taud_d    = 0.0;
  c.joint.l.knee.qd_d      = 0.0;
  c.joint.l.knee.kp        = 25.0;
  c.joint.l.knee.kd        = 0.02;
  c.joint.l.knee.qref      = 0.0;
  c.joint.l.knee.data      = 0.0;
  c.joint.l.knee.kp_bi     = 0.0;// This is an artifact of makevars.scm, where this param is created. This param is obsolete
  c.joint.l.knee.kd_bi     = 0.0;// This is an artifact of makevars.scm, where this param is created. This param is obsolete
  c.joint.l.knee.turbo_boost  = 0;

  c.joint.l.ankley.mode        = 0; 
  c.joint.l.ankley.q_d         = 0.0;
  c.joint.l.ankley.q_dprev     = 0.0;
  c.joint.l.ankley.tau_d       = 0.0;
  c.joint.l.ankley.tau_dprev   = 0.0;
  c.joint.l.ankley.taud_d      = 0.0;
  c.joint.l.ankley.qd_d        = 0.0;
  c.joint.l.ankley.kspring     = 13046.0;
  c.joint.l.ankley.kp          = 16.0;
  c.joint.l.ankley.kd          = 0.1;
  c.joint.l.ankley.kpp         = 200.0;
  c.joint.l.ankley.kdd         = 1.0;
  c.joint.l.ankley.qref        = 0.0;
  c.joint.l.ankley.tau_load    = 0.0;
  c.joint.l.ankley.turbo_boost = 0;

  c.joint.r.hipy.mode      = 0;
  c.joint.r.hipy.q_d       = 0.0;
  c.joint.r.hipy.q_dprev   = 0.0;
  c.joint.r.hipy.tau_d     = 0.0;
  c.joint.r.hipy.tau_dprev = 0.0;
  c.joint.r.hipy.taud_d    = 0.0;
  c.joint.r.hipy.qd_d      = 0.0;
  c.joint.r.hipy.kp        = 40.0;
  c.joint.r.hipy.kd        = 0.05;
  c.joint.r.hipy.qref      = 0.0;
  c.joint.r.hipy.data      = 0.0;
  c.joint.r.hipy.kp_bi     = 50.0;
  c.joint.r.hipy.kd_bi     = 1.0;
  c.joint.r.hipy.turbo_boost  = 0;

  c.joint.r.knee.mode      = 0;
  c.joint.r.knee.q_d       = 0.0;
  c.joint.r.knee.q_dprev   = 0.0;
  c.joint.r.knee.tau_d     = 0.0;
  c.joint.r.knee.tau_dprev = 0.0;
  c.joint.r.knee.taud_d    = 0.0;
  c.joint.r.knee.qd_d      = 0.0;
  c.joint.r.knee.kp        = 25.0;
  c.joint.r.knee.kd        = 0.02;
  c.joint.r.knee.qref      = 0.0;
  c.joint.r.knee.data      = 0.0;
  c.joint.r.knee.kp_bi     = 0.0;// This is an artifact of makevars.scm, where this param is created. This param is obsolete
  c.joint.r.knee.kd_bi     = 0.0;// This is an artifact of makevars.scm, where this param is created. This param is obsolete
  c.joint.r.knee.turbo_boost  = 0;

  c.joint.r.ankley.mode        = 0;
  c.joint.r.ankley.q_d         = 0.0;
  c.joint.r.ankley.q_dprev     = 0.0;
  c.joint.r.ankley.tau_d       = 0.0;
  c.joint.r.ankley.tau_dprev   = 0.0;
  c.joint.r.ankley.taud_d      = 0.0;
  c.joint.r.ankley.qd_d        = 0.0;
  c.joint.r.ankley.kspring     = 12445.0;
  c.joint.r.ankley.kp          = 16.0;
  c.joint.r.ankley.kd          = 0.1;
  c.joint.r.ankley.kpp         = 200.0;
  c.joint.r.ankley.kdd         = 1.0;
  c.joint.r.ankley.qref        = 0.0;
  c.joint.r.ankley.tau_load    = 0.0;
  c.joint.r.ankley.turbo_boost = 0;
}

/****************************************************************/
// Perform a control action on a joint, depending on the mode.
inline void
compute_joint_torque( joint_param_t *j, float *tau, float dt, float q, float qd )
{
  switch( j->mode ) {
  case JOINT_OFF: // no action
    return;

  case JOINT_LIMP: // limp
    *tau = 0.0;
    // let the reference position track the actual position in case the mode switches
    j->qref = q;
    return;

  case JOINT_P: // P servo around the desired position.
    *tau = ( j->kp * (j->q_d - q));
    break;

  case JOINT_PD: // PD servo around the desired position.
    *tau =  j->kp * (j->q_d - q) + j->kd * (j->qd_d - qd );
    break;

  case JOINT_QDPD:
    // A constant-velocity control function for each joint.  This
    // moves a reference position at fixed speed to the goal.
    {
      float referror  = j->q_d - j->qref;    // error in the reference position
      float dqref     = dt * fabs(j->qd_d);  // incremental distance that reference can move

      // move the reference position if needed
      if (fabs(referror) < dqref) j->qref  = j->q_d;  // check for final state
      else if (referror > 0)      j->qref += dqref;
      else                        j->qref -= dqref;
    }
    // PD servo around the reference position
    *tau =  ( j->kp * (j->qref - q)) + j->kd *(j->qd_d - qd);// + j->ff_tau;
    return;

  case JOINT_PD_FF_TAU: // PD servo around the desired position.
    *tau =  j->kp * (j->q_d - q) + j->kd * (j->qd_d - qd ) + j->tau_d;
    break;
  }
}

/****************************************************************/
// Perform a control action on the series elastic joint, depending on the mode.
inline void
compute_joint_torque_sea( joint_param2_t *j, float *tau, float dt, float qjoint, float qdjoint, float qmot, float qdmot )
{
  switch( j->mode ) {
  case JOINT_OFF: // no action
    return;

  case JOINT_LIMP: // limp
    *tau = 0.0;
    return;

  case JOINT_TORQUE_PD_ANKLE: // PD servo around the desired torque.
    { 
      float dtau_load = 0.0;
      j->tau_load = j->kspring*R2*R2*(qmot - qjoint);
      dtau_load = j->kspring*R2*R2*(qdmot - qdjoint);
      j->taud_d = (j->tau_d - j->tau_dprev)/dt;
      *tau = (j->tau_d - j->tau_load)*j->kp + (j->taud_d - dtau_load)*j->kd;
    }
    
    j->tau_dprev = j->tau_d;
    return;

  case JOINT_TORQUE_PD_ANKLE_FF_TAU: // PD servo around the desired torque.
    { 
      float dtau_load = 0.0;
      j->tau_load = j->kspring*R2*R2*(qmot - qjoint);
      dtau_load = j->kspring*R2*R2*(qdmot - qdjoint);
      j->taud_d = (j->tau_d - j->tau_dprev)/dt;
      *tau = j->tau_d + (j->tau_d - j->tau_load)*j->kp + (j->taud_d - dtau_load)*j->kd;
    }
    
    j->tau_dprev = j->tau_d;
    return;

  case JOINT_POSITION_PD_ANKLE: // PD servo around the desired position.
    { 
      // The raw desired torque from the position controller:
      float tau_d_raw = (j->q_d - qjoint)*j->kpp + (j->qd_d - qdjoint)*j->kdd;
      float dtau_load = 0.0;
      j->tau_load = j->kspring*R2*R2*(qmot - qjoint);
      dtau_load = j->kspring*R2*R2*(qdmot - qdjoint);
        
      // first of all write the previous tau_d to taud_prev      
      j->tau_dprev = j->tau_d;
      // Now we have to filter the desired torque to a smooth signal, so velocity estimation is better.
      // This is done with a first order transfer function which is determined with the aid of a Matlab file; 
      // tau_des_filter_sea.m. The velocity estimation stays noisy, but a higher order transfer function can also 
      // be implemented.
      j->tau_d = 15.0/16.0*j->tau_dprev + 1.0/16.0*tau_d_raw;
      // The desired torque can never be lower than -1 / 0 because the spring will hang lose and it will think 
      // that no motor torque is required.
      if ( j->tau_d < -1.0 ) { j->tau_d = -1.0; }
      
      j->taud_d = (-1.0/16.0*j->tau_dprev + 1.0/16.0*j->tau_d)/dt;
      // These two (desired torque and desired torq velocity) go into torque control:
      *tau = (j->tau_d - j->tau_load)*j->kp + (j->taud_d - dtau_load)*j->kd;
    }

  case JOINT_POSITION_PD_ANKLE_FF_TAU: // PD servo around the desired position.
    { 
      float tau_d_raw = (j->q_d - qjoint)*j->kpp + (j->qd_d - qdjoint)*j->kdd;
      float dtau_load = 0.0;
      j->tau_load = j->kspring*R2*R2*(qmot - qjoint);
      dtau_load = j->kspring*R2*R2*(qdmot - qdjoint);
      j->tau_dprev = j->tau_d;
      j->tau_d = 15.0/16.0*j->tau_dprev + 1.0/16.0*tau_d_raw;
      if ( j->tau_d < -1.0 ) { j->tau_d = -1.0; }
      j->taud_d = (-1.0/16.0*j->tau_dprev + 1.0/16.0*j->tau_d)/dt;

      *tau = j->tau_d + (j->tau_d - j->tau_load)*j->kp + (j->taud_d - dtau_load)*j->kd;
    }
    return;
  }
}

/****************************************************************/
// control polling function
void update_joint_controllers( void )
{
  // For each joint, take action depending on mode.
  compute_joint_torque( &c.joint.hipx,     &s.tau.hipx,      c.dt, s.q.hipx,      s.qd.hipx      );
  compute_joint_torque( &c.joint.l.hipy,   &s.tau.l.hipy,    c.dt, s.q.l.hipy,    s.qd.l.hipy    );
  compute_joint_torque( &c.joint.r.hipy,   &s.tau.r.hipy,    c.dt, s.q.r.hipy,    s.qd.r.hipy    );
  compute_joint_torque( &c.joint.l.knee,   &s.tau.l.knee,    c.dt, s.q.l.knee,    s.qd.l.knee    );
  compute_joint_torque( &c.joint.r.knee,   &s.tau.r.knee,    c.dt, s.q.r.knee,    s.qd.r.knee    );
  compute_joint_torque_sea( &c.joint.l.ankley, &s.tau.l.ankley, c.dt, \
			     s.q.l.ankley,      s.qd.l.ankley,  s.q.l.ankleymot, s.qd.l.ankleymot );
  compute_joint_torque_sea( &c.joint.r.ankley, &s.tau.r.ankley,c.dt, \
			     s.q.r.ankley,      s.qd.r.ankley,  s.q.r.ankleymot, s.qd.r.ankleymot );
}

/****************************************************************/
// control polling function
void update_joint_controllers_exercise( void )
{
  // This is special for finding the index pulses which can 
  // only be done based on the data of the motor angles, not 
  // on the joint angles as they will jump when an index pulse 
  // is found.

  compute_joint_torque( &c.joint.hipx,     &s.tau.hipx,      c.dt, s.q.hipx,         s.qd.hipx         );
  compute_joint_torque( &c.joint.l.hipy,   &s.tau.l.hipy,    c.dt, s.q.l.hipymot,    s.qd.l.hipymot    );
  compute_joint_torque( &c.joint.r.hipy,   &s.tau.r.hipy,    c.dt, s.q.r.hipymot,    s.qd.r.hipymot    );
  compute_joint_torque( &c.joint.l.knee,   &s.tau.l.knee,    c.dt, s.q.l.kneemot,    s.qd.l.kneemot    );
  compute_joint_torque( &c.joint.r.knee,   &s.tau.r.knee,    c.dt, s.q.r.kneemot,    s.qd.r.kneemot    );
  compute_joint_torque_sea( &c.joint.l.ankley, &s.tau.l.ankley,   c.dt, \
			    s.q.l.ankleymot,    s.qd.l.ankleymot, s.q.l.ankleymot,  s.qd.l.ankleymot );
  compute_joint_torque_sea( &c.joint.r.ankley, &s.tau.r.ankley,   c.dt, \
			    s.q.r.ankleymot,    s.qd.r.ankleymot, s.q.r.ankleymot,  s.qd.r.ankleymot );
}
/****************************************************************/
void set_joint_OFF_mode( joint_param_t *j )
{
  j->mode = JOINT_OFF;
  j->kp = 0.0;        // reset everything to help detect bugs
  j->kd = 0.0;
  j->qref = 0.0;
  j->q_d = 0.0;
  j->qd_d = 0.0;
}
void set_joint_OFF_mode_ankle( joint_param2_t *j )
{
  j->mode = JOINT_OFF;
}
void set_joint_LIMP_mode( joint_param_t *j )
{
  j->mode = JOINT_LIMP;
}
void set_joint_LIMP_mode_ankle( joint_param2_t *j )
{
  j->mode = JOINT_LIMP;
}

void set_joint_P_mode(joint_param_t *j)
{
  j->mode = JOINT_P;
}

void set_joint_PD_mode(joint_param_t *j)
{
  j->mode = JOINT_PD;
}

void set_joint_PD_ff_tau_mode(joint_param_t *j)
{
  j->mode = JOINT_PD_FF_TAU;
}

void set_joint_torque_PD_mode_ankle(joint_param2_t *j )
{
  j->mode = JOINT_TORQUE_PD_ANKLE;
}
void set_joint_torque_PD_mode_ankle_ff_tau(joint_param2_t *j )
{
  j->mode = JOINT_TORQUE_PD_ANKLE_FF_TAU;
}
void set_joint_position_PD_mode_ankle(joint_param2_t *j) 
{
  j->mode = JOINT_POSITION_PD_ANKLE;
}
void set_joint_position_PD_mode_ankle_ff_tau(joint_param2_t *j) 
{
  j->mode = JOINT_POSITION_PD_ANKLE_FF_TAU;
}
void set_joint_QDPD_mode(joint_param_t *j, float q_d, float qd_d, float qref) // this is only used in standing.c to go to 
// the desired position. This can be replaced by a PD controller with position- or torque trajectory before removing this.
{
  j->mode = JOINT_QDPD;
  j->q_d = q_d;
  j->qd_d = qd_d;
  j->qref = qref;
}

