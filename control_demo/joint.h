// joint.h : convenient biped robot individual axis controllers

// Copyright (C) 2001-2006 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#ifndef __JOINT_H_INCLUDED__
#define __JOINT_H_INCLUDED__

#include "globals.h"

// Defines the modes for the joint controller (s.joint.*.mode variables).
#define JOINT_OFF                       0   // no joint controller
#define JOINT_LIMP                      1   // torque is zero
#define JOINT_P                         2   // P servo using motor encoders
#define JOINT_PD                        3   // PD servo using joint or motor encoders
#define JOINT_PD_FF_TAU                 4   // PD servo using joint or motor encoders and tau_desired
#define JOINT_TORQUE_PD_ANKLE           5   // PD servo ankle with a desired torque
#define JOINT_TORQUE_PD_ANKLE_FF_TAU    6   // PD servo ankle with a desired torque
#define JOINT_POSITION_PD_ANKLE         7   // PD servo ankle with a desired position
#define JOINT_POSITION_PD_ANKLE_FF_TAU  8   // PD servo ankle with a desired position
#define JOINT_QDPD                      9   // constant velocity setpoint, with PD servo, using motor encoders

// For standing these angles are desired (standing.c & start_walking.c)
#define ANKLE_POS       0.00
#define HIPY_POS        0.01
#define KNEE_POS       -0.08
// For walking the hipx position needs to be smaller than for gait initiation (I used factor 2 approx)
#define HIPX_POS        0.035//0.06

#define R2         0.0495 // distance from heel to ankle joint
//#define FILT_NUM   0.1813 // Numerator in transfer function of first order filter
//#define FILT_DEN   0.8187 // Denominator in transfer function of first order filter

extern void init_joint_controllers( void );
extern void update_joint_controllers( void );

extern void set_joint_OFF_mode( joint_param_t *j );
extern void set_joint_LIMP_mode( joint_param_t *j );
extern void set_joint_LIMP_mode_ankle( joint_param2_t *j );
extern void set_joint_P_mode(joint_param_t *j);
extern void set_joint_PD_mode(joint_param_t *j);
extern void set_joint_PD_ff_tau_mode(joint_param_t *j);
extern void set_joint_torque_PD_mode_ankle(joint_param2_t *j );
extern void set_joint_torque_PD_mode_ankle_ff_tau(joint_param2_t *j );
extern void set_joint_position_PD_mode_ankle(joint_param2_t *j );
extern void set_joint_position_PD_mode_ankle_ff_tau(joint_param2_t *j );
extern void set_joint_QDPD_mode(joint_param_t *j, float q_d, float qd_d, float qref);

#endif /*  __JOINT_H_INCLUDED__ */
