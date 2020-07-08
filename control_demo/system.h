/* GENERATED FILE - DO NOT EDIT */
/* This file contains data structure declarations created by makevars.scm */

#ifndef __SYSTEM_H_INCLUDED__
#define __SYSTEM_H_INCLUDED__

#include <utility/system_state_var.h>

typedef struct {
  float hipy;                      // hip pitch joint angle wrt body (around y axis), in radians
  float knee;                      // knee joint angle wrt body (around y axis), in radians
  float ankley;                    // ankle pitch joint angle wrt body (around y axis), in radians
} leg_gains_t;

typedef struct {
  float hipx;                      // hip roll joint angle wrt body (around x axis), in radians
  leg_gains_t l;                   // left leg
  leg_gains_t r;                   // right leg
} flame_gains_t;

typedef struct {
  float sensor_processing;         // duration of the sensor processing on the previous cycle
  float total_cycle;               // duration of the previous complete processing cycle
} timing_data_t;

typedef struct {
  int inx;                         // state machine index
  float tstart;                    // start time for the state
  float amplitude;                 // sine wave amplitude
  float frequency;                 // sine wave frequency
} test_controller_state;

typedef struct {
  int inx;                         // state machine index
  float tstart;                    // start time for the state
} standing_con_t;

typedef struct {
  int inx;                         // state machine index
  float tstart;                    // start time for the state
} exercise_con_t;

typedef struct {
  int mode;                        // joint control mode
  float q_d;                       // desired position
  float q_dprev;                   // previous desired position
  float tau_d;                     // desired torque
  float tau_dprev;                 // previous desired torque
  float taud_d;                    // derivative of desired torque
  float qd_d;                      // desired velocity
  float kp;                        // proportional gain
  float kd;                        // damping
  float qref;                      // reference trajectory position
  float data;                      // empty slot for any data item
  float kp_bi;                     // stiffness for bisecting mechanism in hip
  float kd_bi;                     // damping for bisecting mechanism in hip
  int turbo_boost;                 // maximum amps boost
} joint_param_t;

typedef struct {
  int mode;                        // joint control mode
  float q_d;                       // desired position
  float q_dprev;                   // previous desired position
  float tau_d;                     // desired torque
  float tau_dprev;                 // previous desired torque
  float taud_d;                    // derivative of desired torque
  float qd_d;                      // desired velocity
  float kspring;                   // spring stiffness
  float kp;                        // proportional gain
  float kd;                        // damping
  float kpp;                       // stiffness of position controller
  float kdd;                       // damping of position controller
  float qref;                      // reference trajectory position
  float tau_load;                  // calculated torque present in ankle by cable
  int turbo_boost;                 // Boost to overwrite the maximum output torque
} joint_param2_t;

typedef struct {
  joint_param_t hipy;              // gains and desired positions
  joint_param_t knee;              // gains and desired positions
  joint_param2_t ankley;           // gains and desired positions for ankles
} joint_leg_t;

typedef struct {
  joint_param_t hipx;              // gains and desired positions
  joint_leg_t l;                   // gains and desired positions
  joint_leg_t r;                   // gains and desired positions
} joint_con_t;

typedef struct {
  float q_d;                       // lateral desired angle
  float qd_d;                      // lateral desired angular velocity
  float ymass;                     // location of the mass as seen from the foot
} walking_param_hip_t;

typedef struct {
  float q_d;                       // lateral desired angle
  float q_prev;                    // previous lateral desired angle
  float qd_d;                      // lateral desired angular velocity
  float kmp;                       // stiffness for feedback controller lateral direction
  float kmd;                       // damping for feedback controller lateral direction
  float yfoot;                     // distance to the other foot (y-direction)
  float tau_extra;                 // extra torque to get the sideways motion in action
  float tau_prev;                  // last torque in the other ankle for smooth transition
} walking_param_anklex_t;

typedef struct {
  float q_d;                       // desired angle
  float q_prev;                    // previous desired angle
  float qd_d;                      // desired angular velocity
  float qd_prev;                   // initial angle when both feet are on ground
  float tau_d;                     // desired torque
  float tau_prev;                  // last torque in the other ankle for smooth transition
  float taud_prev;                 // previous desired torque speed
} walking_param_leg_t;

typedef struct {
  walking_param_anklex_t anklex;   // walking parameters
} walking_ankle_t;

typedef struct {
  walking_param_leg_t hipy;        // gains and desired positions
  walking_param_leg_t knee;        // gains and desired positions
  walking_param_leg_t ankley;      // gains and desired positions for ankles
} walking_leg_t;

typedef struct {
  int inx;                         // state machine index
  int knee_inx;                    // knee passed index
  float tstart;                    // time that current state began
  float t1;                        // time of a certain moment
  float t2;                        // time of a certain moment
  int stance_leg;                  // index of stance leg: 0 = right, 1 = left
  walking_param_hip_t hipx;        // gains and desired postition
  walking_leg_t swing;             // gains and desired position for swing leg
  walking_leg_t stance;            // gains and desired position for stance leg
  walking_ankle_t l;               // desired angle for lateral movement
  walking_ankle_t r;               // desired angle for lateral movement
  float rel_hipangle_q;            // relative angle
  float rel_hipangle_qd;           // relative angular velocity
  float rel_hipangle_q_d;          // desired relative angle
  float rel_hipangle_prev;         // previous relative angle between legs
  float rel_hipangle_qd_d;         // desired angular velocity at end
} walking_con_t;

typedef struct {
  int samples;                     // number of IMU samples
  float yaw;                       // Euler angle
  float pitch;                     // Euler angle
  float roll;                      // Euler angle
} imu_data_t;

typedef struct {
  int mode;                        // index of the controller mode
  float t;                         // sensor clock time, in seconds
  float dt;                        // the idealized time step between control updates
  float t_mode;                    // timestamp when mode was entered
  timing_data_t timing;            // timing diagnostics
  int LEDS;                        // current output value of indicator LEDs
  flame_gains_t q_start;           // start position for sin wave for DEMO
  imu_data_t imu;                  // inertial data
  float mot_sw_filt;               // filtered switched motor battery voltage readings
  test_controller_state demo;      // a test controller
  standing_con_t standing;         // a simple stand-in-place controller
  exercise_con_t exercise;         // a exercise program, to warm up and detect index pulses
  walking_con_t walking;           // a initiation of walking controller
  joint_con_t joint;               // set of individual joint controllers
} controller_state_t;

#endif /*  __SYSTEM_H_INCLUDED__ */
