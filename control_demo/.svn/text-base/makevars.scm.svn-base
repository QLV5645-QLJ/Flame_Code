#!/usr/bin/guile \
-e main -s
!#
;;
;; Guile scheme script to generate the variable declarations and
;; tables for the dynamic system in the bp3dsim biped simulator.
;;
;; Copyright (C) 2001-2005 Garth Zeglin.  Provided under the terms of the
;; GNU General Public License as included in the top level directory.
;;
;; This program creates the following:
;;   auto_system.h       state variable declarations
;;   auto_sysvars.h      state variable description table entries

;; All of the information about variables is contained in the
;; statically defined arrays below.  The advantage of this script
;; is that one representation can be used to generate all the
;; scattered bits of code.  Note that some of the declaration code
;; is still handwritten, so the files are included into system.h
;; and sysvars.h

(load "../scheme/variable-tables.scm")


;; ###############################################################
;; Define state variable structures.

;; The first element is the type name of the structure, followed
;; by lists with (name, type, desc).

(define timing_data
  `( "timing_data_t"
     ( "sensor_processing"   "SYS_FLOAT"     "duration of the sensor processing on the previous cycle" )
     ( "total_cycle"         "SYS_FLOAT"     "duration of the previous complete processing cycle" )
     ))


;; Define structures to store controller gains.
(define leg_gains
  `( "leg_gains_t"
     ( "hipy"           "SYS_FLOAT" 	 "hip pitch joint angle wrt body (around y axis), in radians")
     ( "knee"           "SYS_FLOAT" 	 "knee joint angle wrt body (around y axis), in radians" )
     ( "ankley"         "SYS_FLOAT"      "ankle pitch joint angle wrt body (around y axis), in radians" )
     ))

(define flame_gains
  `( "flame_gains_t"
     ( "hipx"        "SYS_FLOAT"  	 "hip roll joint angle wrt body (around x axis), in radians")
     ( "l" 	     ,leg_gains     	  "left leg" )
     ( "r" 	     ,leg_gains     	  "right leg" )
     ))
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define the variables for the demonstration controller
(define demo_control
  `( "test_controller_state"
     ( "inx"          "SYS_INT"         "state machine index")
     ( "tstart"	      "SYS_FLOAT"       "start time for the state")
     ( "amplitude"    "SYS_FLOAT"	"sine wave amplitude")
     ( "frequency"    "SYS_FLOAT"	"sine wave frequency")
     ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define the variables for the individual joint controllers in joint.c.

(define joint_param ;; parameters for a controller for a single joint
  `( "joint_param_t"
     ( "mode"         "SYS_INT"          "joint control mode")
     ( "q_d"          "SYS_FLOAT"        "desired position" )
     ( "q_dprev"      "SYS_FLOAT"        "previous desired position" )
     ( "tau_d"        "SYS_FLOAT"        "desired torque")
     ( "tau_dprev"    "SYS_FLOAT"        "previous desired torque")
     ( "taud_d"       "SYS_FLOAT"        "derivative of desired torque")
     ( "qd_d"         "SYS_FLOAT"        "desired velocity" )
     ( "kp"           "SYS_FLOAT"        "proportional gain" )
     ( "kd"           "SYS_FLOAT"        "damping" )
     ( "qref"         "SYS_FLOAT"        "reference trajectory position" )
     ( "data"         "SYS_FLOAT"	 "empty slot for any data item")
     ( "kp_bi"        "SYS_FLOAT"        "stiffness for bisecting mechanism in hip" )
     ( "kd_bi"        "SYS_FLOAT"        "damping for bisecting mechanism in hip" )
     ( "turbo_boost"  "SYS_INT"          "maximum amps boost" )
     ))

(define joint_param2 ;; parameters for a controller for the ankle joint
  `( "joint_param2_t"
     ( "mode"         "SYS_INT"          "joint control mode")
     ( "q_d"          "SYS_FLOAT"        "desired position" )
     ( "q_dprev"      "SYS_FLOAT"        "previous desired position" )
     ( "tau_d"        "SYS_FLOAT"        "desired torque")
     ( "tau_dprev"    "SYS_FLOAT"        "previous desired torque")
     ( "taud_d"       "SYS_FLOAT"        "derivative of desired torque")
     ( "qd_d"         "SYS_FLOAT"        "desired velocity" )
     ( "kspring"      "SYS_FLOAT"        "spring stiffness" )
     ( "kp"           "SYS_FLOAT"        "proportional gain" )
     ( "kd"           "SYS_FLOAT"        "damping" )
     ( "kpp"          "SYS_FLOAT"	 "stiffness of position controller") 
     ( "kdd"          "SYS_FLOAT"	 "damping of position controller") 
     ( "qref"         "SYS_FLOAT"        "reference trajectory position" )
     ( "tau_load"     "SYS_FLOAT"	 "calculated torque present in ankle by cable")
     ( "turbo_boost"  "SYS_INT"          "Boost to overwrite the maximum output torque")
     ))

(define joint_leg  ;; parameters for joint controllers for a whole leg
  `( "joint_leg_t"
    ( "hipy"        ,joint_param         "gains and desired positions")
    ( "knee"        ,joint_param         "gains and desired positions")
    ( "ankley"      ,joint_param2        "gains and desired positions for ankles")
    ))

(define joint_control ;; parameters for the joint controller
  `( "joint_con_t"
    ( "hipx"          ,joint_param        "gains and desired positions")
    ( "l"             ,joint_leg          "gains and desired positions")
    ( "r"             ,joint_leg          "gains and desired positions")
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define the variables used by the "standing" controller.
(define standing_control
  `( "standing_con_t"
     ( "inx"          "SYS_INT"           "state machine index")
     ( "tstart"	      "SYS_FLOAT"         "start time for the state")
     ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define the variables used by the "exercise" controller.
(define exercise_control
  `( "exercise_con_t"
     ( "inx"          "SYS_INT"           "state machine index")
     ( "tstart"	      "SYS_FLOAT"         "start time for the state")
     ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define the IMU data.
(define imu_data
  `( "imu_data_t"
     ( "samples"      "SYS_INT"           "number of IMU samples")
     ( "yaw"          "SYS_FLOAT"         "Euler angle")
     ( "pitch"        "SYS_FLOAT"         "Euler angle")
     ( "roll"         "SYS_FLOAT"         "Euler angle")
     ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define the variables used by the 'walking' controller
(define walking_param_anklex
  `("walking_param_anklex_t"
    ( "q_d"             "SYS_FLOAT"    	  "lateral desired angle")
    ( "q_prev"          "SYS_FLOAT"    	  "previous lateral desired angle")
    ( "qd_d"            "SYS_FLOAT"    	  "lateral desired angular velocity")
    ( "kmp"             "SYS_FLOAT"    	  "stiffness for feedback controller lateral direction")
    ( "kmd"             "SYS_FLOAT"	  "damping for feedback controller lateral direction") 
    ( "yfoot"           "SYS_FLOAT"       "distance to the other foot (y-direction)")
    ( "tau_extra"       "SYS_FLOAT"    	  "extra torque to get the sideways motion in action")
    ( "tau_prev"        "SYS_FLOAT"    	  "last torque in the other ankle for smooth transition")
    ))

(define walking_param_leg
  `("walking_param_leg_t"
    ( "q_d"             "SYS_FLOAT"    	  "desired angle")
    ( "q_prev"          "SYS_FLOAT"    	  "previous desired angle")
    ( "qd_d"            "SYS_FLOAT"    	  "desired angular velocity")
    ( "qd_prev"         "SYS_FLOAT"    	  "initial angle when both feet are on ground")
    ( "tau_d"           "SYS_FLOAT"    	  "desired torque")
    ( "tau_prev"        "SYS_FLOAT"    	  "last torque in the other ankle for smooth transition")
    ( "taud_prev"       "SYS_FLOAT"    	  "previous desired torque speed")
    ))

(define walking_param_hip
  `("walking_param_hip_t"
    ( "q_d"          "SYS_FLOAT"    	  "lateral desired angle")
    ( "qd_d"         "SYS_FLOAT"    	  "lateral desired angular velocity")
    ( "ymass"        "SYS_FLOAT"    	  "location of the mass as seen from the foot")
    ))

(define walking_ankle
  `("walking_ankle_t"
    ( "anklex"      ,walking_param_anklex      "walking parameters")
    ))

(define walking_leg
  `( "walking_leg_t"
    ( "hipy"        ,walking_param_leg         "gains and desired positions")
    ( "knee"        ,walking_param_leg         "gains and desired positions")
    ( "ankley"      ,walking_param_leg         "gains and desired positions for ankles")
    ))

(define walking_control
  `("walking_con_t"
    ;; state machine 
    ( "inx"             "SYS_INT"    	  "state machine index")
    ( "knee_inx"        "SYS_INT"    	  "knee passed index")
    ( "tstart"          "SYS_FLOAT"       "time that current state began")
    ( "t1"              "SYS_FLOAT"       "time of a certain moment")
    ( "t2"              "SYS_FLOAT"       "time of a certain moment")
    ( "stance_leg"      "SYS_INT"    	  "index of stance leg: 0 = right, 1 = left")

    ;; Controller. You need different names for swing leg and stance leg
    ( "hipx"            ,walking_param_hip      "gains and desired postition")
    ( "swing"           ,walking_leg            "gains and desired position for swing leg")
    ( "stance"          ,walking_leg            "gains and desired position for stance leg")
    ( "l"               ,walking_ankle          "desired angle for lateral movement")
    ( "r"               ,walking_ankle          "desired angle for lateral movement")

    ;; for controlling the upper body, a relative angle between both upper legs is required.
    ( "rel_hipangle_q"              "SYS_FLOAT"            "relative angle")
    ( "rel_hipangle_qd"             "SYS_FLOAT"            "relative angular velocity")
    ( "rel_hipangle_q_d"            "SYS_FLOAT"            "desired relative angle")
    ( "rel_hipangle_prev"           "SYS_FLOAT"            "previous relative angle between legs")
    ( "rel_hipangle_qd_d"           "SYS_FLOAT"            "desired angular velocity at end")
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define the variables used by the "garthz" walking controller.
;;(define garthz_control
;;  `("garthz_con_t"
;;    ;; state machine 
;;    ( "inx"          "SYS_INT"    	  "state machine index" )
;;    ( "inxname"      "SYS_STRING" 	  "state name" )
;;    ( "tstart"       "SYS_FLOAT"         "time that current state began")
;;    ( "st_leg"       "SYS_INT"    	  "index of stance leg: 0 = left, 1 = right" )
;;    ( "motors"       "SYS_INT"     	  "mask for active motors" ) 
;;
;;    ;; computed values
;;    ( "pitch"        "SYS_FLOAT"          "computed value of body pitch")  
;;    ( "sw_leg_ang"   "SYS_FLOAT"         "computed value of hip-foot leg angle" )
;;    ( "st_leg_ang"   "SYS_FLOAT"         "computed value of hip-foot leg angle" )
;;
;;    ( "rel_ang_COM"    "SYS_FLOAT"       "computed value of the relative leg COM angles")
;;    ( "rel_omega_COM"  "SYS_FLOAT"       "computed value of the relative leg COM angular velocity")
;;    ( "rel_d_leg_ang"  "SYS_FLOAT"   	  "computed desired relative leg angle, measured from hip to ankle")
;;
;;    ( "t0_swing"       "SYS_FLOAT"       "recorded timestamp of beginning of stride")
;;    ( "rel0_ang_COM"   "SYS_FLOAT"       "recorded value of the relative leg COM angles at beginning of stride")
;;
;;   ;; controller parameters
;;    ( "q_d_pitch"    "SYS_FLOAT"         "desired body pitch angle")
;;
;;    ( "tmin_swing"   "SYS_FLOAT"         "minimum duration of swing")
;;
;;    ( "q_d_hipx"	"SYS_FLOAT"   	  "desired hipx angle")
;;    ( "qd_d_hipx"	"SYS_FLOAT"   	  "desired hipx velocity")
;;    ( "k_hipx"		"SYS_FLOAT" 	  "hipx position gain" )
;;    ( "b_hipx"		"SYS_FLOAT" 	  "hipx damping" )
;;
;;   ( "k_yd_hipx"    "SYS_FLOAT" 	  "foot placement gains" )
;;    ( "k_roll_hipx"  "SYS_FLOAT" 	  "foot placement gains" )
;;
;;    ( "sw_k_hipy"    "SYS_FLOAT" 	  "swing gains" )
;;    ( "sw_b_hipy"    "SYS_FLOAT" 	  "swing gains" )
;;    ( "st_k_hipy"    "SYS_FLOAT" 	  "stance gains" )
;;    ( "st_b_hipy"    "SYS_FLOAT" 	  "stance gains" )
;;
;;    ( "t_hipy_extend"       "SYS_FLOAT"   "trajectory reference time for leg swing")
;;    ( "ext_rel_ang_COM"     "SYS_FLOAT"   "the relative leg angle at extension")
;;    ( "ret_rel_omega_COM"   "SYS_FLOAT"   "the retraction velocity at that point")
;;    ( "ret_max_ang_COM"     "SYS_FLOAT"   "the retraction limit angle" )
;;
;;    ( "sw_q_d_knee"  "SYS_FLOAT"   	  "desired angle")
;;    ( "sw_qd_d_knee" "SYS_FLOAT"   	  "desired velocity")
;;    ( "sw_k_knee"    "SYS_FLOAT" 	  "swing gains" )
;;    ( "sw_b_knee"    "SYS_FLOAT" 	  "swing gains" )
;;    ( "st_q_d_knee"  "SYS_FLOAT"   	  "desired angle")
;;    ( "st_qd_d_knee" "SYS_FLOAT"   	  "desired velocity")
;;    ( "st_k_knee"    "SYS_FLOAT" 	  "stance gains" )
;;    ( "st_b_knee"    "SYS_FLOAT" 	  "stance gains" )
;;
;;    ( "t_knee_retract" "SYS_FLOAT"       "knee motion duration")
;;    ( "t_knee_bent"    "SYS_FLOAT"       "knee motion duration")
;;    ( "t_knee_extend"  "SYS_FLOAT"       "knee motion duration")
;;
;;    ( "st_q_d_ankley" "SYS_FLOAT" 	  "stance ankle position")
;;    ( "st_qd_d_ankley" "SYS_FLOAT" 	  "stance ankle velocity")
;;   ( "st_k_ankley" "SYS_FLOAT" 	  "stance ankle gain")
;;    ( "st_b_ankley" "SYS_FLOAT" 	  "stance ankle gain")
;;
;;    ( "sw_q_d_ankley" "SYS_FLOAT" 	  "swing ankle position")
;;    ( "sw_qd_d_ankley" "SYS_FLOAT" 	  "swing ankle velocity")
;;    ( "sw_k_ankley" "SYS_FLOAT" 	  "swing ankle gain")
;;    ( "sw_b_ankley" "SYS_FLOAT" 	  "swing ankle gain")
;;    ( "t_ankley_bent"  "SYS_FLOAT"       "swing retraction duration")
;;
;;    ( "to_q_d_ankley"    "SYS_FLOAT" 	  "toeoff ankle position")
;;    ( "to_k_ankley" "SYS_FLOAT" 	  "toeoff ankle gain")
;;    ( "to_b_ankley" "SYS_FLOAT" 	  "toeoff ankle gain")
;;    ( "to_ff_tau_ankley" "SYS_FLOAT" 	  "toeoff feedforward torques")
;;    ( "tmin_toeoff"  "SYS_FLOAT"         "minimum duration of toeoff")
;;    ( "tmax_toeoff"  "SYS_FLOAT"         "maximum duration of toeoff")
;;    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Top level controller state structure.
(define controller_state
  `( "controller_state_t"
     ( "mode"        "SYS_INT"         "index of the controller mode" )
     ( "t"           "SYS_FLOAT"       "sensor clock time, in seconds" )
     ( "dt"          "SYS_FLOAT"       "the idealized time step between control updates")
     ( "t_mode"      "SYS_FLOAT"       "timestamp when mode was entered")
     ( "timing"      ,timing_data      "timing diagnostics" )
     ( "LEDS"        "SYS_INT"         "current output value of indicator LEDs")
     ( "q_start"     ,flame_gains      "start position for sin wave for DEMO")
     ( "imu"         ,imu_data         "inertial data")

     ;; filter on the motor voltage
     ( "mot_sw_filt"	"SYS_FLOAT"	"filtered switched motor battery voltage readings" )

     ;; parameters for particular controllers
     ( "demo"		,demo_control           "a test controller")
     ( "standing"       ,standing_control       "a simple stand-in-place controller")
     ( "exercise"       ,exercise_control       "a exercise program, to warm up and detect index pulses")
     ( "walking"        ,walking_control        "a initiation of walking controller")
     ( "joint"          ,joint_control          "set of individual joint controllers")
     ))

;;###############################################################

(define (main args)
  (with-output-to-file "system.h"
    (lambda ()
      (format #t "/* GENERATED FILE - DO NOT EDIT */~%")
      (format #t "/* This file contains data structure declarations created by makevars.scm */~%~%")
      (format #t "#ifndef __SYSTEM_H_INCLUDED__~%#define __SYSTEM_H_INCLUDED__~%~%")
      (format #t "#include <utility/system_state_var.h>~%~%")

      ;; Emit structure definitions.
      (define-struct leg_gains)
      (define-struct flame_gains)
      (define-struct timing_data)

      (define-struct demo_control)
      (define-struct standing_control)
      (define-struct exercise_control)
 
      (define-struct joint_param)
      (define-struct joint_param2)
      (define-struct joint_leg)
      (define-struct joint_control)
      (define-struct walking_param_hip)
      (define-struct walking_param_anklex)
      (define-struct walking_param_leg)
      (define-struct walking_ankle)
      (define-struct walking_leg)

      (define-struct walking_control)
      (define-struct imu_data)
      (define-struct controller_state)

      (format #t "#endif /*  __SYSTEM_H_INCLUDED__ */~%")
      ))

  (with-output-to-file "sysvars.h"
    (lambda ()
      (format #t "/* GENERATED FILE - DO NOT EDIT */~%")
      (format #t "/* This file contains system variable table entries created by makevars.scm */~%")

      ;; Generates entries in the state structure table system_vars.
      (make-sysvar-table controller_state "c")

      (force-output (current-output-port))
      ))
  )

