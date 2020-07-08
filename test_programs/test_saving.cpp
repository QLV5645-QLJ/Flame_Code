// $Id: test_saving.c,v 1.1 2005/12/14 08:32:07 garthz Exp $
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.
//

#include <stdio.h>
#include <utility/utility.h>
#include <utility/system_state_var.h>
#include <errno.h>
#include <regex.h>

typedef struct {
  float qe;                        // estimated position
  float l_q;                       // position error gain
  float l_qd;                      // velocity error gain
} velocity_estimator_t;

typedef struct {
  velocity_estimator_t hipy;       // hip pitch joint angle wrt body (around y axis), in radians
  velocity_estimator_t knee;       // knee joint angle wrt body (around y axis), in radians
  velocity_estimator_t ankley;     // ankle pitch joint angle wrt body (around y axis), in radians
  velocity_estimator_t anklex;     // ankle roll joint angle wrt body (around x axis), in radians
  velocity_estimator_t hipymot;    // hip pitch motor angle, in radians
  velocity_estimator_t kneemot;    // knee motor angle, in radians
  velocity_estimator_t ankleymot;  // ankle motor angle, in radians
} leg_velocity_estimators_t;

typedef struct {
  velocity_estimator_t hipx;       // hip roll joint
  leg_velocity_estimators_t l;     // left leg
  leg_velocity_estimators_t r;     // right leg
} flame_velocity_estimators_t;

typedef struct {
  float sensor_processing;         // duration of the sensor processing on the previous cycle
  float total_cycle;               // duration of the previous complete processing cycle
} timing_data_t;

typedef struct {
  int mode;                        // index of the controller mode
  float t;                         // sensor clock time, in seconds
  timing_data_t timing;            // timing diagnostics
  int LEDS;                        // current output value of indicator LEDs
  flame_velocity_estimators_t vel_estim; // velocity filters
} controller_state_t;

static controller_state_t c;     // the controller state blackboard

static system_state_var_t system_vars[] = {
  { "mode", SYS_INT, &c.mode },
  { "t", SYS_FLOAT, &c.t },
  { "timing.sensor_processing", SYS_FLOAT, &c.timing.sensor_processing },
  { "timing.total_cycle", SYS_FLOAT, &c.timing.total_cycle },
  { "LEDS", SYS_INT, &c.LEDS },
  { "vel_estim.hipx.qe", SYS_FLOAT, &c.vel_estim.hipx.qe },
  { "vel_estim.hipx.l_q", SYS_FLOAT, &c.vel_estim.hipx.l_q },
  { "vel_estim.hipx.l_qd", SYS_FLOAT, &c.vel_estim.hipx.l_qd },
  { "vel_estim.l.hipy.qe", SYS_FLOAT, &c.vel_estim.l.hipy.qe },
  { "vel_estim.l.hipy.l_q", SYS_FLOAT, &c.vel_estim.l.hipy.l_q },
  { "vel_estim.l.hipy.l_qd", SYS_FLOAT, &c.vel_estim.l.hipy.l_qd },
  { "vel_estim.l.knee.qe", SYS_FLOAT, &c.vel_estim.l.knee.qe },
  { "vel_estim.l.knee.l_q", SYS_FLOAT, &c.vel_estim.l.knee.l_q },
  { "vel_estim.l.knee.l_qd", SYS_FLOAT, &c.vel_estim.l.knee.l_qd },
  { "vel_estim.l.ankley.qe", SYS_FLOAT, &c.vel_estim.l.ankley.qe },
  { "vel_estim.l.ankley.l_q", SYS_FLOAT, &c.vel_estim.l.ankley.l_q },
  { "vel_estim.l.ankley.l_qd", SYS_FLOAT, &c.vel_estim.l.ankley.l_qd },
  { "vel_estim.l.anklex.qe", SYS_FLOAT, &c.vel_estim.l.anklex.qe },
  { "vel_estim.l.anklex.l_q", SYS_FLOAT, &c.vel_estim.l.anklex.l_q },
  { "vel_estim.l.anklex.l_qd", SYS_FLOAT, &c.vel_estim.l.anklex.l_qd },
  { "vel_estim.l.hipymot.qe", SYS_FLOAT, &c.vel_estim.l.hipymot.qe },
  { "vel_estim.l.hipymot.l_q", SYS_FLOAT, &c.vel_estim.l.hipymot.l_q },
  { "vel_estim.l.hipymot.l_qd", SYS_FLOAT, &c.vel_estim.l.hipymot.l_qd },
  { "vel_estim.l.kneemot.qe", SYS_FLOAT, &c.vel_estim.l.kneemot.qe },
  { "vel_estim.l.kneemot.l_q", SYS_FLOAT, &c.vel_estim.l.kneemot.l_q },
  { "vel_estim.l.kneemot.l_qd", SYS_FLOAT, &c.vel_estim.l.kneemot.l_qd },
  { "vel_estim.l.ankleymot.qe", SYS_FLOAT, &c.vel_estim.l.ankleymot.qe },
  { "vel_estim.l.ankleymot.l_q", SYS_FLOAT, &c.vel_estim.l.ankleymot.l_q },
  { "vel_estim.l.ankleymot.l_qd", SYS_FLOAT, &c.vel_estim.l.ankleymot.l_qd },
  { "vel_estim.r.hipy.qe", SYS_FLOAT, &c.vel_estim.r.hipy.qe },
  { "vel_estim.r.hipy.l_q", SYS_FLOAT, &c.vel_estim.r.hipy.l_q },
  { "vel_estim.r.hipy.l_qd", SYS_FLOAT, &c.vel_estim.r.hipy.l_qd },
  { "vel_estim.r.knee.qe", SYS_FLOAT, &c.vel_estim.r.knee.qe },
  { "vel_estim.r.knee.l_q", SYS_FLOAT, &c.vel_estim.r.knee.l_q },
  { "vel_estim.r.knee.l_qd", SYS_FLOAT, &c.vel_estim.r.knee.l_qd },
  { "vel_estim.r.ankley.qe", SYS_FLOAT, &c.vel_estim.r.ankley.qe },
  { "vel_estim.r.ankley.l_q", SYS_FLOAT, &c.vel_estim.r.ankley.l_q },
  { "vel_estim.r.ankley.l_qd", SYS_FLOAT, &c.vel_estim.r.ankley.l_qd },
  { "vel_estim.r.anklex.qe", SYS_FLOAT, &c.vel_estim.r.anklex.qe },
  { "vel_estim.r.anklex.l_q", SYS_FLOAT, &c.vel_estim.r.anklex.l_q },
  { "vel_estim.r.anklex.l_qd", SYS_FLOAT, &c.vel_estim.r.anklex.l_qd },
  { "vel_estim.r.hipymot.qe", SYS_FLOAT, &c.vel_estim.r.hipymot.qe },
  { "vel_estim.r.hipymot.l_q", SYS_FLOAT, &c.vel_estim.r.hipymot.l_q },
  { "vel_estim.r.hipymot.l_qd", SYS_FLOAT, &c.vel_estim.r.hipymot.l_qd },
  { "vel_estim.r.kneemot.qe", SYS_FLOAT, &c.vel_estim.r.kneemot.qe },
  { "vel_estim.r.kneemot.l_q", SYS_FLOAT, &c.vel_estim.r.kneemot.l_q },
  { "vel_estim.r.kneemot.l_qd", SYS_FLOAT, &c.vel_estim.r.kneemot.l_qd },
  { "vel_estim.r.ankleymot.qe", SYS_FLOAT, &c.vel_estim.r.ankleymot.qe },
  { "vel_estim.r.ankleymot.l_q", SYS_FLOAT, &c.vel_estim.r.ankleymot.l_q },
  { "vel_estim.r.ankleymot.l_qd", SYS_FLOAT, &c.vel_estim.r.ankleymot.l_qd },
    

  // a final entry to mark the end of the list
  { NULL, SYS_NOTYPE, NULL }
};




int main(int argc, char **argv)
{
  int r;
  
  r = system_state_var_array_set_from_file( system_vars, "PARAMS" );
  if ( r ) {
    printf("unable to open parameter file: %s\n", strerror(errno));
  }

  r = system_state_var_array_save_to_file( system_vars, "PARAMS" );
  if ( r ) {
    printf("unable to save parameter file: %s\n", strerror(errno));
  }

  
  // test of regexec
  {
    system_state_var_t *v = system_vars;
    regex_t pattern1;
    regcomp( &pattern1, "vel_estim\\..*l_qd*$", REG_NOSUB );

    while ( v->name ) {
      if ( !regexec( &pattern1, v->name, 0, NULL, 0 ) ) {
	printf(" pattern1 matches %s\n", v->name );
      }
      v++;
    }
    regfree( &pattern1 );
  }
}
