// Declarations for the various controller functions.

#ifndef __CONTROL_DEMO_H_INCLUDED__
#define __CONTROL_DEMO_H_INCLUDED__

#include "globals.h"
#include "joint.h"

/****************************************************************/
// define symbols for the exercise state machine

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

// define symbols for the state machine
enum {
  S_SLEEP = 0,
  S_STAND,
  S_STAND_CONTROL,
  S_NOSTANDSTATE
};

extern void init_standing_controller(void);
extern void update_standing_controller(void);

extern void init_exercise_controller(void);
extern void update_exercise_controller(void);

extern void init_demo_controller(void);
extern void update_demo_controller(void);

extern void init_start_walking_controller(void);
extern void update_start_walking_controller(void);

extern void update_walking_controller(void);

#endif
