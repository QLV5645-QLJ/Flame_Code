#ifndef __CTRLSTANDING_H_INCLUDED
#define __CTRLSTANDING_H_INCLUDED

#include "StateMachines.h"

#include "utility/utility.h"
#include "polynomials.h"
#include "globals.h"
//#include "joint.h"
#include <hardware_drivers/FlameIO.h>

#define LBODY 0.175        // length body mass above hipx: estimated
#define LHIP  0.069        // width of hip (anklex to hipx)
#define LLEG  0.647        // length total leg (anklex hipx)!

#define ANKLEY_POS       0.00
#define HIPY_POS        0.01
#define KNEE_POS       -0.08
#define HIPX_POS        0.035//0.06

enum
{
  S_POSITION = 0,
  S_TORQUE,
  S_UNKNOWN,
};

class CStandingController: public CStateMachine
{
	public:
		void	Init();
		void	Deinit();
};

class CStanding_StSleep: public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};



class CStanding_StStandControl: public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};

// External declaration of the actual objects
extern CStandingController			gStandingController;
extern CStanding_StSleep			gStanding_StSleep;
extern CStanding_StStandControl		gStanding_StStandControl;

#endif // __CTRLSTANDING_H_INCLUDED
