#ifndef __CTRLWALKING_H_INCLUDED
#define __CTRLWALKING_H_INCLUDED

#include "StateMachines.h"

//#include "System.h"
#include "utility/utility.h"
#include "polynomials.h"
#include "globals.h"
#include "LookupTables.h"
//#include "joint.h"

#define FLAME_PUSHBUTTON_PRESSED(bits, mask) (!((bits) & (mask)))

#define STANCE_LEG_RIGHT	0
#define STANCE_LEG_LEFT		1



class CWalkingController: public CStateMachine
{
	public:
		void	Init();
		void	Deinit();

		void	SetStanceLeg(bool left_is_stance);
		bool	GetStanceLeg();
};

class CWalkingState: public CStateMachineState
{
	protected:
		float mT1;						// time of a certain moment
		float mT2;						// time of a certain moment
	public:
		CWalkingState();

		CWalkingController *Controller()	{ return (CWalkingController*)mParent;}
};


/****************************************************************/
// The states of walking
/****************************************************************/

class CWalking_StGetReady: public CWalkingState
{
	protected:
		int		touched_down;
	public:
		void	Init();
		void	Update();
};

class CWalking_StPushoff: public CWalkingState
{
	protected:
		float 	local_stance_control_gain;
		float 	local_stance_control_des;
		float 	time_elapsed_step;
		
	public:
		void	Init();
		void	Update();
};

class CWalking_StSwing: public CWalkingState
{
	protected:
		float 	local_stance_control_gain;
		float 	local_stance_control_des;
		float 	time_elapsed_step;
		int		footplacement_determined;
		float	hipx_ref_startstep;
		float	hipx_ref_startfp;
		
	public:
		CLUTLinear ff_swing_interleg;
		CLUTLinear coupling_hipx_interleg;
		CLUTLinear roll_trajectory;
	
		void	Init();
		void	Update();
};

class CWalking_StStanceFootRelease: public CWalkingState
{
	protected:
		float	hipx_q_start;
		float	l_hipy_q_start;
		float	r_hipy_q_start;
		float	l_knee_q_start;
		float	r_knee_q_start;
		float	l_ankley_q_start;
		float	r_ankley_q_start;
	public:
		void	Init();
		void	Update();
};

class CWalking_StCrash: public CWalkingState
{
	public:
		void	Init();
		void	Update();
};

extern CWalkingController		gWalkingController;
extern CWalking_StGetReady		gWalking_StGetReady;

#endif
