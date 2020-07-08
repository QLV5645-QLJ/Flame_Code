#ifndef __CTRLFLAME_H_INCLUDED
#define __CTRLFLAME_H_INCLUDED

#include "StateMachines.h"
#include "globals.h"

class CFlameController: public CStateMachine
{
	protected:
		bool			mShouldShutdown;
	public:
		// Constructor/destructor
		CFlameController();

		void			CallForShutdown();
		bool			ShouldShutdown();

		virtual void	Init();
		//virtual void	Update();
		virtual void	Deinit();
		//virtual void	Transition(CStateMachineState* newState);	// Transition to a new state

};

/*
MODE_STARTUP
MODE_IDLE
MODE_BEGIN_SHUTDOWN
MODE_SHUTDOWN
MODE_POWER_UP_DRIVERS
MODE_POWER_DOWN_DRIVERS
MODE_STANDING
MODE_EXERCISE
//MODE_START_WALKING
MODE_WALKING

*/

class CFlame_StStartup : public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};

class CFlame_StIdle : public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};

class CFlame_StBeginShutdown : public CStateMachineState
{
	public:
		CFlameController*	Controller()	{ return (CFlameController*)mParent;}

		void	Init();
		void	Update();
};
/*
class CFlame_StShutdown : public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};
*/
class CFlame_StPowerUpDrivers : public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};

class CFlame_StPowerDownDrivers : public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};

class CFlame_StStanding : public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};

class CFlame_StExercise : public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};


class CFlame_StWalking : public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};



extern CFlameController gFlameController;
extern CFlame_StIdle    gFlame_StIdle;
extern CFlame_StBeginShutdown  gFlame_StBeginShutdown;







#endif // __CTRLFLAME_H_INCLUDED
