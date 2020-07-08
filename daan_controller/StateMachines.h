#ifndef __STATEMACHINES_H_INCLUDED
#define __STATEMACHINES_H_INCLUDED

class CStateMachineState;

class CStateMachine
{
	protected:
		CStateMachineState	*mCurrentState;
	public:
		float mStateStartingTime;		// time that current state began

		CStateMachine();
		virtual void	Init()		{}
		virtual void	Update();
		virtual void	DeInit()	{}
		virtual void	Transition(CStateMachineState* newState);	// Transition to a new state
		bool			IsInState(CStateMachineState* state);

		float			TimeElapsed();

};

class CStateMachineState
{
	protected:
		CStateMachine	*mParent;
	public:
		//
		void 			SetParent(CStateMachine* newParent)	{ mParent = newParent; }
		// Access to the parent
		CStateMachine*	GetParent()					{ return mParent;}
		// .. usually, it is called the controller
		CStateMachine*	Controller()				{ return mParent;}

		virtual	void	Init()			{};	// Called every time the state machine SWITCHES to this state
		virtual	void	Update()		{};
		virtual void 	DeInit()		{};
};

#endif // __STATEMACHINES_H_INCLUDED
