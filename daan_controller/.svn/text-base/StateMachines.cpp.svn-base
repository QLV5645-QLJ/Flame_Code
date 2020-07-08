#include "StateMachines.h"
#include "globals.h"	// TODO: If you want to remove this, implement a new CTimedStateMachine class

CStateMachine::CStateMachine()
{
	mCurrentState = 0;
	mStateStartingTime = 0;
}

void CStateMachine::Update()
{
	mCurrentState->Update();
}

float CStateMachine::TimeElapsed()
{
	return (s.t - mStateStartingTime);
}

void CStateMachine::Transition(CStateMachineState* newState)
{
	if (mCurrentState != 0)
		mCurrentState->DeInit();
	
	mStateStartingTime = s.t;
	mCurrentState = newState;
	mCurrentState->SetParent(this);
	mCurrentState->Init();
}

bool CStateMachine::IsInState(CStateMachineState* state)
{
	return state == mCurrentState;
}
