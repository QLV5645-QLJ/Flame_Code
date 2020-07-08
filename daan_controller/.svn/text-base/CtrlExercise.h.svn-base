#ifndef __CTRLEXERCISE_H_INCLUDED
#define __CTRLEXERCISE_H_INCLUDED

#include "StateMachines.h"
#include "globals.h"
#include <hardware_drivers/FlameIO.h>
#include "polynomials.h"
#include "LookupTables.h"
#include <utility/utility.h>


class CExerciseController: public CStateMachine
{
	public:
		void	Init();
};

class CExercise_StEnd: public CStateMachineState
{

};

class CExercise_StHipx: public CStateMachineState
{
	public:
		void	Init();
		void	Update();
		void	DeInit();
};

class CExercise_StHipy: public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};

class CExercise_StKnee: public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};

class CExercise_StAnkley: public CStateMachineState
{
	public:
		void	Init();
		void	Update();
};

class CExercise_StHipyKneeAnkley_FindIndex: public CStateMachineState
{
	public:
		void	Init();
		void	Update();
		void	DeInit();		
};

class CExercise_StHipyKneeAnkley_FindZeroTorque: public CStateMachineState
{
	// define sum values for averaging difference tau and tauSEA
	float sum_l_hipy_taudiff;
	float sum_r_hipy_taudiff;
	float sum_l_knee_taudiff;
	float sum_r_knee_taudiff;
	float sum_l_ankley_taudiff;
	float sum_r_ankley_taudiff;
	
	float count_taudiff;
	
	public:
		void	Init();
		void	Update();
		void	DeInit();		
};

class CExercise_StTorqueControl: public CStateMachineState
{
	public:
		void	Init();
		void	Update();		
};

class CExercise_StAngleControl: public CStateMachineState
{
	public:
		CLUTLinear ff_l_hipy;
		CLUTLinear ff_r_hipy;
		
		void	Init();
		void	Update();	
};

extern CExerciseController	gExerciseController;
extern CExercise_StEnd		gExercise_StEnd;

#endif
