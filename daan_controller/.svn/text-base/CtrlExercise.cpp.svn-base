#include "CtrlExercise.h"

CExerciseController	gExerciseController;
CExercise_StEnd		gExercise_StEnd;
CExercise_StHipx	gExercise_StHipx;
CExercise_StHipy	gExercise_StHipy;
CExercise_StKnee	gExercise_StKnee;
CExercise_StAnkley	gExercise_StAnkley;
CExercise_StHipyKneeAnkley_FindIndex		gExercise_StHipyKneeAnkley_FindIndex;
CExercise_StHipyKneeAnkley_FindZeroTorque	gExercise_StHipyKneeAnkley_FindZeroTorque;
CExercise_StTorqueControl					gExercise_StTorqueControl;
CExercise_StAngleControl					gExercise_StAngleControl;

float	hipx_q_start;
float	hipy_q_start[2];
float 	knee_q_start[2];
float	ankley_q_start[2];
float 	hipx_maxreachedangle = 0;

//float ff_l_hipy_AngleControl[] = {0.0, 0.1, 0.2, 0.5};
//float ff_r_hipy_AngleControl[] = {0.0, 0.1, 0.2, 0.5};


#ifndef max
#define max(a, b)  (((a) > (b)) ? (a) : (b))
#endif

//****************************************************************/
// The standing controller
//****************************************************************/

void CExerciseController::Init()
{
	// disable all controllers
	joints.hipx.Disable();
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		joints.legs[iLeg].hipy.Disable();
		joints.legs[iLeg].knee.Disable();
		joints.legs[iLeg].ankley.Disable();
		
		joints.interleg.Disable();
		joints.upperbody.Disable();
	}
	
	s.powered = 0;//FLAME_ALL_MOTORS;
	
	// Goto Hipx movement
	Transition(&gExercise_StHipx);
}

//****************************************************************/
// Controller states
//****************************************************************/

void CExercise_StHipx::Init()
{
	logprintf("Start Hipx motion\n");
	
	// store angle hipx at start motion
	hipx_q_start = s.hipx.q;
	
	s.powered = FLAME_HIPX_MOTOR;
	joints.hipx.kp = 400;
}

void CExercise_StHipx::Update()
{
	float hipx_maxrefangle = 0.25;
	float hipx_q_maxactvalue = 0.245;
	float hipx_finalrefangle = 0.13;//0.12;
	
	if (Controller()->TimeElapsed() <= 2.0)
	{
		joints.hipx.q.ref = compute_quintic_spline( 0.5*Controller()->TimeElapsed(), hipx_q_start, 0.0, 0.0, hipx_maxrefangle, 0.0, 0.0);
		// find max reached angle
		hipx_maxreachedangle = max(hipx_maxreachedangle, s.hipx.q);
	}
	else
	{
		// adjust offset based om max reached angle
		params.hipx.q.offset = hipx_q_maxactvalue - hipx_maxreachedangle;
		
		if (Controller()->TimeElapsed() <= 4.0)
			joints.hipx.q.ref = compute_quintic_spline( 0.5*(Controller()->TimeElapsed() - 2.0), hipx_q_maxactvalue, 0.0, 0.0, hipx_finalrefangle, 0.0, 0.0);
	}
	
	joints.Update(&s);
	
	if (Controller()->TimeElapsed() >= 4.0)
		Controller()->Transition(&gExercise_StHipyKneeAnkley_FindIndex);//Controller()->Transition(&gExercise_StHipy);
}

void CExercise_StHipx::DeInit()
{	

}

void CExercise_StHipy::Init()
{
	logprintf("Start Hipy motion\n");
	
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		// store angle hipy at start motion
		hipy_q_start[iLeg] = s.legs[iLeg].hipymot.q;
		
		joints.legs[iLeg].hipy.mMode = SEA_MODE_MOTANGLE;
		joints.legs[iLeg].hipy.motangCtrl.kp = 80;
	}
	
	s.powered = FLAME_HIPX_MOTOR | FLAME_LHIPY_MOTOR | FLAME_RHIPY_MOTOR;
	
}

void CExercise_StHipy::Update()
{
	float hipy_maxrefangle = 0.3;
	
	if (Controller()->TimeElapsed() <= 1.0)
	{
		joints.l().hipy.qmot.ref = compute_quintic_spline( Controller()->TimeElapsed(), hipy_q_start[0], 0.0, 0.0, hipy_maxrefangle, 0.0, 0.0);
		joints.r().hipy.qmot.ref = compute_quintic_spline( Controller()->TimeElapsed(), hipy_q_start[1], 0.0, 0.0, -hipy_maxrefangle, 0.0, 0.0);
	}
		else
		{
			if (Controller()->TimeElapsed() <= 3.0)
			{
				joints.l().hipy.qmot.ref = compute_quintic_spline( 0.5*(Controller()->TimeElapsed() - 1.0), hipy_maxrefangle, 0.0, 0.0, -hipy_maxrefangle, 0.0, 0.0);
				joints.r().hipy.qmot.ref = compute_quintic_spline( 0.5*(Controller()->TimeElapsed() - 1.0), -hipy_maxrefangle, 0.0, 0.0, hipy_maxrefangle, 0.0, 0.0);
			}
			else
			{
				if (Controller()->TimeElapsed() <= 4.0)
				{
					joints.l().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 3.0), -hipy_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
					joints.r().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 3.0), hipy_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
				}
			}
		}
	
	joints.Update(&s);
	
	if (Controller()->TimeElapsed() >= 4.0)
		Controller()->Transition(&gExercise_StKnee);
		
}

void CExercise_StKnee::Init()
{
	logprintf("Start Knee motion\n");
		
		for (int iLeg=0; iLeg<2; iLeg++)
		{
			// store angle knee at start motion
			knee_q_start[iLeg] = s.legs[iLeg].kneemot.q;
			
			joints.legs[iLeg].knee.mMode = SEA_MODE_MOTANGLE;
			joints.legs[iLeg].knee.motangCtrl.kp = 25;
		}
		
		s.powered = FLAME_HIPX_MOTOR | FLAME_LHIPY_MOTOR | FLAME_RHIPY_MOTOR | FLAME_LKNEE_MOTOR | FLAME_RKNEE_MOTOR;
}

void CExercise_StKnee::Update()
{
	float knee_maxrefangle = 0.6;
		
	if (Controller()->TimeElapsed() <= 1.0)
	{
		joints.l().knee.qmot.ref = compute_quintic_spline( Controller()->TimeElapsed(), knee_q_start[0], 0.0, 0.0, knee_maxrefangle, 0.0, 0.0);
		joints.r().knee.qmot.ref = compute_quintic_spline( Controller()->TimeElapsed(), knee_q_start[1], 0.0, 0.0, knee_maxrefangle, 0.0, 0.0);
	}
		else
		{
			if (Controller()->TimeElapsed() <= 2.0)
			{
				joints.l().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 1.0), knee_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
				joints.r().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 1.0), knee_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
			}
		}
	
	joints.Update(&s);
	
	if (Controller()->TimeElapsed() >= 2.0)
		Controller()->Transition(&gExercise_StAnkley);
}

void CExercise_StAnkley::Init()
{
	logprintf("Start Ankley motion\n");
	
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		// store angle ankley at start motion
		ankley_q_start[iLeg] = s.legs[iLeg].ankleymot.q;
		
		joints.legs[iLeg].ankley.mMode = SEA_MODE_MOTANGLE;
		joints.legs[iLeg].ankley.motangCtrl.kp = 40;
	}
	
	s.powered = FLAME_ALL_MOTORS;
}

void CExercise_StAnkley::Update()
{
	float ankley_maxrefangle = 0.6;//0.6;
			
	if (Controller()->TimeElapsed() <= 1.0)
	{
		joints.l().ankley.qmot.ref = compute_quintic_spline( Controller()->TimeElapsed(), ankley_q_start[0], 0.0, 0.0, ankley_maxrefangle, 0.0, 0.0);
		joints.r().ankley.qmot.ref = compute_quintic_spline( Controller()->TimeElapsed(), ankley_q_start[1], 0.0, 0.0, ankley_maxrefangle, 0.0, 0.0);
	}
		else
		{
			if (Controller()->TimeElapsed() <= 2.0)
			{
				joints.l().ankley.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 1.0), ankley_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
				joints.r().ankley.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 1.0), ankley_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
			}
		}
	
	joints.Update(&s);
	
	if (Controller()->TimeElapsed() >= 2.0)
		Controller()->Transition(&gExercise_StEnd);
}

void CExercise_StHipyKneeAnkley_FindIndex::Init()
{
	logprintf("Start HipyKneeAnkley FindIndex\n");
	
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		// store angle hipy at start motion
		hipy_q_start[iLeg] = s.legs[iLeg].hipymot.q;
		
		joints.legs[iLeg].hipy.mMode = SEA_MODE_MOTANGLE;
		joints.legs[iLeg].hipy.motangCtrl.kp = 80;
		
		// store angle knee at start motion
		knee_q_start[iLeg] = s.legs[iLeg].kneemot.q;
					
		joints.legs[iLeg].knee.mMode = SEA_MODE_MOTANGLE;
		joints.legs[iLeg].knee.motangCtrl.kp = 25;
		
		// store angle ankley at start motion
		ankley_q_start[iLeg] = s.legs[iLeg].ankleymot.q;
				
		joints.legs[iLeg].ankley.mMode = SEA_MODE_MOTANGLE;
		joints.legs[iLeg].ankley.motangCtrl.kp = 40;
	}
	
	s.powered = FLAME_ALL_MOTORS;
	
}

void CExercise_StHipyKneeAnkley_FindIndex::Update()
{
	float hipy_maxrefangle = 0.3;
	float knee_maxrefangle = 0.9;
	float ankley_maxrefangle = 0.8;//0.6;
	
	if (Controller()->TimeElapsed() <= 1.0)
	{
		joints.l().hipy.qmot.ref = compute_quintic_spline( Controller()->TimeElapsed(), hipy_q_start[0], 0.0, 0.0, hipy_maxrefangle, 0.0, 0.0);		
		joints.l().knee.qmot.ref = knee_q_start[0];
		joints.l().ankley.qmot.ref = compute_quintic_spline( Controller()->TimeElapsed(), ankley_q_start[0], 0.0, 0.0, ankley_maxrefangle, 0.0, 0.0);
		
		joints.r().hipy.qmot.ref = hipy_q_start[1];
		joints.r().knee.qmot.ref = knee_q_start[1];
		joints.r().ankley.qmot.ref = ankley_q_start[1];
	}
	else
	{
		if (Controller()->TimeElapsed() <= 2.0)
		{
			joints.l().hipy.qmot.ref = compute_quintic_spline( 0.5*(Controller()->TimeElapsed() - 1.0), hipy_maxrefangle, 0.0, 0.0, -hipy_maxrefangle, 0.0, 0.0);
			joints.l().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 1.0), knee_q_start[0], 0.0, 0.0, knee_maxrefangle, 0.0, 0.0);
			joints.l().ankley.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 1.0), ankley_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);

			joints.r().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 1.0), hipy_q_start[1], 0.0, 0.0, hipy_maxrefangle, 0.0, 0.0);
			joints.r().knee.qmot.ref = knee_q_start[1];
			joints.r().ankley.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 1.0), ankley_q_start[1], 0.0, 0.0, ankley_maxrefangle, 0.0, 0.0);

		}
		else	
		{
			if (Controller()->TimeElapsed() <= 3.0)
			{
				joints.l().hipy.qmot.ref = compute_quintic_spline( 0.5*(Controller()->TimeElapsed() - 1.0), hipy_maxrefangle, 0.0, 0.0, -hipy_maxrefangle, 0.0, 0.0);
				joints.l().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 2.0), knee_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
				joints.l().ankley.qmot.ref = 0.0;
				
				joints.r().hipy.qmot.ref = compute_quintic_spline( 0.5*(Controller()->TimeElapsed() - 2.0), hipy_maxrefangle, 0.0, 0.0, -hipy_maxrefangle, 0.0, 0.0);
				joints.r().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 2.0), knee_q_start[1], 0.0, 0.0, knee_maxrefangle, 0.0, 0.0);
				joints.r().ankley.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 2.0), ankley_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);

			}
			else
			{
				if (Controller()->TimeElapsed() <= 4.0)
				{
					joints.l().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 3.0), -hipy_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
					joints.l().knee.qmot.ref = 0.0;
					joints.l().ankley.qmot.ref = 0.0;
					
					joints.r().hipy.qmot.ref = compute_quintic_spline( 0.5*(Controller()->TimeElapsed() - 2.0), hipy_maxrefangle, 0.0, 0.0, -hipy_maxrefangle, 0.0, 0.0);
					joints.r().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 3.0), knee_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
					joints.r().ankley.qmot.ref = 0.0;
				}
				else
				{
					if (Controller()->TimeElapsed() <= 5.0)
						{
							joints.l().hipy.qmot.ref = 0.0;
							joints.l().knee.qmot.ref = 0.0;
							joints.l().ankley.qmot.ref = 0.0;
											
							joints.r().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 4.0), -hipy_maxrefangle, 0.0, 0.0, 0.0, 0.0, 0.0);
							joints.r().knee.qmot.ref = 0.0;
							joints.r().ankley.qmot.ref = 0.0;
						}
				}
			}
		}
	}
	
	joints.Update(&s);
	
	if (Controller()->TimeElapsed() >= 5.0)
		Controller()->Transition(&gExercise_StHipyKneeAnkley_FindZeroTorque);
}

void CExercise_StHipyKneeAnkley_FindIndex::DeInit()
{
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		joints.legs[iLeg].hipy.motangCtrl.kp = 0;
		joints.legs[iLeg].knee.motangCtrl.kp = 0;
		joints.legs[iLeg].ankley.motangCtrl.kp = 0;
		
		// set motor angles equal to joint angles
		// permanently for following measurements
		params.legs[iLeg].hipymot.q.offset    +=  s.legs[iLeg].hipy.q   - s.legs[iLeg].hipymot.q;
		params.legs[iLeg].kneemot.q.offset    +=  s.legs[iLeg].knee.q   - s.legs[iLeg].kneemot.q;
		params.legs[iLeg].ankleymot.q.offset  +=  s.legs[iLeg].ankley.q - s.legs[iLeg].ankleymot.q;
		// and in this iteration to facilitate FindZeroTorque
		s.legs[iLeg].hipymot.q 		= s.legs[iLeg].hipy.q;
		s.legs[iLeg].kneemot.q		= s.legs[iLeg].knee.q;
		s.legs[iLeg].ankleymot.q	= s.legs[iLeg].ankley.q;
	}
	
	joints.Update(&s);	
}

void CExercise_StHipyKneeAnkley_FindZeroTorque::Init()
{
	logprintf("Start HipyKneeAnkley FindZeroTorque\n");
	
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		// store angle hipy at start motion
		hipy_q_start[iLeg] = s.legs[iLeg].hipymot.q;
		
		//logprintf("hipy_q_start = %f, hipy_q = %f, hipymot_q = %f\n", hipy_q_start[iLeg], s.legs[iLeg].hipymot.q, s.legs[iLeg].hipy.q);
		
		joints.legs[iLeg].hipy.mMode = SEA_MODE_MOTANGLE;
		joints.legs[iLeg].hipy.motangCtrl.kp = 80;
		
		// store angle knee at start motion
		knee_q_start[iLeg] = s.legs[iLeg].kneemot.q;
					
		joints.legs[iLeg].knee.mMode = SEA_MODE_MOTANGLE;
		joints.legs[iLeg].knee.motangCtrl.kp = 25;
		
/*		// store angle ankley at start motion
		ankley_q_start[iLeg] = s.legs[iLeg].ankleymot.q;
				
		joints.legs[iLeg].ankley.mMode = SEA_MODE_MOTANGLE;
		joints.legs[iLeg].ankley.motangCtrl.kp = 40;
*/		
		// store angle ankley at start motion
		ankley_q_start[iLeg] = s.legs[iLeg].ankleymot.q;
						
		joints.legs[iLeg].ankley.mMode = SEA_MODE_TORQUE;
		joints.legs[iLeg].ankley.trqCtrl.kp = 0;
		joints.legs[iLeg].ankley.trqCtrl.kd = 0;
	}
	
	// define sum values for averaging difference tau and tauSEA
	sum_l_hipy_taudiff = 0.0;
	sum_r_hipy_taudiff = 0.0;
	sum_l_knee_taudiff = 0.0;
	sum_r_knee_taudiff = 0.0;
	sum_l_ankley_taudiff = 0.0;
	sum_r_ankley_taudiff = 0.0;
		
	count_taudiff = 0.0;
	
}

void CExercise_StHipyKneeAnkley_FindZeroTorque::Update()
{
	
	//TODO: ankle zero torque still needs to be defined
	
	float hipy_backangle = 0.3;
	float hipy_frontangle = -0.4;
	float knee_backangle = 0.1;
	float knee_frontangle = 0.0;
	float ankle_torque = 2.0;
	
	if (Controller()->TimeElapsed() <= 0.5)
	{
		joints.l().hipy.qmot.ref = compute_quintic_spline( 2*Controller()->TimeElapsed(), hipy_q_start[0], 0.0, 0.0, 0.0, 0.0, 0.0);		
		joints.l().knee.qmot.ref = compute_quintic_spline( 2*Controller()->TimeElapsed(), knee_q_start[0], 0.0, 0.0, 0.0, 0.0, 0.0);
		joints.l().ankley.tauSEA.ref = compute_quintic_spline( 0.66*Controller()->TimeElapsed(), 0.0, 0.0, 0.0, ankle_torque, 0.0, 0.0);
		
		joints.r().hipy.qmot.ref = compute_quintic_spline( 2*Controller()->TimeElapsed(), hipy_q_start[1], 0.0, 0.0, 0.0, 0.0, 0.0);		
		joints.r().knee.qmot.ref = compute_quintic_spline( 2*Controller()->TimeElapsed(), knee_q_start[1], 0.0, 0.0, 0.0, 0.0, 0.0);
		joints.r().ankley.tauSEA.ref = compute_quintic_spline( 0.66*Controller()->TimeElapsed(), 0.0, 0.0, 0.0, ankle_torque, 0.0, 0.0);
	}
	else
	{
		if (Controller()->TimeElapsed() <= 1.5)
		{
			joints.l().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 0.5), 0.0, 0.0, 0.0, hipy_backangle, 0.0, 0.0);
			joints.l().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 0.5), 0.0, 0.0, 0.0, knee_backangle, 0.0, 0.0);
			joints.l().ankley.tauSEA.ref = compute_quintic_spline( 0.66*Controller()->TimeElapsed(), 0.0, 0.0, 0.0, ankle_torque, 0.0, 0.0);
			
			joints.r().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 0.5), 0.0, 0.0, 0.0, hipy_frontangle, 0.0, 0.0);
			joints.r().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 0.5), 0.0, 0.0, 0.0, knee_frontangle, 0.0, 0.0);
			joints.r().ankley.tauSEA.ref = compute_quintic_spline( 0.66*Controller()->TimeElapsed(), 0.0, 0.0, 0.0, ankle_torque, 0.0, 0.0);
		}
		else
		{
			if (Controller()->TimeElapsed() <= 2.5)
			{
				joints.l().ankley.tauSEA.ref = ankle_torque;
				joints.r().ankley.tauSEA.ref = ankle_torque;
				// wait				
			} 
			else
			{
				if (Controller()->TimeElapsed() <= 3.5)
				{
					joints.l().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 2.5), hipy_backangle, 0.0, 0.0, 0.0, 0.0, 0.0);
					joints.l().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 2.5), knee_backangle, 0.0, 0.0, 0.0, 0.0, 0.0);

					joints.r().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 2.5), hipy_frontangle, 0.0, 0.0, 0.0, 0.0, 0.0);
					joints.r().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 2.5), knee_frontangle, 0.0, 0.0, 0.0, 0.0, 0.0);
					
					joints.l().ankley.tauSEA.ref = ankle_torque;
					joints.r().ankley.tauSEA.ref = ankle_torque;
				}
				else
				{
					if (Controller()->TimeElapsed() <= 4.5)
					{
						joints.l().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 3.5), 0.0, 0.0, 0.0, hipy_frontangle, 0.0, 0.0);
						joints.l().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 3.5), 0.0, 0.0, 0.0, knee_frontangle, 0.0, 0.0);
						
						joints.r().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 3.5), 0.0, 0.0, 0.0, hipy_backangle, 0.0, 0.0);
						joints.r().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 3.5), 0.0, 0.0, 0.0, knee_backangle, 0.0, 0.0);
						
						joints.l().ankley.tauSEA.ref = ankle_torque;
						joints.r().ankley.tauSEA.ref = ankle_torque;
					}
					else
					{
						if (Controller()->TimeElapsed() <= 5.5)
						{
							joints.l().ankley.tauSEA.ref = ankle_torque;
							joints.r().ankley.tauSEA.ref = ankle_torque;
							// wait
						} 
						else
						{
							if (Controller()->TimeElapsed() <= 6.5)
							{
								joints.l().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 5.5), hipy_frontangle, 0.0, 0.0, 0.0, 0.0, 0.0);
								joints.l().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 5.5), knee_frontangle, 0.0, 0.0, 0.0, 0.0, 0.0);
								joints.l().ankley.tauSEA.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 5.5), ankle_torque, 0.0, 0.0, 0.0, 0.0, 0.0);

								joints.r().hipy.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 5.5), hipy_backangle, 0.0, 0.0, 0.0, 0.0, 0.0);
								joints.r().knee.qmot.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 5.5), knee_backangle, 0.0, 0.0, 0.0, 0.0, 0.0);
								joints.r().ankley.tauSEA.ref = compute_quintic_spline( (Controller()->TimeElapsed() - 5.5), ankle_torque, 0.0, 0.0, 0.0, 0.0, 0.0);
							}
						}
					}
				}
			}
		}
	}
	
	joints.Update(&s);
	
	// when hanging still get difference between tauSEA and tau to find zero torque later
	if ((Controller()->TimeElapsed() > 1.5 && Controller()->TimeElapsed() <= 2.5) ||
			(Controller()->TimeElapsed() > 4.5 && Controller()->TimeElapsed() <= 5.5))
	{
		sum_l_hipy_taudiff += joints.l().hipy.tauSEA.cur - s.l().hipy.tau;
		sum_r_hipy_taudiff += joints.r().hipy.tauSEA.cur - s.r().hipy.tau;
		sum_l_knee_taudiff += joints.l().knee.tauSEA.cur - s.l().knee.tau;
		sum_r_knee_taudiff += joints.r().knee.tauSEA.cur - s.r().knee.tau;
		sum_l_ankley_taudiff += joints.l().ankley.tauSEA.cur - 0.0;
		sum_r_ankley_taudiff += joints.r().ankley.tauSEA.cur - 0.0;
		
		count_taudiff++;				
	}
	
	if (Controller()->TimeElapsed() >= 6.5)
		Controller()->Transition(&gExercise_StTorqueControl);
	
}

void CExercise_StHipyKneeAnkley_FindZeroTorque::DeInit()
{
	for (int iLeg=0; iLeg<2; iLeg++)
		{
			joints.legs[iLeg].hipy.motangCtrl.kp = 0;
			joints.legs[iLeg].knee.motangCtrl.kp = 0;
			//joints.legs[iLeg].ankley.mMode = SEA_MODE_MOTANGLE;
			joints.legs[iLeg].ankley.motangCtrl.kp = 0;
		}
	
	// adapt offset motor angle according to diff tauSEA and tau
	params.l().hipymot.q.offset    -=  sum_l_hipy_taudiff/(count_taudiff*joints.l().hipy.stiffSEA);
	params.l().kneemot.q.offset    -=  sum_l_knee_taudiff/(count_taudiff*joints.l().knee.stiffSEA);
	params.l().ankleymot.q.offset  -=  sum_l_ankley_taudiff/(count_taudiff*joints.l().ankley.stiffSEA);
	params.r().hipymot.q.offset    -=  sum_r_hipy_taudiff/(count_taudiff*joints.r().hipy.stiffSEA);
	params.r().kneemot.q.offset    -=  sum_r_knee_taudiff/(count_taudiff*joints.r().knee.stiffSEA);
	params.r().ankleymot.q.offset  -=  sum_r_ankley_taudiff/(count_taudiff*joints.r().ankley.stiffSEA);
	
	joints.Update(&s);
}

void CExercise_StTorqueControl::Init()
{
	logprintf("Start Torque Control\n");
	logprintf("Press second button to go to Angle Control\n");
	logprintf("If third toggle is down prepare for standing, if it is up prepare for walking\n");
		
	for (int iLeg=0; iLeg<2; iLeg++)
	{					
		joints.legs[iLeg].knee.mMode = SEA_MODE_TORQUE;
		joints.legs[iLeg].knee.trqCtrl.kp = 10;
		joints.legs[iLeg].knee.trqCtrl.kd = 0.25;//0.25
		
		joints.legs[iLeg].hipy.mMode = SEA_MODE_TORQUE;
		joints.legs[iLeg].hipy.trqCtrl.kp = 10;
		joints.legs[iLeg].hipy.trqCtrl.kd = 0.25;//0.5;
		
		joints.legs[iLeg].ankley.mMode = SEA_MODE_TORQUE;
		joints.legs[iLeg].ankley.trqCtrl.kp = 10;
		joints.legs[iLeg].ankley.trqCtrl.kd = 1;//0.5;
	}
	
	s.powered = FLAME_ALL_MOTORS;
}

void CExercise_StTorqueControl::Update()
{
/*	if ( FLAME_TOGGLE_UP( s.front_panel_sw, TOGGLE2 ) )
		s.powered = FLAME_HIPX_MOTOR;
	else
		s.powered = FLAME_ALL_MOTORS;
*/	
	// compensate for passive spring in ankle
	joints.l().ankley.tauSEA.ref = (s.l().ankley.q + 0.38)*5;
	joints.r().ankley.tauSEA.ref = (s.r().ankley.q + 0.38)*5;
	
	joints.Update(&s);
	
	static int debounce = 0;
	if ( FLAME_PUSHBUTTON_PRESSED( s.front_panel_sw, PUSHBUTTON1 ))
	{
		debounce++;
		if ( debounce == 5 )
		{
			Controller()->Transition(&gExercise_StAngleControl);
		}
	}
}

void CExercise_StAngleControl::Init()
{
	logprintf("Start Angle Control\n");
		
	for (int iLeg=0; iLeg<2; iLeg++)
	{					
		joints.legs[iLeg].hipy.mMode = SEA_MODE_ANGLE;
		joints.legs[iLeg].hipy.angCtrl.kp = 40;
		joints.legs[iLeg].hipy.angCtrl.kd = 4.0;
		
		joints.legs[iLeg].knee.mMode = SEA_MODE_ANGLE;
		joints.legs[iLeg].knee.angCtrl.kp = 10;
		joints.legs[iLeg].knee.angCtrl.kd = 1.0;
		
		joints.legs[iLeg].ankley.mMode = SEA_MODE_ANGLEPLUSTORQUE;
		joints.legs[iLeg].ankley.angCtrl.kp = 4;
		joints.legs[iLeg].ankley.angCtrl.kd = 0;//0.2;
		
		joints.legs[iLeg].knee.q.ref = 0;
		
		// store angle ankley at start motion
		ankley_q_start[iLeg] = s.legs[iLeg].ankley.q;
	}
		
	//ff_l_hipy.Init(0.0, 1.0, ff_l_hipy_AngleControl, sizeof(ff_l_hipy_AngleControl)/sizeof(float));
	//ff_r_hipy.Init(0.0, 1.0, ff_r_hipy_AngleControl, sizeof(ff_r_hipy_AngleControl)/sizeof(float));
	
	s.powered = FLAME_ALL_MOTORS;

}

void CExercise_StAngleControl::Update()
{
	//joints.interleg.ff = ff_l_hipy.GetValue( Controller()->TimeElapsed() );

	// compensate for passive spring in ankle
	joints.l().ankley.tauSEA.ref = (s.l().ankley.q + 0.38)*5;
	joints.r().ankley.tauSEA.ref = (s.r().ankley.q + 0.38)*5;
		
	for (int iLeg=0; iLeg<2; iLeg++)
	{							
		// and apply angle control to zero angle
		joints.legs[iLeg].ankley.q.ref = compute_quintic_spline( Controller()->TimeElapsed(), ankley_q_start[iLeg], 0.0, 0.0, 0, 0.0, 0.0);	
		
		// ff to overstretch knee
		joints.legs[iLeg].knee.angCtrl.ff = -1.5;
	}
	
	// if prepare for walking, get legs in start walking angle
	if ( FLAME_TOGGLE_UP( s.front_panel_sw, TOGGLE2 ) )
	{
		joints.l().hipy.q.ref = compute_quintic_spline( Controller()->TimeElapsed(), 0.0, 0.0, 0.0, 0.2, 0.0, 0.0);
		joints.r().hipy.q.ref = compute_quintic_spline( Controller()->TimeElapsed(), 0.0, 0.0, 0.0, -0.4, 0.0, 0.0);
		
		// define ff to deal with gravity
		joints.l().hipy.angCtrl.ff = joints.l().hipy.q.ref*8.0;
		joints.r().hipy.angCtrl.ff = joints.r().hipy.q.ref*8.0;
	}
	
	joints.Update(&s);
	
	if (Controller()->TimeElapsed() >= 1)
		Controller()->Transition(&gExercise_StEnd);
}
