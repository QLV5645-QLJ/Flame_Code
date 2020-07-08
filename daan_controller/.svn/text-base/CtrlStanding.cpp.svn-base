#include "CtrlStanding.h"

#include <math.h>

// Disable warnings for 'double to float' issues
#pragma warning( disable : 4244 4305 )

// The global objects
CStanding_StSleep			gStanding_StSleep;
CStanding_StStandControl	gStanding_StStandControl;

CStandingController			gStandingController;

//****************************************************************
// Helper functions
//****************************************************************


// determine which controller to use
inline static char determine_controller(void) 
{
  static char controller_choice = S_UNKNOWN;
  if ((s.l().foot.back.count > 30 || s.l().foot.front.count > 30 ) && (s.r().foot.front.count > 30 || s.r().foot.back.count > 30))
       controller_choice = S_POSITION;
  
  if ((s.l().foot.back.count <-30 && s.l().foot.front.count <-30) || (s.r().foot.front.count < -30 && s.r().foot.back.count < -30))
      controller_choice = S_TORQUE; 

  return ( controller_choice );
}


//****************************************************************/
// The standing controller
//****************************************************************/

void CStandingController::Init()
{

}

void CStandingController::Deinit()
{
	// Nothing to do here
}

//****************************************************************/
// Controller states
//****************************************************************/

void CStanding_StSleep::Init()
{
	joints.hipx.q.ref = 0.15;//0.13;//0.12;
	
	for (int iLeg=0; iLeg<2; iLeg++)
	{					
		joints.legs[iLeg].knee.mMode = SEA_MODE_ANGLE;
		joints.legs[iLeg].knee.angCtrl.kp = 10;
		joints.legs[iLeg].knee.angCtrl.kd = 1.0;
		joints.legs[iLeg].knee.trqCtrl.kp = 10;
		joints.legs[iLeg].knee.trqCtrl.kd = 0.25;//0.25
		
		joints.legs[iLeg].hipy.mMode = SEA_MODE_ANGLE;
		joints.legs[iLeg].hipy.angCtrl.kp = 20;//20;
		joints.legs[iLeg].hipy.angCtrl.kd = 2.0;
		joints.legs[iLeg].hipy.trqCtrl.kp = 10;
		joints.legs[iLeg].hipy.trqCtrl.kd = 0.25;//0.5;
		
		joints.legs[iLeg].ankley.mMode = SEA_MODE_ANGLEPLUSTORQUE;
		joints.legs[iLeg].ankley.angCtrl.kp = 4;
		joints.legs[iLeg].ankley.angCtrl.kd = 0;//0.2;
		joints.legs[iLeg].ankley.trqCtrl.kp = 10;
		joints.legs[iLeg].ankley.trqCtrl.kd = 1;//0.5;
		
		joints.legs[iLeg].knee.q.ref = 0;
		joints.legs[iLeg].ankley.q.ref = 0;//-0.05;

	}
	
	int iLeg=0;

	logprintf("standing controller initializing STAND state\nPut Flame on ground to go to strong controller.\n");

//	s.powered = FLAME_ALL_MOTORS;
}

void CStanding_StSleep::Update()
{
	// make all joints limp by default
	//joints.Disable();

	for (int iLeg=0; iLeg<2; iLeg++)
	{
		// compensate for passive spring in ankle
			joints.l().ankley.tauSEA.ref = (s.l().ankley.q + 0.38)*5;
			joints.r().ankley.tauSEA.ref = (s.r().ankley.q + 0.38)*5;
		
		// ff to overstretch knee
		joints.legs[iLeg].knee.angCtrl.ff = -3;

	}
	
	joints.Update(&s);

	//s.powered = 0;

	// If Flame is placed on ground, go to the normal PD controller
	if ( s.l().foot.back.state && s.r().foot.back.state )
		Controller()->Transition(&gStanding_StStandControl);
}

void CStanding_StStandControl::Init()
{
	logprintf("STAND PD CONTROL state\nPress button 1 to go start walking, Press button 2 to increase the Kpp\n");

	joints.left_is_stance = true;
	
	joints.upperbody.q.ref = 0.1;
}

void CStanding_StStandControl::Update()
{	
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		// compensate for passive spring in ankle
		joints.l().ankley.tauSEA.ref = (s.l().ankley.q + 0.38)*5;
		joints.r().ankley.tauSEA.ref = (s.r().ankley.q + 0.38)*5;
		
		// ff to overstretch knee
		joints.legs[iLeg].knee.angCtrl.ff = -5;
		
	}

			
	// if both feet are on the ground, a position controller can be used. This is not possible when the 
	// feet are not on the ground because the gains are way to high for the small mass of the feet.
	if ( determine_controller() == S_POSITION )
	{ 
						
		// position controller
		for (int iLeg=0; iLeg<2; iLeg++)
		{
			//joints.legs[iLeg].hipy.angCtrl.kp = 40.0; 
			//joints.legs[iLeg].hipy.angCtrl.kd = 4.0;
			//joints.legs[iLeg].hipy.mMode = SEA_MODE_ANGLE;
						
			joints.upperbody.Disable();
			
			joints.stance().hipy.angCtrl.kp = 80;// 
			joints.stance().hipy.angCtrl.kd = 8.0;
			joints.stance().hipy.mMode = SEA_MODE_ANGLE;
			
			joints.swing().hipy.mMode = SEA_MODE_TORQUE;
						
			joints.legs[iLeg].ankley.angCtrl.kp = 120.0; 
			joints.legs[iLeg].ankley.angCtrl.kd = 12.0; 
//			joints.legs[iLeg].ankley.mMode = SEA_MODE_ANGLEPLUSTORQUE;
		}
		
		if ( FLAME_TOGGLE_UP( s.front_panel_sw, TOGGLE2 ) )
		{
			for (int iLeg=0; iLeg<2; iLeg++)
			{
				//joints.legs[iLeg].hipy.tauSEA.ref = 40.0*(s.imu.pitch-0.1)+4.0*(s.imu.pitchd);
				//joints.legs[iLeg].hipy.mMode = SEA_MODE_TORQUE;
					
				joints.upperbody.kp = 80.0;
				joints.upperbody.kd = 8.0;
				joints.stance().hipy.mMode = SEA_MODE_TORQUE;
				
				joints.swing().hipy.tauSEA.ref = 0;
				joints.swing().hipy.mMode = SEA_MODE_TORQUE;
				
				joints.legs[iLeg].ankley.angCtrl.kp = 120.0; 
				joints.legs[iLeg].ankley.angCtrl.kd = 12.0; 
	//			joints.legs[iLeg].ankley.mMode = SEA_MODE_ANGLEPLUSTORQUE;
			}
		}
		
	}
	else if ( determine_controller() == S_TORQUE )
	{
		// Torque controller
		for (int iLeg=0; iLeg<2; iLeg++)
		{
			joints.legs[iLeg].hipy.angCtrl.kp = 20.0; 
			joints.legs[iLeg].hipy.angCtrl.kd = 2.0;
			joints.legs[iLeg].hipy.mMode = SEA_MODE_ANGLE;
						
			joints.legs[iLeg].ankley.angCtrl.kp  = 4.0;
			joints.legs[iLeg].ankley.angCtrl.kd  = 0;//0.2;
//			joints.legs[iLeg].ankley.mMode = SEA_MODE_ANGLEPLUSTORQUE;
		}
	}

	joints.Update(&s);
}


// Get default warnings again for 'double to float' issues
#pragma warning( default : 4244 4305 )
