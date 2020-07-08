#include "CtrlWalking.h"

#include <memory.h>	// For the use of memset()
#include <math.h>

// support for the IMU
#include <xsens/xsens.h>

// Disable warnings for 'double to float' issues
#pragma warning( disable : 4244 4305 )
// Globals
// Walking states
CWalking_StGetReady				gWalking_StGetReady;
CWalking_StPushoff				gWalking_StPushoff;
CWalking_StSwing				gWalking_StSwing;
CWalking_StStanceFootRelease	gWalking_StStanceFootRelease;
CWalking_StCrash				gWalking_StCrash;


CWalkingController		gWalkingController;

int count_contact;

//float ff_swing_interleg_AngleControl[] = {-0.5, -1.5, -2.5, -1.5, 0.0, 2.5, 3.0, -2.5, -2.5, -1.8, -1.8};
float ff_swing_interleg_AngleControl[] = {-1.0, -2.0, -3.0, -1.0, 5.0, 5.0, -2.0, -5.0, -4.5, -1.8, -1.8};
float time_elapsed_pushoff;
float local_stance_control_maxgain;
float roll_start_step;

#ifndef max
#define max(a, b)  (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a, b)  (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************/
// Helper functions
/****************************************************************/

CWalkingState::CWalkingState()
{
	mT1 = 0;
	mT2 = 0;
}


//****************************************************************/
// The walking controller
//****************************************************************/

void CWalkingController::Init()
{
	// Set the starting state
	Transition(&gWalking_StGetReady);
}

void CWalkingController::Deinit()
{
	// Nothing to do here
}

void CWalkingController::SetStanceLeg(bool left_is_stance)
{
	s.left_is_stance = left_is_stance;
	joints.left_is_stance = left_is_stance;
}

bool CWalkingController::GetStanceLeg()
{
	return s.left_is_stance;
}

//****************************************************************/
// The states of the walking controller
//****************************************************************/

void CWalking_StGetReady::Init()
{
	logprintf("Begin start walking, wait for left foot to be fully on the ground\n");

	joints.hipx.kp = 1200;//2400;//1200;
	joints.hipx.kd = 80;//240;//80;
	joints.hipx.q.ref = 0.10;//0.11;//0.12;

	for (int iLeg=0; iLeg<2; iLeg++)
	{
		joints.legs[iLeg].knee.mMode = SEA_MODE_ANGLE;
		joints.legs[iLeg].knee.angCtrl.kp = 10;
		joints.legs[iLeg].knee.angCtrl.kd = 0.5;
		joints.legs[iLeg].knee.trqCtrl.kp = 10;
		joints.legs[iLeg].knee.trqCtrl.kd = 0.25;//0.25;//0.25
		
		joints.legs[iLeg].hipy.mMode = SEA_MODE_ANGLE;
		joints.legs[iLeg].hipy.angCtrl.kp = 40;
		joints.legs[iLeg].hipy.angCtrl.kd = 4.0;
		joints.legs[iLeg].hipy.trqCtrl.kp = 20;//10;
		joints.legs[iLeg].hipy.trqCtrl.kd = 0.25;//0.25;//0.5;
		
		joints.legs[iLeg].ankley.mMode = SEA_MODE_ANGLEPLUSTORQUE;
		joints.legs[iLeg].ankley.angCtrl.kp = 4;
		joints.legs[iLeg].ankley.angCtrl.kd = 0;//0.2;
		joints.legs[iLeg].ankley.trqCtrl.kp = 10;
		joints.legs[iLeg].ankley.trqCtrl.kd = 1;//0.5;
		
		joints.legs[iLeg].knee.q.ref = 0;
		joints.legs[iLeg].ankley.q.ref = 0;//-0.05;
	}

	joints.l().hipy.q.ref = 0.2;
	joints.r().hipy.q.ref = -0.4;
	
	joints.upperbody.q.ref = 0.005 ;//-0.005;//-0.01;//0.005;//-0.02;//0.0;//0.05;
	joints.interleg.q.ref = -0.6;
	
	// initialize counter
	count_contact = 0;
	touched_down = 0;

	// reset orientation xsens
	reset_orientation_xsens_IMU();
	
	// The left leg (1) is stance leg
	Controller()->SetStanceLeg(STANCE_LEG_LEFT);
}

void CWalking_StGetReady::Update()
{
	
	if ( s.stance().foot.back.state || s.stance().foot.front.state )
		touched_down = 1;
	
	if ( touched_down == 1 )
	{
		// detect foot release and react
		if ( !(s.stance().foot.back.state || s.stance().foot.front.state) )
			count_contact++;
		else
			count_contact = 0;
		
		if ( count_contact == 200 )
			touched_down = 0;
	}
		
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		joints.upperbody.Disable();
		joints.interleg.Disable();
					
		joints.legs[iLeg].hipy.angCtrl.kp = 40.0; 
		joints.legs[iLeg].hipy.angCtrl.kd = 4.0;
		joints.legs[iLeg].hipy.mMode = SEA_MODE_ANGLE;
		
		joints.legs[iLeg].ankley.angCtrl.kp = 4;
		joints.legs[iLeg].ankley.angCtrl.kd = 0;
		
		// compensate for passive spring in ankle
		//joints.legs[iLeg].ankley.tauSEA.ref = (s.legs[iLeg].ankley.q + 0.38)*5;
		joints.l().ankley.tauSEA.ref = (s.l().ankley.q + 0.38)*5;
		joints.r().ankley.tauSEA.ref = (s.r().ankley.q + 0.38)*5;
		
		// define ff to deal with gravity
		joints.legs[iLeg].hipy.angCtrl.ff = joints.legs[iLeg].hipy.q.ref*8.0;
		
		// ff to overstretch knee
		joints.legs[iLeg].knee.angCtrl.ff = -5;
	}

	if (touched_down == 1) 
	{
		
		joints.stance().ankley.angCtrl.kp = 80;//80;//100;//120;
		joints.swing().ankley.angCtrl.kp = 80;//80;
		
		joints.swing().ankley.q.ref = 0.05;
		
		if (s.stance().ankley.q < -0.1)
			joints.stance().ankley.turbo_boost = 1;
		else
			joints.stance().ankley.turbo_boost = 0;
		
		joints.upperbody.kp = 90.0;
		joints.upperbody.kd = 8.0;//4.0;
		joints.stance().hipy.mMode = SEA_MODE_TORQUE;
		
		joints.interleg.kp = 40.0;
		joints.interleg.kd = 2.0;
		joints.swing().hipy.mMode = SEA_MODE_TORQUE;
					
		if (s.swing().foot.back.state)
			Controller()->Transition(&gWalking_StPushoff);
	}
	
	joints.Update(&s);
	
}

void CWalking_StPushoff::Init()
{
	logprintf("Switch stance leg and start pushoff\n");
	
	// initialize counter
	count_contact = 0;
	
	Controller()->SetStanceLeg( !Controller()->GetStanceLeg() );
	
	if ( Controller()->GetStanceLeg() == 1 )
		joints.hipx.q.ref = joints.hipx.q.ref;//cur;//0.12;//0.11; // when left just landed
	else
		joints.hipx.q.ref = joints.hipx.q.ref;//cur;//0.12;//0.11; // when right just landed
			
	joints.upperbody.kp = 90.0;
	joints.upperbody.kd = 8.0;//8.0;//4.0;
	joints.stance().hipy.mMode = SEA_MODE_TORQUE;
	
	joints.interleg.kp = 40.0;
	joints.interleg.kd = 2.0;
	joints.swing().hipy.mMode = SEA_MODE_TORQUE;
			
	joints.swing().ankley.angCtrl.kp = 80;//80;//100;//120;
	// get derivative action out of torque loop to ensure fast pushoff, also apply turbo_boost
	joints.swing().ankley.trqCtrl.kd = 0.1;//0.5;
	joints.swing().ankley.tauSEA.alpha	= 1.0/16.0;//1.0/32.0;
	joints.swing().ankley.turbo_boost = 1;
	
	joints.stance().ankley.q.ref = 0;
	local_stance_control_maxgain = 40;//120;//80;//100;
	//joints.swing().ankley.q.ref = 0.15;//0.075;//0.035;//0.05;//0.2;
	if ( Controller()->GetStanceLeg() == 1 )
		joints.swing().ankley.q.ref = 0.05;//0.05;//0.06;//0.18;//0.18; // when left just landed
	else
		joints.swing().ankley.q.ref = 0.05;//0.05;//0.06;//0.18;//0.18; // when right just landed
	
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		joints.legs[iLeg].knee.q.ref = 0;
	}
	
	//joints.upperbody.q.ref = 0;//0.05;
	
	joints.interleg.q.ref = 0.6;
	joints.interleg.q.SetZeroDerivRef();
	
	roll_start_step = s.imu.roll;
}

void CWalking_StPushoff::Update()
{
	// switch between landing and other ankle stiffness
	if ( s.stance().ankley.q > 0 )
		joints.stance().ankley.angCtrl.kp = 5;//5;//30;
	else
		joints.stance().ankley.angCtrl.kp = 80;//80;//100;//120;
	
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		// compensate for passive spring in ankle
		//joints.legs[iLeg].ankley.tauSEA.ref = (s.legs[iLeg].ankley.q + 0.38)*5;
		joints.l().ankley.tauSEA.ref = (s.l().ankley.q + 0.38)*5;
		joints.r().ankley.tauSEA.ref = (s.r().ankley.q + 0.38)*5;
	}
	
	// local stance ankle control
	time_elapsed_step = Controller()->TimeElapsed();
	time_elapsed_pushoff = time_elapsed_step;
	if ( time_elapsed_step < 0.2 )
		local_stance_control_gain = local_stance_control_maxgain*time_elapsed_step*5;
	else if ( (time_elapsed_step >= 0.2) && (time_elapsed_step < 0.5) )
		local_stance_control_gain = local_stance_control_maxgain;
	else if ( (time_elapsed_step >= 0.5) && (time_elapsed_step < 0.7) )
		local_stance_control_gain = local_stance_control_maxgain*(0.7-time_elapsed_step)*5;
	else
		local_stance_control_gain = 0;
	
	local_stance_control_des = 0.3 - 0.6*(time_elapsed_step*1.25);
	
	joints.stance().ankley.tauSEA.ref += local_stance_control_gain*(local_stance_control_des - s.stance().ankley.q);
	
	
	
	
	// ff to overstretch knee
	joints.stance().knee.angCtrl.ff = -5;
	joints.swing().knee.angCtrl.ff = -2;
	
	joints.Update(&s);
	
	// detect foot release and react
	if ( !(s.stance().foot.back.state || s.stance().foot.front.state) && !(s.swing().foot.front.state) )
		count_contact++;
	else
		count_contact = 0;
	
	if ( count_contact == 500 )
		Controller()->Transition(&gWalking_StStanceFootRelease);
	
	// detect end of pushoff and goto swing
	// ankle torque is below certain level and decreasing
	//if ( (joints.swing().ankley.tauSEA.ref - (s.swing().ankley.q + 0.38)*5 < 15) &&
	//		(joints.swing().ankley.tauSEA.d.ref < 0) )
	//	Controller()->Transition(&gWalking_StSwing);
	if ( ( (joints.swing().ankley.q.ref - joints.swing().ankley.q.cur)*joints.swing().ankley.angCtrl.kp < 10) &&
			(joints.swing().ankley.tauSEA.d.ref < 0) )
		Controller()->Transition(&gWalking_StSwing);
	
}

void CWalking_StSwing::Init()
{
	logprintf("Start swing\n");
			
	//joints.upperbody.kp = 80.0;
	//joints.upperbody.kd = 4.0;
	//joints.stance().hipy.mMode = SEA_MODE_TORQUE;
	
	//joints.interleg.kp = 40.0;
	//joints.interleg.kd = 2.0;
	//joints.swing().hipy.mMode = SEA_MODE_TORQUE;
			
	joints.swing().ankley.angCtrl.kp = 80;//80;//?!
	joints.stance().ankley.q.ref = 0;
	joints.swing().ankley.q.ref = -0.2;
	
	// reactivate derivative action in torque loop and switch off turbo_boost
	joints.swing().ankley.trqCtrl.kd = 1;
	joints.swing().ankley.tauSEA.alpha	= 1.0/16.0;
	joints.swing().ankley.turbo_boost = 0;
	
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		joints.legs[iLeg].knee.q.ref = 0;
	}
	
	/*// ensure fast (no filter) knee actuation in spite of overstretching ff that was present
	joints.swing().knee.tauSEA.ref = 0.0;
	joints.swing().knee.tauSEA.SetZeroDerivRef();
	*/
	//joints.upperbody.q.ref = 0;//0.05;
	
	joints.interleg.q.ref = 0.6;
	joints.interleg.q.SetZeroDerivRef();
	
	joints.hipx_footplacement = joints.hipx.q.ref;
	
	ff_swing_interleg.Init(0.0, 0.9, ff_swing_interleg_AngleControl, sizeof(ff_swing_interleg_AngleControl)/sizeof(float));
	
	hipx_ref_startstep = joints.hipx.q.ref;
	footplacement_determined = 0;
}

void CWalking_StSwing::Update()
{
	// switch between landing and other ankle stiffness
	if ( s.stance().ankley.q > 0 )
		joints.stance().ankley.angCtrl.kp = 5;//5;//30;
	else
		joints.stance().ankley.angCtrl.kp = 80;//80;//100;//120;
	
	if (s.stance().ankley.q < -0.1)
		joints.stance().ankley.turbo_boost = 1;
	else
		joints.stance().ankley.turbo_boost = 0;
	
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		// compensate for passive spring in ankle
		//joints.legs[iLeg].ankley.tauSEA.ref = (s.legs[iLeg].ankley.q + 0.38)*5;
		joints.l().ankley.tauSEA.ref = (s.l().ankley.q + 0.38)*5;
		joints.r().ankley.tauSEA.ref = (s.r().ankley.q + 0.38)*5;
	}
	
	// local stance ankle control
	time_elapsed_step = time_elapsed_pushoff+Controller()->TimeElapsed();
	if ( time_elapsed_step < 0.2 )
		local_stance_control_gain = local_stance_control_maxgain*time_elapsed_step*5;
	else if ( (time_elapsed_step >= 0.2) && (time_elapsed_step < 0.5) )
		local_stance_control_gain = local_stance_control_maxgain;
	else if ( (time_elapsed_step >= 0.5) && (time_elapsed_step < 0.7) )
		local_stance_control_gain = local_stance_control_maxgain*(0.7-time_elapsed_step)*5;
	else
		local_stance_control_gain = 0;
	
	local_stance_control_des = 0.3 - 0.6*(time_elapsed_step*1.25);
	
	joints.stance().ankley.tauSEA.ref += max( min(local_stance_control_gain*(local_stance_control_des - s.stance().ankley.q), 40), -20) ;

		
	if ( Controller()->TimeElapsed() > 0.3 && joints.swing().knee.q.cur < 0.7 )
	{
		if ( Controller()->GetStanceLeg() == 1 )
			joints.swing().ankley.q.ref = 0.0;
		else
			joints.swing().ankley.q.ref = 0.0;
	}
	// ff to overstretch stance knee
	joints.stance().knee.angCtrl.ff = -5.5;
	
	// generate knee swing trajectory
	if ( Controller()->TimeElapsed() < 0.2 )//0.3
		joints.swing().knee.q.ref = compute_quintic_spline( 5*Controller()->TimeElapsed(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
	else if ( Controller()->TimeElapsed() < 0.5 )//0.7
		joints.swing().knee.q.ref = compute_quintic_spline( 3.33*(Controller()->TimeElapsed()-0.2), 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	else
	{
		joints.swing().knee.q.ref = 0.0;
		joints.swing().knee.angCtrl.ff = -5;
	}
	
	// generate hip (interleg) swing trajectory
	if ( Controller()->TimeElapsed() < 0.6 )
		joints.interleg.q.ref = compute_quintic_spline( 1.67*(Controller()->TimeElapsed()), 0.6, 0.0, -25.0, -0.6, 0.0, 0.0);
	else
		joints.interleg.q.ref = -0.6;
	// and add ff
	joints.interleg.ff = ff_swing_interleg.GetValue( Controller()->TimeElapsed() );
	
	// make hipx motion, dependent on actual interleg angle and with footplacement controller
	static float hipx_footplacement;
	float roll_des;
	float rolld_des;
	static float roll_diff_sum;
	static float rolld_diff_sum;
	static int count;
	
	roll_des = compute_quadratic_spline( time_elapsed_step*1.25, -0.16, 0.3, -0.06 );
	rolld_des = 2*(-0.06 - -0.16 - 0.3)*(time_elapsed_step*1.25) + 0.3; // velocity belonging to quadratic spline above
	
	if ( Controller()->GetStanceLeg() == 1 )
	{
		joints.roll_des = -roll_des;
		joints.rolld_des = -rolld_des;
		
		joints.stancerolld_filt = joints.rolld_filt + joints.hipxd_filt;
		joints.stanceroll_filt = joints.roll_filt + joints.hipx_filt;
	}
	else
	{
		joints.roll_des = roll_des;
		joints.rolld_des = rolld_des;
		
		joints.stancerolld_filt = joints.rolld_filt - joints.hipxd_filt;
		joints.stanceroll_filt = joints.roll_filt - joints.hipx_filt;
	}
	
	if ( time_elapsed_step > 0.4 )
	{
		count++;
		
		if ( Controller()->GetStanceLeg() == 1 )
		{	
			roll_diff_sum += roll_des + joints.stanceroll_filt;//_filt;
			rolld_diff_sum += rolld_des + joints.stancerolld_filt;
		}
		else
		{
			roll_diff_sum += roll_des - joints.stanceroll_filt;//_filt;
			rolld_diff_sum += rolld_des - joints.stancerolld_filt;
		}
	}
	else
	{
		count = 0;
		roll_diff_sum = 0;
		rolld_diff_sum = 0;
	}
	
	if ( Controller()->TimeElapsed() > 0.4 )
	{
		if ( Controller()->GetStanceLeg() == 1 )
		{
			hipx_footplacement = 0.10 + 0.5*(roll_des + joints.stanceroll_filt) + 0.5*(rolld_des + joints.stancerolld_filt);
		}
		else
		{
			hipx_footplacement = 0.10 + 0.5*(roll_des - joints.stanceroll_filt) + 0.5*(rolld_des - joints.stancerolld_filt);
		}
		hipx_footplacement = min( max( hipx_footplacement , 0.08 ) , 0.22);
		
		joints.hipx_footplacement = hipx_footplacement;//0.99*joints.hipx_footplacement + 0.01*hipx_footplacement;
		
		joints.hipx.q.ref = 0.995*joints.hipx.q.ref + 0.005*joints.hipx_footplacement;//joints.hipx_footplacement;
		joints.hipx.q.ref = min( max( joints.hipx.q.ref , 0.10 ) , 0.16);
	}
	
	if ( Controller()->TimeElapsed() < 0.3 )
	{
		//if ( hipx_ref_startstep < 0.13 )
			joints.hipx_footplacement = compute_quintic_spline( 3.33*(Controller()->TimeElapsed()), hipx_ref_startstep , 0.0, 0.0, 0.13, 0.0, 0.0);
		
		joints.hipx.q.ref =  compute_quintic_spline( 3.33*(Controller()->TimeElapsed()), hipx_ref_startstep , 0.0, 0.0, 0.13, 0.0, 0.0);
	}
	//if ( Controller()->TimeElapsed() > 0.3 && Controller()->TimeElapsed() < 0.6 )
	//	joints.hipx.q.ref =  compute_quintic_spline( 3.33*(Controller()->TimeElapsed() - 0.3), 0.13 , 0.0, 0.0, 0.10, 0.0, 0.0);

	joints.Update(&s);
		
	// detect foot release and react
	if ( !(s.stance().foot.back.state || s.stance().foot.front.state) )
		count_contact++;
	else
		count_contact = 0;
	
	if ( count_contact == 500 )
		Controller()->Transition(&gWalking_StStanceFootRelease);
	
	// detect swing foot strike and goto pushoff
	if ( ( s.swing().foot.back.state
			|| ( s.swing().ankley.q > 0.05 && s.swing().ankley.qd > 2.0 ) )
			&& Controller()->TimeElapsed() > 0.5 )
		Controller()->Transition(&gWalking_StPushoff);
}

void CWalking_StStanceFootRelease::Init()
{
	logprintf("Stance foot release\n");
	
	hipx_q_start = s.hipx.q;
	l_hipy_q_start = s.l().hipy.q;
	r_hipy_q_start = s.r().hipy.q;
	l_knee_q_start = s.l().knee.q;
	r_knee_q_start = s.r().knee.q;
	l_ankley_q_start = s.l().ankley.q;
	r_ankley_q_start = s.r().ankley.q;
	
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		joints.legs[iLeg].knee.mMode = SEA_MODE_ANGLE;
		
		joints.legs[iLeg].hipy.mMode = SEA_MODE_ANGLE;
		
		joints.legs[iLeg].ankley.mMode = SEA_MODE_ANGLEPLUSTORQUE;
		joints.legs[iLeg].ankley.angCtrl.kp = 4;
		joints.legs[iLeg].ankley.angCtrl.kd = 0;//0.2;
	}
	
	joints.interleg.Disable();
	joints.upperbody.Disable();
		
	//s.powered = 0;
}

void CWalking_StStanceFootRelease::Update()
{
	for (int iLeg=0; iLeg<2; iLeg++)
	{
		// compensate for passive spring in ankle
		//joints.legs[iLeg].ankley.tauSEA.ref = (s.legs[iLeg].ankley.q + 0.38)*5;
		joints.l().ankley.tauSEA.ref = (s.l().ankley.q + 0.38)*5;
		joints.r().ankley.tauSEA.ref = (s.r().ankley.q + 0.38)*5;
	}
	
	if ( Controller()->TimeElapsed() <= 1.0 )
	{
		joints.hipx.q.ref = hipx_q_start;
		joints.l().hipy.q.ref = l_hipy_q_start;
		joints.r().hipy.q.ref = r_hipy_q_start;
		joints.l().knee.q.ref = l_knee_q_start;
		joints.r().knee.q.ref = r_knee_q_start;
		joints.stance().knee.q.ref = 0;
		joints.stance().knee.angCtrl.ff = -5;
		joints.l().ankley.q.ref = l_ankley_q_start;
		joints.r().ankley.q.ref = r_ankley_q_start;
	}
	else if ( Controller()->TimeElapsed() <= 2.0 )
	{	
		joints.hipx.q.ref = compute_quintic_spline( Controller()->TimeElapsed()-1.0, hipx_q_start, 0.0, 0.0, 0.14, 0.0, 0.0);
		joints.l().hipy.q.ref = compute_quintic_spline( Controller()->TimeElapsed()-1.0, l_hipy_q_start, 0.0, 0.0, 0.0, 0.0, 0.0);
		joints.r().hipy.q.ref = compute_quintic_spline( Controller()->TimeElapsed()-1.0, r_hipy_q_start, 0.0, 0.0, 0.0, 0.0, 0.0);
		joints.l().knee.q.ref = compute_quintic_spline( Controller()->TimeElapsed()-1.0, l_knee_q_start, 0.0, 0.0, 0.0, 0.0, 0.0);
		joints.r().knee.q.ref = compute_quintic_spline( Controller()->TimeElapsed()-1.0, r_knee_q_start, 0.0, 0.0, 0.0, 0.0, 0.0);
		joints.l().ankley.q.ref = compute_quintic_spline( Controller()->TimeElapsed()-1.0, l_ankley_q_start, 0.0, 0.0, 0.0, 0.0, 0.0);
		joints.r().ankley.q.ref = compute_quintic_spline( Controller()->TimeElapsed()-1.0, r_ankley_q_start, 0.0, 0.0, 0.0, 0.0, 0.0);
	}
	else if ( Controller()->TimeElapsed() <= 3.0 )
	{
		for (int iLeg=0; iLeg<2; iLeg++)
		{
			// ff to overstretch knee
			joints.legs[iLeg].knee.angCtrl.ff = -1.5;
		}
		
		joints.l().hipy.q.ref = compute_quintic_spline( Controller()->TimeElapsed()-2.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0);
		joints.r().hipy.q.ref = compute_quintic_spline( Controller()->TimeElapsed()-2.0, 0.0, 0.0, 0.0, -0.4, 0.0, 0.0);
				
		// define ff to deal with gravity
		joints.l().hipy.angCtrl.ff = joints.l().hipy.q.ref*8.0;
		joints.r().hipy.angCtrl.ff = joints.r().hipy.q.ref*8.0;
	}
	
	joints.Update(&s);
	
	if ( Controller()->TimeElapsed() >= 3.0 )
		Controller()->Transition(&gWalking_StGetReady);
	
}

void CWalking_StCrash::Init()
{
	logprintf("CRASH!!\n");
}

void CWalking_StCrash::Update()
{
	s.powered = 0;
}



// Get default warnings again for 'double to float' issues
#pragma warning( default : 4244 4305 )
