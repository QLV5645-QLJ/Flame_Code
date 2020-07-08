#include "FlameJoints.h"
#include "globals.h"

void CFlameJoints::Init()
{
	joints.interleg.q.ref		= 0.0;
	joints.interleg.q.d.ref		= 0.0;
	joints.interleg.q.alpha		= 1.0;
	joints.interleg.kp			= 20.0;
	joints.interleg.kd			= 2.0;
	joints.interleg.turbo_boost = 0;

	joints.upperbody.q.ref		= 0.0;
	joints.upperbody.q.d.ref	= 0.0;
	joints.upperbody.q.alpha	= 1.0;
	joints.upperbody.kp			= 40.0;
	joints.upperbody.kd			= 4.0;
	joints.upperbody.turbo_boost= 0;
	
	joints.hipx.q.ref		= 0.0;
	joints.hipx.q.d.ref		= 0.0;
	joints.hipx.q.alpha		= 1.0;
	joints.hipx.kp			= 400.0;
	joints.hipx.kd			= 2.0;
	joints.hipx.turbo_boost = 0;


	for (int iLeg=0; iLeg<2; iLeg++)
	{
		joints.legs[iLeg].hipy.q.ref		= 0.0;
		joints.legs[iLeg].hipy.q.d.ref		= 0.0;
		joints.legs[iLeg].hipy.tauSEA.ref	= 0.0;
		joints.legs[iLeg].hipy.tauSEA.d.ref = 0.0;
		joints.legs[iLeg].hipy.q.alpha      = 1.0;
		joints.legs[iLeg].hipy.qmot.alpha   = 1.0;
		joints.legs[iLeg].hipy.tauSEA.alpha = 1.0/16.0;
		joints.legs[iLeg].hipy.angCtrl.kp	= 20.0;
		joints.legs[iLeg].hipy.angCtrl.kd	= 2.0;		
		// new after Thijs, need to be defined
		joints.legs[iLeg].hipy.stiffSEA		= 115.9;//stiffness springs in bag 12/12:100.8;//50.4;
		joints.legs[iLeg].hipy.trqCtrl.kp	= 0.0;
		joints.legs[iLeg].hipy.trqCtrl.kd	= 0.0;
		joints.legs[iLeg].hipy.motangCtrl.kp= 0.0;
		joints.legs[iLeg].hipy.motangCtrl.kd= 0.0;
				
		joints.legs[iLeg].hipy.turbo_boost	= 0;

		joints.legs[iLeg].knee.q.ref		= 0.0;
		joints.legs[iLeg].knee.q.d.ref		= 0.0;
		joints.legs[iLeg].knee.tauSEA.ref	= 0.0;
		joints.legs[iLeg].knee.tauSEA.d.ref = 0.0;
		joints.legs[iLeg].knee.q.alpha      = 1.0;
		joints.legs[iLeg].knee.qmot.alpha    = 1.0;
		joints.legs[iLeg].knee.tauSEA.alpha = 1.0/16.0;
		joints.legs[iLeg].knee.angCtrl.kp	= 10.0;
		joints.legs[iLeg].knee.angCtrl.kd	= 1.0;
		// new after Thijs, need to be defined
		joints.legs[iLeg].knee.stiffSEA		= 16.9;//13.6;//1st less stiff spring//6.8;
		joints.legs[iLeg].knee.trqCtrl.kp	= 0.0;
		joints.legs[iLeg].knee.trqCtrl.kd	= 0.0;
		joints.legs[iLeg].knee.motangCtrl.kp= 0.0;
		joints.legs[iLeg].knee.motangCtrl.kd= 0.0;

		joints.legs[iLeg].knee.turbo_boost	= 0;

		joints.legs[iLeg].ankley.q.ref			= 0.0;
		joints.legs[iLeg].ankley.q.d.ref		= 0.0;
		joints.legs[iLeg].ankley.tauSEA.ref		= 0.0;
		joints.legs[iLeg].ankley.tauSEA.d.ref	= 0.0;
		joints.legs[iLeg].ankley.q.alpha		= 1.0;
		joints.legs[iLeg].ankley.qmot.alpha		= 1.0;
		joints.legs[iLeg].ankley.tauSEA.alpha	= 1.0/16.0;
		joints.legs[iLeg].ankley.angCtrl.kp		= 200.0;
		joints.legs[iLeg].ankley.angCtrl.kd		= 1.0;
		joints.legs[iLeg].ankley.stiffSEA		= 71.6;//old lever:31.6;
		joints.legs[iLeg].ankley.trqCtrl.kp		= 16.0;
		joints.legs[iLeg].ankley.trqCtrl.kd		= 0.1;
		joints.legs[iLeg].ankley.motangCtrl.kp	= 0.0;
		joints.legs[iLeg].ankley.motangCtrl.kd	= 0.0;
		
		joints.legs[iLeg].ankley.turbo_boost	= 0;
	}
}

void CFlameJoints::Update(FlameIO_state_t* flameState)
{
	int iLeg=0;

	// **** INPUT ***//
	// First, copy current values from the s struct to the joint controllers
	
	// multijoint controllers
	interleg.q.SetCur(flameState->swing().hipy.q-flameState->stance().hipy.q, flameState->swing().hipy.qd-flameState->stance().hipy.qd);
	upperbody.q.SetCur(flameState->imu.pitch, flameState->imu.pitchd);
	
	// local joint controllers
	hipx.q.SetCur(flameState->hipx.q, flameState->hipx.qd);
	for (iLeg=0; iLeg<2; iLeg++)
	{
		// q data
		legs[iLeg].hipy.q.SetCur(flameState->legs[iLeg].hipy.q, flameState->legs[iLeg].hipy.qd);
		legs[iLeg].knee.q.SetCur(flameState->legs[iLeg].knee.q, flameState->legs[iLeg].knee.qd);
		legs[iLeg].ankley.q.SetCur(flameState->legs[iLeg].ankley.q, flameState->legs[iLeg].ankley.qd);
		legs[iLeg].anklex.q.SetCur(flameState->legs[iLeg].anklex.q, flameState->legs[iLeg].anklex.qd);
		// qmot data
		legs[iLeg].hipy.qmot.SetCur(flameState->legs[iLeg].hipymot.q, flameState->legs[iLeg].hipymot.qd);
		legs[iLeg].knee.qmot.SetCur(flameState->legs[iLeg].kneemot.q, flameState->legs[iLeg].kneemot.qd);
		legs[iLeg].ankley.qmot.SetCur(flameState->legs[iLeg].ankleymot.q, flameState->legs[iLeg].ankleymot.qd);
	}
	
	// get filtered values of desired variables
	roll_filt = 0.1*flameState->imu.roll + 0.9*roll_filt;
	rolld_filt = 0.01*flameState->imu.rolld_gyro + 0.99*rolld_filt;
	
	hipx_filt = 0.1*flameState->hipx.q + 0.9*hipx_filt;
	hipxd_filt = 0.01*flameState->hipx.qd + 0.99*hipxd_filt;

	// *** OUTPUT ***//
	// Now, perform the updates
	// first the multijoint controllers
	// write tau of multijoint controllers as desired tau of local controllers, do a += operation to allow addition of 
	// local action and multijoint action
	tau_interleg = interleg.GetTau(flameState->dt);
	swing().hipy.tauSEA.ref += tau_interleg;
	stance().hipy.tauSEA.ref -= tau_interleg;
	
	stance().hipy.tauSEA.ref -= upperbody.GetTau(flameState->dt);
	
	// then apply the local joint controllers
	flameState->hipx.tau = hipx.GetTau(flameState->dt);
	for (iLeg=0; iLeg<2; iLeg++)
	{
		flameState->legs[iLeg].hipy.tau		= legs[iLeg].hipy.GetTau(flameState->dt);
		flameState->legs[iLeg].knee.tau		= legs[iLeg].knee.GetTau(flameState->dt);
		flameState->legs[iLeg].ankley.tau	= legs[iLeg].ankley.GetTau(flameState->dt);
	}
}

