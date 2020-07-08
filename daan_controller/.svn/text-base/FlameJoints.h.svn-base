#ifndef __FLAMEJOINTS_H_INCLUDED
#define __FLAMEJOINTS_H_INCLUDED

#include "CtrlJoints.h"
#include <hardware_drivers/FlameIO_defs.h>

class CLegJoints: public CSysVarredClass
{
	public:
		CJointControllerSEA	hipy;              // gains and desired positions
		CJointControllerSEA	knee;              // gains and desired positions
		CJointControllerSEA	ankley;           // gains and desired positions for ankles
		CJointNoControl anklex;
		void	Disable()		{hipy.Disable(); knee.Disable(); ankley.Disable();}
		
		// add desired data to sysvars to get logged in ringbuffer
		void GetSysVars(CSysVars* sysvars)
		{
			SYSVARS_ADD_CHILD(sysvars, hipy );
			SYSVARS_ADD_CHILD(sysvars, knee );
			SYSVARS_ADD_CHILD(sysvars, ankley );
			SYSVARS_ADD_CHILD(sysvars, anklex );
		}
};

class CFlameJoints: public CSysVarredClass
{
	protected:
		float				tau_interleg;
	public:
		float 				hipx_footplacement;
		float				roll_filt;
		float				rolld_filt;
		float				hipx_filt;
		float				hipxd_filt;
		float				stancerolld_filt;
		float				stanceroll_filt;
		float				roll_des;
		float				rolld_des;
		
		CJointController	interleg;			// multijoint controllers
		CJointController	upperbody;
		
		CJointController	hipx;              // gains and desired positions

		// This union is basically a struct containing 2 leg structs.
		// Because it is a union, the left leg can be accessed in 3 ways:
		// 1) .l
		// 2) .legs[0]
		// 3) .swing() when left_is_swing==true, otherwise .stance()
		CLegJoints legs[2];
		inline CLegJoints &l()	{ return legs[0];}
		inline CLegJoints &r()	{ return legs[1];}
		// stance and swing interface
		bool		left_is_stance;
		inline CLegJoints &stance()	{ return legs[(int)(!left_is_stance)];}
		inline CLegJoints &swing()	{ return legs[(int)(left_is_stance)];}
		void	Disable()			{ hipx.Disable(); legs[0].Disable(); legs[1].Disable();}


		void	Init();
		void	Update(FlameIO_state_t* flameState);
		
		// add desired data to sysvars to get logged in ringbuffer
		void GetSysVars(CSysVars* sysvars)
		{
			SYSVARS_ADD_FLOAT(sysvars, hipx_footplacement );
			SYSVARS_ADD_FLOAT(sysvars, roll_filt );
			SYSVARS_ADD_FLOAT(sysvars, rolld_filt );
			SYSVARS_ADD_FLOAT(sysvars, hipx_filt );
			SYSVARS_ADD_FLOAT(sysvars, hipxd_filt );
			SYSVARS_ADD_FLOAT(sysvars, stancerolld_filt );
			SYSVARS_ADD_FLOAT(sysvars, stanceroll_filt );
			SYSVARS_ADD_FLOAT(sysvars, roll_des );
			SYSVARS_ADD_FLOAT(sysvars, rolld_des );
			SYSVARS_ADD_CHILD(sysvars, interleg );
			SYSVARS_ADD_CHILD(sysvars, upperbody );
			SYSVARS_ADD_CHILD(sysvars, hipx );
			SYSVARS_ADD_CHILD(sysvars, l() );
			SYSVARS_ADD_CHILD(sysvars, r() );
		}

};

#endif
