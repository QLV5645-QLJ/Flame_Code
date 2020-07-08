#ifndef __CTRLJOINTS_H_INCLUDED
#define __CTRLJOINTS_H_INCLUDED

#include <utility/system_state_var.h>

class CCtrlData: public CSysVarredClass
{
	public:
		float	cur;	// Current (actual) value
		float	ref;	// Reference (desired) value
		virtual void	Zero()	{ cur=0; ref=0; }
		
		// add desired data to sysvars to get logged in ringbuffer
		void GetSysVars(CSysVars* sysvars)
		{
			SYSVARS_ADD_FLOAT(sysvars, cur );
			SYSVARS_ADD_FLOAT(sysvars, ref );
		}
};

class CCtrlDataD: public CCtrlData
{
	public:
		// Inherited: cur & ref
		CCtrlData	d;	// Added: derivative cur & ref
		virtual void	Zero()	{ CCtrlData::Zero(); d.Zero(); }
		inline void		SetCur(float x_cur, float xd_cur)
		{
			cur = x_cur;
			d.cur = xd_cur;
		}
		
		// add desired data to sysvars to get logged in ringbuffer
		void GetSysVars(CSysVars* sysvars)
		{
			CCtrlData::GetSysVars(sysvars);
			SYSVARS_ADD_CHILD(sysvars, d );
		}		
};

class CCtrlDataD_CalcDRef: public CCtrlDataD
{
	protected:
		// Filter memory:
		float	refPrev;
	public:
		float	alpha;
		
		CCtrlDataD_CalcDRef();
		
		// ensure zero derivative for some discrete reference switches (f.i. stance <-> swing)
		inline void SetZeroDerivRef()
		{
			refPrev = ref;
		}
		
		inline void FiltandDerivRef(const float dt)
		{
			ref			= alpha*ref + (1.0f-alpha)*refPrev;
			d.ref		= (ref - refPrev)/dt;
			refPrev		= ref;
		}

		// add desired data to sysvars to get logged in ringbuffer
		void GetSysVars(CSysVars* sysvars)
		{
			CCtrlDataD::GetSysVars(sysvars);
		}
};

class CPDController: public CSysVarredClass
{
	public:
		float kp;			// proportional gain
		float kd;			// damping
		float ff;			// feed forward
		//int ffNumElements;

		inline float	PD(CCtrlDataD* x) const
		{
			return kp*(x->ref - x->cur) + kd*(x->d.ref - x->d.cur);
		}
		inline virtual float	PD_ff(CCtrlDataD* x) const
		{
			return ff + PD(x);
		}

		virtual void	Disable()	{ kp=0; kd=0; ff=0; }

		void			SetModeP()
		{
			kd = 0;
			ff = 0;
		}

		void			SetModePD()
		{
			ff = 0;
		}

		// add desired data to sysvars to get logged in ringbuffer
		void GetSysVars(CSysVars* sysvars)
		{
			SYSVARS_ADD_FLOAT(sysvars, ff );
		}
};

class CJointNoControl: public CSysVarredClass
{
	public:
		CCtrlDataD_CalcDRef		q;

		// add desired data to sysvars to get logged in ringbuffer
		void GetSysVars(CSysVars* sysvars)
		{
			SYSVARS_ADD_CHILD(sysvars, q );
		}
};

// Standard joint controller
class CJointController: public CPDController
{
	public:
		CCtrlDataD_CalcDRef		q;
		float data;			// empty slot for any data item

		int turbo_boost;	// maximum amps boost
		virtual void	Disable();

		inline float GetTau(const float dt)
		{
			// filter and calculate derivative of reference
			q.FiltandDerivRef(dt);
			return PD_ff(&q);
		}
		
		// add desired data to sysvars to get logged in ringbuffer
		void GetSysVars(CSysVars* sysvars)
		{
			CPDController::GetSysVars(sysvars);
			SYSVARS_ADD_CHILD(sysvars, q );
		}
};

enum ECtrlSEAMode
{
	SEA_MODE_TORQUE,
	SEA_MODE_ANGLE,
	SEA_MODE_ANGLEPLUSTORQUE,
	SEA_MODE_MOTANGLE
};

class CJointControllerSEA: public CSysVarredClass
{
	private:
		// You don't have to call this function yourself
		void			CalcTauCur()
		{
			// stiffSEA voor de enkels = kspring*R2*R2
			tauSEA.cur		= stiffSEA*(qmot.cur - q.cur);
			tauSEA.d.cur	= stiffSEA*(qmot.d.cur - q.d.cur);
		}
		
	public:
		ECtrlSEAMode			mMode;	// Choose between ANGLE control or TORQUE control
		CCtrlDataD_CalcDRef		q;		
		CCtrlDataD_CalcDRef		qmot;
		CCtrlDataD_CalcDRef		tauSEA;
		CPDController			angCtrl;
		CPDController			motangCtrl;
		CPDController			trqCtrl;

		int						turbo_boost;	// maximum amps boost
		float					stiffSEA;

		virtual void	Disable();

		inline float GetTau(const float dt)
		{
			CalcTauCur();

			// filter and calculate derivative of reference
			q.FiltandDerivRef(dt);

			if (mMode == SEA_MODE_TORQUE)
				tauSEA.ref = tauSEA.ref;
			else
			{
				if (mMode == SEA_MODE_ANGLE)
					tauSEA.ref = angCtrl.PD_ff(&q);
				else
				{
					if (mMode == SEA_MODE_ANGLEPLUSTORQUE)
						tauSEA.ref = angCtrl.PD_ff(&q) + tauSEA.ref;
					else
					{
						if (mMode == SEA_MODE_MOTANGLE)
						{
							qmot.FiltandDerivRef(dt);
							return motangCtrl.PD(&qmot);
						}
						else
							return 0;
					}
				}
				
			}
			
			// filter and calculate derivative of reference torque
			tauSEA.FiltandDerivRef(dt);
			
			trqCtrl.ff = tauSEA.ref;
			return trqCtrl.PD_ff(&tauSEA);
		}
		
		// add desired data to sysvars to get logged in ringbuffer
		void GetSysVars(CSysVars* sysvars)
		{
			SYSVARS_ADD_CHILD(sysvars, angCtrl );
			SYSVARS_ADD_CHILD(sysvars, q );
			//SYSVARS_ADD_CHILD(sysvars, qmot );
			SYSVARS_ADD_CHILD(sysvars, tauSEA );
		}
};


#endif
