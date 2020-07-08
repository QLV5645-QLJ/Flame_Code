#include "CtrlJoints.h"
#include <memory.h>
#include <math.h>

CCtrlDataD_CalcDRef::CCtrlDataD_CalcDRef()
{
	refPrev = 0;
}

void CJointController::Disable()
{
	CPDController::Disable();
	//q.Zero();
	data		= 0;	// empty slot for any data item
	turbo_boost	= 0;	// maximum amps boost
}

void CJointControllerSEA::Disable()
{
	//tauSEA.Zero();
	//qmot.Zero();
	angCtrl.Disable();
	trqCtrl.Disable();
}

