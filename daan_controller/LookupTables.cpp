#include "LookupTables.h"

#include <math.h>
#include <stdlib.h>

CLUTLinear::CLUTLinear()
{
	mY = NULL;
	mNumElements = 0;
}

void CLUTLinear::Init(float xStart, float xEnd, float* yVec, int numElements)
{
	mY	= yVec;
	mXStart	= xStart;
	mXEnd	= xEnd;
	mXStepsize		= (xEnd - xStart)/(numElements-1);
	mX1OverStepsize	= (numElements-1)/(xEnd - xStart);	// Multiplying is faster than dividing
	mNumElements = numElements;
}


float CLUTLinear::GetValue(float xValue)
{
	int x0 = (int)floor( (xValue - mXStart) * mX1OverStepsize );

	// Hold first y-value (don't extrapolate)
	if (x0 < 0)
		return mY[0];
	
	// Hold last y-value (don't extrapolate)
	if (x0 >= mNumElements-1)
		return mY[mNumElements-1];

	// Interpolate
	return mY[x0] + ((xValue - mXStart - x0*mXStepsize)* mX1OverStepsize)*(mY[x0+1] - mY[x0]);
}
