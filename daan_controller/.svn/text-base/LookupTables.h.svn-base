#ifndef __LOOKUPTABLES_H_INCLUDED
#define __LOOKUPTABLES_H_INCLUDED

class CLUTLinear
{
	protected:
		float	*mY;
		int		mNumElements;
		float	mXStart;
		float	mXEnd;
		float	mXStepsize;
		float	mX1OverStepsize;
	public:
		CLUTLinear();
		void	Init(float xStart, float xEnd, float* yVec, int numElements);

		float	GetValue(float xValue);
};


#endif
