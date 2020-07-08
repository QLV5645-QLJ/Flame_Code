// polynomials.h : basic polynomial evaluators for typical spline segments.
//
// Copyright (C) 2001-2005 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//

#ifndef __POLYNOMIALS_H_DEFINED__
#define __POLYNOMIALS_H_DEFINED__

extern float compute_quadratic_spline( float t, float x0, float v0, float x1 );

extern float compute_cubic_spline( float t, float x0, float v0, float x1, float v1 );

extern float compute_quintic_spline_pp( float t, float x0, float x1 );

extern float compute_quintic_spline( float t, 
				     float x0, float v0, float a0,
				     float x1, float v1, float a1 );

#endif  //__POLYNOMIALS_H_DEFINED__
