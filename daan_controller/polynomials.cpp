// polynomials.c : basic polynomial evaluators for typical spline segments.
//
// Copyright (C) 2001-2005 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
// For simple single-segment spline position trajectory generation
// this code is simpler and more efficient than general polynomial or
// spline evaluation.  These are suitable for hand-tuning the
// coefficients.
//

float
compute_quadratic_spline( float t, float x0, float v0, float x1 )
{
  float c = x0;              // initial position
  float b = v0;
  float a = x1 - x0 - v0;

  if (t < 0) t = 0; else if (t > 1) t = 1;
  return ( a * t * t   +   b * t   +  c  );
}

/****************************************************************/
// General cubic spline function.  Takes an initial and final position
// and velocity, and a time parameter that varies from 0 to 1, returns
// a position.

float
compute_cubic_spline( float t, float x0, float v0, float x1, float v1 )
{
  // compute a cubic polynomial trajecotry, assuming a desired final velocity of zero
  float d = x0;              // initial position
  float c = v0;
  float b = 3 * (x1 - x0) - 2*v0 - v1;
  float a = v1 + v0 - 2 * (x1 - x0);

  if (t < 0) t = 0; else if (t > 1) t = 1;
  return ( a * t * t * t   +   b * t * t   +  c * t  + d );
}


/****************************************************************/
// Quintic splines.
//
// The cubic spline has a discontinuous second derivative at the
// boundaries, which can be a problem for inertial feedforward
// torques.  The quintic spline can specify accelerations at the
// boundaries.
//
// 5th order splines: x = a t^5 + b t^4 + c t^3 + d t^2 + e t + f
//   6 coefficients per segment.
//   Position is 5th order.
//   Velocity is 4th order.
//   Acceleration is cubic, can be continuous at boundaries.
//   Jerk is quadratic, has a discontinuous step at boundaries.
//
// Solving for the coefficients in terms of the boundary conditions
// at t==0 and t==1 yields the following:
//  
//     T = transpose [t^5 t^4 t^3 t^2 t 1]
//  
//    at T = 0
//     pos =  [  a   b   c   d   e   f ] T =  f
//     vel =  [  0  5a  4b  3c  2d   e ] T =  e
//     acc =  [  0  0  20a 12b  6c  2d ] T = 2d
//     jrk =  [  0  0    0 60a 24b  6c ] T = 6c
//  
//    at T = 1
//     pos =  [  a   b   c   d   e   f ] T =    a +   b +  c +  d + e + f
//     vel =  [  0  5a  4b  3c  2d   e ] T =   5a +  4b + 3c + 2d + e
//     acc =  [  0  0  20a 12b  6c  2d ] T =  20a + 12b + 6c + 2d
//     jrk =  [  0  0    0 60a 24b  6c ] T =  60a + 24b +  c
//  
//     f = x0
//     e = v0
//     d = a0 / 2
//  
//     [ 1  1  1] [a]   [x1 -  d - e - f]
//     [ 5  4  3] [b] = [v1 - 2d - e]
//     [20 12  6] [c]   [a1 - 2d]          
//
//     [a]   [    6.00000   -3.00000    0.50000  ] [x1 -  d - e - f]  
//     [b] = [  -15.00000    7.00000   -1.00000  ] [v1 - 2d - e]	   
//     [c]   [   10.00000   -4.00000    0.50000  ] [a1 - 2d]          


// For a point to point move, we want the initial and
// final velocity and acceleration to be zero, so a lot of terms
// become zero.
//
//   v0 = v1 = a0 = a1 = 0
//
//    f = x0
//    e = d = 0
//
//     [a]   [    6.00000   -3.00000    0.50000  ] [x1 -  x0]  
//     [b] = [  -15.00000    7.00000   -1.00000  ] [   0   ]	   
//     [c]   [   10.00000   -4.00000    0.50000  ] [   0   ]
//
//    a =   6 ( x1 - x0 )
//    b = -15 ( x1 - x0 )
//    c =  10 ( x1 - x0 )

float 
compute_quintic_spline_pp( float t, float x0, float x1 )
{
  float t2, t3, t4, t5;
  float xdiff = x1 - x0;

  if ( t < 0 ) t = 0; else if ( t > 1 ) t = 1;
  t2 = t*t;
  t3 = t2*t;
  t4 = t3*t;
  t5 = t4*t;
  return ( xdiff * ( 6*t5 -  15*t4 + 10*t3 ) + x0 );
}

// Fully specified boundary conditions.
//     f = x0
//     e = v0
//     d = a0 / 2
//  
//     [ 1  1  1] [a]   [x1 -  d - e - f]
//     [ 5  4  3] [b] = [v1 - 2d - e]
//     [20 12  6] [c]   [a1 - 2d]          
//
//     [a]   [    6.00000   -3.00000    0.50000  ] [x1 -  d - e - f]  
//     [b] = [  -15.00000    7.00000   -1.00000  ] [v1 - 2d - e]	   
//     [c]   [   10.00000   -4.00000    0.50000  ] [a1 - 2d]          

float 
compute_quintic_spline( float t, 
			float x0, float v0, float a0,
			float x1, float v1, float a1 )
{
  float t2, t3, t4, t5;          // powers of time variable
  float a, b, c, d, e, f;        // polynomial coefficients
  float y1, y2, y3;              // intermediate values

  if ( t < 0 ) t = 0; else if ( t > 1 ) t = 1;
  t2 = t*t;
  t3 = t2*t;
  t4 = t3*t;
  t5 = t4*t;

  f = x0;
  e = v0;
  d = 0.5f * a0;

  // compute the rightmost column of the solution for [a b ]
  y1 = x1 -   d - e - f;
  y2 = v1 - 2*d - e;
  y3 = a1 - 2*d;

  // multiply out the solution matrix to find [a b c]
  a =   6*y1 - 3*y2 + 0.5f*y3;
  b = -15*y1 + 7*y2 -     y3;
  c =  10*y1 - 4*y2 + 0.5f*y3;

  // compute the polynomial
  return ( a*t5 + b*t4 + c*t3 + d*t2 + e*t + f );
}
