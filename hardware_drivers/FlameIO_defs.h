
/****************************************************************/
// Define structures to describe the state of the robot.  There
// are so many symmetries in the variable naming that it is
// convenient to use nested structures.  For example, a given
// joint will typically have a position, velocity, and offset, so
// a structure containing these values can be instanced once for
// every joint to efficiently create all the state.  And many
// values are symmetric for left and right.  This also makes it
// easier to write walking code which is independent of the
// current stance foot.

// The naming follows the Marc Raibert lab convention:
//   X is forward
//   Z is up
//   Y is left    (right hand coordinates system)
//   
// Joints are named according to their axis direction in the
// reference position.  E.g., the hipx axis has an axis aligned
// with the forward direction, and so is the one which allows
// lateral leg motion.
//
// These should be the calibrated, useful, real-world values.
//
#ifndef FLAMEIO_DEFS_H_INCLUDED
#define FLAMEIO_DEFS_H_INCLUDED

#include <utility/system_state_var.h>

// Definition of state struct

class flame_foot_switch_t: public CSysVarredClass
{
public:
  float input;                     // pressure sensor reading, arbitrary units
  float threshold;                 // input switching threshold
  int count;                       // debounce count
  int state;                       // discretized value

  // add desired data to sysvars to get logged in ringbuffer
  void GetSysVars(CSysVars* sysvars)
  {
	//SYSVARS_ADD_FLOAT(sysvars, input);	  	
	//SYSVARS_ADD_INT(sysvars, count);
	SYSVARS_ADD_INT(sysvars, state);
  }
};

class flame_foot_t: public CSysVarredClass
{
public:
  flame_foot_switch_t front;       // state for the front switch
  flame_foot_switch_t back;        // state for the rear switch
  
  // add desired data to sysvars to get logged in ringbuffer
  void GetSysVars(CSysVars* sysvars)
  {
	SYSVARS_ADD_CHILD(sysvars, front);
	SYSVARS_ADD_CHILD(sysvars, back);
  }
} ;

class flame_battery_t: public CSysVarredClass
{
public:
  float mot_un;                    // unswitched motor battery voltage readings
  float mot_sw;                    // switched motor battery voltage readings
  float com_un;                    // unswitched computer battery voltage readings
  float com_sw;                    // switched computer battery voltage readings
} ;

class flame_dof_t: public CSysVarredClass
{
public:
	float	q;
	float	qd;
	float	qdold;
	float	imon;
	float	tau;

	// add desired data to sysvars to get logged in ringbuffer
	void GetSysVars(CSysVars* sysvars)
	{
//		SYSVARS_ADD_FLOAT(sysvars, q);
//		SYSVARS_ADD_FLOAT(sysvars, qd);
//		SYSVARS_ADD_FLOAT(sysvars, qdold);
		SYSVARS_ADD_FLOAT(sysvars, imon);
		SYSVARS_ADD_FLOAT(sysvars, tau);
	}
} ;

class flame_leg_t: public CSysVarredClass
{
public:
	flame_foot_t	foot;
	flame_dof_t		hipy;                      // hip pitch joint angle wrt body (around y axis), in radians
	flame_dof_t		knee;                      // knee joint angle wrt body (around y axis), in radians
	flame_dof_t		ankley;                    // ankle pitch joint angle wrt body (around y axis), in radians
	flame_dof_t		anklex;                    // ankle roll joint angle wrt body (around x axis), in radians
	flame_dof_t		hipymot;                   // hip pitch motor angle, in radians
	flame_dof_t		kneemot;                   // knee motor angle, in radians
	flame_dof_t		ankleymot;                 // ankle motor angle, in radians

	// add desired data to sysvars to get logged in ringbuffer
	void GetSysVars(CSysVars* sysvars)
	{
		SYSVARS_ADD_CHILD(sysvars, foot);
		SYSVARS_ADD_CHILD(sysvars, hipy);
		SYSVARS_ADD_CHILD(sysvars, knee);
		SYSVARS_ADD_CHILD(sysvars, ankley);
		//SYSVARS_ADD_CHILD(sysvars, anklex);
		//SYSVARS_ADD_CHILD(sysvars, hipymot);
		//SYSVARS_ADD_CHILD(sysvars, kneemot);
		//SYSVARS_ADD_CHILD(sysvars, ankleymot);
	}
} ;

class timing_data_t: public CSysVarredClass
{
public:
  float sensor_processing;         // duration of the sensor processing on the previous cycle
  float total_cycle;               // duration of the previous complete processing cycle
  
  // add desired data to sysvars to get logged in ringbuffer
    void GetSysVars(CSysVars* sysvars)
    {
    	SYSVARS_ADD_FLOAT(sysvars, sensor_processing);
    	SYSVARS_ADD_FLOAT(sysvars, total_cycle);
    }
} ;

class imu_data_t: public CSysVarredClass
{
public:
  int samples;                     // number of IMU samples
  float yaw;                       // Euler angle
  float pitch;                     // Euler angle
  float pitchd;
  float roll;                      // Euler angle
  float rolld;
  float rolld_gyro;

	// add desired data to sysvars to get logged in ringbuffer
  	void GetSysVars(CSysVars* sysvars)
  	{
  		SYSVARS_ADD_INT(sysvars, samples);
  		SYSVARS_ADD_FLOAT(sysvars, yaw);
  		SYSVARS_ADD_FLOAT(sysvars, pitch);
  		SYSVARS_ADD_FLOAT(sysvars, pitchd);
  		SYSVARS_ADD_FLOAT(sysvars, roll);
  		SYSVARS_ADD_FLOAT(sysvars, rolld);
  		SYSVARS_ADD_FLOAT(sysvars, rolld_gyro);
  	}
} ;

class FlameIO_state_t: public CSysVarredClass
{
public:
	int front_panel_sw;              // switch state, as bit flags
	int powered;                     // flags to indicate enabled motor channels
	int motor_faults;                // flags to indicate drivers in fault state
	flame_battery_t battery;         // battery voltage readings
	flame_dof_t		hipx;

	// data that came from the c struct:
	float t;
	float dt;                        // the idealized time step between control updates
	int LEDS;                        // current output value of indicator LEDs
	timing_data_t timing;            // timing diagnostics
	imu_data_t imu;                  // inertial data


	// This union is basically a struct containing 2 leg structs.
	// Because it is a union, the left leg can be accessed in 3 ways:
	// 1) .l()
	// 2) .legs[0]
	// 3) .stance() or .swing(), depending on the bool left_is_swing
	flame_leg_t legs[2];
	inline flame_leg_t &l()	{ return legs[0];}
	inline flame_leg_t &r()	{ return legs[1];}
	// stance and swing interface
	bool		left_is_stance;
	inline flame_leg_t &stance()	{ return legs[(int)(!left_is_stance)];}
	inline flame_leg_t &swing()		{ return legs[(int)(left_is_stance)];}

	
	// add desired data to sysvars to get logged in ringbuffer
	void GetSysVars(CSysVars* sysvars)
	{
//		SYSVARS_ADD_INT(sysvars, front_panel_sw);
		SYSVARS_ADD_FLOAT(sysvars, t);
		SYSVARS_ADD_CHILD(sysvars, timing );
		SYSVARS_ADD_INT(sysvars, left_is_stance );
		SYSVARS_ADD_CHILD(sysvars, imu );
		SYSVARS_ADD_CHILD(sysvars, hipx );
		SYSVARS_ADD_CHILD(sysvars, l() );
		SYSVARS_ADD_CHILD(sysvars, r() );
	}
	
} ;



// Definition params struct

class params_flame_offsetscale_t: public CSysVarredClass
{
public:
	float	offset;
	float	scale;
} ;

class params_flame_foot_t: public CSysVarredClass
{
public:
	params_flame_offsetscale_t	front;
	params_flame_offsetscale_t	back;
} ;

class params_flame_dof_t: public CSysVarredClass
{
public:
	params_flame_offsetscale_t	q;
	params_flame_offsetscale_t	imon;
	params_flame_offsetscale_t	tau;
	float	taumax;
} ;

class params_flame_leg_t: public CSysVarredClass
{
public:
	params_flame_foot_t		foot;
	params_flame_dof_t		hipy;                      // hip pitch joint angle wrt body (around y axis), in radians
	params_flame_dof_t		knee;                      // knee joint angle wrt body (around y axis), in radians
	params_flame_dof_t		ankley;                    // ankle pitch joint angle wrt body (around y axis), in radians
	params_flame_dof_t		anklex;                    // ankle roll joint angle wrt body (around x axis), in radians
	params_flame_dof_t		hipymot;                   // hip pitch motor angle, in radians
	params_flame_dof_t		kneemot;                   // knee motor angle, in radians
	params_flame_dof_t		ankleymot;                 // ankle motor angle, in radians
} ;

class params_flame_battery_t: public CSysVarredClass
{
public:
  params_flame_offsetscale_t	mot_un;                    // unswitched motor battery voltage readings
  params_flame_offsetscale_t	mot_sw;                    // switched motor battery voltage readings
  params_flame_offsetscale_t	com_un;                    // unswitched computer battery voltage readings
  params_flame_offsetscale_t	com_sw;                    // switched computer battery voltage readings
} ;

class FlameIO_params_t: public CSysVarredClass
{
public:
	params_flame_battery_t	battery;         // battery voltage readings
	params_flame_dof_t		hipx;

	// This union is basically a struct containing 2 leg structs.
	// Because it is a union, the left leg can be accessed in 3 ways:
	// 1) .l
	// 2) .legs[0]
	// 3) .stance() or .swing(), depending on the bool left_is_swing
	params_flame_leg_t legs[2];
	inline params_flame_leg_t &l()	{ return legs[0];}
	inline params_flame_leg_t &r()	{ return legs[1];}
	// stance and swing interface
	bool		left_is_stance;
	inline params_flame_leg_t &stance()		{ return legs[(int)(!left_is_stance)];}
	inline params_flame_leg_t &swing()		{ return legs[(int)(left_is_stance)];}

} ;

#endif // FLAMEIO_DEFS_H_INCLUDED
