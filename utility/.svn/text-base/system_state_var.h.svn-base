// $Id: system_state_var.h,v 1.2 2005/12/14 08:35:56 garthz Exp $
// system_state_var.h : the meta-data structure for describing system state variables.
//
// Copyright (C) 2001-2005 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.

#ifndef __SYSTEM_STATE_VAR_H_INCLUDED__
#define __SYSTEM_STATE_VAR_H_INCLUDED__

#include <stdio.h>

#define SYSVAR_FLOAT(x) float x;\
						const char *SYSVAR_NAME_##x = #x

#define SYSVARS_ADD_FLOAT(sv, x)	(sv)->AddSysVar(#x, SYS_FLOAT, &x);
#define SYSVARS_ADD_INT(sv, x)		(sv)->AddSysVar(#x, SYS_INT, &x);
#define SYSVARS_ADD_CHILD(sv, x)	(sv)->Add(&x, #x);

// This enumeration must match the corresponding enumeration in dataset.h for
// the connecting code to work properly.  This should probably just be one 
// definition, but are different for historical reasons.

enum system_state_var_type_t {
  SYS_NOTYPE = 0, 
  SYS_INT,           // 32 bit signed integer
  SYS_FLOAT,         // 32 bit IEEE floating point
  SYS_DOUBLE,        // 64 bit IEEE floating point
  SYS_STRING         // fixed length string value
};

// State variables are described by a null-terminated array of
// the following elements:

typedef struct {
  char *name;       // Name of the variable.

  // Define the possible variable data types.  
  enum system_state_var_type_t type;

  void *data; // address of the datum in the state structure

  // flags 
  unsigned dirty : 1;       // true if the UI has modified the value

} system_state_var_t;

// The minimum size of a SYS_STRING data area, and the most number of
// bytes written when setting a SYS_STRING from the library code.
// These variables are mostly used to indicate a state name (i.e., for
// short constant strings), and statically allocating space is less
// error-prone than dealing with the semantics of dynamic allocation
// or pointers to static strings.

#define SYS_STRING_LEN 100

/****************************************************************/
// Utility functions.

// Find a variable in an array, returns the index or -1.
extern int system_state_var_find(system_state_var_t *s, const char *name);

// Copy values of variables from source and destination as
// described by two description arrays.  Variables not found in
// the destination and strings are silently ignored.
extern void system_state_var_copy_values(system_state_var_t *dest, system_state_var_t *src);

// Sets the value of a particular state variable from a string.  Returns true
// on an error.
extern int system_state_var_set_from_string(system_state_var_t *var, char *value);

// Count the elements of a NULL terminated array of variable descriptors.
extern int system_state_var_count(system_state_var_t *var);

// Set all the flags to default values in a NULL terminated array.
extern void system_state_var_init_flags( system_state_var_t *var );

// load and save parameter files
extern int system_state_var_array_save_to_stream( system_state_var_t *array, FILE *stream );
extern int system_state_var_array_save_to_file( system_state_var_t *array, char *filename );
extern int system_state_var_array_set_from_stream( system_state_var_t *array, FILE *stream );
extern int system_state_var_array_set_from_file( system_state_var_t *array, char *filename );


// ****  Object oriented code for making classes ready for the 'sysvar system' **** //
class CSysVarredClass;

class CSysVars
{
	protected:
		int		mNumElements;

		int		mGrowSize;
		int		mNumInternalElements;
		void	InternalAlloc(const int numExtraVars);
		void	Alloc(const int numExtraVars);
	public:
		system_state_var_t*	mData;

		CSysVars();
		~CSysVars();

		int		GetNumElements()	{ return mNumElements;}
		void	AddSysVar(char* name, system_state_var_type_t type, void* data);
		void	Add(CSysVarredClass* child, char* parentName);
};

class CSysVarredClass
{
	public:
		virtual void	GetSysVars(CSysVars* sysvars) {}
};
// **** End of object oriented part **** //


#endif /*  __SYSTEM_STATE_VAR_H_INCLUDED__ */
