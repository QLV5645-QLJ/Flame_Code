// $Id: system_state_var.c,v 1.3 2005/12/15 16:01:20 garthz Exp $ 
//
// system_state_var.c : utilities functions for use with system_state_var_t 
// arrays.
//
// Copyright (C) 2002-2005 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <errno.h>
#include "system_state_var.h"


// Declare max macro (delete if it comes from another header file)
#define max(a, b)  (((a) > (b)) ? (a) : (b))


// Find a variable in an array, returns the index or -1.
int system_state_var_find(system_state_var_t *s, const char *name)
{
  int idx;
  if (s == NULL) return -1;

  for (idx = 0; s->name != NULL; idx++, s++) {
    if (!strcmp(s->name, name)) return idx;
  }
  return -1;
}

// Copy values of variables from source and destination as
// described by two description arrays.  Variables not found in
// the destination are silently ignored.

void
system_state_var_copy_values(system_state_var_t *dest, system_state_var_t *src)
{
  for (  ; src->name != NULL ; src++) {

    // find the source variable in the destination array
    int idx = system_state_var_find(dest, src->name);

    if (idx != -1) {
      system_state_var_t *dvar = &dest[idx];

      // Do a limited amount of type casting.
      switch(dvar->type) {

      case SYS_FLOAT:
	switch (src->type) {
	case SYS_FLOAT:	   *((float *) dvar->data) = *((float *) src->data); break;
	case SYS_DOUBLE:   *((float *) dvar->data) = *((double *) src->data); break;
	}
	break;

      case SYS_DOUBLE:
	switch (src->type) {
	case SYS_FLOAT:	  *((double *) dvar->data) = *((float *) src->data); break;
	case SYS_DOUBLE:  *((double *) dvar->data) = *((double *) src->data); break;
	}
	break;

      case SYS_INT:
	if (src->type == dvar->type) *((int *) dvar->data) = *((int *) src->data);
	break;

	// There is no way to know how the destination string is allocated, so strings
	// are ignored.
      case SYS_STRING:
      case SYS_NOTYPE:
      default:
	break;
      }
    }
  }
}

// returns true on error
int 
system_state_var_set_from_string(system_state_var_t *var, char *value)
{
  switch(var->type) {

  case SYS_FLOAT:
    if (1 == sscanf(value, "%f", (float *)var->data)) {
      var->dirty = 1;

      return 0;
    } else return 1;
    break;

  case SYS_DOUBLE:
    if (1 == sscanf(value, "%lf", (double *)var->data)) {
      var->dirty = 1;

      return 0;
    } else return 1;
    break;

  case SYS_INT:
    if (1 == sscanf(value, "%d", (int *)var->data)) {
      var->dirty = 1;

      return 0;
    } else return 1;
    break;

  case SYS_STRING:         // can't modify strings
    return 1;

  case SYS_NOTYPE:
  default:
    return 1;
  }
}
// Count the elements of a NULL terminated array of variable descriptors.
int system_state_var_count(system_state_var_t *var)
{
  int varcount = 0;
  if (var != NULL) {
    while (var->name != NULL) {
      varcount++;
      var++;
    }
  }
  return varcount;
}

// Set all the flags to default values in a NULL terminated array.
void
system_state_var_init_flags( system_state_var_t *var )
{
  if (var != NULL) {
    while (var->name != NULL) {
      var->dirty = 0;
      var++;
    }
  }
}
/****************************************************************/
// load and save parameter files

int system_state_var_array_save_to_stream( system_state_var_t *var, FILE *stream )
{
  if ( var == NULL) {
    errno = EINVAL;
    return -1;
  }

  while (var->name != NULL) {
    switch ( var->type ) {

    case SYS_FLOAT:  
      // The default precision doesn't have enough decimal
      // places, so the precision is ridiculously high just to
      // make sure no bits are lost.
      fprintf( stream, "%s %10.20f\n", var->name, *((float *) var->data) ); 
      break;

    case SYS_DOUBLE: 
      fprintf( stream, "%s %10.20f\n", var->name, *((double *) var->data) ); 
      break;

    case SYS_INT: 
      fprintf( stream, "%s %d\n", var->name, *((int *) var->data) ); 
      break;

    case SYS_STRING: 
      fprintf( stream, "%s %s\n", var->name, (char *) var->data );
      break;
    }
    var++;
  }
  return ferror( stream );
}

int system_state_var_array_save_to_file( system_state_var_t *array, char *filename )
{
  int r;
  FILE *f;

  if ( array == NULL || filename == NULL ) {
    errno = EINVAL; 
    return -1;
  }

  f = fopen( filename, "w");
  if ( f == NULL ) return -1;

  r = system_state_var_array_save_to_stream( array, f );
  fclose( f );
  return r;
}
/****************************************************************/

/* Parse a line into arguments, storing pointers in the provided argv array,
   returning the arg count. */

static int parse_arg_line(char *string, char **argv, int argmax)
{
  int argc = 0;
  char *cp = string;
  while (*cp && argc < argmax) {
    while (*cp && isspace(*cp)) cp++;     /* scan past white space */
    if (*cp) argv[argc++] = cp;           /* save pointer to text */  
    else break;
    while (*cp && !isspace(*cp)) cp++;    /* scan past text */
    if (*cp) *cp++ = 0;
  }
  return argc;
}
/****************************************************************/
int system_state_var_array_set_from_stream( system_state_var_t *array, FILE *stream )
{
#define MAXLINE 200
  char buffer[MAXLINE];

  if ( array == NULL ) {
    errno = EINVAL; 
    return -1;
  }

  while ( !feof ( stream ) ) {
    char *line = fgets( buffer, MAXLINE, stream );
    char *argv[2];
    int argc, idx;

    if ( line == NULL ) break;

    // Separate the line into non-space substrings.  This only handles
    // string-type values with a single non-space token.
    argc = parse_arg_line( line, argv, 2 );
    if ( argc < 2 ) continue;

    idx = system_state_var_find( array, argv[0] );
    if ( idx < 0 ) continue;

    // printf("found %s as variable %d, setting to '%s'\n", argv[0], idx, argv[1]);
    system_state_var_set_from_string( &array[idx], argv[1] );
  }

  return ferror( stream );
}


int system_state_var_array_set_from_file( system_state_var_t *array, char *filename )
{
  int r;
  FILE *f;

  if ( array == NULL || filename == NULL ) {
    errno = EINVAL; 
    return -1;
  }

  f = fopen( filename, "r");
  if ( f == NULL ) return -1;

  r = system_state_var_array_set_from_stream( array, f );
  fclose( f );
  return r;
}


// ******** Object oriented part ******** //
CSysVars::CSysVars()
{
	mData					= NULL;
	mNumElements			= 0;
	mNumInternalElements	= 0;
	mGrowSize				= 20;
}

CSysVars::~CSysVars()
{
	if (mData != NULL)
	{
		// First, free all strings
		for (int i=0; i<mNumElements; i++)
			free(mData[i].name);

		// Free all elements
		free(mData);
	}
}

void CSysVars::AddSysVar(char* name, system_state_var_type_t type, void* data)
{
	Alloc(1);
	mData[mNumElements-1].data = data;
	mData[mNumElements-1].type = type;
	mData[mNumElements-1].name = (char*)malloc(SYS_STRING_LEN);
	strcpy(mData[mNumElements-1].name, name);
}

void CSysVars::Add(CSysVarredClass* child, char* parentName)
{
	int numElementsStart = mNumElements;
	// New sysvars are added to the end
	child->GetSysVars(this);
	// Now change the name of the childs to include the parent name
	for (int i=numElementsStart; i<mNumElements; i++)
	{
		char *newName = (char*)malloc(SYS_STRING_LEN);
		strcpy(newName, parentName);
		strcat(newName, ".");
		strcat(newName, mData[i].name);
		char *oldName = mData[i].name;
		mData[i].name = newName;
		free(oldName);
	}
}

void CSysVars::InternalAlloc(const int numExtraVars)
{
	mNumInternalElements += numExtraVars;
	mData = (system_state_var_t*)realloc(mData, mNumInternalElements*sizeof(system_state_var_t));
}

void CSysVars::Alloc(const int numExtraVars)
{
	if (mNumInternalElements < mNumElements+numExtraVars)
		InternalAlloc(max(mGrowSize, numExtraVars));

	mNumElements += numExtraVars;
}

// ******** End of object oriented part ******** //

