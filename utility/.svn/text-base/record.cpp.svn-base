// $Id: record.c,v 1.3 2005/12/14 08:35:55 garthz Exp $
// record.c : code to support data recording for the biped control code.
//
// Copyright (C) 1995-2005 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "record.h"
#include "dataset.h"
#include "utility.h"
#include "system_state_var.h"


// Create a ring buffer for storing data from the system state
// variable description and the specified number of samples.

// adapted by Daan 25-09-07 since no NULL as last var

dataset_t *
create_ring_buffer(system_state_var_t *sys_vars, unsigned length, int NumVars)
{
  dataset_t *d;   // The dataset object used for the buffer.
  int v;

  // Allocate a data object.
  d = new_dataset(NumVars);
  ds_set_columns(d, length);

  // Loop through all variables to initialize each row of the matrix.
  for ( v = 0 ; v < NumVars  ; v++ ) {

	// The dataset and system_state_var type codes are now interchangable.
    enum dsType type = (enum dsType) sys_vars[v].type;
    
    // set the properties of the data set variable and create a buffer 
    ds_init_variable(d, v, 
		     sys_vars[v].name, 
		     NULL,              // description string
		     type, 
		     DS_DIMENSIONLESS,  // units
		     -1.0, 1.0);        // lower, upper
  }

  return d;
}

// Write the ring buffer out to a new file.  Returns a newly
// allocated string with the name on success or NULL on failure.
// The string must be freed by the caller.

char *
write_ring_buffer(dataset_t *d)
{
  int r;
  FILE *file;
  char *filename;
  char current_dir[500];
  int switched_dirs = 0;

  // Try to save the current directory and change into the "data"
  // subdirectory, which may well be a link.
  if ( ( getcwd( current_dir, (size_t) 500) != NULL ) && ( chdir ("data") == 0 )) {
    // logprintf("  moved to data subdirectory.\n");
    switched_dirs = 1;
  }

  // search the directory to figure out the correct new file name
  filename = new_data_file_name();

  // switch back to the initial directory
  if (switched_dirs) chdir (current_dir);

  if ( filename == NULL ) return NULL;

  // if needed, prepend a path
  if ( switched_dirs ) {
    char *newname;
    asprintf( &newname, "data/%s", filename );
    free(filename);
    filename = newname;
  }

  // the "b" is for non-UNIX systems
  file = fopen(filename, "wb");

  if (file == NULL) {
    free(filename);
    return NULL;
  }

  // logprintf("  writing data file %s\n", filename );

  ds_set_timestamp(d);
  r = ds_write_dataset(d, file);
  fclose(file);

  if ( r ) {
    free(filename);
    return NULL;
  } 

  return filename;
}

// Copy a snapshot of all variables into a column of the data set.
void
ring_buffer_snapshot(system_state_var_t *sys_vars, dataset_t *d, int NumVars)
{
  int col;        // the column to fill
  int v;

  if ( d == NULL || sys_vars == NULL ) return;

  // figure out the next column to write
  col = (d->startpos + d->samples) % d->columns;

  // Update the ring buffer indices.  When the buffer is full,
  // the oldest entry is thrown away and the start position must
  // move.  This increments it and wraps it.
  if (d->samples >= d->columns) {
    if (++d->startpos >= d->columns) d->startpos = 0;
  }

  // the number of valid samples increases until the buffer is full
  if (++d->samples > d->columns) d->samples = d->columns;

  // Loop through all variables and copy over the data.
  for ( v = 0 ; v < NumVars  ; v++ ) {
    // determine the dataset variable type.
    switch(sys_vars[v].type) {

    case SYS_FLOAT:  
      (ds_float(d, v))[col] = *((float *) sys_vars[v].data);
      break;

    case SYS_DOUBLE: 
      (ds_double(d, v))[col] = *((double *) sys_vars[v].data);
      break;

    case SYS_INT: 
      (ds_int(d, v))[col] = *((int *) sys_vars[v].data);
      break;

    case SYS_STRING: 
      ds_set_string(d, v, col, (char *) sys_vars[v].data);
      break;

    case SYS_NOTYPE:
      break;
    }
  }    
}

// Clear the ring buffer by resetting the pointers.
void
clear_ring_buffer(dataset_t *d)
{
  if ( d != NULL ) {
    d->startpos = 0;
    d->samples = 0;
  }
}
