// $Id: record.h,v 1.1 2005/12/09 09:11:39 garthz Exp $ 
//
// record.c : code to support data recording for thet biped control code.
//
// Copyright (C) 1995-2001 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.

#ifndef RECORD_H_INCLUDED
#define RECORD_H_INCLUDED

#include "system_state_var.h"
#include "dataset.h"

// Create a ring buffer for storing data from the system state
// variable description and the specified number of samples.
extern dataset_t *create_ring_buffer(system_state_var_t *sys_vars, unsigned length, int NumVars);

// Write the ring buffer out to a new file.  Returns a newly
// allocated string with the name on success or NULL on nfailure.
// The string must be freed by the caller.
extern char *write_ring_buffer(dataset_t *d);

// Copy a snapshot of all variables into a column of the data set.
extern void ring_buffer_snapshot(system_state_var_t *sys_vars, dataset_t *d, int NumVars);

// Clear the ring buffer by resetting the pointers.
extern void clear_ring_buffer(dataset_t *d);

#endif /**************** RECORD_H_INCLUDED ****************/





