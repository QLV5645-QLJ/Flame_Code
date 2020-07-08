// $Id: dataset.h,v 1.1 2005/12/09 09:11:39 garthz Exp $
// dataset.h : state variable trajectory recording.
//
// Copyright (C) 1995-2001 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
// This is version 1.  Version 0 was coded in C++ as part of my
// Ph.D. work.  In some respects, this version is simpler; I've
// eliminated the idea of separating the variables into different
// modules; that could now be done based on systematic naming
// without explicit structure in the file.  However, it now
// supports multiple data types.
//
// A "dataset_t" is a structure that represents a single large
// matrix with a named variable on each row and a column for each
// sample in time.  The code can read and write this structure to
// a file.  The file is written out by columns, i.e., each frame
// in the file is a snapshot in time, and the file can be
// arbitrarily long.  The in-memory matrix representation can be
// used as a window on long file, an entire file, or a ring
// buffer.

#ifndef DATASET_H_INCLUDED
#define DATASET_H_INCLUDED

#include <sys/types.h>
#include <stdio.h>
#include <string.h>

// The in-memory representation of the data is contained in these structures.

// The units of each variable are encoded as follows:

enum dsUnits {
  DS_DIMENSIONLESS, 
  DS_RADIANS, 
  DS_RADIANPERSEC, 
  DS_METERS,
  DS_METERSPERSEC,

  DS_UNITS_MAX       // count of items in enum
};
  
// The data type of each variable is encoded as follows:
enum dsType {
  DS_NOTYPE = 0,
  DS_INT,           // 32 bit signed integer
  DS_FLOAT,         // 32 bit IEEE floating point
  DS_DOUBLE,        // 64 bit IEEE floating point
  DS_STRING,        // arbitrary length string value

  DS_TYPE_MAX       // count of items in enum
};

// The following structure describes the data in each "row" of the dataset:
struct dsVariable {
  char *name;           // name of the variable name
  enum dsType type;     // data type
  enum dsUnits units;   // units, i.e., physical interpretation of values
  char *desc;           // comment string describing the meaning of the variable
  double upper;         // the "normal" upper bound for a numerical variable
  double lower;         // the "normal" lower bound for a numerical variable

  unsigned archivable : 1;  // true if this variable should be included in files

  // It might also be useful to have an active flag to control
  // whether the variable is available at all, although that
  // might also be indicated by a null data array.

  // It might also be useful to store an integer source index, to
  // allow mapping this variable to a position within another
  // data source.
};

// The following structure describes the data in the full data
// matrix.  "columns" describes the size of the matrix in memory.
// If the matrix is used as a ring buffer, "startpos" is the
// index of the column that is logically the first in time, and
// "samples" is the number of valid samples in the matrix.  If
// the matrix is used as a window on a file, then "filepos" is
// the index of the sample (in file coordinates) that corresponds
// with the "startpos" column in memory.

typedef struct {
  unsigned int variables;      // number of variables (number of rows)
  unsigned int columns;        // number of columns of each variable in memory
  unsigned int startpos;       // the index of the "first" sample in the data in memory 
  unsigned int samples;        // number of valid samples in memory
  unsigned int filepos;        // the sample index within a file that startpos represents
  char *comment;               // description string for the whole set or file
  struct dsVariable *vars;     // array of variable descriptions
  void **data;                 // array of pointers to data buffers
  time_t timestamp;            // UNIX timestamp when object was created or file written
} dataset_t;

/****************************************************************/
// Set the error stream for more verbose debugging.
extern void ds_error_stream(FILE *f);

/****************************************************************/
// Functions to create and manipulate datasets.

// Allocate an empty dataset of the specified number of rows.  No
// data buffers are allocated, however the number of columns
// needs to be set before the first variable is initialized.
extern dataset_t *new_dataset(int variables);

// Delete a dataset and free up all memory.
extern void delete_dataset(dataset_t *d);

// Set the global comment.
extern void ds_set_comment(dataset_t *d, char *string);

// Set the number of columns.
extern void ds_set_columns(dataset_t *d, int cols);

// Set the timestamp to the current time.
extern void ds_set_timestamp(dataset_t *d);

// Initialize each individual variable entry, which was already allocated by new_dataset.
extern void 
ds_init_variable(dataset_t *d, int row, 
		 char* name, char *desc,
		 enum dsType type, enum dsUnits units, 
		 double lower, double upper);

// Add a variable to an existing data set.  This adds a row to the end, no existing indicies will change.
extern void 
ds_add_variable( dataset_t *d,
		 char *name, char *desc,
		 enum dsType type, enum dsUnits units, 
		 double lower, double upper );

// Rename a variable.  The string is internally duplicated.
extern void ds_rename_variable( dataset_t *d, int row, char *newname );

// Remove a variable (a row) from dataset.  Note that the row index
// for all subsequent variables will change.
extern void ds_delete_variable( dataset_t *d, int row );

// Resize the width, i.e., throw away all but the specified range of columns.  Note that 
// this operates on the raw number of columns irrespective of how many samples exist or
// where the first sample is located.
extern void ds_resize_columns ( dataset_t *d, int first_column, int number_of_columns );

// Write an in-memory matrix to the given stream.  The ring
// buffer indicators are respected, so only valid columns will be
// included. The file representation is in binary.  Returns 0 on
// success, else an error code.
extern int ds_write_dataset(dataset_t *d, FILE* file);

// Creates a new dataset object from data from a stream.  Since
// streams can represent matrices of unbounded width, maxcolumns
// specifies the maximum number of data frames that should be
// read.  If set to DS_UNSPECIFIED_LENGTH, only streams with
// known length can be read.
//
// Returns a pointer on success, else NULL.

#define DS_UNSPECIFIED_LENGTH 0xffffffff

extern dataset_t *new_dataset_from_stream(FILE* file, unsigned maxcolumns);

// Return points to strings to describe the codes that define a variable.
extern const char * ds_get_type_string(enum dsType t);
extern const char * ds_get_units_string(enum dsUnits t);

// Functions to access the matrix data. 

// Return array pointer to the specified variable, or 
// NULL if the type doesn't match.  These can be used for reading
// or writing, but be sure to mind the columns limit. 

extern int    *ds_int(dataset_t *d, int row);
extern float  *ds_float(dataset_t *d, int row);
extern double *ds_double(dataset_t *d, int row);

// Copy a string into a given position.  Does nothing if variable is not
// a string type.
extern void ds_set_string(dataset_t *d, int row, int col, const char *s);

// Retrieve the string pointer from a given position (which could
// be NULL) .  Returns NULL if variable is not a string type.
extern const char *ds_get_string(dataset_t *d, int row, int col);

// Print out diagnostics about a dataset to a stream.
extern void ds_print_info(dataset_t *d, FILE* file, int verbose);

// Print out ASCII representation of a datum.
extern void ds_print_value(dataset_t *d, FILE *file, int row, int col);

// Find the index of a variable given the name.  Returns -1 if
// an exact match is not found.
extern int ds_find_variable(dataset_t *d, const char *name); 

// Return a row of a dataset as a newly allocated array of doubles, or
// NULL if the row is not a numeric type.  The caller is responsible
// for freeing the array.  The array is of length "samples".
extern double *ds_row_as_double_array( dataset_t *d, int row );

// Set an entire row at once.
extern void ds_set_row_from_double_array( dataset_t *d, int row, double *array );


#endif /**************** DATASET_H_INCLUDED ****************/





