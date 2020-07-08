// dataset.c : state variable trajectory recording.
//
// Copyright (C) 1995-2001 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
// This code represents a single large matrix with a named
// variable on each row, and a column for each sample in time.
// The code can read and write this structure to a file.
//
// This is version 1.  Version 0 was coded in C++ as part of my
// Ph.D. work.  In some respects, this version is simpler; I've
// eliminated the idea of separating the variables into different
// modules; that could be done now external to this code based on
// systematic naming.  However, it now supports multiple data types 
// and writes out the data by columns to support streaming.

// The file format assumes:
//  sizeof(unsigned int) = 4
//  sizeof(float) = 4
//  sizeof(double) = 8
//  sizeof(short) = 2

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>     // for strerror()
#include <errno.h>

#ifdef linux
#include <endian.h>
#endif

#ifdef __MINGW32__
#include <sys/param.h>   // for BYTE_ORDER
#endif

#include "dataset.h"

#define DATASET_MAGIC_NUM 0xdaaadeaf  // arbitrary 32 bit magic number for header
#define HEADER_WORD_1     0xf63b3128  // arbitrary 48 bit data frame boundary marker
#define HEADER_WORD_2     0xf8a50000

#define SAFE_FREE(mem)  if ((mem) != NULL) free(mem)
  
static char *typenames[DS_TYPE_MAX] =
  { "no type", 
    "integer", 
    "float", 
    "double", 
    "string"
  };

static char *unitsnames[DS_UNITS_MAX] =
  { "dimensionless",
    "radians",
    "radians/sec",
    "meters",
    "meters/sec"
  };

// Return pointer to strings to describe the codes that define a variable.
const char * ds_get_type_string(enum dsType t)
{
  if (t < DS_TYPE_MAX) return typenames[t];
  else return "<bad type code>";
}
const char * ds_get_units_string(enum dsUnits t)
{
  if (t < DS_UNITS_MAX) return unitsnames[t];
  else return "<bad units code>";
}
/****************************************************************/
// Common point for error messages.

static FILE *errstream = NULL;

static
void ds_errprintf(char *format, ...)
{
  va_list args;

  if (errstream != NULL) {
    va_start(args, format);
    vfprintf(stderr, format, args);
    fflush(stderr);
    va_end(args);
  }
}

// Set the error stream for more verbose debugging.
void
ds_error_stream(FILE *f)
{
  errstream = f;
}
/****************************************************************/
// MinGW doesn't provide gettimeofday, so we simulate one.  It does,
// however, provide a timeval struct.
#ifdef __MINGW32__
int gettimeofday( struct timeval *tv, void *tz )
{
  if (tv != NULL) {
    time_t now = time(NULL);
    tv->tv_sec = now;
    tv->tv_usec = 0;
  }
  return 0;
}
#endif

/****************************************************************/
// Allocate an empty dataset of the specified number of rows.  No
// data buffers are allocated, however the number of columns
// needs to be set before the first variable is initialized.

dataset_t *new_dataset(int variables)
{
  struct timeval now; 
  dataset_t *d = (dataset_t *) malloc (sizeof(dataset_t));

  gettimeofday(&now, NULL);

  d->columns = 0;
  d->variables = variables;
  d->comment = NULL;
  d->timestamp = now.tv_sec;

  d->startpos = 0;   // the ring buffer starts out empty
  d->samples = 0;
  d->filepos = 0;    // at the logical beginning of the output

  // allocate an array of dsVariable structures
  d->vars = (struct dsVariable *) calloc (d->variables, sizeof(struct dsVariable));

  // allocate an array of pointers to buffers, but not actually the data buffers.
  d->data = (void **) calloc(d->variables, sizeof(void *));
  return d;
}

/****************************************************************/
// Delete a dataset and free up all memory.
void delete_dataset(dataset_t *d)
{
  int v;
  if (d == NULL) return;

  // Free the global description
  SAFE_FREE(d->comment);

  // For each variable, free its strings and then its data buffer.
  for (v = 0; v < d->variables; v++) {
    SAFE_FREE(d->vars[v].name);
    SAFE_FREE(d->vars[v].desc);

    // If the column size is unreasonable, no data buffers exist to 
    // de-allocate.
    if (d->columns == 0 || d->columns == DS_UNSPECIFIED_LENGTH) 
      continue;

    // For most data types the data buffer is simply an array of
    // values.  However, for strings the buffer is an array of
    // pointers to strings, allocated separately.

    if (d->vars[v].type == DS_STRING) {
      int s;

      // Free each individual string.
      for (s = 0; s < d->columns; s++) SAFE_FREE(((char **)(d->data[v])) [s]);
    }      
    // Free the fixed sized row data (for any type).
    SAFE_FREE(d->data[v]);
  }
  // Finally, free the array of data buffer pointers itself.
  SAFE_FREE(d->data);  

  // Finally, free the array of variable descriptions.
  SAFE_FREE(d->vars);
}

/****************************************************************/
// Set the global comment.
void 
ds_set_comment(dataset_t *d, char *string)
{
  if (d == NULL) return;
  SAFE_FREE(d->comment);

  d->comment = strdup(string);
}
// Set the number of columns.
void 
ds_set_columns(dataset_t *d, int cols)
{
  if (d == NULL) return;
  d->columns = cols;
}
// Set the timestamp to the current time.
void
ds_set_timestamp(dataset_t *d)
{
  struct timeval now; 

  if (d == NULL) return;

  gettimeofday(&now, NULL);
  d->timestamp = now.tv_sec;
}
/****************************************************************/
// Allocate a data buffer for a variable, with size depending on data type.
static void
ds_allocate_variable_data(dataset_t *d, int row)
{
  switch (d->vars[row].type) {
  case DS_NOTYPE:
  case DS_TYPE_MAX:
    // This shouldn't happen.
    break;

  case DS_INT:           // 32 bit signed integer
    d->data[row] = calloc(d->columns, sizeof(int));
    break;

  case DS_FLOAT:         // 32 bit IEEE floating point
    d->data[row] = calloc(d->columns, sizeof(float));
    break;

  case DS_DOUBLE:        // 64 bit IEEE floating point
    d->data[row] = calloc(d->columns, sizeof(double));
    break;

  case DS_STRING:        // arbitrary length string value
    d->data[row] = calloc(d->columns, sizeof(char *));
    break;
  }
}
/****************************************************************/
// Initialize each individual variable entry.  
void 
ds_init_variable(dataset_t *d, int row, 
		 char* name, char *desc,
		 enum dsType type, enum dsUnits units, 
		 double lower, double upper)
{
  struct dsVariable *var; 
  if (d == NULL) return;

  var = & (d->vars[row]);     // Pointer to the structure we are initializating

  // Fill in all fields.  Strings are duplicated.
  if (name != NULL) var->name  = strdup(name); 
  if (desc != NULL) var->desc  = strdup(desc);

  var->type  = type;
  var->units = units;
  var->upper = upper;
  var->lower = lower;
  var->archivable = 1;  // everything is saved in files by default

  ds_allocate_variable_data(d, row);  // create a buffer
}
/****************************************************************/
// Add a variable to a dataset.
void 
ds_add_variable( dataset_t *d, char *name, char *desc, enum dsType type, enum dsUnits units, double lower, double upper )
{
  void *new_vars, *new_data;

  if ( d == NULL ) return;
  d->variables++;
  
  // Reallocate the dsVariable and data pointer arrays.
  new_vars = realloc( d->vars, d->variables * sizeof(struct dsVariable) );
  new_data = realloc( d->data, d->variables * sizeof(void *) );

  // check if realloc failed, this shouldn't happen
  if ( new_vars == NULL || new_data == NULL ) {  
    d->variables--;
    return;
  }

  d->vars = (struct dsVariable *) new_vars;
  d->data = (void **) new_data;

  ds_init_variable( d, d->variables-1, name, desc, type, units, lower, upper );
}

/****************************************************************/
// Rename a variable.  The string is internally duplicated.
void ds_rename_variable( dataset_t *d, int row, char *newname )
{
  if ( d == NULL || row < 0 || row >= d->variables ) return;
  else {
    struct dsVariable *var; 
    var = & (d->vars[row]);     // Pointer to the structure we are renaming.
    SAFE_FREE( var->name );
    var->name = strdup( newname );
  }

}
/****************************************************************/
// Delete a variable from a dataset.  This completely removes all
// associated data, which means many variable indicies will likely
// change.

void ds_delete_variable( dataset_t *d, int row )
{
  struct dsVariable *var; 
  if ( d == NULL || row < 0 || row >= d->variables ) return;
  var = & (d->vars[row]);     // Pointer to the structure we are deleting
  
  SAFE_FREE( var->name );
  SAFE_FREE( var->desc );
  SAFE_FREE( d->data[row] );
  
  // Now shift all the subsequent variables up by one position.  This
  // doesn't bother reallocating the memory for the dsVariable or void*
  // arrays, although they could be reduced by one item each.
  {
    int vars_to_move = d->variables - row - 1;
    if (vars_to_move > 0) {
      memmove( var, var+1, vars_to_move * sizeof( struct dsVariable ) );
      memmove( &d->data[row], &d->data[row+1], vars_to_move * sizeof( void *) );
    }
  }
  // And reduce the variable count.
  d->variables--;

}
/****************************************************************/
// Resize the width, i.e., throw away all but the specified range of columns.
void ds_resize_columns ( dataset_t *d, int first_column, int number_of_columns )
{
  if ( d != NULL ) {
    int original_columns = d->columns;
    int original_startpos = d->startpos;
    int original_samples = d->samples;
    int row, col;

    if ( first_column < 0 ) first_column = 0;
    else if ( first_column >= original_columns ) first_column = original_columns;

    if ( number_of_columns < 0 ) number_of_columns = 0;
    else if ( first_column + number_of_columns > original_columns ) number_of_columns = original_columns - first_column;

    d->columns = number_of_columns;

    // find the first valid sample within the new range, and determine how many valid samples were retained in a contiguous block
    if ( d->startpos < first_column ) {                              // startpos falls before the retained range
      d->samples -= first_column - d->startpos;
      if ( d->samples > number_of_columns ) d->samples = number_of_columns;      
      d->startpos = 0;
    } else if ( d->startpos >= first_column + number_of_columns ) {  // startpos falls after the retained range
      d->samples = 0;
      d->startpos = 0;
    } else {                                                         // else startpos falls within the retained range
      if ( d->startpos + d->samples > first_column + number_of_columns ) d->samples = first_column + number_of_columns - d->startpos;
      d->startpos -= first_column;  
    }

    printf("Resizing dataset to columns [%d %d], startpos changed from %d to %d, samples from %d to %d.\n",
	   first_column, first_column + number_of_columns - 1, original_startpos, d->startpos, original_samples, d->samples );

    for ( row = 0; row < d->variables ; row++ ) {
      void *old_data = d->data[row];
      void *new_data;

      ds_allocate_variable_data( d, row );  // allocate a new buffer
      new_data = d->data[row];

      // copy over the columns we want
      switch ( d->vars[row].type ) {
      case DS_INT:    memcpy( new_data, (int *) old_data + first_column    , number_of_columns * sizeof(int) ); break;
      case DS_FLOAT:  memcpy( new_data, (float *) old_data + first_column  , number_of_columns * sizeof(float) ); break;
      case DS_DOUBLE: memcpy( new_data, (double *) old_data + first_column , number_of_columns * sizeof(double) ); break;

      case DS_STRING: 
	memcpy( new_data, (char **) old_data + first_column, number_of_columns * sizeof(char *) ); 

	// free any released strings
	for ( col = 0; col < first_column; col++ ) SAFE_FREE( ((char **) old_data)[col] );
	for ( col = first_column + number_of_columns; col < original_columns; col++ ) SAFE_FREE( ((char **) old_data)[col]);
	break;
      }
    }
  }
}

/****************************************************************/
// Utility functions to read or write pieces of a data file.  In each
// case, returns 0 on success or true on error.
//

inline static int
write_u_short(FILE *f, unsigned short data)
{
#if BYTE_ORDER==BIG_ENDIAN
  unsigned short value;
  unsigned char *dst = (unsigned char *) &value;
  unsigned char *src = (unsigned char *) &data;
  dst[0] = src[1];
  dst[1] = src[0];
  return fwrite(&value, sizeof(unsigned short), 1, f) != 1;

#else
  return fwrite(&data, sizeof(unsigned short), 1, f) != 1;
#endif
}

inline static int
read_u_short(FILE *f, unsigned short *buf)
{
#if BYTE_ORDER==BIG_ENDIAN
  unsigned short value;
  unsigned char *dst = (unsigned char *) buf;
  unsigned char *src = (unsigned char *) &value;
  int count = fread(&value, sizeof(unsigned short), 1, f);
  dst[0] = src[1];
  dst[1] = src[0];
  return (count != 1);

#else
    return fread(buf, sizeof(unsigned short), 1, f) != 1;
#endif
}

inline static int
write_u_int(FILE *f, unsigned int data)
{
#if BYTE_ORDER==BIG_ENDIAN
  unsigned int value;
  unsigned char *dst = (unsigned char *) &value;
  unsigned char *src = (unsigned char *) &data;
  dst[0] = src[3];
  dst[1] = src[2];
  dst[2] = src[1];
  dst[3] = src[0];
  return fwrite(&value, sizeof(unsigned int), 1, f) != 1;

#else
  return fwrite(&data, sizeof(unsigned int), 1, f) != 1;
#endif
}

inline static int
read_u_int(FILE *f, unsigned int *buf)
{
#if BYTE_ORDER==BIG_ENDIAN
  unsigned int value;
  unsigned char *dst = (unsigned char *) buf;
  unsigned char *src = (unsigned char *) &value;
  int count = fread(&value, sizeof(unsigned int), 1, f);
  dst[0] = src[3];
  dst[1] = src[2];
  dst[2] = src[1];
  dst[3] = src[0];
  return (count != 1);

#else
    return fread(buf, sizeof(unsigned int), 1, f) != 1;
#endif
}

inline static int
write_float(FILE *f, float data)
{
#if BYTE_ORDER==BIG_ENDIAN
  float value;
  unsigned char *dst = (unsigned char *) &value;
  unsigned char *src = (unsigned char *) &data;
  dst[0] = src[3];
  dst[1] = src[2];
  dst[2] = src[1];
  dst[3] = src[0];
  return fwrite(&value, sizeof(float), 1, f) != 1;

#else
  return fwrite(&data, sizeof(float), 1, f) != 1;
#endif
}

inline static int
read_float(FILE *f, float *buf)
{
#if BYTE_ORDER==BIG_ENDIAN
  float value;
  unsigned char *dst = (unsigned char *) buf;
  unsigned char *src = (unsigned char *) &value;
  int count = fread(&value, sizeof(float), 1, f);
  dst[0] = src[3];
  dst[1] = src[2];
  dst[2] = src[1];
  dst[3] = src[0];
  return (count != 1);

#else
  return fread(buf, sizeof(float), 1, f) != 1;
#endif
}
inline static int
write_double(FILE *f, double data)
{
#if BYTE_ORDER==BIG_ENDIAN
  double value;
  unsigned char *dst = (unsigned char *) &value;
  unsigned char *src = (unsigned char *) &data;
  dst[0] = src[7];
  dst[1] = src[6];
  dst[2] = src[5];
  dst[3] = src[4];
  dst[4] = src[3];
  dst[5] = src[2];
  dst[6] = src[1];
  dst[7] = src[0];
  return fwrite(&value, sizeof(double), 1, f) != 1;

#else
  return fwrite(&data, sizeof(double), 1, f) != 1;
#endif
}
inline static int
read_double(FILE *f, double *buf)
{
#if BYTE_ORDER==BIG_ENDIAN
  double value;
  unsigned char *dst = (unsigned char *) buf;
  unsigned char *src = (unsigned char *) &value;
  int count = fread(&value, sizeof(double), 1, f);
  dst[0] = src[7];
  dst[1] = src[6];
  dst[2] = src[5];
  dst[3] = src[4];
  dst[4] = src[3];
  dst[5] = src[2];
  dst[6] = src[1];
  dst[7] = src[0];
  return (count != 1);

#else
  return fread(buf, sizeof(double), 1, f) != 1;
#endif
}

// In the stream, strings are stored as a unsigned 16 bit byte
// count, followed by the characters, with no zero termination.
// However, null strings are written with a byte count of 0xffff
// (and zero data bytes).  This allows null strings to be
// distinguished from zero length strings.

static int
write_string(FILE *f, char *s)
{
  int r;
  unsigned len;
  unsigned short shortlen;

  // special case for NULL string, just write the length code for NULL
  if (s == NULL) return write_u_short(f, 0xffff);

  // else usual case, including zero-length strings
  len = strlen(s);
  if (len > 0xfffe) shortlen = 0xfffe;    // clamp the length and cast to 16 bits 
  else shortlen = len;                    

  r = write_u_short(f, shortlen);

  if (shortlen > 0) 
    r = r || (fwrite(s, sizeof(char), shortlen, f) != shortlen);

  return r;
}

// The read function allocates a new buffer for the string if it
// is not null.  The new string or NULL is returned in *s.

static int
read_string(FILE *f, char **s)
{
  int r;
  unsigned short length;

  if (s == NULL) return 1;
  *s = NULL;                 // default string is null

  if (read_u_short(f, &length)) return 1;
  if (length == 0xffff) return 0;  // special case, null string

  *s = (char *) malloc(length + 1);  // allocate new buffer
  **s = 0;                           // default empty string

  if (length == 0) return 0;       // empty string

  // read character data
  r = fread(*s, sizeof(char), length, f) != length;

  if (r) {    // if read of the string data fails, free the new buffer
    free(*s);
    *s = NULL;
    return r;
  }
  
  (*s)[length] = 0;    // add zero termination

  return 0;
}
/****************************************************************/
// Write out the dataset file header to a stream.  "samples" can
// be set to the special indicator DS_UNSPECIFIED_LENGTH if the
// header will precede a stream whose length cannot be determined
// in advance.

static int
ds_write_header(dataset_t *d, FILE* file)
{
  int writable_vars = 0;
  int r, v;
  if (d == NULL) return 1;

  // Count the number of variables that will actually be written.
  for (v = 0; v < d->variables; v++) {
    if ( d->vars[v].archivable ) writable_vars++;
  }

  // Each of these returns zero on success, which means if r is set 
  // on an error none of the successive writes will execute.

  r =      write_u_int(file, DATASET_MAGIC_NUM); // magic number
  r = r || write_u_int(file, 1);               	 // version code
  r = r || write_u_int(file, d->samples);      	 // number of samples in file
  r = r || write_u_int(file, writable_vars );  	 // number of variables in file
  r = r || write_string(file, d->comment);     	 // global comment

  r = r || write_u_int(file, d->timestamp);      // 64 bit UNIX time value
  r = r || write_u_int(file, 0);

  r = r || write_u_int(file, 0);                 // 64 bits of padding for expansion
  r = r || write_u_int(file, 0);


  // Write out all the variable descriptors.  The intent of
  // writing everything separately is to avoid any alignment
  // questions between different compilers.  The write functions
  // write everything in Intel little-endian IEEE floating point
  // and little-endian integers.

  for (v = 0; v < d->variables; v++) {

    // check if the variable is flagged for writing to disk
    if ( d->vars[v].archivable ) {
      struct dsVariable *var = &(d->vars[v]);

      r = r || write_string(file, var->name);     // Write each field of the dsVariable structure.
      r = r || write_u_int(file, var->type);
      r = r || write_u_int(file, var->units);
      r = r || write_string(file, var->desc);
      r = r || write_double(file, var->upper);
      r = r || write_double(file, var->lower);
      r = r || write_u_int(file, 0);              // 64 bits of padding for expansion
      r = r || write_u_int(file, 0);
    }
  }
  // End of the header.  Data frames can follow.
  return r;
}

// Reads a dataset header from a stream into a new dataset
// object.  Returns a pointer on success, else NULL.  The object
// has "columns" set to the number of samples specified in the
// file, which may be DS_UNSPECIFIED_LENGTH.

static dataset_t*
ds_read_header(FILE* file)
{
  unsigned int magic=0, version=0, samples, variables, padding, v;
  struct timeval now; 
  int r;
  dataset_t *d;
  
  r =      read_u_int(file, &magic);
  r = r || read_u_int(file, &version);
  r = r || read_u_int(file, &samples);
  r = r || read_u_int(file, &variables);

  if (r) {
    ds_errprintf("I/O error occurred while reading header.\n");
    return NULL;
  }
  if ((magic != DATASET_MAGIC_NUM) || (version != 1)) {
    ds_errprintf("Invalid magic number (0x%08x) or version (%d).\n", magic, version);
    return NULL;
  }

  // if file fingerprint checks out, create the object  
  d = new_dataset(variables);
  d->columns = samples;

  // and read the remainder of the header
  r = r || read_string(file, &d->comment);

  r = r || read_u_int(file, (unsigned int *) &now.tv_sec);  // 64 bit time value
  r = r || read_u_int(file, &padding);           
  d->timestamp = now.tv_sec;

  r = r || read_u_int(file, &padding);        // 64 bits of padding for expansion
  r = r || read_u_int(file, &padding);

  if (r) {
    ds_errprintf("Error reading global header.\n");
    delete_dataset(d);
    return NULL;
  }
  // and each variable definition
  for (v = 0; v < d->variables; v++) {
    struct dsVariable *var = &(d->vars[v]);

    r = r || read_string(file, &var->name);     // Read each field of the dsVariable structure.

    if (r) {
      ds_errprintf("Error reading variable name.\n");
      break;
    }
    
    // added dummy with change from c to c++
    unsigned int typeDummy;
    r = r || read_u_int(file, &typeDummy);
    var->type = (dsType) typeDummy;
    
    r = r || read_u_int(file, &typeDummy);
    var->units = (dsUnits) typeDummy;
        
//    r = r || read_u_int(file, &var->type);
//    r = r || read_u_int(file, &var->units);
    r = r || read_string(file, &var->desc);
    r = r || read_double(file, &var->upper);
    r = r || read_double(file, &var->lower);
    r = r || read_u_int(file, &padding);        // 64 bits of padding for expansion
    r = r || read_u_int(file, &padding);

    var->archivable = 1; // by default everything should be written out

    if (r) {
      ds_errprintf("Error reading variable description.\n");
      break;
    }
  }

  // If anything failed, free up the new object.
  if (r) {
    delete_dataset(d);
    return NULL;
  } 
  return d;
}


/****************************************************************/
// Write a frame of data to a stream, i.e., one column of the
// data matrix.  col is the column index.  samp is the sample
// index within the stream.  Returns 0 on success, else an error
// code.

static int
ds_write_data_frame(dataset_t *d, FILE* file, unsigned int col, unsigned int samp)
{
  int r, v;

  // First write out the frame header.
  r =      write_u_int(file, HEADER_WORD_1);  // 64 bit marker
  r = r || write_u_int(file, HEADER_WORD_2);  // that may eventually include flags
  r = r || write_u_int(file, samp);           // sample index

  if (r) return r;

  // Write out each variable in sequence.
  for (v = 0; v < d->variables; v++) {

    // check if the variable is flagged for writing to disk
    if ( d->vars[v].archivable ) {

      // then write one element according to the data type
      switch (d->vars[v].type) {

      case DS_NOTYPE:        // This only happens if a variable was un-initialized.
      case DS_TYPE_MAX:
	break;

      case DS_INT:           // 32 bit signed integer
	r = r || write_u_int(file, ((unsigned *)(d->data[v]))[col]);
	break;

      case DS_FLOAT:         // 32 bit IEEE floating point
	r = r || write_float(file, ((float *)(d->data[v]))[col]);
	break;

      case DS_DOUBLE:        // 64 bit IEEE floating point
	r = r || write_double(file, ((double *)(d->data[v]))[col]);
	break;

      case DS_STRING:        // arbitrary length string value
	r = r || write_string(file, ((char **)(d->data[v]))[col]);
	break;
      }
    }
  }
  return r;
}

/****************************************************************/
// Read a frame of data from a stream, i.e., one column of the
// data matrix.  col is the column index. Returns 0 on success,
// else an error code.

static int
ds_read_data_frame(dataset_t *d, FILE* file, unsigned int col)
{
  unsigned int header1, header2, index;
  int r, v;

  // First read in the frame header.
  r =      read_u_int(file, &header1);  // 64 bit marker
  r = r || read_u_int(file, &header2);  // that may eventually include flags
  r = r || read_u_int(file, &index);    // sample index

  if (r) {
    ds_errprintf("Unable to read data frame header.\n");
    return 1;
  }

  if ((header1 != HEADER_WORD_1) || (header2 != HEADER_WORD_2)) {
    ds_errprintf("Invalid data frame header sentinel value: 0x%x%x.\n", header1, header2);
    return 1;
  }
  
  // Read in each variable in sequence.
  for (v = 0; v < d->variables; v++) {
    switch (d->vars[v].type) {

    case DS_NOTYPE:        // This only happens if a variable was un-initialized.
    case DS_TYPE_MAX:
      break;

    case DS_INT:           // 32 bit signed integer
      r = r || read_u_int(file, & ((unsigned *)(d->data[v]))[col]);
      break;

    case DS_FLOAT:         // 32 bit IEEE floating point
      r = r || read_float(file, & ((float *)(d->data[v]))[col]);
      break;

    case DS_DOUBLE:        // 64 bit IEEE floating point
      r = r || read_double(file, & ((double *)(d->data[v]))[col]);
      break;

    case DS_STRING:        // arbitrary length string value
      {
	char **ptr = & ((char **)(d->data[v]))[col];	
	if (!r) {
	  SAFE_FREE(*ptr);  // free any existing string
	  r = r || read_string(file, ptr);

	  if (r) {
	    ds_errprintf("Error reading datum string for variable \"%s\"\n", d->vars[v].name);
	    return 1;
	  }
	}
      }
      break;
    }

    if (r) {
      ds_errprintf("Error reading numeric datum for variable \"%s\": %s\n", d->vars[v].name, strerror(errno));
      break;
    }
  }
  return r;
}
/****************************************************************/
// Write a partial or entire matrix to the given stream.  Since
// the data matrix is often used as a ring buffer, the logical
// beginning and length are specified by the startpos and samples
// variables.  The file representation is in binary.  Returns 0
// on success, else an error code.
int 
ds_write_dataset(dataset_t *d, FILE* file)
{
  int r, col, samp;

  if (d == NULL) return 1;

  // Limit the number of samples written to the size of the matrix.
  if (d->samples > d->columns) d->samples = d->columns;

  // Write the header.
  r = ds_write_header(d, file);
  if (r) return r;

  // Write out the frames of data.
  for (col = d->startpos, samp = 0; samp < d->samples; samp++) {
    r = r || ds_write_data_frame(d, file, col, samp);

    // advance the column pointer, wrapping around if necessary
    if (++col >= d->columns) col = 0;
  }
  return r;
}

/****************************************************************/
// Creates a new dataset object from data from a stream.
// Returns a pointer on success, else NULL.
dataset_t *new_dataset_from_stream(FILE* file, unsigned maxcolumns)
{
  unsigned int cols, v, c;
  int r = 0;
  dataset_t *d;

  d = ds_read_header(file);


  if (d == NULL) {
    ds_errprintf("Unable to read header.\n");
    return NULL;
  }

  // Determine the number of columns for the matrix.
  cols = d->columns;
  if (cols > maxcolumns) cols = maxcolumns;

  // fail if both samples and maxcolumns are unspecified
  if (cols == DS_UNSPECIFIED_LENGTH) {
    delete_dataset(d);
    ds_errprintf("Error: both the file and the memory object have unspecified length.\n");
    return NULL;
  }

  // create a set of buffers
  d->columns = cols;
  for (v = 0; v < d->variables; v++)
    ds_allocate_variable_data(d, v);

  // and read in a set of data frames
  for (c = 0; c < d->columns; c++) { 
    r = r || ds_read_data_frame(d, file, c);

    if (r) {
      ds_errprintf("Unable to read data frame %d.\n", c);
      break;
    }
  }

  // if anything failed, clean up
  if (r) {
    delete_dataset(d);
    return NULL;
  }

  // If everything succeeded, set the number of valid samples.
  d->samples = d->columns;
  return d;
}

/****************************************************************/
// Create ASCII output for datum.

void
ds_print_value(dataset_t *d, FILE *file, int v, int c)
{
  switch(d->vars[v].type) {
    case DS_NOTYPE:        // This only happens if a variable was un-initialized.
    case DS_TYPE_MAX:
      break;

    case DS_INT:           // 32 bit signed integer
      fprintf (file, "%d", (ds_int(d, v))[c]);
      break;

    case DS_FLOAT:         // 32 bit IEEE floating point
      fprintf (file, "%f", (ds_float(d, v))[c]);
      break;

    case DS_DOUBLE:        // 64 bit IEEE floating point
      fprintf (file, "%f", (ds_double(d, v))[c]);
      break;

    case DS_STRING:        // arbitrary length string value
      {
	const char *ptr = ds_get_string(d, v, c);
	if (ptr == NULL) ptr = "<null string>";
	fprintf (file, "\"%s\"", ptr);
      }
  }
}
/****************************************************************/
// Generate diagnostic output to stream.
void ds_print_info(dataset_t *d, FILE* file, int verbose)
{
  int v;
  fprintf(file, "    comment: %s\n", d->comment);
  fprintf(file, "  timestamp: %s"  , ctime(&d->timestamp));
  fprintf(file, "    columns: %d\n", d->columns);
  fprintf(file, "  variables: %d\n", d->variables);
  fprintf(file, "   startpos: %d\n", d->startpos);
  fprintf(file, "    samples: %d\n", d->samples);
  fprintf(file, "\n");

  if (verbose > 0) {
    fprintf(file, "  variables:\n");

    for (v = 0; v < d->variables; v++) {
      fprintf(file, "    %s\n", d->vars[v].name);
      if (verbose > 1) {
	fprintf(file, "      desc:  %s\n", d->vars[v].desc);
	fprintf(file, "      type:  %s\n", ds_get_type_string(d->vars[v].type));
	fprintf(file, "      units: %s\n", ds_get_units_string(d->vars[v].units));
	if (d->vars[v].type != DS_STRING) {
	  fprintf(file, "      upper: %g\n", d->vars[v].upper);
	  fprintf(file, "      lower: %g\n", d->vars[v].lower);
	}

	// For a very extended report, print out all values, one per line.
	if (verbose > 2) {
	  int c = 0; 
	  for (c = 0; c < d->columns; c++) {
	    fprintf (file, "        ");
	    ds_print_value(d, file, v, c);
	    fprintf (file, "\n");
	  }
	}
	fprintf(file, "\n");    
      }
    }
  }
}

/****************************************************************/
// Data access functions.
void 
ds_set_string(dataset_t *d, int v, int col, const char *s)
{
  if (d != NULL 
      && d->vars[v].type == DS_STRING 
      && col < d->columns) {

    char **ptr = & ((char **)(d->data[v]))[col];
    
    SAFE_FREE(*ptr);   // free any existing string
    *ptr = strdup(s);  // and copy this one
  }
}
const char *
ds_get_string(dataset_t *d, int v, int col)
{
  if (d != NULL 
      && d->vars[v].type == DS_STRING 
      && col < d->columns) {

    return ((char **)(d->data[v])) [col];    

  } else return NULL;
}


// Return array pointer for the specified variable, or NULL if the
// type doesn't match.  The pointer can be used for reading or writing,
// but be sure to mind the columns limit.

int *
ds_int(dataset_t *d, int v)
{
  if (d != NULL && d->vars[v].type == DS_INT) 
    return (int *)(d->data[v]);
  else 
    return NULL;
}

float  *
ds_float(dataset_t *d, int v)
{
  if (d != NULL && d->vars[v].type == DS_FLOAT) 
    return (float *)(d->data[v]);
  else 
    return NULL;
}

double *
ds_double(dataset_t *d, int v)
{
  if (d != NULL && d->vars[v].type == DS_DOUBLE) 
    return (double *)(d->data[v]);
  else 
    return NULL;
}

// Find a variable, returns the index or -1.
extern int ds_find_variable(dataset_t *d, const char *name)
{
  int v;
  if (d == NULL) return -1;

  for (v = 0; v < d->variables; v++) {
    if (!strcmp(d->vars[v].name, name)) return v;
  }
  return -1;
}

// Return a row of a dataset as a newly allocated array of doubles, or
// NULL if the row is not a numeric type.  The caller is responsible
// for freeing the array.  The array is of length "samples".
double *ds_row_as_double_array( dataset_t *d, int row )
{
  if ( d == NULL || row < 0 || row >= d->variables )  return NULL;
  else {
    struct dsVariable *var  = &d->vars[row];
    if ( var->type != DS_INT && var->type != DS_FLOAT && var->type != DS_DOUBLE ) return NULL;
    else {
      double *result = (double *) calloc (d->samples, sizeof(double) );
      int samples = d->samples;
      int srcpos  = d->startpos;
      void *data  = d->data[row];
      int i = 0;

      switch ( var->type ) {
      case DS_DOUBLE:
	while ( samples-- > 0 ) {
	  result[i++] = ((double *) data) [ srcpos++ ];
	  if ( srcpos >= d->columns ) srcpos = 0;
	}
	break;
      case DS_FLOAT:
	while ( samples-- > 0 ) {
	  result[i++] = (double) (((float *) data) [ srcpos++ ]);
	  if ( srcpos >= d->columns ) srcpos = 0;
	}
	break;
      case DS_INT:
	while ( samples-- > 0 ) {
	  result[i++] = (double) (((int *) data) [ srcpos++ ]);
	  if ( srcpos >= d->columns ) srcpos = 0;
	}
	break;
      }
      return result;
    }
  }
}
// Set an entire row at once.
void ds_set_row_from_double_array( dataset_t *d, int row, double *array )
{
  if ( d == NULL || row < 0 || row >= d->variables )  return;
  else {
    struct dsVariable *var  = &d->vars[row];
    if ( var->type != DS_INT && var->type != DS_FLOAT && var->type != DS_DOUBLE ) return;
    else {
      int samples = d->samples;
      int dstpos  = d->startpos;
      void *data  = d->data[row];
      int i = 0;

      switch ( var->type ) {
      case DS_DOUBLE:
	while ( samples-- > 0 ) {
	  ((double *) data) [ dstpos++ ] = array[i++];
	  if ( dstpos >= d->columns ) dstpos = 0;
	}
	break;
      case DS_FLOAT:
	while ( samples-- > 0 ) {
	  ((float *) data) [ dstpos++ ] = (float) array[i++];
	  if ( dstpos >= d->columns ) dstpos = 0;
	}
	break;
      case DS_INT:
	while ( samples-- > 0 ) {
	  ((int *) data) [ dstpos++ ] = (int) array[i++];
	  if ( dstpos >= d->columns ) dstpos = 0;
	}
	break;
      }
    }
  }
}
