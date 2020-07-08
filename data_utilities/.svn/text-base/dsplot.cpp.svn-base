// dsplot.c : extract plot data from state variable trajectory recordings.
//
// Copyright (C) 1995-2001 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
// This is an updated version of dsconv.cc that uses my newer
// data set code.  It can extract ASCII data files from the
// binary data set.


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <utility/dataset.h>

// formats
enum {
  GNUPLOT,
  PARAMETRIC,
  MRDPLOT 
};

// global flags
static int verbose = 0;  

// forward declarations
void generate_plotfiles(dataset_t * ds, int argc, char **argv);
void generate_parametric(dataset_t * ds, int argc, char **argv, char *filename);
void generate_mrdplot(dataset_t * ds, int argc, char **argv, char *filename);

static char *ProgName;

void usage (void)
{
  fprintf(stderr,"\n");
  fprintf(stderr,"Usage: %s [-g|-p|-m] [-v][-i][-f<name>] [<vn> ...] <infile >outfile\n", ProgName);
  fprintf(stderr,"\n");
  fprintf(stderr,"  Format options, only one may be included:\n");
  fprintf(stderr,"    [-g] <varname> [<varname> ...]  output files for gnuplot.\n");
  fprintf(stderr,"    [-p] <varname> [<varname> ...]  parametric output file for gnuplot.\n");
  fprintf(stderr,"    [-m] [ <varname> ...]           data file for mrdplot.  With no names all\n");
  fprintf(stderr,"                                    channels are included. The default output\n");
  fprintf(stderr,"                                    is a file named mrddata.fig\n");
  fprintf(stderr,"\n");
  fprintf(stderr,"  Modifier options:\n");
  fprintf(stderr,"    [-v]            for verbose output.\n");
  fprintf(stderr,"    [-i]            to print file information. Use more than once for verbose output.\n");
  fprintf(stderr,"    [-f]<filename>  to specify a filename for single file output formats, including \n");
  fprintf(stderr,"                    mrdplot and gnuplot files.  There must be no space between -f\n");
  fprintf(stderr,"                    and the name.\n");
  exit(1);
}
/****************************************************************/

int main (int argc, char *argv[])
{
  int format = -1;
  int agc = argc;
  int info = -1;
  char **agv = argv;
  char *filename = NULL;
  dataset_t *d;

  /**************** process arguments ****************/

  ProgName = argv[0];
  if (agc > 1) {
    while (--agc > 0) {
      ++agv;
      if (**agv == '-') {
	if ((*agv)[1] == 'v') verbose++;
	else if ((*agv)[1] == 'i') info++;
	else if ((*agv)[1] == 'g') format = GNUPLOT;
	else if ((*agv)[1] == 'p') format = PARAMETRIC;
	else if ((*agv)[1] == 'm') format = MRDPLOT;
	else if ((*agv)[1] == 'f') filename = *agv + 2;
	else usage();
      } 	
    }
  }
  /****************/
  
  if (verbose) ds_error_stream(stderr);

#ifdef __MINGW32__
  setmode(fileno(stdin), O_BINARY); // to read binary files correctly
#endif

  d = new_dataset_from_stream(stdin, DS_UNSPECIFIED_LENGTH);

  if (d == NULL) {
    fprintf(stderr, "Error reading input stream.\n"); 
    exit(1);
  }

  if (info >= 0) ds_print_info(d, stderr, info);

  switch(format) {
  case GNUPLOT:
    generate_plotfiles(d, argc, argv);
    break;

  case PARAMETRIC:
    if (filename == NULL) filename = "output.data";
    generate_parametric(d, argc, argv, filename);
    break;

  case MRDPLOT:
    if (filename == NULL) filename = "mrddata.fig";
    generate_mrdplot(d, argc, argv, filename);
    break;

  default:
    break;
  }
  return 0;
}
/****************************************************************/

/* Generate a set of files for plotting. Each file contains one
   variables data, with one data point per line expressed as two 
   numbers in ASCII. The first number is the domain, and the second
   is the data.
*/
void generate_plotfiles(dataset_t * ds, int argc, char **argv)
{
  int idx;
  int timevar, var;
  FILE *out;

  // Look up the time variable to use as the ordinate (domain) variable.
  if ((timevar = ds_find_variable(ds, "t")) == -1) {
    fprintf(stderr, "Unable to find the time variable \"t\" to use for the ordinate axis.\n");
    return;
  }

  // Scan the list of arguments looking for variables to plot.
  while (--argc > 0) {
    ++argv;
    if (**argv != '-') {

      // look up the name
      if ((var = ds_find_variable(ds, *argv)) == -1) {
	fprintf(stderr, "Unable to find variable \"%s\"\n", *argv);
	continue;
      }

      // open output file
      if ((out = fopen(*argv, "wb")) == NULL) {   // The "b" is for non-Unix systems. 
	fprintf(stderr, "Error: cannot open output file %s.\n", *argv);
	continue;
      }

      // print out all values, one set per line
      for (idx = 0; idx < ds->samples; idx++) {
	ds_print_value(ds, out, timevar, idx);
	fprintf(out, " ");
	ds_print_value(ds, out, var, idx);
	fprintf(out, "\n");
      }
      fclose(out);
    }
  }
}

#define MAXVARS 100

/* Generate a single file with multicolumn data for plotting. 
*/
void generate_parametric(dataset_t * ds, int argc, char **argv, char *filename)
{
  int idx, v, vars;
  int var[MAXVARS];
  FILE *out;

  // find all the variables

  vars = 0;
  while (--argc > 0) {
    ++argv;
    if (**argv != '-') {

      // look up the name
      if ((var[vars] = ds_find_variable(ds, *argv)) == -1) {
	fprintf(stderr, "Unable to find variable \"%s\"\n", *argv);
	continue;
      }
      vars++;
    }
  }

  // Open the single multi-column output file.
  if ((out = fopen(filename, "wb")) == NULL) {
    fprintf(stderr, "Error: cannot open output file %s.\n", filename);
    return;
  }

  // Generate each row of the output from a column of the data.
  for (idx = 0; idx < ds->samples; idx++) {
    for (v = 0; v < vars; v++) {
      ds_print_value(ds, out, var[v], idx);
      fprintf(out, " ");
    }
    fprintf(out, "\n");
  }
  fclose(out);
}

/****************************************************************/
/* Functions to generate a file for mrdplot, a matlab-based
   plotting tool. */

// The mrdplot code assumes big-endian 32 floating point numbers.
// Internally the data values are stored as a 32 bit integer.
#define BIN_DATA_TYPE unsigned

// Convert native number to MRD binary format.
static inline BIN_DATA_TYPE mrd_datum(float f)
{
  union {
    float f;      // IEEE little-endian float
    unsigned char c[4];
    unsigned u;
  } little, big;
  
  // assume an Intel machine 
  little.f = f;
  
  big.c[0] = little.c[3];   // swap all bytes 
  big.c[1] = little.c[2];
  big.c[2] = little.c[1];
  big.c[3] = little.c[0];

  return big.u;
}

// Structure to define an MRD data set. 
typedef struct _MRD_DATA
{
  BIN_DATA_TYPE *data;      // array of 'data_len' data values in column order 
  int data_len;             // samples*dim
  int dim;                  // dimension of data vector
  int samples;              // total number of samples
  double freq;              // sampling frequency
  char **varNames;          // strings to save variable names
  const char **varUnits;    // strings to save variable units
} MRD_DATA;

// Create the buffers associated with the MRD data.
static void allocate_MRD(MRD_DATA *mrd)
{
  // Allocate a single large array for the numeric data.
  mrd->data = (BIN_DATA_TYPE *) malloc(sizeof (BIN_DATA_TYPE) * mrd->samples * mrd->dim);

  // Allocate an array of character pointers for the names and
  // the units.  Note that the strings themselves are not
  // allocated here.

  mrd->varNames = (char **) calloc(mrd->dim, sizeof(char *));
  mrd->varUnits = (const char **) calloc(mrd->dim, sizeof(char *));
}

// Write out the MRD data set to a file.  This must match the
// matlab code (in one case including space characters).

static int write_MRD_data(char *filename, MRD_DATA *mrd)
{
  int i;
  FILE *fp;
  fp = fopen(filename, "wb");
  if(fp == NULL) return 0;

  // write header
  fprintf(fp, "%d %d %d %g", mrd->data_len, mrd->dim, mrd->samples, mrd->freq);
  for(i = 0; i < mrd->dim; i++)
    fprintf(fp, "%s  %s  ", mrd->varNames[i], mrd->varUnits[i]);

  fprintf(fp, "\n");  // This whitespace length is assumed to be three chars(2 spc+cr). 

  // write binary data
  fwrite(mrd->data, sizeof(BIN_DATA_TYPE), mrd->samples*mrd->dim, fp);

  fclose(fp);

  if (verbose > 0) 
    printf("%d samples of numeric data of dimension %d written to '%s'.\n", 
	   mrd->samples, mrd->dim, filename);

  return 1;
}

/* Generate a mrdplot data file from the numeric variables within
   a data set. */

void generate_mrdplot(dataset_t * ds, int argc, char **argv, char *filename)
{
  MRD_DATA mrd;
  int v, s, count = 0, mrdvar = 0, datum = 0;

  // allocate a list large enough for all variables
  int *var = (int *) calloc( ds->variables, sizeof (int *));

  // Find all the variables specified on the command line, and include only those in the output.
  while (--argc > 0) {
    ++argv;
    if (**argv != '-') {
      // look up the name
      if ((v = ds_find_variable(ds, *argv)) == -1) {
	fprintf(stderr, "Unable to find variable \"%s\"\n", *argv);
	continue;

      } else if (ds->vars[v].type == DS_INT || 
		 ds->vars[v].type == DS_FLOAT || 
		 ds->vars[v].type == DS_DOUBLE) {
	var[count++] = v;
      } else {
	fprintf(stderr, "Found variable \"%s\", but it is not numeric, ignoring it.\n", *argv);
      }
    }
  }

  // If no variables were found, just assume that all numeric variables should be included.
  if ( count == 0 ) {
    fprintf(stderr, "No (valid) variables specified, so all numeric channels will be included.\n");

    // Count the number of numeric variables, and include all in the output file
    for (v = 0; v < ds->variables; v++) {
      if (ds->vars[v].type == DS_INT || 
	  ds->vars[v].type == DS_FLOAT || 
	  ds->vars[v].type == DS_DOUBLE) {
	var[count++] = v;
      }
    }
  }

  fprintf(stderr, "Including %d variables in mrdplot file.\n", count);

  mrd.samples  = ds->samples;
  mrd.dim      = count;
  mrd.data_len = mrd.samples * mrd.dim;

  // Determine the sampling rate from the usual controlling variable.
  {
    int timestep_var = ds_find_variable(ds, "record_dt");
    if ( timestep_var == -1 ) {
      // fprintf(stderr, "Warning: no record_dt variable found, assuming 1000 Hz sampling.\n");
      mrd.freq     = 1000;
    } else {
      if ( ds->samples != 0 ) {
	double dt;
	switch ( ds->vars[ timestep_var ].type ) {
	case DS_FLOAT:  dt = (ds_float(ds, timestep_var))[0]; break;
	case DS_DOUBLE: dt = (ds_double(ds, timestep_var))[0]; break;
	default: 
	  // fprintf(stderr, "Warning: record_dt not a real number, assuming 1000 Hz sampling.\n");
	  dt = 0.001;
	  break;
	}
	mrd.freq = 1.0 / dt;
	fprintf(stderr, "Using sampling rate of %f Hz.\n", mrd.freq);
      } 
      else mrd.freq = 1000;  // no samples, doesn't matter
    }
  }

  // create data buffers
  allocate_MRD(&mrd);

  // Copy dataset to MRD structure.

  // copy over pointers to the variable and units names
  for (mrdvar = 0; mrdvar < count; mrdvar++) {
    int v = var[mrdvar];  // look up associated variable index

    mrd.varNames[mrdvar] = ds->vars[v].name;

    if ( ds->vars[v].units == DS_DIMENSIONLESS ) {
      mrd.varUnits[mrdvar] = "-";   // this is easier on the eyes
    } else {
      mrd.varUnits[mrdvar] = ds_get_units_string(ds->vars[v].units);
    }
  }

  // and copy the data over by columns

  datum = 0;             // index into output array

  for (s = 0; s < ds->samples; s++) {
    for (mrdvar = 0; mrdvar < count; mrdvar++ ) {
      int v = var[mrdvar];  // look up associated variable index


      switch(ds->vars[v].type) {
      case DS_INT:
	mrd.data[datum++] = mrd_datum ((float) ((ds_int(ds, v))[s]));
	break;

      case DS_FLOAT:
	mrd.data[datum++] = mrd_datum ((float) ((ds_float(ds, v))[s]));
	break;

      case DS_DOUBLE:
	mrd.data[datum++] = mrd_datum ((float) ((ds_double(ds, v))[s]));
	break;

      default: // oops
	fprintf(stderr, "Warning: an unsupported type slipped through, mrdplot file invalid.\n");
	break;
      }
    }
  }

  // write the file
  if (!write_MRD_data(filename, &mrd)) {
    fprintf(stderr, "Error writing MRD file.\n");
  }    
}
