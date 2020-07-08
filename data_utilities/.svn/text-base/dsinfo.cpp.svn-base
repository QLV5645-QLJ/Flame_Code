// dsinfo.c : print summary information about a state variable
// trajectory recording.
//
// Copyright (C) 1995-2001 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <utility/dataset.h>

static int verbose = 0;

void
file_info(char *name)
{
  FILE *in;
  dataset_t *d;

  // The "b" is for non-UNIX systems.
  in = fopen(name, "rb");
  if (in == NULL) {
    fprintf(stderr, "Unable to open %s: ", name);
    perror("");
    return;
  }
  // enable more verbose debugging
  ds_error_stream( stderr );

  // read in the entire data file
  d = new_dataset_from_stream(in, DS_UNSPECIFIED_LENGTH);
  fclose(in);

  if (d == NULL) {
    fprintf(stderr, "Unable to read %s.\n", name);
  } else {
    ds_print_info(d, stdout, verbose);
  }
  delete_dataset(d);
}


int main (int argc, char **argv)
{
  char **arg = argv;       /* pointer to walk down argument list */
  /* interpret the flags as a script */

  while (--argc > 0) {
    ++arg;

    // If a flag
    if (**arg == '-') {
      if ((*arg)[1] == 'v') {
	if (isdigit((*arg)[2])) {
	  verbose = atoi(*arg + 2);
	} else verbose++;
      }

      else {
	fprintf(stderr, "Usage: dataset_info [-v[#]] filename [-v[#]] [filename] ...\n");
	exit(1);
      }
    }

    // else a bare name, treat as a filename
    else file_info(*arg);
  }
  return 0;
}
