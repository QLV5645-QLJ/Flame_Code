// $Id: choose_filename.c,v 1.3 2005/12/15 16:01:20 garthz Exp $ 
//
// choose_filename.c : odds and ends to support biped control code.
//
// Copyright (C) 1995-2005 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <errno.h>
#include <dirent.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>  // for exit()
#include "utility.h"

/****************************************************************/
/* This searches the current directory for systematically named
   data files based on the current date, and returns the next
   successive name.  Returns a new name allocated with strdup, or
   NULL on an error. */

char *new_data_file_name(void)
{
  DIR *dir;            // directory entry stream
  time_t clock;        // various representations of the current time
  struct tm converted_time;
  struct tm *now;
  char basename[15];   // partial file name
  int suffix;          // the suffix found after the base name, which should be an index number
  int max = 0;         // the maximum index number found

  struct dirent *dirp;

  // Open the current directory.
  // logprintf("new_data_file_name opening the current directory.\n");

  dir = opendir( "." );
  if ( dir == NULL ) {
    errprintf("new_data_file_name unable to open current directory: %s\n", strerror(errno));
    return NULL;
  }

  // Create the basic systematic name: YY-MM-DD-<index>
  clock = time(NULL);
  now = localtime_r( &clock, &converted_time );
  sprintf(basename, "%02d-%02d-%02d-", now->tm_year % 100, now->tm_mon+1, now->tm_mday);

  // Scan the directory looking for matching names.
  for (dirp = readdir(dir); dirp != NULL; dirp = readdir(dir)) {

    // If the name is too short, skip it.
    if (strlen(dirp->d_name) < 10) continue;

    // If the base name matches, look for an index number 
    if (!strncmp(basename, dirp->d_name, 9)) {
      if (sscanf(dirp->d_name+9, "%d", &suffix) < 1) continue;
      
      // keep track of the greatest index number seen
      if (suffix > max) max = suffix;
    }
  }
  closedir(dir);
  
  // The new index number will be one greater than any existing.
  max++;

  // Some kind of trivial sanity check.
  if (max > 99999) return NULL;

  // insert the index number in the name
  sprintf(basename+9, "%d", max);
  
  return strdup(basename);
}
