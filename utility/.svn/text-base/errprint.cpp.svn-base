// $Id: errprint.c,v 1.1 2005/11/17 16:13:58 garthz Exp $
//
// errprint.c : support for error and logging streams
//
// Copyright (C) 1995-2005 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
// #include <sys/time.h>
// #include <dirent.h>
#include <time.h>
// #include <unistd.h>
#include <stdlib.h>  // for exit()
#include <stdarg.h>

#include "utility.h"

/****************************************************************/
// Common entry point for error messages. Adds a timestamp.
void errprintf(char *format, ...)
{
  va_list args;
  char nowstr[26];
  time_t now = time(NULL);

  va_start(args, format);
  strcpy(nowstr, ctime(&now));
  nowstr[24] = ':';                 // replace newline terminator
  fprintf(stderr, "%s ", nowstr);
  vfprintf(stderr, format, args);
  fflush(stderr);

  va_end(args);
}

/****************************************************************/
// Common entry point for normal logging messages, to allow
// future redirection to a log file.

void logprintf(char *format, ...)
{
  va_list args;

  va_start(args, format);
  vfprintf(stdout, format, args);
  fflush(stderr);

  va_end(args);
}

/****************************************************************/
void redirect_stderr_to_file(char *errlog)
{
  /* redirect the error stream to a file */
  if (freopen(errlog, "w", stderr) == NULL) {
    time_t now = time(NULL);
    printf("Unable to open %s at %squitting...\n", errlog, ctime(&now));
    exit(1);
  }
  errprintf("Opened %s.\n", errlog);
}
