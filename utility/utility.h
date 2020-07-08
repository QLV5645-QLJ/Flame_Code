// $Id: utility.h,v 1.4 2005/12/14 17:29:52 garthz Exp $
// errprint.h : declarations for miscellaneous support routines
//
// Copyright (c) 1995-2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#ifndef UTILITY_H_INCLUDED
#define UTILITY_H_INCLUDED


// errprint.c
extern void errprintf(char *format, ...);
extern void logprintf(char *format, ...);
extern void redirect_stderr_to_file(char *errlog);

// delay.c
extern void delay_microseconds( unsigned long usecs );

// choose_filename.c
// This searches the current directory for systematically named
// data files based on the current date, and returns the next
// successive name.  Returns a new name allocated with strdup, or
// NULL on an error.
extern char *new_data_file_name(void);

// kbhit.c
extern int kbhit(void);

#endif // UTILITY_H_INCLUDED
