// $Id: delay.c,v 1.1 2005/11/17 16:13:58 garthz Exp $
//
// delay.c : reliable sleeping
//
// Copyright (C) 1995-2005 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//

#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <utility/utility.h>

// nanosleep is almost perfect, except that it can be interrupted  by a signal, so this
// detects interruptions and resumes sleeping.

void delay_microseconds( unsigned long usecs )
{
  int err;
  int timeleft = 1;
  struct timespec duration, remainder;

  duration.tv_nsec = 1000 * (usecs % 1000000);
  duration.tv_sec  = usecs / 1000000;

  do {
    err = nanosleep( &duration, &remainder );

    if ( err == -1 && errno == EINTR ) {
      duration = remainder;
      continue;

    } else {
      timeleft = 0;
    }

  } while (timeleft);
}
