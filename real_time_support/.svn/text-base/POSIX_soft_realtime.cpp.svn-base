// $Id: POSIX_soft_realtime.c,v 1.1 2005/11/17 16:14:48 garthz Exp $
// POSIX_soft_realtime.c : interface to the standard Linux POSIX real time scheduler
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include <stdarg.h>

#include <utility/utility.h>

/****************************************************************/
int
initialize_POSIX_soft_realtime(void)
{
  struct sched_param params;
  int err;
  struct timeval now, then;

  int uid = getuid();
  int euid = geteuid();
  
  logprintf("initialize_POSIX_soft_realtime running as UID %d and EUID %d.\n", uid, euid);
  logprintf("Attempting to enable real-time priority scheduler.\n");
  if ( euid != 0 ) errprintf("Not root, so this shouldn't work.\n");

  params.sched_priority = 1;            // can be 1 to 99
  err = sched_setscheduler( 0,          // pid_t pid, zero means this one
			    SCHED_FIFO, // int policy, SCHED_FIFO means POSIX real-time with FIFO algorithm
			    &params );  // const struct  sched_param

  if (err) {
    errprintf( "initialize_POSIX_soft_realtime couldn't set real-time priority: %s\n", strerror(errno));
    return -1;

  } else {
    return 0;
  }
}
