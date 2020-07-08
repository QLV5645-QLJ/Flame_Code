// $Id: RTAI_user_space_realtime.h,v 1.6 2005/12/12 17:25:31 garthz Exp $
// RTAI_user_space_realtime.h : interface to the RTAI lxrt user space realtime functionality
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#ifndef RTAI_USER_SPACE_REALTIME_H_INCLUDED
#define RTAI_USER_SPACE_REALTIME_H_INCLUDED

// These names are getting somewhat long, but hey, that's what emacs completion is 
// for: ESC-/, otherwise known as dabbrev-expand.

// This returns an opaque task structure or NULL.
extern struct realtime_task *create_RTAI_user_space_task( char *name );

extern int run_RTAI_user_space_realtime_periodic_thread( struct realtime_task *task,
							 int (*realtime_thread)( long long timestamp, void *userdata ), 
							 int period_in_nanoseconds,
							 void *userdata );

// Can be called after run_RTAI_user_space_realtime_periodic_thread to print out cycle time averages.
extern void RTAI_usr_print_task_statistics( struct realtime_task *task );

// Must be called to clean up RTAI resources.
extern void shutdown_RTAI_user_space_task( struct realtime_task *task );

// Test if RTAI is loaded into the kernel
extern int RTAI_is_ready(void);

#endif // RTAI_USER_SPACE_REALTIME_H_INCLUDED
