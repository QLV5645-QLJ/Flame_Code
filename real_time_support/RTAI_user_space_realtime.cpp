// $Id: RTAI_user_space_realtime.c,v 1.9 2005/12/16 16:15:04 garthz Exp $
// RTAI_user_space_realtime.c : interface to the RTAI lxrt user space realtime functionality
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <pthread.h>
#include <rtai_bits.h>
#include <rtai_lxrt.h>
#include <rtai_mbx.h>
#include <rtai_msg.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>

#include <utility/utility.h>

/****************************************************************/
// The opaque task structure returned to user code, intended to
// hide all the details of the RTAI interface.

struct realtime_task {

  RT_TASK *task;   // handle for the RTAI master task this structure represents

  // Variables for keeping statistics on timing.
  int ticks;
  double total_interval;
  double total_squared_interval;
  double max_interval;
  double min_interval;
};

/****************************************************************/
struct realtime_task *
create_RTAI_user_space_task( char *name )
{
  struct realtime_task *task = (struct realtime_task *) calloc(1, sizeof( struct realtime_task ) );
  int err;

  if ( task == NULL ) {
    errprintf("unable to allocate memory in initialize_RTAI_user_space_realtime.");
    return NULL;
  }

  task->ticks = 0;
  task->total_interval = 0;
  task->total_squared_interval = 0;
  task->max_interval = 0.0;
  task->min_interval = 1e38;

  logprintf("Creating master task.\n");
  task->task = rt_task_init_schmod( nam2num( name ),    // name
				    0,                    // priority
				    0, 	            // stack_size
				    0, 		    // max_msg_size
				    SCHED_FIFO,           // policy
				    0xF                   // cpus_allowep
				    );
  if ( task->task == NULL ) {
    errprintf("Cannot init master task.\n");
    goto fail;
  }

  logprintf("Locking memory pages down.\n");
  if ( mlockall(MCL_CURRENT | MCL_FUTURE) ) {
    errprintf("Unable to lock memory: %s\n", strerror(errno) );
    goto fail;
  }

  // All of our tasks use floating point.
  logprintf("Enabling real-time FPU use.\n");
  rt_task_use_fpu( task->task, 1 );
  rt_linux_use_fpu( 1 );

  return task; // otherwise, success!

fail:
  errprintf("Error in create_RTAI_user_space_task, deallocating resources.\n");
  if ( task->task != NULL) rt_task_delete( task->task );
  free(task);
  return NULL;
}

/****************************************************************/
void shutdown_RTAI_user_space_task( struct realtime_task *task )
{
  // logprintf("Shutting down real time resources.\n");

  if ( task == NULL ) {
    errprintf("shutdown_user_space_realtime error: task doesn't exist.\n");
  } else {
    if ( task->task != NULL ) rt_task_delete( task->task );
    free(task);
  }
}

/****************************************************************/
int run_RTAI_user_space_realtime_periodic_thread( struct realtime_task *task,
						  int (*realtime_thread)( long long timestamp, void *userdata ), 
						  int period_in_nanoseconds,
						  void *userdata )
{
  logprintf("run_RTAI_user_space_realtime_periodic_thread beginning.\n");

  if ( task == NULL || task->task == NULL ) {
    errprintf("run_RTAI_user_space_realtime_periodic_thread error: the realtime task doesn't exist\n");
    return -1;
  }

  logprintf("Initializing timer.\n");

  // rt_set_periodic_mode();
  rt_set_oneshot_mode();
  start_rt_timer( nano2count( period_in_nanoseconds ) );  // In the oneshot mode the period value is ignored.

  logprintf( "The timer period is %d nanoseconds, or %d counts.\n", 
	     period_in_nanoseconds,
	     nano2count( period_in_nanoseconds ) );

  logprintf( "Entering hard real time mode.\n");

  rt_make_hard_real_time();

  // Now in real time mode, must not do anything that triggers a system call.
  {
    int loopcount;
    RTIME period, start_time, now, then;
    int keep_running = 0;

    // Convert nanoseconds to internal count units.
    period = nano2count( period_in_nanoseconds ); 

    // Get the current time (in counts), add five periods to get the start time.
    start_time = rt_get_time() + 5*period;        

    // Start the timer interrupts.
    rt_task_make_periodic( task->task, start_time, period);

    // Wait for start time to expire.
    rt_task_wait_period();        

    // Get the current time in nanoseconds.
    then = rt_get_cpu_time_ns();  

    // Enter the event loop.
    do {
      double interval;

      rt_task_wait_period();
      now = rt_get_cpu_time_ns();  // get the current time in nanoseconds.

      // Do something timely.
      if ( realtime_thread != NULL ) {
	keep_running = (*realtime_thread)( (long long) now, userdata );
      } else keep_running = 0;

      // Keep track of timing statistics.
      interval = (double) (now - then);
      task->total_interval += interval;
      task->total_squared_interval += interval*interval;
      if ( interval < task->min_interval ) task->min_interval = interval;
      if ( interval > task->max_interval ) task->max_interval = interval;
      task->ticks++;
      then = now;
      
    } while ( keep_running );

    // Exit event loop, leave hard real-time mode.
    rt_make_soft_real_time();
    stop_rt_timer();	
  }
}
/****************************************************************/
void RTAI_usr_print_task_statistics( struct realtime_task *task )
{
  double stdev;

  // Compute some simple central measures; mean and standard deviation.
  stdev = sqrt( ( (double) task->ticks * task->total_squared_interval - task->total_interval * task->total_interval) /
		( (double) task->ticks * (double)(task->ticks - 1)) );

  logprintf("Ran for %d ticks.\n", task->ticks);
  logprintf("Total run time      : %f seconds\n", 1e-9 * task->total_interval);
  logprintf("Average loop time   : %f msec\n", 1e-6 * task->total_interval / task->ticks);
  logprintf("Std dev of loop time: %f msec\n", 1e-6 * stdev);
  logprintf("Minimum interval    : %f msec\n", 1e-6 * task->min_interval);
  logprintf("Maximum interval    : %f msec\n", 1e-6 * task->max_interval);
}

/****************************************************************/
// Attempt to detect if RTAI is loaded.  This is something of
// hack, but I don't yet know the right way.
int RTAI_is_ready(void)
{
  FILE *proc;
  char line_buffer[200]; 

  proc = fopen("/proc/modules", "r" );
  if ( proc == NULL ) {
    errprintf("RTAI_is_ready unable to open /proc/modules: %s\n", strerror(errno));
    return 0;
  }

  while (!feof(proc)) {
    if ( fgets( line_buffer, 200, proc ) == NULL ) break;

    if ( !strncmp ("rtai_lxrt", line_buffer, 9 ) ) {
      logprintf("RTAI_is_ready found RTAI module, assuming RTAI loaded.\n");
      fclose(proc);
      return 1;
    }
  }
  fclose(proc);

  errprintf( "RTAI_is_ready did not find indicative RTAI module.\n");
  
  return 0;
}
