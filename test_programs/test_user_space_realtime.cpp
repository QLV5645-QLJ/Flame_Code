#include <stdio.h>

#include <real_time_support/RTAI_user_space_realtime.h>
#include <real_time_support/messages.h>
#include <utility/utility.h>

// opaque handle for the real time control thread task
static struct realtime_task *control_task = NULL;

/****************************************************************/
// This function will be called at regular intervals.
// No system calls are allowed inside this function. 
// Returns true if the periodic calls should continue.
int realtime_thread( long long timestamp, void *userdata )
{
  static int count = 0;
  
  count++;

  return (count < 5000);
}

/****************************************************************/
int main(int argc, char **argv) 
{
  logprintf("test_user_space_realtime starting up.\n");

  control_task = create_RTAI_user_space_task( REALTIME_PROCESS_NAME );
  if ( control_task == NULL ) {
    errprintf("Failed to initialize RTAI interface.\n");
    exit(1);
  }

  logprintf("starting real time thread, it should run for five seconds...\n");
  run_RTAI_user_space_realtime_periodic_thread(  control_task, realtime_thread, 1000000 /* nanoseconds */, NULL );

  logprintf("Real time thread exited.\n");
  
  shutdown_RTAI_user_space_task( control_task );

  logprintf("test_user_space_realtime done.\n");
}
