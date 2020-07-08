// $Id: kbhit.c,v 1.1 2005/12/14 17:29:52 garthz Exp $ 
// kbhit.c : non-blocking check of keyboard input
//
// Copyright (C) 1995-2005 Garth Zeglin.  Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <utility/utility.h>


// Report whether input is availabe on stdin.  If the console is
// in the usual 'cooked' mode, input is line-buffered so this
// will not respond until Enter is pressed.  If the console is in
// raw mode it will respond to an individual key.
int kbhit(void)
{
  fd_set fdset;        // file descriptor bit flags for select()
  struct timeval timeout;   

  int fd = fileno(stdin);
  
  // Create a fd_set and timeout for performing select() on stdin.
  FD_ZERO( &fdset );
  FD_SET( fd, &fdset );
  timeout.tv_usec = timeout.tv_sec  = 0;

  if (select( fd + 1, &fdset, NULL, NULL, &timeout ) == -1) {
    errprintf("select failed in kbhit(): %s", strerror(errno));
    return 0;
  }
  return FD_ISSET( fd, &fdset );
}
