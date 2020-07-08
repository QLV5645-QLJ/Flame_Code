// $Id: IO_permissions.c,v 1.2 2005/12/07 15:25:38 garthz Exp $
// IO_permissions.c : utility routines to obtain port access under Linux
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
#include <stdio.h>
#include <sys/io.h>
#include <asm/io.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <signal.h>

#include <utility/utility.h>

/****************************************************************/
void enable_IO_port_access(void)
{
  // logprintf( "Requesting permission for I/O port access.\n" );

#if 1
  // Get access to ALL ports.
  if (iopl(3)) {
    errprintf("enable_port_access: iopl failed: %s", strerror(errno));
  }
#endif

#if 0
  // In principle this could be used to get access to specific
  // ranges of ports.

  if (ioperm( (unsigned long) 0x070, (unsigned long) 17, (int) 1) ) {
    errprintf("enable_port_access: ioperm failed: %s", strerror(errno));
  }
#endif
}
