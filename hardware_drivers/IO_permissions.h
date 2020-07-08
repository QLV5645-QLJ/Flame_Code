// $Id: IO_permissions.h,v 1.1 2005/11/24 17:30:58 garthz Exp $
// IO_permissions.h : obtain port I/O permission under Linux
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#ifndef IO_PERMISSIONS_H_INCLUDED
#define IO_PERMISSIONS_H_INCLUDED

// Enable access to the I/O ports under Linux.  Must be root for this to work.
extern void enable_IO_port_access(void);

#endif // FLAMEIO_H_INCLUDED

