// $Id: UDP_messaging.h,v 1.1 2005/12/07 15:24:46 garthz Exp $
// UDP_messaging.h : simple UDP transport for message packets
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
// Define a simple wrapper for UDP sockets to support
// connectionless network communication of messages.  Since UDP
// is not intrinsically bidirectional, each message port can
// be input or output or both.
//
//   The output port binds a socket port to use as the source
//   address and looks up and1 records a particular destination
//   host port.
//
//   The input port binds a socket port to use as a receiving
//   address which receives messages from any host. It is up to
//   the application protocol to set up the correct set of ports
//   and filter out extraneous traffic.
// 
//   The bidirectional port shares the underlying socket, so the
//   same port number is used as the source address and as a
//   destination address.  However, the input can be configured
//   to ignore messages not from the remote port.
 

#ifndef __UDP_MESSAGING_H_DEFINED__
#define __UDP_MESSAGING_H_DEFINED__

#include <real_time_support/messaging.h>

// This loosely follows Objective-C semantics.  Given a
// dynamically or statically allocated message_port structure,
// initialize it to be a specific port type.  It can be closed
// or deallocated with the generic message_port functions.

// The localname is optional; if null it defaults to "localhost".
// This is useful to distinguish between multiple interfaces on
// the same host.

extern message_port *init_UDP_message_port( message_port *port, 
					    char *localname, int local_port, 
					    char *remotename, int remote_port, 
					    int flags );
// Flag values to be or-ed together.
#define UDP_MESSAGE_PORT_INPUT 1
#define UDP_MESSAGE_PORT_OUTPUT 2
#define UDP_MESSAGE_EXCLUSIVE_INPUT 4  

// UDP_MESSAGE_EXCLUSIVE_INPUT is only valid for input-output
// ports; if true, will ignore input messages from hosts other
// than the designated remote host.

#endif // __UDP_MESSAGING_H_DEFINED__
