// $Id: RTAI_mailbox_messaging.c,v 1.1 2005/12/07 15:24:46 garthz Exp $
// RTAI_mailbox_messaging.c : support for message communications using RTAI lxrt user space realtime mailboxes
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

// Support for messaging using a RTAI mailbox.  The RTAI
// documentation for this is poor, but I have observed some
// details.  Messages may be submitted to mailboxes as a packet,
// but they act like a stream; a partial message can be retrieved
// in a receive call, and then the balance in a subsequent call.
// Mailboxes which are not closed persist between program
// invocations, and any unread data is also kept in the buffer.

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
#include <real_time_support/messaging.h>
#include <real_time_support/messages.h>

// Number of full message structures (large) which the mailbox might hold.
#define MAX_MAILBOX_SIZE_IN_MESSAGES 10

/****************************************************************/
// Private functions for sending and receiving.
static void
RTAI_mailbox_send_message( message_port *port, union message_t *msg )
{
  int unsent_bytes;
  if ( port != NULL && port->initialized && !port->closed && 
       port->userdata != NULL && msg != NULL && msg->header.length > 0 ) {

    unsent_bytes = rt_mbx_send_if( (MBX *) port->userdata, msg, msg->header.length );
    // if unsent_bytes is non-zero, then the packet isn't put in the mailbox and is simply dropped
  }
} 
//-----------------------------------
// Poll to see if a message is available, returns true if one is available.
static int
RTAI_mailbox_recv_message( message_port *port, union message_t *msgarea )
{
  if ( port != NULL && port->initialized && !port->closed && msgarea != NULL ) {

    int unreceived;

    // If no header has been previously buffered (the normal case).
    if ( !port->header_valid ) {

      // First get the header atomically.
      unreceived = rt_mbx_receive_if( (MBX *) port->userdata, &port->header, sizeof( port->header ));

      // Either receive a header, or exit.
      if ( unreceived == 0 ) port->header_valid = 1; 
      else return 0;
    }

    // If this point is reached, the header has been successfully
    // received, so return the header if it is a complete
    // message, or try to receive any message body atomically.

    if ( port->header.length == sizeof(port->header) ) {
      // the header is a complete message, return it
      memcpy( msgarea, &port->header, sizeof( port->header ));
      port->header_valid = 0;
      return 1;

    } else {
      // Try to get a message body.
      unreceived = rt_mbx_receive_if( (MBX *) port->userdata, 
				      (void *) (((char *) msgarea) + sizeof( port->header )), 
				      port->header.length - sizeof(port->header));

      // check if a message was received
      if ( unreceived == 0 ) {
	memcpy( msgarea, &port->header, sizeof( port->header ));
	port->header_valid = 0;
	return 1;
      }
    }
  }
  return 0;
}
//-----------------------------------
static void
RTAI_mailbox_close( message_port *port )
{
  if ( port != NULL ) { 

    // It is arguable whether we actually want to close the
    // underlying mailbox; any messages remaining in the pipe
    // will be thrown away, so a sender cannot ensure that final
    // messages will be received.  Or it is possible someone
    // would want to have multiple threads feeding one box, which
    // might call for leaving it open even when just one thread
    // quits.

    // if ( port->userdata != NULL ) rt_mbx_delete( (MBX *) port->userdata );

    port->userdata = NULL;
    port->closed = 1;
  }
}

/****************************************************************/

message_port *
init_RTAI_mailbox_message_port( message_port *port, char *name )
{
  MBX *mailbox = NULL;

  if (port == NULL) return port;

  // if the port already is something, reset it
  if ( port->initialized && !port->closed && port->close != NULL ) (*port->close)(port);

  // reset to a generic state
  message_port_init( port );

  // validate the name input
  if ( name == NULL || strlen(name) != 6 ) return port;

  // first test if it already exists
  mailbox = (MBX*) rt_get_adr( nam2num( name ) );

  if ( mailbox != NULL ) {
    // This is not an error, since one of the endpoints will 
    // create it and the others will find it.
    logprintf("RTAI_usr_create_mailbox_message_port: found existing %s.\n", name);

  } else {
    logprintf("RTAI_usr_create_mailbox_message_port: Creating mailbox.\n");
    mailbox = (MBX*) rt_mbx_init( nam2num( name ), MAX_MAILBOX_SIZE_IN_MESSAGES * sizeof( union message_t) );
    if ( mailbox == NULL ) {
      errprintf("RTAI_usr_create_mailbox_message_port: cannot create mailbox.\n");
      return port;
    }
  }
  port->userdata = (void *) mailbox;
  port->send = RTAI_mailbox_send_message;
  port->recv = RTAI_mailbox_recv_message;
  port->close = RTAI_mailbox_close;
  port->initialized = 1;
  port->closed = 0;

  return port;
}
