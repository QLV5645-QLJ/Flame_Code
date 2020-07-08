// $Id: messaging.c,v 1.4 2005/12/14 08:32:50 garthz Exp $
// An abstracted interface for sending formatted messages to and from a real time process.
//
// Copyright (c) 2001-2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.
//
// It is a thin layer over a set of send and receive functions.

#include <stdlib.h>
#include <real_time_support/messaging.h>
#include <real_time_support/protocol_version.h>
#include <utility/utility.h>
#include <string.h>

message_port *
message_port_alloc( void )
{
  // It would be better if this were a compile time check, but
  // the preprocessor doesn't know the union size.

  if (sizeof(union message_t) > MSG_MTU) {
    errprintf( "Error: the message union type is larger than the maximum message packet size.  This is really a compile-time problem, see messages.h.\n");
  }

  return (message_port *) calloc(1, sizeof(message_port) );
}

message_port *
message_port_init( message_port *port )
{
  if ( port != NULL ) {
    port->userdata = NULL;
    port->send = NULL;
    port->recv = NULL;
    port->close = NULL;
    port->initialized = 0;
    port->closed = 0;
    port->header_valid = 0;
  }
  return port;
}

void
message_port_close( message_port *port)
{
  if (port != NULL) {
    if (port->initialized && !port->closed && port->close != NULL) (*port->close)(port);
    port->closed = 1;
  }
}

void
message_port_dealloc( message_port *port)
{
  if (port != NULL) {
    if (port->initialized && !port->closed && port->close != NULL) (*port->close)(port);
    free(port);
  }
}

/****************************************************************/
// Compute a checksum on a message.  Messages should sum to zero.
static unsigned char message_checksum(union message_t *msg)
{
  unsigned char accum = 0;
  int i;
  if ((msg == NULL) || msg->header.length > MSG_MTU) return 1;
  for (i = 0; i < msg->header.length; i++) accum += msg->raw_data[i];
  return accum;
}
/****************************************************************/
// General non-blocking receive. Returns true if a message was supplied.
int 
message_receive( message_port *port, union message_t *msgarea )
{
  if ( port != NULL && port->initialized && !port->closed && port->recv != NULL && msgarea != NULL ) {
    return (*port->recv)( port, msgarea );
  } else {
    return 0;
  }
}


/****************************************************************/
// Send a message to a port.
void
send_message( message_port *port, union message_t *msg )
{
  // If the pipe is closed, throw it away.
  if ( port != NULL && port->initialized && !port->closed && port->send != NULL) {
    unsigned short sum;
    int err;

    // first fill in the packet checksum
    msg->header.checksum = 0;
    sum = message_checksum(msg);
    msg->header.checksum = (unsigned char) (0x100 - sum);

    // then call the (non-blocking) send function
    (*port->send)(port, msg );
  }
}

/****************************************************************/
// Message formatting functions.

// Send any of the signaling messages that have no data, i.e., the packet has just a header.

void send_signal( message_port *port, enum msg_subtypes signal )
{
  union message_t sendbuf;  // a buffer for formatting the message

  sendbuf.header.type     = FLAME_MESSAGE;
  sendbuf.header.subtype  = signal;
  sendbuf.header.length   = sizeof (sendbuf.header);
  send_message( port, &sendbuf );
}

// Send a print request.  The string in the packet isn't zero-terminated, mostly to force
// the recipient to honor the length field, which will be more robust in the event of packet
// truncation.

void send_print( message_port *port, char *str )
{
  union message_t sendbuf;  // a buffer for formatting the message
  int len = strlen(str);

  if (len > MSG_MAXDATA) len = MSG_MAXDATA;

  sendbuf.header.type     = FLAME_MESSAGE;
  sendbuf.header.subtype  = MSG_PRINT;
  sendbuf.header.length   = sizeof(sendbuf.header) + len;
  memcpy(&sendbuf.print.data, str, len);

  send_message( port, &sendbuf );
}

// Send a reply to a ping.
void send_pong( message_port *port )
{
  union message_t sendbuf;  // a buffer for formatting the message

  sendbuf.header.type     = FLAME_MESSAGE;
  sendbuf.header.subtype  = MSG_PONG;
  sendbuf.header.length   = sizeof(sendbuf.pong);
  sendbuf.pong.version    = MESSAGE_PROTOCOL_VERSION;
  send_message( port, &sendbuf );
}

