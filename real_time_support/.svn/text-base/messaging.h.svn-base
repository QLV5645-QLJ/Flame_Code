// $Id: messaging.h,v 1.4 2005/12/14 08:32:50 garthz Exp $
// messaging.h : An abstracted interface for sending formatted messages to and from a real time process.
//
// Copyright (c) 2001-2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.


#ifndef MESSAGING_H_INCLUDED
#define MESSAGING_H_INCLUDED

#include <real_time_support/messages.h>

// A simple abstraction of a port which can send and receive
// message packets of variable but bounded length.  This might be
// implemented via RTAI mailboxes, shared memory, a TCP or UDP
// network, etc.

// This is similar to a C++ abstract class, only with virtual
// function method pointers contained with the object itself.
// The goal here is to provide a simple, transparent means to
// communicate between all the different processes in the system.

// It isn't strictly defined whether message ports are one-way or
// two-way, but each port represents a single logical connection,
// since no other address is specified when sending a message.

typedef struct message_port_t {

  // general purpose pointers
  void *userdata;

  // method pointers
  void (*send) ( struct message_port_t *, union message_t *); // non-blocking send
  int  (*recv) ( struct message_port_t *, union message_t *); // non-blocking receive
  void (*close)( struct message_port_t *);                    // free system resources

  // state flags
  unsigned int initialized   :1;
  unsigned int closed        :1;
  unsigned int header_valid  :1;

  // Some implementations benefit from first receiving the header
  // to get the message length; this provides a space to keep the
  // header for a partially received packet.
  struct _msg_hdr header;

} message_port;

// Object creation and destruction.
extern message_port *message_port_init( message_port * );
extern void message_port_close( message_port *port);

extern message_port *message_port_alloc( void );
extern void message_port_dealloc( message_port * );

// General non-blocking receive. Returns true if a message was supplied.
extern int message_receive( message_port *port, union message_t *msgarea );

// Send a general message to a port.
extern void send_message( message_port *port, union message_t *msg );

// Message sending functions.
extern void send_signal( message_port *port, enum msg_subtypes signal );
extern void send_print( message_port *port, char *str );
extern void send_pong( message_port *port );

#define MESSAGE_PORT_READY(port) (((port)!=NULL)&&((port)->initialized)&&(!((port)->closed)))

#endif // MESSAGING_H_INCLUDED
