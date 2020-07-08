// $Id: RTAI_mailbox_messaging.h,v 1.1 2005/12/07 15:24:46 garthz Exp $
// RTAI_mailbox_messaging.h : support for message communications using RTAI lxrt user space realtime mailboxes
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#ifndef RTAI_MAILBOX_MESSAGING_H_INCLUDED
#define RTAI_MAILBOX_MESSAGING_H_INCLUDED

#include <real_time_support/messaging.h>

// This loosely follows Objective-C semantics.  Given a
// dynamically or statically allocated message_port structure,
// initialize it to be a specific port type.  It can be closed
// or deallocated with the generic message_port functions.

// The name is a six-character RTAI resource identifier naming a
// mailbox.  The mailbox is effectively a queue which can be
// either a destination for a message or a source of messages.
// This means that for any given client the port is
// unidirectional, unless it is sending messages to itself.
// Generally, a client will open a pair of mailboxes, one for
// input, one for output.  The input port can then receive
// messages from any process in the system which also opens it.

extern message_port *init_RTAI_mailbox_message_port( message_port *port, char *name );

#endif // RTAI_MAILBOX_MESSAGING_H_INCLUDED
