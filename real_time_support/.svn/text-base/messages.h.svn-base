// messages.h : real time process message formats
//
// Copyright (c) 2001-2006 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#ifndef MESSAGES_H_INCLUDED
#define MESSAGES_H_INCLUDED

// This file is included by both the real-time and the
// non-real-time code to define the format of the messages passed
// between processes.  These messages generally are sent across
// the real time message ports, but the format might also be used
// to store trace or log files.  All of the message types are
// defined at compile time; this keeps the code clean, although
// it means that this global definition needs to be modified as
// the system is extended.

// The goal here is simplicity; by having just one type of
// message structure the code can be straightforward.  It is up
// to the user to use the correct message types in the correct
// context, since some of these only have meaning for one of the
// nodes in the system.  For this reason all message processing
// functions should safely ignore unhandled types.

// Having said that, there is an exception, since a user is free
// to define additional structures in the same format and use
// them as message types, only care must be taken so no subtype
// codes overlap.

// Note that all values are sent in i386 native byte order!  This
// will be a problem if this code is ever used between systems of
// different endian-ness, e.g., networking a PowerPC with an
// Intel host.

//================================================================
// Define the names of RTAI mailboxes for communication between
// processes on the real time target system.

// RTAI identifier names must be six characters long, which are
// converted by nam2num to a unsigned long.

#define REALTIME_PROCESS_NAME        "FLMRTS"
#define DISPLAY_PROCESS_NAME         "FLMDSP"
#define REALTIME_OUTPUT_MAILBOX_NAME "FLMRTO"
#define REALTIME_INPUT_MAILBOX_NAME  "FLMRTI"

// Define some default UDP port numbers to use for communication.
#define REALTIME_UDP_PORT   4000
#define DISPLAY_UDP_PORT    5000
#define REALTIME_HOST_NAME  "robot"
#define DISPLAY_HOST_NAME   "mothership"

//================================================================
// Header for message packets.  All messages must include this as the
// first element.  Each message type is of a bounded size, most are of
// fixed size.  The length field is included for generic handling.  A
// checksum is included in case there are problems with buffer overrun
// or the underlying protocol has dropouts.

struct _msg_hdr {
  unsigned short type;            
  unsigned short length;   // length of the entire packet, including header
  unsigned char subtype;
  unsigned char checksum;  // value to make body and header sum to zero
  // In most messages, this will be followed by additional data.
};

//================================================================
// For now, just one major message type.  Most of this code was copied 
// from the biped project at Carnegie Mellon, however, this major number is 
// different, in case there is some future sharing of data files.

#define FLAME_MESSAGE (0x7293)

// The following are the message subtypes.
enum msg_subtypes {
  MSG_NOMESSAGE = 0,   // no-op
  MSG_PING,            // a status query from the UI to the real time system
  MSG_PONG,            // a status response from the RT to the UI 
  MSG_STOP,            // controller stop, no data
  MSG_RUN,             // controller run, no data
  MSG_RESET,           // controller reset, no data
  MSG_SHUTDOWN,        // controller shutdown, no data
  MSG_RECALIBRATE,     // controller encoder recalibration, no data
  MSG_STATUS,          // basic heartbeat messages from RT
  MSG_PRINT,           // message from the RT to the UI console 

  MSG_LASTMESSAGENUM   // indicator; keep this last 
};

// The maximum packet length, including headers, is fixed at the
// following value.  This is longer than some message ports might
// support, however, so those ports will reject full length
// messages (e.g., UDP ports).
#define MSG_MTU 4096  

// The maximum non-header data length, defined for convenience.
#define MSG_MAXDATA (MSG_MTU - sizeof(struct _msg_hdr))

//================================================================

// A large union to contain all message types.
  
union message_t {
  // A structure for access to the header fields.
  struct _msg_hdr header;

  // An array for access to raw packet data.
  unsigned char raw_data[MSG_MTU];

  /****************************************************************/
  // All normal messages follow.  Each must begin with a struct _msg_hdr.

  // A print request.
  struct msg_print {
    struct _msg_hdr hdr;

    // This is followed by a block of data to be printed, not
    // guaranteed to be zero-terminated.
    char data;  // placeholder

  } print;

  // A status query response.
  struct msg_pong {
    struct _msg_hdr hdr;
    unsigned version;        // protocol version number, for safety
  } pong;

  // Status heartbeat.
  struct msg_status {
    struct _msg_hdr hdr;
    unsigned int sensor_processing_duration;
    unsigned int total_cycle_duration;
  } status;

};

/****************************************************************/

#endif // MESSAGES_H_INCLUDED
