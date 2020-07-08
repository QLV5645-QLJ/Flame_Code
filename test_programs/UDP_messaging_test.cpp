// $Id: UDP_messaging_test.c,v 1.1 2005/12/07 15:27:25 garthz Exp $
// UDP_messaging_test.c : test program for UDP mailboxes
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#include <stdio.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <errno.h>

#include <real_time_support/UDP_messaging.h>
#include <real_time_support/protocol_version.h>
#include <utility/utility.h>

/****************************************************************/
int main(int argc, char **argv) 
{
  int keep_running = 1;
  message_port *port;

  printf("UDP_messaging_test starting up.\n");
  printf("This will periodically send a ping message to the remote host, which should respond.\n");

  if (argc < 5) {
    printf("Usage: %s local_name local_port remote_name remote_port\n", argv[0]);
    exit(0);
  }
  
  port = init_UDP_message_port( message_port_alloc(), 
				argv[1], atoi(argv[2]), 
				argv[3], atoi(argv[4]), 
				UDP_MESSAGE_PORT_INPUT | UDP_MESSAGE_PORT_OUTPUT | UDP_MESSAGE_EXCLUSIVE_INPUT
				);
  
  if (!MESSAGE_PORT_READY( port ) ) {
    printf("message port initialization failed.\n");
    exit(1);
  }

  // begin message receiving loop
  while ( keep_running ) {
    union message_t msg;
    int ready;

    ready = message_receive( port, &msg );

    if ( !ready ) {
      logprintf("no message, sleeping.\n"); 
      delay_microseconds( 500000 );
      send_signal( port, MSG_PING );
      continue; 

    } else {
      logprintf("Received message: type 0x%04x subtype %d length %d.\n", 
		msg.header.type, msg.header.subtype, msg.header.length );

      // drop any packets without the correct type
      if (msg.header.type != FLAME_MESSAGE) {
	errprintf( "  Error: major message type not correct.\n");

      } else {
	// select based on subtype
	switch (msg.header.subtype) {

	case MSG_PONG:
	  logprintf("  Received PONG with version code %d (%s).\n", msg.pong.version, 
		    (msg.pong.version == MESSAGE_PROTOCOL_VERSION) ? "correct" : "wrong!");
	  break;

	case MSG_PING:
	  logprintf("  Received PING, sending PONG reply.\n");
	  send_pong( port );
	  break;

	case MSG_SHUTDOWN:
	  logprintf("  Received SHUTDOWN from real time thread.\n");
	  keep_running = 0;
	  break;

	case MSG_RESET:
	  logprintf("  Received RESET from real time thread, which doesn't make sense.\n");
	  break;

	case MSG_PRINT:
	  printf("  Received PRINT message:\n    ");
	  fwrite( &msg.print.data, 1, msg.header.length - sizeof(msg.header), stdout);
	  printf("\n");
	  break;

	default:
	  logprintf("  Unhandled message type.\n");
	  break;
	}
      }
    }
  }

  message_port_dealloc( port );

  logprintf("UDP_messaging_test done.\n");
  exit(0);
}

