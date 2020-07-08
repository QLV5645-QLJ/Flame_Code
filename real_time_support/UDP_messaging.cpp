// $Id: UDP_messaging.c,v 1.1 2005/12/07 15:24:46 garthz Exp $
// UDP_messaging.c : simple UDP transport for message packets
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.

#include <errno.h>	/* error processing */
#include <netdb.h>      /* for gethostbyname() */
#include <sys/ioctl.h>	/* for non-blocking ioctl call */
#include <netinet/in.h>	/* for internet */
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <string.h>     /* for bcopy() */
#include <fcntl.h>
#include <stdlib.h>     /* for free() */
#include <stdio.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <net/ethernet.h>
#include <arpa/inet.h>

#include <utility/utility.h>
#include <real_time_support/messaging.h>
#include <real_time_support/messages.h>
#include <real_time_support/UDP_messaging.h>

/****************************************************************/
// Opaque definition for the private data for a UDP port.
struct udp_port_t {
  int sfd;                 // socket file descriptor, same used for input and output

  // These addresses are specified when the message_port is initialized.
  struct sockaddr_in local;    // the locally bound socket, valid for input or output
  struct sockaddr_in remote;   // the address of a remote host, valid for output port

  // Flags if the port acts as the given type.  The port can be both input and output.
  unsigned int input_port    :1;
  unsigned int output_port   :1;

  // Flag to filter input packets based on source address.
  unsigned int exclusive_input :1;

};

//================================================================
// Non-blocking message packet send using a UDP socket.  All addresses
// were specified when the port was created.  Over-long messages are
// silently truncated.

static void
UDP_message_port_send_message( struct message_port_t *port, union message_t *msg )
{
  if ( msg != NULL && port != NULL && port->initialized && !port->closed ) {

    struct udp_port_t *priv = (struct udp_port_t *) port->userdata;

    if ( priv != NULL && priv->output_port ) {

      int err = sendto( priv->sfd, (void *) msg, msg->header.length, 0 /* flags */, 
			(struct sockaddr *) &priv->remote, sizeof( priv->remote ) );

      if ( err == -1 ) errprintf("UDP_message_port_send_message unable to send: %s", strerror(errno));
    }
  }
}

//================================================================
// Non-blocking message receive on a UDP socket.  Returns true if a
// message was received.

static int 
UDP_message_port_recv_message( message_port *port, union message_t *msgarea )
{
  if ( msgarea != NULL && port != NULL && port->initialized && !port->closed ) {
    struct udp_port_t *priv = (struct udp_port_t *) port->userdata;

    if ( priv != NULL && priv->input_port ) {
      fd_set recv_fd_set;        // file descriptor bit flags for select()
      struct timeval timeout;   

      // Create a fd_set and timeout for performing select() on the input.
      FD_ZERO( &recv_fd_set );
      FD_SET( priv->sfd, &recv_fd_set );
      timeout.tv_usec = timeout.tv_sec  = 0;

      if (select( priv->sfd+1, &recv_fd_set, NULL, NULL, &timeout ) == -1) {
	errprintf("in UDP_message_port_recv_message select failed: %s", strerror(errno));
	return 0;
      }

      // if no packet was ready the select will simply timeout and return with no flags set
      if ( !FD_ISSET( priv->sfd, &recv_fd_set ) ) return 0;

      // else some data is ready
      {
	struct sockaddr_in client;    // address for the received packet

	// client_length initially holds the length of the address buffer, then is filled in with actual address length
	socklen_t client_length = sizeof( client );

	int msglength = recvfrom( priv->sfd, msgarea, sizeof( *msgarea ), 0 /* flags */, 
				  (struct sockaddr *) &client,  // return buffer for source address
				  &client_length);                   

	if ( msglength == -1) {
	  errprintf("UDP_message_port_recv_message recvfrom failed: %s", strerror(errno));
	  return 0;
	}
	
	if ( msglength < sizeof( msgarea->header ) ) {
	  errprintf( "UDP_message_port_recv_message: received short message of %d bytes, less than minimum %d bytes.",
		     msglength, sizeof( msgarea->header ) );
	  return 0;
	}

	if ( msglength != msgarea->header.length ) {
	  errprintf( "UDP_message_port_recv_message: the received message length %d does not match header length field %d", 
		     msglength, msgarea->header.length );
	  return 0;
	}

	if ( priv->exclusive_input ) {
	  // Enforce a policy here to ignore packets not from the
	  // designed remote host.  Otherwise the UDP input is
	  // treated as an endpoint for anyone.
	  if ( client_length != sizeof( client ) ) {
	    errprintf( "source address length %d does not match expected %d", client_length, sizeof(client));
	    return 0;
	  }

	  if ( memcmp( &priv->remote, &client, sizeof( client ) ) ) {
	    errprintf( "source address does not match remote host, filtering spurious packet");
	    return 0;
	  }
	}

	return 1; // received valid message

      }
    }
  }
  return 0;
}

/****************************************************************/
static void UDP_message_port_close ( message_port *port )
{
  if ( port != NULL && port->initialized && !port->closed ) {
    struct udp_port_t *priv = (struct udp_port_t *) port->userdata;

    if ( priv != NULL ) {

      // close the socket file descriptor, which frees the system resources
      if ( priv->sfd != -1 ) close( priv->sfd );

      // free the private data memory
      free( priv );
      port->userdata = NULL;
    }
    port->closed = 1;
  }
}

/****************************************************************/
// Public entry point.  Initialize a generic message_port to be a UDP port.

message_port *init_UDP_message_port( message_port *port, 
				     char *localname, int local_port, 
				     char *remotename, int remote_port, 
				     int flags )
{
  struct hostent *host;
  struct sockaddr_in server;     // server address information
  struct udp_port_t *priv;

  if (port == NULL) return port;

  // if the port already is something, reset it
  if ( port->initialized && !port->closed && port->close != NULL ) (*port->close)(port);

  // reset to a generic state
  message_port_init( port );

  if ( remotename == NULL ) return port;

  // create a private data area
  priv = (struct udp_port_t *) calloc(1, sizeof( struct udp_port_t ));
  if (priv == NULL) return port;
  port->userdata = (void *) priv;
  port->close = UDP_message_port_close;
  priv->sfd = -1;

  priv->input_port   	= (flags & UDP_MESSAGE_PORT_INPUT ) ? 1 : 0;
  priv->output_port  	= (flags & UDP_MESSAGE_PORT_OUTPUT ) ? 1 : 0;
  priv->exclusive_input = (flags & UDP_MESSAGE_EXCLUSIVE_INPUT ) ? 1 : 0;

  // The port needs at least one of input or output true.
  if ( ! ( priv->input_port || priv->output_port ) ) return port;

  // Each port creates and binds one socket for input and/or
  // output.  Start off by creating the socket.

  priv->sfd = socket( PF_INET, SOCK_DGRAM, 0);
  if ( priv->sfd == -1 ) {
    errprintf("unable to create UDP socket: %s", strerror(errno));
    goto fail;
  }

  // Look up the local server and get the first address from the list.
  if (localname == NULL) localname = "localhost";
  host = gethostbyname( localname );

  if ( host == NULL ) {
    errprintf("host %s not found, gethostbyname returned no result\n", localname );
    goto fail;
  }
  if ( host->h_length == sizeof( priv->local.sin_addr ) ) {
    memcpy( (void *)&priv->local.sin_addr, (void *)host->h_addr_list[0], host->h_length );
    logprintf( "found %s listed as %s\n", localname, inet_ntoa( priv->local.sin_addr ));

  } else {
    errprintf( "gethostbyname on %s returned %d byte address, but %d byte address is required.\n", 
	       localname, host->h_length, sizeof( priv->local.sin_addr) );
    goto fail;
  }

  // Bind the socket on the local host.
  priv->local.sin_family = AF_INET;              // internet protocol family
  priv->local.sin_port   = htons( local_port );

  if ( bind ( priv->sfd, (struct sockaddr *) &priv->local, sizeof( priv->local )) == -1 ) {
    errprintf("unable to bind socket to local host: %s", strerror(errno));
    goto fail;
  }

  // If an output port, look up the remote server and cache the
  // first address from the list.

  if ( priv->output_port ) {
    host = gethostbyname( remotename );

    if ( host == NULL ) {
      errprintf("host %s not found, gethostbyname returned no result\n", remotename );
      goto fail;
    }

    if ( host->h_length == sizeof( priv->remote.sin_addr ) ) {
      memcpy((void *)&priv->remote.sin_addr, (void *)host->h_addr_list[0], host->h_length);
      logprintf( "found %s listed as %s\n", remotename, inet_ntoa( priv->remote.sin_addr ));
    } else {
      errprintf( "gethostbyname on %s returned %d byte address, but %d byte address is required.\n", 
		 remotename, host->h_length, sizeof( priv->remote.sin_addr) );
      goto fail;
    }

    // set address protocol family and port number
    priv->remote.sin_family = AF_INET;
    priv->remote.sin_port = htons( remote_port );
  }

  // Set up the function callbacks.
  port->send = UDP_message_port_send_message;
  port->recv = UDP_message_port_recv_message;
  port->initialized = 1;

  return port;

 fail:
  UDP_message_port_close( port );
  return port;

}

/****************************************************************/
