/*
 * "$Id: pqissl.cc,v 1.28 2007-03-17 19:32:59 rmf24 Exp $"
 *
 * 3P/PQI network interface for RetroShare.
 *
 * Copyright 2004-2006 by Robert Fernie.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License Version 2 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA.
 *
 * Please report all bugs and problems to "retroshare@lunamutt.com".
 *
 */





#include "pqi/pqissl.h"
#include "pqi/pqinetwork.h"
#include "pqi/sslfns.h"

#include "util/rsnet.h"
#include "util/rsdebug.h"

#include <unistd.h>
#include <errno.h>
#include <openssl/err.h>

#include <sstream>

#include "pqi/pqissllistener.h"

const int pqisslzone = 37714;

/*********
#define WAITING_NOT            0
#define WAITING_LOCAL_ADDR     1  
#define WAITING_REMOTE_ADDR    2  
#define WAITING_SOCK_CONNECT   3
#define WAITING_SSL_CONNECTION 4
#define WAITING_SSL_AUTHORISE  5
#define WAITING_FAIL_INTERFACE 6

#define PQISSL_PASSIVE  0x00
#define PQISSL_ACTIVE   0x01

#define PQISSL_DEBUG 1

const int PQISSL_LOCAL_FLAG = 0x01;
const int PQISSL_REMOTE_FLAG = 0x02;
const int PQISSL_UDP_FLAG = 0x02;
***********/

static const int PQISSL_MAX_READ_ZERO_COUNT = 20;
static const time_t PQISSL_MAX_READ_ZERO_TIME = 15; // 15 seconds of no data => reset. (atm HeartBeat pkt sent 5 secs)

static const int PQISSL_SSL_CONNECT_TIMEOUT = 30;

/********** PQI SSL STUFF ******************************************
 *
 * A little note on the notifyEvent(FAILED)....
 *
 * this is called from 
 * (1) reset if needed!
 * (2) Determine_Remote_Address (when all options have failed).
 *
 * reset() is only called when a TCP/SSL connection has been
 * established, and there is an error. If there is a failed TCP 
 * connection, then an alternative address can be attempted.
 *
 * reset() is called from
 * (1) destruction.
 * (2) disconnect()
 * (3) bad waiting state.
 *
 * // TCP/or SSL connection already established....
 * (5) pqissl::SSL_Connection_Complete() <- okay -> cos we made a TCP connection already.
 * (6) pqissl::accept() <- okay cos something went wrong.
 * (7) moretoread()/cansend() <- okay cos 
 *
 */

pqissl::pqissl(pqissllistener *l, PQInterface *parent, p3ConnectMgr *cm)
	:NetBinInterface(parent, parent->PeerId()), 
	waiting(WAITING_NOT), active(false), certvalid(false), 
	sslmode(PQISSL_ACTIVE), ssl_connection(NULL), sockfd(-1), 
	pqil(l),  // no init for remote_addr.
	readpkt(NULL), pktlen(0), 
	attempt_ts(0),
	net_attempt(0), net_failure(0), net_unreachable(0), 
	sameLAN(false), n_read_zero(0), mReadZeroTS(0), 
	mConnectDelay(0), mConnectTS(0),
        mConnectTimeout(0), mTimeoutTS(0), mConnMgr(cm)

{
	/* set address to zero */
        sockaddr_clear(&remote_addr);

  	{
	  std::ostringstream out;
	  out << "pqissl for PeerId: " << PeerId();
	  rslog(RSL_ALERT, pqisslzone, out.str());
	}

#if 0
	if (!(AuthSSL::getAuthSSL()->isAuthenticated(PeerId())))
	{
  	  rslog(RSL_ALERT, pqisslzone, 
	    "pqissl::Warning Certificate Not Approved!");

  	  rslog(RSL_ALERT, pqisslzone, 
	    "\t pqissl will not initialise....");

	}
#else
  	  rslog(RSL_ALERT, pqisslzone, 
	    "pqissl::Warning SSL Certificate Approval Not CHECKED??");
#endif

	return;
}

	pqissl::~pqissl()
{ 
  	rslog(RSL_ALERT, pqisslzone, 
	    "pqissl::~pqissl -> destroying pqissl");
	stoplistening(); /* remove from pqissllistener only */
	reset(); 
	return;
}


/********** Implementation of NetInterface *************************/

int	pqissl::connect(struct sockaddr_in raddr)
{
	// reset failures
	net_failure = 0;
	remote_addr = raddr;
	remote_addr.sin_family = AF_INET;

	return ConnectAttempt();
}

// tells pqilistener to listen for us.
int	pqissl::listen()
{
	if (pqil)
	{
		return pqil -> addlistenaddr(PeerId(), this);
	}
	return 0;
}

int 	pqissl::stoplistening()
{
	if (pqil)
	{
		pqil -> removeListenPort(PeerId());
	}
	return 1;
}

int 	pqissl::disconnect()
{
	return reset();
}

int pqissl::getConnectAddress(struct sockaddr_in &raddr) {
    raddr = remote_addr;
    return (remote_addr.sin_addr.s_addr == 0);
}

/* BinInterface version of reset() for pqistreamer */
int 	pqissl::close() 
{
	return reset();
}

// put back on the listening queue.
int 	pqissl::reset()
{
	std::ostringstream out;
	std::ostringstream outAlert;

	/* a reset shouldn't cause us to stop listening 
	 * only reasons for stoplistening() are;
	 *
	 * (1) destruction.
	 * (2) connection.
	 * (3) WillListen state change
	 *
	 */

	outAlert << "pqissl::reset():" << PeerId();
	rslog(RSL_ALERT, pqisslzone, outAlert.str());


	out << "pqissl::reset() State Before Reset:" << std::endl;
	out << "\tActive: " << (int) active << std::endl;
	out << "\tsockfd: " << sockfd << std::endl;
	out << "\twaiting: " << waiting << std::endl;
	out << "\tssl_con: " << ssl_connection << std::endl;
	out << std::endl;

	bool neededReset = false;

	if (ssl_connection != NULL)
	{
		out << "pqissl::reset() Shutting down SSL Connection";
		out << std::endl;
		SSL_shutdown(ssl_connection);
		SSL_free (ssl_connection);

		neededReset = true;	
	}

	if (sockfd > 0)
	{
		out << "pqissl::reset() Shutting down (active) socket";
		out << std::endl;
		net_internal_close(sockfd);
		sockfd = -1;
		neededReset = true;	
	}
	active = false;
	sockfd = -1;
	waiting = WAITING_NOT;
	ssl_connection = NULL;
	sameLAN = false;
	n_read_zero = 0;
	mReadZeroTS = 0;
	total_len = 0 ;
        mTimeoutTS = 0;

	if (neededReset)
	{
		out << "pqissl::reset() Reset Required!" << std::endl;
		out << "pqissl::reset() Will Attempt notifyEvent(FAILED)";
		out << std::endl;
	}

	out << "pqissl::reset() Complete!" << std::endl;
	rslog(RSL_DEBUG_BASIC, pqisslzone, out.str());

	// notify people of problem!
	// but only if we really shut something down.
	if (neededReset)
	{
		// clean up the streamer 
		if (parent())
		{
		  parent() -> notifyEvent(this, NET_CONNECT_FAILED);
		}
	}
	return 1;
}

bool 	pqissl::connect_parameter(uint32_t type, uint32_t value)
{
	{
                std::ostringstream out;
		out << "pqissl::connect_parameter() Peer: " << PeerId();
		out << " type: " << type << "value: " << value;
		rslog(RSL_DEBUG_ALL, pqisslzone, out.str());
	}

        if (type == NET_PARAM_CONNECT_DELAY)
	{
                std::ostringstream out;
		out << "pqissl::connect_parameter() Peer: " << PeerId();
		out << " DELAY: " << value;
		rslog(RSL_WARNING, pqisslzone, out.str());


		mConnectDelay = value;
		return true;
	}
        else if (type == NET_PARAM_CONNECT_TIMEOUT)
	{
                std::ostringstream out;
		out << "pqissl::connect_parameter() Peer: " << PeerId();
		out << " TIMEOUT: " << value;
		rslog(RSL_WARNING, pqisslzone, out.str());

		mConnectTimeout = value;
		return true;
	}
	return false;
        //return NetInterface::connect_parameter(type, value);
}


/********** End of Implementation of NetInterface ******************/
/********** Implementation of BinInterface **************************
 * Only status() + tick() are here ... as they are really related
 * to the NetInterface, and not the BinInterface,
 *
 */

/* returns ...
 * -1 if inactive.
 *  0 if connecting.
 *  1 if connected.
 */

int 	pqissl::status()
{
	int alg;

	std::ostringstream out;
	out << "pqissl::status()";

	if (active)
	{
		out << " active: " << std::endl;
		// print out connection.
		out << "Connected TO : " << PeerId();
		out << std::endl;
		// print out cipher.
		out << "\t\tSSL Cipher:" << SSL_get_cipher(ssl_connection);
		out << " (" << SSL_get_cipher_bits(ssl_connection, &alg);
		out << ":" << alg << ") ";
		out << "Vers:" << SSL_get_cipher_version(ssl_connection);
		out << std::endl;
		out << std::endl;

	}
	else
	{
		out << " Waiting for connection!" << std::endl;
	}

	rslog(RSL_DEBUG_BASIC, pqisslzone, out.str());

	if (active)
	{
		return 1;
	}
	else if (waiting > 0)
	{
		return 0;
	}
	return -1;
}

	// tick......
int	pqissl::tick()
{
	//pqistreamer::tick();

	// continue existing connection attempt.
	if (!active)
	{
		// if we are waiting.. continue the connection (only)
		if (waiting > 0)
		{
			std::ostringstream out;
			out << "pqissl::tick() ";
			out << "Continuing Connection Attempt!";
			rslog(RSL_DEBUG_BASIC, pqisslzone, out.str());

                        ConnectAttempt();
			return 1;
		}
	}
	return 1;
}

/********** End of Implementation of BinInterface ******************/
/********** Internals of SSL Connection ****************************/


int 	pqissl::ConnectAttempt()
{
	switch(waiting)
	{
		case WAITING_NOT:

			sslmode = PQISSL_ACTIVE; /* we're starting this one */

  	  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			  "pqissl::ConnectAttempt() STATE = Not Waiting, starting connection");

		case WAITING_DELAY:

  	  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			  "pqissl::ConnectAttempt() STATE = Waiting Delay, starting connection");
	
			return Delay_Connection();
			//return Initiate_Connection(); /* now called by Delay_Connection() */

			break;

		case WAITING_SOCK_CONNECT:

  	  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			  "pqissl::ConnectAttempt() STATE = Waiting Sock Connect");

			return Initiate_SSL_Connection();
			break;

		case WAITING_SSL_CONNECTION:

  	  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			  "pqissl::ConnectAttempt() STATE = Waiting SSL Connection");

			return Authorise_SSL_Connection();
			break;

		case WAITING_SSL_AUTHORISE:

  	  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			  "pqissl::ConnectAttempt() STATE = Waiting SSL Authorise");

			return Authorise_SSL_Connection();
			break;
		case WAITING_FAIL_INTERFACE:

  	  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			  "pqissl::ConnectAttempt() Failed - Retrying");

			return Failed_Connection();
			break;


		default:
  	  		rslog(RSL_ALERT, pqisslzone, 
		 "pqissl::ConnectAttempt() STATE = Unknown - Reset");

			reset();
			break;
	}
  	rslog(RSL_ALERT, pqisslzone, "pqissl::ConnectAttempt() Unknown");

	return -1;
}


/****************************** FAILED ATTEMPT ******************************
 * Determine the Remote Address.
 *
 * Specifics:
 * TCP / UDP
 * TCP - check for which interface to use.
 * UDP - check for request proxies....
 *
 * X509 / XPGP - Same.
 *
 */

int 	pqissl::Failed_Connection()
{
	rslog(RSL_DEBUG_BASIC, pqisslzone, 
		"pqissl::ConnectAttempt() Failed - Notifying");

	if (parent())
	{
		parent() -> notifyEvent(this, NET_CONNECT_UNREACHABLE);
	}
	waiting = WAITING_NOT;

	return 1;
}

/****************************** MAKE CONNECTION *****************************
 * Open Socket and Initiate Connection.
 *
 * Specifics:
 * TCP / UDP
 * TCP - socket()/connect()
 * UDP - tou_socket()/tou_connect()
 *
 * X509 / XPGP - Same.
 *
 */

int 	pqissl::Delay_Connection()
{
  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Delay_Connection() Attempting Outgoing Connection....");

	if (waiting == WAITING_NOT)
	{
		waiting = WAITING_DELAY;

		/* set delay */
		if (mConnectDelay == 0)
		{
			return Initiate_Connection();
		}

		/* set Connection TS.
		 */
		{ 
			std::ostringstream out;
	  		out << "pqissl::Delay_Connection() ";
			out << " Delaying Connection to ";
			out << PeerId() << " for ";
			out << mConnectDelay;
			out << " seconds";
  			rslog(RSL_DEBUG_BASIC, pqisslzone, out.str());
		}


		mConnectTS = time(NULL) + mConnectDelay;
		return 0;
	}
	else if (waiting == WAITING_DELAY)
	{
		{ 
			std::ostringstream out;
	  		out << "pqissl::Delay_Connection() ";
			out << " Connection to ";
			out << PeerId() << " starting in ";
			out << mConnectTS - time(NULL);
			out << " seconds";
  			rslog(RSL_DEBUG_BASIC, pqisslzone, out.str());
		}

		if (time(NULL) > mConnectTS)
		{
			return Initiate_Connection();
		}
		return 0;
	}

  	rslog(RSL_WARNING, pqisslzone, 
		 "pqissl::Initiate_Connection() Already Attempt in Progress!");
	return -1;
}


int 	pqissl::Initiate_Connection()
{
	int err;
	struct sockaddr_in addr = remote_addr;

  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Initiate_Connection() Attempting Outgoing Connection....");

	if (waiting != WAITING_DELAY)
	{
  		rslog(RSL_WARNING, pqisslzone, 
		 "pqissl::Initiate_Connection() Already Attempt in Progress!");
		return -1;
	}

  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Initiate_Connection() Opening Socket");

	// open socket connection to addr.
	int osock = unix_socket(PF_INET, SOCK_STREAM, 0);

	{
		std::ostringstream out;
		out << "pqissl::Initiate_Connection() osock = " << osock;
  		rslog(RSL_DEBUG_BASIC, pqisslzone, out.str());
	}

	if (osock < 0)
	{
		std::ostringstream out;
		out << "pqissl::Initiate_Connection()";
		out << "Failed to open socket!" << std::endl;
		out << "Socket Error:" << socket_errorType(errno) << std::endl;
  		rslog(RSL_WARNING, pqisslzone, out.str());

		net_internal_close(osock);
		waiting = WAITING_FAIL_INTERFACE;
		return -1;
	}

  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Initiate_Connection() Making Non-Blocking");

        err = unix_fcntl_nonblock(osock);
	if (err < 0)
	{
		std::ostringstream out;
		out << "pqissl::Initiate_Connection()";
		out << "Error: Cannot make socket NON-Blocking: ";
		out << err << std::endl;
  		rslog(RSL_WARNING, pqisslzone, out.str());

		waiting = WAITING_FAIL_INTERFACE;
		net_internal_close(osock);
		return -1;
	}

	{ 
		std::ostringstream out;
		out << "pqissl::Initiate_Connection() ";
		out << "Connecting To: ";
		out << PeerId() << " via: ";
		out << rs_inet_ntoa(addr.sin_addr);
		out << ":" << ntohs(addr.sin_port);
  		rslog(RSL_WARNING, pqisslzone, out.str());
	}

	if (addr.sin_addr.s_addr == 0)
	{
		std::ostringstream out;
		out << "pqissl::Initiate_Connection() ";
		out << "Invalid (0.0.0.0) Remote Address,";
		out << " Aborting Connect.";
		out << std::endl;
  		rslog(RSL_WARNING, pqisslzone, out.str());
		waiting = WAITING_FAIL_INTERFACE;
		net_internal_close(osock);
		return -1;
	}

#ifdef WINDOWS_SYS
	/* Set TCP buffer size for Windows systems */

	int sockbufsize = 0;
	int size = sizeof(int);

	err = getsockopt(osock, SOL_SOCKET, SO_RCVBUF, (char *)&sockbufsize, &size);
#ifdef PQISSL_DEBUG
	if (err == 0) {
		std::cerr << "pqissl::Initiate_Connection: Current TCP receive buffer size " << sockbufsize << std::endl;
	} else {
		std::cerr << "pqissl::Initiate_Connection: Error getting TCP receive buffer size. Error " << err << std::endl;
	}
#endif

	sockbufsize = 0;

	err = getsockopt(osock, SOL_SOCKET, SO_SNDBUF, (char *)&sockbufsize, &size);
#ifdef PQISSL_DEBUG
	if (err == 0) {
		std::cerr << "pqissl::Initiate_Connection: Current TCP send buffer size " << sockbufsize << std::endl;
	} else {
		std::cerr << "pqissl::Initiate_Connection: Error getting TCP send buffer size. Error " << err << std::endl;
	}
#endif

	sockbufsize = WINDOWS_TCP_BUFFER_SIZE;

	err = setsockopt(osock, SOL_SOCKET, SO_RCVBUF, (char *)&sockbufsize, sizeof(sockbufsize));
#ifdef PQISSL_DEBUG
	if (err == 0) {
		std::cerr << "pqissl::Initiate_Connection: TCP receive buffer size set to " << sockbufsize << std::endl;
	} else {
		std::cerr << "pqissl::Initiate_Connection: Error setting TCP receive buffer size. Error " << err << std::endl;
	}
#endif

	err = setsockopt(osock, SOL_SOCKET, SO_SNDBUF, (char *)&sockbufsize, sizeof(sockbufsize));
#ifdef PQISSL_DEBUG
	if (err == 0) {
		std::cerr << "pqissl::Initiate_Connection: TCP send buffer size set to " << sockbufsize << std::endl;
	} else {
		std::cerr << "pqissl::Initiate_Connection: Error setting TCP send buffer size. Error " << err << std::endl;
	}
#endif
#endif // WINDOWS_SYS

	mTimeoutTS = time(NULL) + mConnectTimeout;
	//std::cerr << "Setting Connect Timeout " << mConnectTimeout << " Seconds into Future " << std::endl;

	if (0 != (err = unix_connect(osock, (struct sockaddr *) &addr, sizeof(addr))))
	{
		std::ostringstream out;
		out << "pqissl::Initiate_Connection() connect returns:";
		out << err << " -> errno: " << errno << " error: ";
		out << socket_errorType(errno) << std::endl;
		
		if (errno == EINPROGRESS)
		{
			// set state to waiting.....
			waiting = WAITING_SOCK_CONNECT;
			sockfd = osock;

			out << " EINPROGRESS Waiting for Socket Connection";
  		        rslog(RSL_DEBUG_BASIC, pqisslzone, out.str());
  
			return 0;
		}
		else if ((errno == ENETUNREACH) || (errno == ETIMEDOUT))
		{
			out << "ENETUNREACHABLE: cert: " << PeerId();
  		        rslog(RSL_WARNING, pqisslzone, out.str());

			// Then send unreachable message.
			net_internal_close(osock);
			osock=-1;
			//reset();

			waiting = WAITING_FAIL_INTERFACE;
			// removing unreachables...
			//net_unreachable |= net_attempt;

			return -1;
		}

		/* IF we get here ---- we Failed for some other reason. 
                 * Should abandon this interface 
		 * Known reasons to get here: EINVAL (bad address)
		 */

		out << "Error: Connection Failed: " << errno;
		out << " - " << socket_errorType(errno) << std::endl;

		net_internal_close(osock);
		osock=-1;
		waiting = WAITING_FAIL_INTERFACE;

  		rslog(RSL_WARNING, pqisslzone, out.str());
		// extra output for the moment.
		//std::cerr << out.str();

		return -1;
	}
	else
	{
  		rslog(RSL_DEBUG_BASIC, pqisslzone,
		 "pqissl::Init_Connection() connect returned 0");
	}

	waiting = WAITING_SOCK_CONNECT;
	sockfd = osock;

	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Initiate_Connection() Waiting for Socket Connect");

	return 1;
}


/******************************  CHECK SOCKET   *****************************
 * Check the Socket.
 *
 * select() and getsockopt().
 *
 * Specifics:
 * TCP / UDP
 * TCP - select()/getsockopt()
 * UDP - tou_error()
 *
 * X509 / XPGP - Same.
 *
 */

int 	pqissl::Basic_Connection_Complete()
{
	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Basic_Connection_Complete()...");

	/* new TimeOut code. */
	if (time(NULL) > mTimeoutTS)
	{
		std::ostringstream out;
		out << "pqissl::Basic_Connection_Complete() Connection Timed Out. ";
		out << "Peer: " << PeerId() << " Period: ";
	  	out << mConnectTimeout;

		rslog(RSL_WARNING, pqisslzone, out.str());
		/* as sockfd is valid, this should close it all up */
		
		reset();
		return -1;
	}


	if (waiting != WAITING_SOCK_CONNECT)
	{
		rslog(RSL_ALERT, pqisslzone, 
	  		"pqissl::Basic_Connection_Complete() Wrong Mode");
		return -1;
	}

        if (sockfd == -1)
        {
                rslog(RSL_ALERT, pqisslzone,
                        "pqissl::Basic_Connection_Complete() problem with the socket descriptor. Aborting");
                reset();
                return -1;
        }

        // use select on the opened socket.
	// Interestingly - This code might be portable....
	
	fd_set ReadFDs, WriteFDs, ExceptFDs;
	FD_ZERO(&ReadFDs);
	FD_ZERO(&WriteFDs);
	FD_ZERO(&ExceptFDs);

	FD_SET(sockfd, &ReadFDs);
	FD_SET(sockfd, &WriteFDs);
	FD_SET(sockfd, &ExceptFDs);

	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Basic_Connection_Complete() Selecting ....");

	int sr = 0;
	if (0 > (sr = select(sockfd + 1, 
			&ReadFDs, &WriteFDs, &ExceptFDs, &timeout))) 
	{
		// select error.
  		rslog(RSL_WARNING, pqisslzone, 
	  	  "pqissl::Basic_Connection_Complete() Select ERROR(1)");
		
		net_internal_close(sockfd);
		sockfd=-1;
		//reset();
		waiting = WAITING_FAIL_INTERFACE;
		return -1;
	}

	{
		std::ostringstream out;
		out << "pqissl::Basic_Connection_Complete() Select ";
		out << " returned " << sr;
  		rslog(RSL_DEBUG_BASIC, pqisslzone, out.str());
	}
		

	if (FD_ISSET(sockfd, &ExceptFDs))
	{
		// error - reset socket.
		// this is a definite bad socket!.
		
  		rslog(RSL_WARNING, pqisslzone, 
	  	  "pqissl::Basic_Connection_Complete() Select ERROR(2)");
		
		net_internal_close(sockfd);
		sockfd=-1;
		//reset();
		waiting = WAITING_FAIL_INTERFACE;
		return -1;
	}

	if (FD_ISSET(sockfd, &WriteFDs))
	{
  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  	  "pqissl::Basic_Connection_Complete() Can Write!");
	}
	else
	{
		// happens frequently so switched to debug msg.
  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  	  "pqissl::Basic_Connection_Complete() Not Yet Ready!");
		return 0;
	}

	if (FD_ISSET(sockfd, &ReadFDs))
	{
  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  	  "pqissl::Basic_Connection_Complete() Can Read!");
	}
	else
	{
		// not ready return -1;
  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  	  "pqissl::Basic_Connection_Complete() Cannot Read!");
	}

	int err = 1;
	if (0==unix_getsockopt_error(sockfd, &err))
	{
		if (err == 0)
		{

			{
			std::ostringstream out;
	  	  	out << "pqissl::Basic_Connection_Complete()";
			out << " TCP Connection Complete: cert: ";
			out << PeerId();
			out << " on osock: " << sockfd;
  		        rslog(RSL_WARNING, pqisslzone, out.str());
			}
			return 1;
		}
		else if (err == EINPROGRESS)
		{

			std::ostringstream out;
	  	  	out << "pqissl::Basic_Connection_Complete()";
			out << " EINPROGRESS: cert: " << PeerId();
  		        rslog(RSL_WARNING, pqisslzone, out.str());

			return 0;
		}
		else if ((err == ENETUNREACH) || (err == ETIMEDOUT))
		{
			std::ostringstream out;
	  	  	out << "pqissl::Basic_Connection_Complete()";
			out << " ENETUNREACH/ETIMEDOUT: cert: ";
			out << PeerId();
  		        rslog(RSL_WARNING, pqisslzone, out.str());

			// Then send unreachable message.
			net_internal_close(sockfd);
			sockfd=-1;
			//reset();
			
			waiting = WAITING_FAIL_INTERFACE;
			// removing unreachables...
			//net_unreachable |= net_attempt;

			return -1;
		}
		else if ((err == EHOSTUNREACH) || (err == EHOSTDOWN))
		{
			std::ostringstream out;
	  	  	out << "pqissl::Basic_Connection_Complete()";
			out << " EHOSTUNREACH/EHOSTDOWN: cert: ";
			out << PeerId();
  		        rslog(RSL_WARNING, pqisslzone, out.str());

			// Then send unreachable message.
			net_internal_close(sockfd);
			sockfd=-1;
			//reset();
			waiting = WAITING_FAIL_INTERFACE;

			return -1;
		}
		else if ((err == ECONNREFUSED))
		{
			std::ostringstream out;
	  	  	out << "pqissl::Basic_Connection_Complete()";
			out << " ECONNREFUSED: cert: ";
			out << PeerId();
  		        rslog(RSL_WARNING, pqisslzone, out.str());

			// Then send unreachable message.
			net_internal_close(sockfd);
			sockfd=-1;
			//reset();
			waiting = WAITING_FAIL_INTERFACE;

			return -1;
		}
			
		std::ostringstream out;
		out << "Error: Connection Failed UNKNOWN ERROR: " << err;
		out << " - " << socket_errorType(err);
  		rslog(RSL_WARNING, pqisslzone, out.str());

		net_internal_close(sockfd);
		sockfd=-1;
		//reset(); // which will send Connect Failed,
		return -1;
	}

  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Basic_Connection_Complete() BAD GETSOCKOPT!");
	waiting = WAITING_FAIL_INTERFACE;

	return -1;
}


int 	pqissl::Initiate_SSL_Connection()
{
	int err;

  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Initiate_SSL_Connection() Checking Basic Connection");

	if (0 >= (err = Basic_Connection_Complete()))
	{
		return err;
	}

  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Initiate_SSL_Connection() Basic Connection Okay");

	// setup timeout value.
	ssl_connect_timeout = time(NULL) + PQISSL_SSL_CONNECT_TIMEOUT;

	// Perform SSL magic.
	// library already inited by sslroot().
	SSL *ssl = SSL_new(AuthSSL::getAuthSSL()->getCTX());
	if (ssl == NULL)
	{
  		rslog(RSL_ALERT, pqisslzone, 
		  "pqissl::Initiate_SSL_Connection() SSL_new failed!");

		exit(1);
		return -1;
	}
	
  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Initiate_SSL_Connection() SSL Connection Okay");

	ssl_connection = ssl;

	net_internal_SSL_set_fd(ssl, sockfd);
	if (err < 1)
	{
		std::ostringstream out;
		out << "pqissl::Initiate_SSL_Connection() SSL_set_fd failed!";
		out << std::endl;
		printSSLError(ssl, err, SSL_get_error(ssl, err), 
				ERR_get_error(), out);

  		rslog(RSL_ALERT, pqisslzone, out.str());
	}

  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Initiate_SSL_Connection() Waiting for SSL Connection");

	waiting = WAITING_SSL_CONNECTION;
	return 1;
}

int 	pqissl::SSL_Connection_Complete()
{
  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::SSL_Connection_Complete()??? ... Checking");

	if (waiting == WAITING_SSL_AUTHORISE)
	{
  		rslog(RSL_ALERT, pqisslzone, 
		  "pqissl::SSL_Connection_Complete() Waiting = W_SSL_AUTH");

		return 1;
	}
	if (waiting != WAITING_SSL_CONNECTION)
	{
  		rslog(RSL_ALERT, pqisslzone, 
		  "pqissl::SSL_Connection_Complete() Still Waiting..");

		return -1;
	}

  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::SSL_Connection_Complete() Attempting SSL_connect");

	/* if we are passive - then accept! */
	int err;

	if (sslmode)
	{
  		rslog(RSL_DEBUG_BASIC, pqisslzone, "--------> Active Connect!");
		err = SSL_connect(ssl_connection);
	}
	else
	{
  		rslog(RSL_DEBUG_BASIC, pqisslzone, "--------> Passive Accept!");
		err = SSL_accept(ssl_connection);
	}

	if (err != 1)
	{
		int serr = SSL_get_error(ssl_connection, err);
		if ((serr == SSL_ERROR_WANT_READ) 
				|| (serr == SSL_ERROR_WANT_WRITE))
		{
  			rslog(RSL_DEBUG_BASIC, pqisslzone, 
			  "Waiting for SSL handshake!");

			waiting = WAITING_SSL_CONNECTION;
			return 0;
		}


		std::ostringstream out;
		out << "pqissl::SSL_Connection_Complete()" << std::endl;
		out << "Issues with SSL Connect(" << err << ")!" << std::endl;
		printSSLError(ssl_connection, err, serr, 
				ERR_get_error(), out);

  		rslog(RSL_WARNING, pqisslzone, 
			out.str());

		// attempt real error.
		Extract_Failed_SSL_Certificate();

		reset();
		waiting = WAITING_FAIL_INTERFACE;

		return -1;
	}
	// if we get here... success v quickly.

	{
		std::ostringstream out;
		out << "pqissl::SSL_Connection_Complete() Success!: Peer: " << PeerId();
  		rslog(RSL_WARNING, pqisslzone, out.str());
	}

	waiting = WAITING_SSL_AUTHORISE;
	return 1;
}

int 	pqissl::Extract_Failed_SSL_Certificate()
{
  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Extract_Failed_SSL_Certificate()");

	// Get the Peer Certificate....
	X509 *peercert = SSL_get_peer_certificate(ssl_connection);

	if (peercert == NULL)
	{
  		rslog(RSL_WARNING, pqisslzone, 
		  "pqissl::Extract_Failed_SSL_Certificate() Peer Didnt Give Cert");
		return -1;
	}

  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Extract_Failed_SSL_Certificate() Have Peer Cert - Registering");

	// save certificate... (and ip locations)
	// false for outgoing....
	// we actually connected to remote_addr, 
	// 	which could be 
	//      (pqissl's case) sslcert->serveraddr or sslcert->localaddr.
        AuthSSL::getAuthSSL()->FailedCertificate(peercert, false);

	return 1;
}




int 	pqissl::Authorise_SSL_Connection()
{
  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Authorise_SSL_Connection()");

        if (time(NULL) > ssl_connect_timeout)
        {
		rslog(RSL_DEBUG_BASIC, pqisslzone,
			"pqissl::Authorise_SSL_Connection() Connection Timed Out!");
	        /* as sockfd is valid, this should close it all up */
	        reset();
	}

	int err;
	if (0 >= (err = SSL_Connection_Complete()))
	{
		return err;
	}

  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Authorise_SSL_Connection() SSL_Connection_Complete");

	// reset switch.
	waiting = WAITING_NOT;

	X509 *peercert = SSL_get_peer_certificate(ssl_connection);

	if (peercert == NULL)
	{
  		rslog(RSL_WARNING, pqisslzone, 
		  "pqissl::Authorise_SSL_Connection() Peer Didnt Give Cert");

		// Failed completely
		reset();
		return -1;
	}

        std::string certPeerId;
        getX509id(peercert, certPeerId);
        if (certPeerId != PeerId()) {
                rslog(RSL_WARNING, pqisslzone,
                  "pqissl::Authorise_SSL_Connection() the cert Id doesn't match the Peer id we're trying to connect to.");

                // Failed completely
                reset();
                return -1;
        }

  	rslog(RSL_DEBUG_BASIC, pqisslzone, 
	  "pqissl::Authorise_SSL_Connection() Have Peer Cert");

	// save certificate... (and ip locations)
	// false for outgoing....
	// we actually connected to remote_addr, 
	// 	which could be 
	//      (pqissl's case) sslcert->serveraddr or sslcert->localaddr.

	AuthSSL::getAuthSSL()->CheckCertificate(PeerId(), peercert);
	bool certCorrect = true; /* WE know it okay already! */

	// check it's the right one.
	if (certCorrect)
	{
		// then okay...
		std::ostringstream out;
	  	out << "pqissl::Authorise_SSL_Connection() Accepting Conn. Peer: " << PeerId();
  		rslog(RSL_WARNING, pqisslzone, out.str());

		accept(ssl_connection, sockfd, remote_addr);
		return 1;
	}

	{
		std::ostringstream out;
	  	out << "pqissl::Authorise_SSL_Connection() Something Wrong ... ";
		out << " Shutdown. Peer: " << PeerId();
  		rslog(RSL_WARNING, pqisslzone, out.str());
	}

	// else shutdown ssl connection.

	reset();
	return 0;
}

int	pqissl::accept(SSL *ssl, int fd, struct sockaddr_in foreign_addr) // initiate incoming connection.
{
	if (waiting != WAITING_NOT)
	{
  	  	rslog(RSL_WARNING, pqisslzone, 
			"pqissl::accept() - Two connections in progress - Shut 1 down!");

		// outgoing connection in progress.
		// shut this baby down.
		//
		// Thought I should shut down one in progress, and continue existing one!
		// But the existing one might be broke.... take second.
		// all we need is to never stop listening.
		
		switch(waiting)
		{

		case WAITING_SOCK_CONNECT:

  	  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			  "pqissl::accept() STATE = Waiting Sock Connect - close the socket");

			break;

		case WAITING_SSL_CONNECTION:

  	  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			  "pqissl::accept() STATE = Waiting SSL Connection - close sockfd + ssl_conn");

			break;

		case WAITING_SSL_AUTHORISE:

  	  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			  "pqissl::accept() STATE = Waiting SSL Authorise - close sockfd + ssl_conn");

			break;

		case WAITING_FAIL_INTERFACE:

  	  		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			  "pqissl::accept() STATE = Failed, ignore?");

			break;


		default:
  	  		rslog(RSL_ALERT, pqisslzone, 
		 		"pqissl::accept() STATE = Unknown - ignore?");

			reset();
			break;
		}

		//waiting = WAITING_FAIL_INTERFACE;
		//return -1;
	}

	/* shutdown existing - in all cases use the new one */
	if ((ssl_connection) && (ssl_connection != ssl))
	{
  	 	rslog(RSL_DEBUG_BASIC, pqisslzone, 
		  "pqissl::accept() closing Previous/Existing ssl_connection");
		SSL_shutdown(ssl_connection);
	}

	if ((sockfd > -1) && (sockfd != fd))
	{
  	 	rslog(RSL_DEBUG_BASIC, pqisslzone, 
		  "pqissl::accept() closing Previous/Existing sockfd");
		net_internal_close(sockfd);
	}


	// save ssl + sock.
	
	ssl_connection = ssl;
	sockfd = fd;

	/* if we connected - then just writing the same over, 
	 * but if from ssllistener then we need to save the address.
	 */
	remote_addr = foreign_addr; 

	/* check whether it is on the same LAN */

	peerConnectState details;
	mConnMgr->getOwnNetStatus(details);
	sameLAN = isSameSubnet(&(remote_addr.sin_addr), &(details.currentlocaladdr.sin_addr));

	{
	  std::ostringstream out;
	  out << "pqissl::accept() Successful connection with: " << PeerId();
	  out << std::endl;
	  out << "\t\tchecking for same LAN";
	  out << std::endl;
	  out << "\t localaddr: " << rs_inet_ntoa(details.currentlocaladdr.sin_addr);
	  out << std::endl;
	  out << "\t remoteaddr: " << rs_inet_ntoa(remote_addr.sin_addr);
	  out << std::endl;
//	  if (sameLAN)
//	  {
//	  	out << "\tSAME LAN - no bandwidth restrictions!";
//	  }
//	  else
//	  {
//	  	out << "\tDifferent LANs - bandwidth restrictions!";
//	  }
//	  out << std::endl;

  	  rslog(RSL_WARNING, pqisslzone, out.str());
	}

	// establish the ssl details.
	// cipher name.
	int alg;
	int err;

	{
	  std::ostringstream out;
	  out << "SSL Cipher:" << SSL_get_cipher(ssl) << std::endl;
	  out << "SSL Cipher Bits:" << SSL_get_cipher_bits(ssl, &alg);
	  out << " - " << alg << std::endl;
	  out << "SSL Cipher Version:" << SSL_get_cipher_version(ssl) << std::endl;
  	  rslog(RSL_DEBUG_BASIC, pqisslzone, out.str());
	}

	// make non-blocking / or check.....
        if ((err = net_internal_fcntl_nonblock(sockfd)) < 0)
	{
  	  	rslog(RSL_ALERT, pqisslzone, "Error: Cannot make socket NON-Blocking: ");

		active = false;
		waiting = WAITING_FAIL_INTERFACE;
		// failed completely.
		reset();
		return -1;
	}
	else
	{
  	  	rslog(RSL_DEBUG_BASIC, pqisslzone, "pqissl::accept() Socket Made Non-Blocking!");
	}

	// we want to continue listening - incase this socket is crap, and they try again.
	//stoplistening();

	active = true;
	waiting = WAITING_NOT;

	// Notify the pqiperson.... (Both Connect/Receive)
	if (parent())
	{
	  parent() -> notifyEvent(this, NET_CONNECT_SUCCESS);
	}
	return 1;
}

/********** Implementation of BinInterface **************************
 * All the rest of the BinInterface.
 *
 */

int 	pqissl::senddata(void *data, int len)
{
	int tmppktlen ;

#ifdef PQISSL_DEBUG
	std::cout << "Sending data thread=" << pthread_self() << ", ssl=" << (void*)this << ", size=" << len << std::endl ;
#endif
	tmppktlen = SSL_write(ssl_connection, data, len) ;

	if (len != tmppktlen)
	{
		std::ostringstream out;
		out << "pqissl::senddata()";
		out << " Full Packet Not Sent!" << std::endl; 
		out << " -> Expected len(" << len << ") actually sent(";
		out << tmppktlen << ")" << std::endl;
	
		int err = SSL_get_error(ssl_connection, tmppktlen);
		// incomplete operations - to repeat....
		// handled by the pqistreamer...
		if (err == SSL_ERROR_SYSCALL)
		{
			out << "SSL_write() SSL_ERROR_SYSCALL";
			out << std::endl;
			out << "Socket Closed Abruptly.... Resetting PQIssl";
			out << std::endl;
			std::cerr << out.str() ;
			rslog(RSL_ALERT, pqisslzone, out.str());

			/* extra debugging - based on SSL_get_error() man page */
			{
				int errsys = errno;
				int sslerr = 0;
				std::ostringstream out2;
				out2 << "SSL_ERROR_SYSCALL, ret == " << tmppktlen;
				out2 << " errno: " << errsys << std::endl;

				while(0 != (sslerr = ERR_get_error()))
				{
					out2 << "SSLERR:" << sslerr << " : ";

					char sslbuf[256] = {0};
					out2 << ERR_error_string(sslerr, sslbuf);
					out2 << std::endl;
				}
				rslog(RSL_ALERT, pqisslzone, out2.str());
			}

			reset();
			return -1;
		}
		else if (err == SSL_ERROR_WANT_WRITE)
		{
			out << "SSL_write() SSL_ERROR_WANT_WRITE";
			out << std::endl;
			rslog(RSL_ALERT, pqisslzone, out.str());
//			std::cerr << out.str() ;
			return -1;
		}
		else if (err == SSL_ERROR_WANT_READ)
		{
			out << "SSL_write() SSL_ERROR_WANT_READ";
			out << std::endl;
			rslog(RSL_ALERT, pqisslzone, out.str());
			std::cerr << out.str() ;
			return -1;
		}
		else
		{
			out << "SSL_write() UNKNOWN ERROR: " << err;
			out << std::endl;
			printSSLError(ssl_connection, tmppktlen, err, ERR_get_error(), out);
			out << std::endl;
	        	out << "\tResetting!";
			out << std::endl;
			std::cerr << out.str() ;
			rslog(RSL_ALERT, pqisslzone, out.str());

			reset();
			return -1;
		}
	}
	return tmppktlen;
}

int 	pqissl::readdata(void *data, int len)
{
#ifdef PQISSL_DEBUG
	std::cout << "Reading data thread=" << pthread_self() << ", ssl=" << (void*)this << std::endl ;
#endif

	// There is a do, because packets can be splitted into multiple ssl buffers
	// when they are larger than 16384 bytes. Such packets have to be read in 
	// multiple slices.
	do
	{
		int tmppktlen  ;

#ifdef PQISSL_DEBUG
		std::cerr << "calling SSL_read. len=" << len << ", total_len=" << total_len << std::endl ;
#endif
		tmppktlen = SSL_read(ssl_connection, (void*)( &(((uint8_t*)data)[total_len])), len-total_len) ;
#ifdef PQISSL_DEBUG
		std::cerr << "have read " << tmppktlen << " bytes" << std::endl ;
		std::cerr << "data[0] = " 
			<< (int)((uint8_t*)data)[total_len+0] << " "
			<< (int)((uint8_t*)data)[total_len+1] << " "
			<< (int)((uint8_t*)data)[total_len+2] << " "
			<< (int)((uint8_t*)data)[total_len+3] << " "
			<< (int)((uint8_t*)data)[total_len+4] << " "
			<< (int)((uint8_t*)data)[total_len+5] << " "
			<< (int)((uint8_t*)data)[total_len+6] << " "
			<< (int)((uint8_t*)data)[total_len+7] << std::endl ;
#endif

		// Need to catch errors.....
		//
		if (tmppktlen <= 0) // probably needs a reset.
		{
			std::ostringstream out;

			int error = SSL_get_error(ssl_connection, tmppktlen);
			unsigned long err2 =  ERR_get_error();

			//printSSLError(ssl_connection, tmppktlen, error, err2, out);

			if ((error == SSL_ERROR_ZERO_RETURN) && (err2 == 0))
			{
				/* this code will be called when
				 * (1) moretoread -> returns true. +
				 * (2) SSL_read fails.
				 *
				 * There are two ways this can happen:
				 * (1) there is a little data on the socket, but not enough
				 * for a full SSL record, so there legimitately is no error, and the moretoread()
				 * was correct, but the read fails.
				 *
				 * (2) the socket has been closed correctly. this leads to moretoread() -> true, 
				 * and ZERO error.... we catch this case by counting how many times
				 * it occurs in a row (cos the other one will not).
				 */
				if (n_read_zero == 0)
				{
					/* first read_zero */
					mReadZeroTS = time(NULL);
				}

				++n_read_zero;
                                out << "ssl read : SSL_ERROR_ZERO_RETURN : nReadZero: " << n_read_zero;
				out << std::endl;

				if ((PQISSL_MAX_READ_ZERO_COUNT < n_read_zero)
					&& (time(NULL) - mReadZeroTS > PQISSL_MAX_READ_ZERO_TIME)) 
				{
					out << "Count passed Limit, shutting down!";
					out << " ReadZero Age: " << time(NULL) - mReadZeroTS;
					reset();
				}

				rslog(RSL_ALERT, pqisslzone, out.str());
				std::cerr << out.str() << std::endl ;
				return -1;
			}

			/* the only real error we expect */
			if (error == SSL_ERROR_SYSCALL)
			{
				out << "SSL_read() SSL_ERROR_SYSCALL";
				out << std::endl;
				out << "Socket Closed Abruptly.... Resetting PQIssl";
				out << std::endl;
				rslog(RSL_ALERT, pqisslzone, out.str());

				/* extra debugging - based on SSL_get_error() man page */
				{
					int syserr = errno;
					int sslerr = 0;
					std::ostringstream out2;
					out2 << "SSL_ERROR_SYSCALL, ret == " << tmppktlen;
					out2 << " errno: " << syserr << std::endl;
	
					while(0 != (sslerr = ERR_get_error()))
					{
						out2 << "SSLERR:" << sslerr << " : ";
	
						char sslbuf[256] = {0};
						out2 << ERR_error_string(sslerr, sslbuf);
						out2 << std::endl;
					}
					rslog(RSL_ALERT, pqisslzone, out2.str());
				}

				reset();
				std::cerr << out.str() << std::endl ;
				return -1;
			}
			else if (error == SSL_ERROR_WANT_WRITE)
			{
				out << "SSL_read() SSL_ERROR_WANT_WRITE";
				out << std::endl;
				rslog(RSL_ALERT, pqisslzone, out.str());
				std::cerr << out.str() << std::endl ;
				return -1;
			}
			else if (error == SSL_ERROR_WANT_READ)					// SSL_WANT_READ is not a crittical error. It's just a sign that
			{																	// the internal SSL buffer is not ready to accept more data. So -1 
//				out << "SSL_read() SSL_ERROR_WANT_READ";			// is returned, and the connexion will be retried as is on next
//				out << std::endl;											// call of readdata().
//				rslog(RSL_ALERT, pqisslzone, out.str());
				return -1;
			}
			else
			{
				out << "SSL_read() UNKNOWN ERROR: " << error;
				out << std::endl;
				out << "\tResetting!";
				rslog(RSL_ALERT, pqisslzone, out.str());
				std::cerr << out.str() << std::endl ;
				reset();
				return -1;
			}

			rslog(RSL_ALERT, pqisslzone, out.str());
			//exit(1);
		}
		else
			total_len+=tmppktlen ;
	} while(total_len < len) ;

#ifdef PQISSL_DEBUG
	std::cerr << "pqissl: have read data of length " << total_len << ", expected is " << len << std::endl ;
#endif

	if (len != total_len)
	{
		std::ostringstream out;
		out << "pqissl::readdata()";
		out << " Full Packet Not read!" << std::endl; 
		out << " -> Expected len(" << len << ") actually read(";
		out << total_len << ")" << std::endl;
		std::cerr << out.str() ;
		rslog(RSL_WARNING, pqisslzone, out.str());
	}
	total_len = 0 ;		// reset the packet pointer as we have finished a packet.
	n_read_zero = 0;
	return len;//tmppktlen;
}


// dummy function currently.
int 	pqissl::netstatus()
{
	return 1;
}

int 	pqissl::isactive()
{
	return active;
}

bool 	pqissl::moretoread()
{
	{
		std::ostringstream out;
		out << "pqissl::moretoread()";
		out << "  polling socket (" << sockfd << ")";
		rslog(RSL_DEBUG_ALL, pqisslzone, out.str());
	}

	fd_set ReadFDs, WriteFDs, ExceptFDs;
	FD_ZERO(&ReadFDs);
	FD_ZERO(&WriteFDs);
	FD_ZERO(&ExceptFDs);

	FD_SET(sockfd, &ReadFDs);
	FD_SET(sockfd, &WriteFDs);
	FD_SET(sockfd, &ExceptFDs);

	struct timeval timeout;
	timeout.tv_sec = 0; 
	timeout.tv_usec = 0; 

	if (select(sockfd + 1, &ReadFDs, &WriteFDs, &ExceptFDs, &timeout) < 0) 
	{
		rslog(RSL_ALERT, pqisslzone, 
			"pqissl::moretoread() Select ERROR!");
		return 0;
	}

	if (FD_ISSET(sockfd, &ExceptFDs))
	{
		// error - reset socket.
		rslog(RSL_ALERT, pqisslzone, 
			"pqissl::moretoread() Select Exception ERROR!");

		// this is a definite bad socket!.
		// reset.
		reset();
		return 0;
	}

	if (FD_ISSET(sockfd, &WriteFDs))
	{
		// write can work.
		rslog(RSL_DEBUG_ALL, pqisslzone, 
			"pqissl::moretoread() Can Write!");
	}
	else
	{
		// write can work.
		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			"pqissl::moretoread() Can *NOT* Write!");
	}

	if (FD_ISSET(sockfd, &ReadFDs))
	{
		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			"pqissl::moretoread() Data to Read!");
		return 1;
	}
	else
	{
		rslog(RSL_DEBUG_ALL, pqisslzone, 
			"pqissl::moretoread() No Data to Read!");
		return 0;
	}

}

bool 	pqissl::cansend()
{
	rslog(RSL_DEBUG_ALL, pqisslzone, 
		"pqissl::cansend() polling socket!");

	// Interestingly - This code might be portable....

	fd_set ReadFDs, WriteFDs, ExceptFDs;
	FD_ZERO(&ReadFDs);
	FD_ZERO(&WriteFDs);
	FD_ZERO(&ExceptFDs);

	FD_SET(sockfd, &ReadFDs);
	FD_SET(sockfd, &WriteFDs);
	FD_SET(sockfd, &ExceptFDs);

	struct timeval timeout;
	timeout.tv_sec = 0; 
	timeout.tv_usec = 0; 

	if (select(sockfd + 1, &ReadFDs, &WriteFDs, &ExceptFDs, &timeout) < 0) 
	{
		// select error.
		rslog(RSL_ALERT, pqisslzone, 
			"pqissl::cansend() Select Error!");

		return 0;
	}

	if (FD_ISSET(sockfd, &ExceptFDs))
	{
		// error - reset socket.
		rslog(RSL_ALERT, pqisslzone, 
			"pqissl::cansend() Select Exception!");

		// this is a definite bad socket!.
		// reset.
		reset();
		return 0;
	}

	if (FD_ISSET(sockfd, &WriteFDs))
	{
		// write can work.
		rslog(RSL_DEBUG_ALL, pqisslzone, 
			"pqissl::cansend() Can Write!");
		return 1;
	}
	else
	{
		// write can work.
		rslog(RSL_DEBUG_BASIC, pqisslzone, 
			"pqissl::cansend() Can *NOT* Write!");

		return 0;
	}

}

std::string pqissl::gethash()
{
	std::string dummyhash;
	return dummyhash;
}

/********** End of Implementation of BinInterface ******************/


