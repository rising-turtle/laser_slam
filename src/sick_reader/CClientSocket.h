/*
 * CClientSocket.h
 *
 *  Created on: Nov 26, 2012
 *      Author: liu
 */

#ifndef CCLIENTSOCKET_H_
#define CCLIENTSOCKET_H_

#include "cpp_utils.h"

#define  INVALID_SOCKET		(-1)
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <stdio.h>
#include <string.h>

class CClientSocket
{
	friend class CServerSocket;

public:
	/** See description of CClientTCPSocket */
	static unsigned int DNS_LOOKUP_TIMEOUT_MS;

protected:

	/** The handle for the connected TCP socket, or -1
	 */
	int				m_hSock;

	/** The IP address of the remote part of the connection.
	 */
	std::string		m_remotePartIP;

	/** The TCP port of the remote part of the connection.
	 */
	unsigned short	m_remotePartPort;


	/** Introduces a virtual method responsible for reading from the stream (This method BLOCKS)
	 * This method is implemented as a call to "readAsync" with infinite timeouts.
	 * \sa readAsync
	 */
	size_t  Read(void *Buffer, size_t Count);

	/** Introduces a virtual method responsible for writing to the stream.
	 *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
	 *  This method is implemented as a call to "writeAsync" with infinite timeouts.
	 * \sa writeAsync
	 */
	size_t  Write(const void *Buffer, size_t Count);

	/** Returns a description of the last error */
	std::string  getLastErrorStr();

public:
	/** Default constructor
	 * \sa connect
	 */
	CClientSocket( );

	/** Destructor
	 */
	~CClientSocket( );

	/** Establishes a connection with a remote part.
	 * \param remotePartAddress This string can be a host name, like "server" or "www.mydomain.org", or an IP address "11.22.33.44".
	 * \param remotePartTCPPort The port on the remote machine to connect to.
	 * \param timeout_ms  The timeout to wait for the connection (0: NO TIMEOUT)
	 * \exception This method raises an exception if an error is found with a textual description of the error.
	 */
	void connect(
			const std::string	&remotePartAddress,
			unsigned short		remotePartTCPPort,
			unsigned int		timeout_ms = 0 );

	/** Returns true if this objects represents a successfully connected socket.
	 */
	bool  isConnected();

	/** Closes the connection.
	 */
	void  close();

	/** Writes a string to the socket.
	 * \exception std::exception On communication errors
	 */
	void  sendString( const std::string &str );



	/** A method for reading from the socket with an optional timeout.
	 * \param Buffer The destination of data.
	 * \param Cound The number of bytes to read.
	 * \param timeoutStart_ms The maximum timeout (in milliseconds) to wait for the starting of data from the other side.
	 * \param timeoutBetween_ms The maximum timeout (in milliseconds) to wait for a chunk of data after a previous one.
	 *  Set timeout's to -1 to block until the desired number of bytes are read, or an error happens.
	 *  \return The number of actually read bytes.
	 */
	size_t  readAsync(
			void	*Buffer,
			const size_t	Count,
			const int	timeoutStart_ms = -1,
			const int	timeoutBetween_ms = -1);

	/** A method for writing to the socket with optional timeouts.
	 *  The method supports writing block by block as the socket allows us to write more data.
	 * \param Buffer The data.
	 * \param Cound The number of bytes to write.
	 * \param timeout_ms The maximum timeout (in milliseconds) to wait for the socket to be available for writing (for each block).
	 *  Set timeout's to -1 to block until the desired number of bytes are written, or an error happens.
	 *  \return The number of actually written bytes.
	 */
	size_t  writeAsync(
			const void	*Buffer,
			const size_t Count,
			const int	timeout_ms = -1 );

	/** Send a message through the TCP stream.
	 * \param outMsg The message to be shown.
	 * \param timeout_ms The maximum timeout (in milliseconds) to wait for the socket in each write operation.
	 * \return Returns false on any error, or true if everything goes fine.
	 */
	/*
	bool  sendMessage(
			const CMessage&	outMsg,
			const int timeout_ms = -1
	);
	 */

	/** Waits for an incoming message through the TCP stream.
	 * \param inMsg The received message is placed here.
	 * \param timeoutStart_ms The maximum timeout (in milliseconds) to wait for the starting of data from the other side.
	 * \param timeoutBetween_ms The maximum timeout (in milliseconds) to wait for a chunk of data after a previous one.
	 * \return Returns false on any error (or timeout), or true if everything goes fine.
	 */
	/*
	bool  receiveMessage(
			CMessage&			inMsg,
			const unsigned int	timeoutStart_ms = 100,
			const unsigned int	timeoutBetween_ms = 1000
	);
	 */

	/** Return the number of bytes already in the receive queue (they can be read without waiting) */
	size_t  getReadPendingBytes();

}; // End of class def.


#endif /* CCLIENTSOCKET_H_ */
