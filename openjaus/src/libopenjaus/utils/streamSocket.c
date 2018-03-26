/*****************************************************************************
 *  Copyright (c) 2008, University of Florida
 *  All rights reserved.
 *
 *  This file is part of OpenJAUS.  OpenJAUS is distributed under the BSD
 *  license.  See the LICENSE file for details.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of the University of Florida nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
// File Name: streamSocket.c
//
// Written By: Tom Galluzzo (galluzzo AT gmail DOT com)
//
// Version: 3.3.0
//
// Date: 07/09/08
//
// Description: This file describes the functionality associated with a StreamSocket object.
// Inspired by the class of the same name in the JAVA language.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "utils/streamSocket.h"

StreamSocket streamSocketCreate(short port, InetAddress ipAddress)
{
	StreamSocket streamSocket;
	struct sockaddr_in address;
	socklen_t addressLength = sizeof(address);

#ifdef WIN32
	// Initialize the socket subsystem
	WSADATA wsaData;
	int err;
	err = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if(err != 0)
	{
		return NULL;
	}
#endif

	streamSocket = (StreamSocket) malloc( sizeof(StreamSocketStruct) );
	if( streamSocket == NULL )
	{
		return NULL;
	}

	// Open a socket with: Protocol Family (PF) IPv4, of Stream Socket Type, and an implicit protocol
	streamSocket->descriptor = (int) socket(PF_INET, SOCK_STREAM, 0);
	if(streamSocket->descriptor == -1)
	{
		return NULL;
	}

	memset(&address, 0, sizeof(address));		// Clear the data structure to zero
	address.sin_family = AF_INET;				// Set Internet Socket (sin), Family to: Address Family (AF) IPv4 (INET)
	address.sin_addr.s_addr = ipAddress->value;	// Set Internet Socket (sin), Address to: The ipAddressString argument
	address.sin_port = htons(port);				// Set Internet Socket (sin), Port to: the port argument

	// Bind our open socket to a free port on the localhost, with our defined ipAddress
	if(bind(streamSocket->descriptor, (struct sockaddr *)&address, sizeof(address)))
	{
		CLOSE_SOCKET(streamSocket->descriptor);
		return NULL;
	}

	if(getsockname(streamSocket->descriptor, (struct sockaddr *)&address, &addressLength))
	{
		CLOSE_SOCKET(streamSocket->descriptor);
		return NULL;
	}

	streamSocket->address = inetAddressCreate();
	streamSocket->address->value = ipAddress->value;
	streamSocket->port = ntohs(address.sin_port);
	streamSocket->timeout.tv_sec = 0;
	streamSocket->timeout.tv_usec = 0;
	streamSocket->blocking = 1;

	return streamSocket;
}

void streamSocketDestroy(StreamSocket streamSocket)
{
	CLOSE_SOCKET(streamSocket->descriptor);
	inetAddressDestroy(streamSocket->address);
	free(streamSocket);
#ifdef WIN32
	// Initialize the socket subsystem
	WSACleanup();
#endif
}

StreamSocket streamSocketListenAndAccept(StreamSocket streamSocket)
{
	StreamSocket newSocket;
	struct sockaddr_in fromAddress;
	socklen_t fromAddressLength = sizeof(fromAddress);

	if(listen(streamSocket->descriptor, 1))
	{
		return NULL;
	}

	newSocket = (StreamSocket) malloc( sizeof(StreamSocketStruct) );
	if( newSocket == NULL )
	{
		return NULL;
	}

	newSocket->descriptor = (int)accept(streamSocket->descriptor, (struct sockaddr*)&fromAddress, &fromAddressLength);
	if(newSocket->descriptor == -1)
	{
		return NULL;
	}

	newSocket->port = ntohs(fromAddress.sin_port);
	newSocket->address = inetAddressCreate();
	newSocket->address->value = fromAddress.sin_addr.s_addr;
	newSocket->timeout.tv_sec = 0;
	newSocket->timeout.tv_usec = 0;
	newSocket->blocking = 1;

	return newSocket;
}

int streamSocketConnectTo(StreamSocket streamSocket, short port, InetAddress ipAddress)
{
	struct sockaddr_in toAddress;

	memset(&toAddress, 0, sizeof(toAddress));
	toAddress.sin_family = AF_INET;							// Set Internet Socket (sin), Family to: Address Family (AF) IPv4 (INET)
	toAddress.sin_addr.s_addr = ipAddress->value;			// Set Internet Socket (sin), Address to: Dest ipAddressString
	toAddress.sin_port = htons(port);						// Set Internet Socket (sin), Port to: Dest port

	return connect(streamSocket->descriptor, (struct sockaddr *)&toAddress, sizeof(toAddress));
}

int streamSocketSend(StreamSocket streamSocket, StreamPacket packet)
{
	return send(streamSocket->descriptor, (void *)packet->buffer, packet->bufferSizeBytes, 0);
}

int streamSocketReceive(StreamSocket streamSocket, StreamPacket packet)
{
	struct timeval timeout;
	struct timeval *timeoutPtr = NULL;
	fd_set readSet;
	int selcetReturnVal = -2;

	if(!streamSocket->blocking)
	{
		timeout = streamSocket->timeout;
		timeoutPtr = &timeout;
	}

	FD_ZERO(&readSet);
	FD_SET(streamSocket->descriptor, &readSet);

	selcetReturnVal = select(streamSocket->descriptor + 1, &readSet, NULL, NULL, timeoutPtr);
	if(selcetReturnVal > 0)
	{
		if(FD_ISSET(streamSocket->descriptor, &readSet))
		{
			return recv(streamSocket->descriptor, packet->buffer, packet->bufferSizeBytes, 0);
		}
		else
		{
			return -2;
		}
	}
	else
	{
		return selcetReturnVal;
	}
}

void streamSocketSetTimeout(StreamSocket streamSocket, double timeoutSec)
{
	long sec, usec;

	sec = (long)timeoutSec;
	usec = (long)((timeoutSec - (double)sec) * 1.0e6);

	streamSocket->timeout.tv_sec = sec;
	streamSocket->timeout.tv_usec = usec;
	streamSocket->blocking = 0;
}
