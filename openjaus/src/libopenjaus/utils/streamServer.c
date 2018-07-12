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
// File Name: streamServer.c
//
// Written By: Tom Galluzzo (galluzzo AT gmail DOT com)
//
// Version: 3.3.0
//
// Date: 07/09/08
//
// Description: This file describes the functionality associated with a StreamServer object. 
// Inspired by the class of the same name in the JAVA language.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "utils/streamServer.h"

StreamServer streamServerCreate(short port, InetAddress ipAddress)
{
	StreamServer streamServer;
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

	streamServer = (StreamServer) malloc( sizeof(StreamServerStruct) );
	if( streamServer == NULL )
	{
		return NULL;
	}
	
	// Open a socket with: Protocol Family (PF) IPv4, of Stream Socket Type, and an implicit protocol
	streamServer->descriptor = (int) socket(PF_INET, SOCK_STREAM, 0); 
	if(streamServer->descriptor == -1)
	{
		return NULL;
	}
	
	memset(&address, 0, sizeof(address));		// Clear the data structure to zero
	address.sin_family = AF_INET;				// Set Internet Socket (sin), Family to: Address Family (AF) IPv4 (INET)
	address.sin_addr.s_addr = ipAddress->value;	// Set Internet Socket (sin), Address to: The ipAddressString argument
	address.sin_port = htons(port);				// Set Internet Socket (sin), Port to: the port argument

	// Bind our open socket to a free port on the localhost, with our defined ipAddress
	if(bind(streamServer->descriptor, (struct sockaddr *)&address, sizeof(address)))
	{
		CLOSE_SOCKET(streamServer->descriptor);
		return NULL;
	}

	if(getsockname(streamServer->descriptor, (struct sockaddr *)&address, &addressLength))
	{
		CLOSE_SOCKET(streamServer->descriptor);
		return NULL;
	}

	if(listen(streamServer->descriptor, 1))
	{
		CLOSE_SOCKET(streamServer->descriptor);
		return NULL;
	}

	FD_ZERO(&streamServer->masterSet);
	FD_SET(streamServer->descriptor, &streamServer->masterSet);
	streamServer->maxDescriptor = streamServer->descriptor;
	streamServer->clientAddress = (InetAddress *)malloc((streamServer->maxDescriptor + 1)*sizeof(InetAddress));
	streamServer->clientPort = (unsigned short *)malloc((streamServer->maxDescriptor + 1)*sizeof(unsigned short));
	streamServer->pendingDescriptors = 0;

	streamServer->address = inetAddressCreate();
	streamServer->address->value = ipAddress->value;
	streamServer->port = ntohs(address.sin_port);
	streamServer->timeout.tv_sec = 0;
	streamServer->timeout.tv_usec = 0;
	streamServer->blocking = 1;

	//printf("Server opened %d, on port %d\n", streamServer->descriptor, streamServer->port);

	return streamServer;
}

void streamServerDestroy(StreamServer streamServer)
{
	CLOSE_SOCKET(streamServer->descriptor);
	free(streamServer);
#ifdef WIN32	
	// Initialize the socket subsystem
	WSACleanup();
#endif
}

int streamServerAccept(StreamServer streamServer)
{
	int newDescriptor = 0;
	struct sockaddr_in fromAddress;
	socklen_t fromAddressLength = sizeof(fromAddress);

	newDescriptor = (int)accept(streamServer->descriptor, (struct sockaddr*)&fromAddress, &fromAddressLength);
	if(newDescriptor == -1)
	{
		return 0;
	}

	FD_SET(newDescriptor, &streamServer->masterSet);

	if(newDescriptor > streamServer->maxDescriptor)
	{
		streamServer->clientAddress = (InetAddress *)realloc(streamServer->clientAddress, (newDescriptor + 1)*sizeof(InetAddress));
		streamServer->clientPort = (unsigned short *)realloc(streamServer->clientPort, (newDescriptor + 1)*sizeof(unsigned short));
		streamServer->maxDescriptor = newDescriptor;
	}

	streamServer->clientPort[newDescriptor] = ntohs(fromAddress.sin_port);
	streamServer->clientAddress[newDescriptor] = inetAddressCreate();
	streamServer->clientAddress[newDescriptor]->value = fromAddress.sin_addr.s_addr;

	//printf("Server accepted socket %d, on port: %d\n", newDescriptor, streamServer->clientPort[newDescriptor]);

	return 1;
}

int streamServerSend(StreamServer streamServer, StreamPacket packet)
{
	int i = 0;

	for(i = 0; i <= streamServer->maxDescriptor; i++)
	{
		if(	FD_ISSET(i, &streamServer->masterSet) &&
			i != streamServer->descriptor &&
			streamServer->clientPort[i] == packet->port &&
			streamServer->clientAddress[i]->value == packet->address->value)
		{
			return send(i, (void *)packet->buffer, packet->bufferSizeBytes, 0);
		}
	}

	return 0;
}

int streamServerReceive(StreamServer streamServer, StreamPacket packet)
{
	struct timeval timeout;
	struct timeval *timeoutPtr = NULL;
	int bytesReceived = 0;
	int selectReturnVal = 0;

	if(streamServer->pendingDescriptors < 1)
	{
		if(!streamServer->blocking)
		{
			timeout = streamServer->timeout;
			timeoutPtr = &timeout;
		}
		
		streamServer->readSet = streamServer->masterSet;

		selectReturnVal = select(streamServer->maxDescriptor + 1, &streamServer->readSet, NULL, NULL, timeoutPtr);
		if(selectReturnVal < 1) //Timeout or error has occured
		{
			return selectReturnVal;
		}
		
		streamServer->pendingDescriptors = selectReturnVal;
		streamServer->currentDescriptor = 0;
	}


	while(streamServer->currentDescriptor <= streamServer->maxDescriptor)
	{
		if(FD_ISSET(streamServer->currentDescriptor, &streamServer->readSet))
		{
			streamServer->pendingDescriptors--;

			if(streamServer->currentDescriptor == streamServer->descriptor) // New client connection to listener
			{
				streamServerAccept(streamServer);
			}
			else
			{
				bytesReceived = recv(streamServer->currentDescriptor, packet->buffer, packet->bufferSizeBytes, 0);
				if(bytesReceived > -1)
				{
					packet->address = streamServer->clientAddress[streamServer->currentDescriptor];
					packet->port = streamServer->clientPort[streamServer->currentDescriptor];
					streamServer->currentDescriptor++;
					return bytesReceived;
				}
				else
				{
					CLOSE_SOCKET(streamServer->currentDescriptor);
					FD_CLR(streamServer->currentDescriptor, &streamServer->masterSet);
					if(streamServer->currentDescriptor == streamServer->maxDescriptor)
					{
						streamServer->maxDescriptor--;
					}
					//printf("Server closing socket: %d\n", streamServer->currentDescriptor);
				}
			}
		}

		streamServer->currentDescriptor++;
	}

	streamServer->pendingDescriptors = 0;

	return 0;
}

void streamServerSetTimeout(StreamServer streamServer, double timeoutSec)
{
	long sec, usec;
	
	sec = (long)timeoutSec;
	usec = (long)((timeoutSec - (double)sec) * 1.0e6);
			
	streamServer->timeout.tv_sec = sec;
	streamServer->timeout.tv_usec = usec;
	streamServer->blocking = 0;
}



