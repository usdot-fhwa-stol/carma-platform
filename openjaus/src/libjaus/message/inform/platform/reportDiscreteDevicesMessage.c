/*****************************************************************************
 *  Copyright (c) 2009, OpenJAUS.com
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
// File Name: reportDiscreteDevicesMessage.c
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo (galluzzo AT gmail DOT com)
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines the functionality of a ReportDiscreteDevicesMessage


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "jaus.h"

static const int commandCode = JAUS_REPORT_DISCRETE_DEVICES;
static const int maxDataSizeBytes = 5;

static JausBoolean headerFromBuffer(ReportDiscreteDevicesMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static JausBoolean headerToBuffer(ReportDiscreteDevicesMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static int headerToString(ReportDiscreteDevicesMessage message, char **buf);

static JausBoolean dataFromBuffer(ReportDiscreteDevicesMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static int dataToBuffer(ReportDiscreteDevicesMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static void dataInitialize(ReportDiscreteDevicesMessage message);
static unsigned int dataSize(ReportDiscreteDevicesMessage message);

// ************************************************************************************************************** //
//                                    USER CONFIGURED FUNCTIONS
// ************************************************************************************************************** //

// Initializes the message-specific fields
static void dataInitialize(ReportDiscreteDevicesMessage message)
{
	// Set initial values of message fields
	message->presenceVector = newJausByte(JAUS_BYTE_PRESENCE_VECTOR_ALL_ON);
	message->gear = newJausByte(0);
	message->transferCase = newJausByte(0);

	// Main Propulsion
	message->mainPropulsion = JAUS_FALSE;
	message->mainFuelSupply = JAUS_FALSE;
	message->auxFuelSupply = JAUS_FALSE;
	message->powerAuxDevices = JAUS_FALSE;
	message->startingDevice = JAUS_FALSE;
	message->coldStart = JAUS_FALSE;
	message->automaticStart = JAUS_FALSE;
	message->automaticStop = JAUS_FALSE;

	// Parking, Brake and Horn
	message->parkingBrake = JAUS_FALSE;
	message->horn = JAUS_FALSE;
}

// Return boolean of success
static JausBoolean dataFromBuffer(ReportDiscreteDevicesMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	int index = 0;
	JausByte tempByte;

	if(bufferSizeBytes == message->dataSize)
	{
		// Unpack Message Fields from Buffer
		// Unpack according to presence vector
		if(!jausByteFromBuffer(&message->presenceVector, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
		index += JAUS_BYTE_SIZE_BYTES;

		if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_PROPULSION_BIT))
		{
			//unpack
			if(!jausByteFromBuffer(&tempByte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
			index += JAUS_BYTE_SIZE_BYTES;

			message->mainPropulsion = jausByteIsBitSet(tempByte, JAUS_DEVICES_PROPULSION_BF_MAIN_POWER_BIT)? JAUS_TRUE : JAUS_FALSE;
			message->mainFuelSupply = jausByteIsBitSet(tempByte, JAUS_DEVICES_PROPULSION_BF_MAIN_FUEL_BIT)? JAUS_TRUE : JAUS_FALSE;
			message->auxFuelSupply = jausByteIsBitSet(tempByte, JAUS_DEVICES_PROPULSION_BF_AUXILARY_FUEL_BIT)? JAUS_TRUE : JAUS_FALSE;
			message->powerAuxDevices = jausByteIsBitSet(tempByte, JAUS_DEVICES_PROPULSION_BF_AUXILARY_POWER_BIT)? JAUS_TRUE : JAUS_FALSE;
			message->startingDevice = jausByteIsBitSet(tempByte, JAUS_DEVICES_PROPULSION_BF_STARTING_DEVICE_BIT)? JAUS_TRUE : JAUS_FALSE;
			message->coldStart = jausByteIsBitSet(tempByte, JAUS_DEVICES_PROPULSION_BF_COLD_START_BIT)? JAUS_TRUE : JAUS_FALSE;
			message->automaticStart = jausByteIsBitSet(tempByte, JAUS_DEVICES_PROPULSION_BF_AUTO_START_BIT)? JAUS_TRUE : JAUS_FALSE;
			message->automaticStop = jausByteIsBitSet(tempByte, JAUS_DEVICES_PROPULSION_BF_AUTO_SHUTDOWN_BIT)? JAUS_TRUE : JAUS_FALSE;
		}

		if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_PARKING_BIT))
		{
			//unpack
			if(!jausByteFromBuffer(&tempByte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
			index += JAUS_BYTE_SIZE_BYTES;

			message->parkingBrake = jausByteIsBitSet(tempByte, JAUS_DEVICES_OTHER_BF_PARKING_BRAKE_BIT)? JAUS_TRUE : JAUS_FALSE;
			message->horn = jausByteIsBitSet(tempByte, JAUS_DEVICES_OTHER_BF_HORN_BIT)? JAUS_TRUE : JAUS_FALSE;
		}

		if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_GEAR_BIT))
		{
			//unpack
			if(!jausByteFromBuffer(&message->gear, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
			index += JAUS_BYTE_SIZE_BYTES;
		}

		if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_TRANSFER_BIT))
		{
			//unpack
			if(!jausByteFromBuffer(&message->transferCase, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
			index += JAUS_BYTE_SIZE_BYTES;
		}

		return JAUS_TRUE;
	}
	else
	{
		return JAUS_FALSE;
	}
}

// Returns number of bytes put into the buffer
static int dataToBuffer(ReportDiscreteDevicesMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	int index = 0;
	JausByte tempByte = 0;

	if(bufferSizeBytes >= dataSize(message))
	{
		// Pack Message Fields to Buffer
		// Pack according to presence vector
		if(!jausByteToBuffer(message->presenceVector, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
		index += JAUS_BYTE_SIZE_BYTES;

		if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_PROPULSION_BIT))
		{
			tempByte = 0;
			if(message->mainPropulsion) jausByteSetBit(&tempByte, JAUS_DEVICES_PROPULSION_BF_MAIN_POWER_BIT);
			if(message->mainFuelSupply) jausByteSetBit(&tempByte, JAUS_DEVICES_PROPULSION_BF_MAIN_FUEL_BIT);
			if(message->auxFuelSupply) jausByteSetBit(&tempByte, JAUS_DEVICES_PROPULSION_BF_AUXILARY_FUEL_BIT);
			if(message->powerAuxDevices) jausByteSetBit(&tempByte, JAUS_DEVICES_PROPULSION_BF_AUXILARY_POWER_BIT);
			if(message->startingDevice) jausByteSetBit(&tempByte, JAUS_DEVICES_PROPULSION_BF_STARTING_DEVICE_BIT);
			if(message->coldStart) jausByteSetBit(&tempByte, JAUS_DEVICES_PROPULSION_BF_COLD_START_BIT);
			if(message->automaticStart) jausByteSetBit(&tempByte, JAUS_DEVICES_PROPULSION_BF_AUTO_START_BIT);
			if(message->automaticStop) jausByteSetBit(&tempByte, JAUS_DEVICES_PROPULSION_BF_AUTO_SHUTDOWN_BIT);

			//pack
			if(!jausByteToBuffer(tempByte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
			index += JAUS_BYTE_SIZE_BYTES;
		}

		if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_PARKING_BIT))
		{
			tempByte = 0;
			if(message->parkingBrake) jausByteSetBit(&tempByte, JAUS_DEVICES_OTHER_BF_PARKING_BRAKE_BIT);
			if(message->horn) jausByteSetBit(&tempByte, JAUS_DEVICES_OTHER_BF_HORN_BIT);

			//pack
			if(!jausByteToBuffer(tempByte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
			index += JAUS_BYTE_SIZE_BYTES;
		}

		if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_GEAR_BIT))
		{
			//pack
			if(!jausByteToBuffer(message->gear, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
			index += JAUS_BYTE_SIZE_BYTES;
		}

		if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_TRANSFER_BIT))
		{
			//pack
			if(!jausByteToBuffer(message->transferCase, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
			index += JAUS_BYTE_SIZE_BYTES;
		}
	}

	return index;
}

static int dataToString(ReportDiscreteDevicesMessage message, char **buf)
{
  //message already verified

  //Setup temporary string buffer

  unsigned int bufSize = 700;
  (*buf) = (char*)malloc(sizeof(char)*bufSize);

  strcpy((*buf), "\nPresence Vector: " );
  jausByteToHexString(message->presenceVector, (*buf)+strlen(*buf));


  if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_PROPULSION_BIT))
  {
    strcat((*buf), "\nMain Propulsion\n  Main Propulsion: " );

    if( message->mainPropulsion == JAUS_TRUE )
      strcat((*buf), "On");
    else
      strcat((*buf), "Off");

    strcat((*buf), "\n  Main Energy/Fuel Supply: " );

    if( message->mainFuelSupply == JAUS_TRUE )
      strcat((*buf), "On");
    else
      strcat((*buf), "Off");

    strcat((*buf), "\n  Auxiliary Energy/Fuel Supply: " );

    if( message->auxFuelSupply == JAUS_TRUE )
      strcat((*buf), "On");
    else
      strcat((*buf), "Off");

    strcat((*buf), "\n  Power to Auxiliary Devices: " );

    if( message->powerAuxDevices == JAUS_TRUE )
      strcat((*buf), "On");
    else
      strcat((*buf), "Off");

    strcat((*buf), "\n  Starting Device: " );

    if( message->startingDevice == JAUS_TRUE )
      strcat((*buf), "On");
    else
      strcat((*buf), "Off");

    strcat((*buf), "\n  Cold Start: " );

    jausBooleanToString(message->coldStart, (*buf)+strlen(*buf));

    strcat((*buf), "\n  Commence Automatic Start Sequence: " );

    jausBooleanToString(message->automaticStart, (*buf)+strlen(*buf));

    strcat((*buf), "\n  Commence Automatic Shutdown Sequence: " );

    jausBooleanToString(message->automaticStop, (*buf)+strlen(*buf));
  }

  if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_PARKING_BIT))
  {
    strcat((*buf), "\nParking Brake and Horn\n  Parking Brake: " );

    if( message->parkingBrake == JAUS_TRUE  )
      strcat((*buf), "Set");
    else
      strcat((*buf), "Release");

    strcat((*buf), "\n  Horn: " );

    if( message->horn == JAUS_TRUE )
      strcat((*buf), "On");
    else
      strcat((*buf), "Off");
  }

  if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_GEAR_BIT))
  {
    strcat((*buf), "\nGear: " );

    jausByteToString(message->gear, (*buf)+strlen(*buf));

    if( message->gear == 0 )
      strcat((*buf), " Park");
    else if ( message->gear == 128 )
      strcat((*buf), " Neutral");
    else if ( (message->gear >= 1) && (message->gear <= 127) )
      strcat((*buf), " Forward");
    else if ( message->gear > 128 )
      strcat((*buf), " Reverse");
  }

  if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_TRANSFER_BIT))
  {
    strcat((*buf), "\nTransfer Case: " );

    jausByteToString(message->transferCase, (*buf)+strlen(*buf));

    if ( message->transferCase == 128 )
      strcat((*buf), " Neutral");
    else if ( message->transferCase <= 127 )
      strcat((*buf), " Low");
    else if ( message->transferCase > 128 )
      strcat((*buf), " High");
  }

  return (int)strlen(*buf);
}

// Returns number of bytes put into the buffer
static unsigned int dataSize(ReportDiscreteDevicesMessage message)
{
	int index = 0;

	index += JAUS_BYTE_SIZE_BYTES;

	if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_PROPULSION_BIT))
	{
		index += JAUS_BYTE_SIZE_BYTES;
	}

	if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_PARKING_BIT))
	{
		index += JAUS_BYTE_SIZE_BYTES;
	}

	if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_GEAR_BIT))
	{
		index += JAUS_BYTE_SIZE_BYTES;
	}

	if(jausByteIsBitSet(message->presenceVector, JAUS_DEVICES_PV_TRANSFER_BIT))
	{
		index += JAUS_BYTE_SIZE_BYTES;
	}

	return index;
}

// ************************************************************************************************************** //
//                                    NON-USER CONFIGURED FUNCTIONS
// ************************************************************************************************************** //

ReportDiscreteDevicesMessage reportDiscreteDevicesMessageCreate(void)
{
	ReportDiscreteDevicesMessage message;

	message = (ReportDiscreteDevicesMessage)malloc( sizeof(ReportDiscreteDevicesMessageStruct) );
	if(message == NULL)
	{
		return NULL;
	}

	// Initialize Values
	message->properties.priority = JAUS_DEFAULT_PRIORITY;
	message->properties.ackNak = JAUS_ACK_NAK_NOT_REQUIRED;
	message->properties.scFlag = JAUS_NOT_SERVICE_CONNECTION_MESSAGE;
	message->properties.expFlag = JAUS_NOT_EXPERIMENTAL_MESSAGE;
	message->properties.version = JAUS_VERSION_3_3;
	message->properties.reserved = 0;
	message->commandCode = commandCode;
	message->destination = jausAddressCreate();
	message->source = jausAddressCreate();
	message->dataFlag = JAUS_SINGLE_DATA_PACKET;
	message->dataSize = maxDataSizeBytes;
	message->sequenceNumber = 0;

	dataInitialize(message);
	message->dataSize = dataSize(message);

	return message;
}

void reportDiscreteDevicesMessageDestroy(ReportDiscreteDevicesMessage message)
{
	jausAddressDestroy(message->source);
	jausAddressDestroy(message->destination);
	free(message);
}

JausBoolean reportDiscreteDevicesMessageFromBuffer(ReportDiscreteDevicesMessage message, unsigned char* buffer, unsigned int bufferSizeBytes)
{
	int index = 0;

	if(headerFromBuffer(message, buffer+index, bufferSizeBytes-index))
	{
		index += JAUS_HEADER_SIZE_BYTES;
		if(dataFromBuffer(message, buffer+index, bufferSizeBytes-index))
		{
			return JAUS_TRUE;
		}
		else
		{
			return JAUS_FALSE;
		}
	}
	else
	{
		return JAUS_FALSE;
	}
}

JausBoolean reportDiscreteDevicesMessageToBuffer(ReportDiscreteDevicesMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	if(bufferSizeBytes < reportDiscreteDevicesMessageSize(message))
	{
		return JAUS_FALSE; //improper size
	}
	else
	{
		message->dataSize = dataToBuffer(message, buffer+JAUS_HEADER_SIZE_BYTES, bufferSizeBytes - JAUS_HEADER_SIZE_BYTES);
		if(headerToBuffer(message, buffer, bufferSizeBytes))
		{
			return JAUS_TRUE;
		}
		else
		{
			return JAUS_FALSE; // headerToReportDescreteDevicesBuffer failed
		}
	}
}

ReportDiscreteDevicesMessage reportDiscreteDevicesMessageFromJausMessage(JausMessage jausMessage)
{
	ReportDiscreteDevicesMessage message;

	if(jausMessage->commandCode != commandCode)
	{
		return NULL; // Wrong message type
	}
	else
	{
		message = (ReportDiscreteDevicesMessage)malloc( sizeof(ReportDiscreteDevicesMessageStruct) );
		if(message == NULL)
		{
			return NULL;
		}

		message->properties.priority = jausMessage->properties.priority;
		message->properties.ackNak = jausMessage->properties.ackNak;
		message->properties.scFlag = jausMessage->properties.scFlag;
		message->properties.expFlag = jausMessage->properties.expFlag;
		message->properties.version = jausMessage->properties.version;
		message->properties.reserved = jausMessage->properties.reserved;
		message->commandCode = jausMessage->commandCode;
		message->destination = jausAddressCreate();
		*message->destination = *jausMessage->destination;
		message->source = jausAddressCreate();
		*message->source = *jausMessage->source;
		message->dataSize = jausMessage->dataSize;
		message->dataFlag = jausMessage->dataFlag;
		message->sequenceNumber = jausMessage->sequenceNumber;

		// Unpack jausMessage->data
		if(dataFromBuffer(message, jausMessage->data, jausMessage->dataSize))
		{
			return message;
		}
		else
		{
			return NULL;
		}
	}
}

JausMessage reportDiscreteDevicesMessageToJausMessage(ReportDiscreteDevicesMessage message)
{
	JausMessage jausMessage;

	jausMessage = (JausMessage)malloc( sizeof(struct JausMessageStruct) );
	if(jausMessage == NULL)
	{
		return NULL;
	}

	jausMessage->properties.priority = message->properties.priority;
	jausMessage->properties.ackNak = message->properties.ackNak;
	jausMessage->properties.scFlag = message->properties.scFlag;
	jausMessage->properties.expFlag = message->properties.expFlag;
	jausMessage->properties.version = message->properties.version;
	jausMessage->properties.reserved = message->properties.reserved;
	jausMessage->commandCode = message->commandCode;
	jausMessage->destination = jausAddressCreate();
	*jausMessage->destination = *message->destination;
	jausMessage->source = jausAddressCreate();
	*jausMessage->source = *message->source;
	jausMessage->dataSize = dataSize(message);
	jausMessage->dataFlag = message->dataFlag;
	jausMessage->sequenceNumber = message->sequenceNumber;

	jausMessage->data = (unsigned char *)malloc(jausMessage->dataSize);
	jausMessage->dataSize = dataToBuffer(message, jausMessage->data, jausMessage->dataSize);

	return jausMessage;
}


unsigned int reportDiscreteDevicesMessageSize(ReportDiscreteDevicesMessage message)
{
	return (unsigned int)(dataSize(message) + JAUS_HEADER_SIZE_BYTES);
}

char* reportDiscreteDevicesMessageToString(ReportDiscreteDevicesMessage message)
{
  if(message)
  {
    char* buf1 = NULL;
    char* buf2 = NULL;
    char* buf = NULL;

    int returnVal;

    //Print the message header to the string buffer
    returnVal = headerToString(message, &buf1);

    //Print the message data fields to the string buffer
    returnVal += dataToString(message, &buf2);

buf = (char*)malloc(strlen(buf1)+strlen(buf2)+1);
    strcpy(buf, buf1);
    strcat(buf, buf2);

    free(buf1);
    free(buf2);

    return buf;
  }
  else
  {
    char* buf = "Invalid ReportDiscreteDevices Message";
    char* msg = (char*)malloc(strlen(buf)+1);
    strcpy(msg, buf);
    return msg;
  }
}
//********************* PRIVATE HEADER FUNCTIONS **********************//

static JausBoolean headerFromBuffer(ReportDiscreteDevicesMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	if(bufferSizeBytes < JAUS_HEADER_SIZE_BYTES)
	{
		return JAUS_FALSE;
	}
	else
	{
		// unpack header
		message->properties.priority = (buffer[0] & 0x0F);
		message->properties.ackNak	 = ((buffer[0] >> 4) & 0x03);
		message->properties.scFlag	 = ((buffer[0] >> 6) & 0x01);
		message->properties.expFlag	 = ((buffer[0] >> 7) & 0x01);
		message->properties.version	 = (buffer[1] & 0x3F);
		message->properties.reserved = ((buffer[1] >> 6) & 0x03);

		message->commandCode = buffer[2] + (buffer[3] << 8);

		message->destination->instance = buffer[4];
		message->destination->component = buffer[5];
		message->destination->node = buffer[6];
		message->destination->subsystem = buffer[7];

		message->source->instance = buffer[8];
		message->source->component = buffer[9];
		message->source->node = buffer[10];
		message->source->subsystem = buffer[11];

		message->dataSize = buffer[12] + ((buffer[13] & 0x0F) << 8);

		message->dataFlag = ((buffer[13] >> 4) & 0x0F);

		message->sequenceNumber = buffer[14] + (buffer[15] << 8);

		return JAUS_TRUE;
	}
}

static JausBoolean headerToBuffer(ReportDiscreteDevicesMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	JausUnsignedShort *propertiesPtr = (JausUnsignedShort*)&message->properties;

	if(bufferSizeBytes < JAUS_HEADER_SIZE_BYTES)
	{
		return JAUS_FALSE;
	}
	else
	{
		buffer[0] = (unsigned char)(*propertiesPtr & 0xFF);
		buffer[1] = (unsigned char)((*propertiesPtr & 0xFF00) >> 8);

		buffer[2] = (unsigned char)(message->commandCode & 0xFF);
		buffer[3] = (unsigned char)((message->commandCode & 0xFF00) >> 8);

		buffer[4] = (unsigned char)(message->destination->instance & 0xFF);
		buffer[5] = (unsigned char)(message->destination->component & 0xFF);
		buffer[6] = (unsigned char)(message->destination->node & 0xFF);
		buffer[7] = (unsigned char)(message->destination->subsystem & 0xFF);

		buffer[8] = (unsigned char)(message->source->instance & 0xFF);
		buffer[9] = (unsigned char)(message->source->component & 0xFF);
		buffer[10] = (unsigned char)(message->source->node & 0xFF);
		buffer[11] = (unsigned char)(message->source->subsystem & 0xFF);

		buffer[12] = (unsigned char)(message->dataSize & 0xFF);
		buffer[13] = (unsigned char)((message->dataFlag & 0xFF) << 4) | (unsigned char)((message->dataSize & 0x0F00) >> 8);

		buffer[14] = (unsigned char)(message->sequenceNumber & 0xFF);
		buffer[15] = (unsigned char)((message->sequenceNumber & 0xFF00) >> 8);

		return JAUS_TRUE;
	}
}

static int headerToString(ReportDiscreteDevicesMessage message, char **buf)
{
  //message existance already verified

  //Setup temporary string buffer

  unsigned int bufSize = 500;
  (*buf) = (char*)malloc(sizeof(char)*bufSize);

  strcpy((*buf), jausCommandCodeString(message->commandCode) );
  strcat((*buf), " (0x");
  sprintf((*buf)+strlen(*buf), "%04X", message->commandCode);

  strcat((*buf), ")\nReserved: ");
  jausUnsignedShortToString(message->properties.reserved, (*buf)+strlen(*buf));

  strcat((*buf), "\nVersion: ");
  switch(message->properties.version)
  {
    case 0:
      strcat((*buf), "2.0 and 2.1 compatible");
      break;
    case 1:
      strcat((*buf), "3.0 through 3.1 compatible");
      break;
    case 2:
      strcat((*buf), "3.2 and 3.3 compatible");
      break;
    default:
      strcat((*buf), "Reserved for Future: ");
      jausUnsignedShortToString(message->properties.version, (*buf)+strlen(*buf));
      break;
  }

  strcat((*buf), "\nExp. Flag: ");
  if(message->properties.expFlag == 0)
    strcat((*buf), "Not Experimental");
  else
    strcat((*buf), "Experimental");

  strcat((*buf), "\nSC Flag: ");
  if(message->properties.scFlag == 1)
    strcat((*buf), "Service Connection");
  else
    strcat((*buf), "Not Service Connection");

  strcat((*buf), "\nACK/NAK: ");
  switch(message->properties.ackNak)
  {
  case 0:
    strcat((*buf), "None");
    break;
  case 1:
    strcat((*buf), "Request ack/nak");
    break;
  case 2:
    strcat((*buf), "nak response");
    break;
  case 3:
    strcat((*buf), "ack response");
    break;
  default:
    break;
  }

  strcat((*buf), "\nPriority: ");
  if(message->properties.priority < 12)
  {
    strcat((*buf), "Normal Priority ");
    jausUnsignedShortToString(message->properties.priority, (*buf)+strlen(*buf));
  }
  else
  {
    strcat((*buf), "Safety Critical Priority ");
    jausUnsignedShortToString(message->properties.priority, (*buf)+strlen(*buf));
  }

  strcat((*buf), "\nSource: ");
  jausAddressToString(message->source, (*buf)+strlen(*buf));

  strcat((*buf), "\nDestination: ");
  jausAddressToString(message->destination, (*buf)+strlen(*buf));

  strcat((*buf), "\nData Size: ");
  jausUnsignedIntegerToString(message->dataSize, (*buf)+strlen(*buf));

  strcat((*buf), "\nData Flag: ");
  jausUnsignedIntegerToString(message->dataFlag, (*buf)+strlen(*buf));
  switch(message->dataFlag)
  {
    case 0:
      strcat((*buf), " Only data packet in single-packet stream");
      break;
    case 1:
      strcat((*buf), " First data packet in muti-packet stream");
      break;
    case 2:
      strcat((*buf), " Normal data packet");
      break;
    case 4:
      strcat((*buf), " Retransmitted data packet");
      break;
    case 8:
      strcat((*buf), " Last data packet in stream");
      break;
    default:
      strcat((*buf), " Unrecognized data flag code");
      break;
  }

  strcat((*buf), "\nSequence Number: ");
  jausUnsignedShortToString(message->sequenceNumber, (*buf)+strlen(*buf));

  return (int)strlen(*buf);

}
