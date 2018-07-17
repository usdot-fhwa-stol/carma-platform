/*****************************************************************************
 *  Copyright (c) 2012, lcad.inf.ufes.br; 2009, OpenJAUS.com
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
// File Name: setMotionProfileMessage.c
//
// Written By: Alberto F. De Souza
//
// Version: 0.0.1
//
// Date: 02/21/2012
//
// Description: This file implements the communication funcionality of the XGV 

#include <xgv_client/xgv_client.h>

#include <stdio.h>
#include <stdlib.h>

static const int commandCode = JAUS_SET_MOTION_PROFILE;
static const int maxDataSizeBytes = 12;

static JausBoolean headerFromBuffer(SetMotionProfileMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static JausBoolean headerToBuffer(SetMotionProfileMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static int headerToString(SetMotionProfileMessage message, char **buf);

static JausBoolean dataFromBuffer(SetMotionProfileMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static int dataToBuffer(SetMotionProfileMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static void dataInitialize(SetMotionProfileMessage message);
static void dataDestroy(SetMotionProfileMessage message);
static unsigned int dataSize(SetMotionProfileMessage message);

// ************************************************************************************************************** //
//                                    USER CONFIGURED FUNCTIONS
// ************************************************************************************************************** //

// Initializes the message-specific fields
static void dataInitialize(SetMotionProfileMessage message)
{
    message->properties.expFlag = JAUS_EXPERIMENTAL_MESSAGE;
    
    message->version = newJausByte(1);
    message->numberOfMotions = newJausByte(1);
    message->desiredVelocity = newJausDouble(0.0);
    message->maximumAcceleration = newJausDouble(0.0);
    message->atanOfDesiredCurvature = newJausDouble(0.0);
    message->rateOfChangeOfCurvature = newJausDouble(0.0);
    message->timeDuration = newJausUnsignedShort(0);
}

// Destructs the message-specific fields
static void dataDestroy(SetMotionProfileMessage message)
{
    // Free message fields
    // Nothing to free
}

// Return boolean of success
static JausBoolean dataFromBuffer(SetMotionProfileMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
    int index = 0;
    JausByte tempByte = 0;
    JausUnsignedShort tempUnsignedShort = 0;

    if (bufferSizeBytes == message->dataSize)
    {
        if (!jausByteFromBuffer(&tempByte, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        message->version = tempByte;
        index += JAUS_BYTE_SIZE_BYTES;
        
        if (!jausByteFromBuffer(&tempByte, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        message->numberOfMotions = tempByte;
        index += JAUS_BYTE_SIZE_BYTES;
        
        if (!jausUnsignedShortFromBuffer(&tempUnsignedShort, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        message->desiredVelocity = jausUnsignedShortToDouble(tempUnsignedShort, -65.535, 65.535);
        index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
        
        if (!jausUnsignedShortFromBuffer(&tempUnsignedShort, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        message->maximumAcceleration = jausUnsignedShortToDouble(tempUnsignedShort, 0.0, 100.0);
        index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
        
        if (!jausUnsignedShortFromBuffer(&tempUnsignedShort, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        message->atanOfDesiredCurvature = jausUnsignedShortToDouble(tempUnsignedShort, -JAUS_HALF_PI, JAUS_HALF_PI);
        index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
        
        if (!jausUnsignedShortFromBuffer(&tempUnsignedShort, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        message->rateOfChangeOfCurvature = jausUnsignedShortToDouble(tempUnsignedShort, 0.0, 10.0);
        index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
        
        if (!jausUnsignedShortFromBuffer(&tempUnsignedShort, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        message->timeDuration = tempUnsignedShort;

        return JAUS_TRUE;
    }
    else
    {
        return JAUS_FALSE;
    }
}

// Returns number of bytes put into the buffer
static int dataToBuffer(SetMotionProfileMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
    int index = 0;
    JausByte tempByte = 0;
    JausUnsignedShort tempUnsignedShort = 0;
    
    if (bufferSizeBytes >= dataSize(message))
    {
        tempByte = message->version;
        if (!jausByteToBuffer(tempByte, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;
        
        tempByte = message->numberOfMotions;
        if (!jausByteToBuffer(tempByte, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;
        
        tempUnsignedShort = jausUnsignedShortFromDouble(message->desiredVelocity, -65.535, 65.535);
        if (!jausUnsignedShortToBuffer(tempUnsignedShort, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
        
        tempUnsignedShort = jausUnsignedShortFromDouble(message->maximumAcceleration, 0.0, 100.0);
        if (!jausUnsignedShortToBuffer(tempUnsignedShort, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
        
        tempUnsignedShort = jausUnsignedShortFromDouble(message->atanOfDesiredCurvature, -JAUS_HALF_PI, JAUS_HALF_PI);
        if (!jausUnsignedShortToBuffer(tempUnsignedShort, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
        
        tempUnsignedShort = jausUnsignedShortFromDouble(message->rateOfChangeOfCurvature, 0.0, 10.0);
        if (!jausUnsignedShortToBuffer(tempUnsignedShort, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
        
        tempUnsignedShort = message->timeDuration;
        if (!jausUnsignedShortToBuffer(tempUnsignedShort, buffer+index, bufferSizeBytes-index)) 
            return JAUS_FALSE;
        index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
    }

    return index;
}

static int dataToString(SetMotionProfileMessage message, char **buf)
{
  //message already verified 

  char temp[1024];
  
  (*buf) = (char*)malloc(sizeof(char)*1024);
  
  strcpy((*buf), "\nSet Motion Profile:" );
  
  strcpy((*buf), "\nVersion: " );
  jausByteToString(message->version, temp);
  strcat((*buf), temp);
  
  strcpy((*buf), "\nNumber of Motions: " );
  jausByteToString(message->numberOfMotions, temp);
  strcat((*buf), temp);
  
  strcpy((*buf), "\nDesired Velocity: " );
  jausDoubleToString(message->desiredVelocity, temp);
  strcat((*buf), temp);
  
  strcpy((*buf), "\nMaximum Acceleration: " );
  jausDoubleToString(message->maximumAcceleration, temp);
  strcat((*buf), temp);
  
  strcpy((*buf), "\nArctanget of Desired Curvature: " );
  jausDoubleToString(message->atanOfDesiredCurvature, temp);
  strcat((*buf), temp);
  
  strcpy((*buf), "\nRate of Change of Curvature: " );
  jausDoubleToString(message->rateOfChangeOfCurvature, temp);
  strcat((*buf), temp);
  
  strcpy((*buf), "\nTime Duration (ms): " );
  jausUnsignedShortToString(message->timeDuration, temp);
  strcat((*buf), temp);
  
  return strlen((*buf));
}

// Returns number of bytes put into the buffer
static unsigned int dataSize(SetMotionProfileMessage message)
{
    int index = 0;

    index += JAUS_BYTE_SIZE_BYTES;
    index += JAUS_BYTE_SIZE_BYTES;
    index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
    index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
    index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
    index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
    index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
    
    return index;
}

// ************************************************************************************************************** //
//                                    NON-USER CONFIGURED FUNCTIONS
// ************************************************************************************************************** //

SetMotionProfileMessage setMotionProfileMessageCreate(void)
{
    SetMotionProfileMessage message;

    message = (SetMotionProfileMessage)malloc( sizeof(SetMotionProfileMessageStruct) );
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

void setMotionProfileMessageDestroy(SetMotionProfileMessage message)
{
    dataDestroy(message);
    jausAddressDestroy(message->source);
    jausAddressDestroy(message->destination);
    free(message);
}

JausBoolean setMotionProfileMessageFromBuffer(SetMotionProfileMessage message, unsigned char* buffer, unsigned int bufferSizeBytes)
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

JausBoolean setMotionProfileMessageToBuffer(SetMotionProfileMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
    if(bufferSizeBytes < setMotionProfileMessageSize(message))
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
            return JAUS_FALSE; // headerToSetMotionProfileBuffer failed
        }
    }
}

SetMotionProfileMessage setMotionProfileMessageFromJausMessage(JausMessage jausMessage)
{
    SetMotionProfileMessage message;
    
    if(jausMessage->commandCode != commandCode)
    {
        return NULL; // Wrong message type
    }
    else
    {
        message = (SetMotionProfileMessage)malloc( sizeof(SetMotionProfileMessageStruct) );
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

JausMessage setMotionProfileMessageToJausMessage(SetMotionProfileMessage message)
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


unsigned int setMotionProfileMessageSize(SetMotionProfileMessage message)
{
    return (unsigned int)(dataSize(message) + JAUS_HEADER_SIZE_BYTES);
}

char* setMotionProfileMessageToString(SetMotionProfileMessage message)
{
  if(message)
  {
    char* buf1 = NULL;
    char* buf2 = NULL;
    
    int returnVal;
    
    //Print the message header to the string buffer
    returnVal = headerToString(message, &buf1);
    
    //Print the message data fields to the string buffer
    returnVal += dataToString(message, &buf2);
    
    char* buf;
    buf = (char*)malloc(strlen(buf1)+strlen(buf2));
    strcpy(buf, buf1);
    strcat(buf, buf2);
    
    free(buf1);
    free(buf2);
    
    return buf;
  }
  else
  {
    char* buf = "Invalid SetMotionProfile Message";
    char* msg = (char*)malloc(strlen(buf)+1);
    strcpy(msg, buf);
    return msg;
  }
}
//********************* PRIVATE HEADER FUNCTIONS **********************//

static JausBoolean headerFromBuffer(SetMotionProfileMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
    if(bufferSizeBytes < JAUS_HEADER_SIZE_BYTES)
    {
        return JAUS_FALSE;
    }
    else
    {
        // unpack header
        message->properties.priority = (buffer[0] & 0x0F);
        message->properties.ackNak   = ((buffer[0] >> 4) & 0x03);
        message->properties.scFlag   = ((buffer[0] >> 6) & 0x01);
        message->properties.expFlag  = ((buffer[0] >> 7) & 0x01);
        message->properties.version  = (buffer[1] & 0x3F);
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

static JausBoolean headerToBuffer(SetMotionProfileMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
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

static int headerToString(SetMotionProfileMessage message, char **buf)
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
  
  return strlen((*buf));
  

}
