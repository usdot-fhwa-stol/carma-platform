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
// File Name: jausInteger.c
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo (galluzzo AT gmail DOT com)
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines all the basic JausFloat funtionality, this should be primarily used through the JausType file and its methods

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "jaus.h"

JausInteger newJausInteger(int val)
{
	return val;
}

JausBoolean jausIntegerFromBuffer(JausInteger *jInteger, unsigned char *buf, unsigned int bufferSizeBytes)
{
  if(bufferSizeBytes < JAUS_INTEGER_SIZE_BYTES)
  {
    return JAUS_FALSE;
  }

  int f = 0.0f;

  jausEndianSafeCopy(&f, buf, JAUS_INTEGER_SIZE_BYTES);
  *jInteger = newJausInteger(f);

  return JAUS_TRUE;
}

JausBoolean jausIntegerToBuffer(JausInteger input, unsigned char *buf, unsigned int bufferSizeBytes)
{
  if(bufferSizeBytes < JAUS_INTEGER_SIZE_BYTES)
  {
    return JAUS_FALSE;
  }

  jausEndianSafeCopy(buf, &input, JAUS_INTEGER_SIZE_BYTES);

  return JAUS_TRUE;
}

double jausIntegerToDouble(JausInteger input, double min, double max)
{
	// BUG: What to do when max < min
	double scaleFactor = (max-min)/JAUS_INTEGER_RANGE;
	double bias = (max+min)/2.0; 
		
	return input*scaleFactor + bias;
}

JausInteger jausIntegerFromDouble(double value, double min, double max)
{
	//limit value between min and max Int values
	double scaleFactor = (max-min)/JAUS_INTEGER_RANGE;
	double bias = (max+min)/2.0;
	
	// limit real number between min and max
	if(value < min) value = min;
	if(value > max) value = max;
		
	// return rounded integer value
	return newJausInteger((int)((value - bias)/scaleFactor + 0.5));
}

int jausIntegerToString(JausInteger number, char *buf)
{
  return sprintf(buf, "%d", number);
}
