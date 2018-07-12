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
// File Name: jausGeometryPointLLA.c
//
// Written By: Danny Kent (jaus AT dannykent DOT com)
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines the functions of a JausGeometryPointLLA Object

#include <stdlib.h>
#include "jaus.h"

JausGeometryPointLLA jausGeometryPointLLACreate(void)
{
	JausGeometryPointLLA jausGeometryPointLLA;
	
	jausGeometryPointLLA = (JausGeometryPointLLA)malloc(sizeof(JausGeometryPointLLAStruct));
	
	if(jausGeometryPointLLA)
	{
		jausGeometryPointLLA->latitudeRadians = 0.0;
		jausGeometryPointLLA->longitudeRadians = 0.0;
		jausGeometryPointLLA->altitudeMeters = 0.0;
	
		return jausGeometryPointLLA;
	}
	else
		return NULL;
}

void jausGeometryPointLLADestroy(JausGeometryPointLLA jausGeometryPointLLA)
{
	free(jausGeometryPointLLA);
}

char* jausGeometryPointLLAToString(JausGeometryPointLLA point)
{
  char* buf = NULL;
  char* returnBuf = NULL;

  int bufSize = 100;
  buf = (char*)malloc(sizeof(char)*bufSize);
  
  strcpy(buf, "Latitude(radians): ");
  jausDoubleToString(point->latitudeRadians, buf+strlen(buf));
  
  strcat(buf, "\nLongitude(radians): ");
  jausDoubleToString(point->longitudeRadians, buf+strlen(buf));
  
  strcat(buf, "\nAltitude(meters): ");
  jausDoubleToString(point->altitudeMeters, buf+strlen(buf));
  
  returnBuf = (char*)malloc(sizeof(char)*( strlen(buf)+1 ));
  strcpy(returnBuf, buf);
  
  free(buf);
  return returnBuf;
}
