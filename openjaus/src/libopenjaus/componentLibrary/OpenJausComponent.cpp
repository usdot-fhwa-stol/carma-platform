/*****************************************************************************
 *	Copyright (c) 2010, OpenJAUS.com
 *	All rights reserved.
 *
 *	This file is part of OpenJAUS.	OpenJAUS is distributed under the BSD
 *	license.	See the LICENSE file for details.
 *
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions
 *	are met:
 *
 *		 * Redistributions of source code must retain the above copyright
 *			 notice, this list of conditions and the following disclaimer.
 *		 * Redistributions in binary form must reproduce the above
 *			 copyright notice, this list of conditions and the following
 *			 disclaimer in the documentation and/or other materials provided
 *			 with the distribution.
 *		 * Neither the name of the University of Florida nor the names of its
 *			 contributors may be used to endorse or promote products derived from
 *			 this software without specific prior written permission.
 *
 *	 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *	 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *	 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *	 A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *	 OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *	 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *	 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *	 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *	 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *	 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *	 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
// File Name: OpenJausComponent.cpp
//
// Written By: Rob Meyers
//
// Version: 3.3.0b
//
// Date: 02/05/10
//
// Description: This file contains the code associated with the OpenJAUS
//				Component Library. The Component Library is a wrapper around
//				common component functionality which accelerates and eases
//				the creation of JAUS components.

#include "componentLibrary/OpenJausComponent.h"
#include <sstream>

////////////////////////////////////////////////////////////////////////////////////////////////
//// CTOR / DTOR
////////////////////////////////////////////////////////////////////////////////////////////////
OpenJausComponent::OpenJausComponent(const std::string& name, JausByte id, double hz):
	m_cmpt(NULL),
	m_address(NULL)
{
	m_cmpt = ojCmptCreate((char*)(name.c_str()), id, hz);

	if(m_cmpt)
	{
		setUserData(this);
		m_address = ojCmptGetAddress(m_cmpt);
	}
	else
	{
		throw "OpenJausComponent: unable to create \"" + name + "\".	Is ojNodeManager running?\n";
	}

}

OpenJausComponent::OpenJausComponent(OjCmpt cmp):
	m_cmpt(cmp),
	m_address(NULL)
{
	if(m_cmpt)
	{
		setUserData(this);
		m_address = ojCmptGetAddress(m_cmpt);
	}
	else
	{
		throw "OpenJausCompoent: invalid OjCmpt specified\n";
	}
}

OpenJausComponent::~OpenJausComponent()
{
	// The destroy function shutdowns the component
	// This needs to be done before we free any other data that
	// may still be accessed by the component while it is running
	destroy();

	if(m_address)
	{
		jausAddressDestroy(m_address);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////
//// Public Member Functions
////////////////////////////////////////////////////////////////////////////////////////////////
int OpenJausComponent::run()
{
	if(m_cmpt)
	{
		return ojCmptRun(m_cmpt);
	}
	else
	{
		return -1;
	}
}

void OpenJausComponent::destroy()
{
	if(m_cmpt)
	{
		ojCmptDestroy(m_cmpt);
		m_cmpt = NULL;
	}
}

void OpenJausComponent::setFrequencyHz(double frequencyHz)
{
	if(m_cmpt)
	{
		ojCmptSetFrequencyHz(m_cmpt, frequencyHz);
	}
}

void OpenJausComponent::setState(int state)
{
	if(m_cmpt)
	{
		ojCmptSetState(m_cmpt, state);
	}
}

int OpenJausComponent::setStateCallback(int state, void (*stateCallbackFunction)(OjCmpt))
{
	if(m_cmpt)
	{
		return ojCmptSetStateCallback(m_cmpt, state, stateCallbackFunction);
	}
	else
	{
		return -1;
	}
}

void OpenJausComponent::setMessageCallback(unsigned short commandCode, void (*messageFunction)(OjCmpt, JausMessage))
{
	if(m_cmpt)
	{
		ojCmptSetMessageCallback(m_cmpt, commandCode, messageFunction);
	}
}

void OpenJausComponent::setMessageProcessorCallback(void (*processMessageFunction)(OjCmpt, JausMessage))
{
	if(m_cmpt)
	{
		ojCmptSetMessageProcessorCallback(m_cmpt, processMessageFunction);
	}
}

void OpenJausComponent::setUserData(void *data)
{
	if(m_cmpt)
	{
		ojCmptSetUserData(m_cmpt, data);
	}
}

void OpenJausComponent::setAuthority(JausByte authority)
{
	if(m_cmpt)
	{
		ojCmptSetAuthority(m_cmpt, authority);
	}
}

int OpenJausComponent::sendMessage(JausMessage message)
{
	if(m_cmpt)
	{
		return ojCmptSendMessage(m_cmpt, message);
	}
	else
	{
		return -1;
	}
}

int OpenJausComponent::sendMessage(std::shared_ptr<JausMessageStruct> message)
{
	return sendMessage(message.get());
}

JausByte OpenJausComponent::getAuthority()
{
	if(m_cmpt)
	{
		return ojCmptGetAuthority(m_cmpt);
	}
	else
	{
		return 0;
	}
}

JausAddress OpenJausComponent::getAddress()
{
	if(m_cmpt)
	{
		return ojCmptGetAddress(m_cmpt);
	}
	else
	{
		return NULL;
	}
}

JausAddress OpenJausComponent::getControllerAddress()
{
	if(m_cmpt)
	{
		return ojCmptGetControllerAddress(m_cmpt);
	}
	else
	{
		return NULL;
	}
}

JausBoolean OpenJausComponent::hasController()
{
	if(m_cmpt)
	{
		return ojCmptHasController(m_cmpt);
	}
	else
	{
		return JAUS_FALSE;
	}
}

JausState OpenJausComponent::getState()
{
	if(m_cmpt)
	{
		return static_cast<JausState>(ojCmptGetState(m_cmpt));
	}
	else
	{
		return JAUS_UNDEFINED_STATE;
	}
}

void OpenJausComponent::defaultMessageProcessor(JausMessage message)
{
	if(m_cmpt)
	{
		ojCmptDefaultMessageProcessor(m_cmpt, message);
	}
}

char* OpenJausComponent::getName()
{
	if(m_cmpt)
	{
		return ojCmptGetName(m_cmpt);
	}
	else
	{
		return NULL;
	}
}

void* OpenJausComponent::getUserData()
{
	if(m_cmpt)
	{
		return ojCmptGetUserData(m_cmpt);
	}
	else
	{
		return NULL;
	}
}

double OpenJausComponent::getRateHz()
{
	if(m_cmpt)
	{
		return ojCmptGetRateHz(m_cmpt);
	}
	else
	{
		return 0.0;
	}
}

JausBoolean OpenJausComponent::terminateController()
{
	if(m_cmpt)
	{
		return ojCmptTerminateController(m_cmpt);
	}
	else
	{
		return JAUS_FALSE;
	}
}

JausBoolean OpenJausComponent::addService(JausUnsignedShort serviceType)
{
	if(m_cmpt)
	{
		return ojCmptAddService(m_cmpt, serviceType);
	}
	else
	{
		return JAUS_FALSE;
	}
}

JausBoolean OpenJausComponent::addServiceInputMessage(JausUnsignedShort serviceType, JausUnsignedShort commandCode, JausUnsignedInteger presenceVector)
{
	if(m_cmpt)
	{
		return ojCmptAddServiceInputMessage(m_cmpt, serviceType, commandCode, presenceVector);
	}
	else
	{
		return JAUS_FALSE;
	}
}

JausBoolean OpenJausComponent::addServiceOutputMessage(JausUnsignedShort serviceType, JausUnsignedShort commandCode, JausUnsignedInteger presenceVector)
{
	if(m_cmpt)
	{
		return ojCmptAddServiceOutputMessage(m_cmpt, serviceType, commandCode, presenceVector);
	}
	else
	{
		return JAUS_FALSE;
	}
}

int OpenJausComponent::establishSc(JausUnsignedShort cCode, JausUnsignedInteger pv, JausAddress address, double rateHz, double timeoutSec, int qSize)
{
	if(m_cmpt)
	{
		return ojCmptEstablishSc(m_cmpt, cCode, pv, address, rateHz, timeoutSec, qSize);
	}
	else
	{
		return -1;
	}
}

int OpenJausComponent::terminateSc(int scIndex)
{
	if(m_cmpt)
	{
		return ojCmptTerminateSc(m_cmpt, scIndex);
	}
	else
	{
		return -1;
	}
}

JausBoolean OpenJausComponent::isIncomingScActive(int scIndex)
{
	if(m_cmpt)
	{
		return ojCmptIsIncomingScActive(m_cmpt, scIndex);
	}
	else
	{
		return JAUS_FALSE;
	}
}

void OpenJausComponent::addSupportedSc(unsigned short commandCode)
{
	if(m_cmpt)
	{
		ojCmptAddSupportedSc(m_cmpt, commandCode);
	}
}

void OpenJausComponent::removeSupportedSc(unsigned short commandCode)
{
	if(m_cmpt)
	{
		ojCmptRemoveSupportedSc(m_cmpt, commandCode);
	}
}

ServiceConnection OpenJausComponent::getScSendList(unsigned short commandCode)
{
	if(m_cmpt)
	{
		return ojCmptGetScSendList(m_cmpt, commandCode);
	}
	else
	{
		return NULL;
	}
}

void OpenJausComponent::destroySendList(ServiceConnection scList)
{
	if(m_cmpt)
	{
		ojCmptDestroySendList(scList);
	}
}

JausBoolean OpenJausComponent::isOutgoingScActive(unsigned short commandCode)
{
	return ojCmptIsOutgoingScActive(m_cmpt, commandCode);
}

JausBoolean OpenJausComponent::lookupAddress(JausAddress address)
{
	return ojCmptLookupAddress(m_cmpt, address);
}

JausByte OpenJausComponent::id()
{
	if(m_address)
	{
		return m_address->component;
	}
	else
	{
		return 0;
	}
}

OjCmpt OpenJausComponent::component()
{
	return m_cmpt;
}

bool OpenJausComponent::valid()
{
	if(!m_cmpt)
	{
		return false;
	}

	if(!m_address)
	{
		return false;
	}
	else if(!lookupAddress(m_address))
	{
		return false;
	}

	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////
//// Static Public Member Functions
////////////////////////////////////////////////////////////////////////////////////////////////
const std::string OpenJausComponent::stateAsString(int state)
{
	switch(state)
	{
	case JAUS_INVALID_STATE:
		return "JAUS_INVALID_STATE";
	case JAUS_INITIALIZE_STATE:
		return "JAUS_INITIALIZE_STATE";
	case JAUS_READY_STATE:
		return "JAUS_READY_STATE";
	case JAUS_STANDBY_STATE:
		return "JAUS_STANDBY_STATE";
	case JAUS_SHUTDOWN_STATE:
		return "JAUS_SHUTDOWN_STATE";
	case JAUS_FAILURE_STATE:
		return "JAUS_FAILURE_STATE";
	case JAUS_EMERGENCY_STATE:
		return "JAUS_EMERGENCY_STATE";
	default:
		{
			std::ostringstream oss;
			oss << "<unknown: " << state << ">";
			return oss.str();
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////
//// Private Member Functions
////////////////////////////////////////////////////////////////////////////////////////////////
void OpenJausComponent::_unlockUserData()
{
	if(m_cmpt)
	{
		ojCmptUnlockUserData(m_cmpt);
	}
}

void OpenJausComponent::_lockUserData()
{
	if(m_cmpt)
	{
		ojCmptLockUserData(m_cmpt);
	}
}
