/*****************************************************************************
 *  Copyright (c) 2010, OpenJAUS.com
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
// File Name: OpenJausComponent.h
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

#ifndef OPEN_JAUS_COMPONENT_H
#define OPEN_JAUS_COMPONENT_H

#include <jaus.h>
#include "nodeManagerInterface/nodeManagerInterface.h"
#include "componentLibrary/ojCmpt.h"
#include <string>
#include <memory>
class OpenJausComponent
{
public: // Public Member Functions
	OpenJausComponent(const std::string& name, JausByte id, double hz);
	OpenJausComponent(OjCmpt cmp);

	virtual ~OpenJausComponent();

	operator OjCmpt()
	{
		return m_cmpt;
	}

	int run();
	void destroy();

	void setFrequencyHz(double frequencyHz);
	void setState(int state);
	int setStateCallback(int state, void (*stateCallbackFunction)(OjCmpt));      // Calls method from stateHandler
	void setMessageCallback(unsigned short commandCode, void (*messageFunction)(OjCmpt, JausMessage)); // Calls method from messageHandler
	void setMessageProcessorCallback(void (*processMessageFunction)(OjCmpt, JausMessage)); // Calls method from messageHandler
	void setUserData(void *data);
	void setAuthority(JausByte authority);
	int sendMessage(JausMessage message);
	int sendMessage(std::shared_ptr<JausMessageStruct> message);
	// Accessors
	JausByte getAuthority();
	JausAddress getAddress();
	JausAddress getControllerAddress();
	JausBoolean hasController();
	JausState getState();
	void defaultMessageProcessor(JausMessage message);
	char* getName();
	void* getUserData();
	double getRateHz();

	// Component Control
	JausBoolean terminateController();

	// Services
	JausBoolean addService(JausUnsignedShort serviceType);
	JausBoolean addServiceInputMessage(JausUnsignedShort serviceType, JausUnsignedShort commandCode, JausUnsignedInteger presenceVector);
	JausBoolean addServiceOutputMessage(JausUnsignedShort serviceType, JausUnsignedShort commandCode, JausUnsignedInteger presenceVector);

	// Incoming Service Connections
	int establishSc(JausUnsignedShort cCode, JausUnsignedInteger pv, JausAddress address, double rateHz, double timeoutSec, int qSize);
	int terminateSc(int scIndex);
	JausBoolean isIncomingScActive(int scIndex);

	// Outgoing Service Connections
	void addSupportedSc(unsigned short commandCode);   // add service connection support for this message
	void removeSupportedSc(unsigned short commandCode);  // Removes service connection support for this message
	ServiceConnection getScSendList(unsigned short commandCode);
	void destroySendList(ServiceConnection scList);
	JausBoolean isOutgoingScActive(unsigned short commandCode);

	// System Discovery
	JausBoolean lookupAddress(JausAddress address);

	JausByte id();
	OjCmpt component();

	virtual bool valid();

public: // Static Public Member functions
	static const std::string stateAsString(int state);

private: // Private Member Functions
	void _unlockUserData();
	void _lockUserData();

private: // Private Member Data
	OjCmpt m_cmpt;

protected: // Protected Member Data
	JausAddress m_address;
};

typedef void (*Jaus_void_OjCmpt_Func)(OjCmpt);
typedef void (*Jaus_void_OjCmpt_JausMessage_Func)(OjCmpt, JausMessage);

// For Functions with argument list void(OjCmpt)
template <class HandlerClass, void(HandlerClass::*HandlerFunc)(OjCmpt)>
struct Jaus_void_OjCmpt_Proxy
{
  static void HandlerProxy(OjCmpt a1)
  {
    HandlerClass* p = (HandlerClass*)ojCmptGetUserData(a1);
    (static_cast<HandlerClass*>(p)->*HandlerFunc)(a1);
  }
};

// For Functions with argument list void(OjCmpt)
template <class HandlerClass, void(HandlerClass::*HandlerFunc)(OjCmpt, JausMessage)>
struct Jaus_void_OjCmpt_JausMessage_Proxy
{
  static void HandlerProxy(OjCmpt a1, JausMessage a2)
  {
    HandlerClass* p = (HandlerClass*)ojCmptGetUserData(a1);
    (static_cast<HandlerClass*>(p)->*HandlerFunc)(a1,a2);
  }
};

#define JAUS_STATE_METHOD(Class, Handler) \
   (Jaus_void_OjCmpt_Func)Jaus_void_OjCmpt_Proxy<Class, &Class::Handler>::HandlerProxy

#define JAUS_MESSAGE_METHOD(Class, Handler) \
   (Jaus_void_OjCmpt_JausMessage_Func)Jaus_void_OjCmpt_JausMessage_Proxy<Class, &Class::Handler>::HandlerProxy

// Example macro usage
// setStateCallback(JAUS_READY_STATE, JAUS_STATE_METHOD(LocalPoseSensor, _readyStateSlot));

#endif // OPEN_JAUS_COMPONENT_H
