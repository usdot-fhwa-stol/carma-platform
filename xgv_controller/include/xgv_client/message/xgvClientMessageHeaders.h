/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef XGV_CLIENT_MESSAGE_HEADERS_H
#define XGV_CLIENT_MESSAGE_HEADERS_H

#define JAUS_SET_SIGNALS                0xE322
#define JAUS_SET_MOTION_PROFILE         0xE328
#define JAUS_REPORT_CURVATURE           0xE455
#define JAUS_REPORT_ERROR_COUNT         0xE451
#define JAUS_REPORT_SIGNALS             0xE445
#define JAUS_REPORT_VEHICLE_MODE        0xE456
#define JAUS_REPORT_WHEELS_SPEED        0xE42E

//command
#include "command/platform/setMotionProfileMessage.h"
#include "command/platform/setSignalsMessage.h"


//inform
#include "inform/platform/reportCurvatureMessage.h"
#include "inform/platform/reportErrorCountMessage.h"
#include "inform/platform/reportSignalsMessage.h"
#include "inform/platform/reportVehicleModeMessage.h"
#include "inform/platform/reportWheelsSpeedMessage.h"

#endif //XGV_CLIENT_MESSAGE_HEADERS_H