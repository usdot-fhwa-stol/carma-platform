#pragma once
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

#include "bsm_convertor.h"

void BSMConvertor::convertVehicleSize(const j2735_msgs::VehicleSize& in_msg, cav_msgs::VehicleSize& out_msg) {
  out_msg.vehicle_length = in_msg.vehicle_length;
  out_msg.vehicle_width = in_msg.vehicle_width;
}

void BSMConvertor::convertBrakeSystemStatus(const j2735_msgs::BrakeSystemStatus& in_msg, cav_msgs::BrakeSystemStatus& out_msg) {
  out_msg.abs.anti_lock_brake_status = in_msg.abs.anti_lock_brake_status;
  out_msg.auxBrakes.auxiliary_brake_status = in_msg.auxBrakes.auxiliary_brake_status;
  out_msg.brakeBoost.brake_boost_applied = in_msg.brakeBoost.brake_boost_applied;
  out_msg.scs.stability_control_status = in_msg.scs.stability_control_status;
  out_msg.traction.traction_control_status = in_msg.traction.traction_control_status;
  out_msg.wheelBrakes.brake_applied_status = in_msg.wheelBrakes.brake_applied_status;
}

void BSMConvertor::convertAccelerationSet4Way(const j2735_msgs::AccelerationSet4Way& in_msg, cav_msgs::AccelerationSet4Way& out_msg) {
  out_msg.lateral = in_msg.lateral;
  out_msg.longitudinal = in_msg.longitudinal;
  out_msg.vert = in_msg.vert;
  out_msg.yaw_rate = in_msg.yaw_rate;
}

void BSMConvertor::convertTransmissionState(const j2735_msgs::TransmissionState& in_msg, cav_msgs::TransmissionState& out_msg) {
  out_msg.transmission_state = in_msg.transmission_state;
}

void BSMConvertor::convertPositionalAccuracy(const j2735_msgs::PositionalAccuracy& in_msg, cav_msgs::PositionalAccuracy& out_msg) {
  out_msg.orientation = in_msg.orientation;
  out_msg.semiMajor = in_msg.semiMajor;
  out_msg.semiMinor = in_msg.semiMinor;
}

void BSMConvertor::convertCoreData(const j2735_msgs::BSMCoreData& in_msg, cav_msgs::BSMCoreData& out_msg) {
  // Convert basic data
  out_msg.msg_count = in_msg.msg_count;
  out_msg.id = in_msg.id;
  out_msg.sec_mark = in_msg.sec_mark;
  out_msg.latitude = in_msg.latitude;
  out_msg.longitude = in_msg.longitude;
  out_msg.elev = in_msg.elev;
  out_msg.speed = in_msg.speed;
  out_msg.heading = in_msg.heading;
  out_msg.angle = in_msg.angle;
  // Convert nested messages
  convertPositionalAccuracy(in_msg.accuracy, out_msg.accuracy);
  convertTransmissionState(in_msg.transmission, out_msg.transmission);
  convertAccelerationSet4Way(in_msg.accelSet, out_msg.accelSet);
  convertBrakeSystemStatus(in_msg.brakes, out_msg.brakes);
  convertVehicleSize(in_msg.size, out_msg.size);
}


void BSMConvertor::convert(const j2735_msgs::BSM& in_msg, cav_msgs::BSM& out_msg) {
  out_msg.header = in_msg.header;
  convertCoreData(in_msg.core_data, out_msg.core_data);
}

