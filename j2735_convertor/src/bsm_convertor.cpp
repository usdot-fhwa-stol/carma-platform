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

// void BSMConvertor::convert(const j2735_msgs::VehicleSize& in_msg, cav_msgs::VehicleSize& out_msg) {
//   // Convert Vehicle Width
//   out_msg.vehicle_width = ValueConvertor::valueJ2735ToCav(in_msg.vehicle_width, units::CM_PER_M,
//     out_msg.presence_vector, cav_msgs::VehicleSize::VEHICLE_WIDTH_AVAILABLE, j2735_msgs::VehicleSize::VEHICLE_WIDTH_UNAVAILABLE);
  
//   // Convert Vehicle length
//   out_msg.vehicle_length = ValueConvertor::valueJ2735ToCav(in_msg.vehicle_length, units::CM_PER_M,
//     out_msg.presence_vector, cav_msgs::VehicleSize::VEHICLE_LENGTH_AVAILABLE, j2735_msgs::VehicleSize::VEHICLE_LENGTH_UNAVAILABLE);
// }

// void BSMConvertor::convert(const j2735_msgs::AccelerationSet4Way& in_msg, cav_msgs::AccelerationSet4Way& out_msg) {
//   // Convert Longitudinal and Lateral
//   out_msg.longitudinal = ValueConvertor::valueJ2735ToCav(in_msg.longitudinal, units::CM_PER_M,
//     out_msg.presence_vector, cav_msgs::AccelerationSet4Way::ACCELERATION_AVAILABLE, j2735_msgs::AccelerationSet4Way::ACCELERATION_UNAVAILABLE);
  
//   out_msg.lateral = ValueConvertor::valueJ2735ToCav(in_msg.lateral, units::CM_PER_M,
//     out_msg.presence_vector, cav_msgs::AccelerationSet4Way::ACCELERATION_AVAILABLE, j2735_msgs::AccelerationSet4Way::ACCELERATION_UNAVAILABLE);
  
//   // Convert Vertical
//   out_msg.vert = ValueConvertor::valueJ2735ToCav(in_msg.vert, units::FIFTIETH_G_PER_M_PER_SEC_SQR,
//     out_msg.presence_vector, cav_msgs::AccelerationSet4Way::ACCELERATION_VERTICAL_AVAILABLE, j2735_msgs::AccelerationSet4Way::ACCELERATION_VERTICAL_UNAVAILABLE);

//   // Convert Yaw Rate
//   out_msg.yaw_rate = ValueConvertor::valueJ2735ToCav(in_msg.yaw_rate, units::CENTI_DEG_PER_DEG,
//     out_msg.presence_vector, cav_msgs::AccelerationSet4Way::YAWRATE_AVAILABLE, j2735_msgs::AccelerationSet4Way::YAWRATE_UNAVAILABLE);
// }

// void BSMConvertor::convert(const j2735_msgs::PositionalAccuracy& in_msg, cav_msgs::PositionalAccuracy& out_msg) {
//   // Convert semiMajor Axis
//   out_msg.semiMajor = ValueConvertor::valueJ2735ToCav(in_msg.semiMajor, units::TWENTIETH_M_PER_M,
//     out_msg.presence_vector, cav_msgs::PositionalAccuracy::ACCURACY_AVAILABLE, j2735_msgs::PositionalAccuracy::ACCURACY_UNAVAILABLE);

//   // Convert semiMinor Axis
//   out_msg.semiMinor = ValueConvertor::valueJ2735ToCav(in_msg.semiMinor, units::TWENTIETH_M_PER_M,
//     out_msg.presence_vector, cav_msgs::PositionalAccuracy::ACCURACY_AVAILABLE, j2735_msgs::PositionalAccuracy::ACCURACY_UNAVAILABLE);
  
//   // Convert Orientation
//   out_msg.orientation = ValueConvertor::valueJ2735ToCav(in_msg.orientation, units::DEG_360_OVER_65535_PER_DEG,
//     out_msg.presence_vector, cav_msgs::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE, j2735_msgs::PositionalAccuracy::ACCURACY_ORIENTATION_UNAVAILABLE);
// }

// void BSMConvertor::convert(const j2735_msgs::BSMCoreData& in_msg, cav_msgs::BSMCoreData& out_msg) {
//   // Convert basic data
//   // Convert basic data
//   out_msg.msg_count = in_msg.msg_count;
//   out_msg.id = in_msg.id;
  
//   // Convert TimeMark
//   out_msg.sec_mark = ValueConvertor::valueJ2735ToCav(in_msg.sec_mark, units::UNCHANGED,
//     out_msg.presence_vector, cav_msgs::BSMCoreData::SEC_MARK_AVAILABLE, j2735_msgs::BSMCoreData::SEC_MARK_UNAVAILABLE);
  
//   // Convert Lat/Lon
//   out_msg.latitude = ValueConvertor::valueJ2735ToCav(in_msg.latitude, units::TENTH_MICRO_DEG_PER_DEG,
//     out_msg.presence_vector, cav_msgs::BSMCoreData::LATITUDE_AVAILABLE, j2735_msgs::BSMCoreData::LATITUDE_UNAVAILABLE);
  
//   out_msg.longitude = ValueConvertor::valueJ2735ToCav(in_msg.longitude, units::TENTH_MICRO_DEG_PER_DEG,
//     out_msg.presence_vector, cav_msgs::BSMCoreData::LONGITUDE_AVAILABLE, j2735_msgs::BSMCoreData::LONGITUDE_UNAVAILABLE);

//   out_msg.elev = valueJ2735ToCav(in_msg.elev, units::DECA_M_PER_M,
//     out_msg.presence_vector, cav_msgs::BSMCoreData::ELEVATION_AVAILABLE, j2735_msgs::BSMCoreData::ELEVATION_UNAVAILABLE);

//   // Convert Speed
//   out_msg.speed = ValueConvertor::valueJ2735ToCav(in_msg.speed, units::FIFTIETH_M_PER_M,
//     out_msg.presence_vector, cav_msgs::BSMCoreData::SPEED_AVAILABLE, j2735_msgs::BSMCoreData::SPEED_UNAVAILABLE);
  
//   // Convert Heading
//   out_msg.heading = ValueConvertor::valueJ2735ToCav(in_msg.heading, units::EIGHTIETH_DEG_PER_DEG,
//     out_msg.presence_vector, cav_msgs::BSMCoreData::HEADING_AVAILABLE, j2735_msgs::BSMCoreData::HEADING_UNAVAILABLE);

//   // Convert Steering Angle
//   out_msg.angle = valueJ2735ToCav(in_msg.angle, units::ONE_AND_A_HALF_DEG_PER_DEG,
//     out_msg.presence_vector, cav_msgs::BSMCoreData::STEER_WHEEL_ANGLE_AVAILABLE, j2735_msgs::BSMCoreData::STEER_WHEEL_ANGLE_UNAVAILABLE);

//   // Convert nested messages
//   out_msg.transmission = in_msg.transmission;
//   out_msg.brakes = in_msg.brakes;
//   convert(in_msg.accuracy, out_msg.accuracy);
//   convert(in_msg.accelSet, out_msg.accelSet);
//   convert(in_msg.size, out_msg.size);
// }

// void BSMConvertor::convert(const j2735_msgs::BSM& in_msg, cav_msgs::BSM& out_msg) {
//   out_msg.header = in_msg.header;
//   convert(in_msg.core_data, out_msg.core_data);
// }















// Convert cav_msgs to j2735_msgs

void BSMConvertor::convert(const cav_msgs::VehicleSize& in_msg, j2735_msgs::VehicleSize& out_msg) {
  // Convert Vehicle Width
  out_msg.vehicle_width = ValueConvertor::valueCavToJ2735<uint16_t>(in_msg.vehicle_width, units::CM_PER_M,
    in_msg.presence_vector, cav_msgs::VehicleSize::VEHICLE_WIDTH_AVAILABLE, j2735_msgs::VehicleSize::VEHICLE_WIDTH_UNAVAILABLE);
  
  // Convert Vehicle length
  out_msg.vehicle_length = ValueConvertor::valueCavToJ2735(in_msg.vehicle_length, units::CM_PER_M,
    in_msg.presence_vector, cav_msgs::VehicleSize::VEHICLE_LENGTH_AVAILABLE, j2735_msgs::VehicleSize::VEHICLE_LENGTH_UNAVAILABLE);
}

void BSMConvertor::convert(const cav_msgs::AccelerationSet4Way& in_msg, j2735_msgs::AccelerationSet4Way& out_msg) {
  // Convert Longitudinal and Lateral
  out_msg.longitudinal = ValueConvertor::valueCavToJ2735(in_msg.longitudinal, units::CM_PER_M,
    in_msg.presence_vector, cav_msgs::AccelerationSet4Way::ACCELERATION_AVAILABLE, j2735_msgs::AccelerationSet4Way::ACCELERATION_UNAVAILABLE);
  
  out_msg.lateral = ValueConvertor::valueCavToJ2735(in_msg.lateral, units::CM_PER_M,
    in_msg.presence_vector, cav_msgs::AccelerationSet4Way::ACCELERATION_AVAILABLE, j2735_msgs::AccelerationSet4Way::ACCELERATION_UNAVAILABLE);
  
  // Convert Vertical
  out_msg.vert = ValueConvertor::valueCavToJ2735(in_msg.vert, units::FIFTIETH_G_PER_M_PER_SEC_SQR,
    in_msg.presence_vector, cav_msgs::AccelerationSet4Way::ACCELERATION_VERTICAL_AVAILABLE, j2735_msgs::AccelerationSet4Way::ACCELERATION_VERTICAL_UNAVAILABLE);

  // Convert Yaw Rate
  out_msg.yaw_rate = ValueConvertor::valueCavToJ2735(in_msg.yaw_rate, units::CENTI_DEG_PER_DEG,
    in_msg.presence_vector, cav_msgs::AccelerationSet4Way::YAWRATE_AVAILABLE, j2735_msgs::AccelerationSet4Way::YAWRATE_UNAVAILABLE);
}

void BSMConvertor::convert(const cav_msgs::PositionalAccuracy& in_msg, j2735_msgs::PositionalAccuracy& out_msg) {
  // Convert semiMajor Axis
  out_msg.semiMajor = ValueConvertor::valueCavToJ2735(in_msg.semiMajor, units::TWENTIETH_M_PER_M,
    in_msg.presence_vector, cav_msgs::PositionalAccuracy::ACCURACY_AVAILABLE, j2735_msgs::PositionalAccuracy::ACCURACY_UNAVAILABLE);

  // Convert semiMinor Axis
  out_msg.semiMinor = ValueConvertor::valueCavToJ2735(in_msg.semiMinor, units::TWENTIETH_M_PER_M,
    in_msg.presence_vector, cav_msgs::PositionalAccuracy::ACCURACY_AVAILABLE, j2735_msgs::PositionalAccuracy::ACCURACY_UNAVAILABLE);
  
  // Convert Orientation
  out_msg.orientation = ValueConvertor::valueCavToJ2735(in_msg.orientation, units::DEG_360_OVER_65535_PER_DEG,
    in_msg.presence_vector, cav_msgs::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE, j2735_msgs::PositionalAccuracy::ACCURACY_ORIENTATION_UNAVAILABLE);
}

void BSMConvertor::convert(const cav_msgs::BSMCoreData& in_msg, j2735_msgs::BSMCoreData& out_msg) {
  // Convert basic data
  out_msg.msg_count = in_msg.msg_count;
  out_msg.id = in_msg.id;
  
  // Convert TimeMark
  out_msg.sec_mark = ValueConvertor::valueCavToJ2735(in_msg.sec_mark, units::UNCHANGED,
    in_msg.presence_vector, cav_msgs::BSMCoreData::SEC_MARK_AVAILABLE, j2735_msgs::BSMCoreData::SEC_MARK_UNAVAILABLE);
  
  // Convert Lat/Lon
  out_msg.latitude = ValueConvertor::valueCavToJ2735(in_msg.latitude, units::TENTH_MICRO_DEG_PER_DEG,
    in_msg.presence_vector, cav_msgs::BSMCoreData::LATITUDE_AVAILABLE, j2735_msgs::BSMCoreData::LATITUDE_UNAVAILABLE);
  
  out_msg.longitude = ValueConvertor::valueCavToJ2735(in_msg.longitude, units::TENTH_MICRO_DEG_PER_DEG,
    in_msg.presence_vector, cav_msgs::BSMCoreData::LONGITUDE_AVAILABLE, j2735_msgs::BSMCoreData::LONGITUDE_UNAVAILABLE);

  out_msg.elev = ValueConvertor::valueCavToJ2735(in_msg.elev, units::DECA_M_PER_M,
    in_msg.presence_vector, cav_msgs::BSMCoreData::ELEVATION_AVAILABLE, j2735_msgs::BSMCoreData::ELEVATION_UNAVAILABLE);

  // Convert Speed
  out_msg.speed = ValueConvertor::valueCavToJ2735(in_msg.speed, units::FIFTIETH_M_PER_M,
    in_msg.presence_vector, cav_msgs::BSMCoreData::SPEED_AVAILABLE, j2735_msgs::BSMCoreData::SPEED_UNAVAILABLE);
  
  // Convert Heading
  out_msg.heading = ValueConvertor::valueCavToJ2735(in_msg.heading, units::EIGHTIETH_DEG_PER_DEG,
    in_msg.presence_vector, cav_msgs::BSMCoreData::HEADING_AVAILABLE, j2735_msgs::BSMCoreData::HEADING_UNAVAILABLE);

  // Convert Steering Angle
  out_msg.angle = ValueConvertor::valueCavToJ2735(in_msg.angle, units::ONE_AND_A_HALF_DEG_PER_DEG,
    in_msg.presence_vector, cav_msgs::BSMCoreData::STEER_WHEEL_ANGLE_AVAILABLE, j2735_msgs::BSMCoreData::STEER_WHEEL_ANGLE_UNAVAILABLE);

  // Convert nested messages
  out_msg.transmission = in_msg.transmission;
  out_msg.brakes = in_msg.brakes;
  convert(in_msg.accuracy, out_msg.accuracy);
  convert(in_msg.accelSet, out_msg.accelSet);
  convert(in_msg.size, out_msg.size);
}

void BSMConvertor::convert(const j2735_msgs::BSM& in_msg, cav_msgs::BSM& out_msg) {
  out_msg.header = in_msg.header;
  convert(in_msg.core_data, out_msg.core_data);
}

