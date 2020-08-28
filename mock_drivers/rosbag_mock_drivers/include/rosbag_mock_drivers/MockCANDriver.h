/*
 * Copyright (C) 2019-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#pragma once

#include "rosbag_mock_drivers/MockDriver.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <j2735_msgs/TransmissionState.h>
#include <cav_msgs/TurnSignal.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleStatus.h>
#include <automotive_platform_msgs/VelocityAccel.h>

namespace mock_drivers
{
class MockCANDriver : public MockDriver
{
private:
    steering_wheel_angle_pub_ = nh_->advertise<std_msgs::Float64>("can/steering_wheel_angle", 1);
    brake_position_pub_ = nh_->advertise<std_msgs::Float64>("can/brake_position", 1);
    transmission_pub_ = nh_->advertise<j2735_msgs::TransmissionState>("can/transmission_state", 1);
  const std::string acc_engaged_topic_ = "acc_engaged";
  const std::string acceleration_topic_ = "acceleration";
  const std::string antilock_brakes_topic_ = "antilock_brakes_active";
  const std::string brake_position_topic_ = "can/brake_position";
  const std::string engine_speed_topic_ = "engine_speed";
  const std::string fuel_flow_topic_ = "fuel_flow";
  const std::string odometer_topic_ = "odometer";
  const std::string parking_brake_topic_ = "parking_brake";
  const std::string speed_topic_ = "speed";
  const std::string stability_ctrl_active_topic_ = "stability_ctrl_active";
  const std::string stability_ctrl_enabled_topic_ = "stability_ctrl_enabled";
  const std::string steering_wheel_angle_topic_ = "can/steering_wheel_angle";
  const std::string throttle_position_topic_ = "throttle_position";
  const std::string traction_ctrl_active_topic_ = "traction_ctrl_active";
  const std::string traction_ctrl_enabled_topic_ = "traction_ctrl_enabled";
  const std::string transmission_state_topic_ = "can/transmission_state";
  const std::string turn_signal_state_topic_ = "turn_signal_state";
  const std::string vehicle_twist = "vehicle/twist";
  const std::string vehicle_status_topic_ = "vehicle_status";
  const std::string velocity_accel_topic_ = "velocity_accel";

public:
  MockCANDriver(bool dummy = false);
  int run();
  bool driverDiscovery();
};

}  // namespace mock_drivers