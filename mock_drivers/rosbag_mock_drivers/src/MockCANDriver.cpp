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

#include "rosbag_mock_drivers/MockCANDriver.h"

namespace mock_drivers
{
bool MockCANDriver::driverDiscovery()
{
  cav_msgs::DriverStatus discovery_msg;

  discovery_msg.name = mock_driver_node_.getGraphName();
  discovery_msg.status = cav_msgs::DriverStatus::OPERATIONAL;

  discovery_msg.can = true;
  discovery_msg.radar = false;
  discovery_msg.gnss = false;
  discovery_msg.lidar = false;
  discovery_msg.roadway_sensor = false;
  discovery_msg.comms = false;
  discovery_msg.controller = false;
  discovery_msg.camera = false;
  discovery_msg.imu = false;
  discovery_msg.trailer_angle_sensor = false;
  discovery_msg.lightbar = false;

  mock_driver_node_.publishDataNoHeader<cav_msgs::DriverStatus>("driver_discovery", discovery_msg);

  return true;
}

MockCANDriver::MockCANDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);
}

int MockCANDriver::run()
{
  mock_driver_node_.init();

  // data topic publishers
  addPassthroughPubNoHeader<std_msgs::Float64>(bag_prefix_ + brake_position_topic_, brake_position_topic_, false, 10);
  addPassthroughPubNoHeader<std_msgs::Float64>(bag_prefix_ + steering_wheel_angle_topic_, steering_wheel_angle_topic_,
                                               false, 10);
  addPassthroughPubNoHeader<j2735_msgs::TransmissionState>(bag_prefix_ + transmission_state_topic_,
                                                           transmission_state_topic_, false, 10);

  addPassthroughPub<geometry_msgs::TwistStamped>(bag_prefix_ + vehicle_twist, vehicle_twist, false, 10);

  // driver discovery publisher
  mock_driver_node_.addPub(driver_discovery_pub_ptr_);
  mock_driver_node_.setSpinCallback(std::bind(&MockCANDriver::driverDiscovery, this));

  mock_driver_node_.spin(50);

  return 0;
}

}  // namespace mock_drivers