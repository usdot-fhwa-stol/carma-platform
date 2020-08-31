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

#include "rosbag_mock_drivers/MockGNSSDriver.h"

namespace mock_drivers
{
bool MockGNSSDriver::driverDiscovery()
{
  cav_msgs::DriverStatus discovery_msg;

  discovery_msg.name = mock_driver_node_.getGraphName();
  discovery_msg.status = cav_msgs::DriverStatus::OPERATIONAL;

  discovery_msg.can = false;
  discovery_msg.radar = false;
  discovery_msg.gnss = true;
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

MockGNSSDriver::MockGNSSDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);
}

int MockGNSSDriver::run()
{
  mock_driver_node_.init();

  // driver publisher and subscriber
  addPassthroughPub<gps_common::GPSFix>(bag_prefix_ + gnss_fix_fuxed_topic_, gnss_fix_fuxed_topic_, false, 10);

  // driver discovery publisher
  mock_driver_node_.addPub(driver_discovery_pub_ptr_);
  mock_driver_node_.setSpinCallback(std::bind(&MockGNSSDriver::driverDiscovery, this));

  mock_driver_node_.spin(20);

  return 0;
}

}  // namespace mock_drivers