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

#include "rosbag_mock_drivers/MockRoadwaySensorDriver.h"

namespace mock_drivers
{
std::vector<DriverType> MockRoadwaySensorDriver::getDriverTypes()
{
  return { DriverType::ROADWAY_SENSOR };
}

uint8_t MockRoadwaySensorDriver::getDriverStatus()
{
  return cav_msgs::DriverStatus::OPERATIONAL;
}

MockRoadwaySensorDriver::MockRoadwaySensorDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);
}

int MockRoadwaySensorDriver::run()
{
  mock_driver_node_.init();

  // driver publishers
  addPassthroughPub<derived_object_msgs::ObjectWithCovariance>(bag_prefix_ + detected_objects_topic_,
                                                               detected_objects_topic_, false, 10);
  addPassthroughPub<derived_object_msgs::LaneModels>(bag_prefix_ + lane_models_topics_, lane_models_topics_, false, 10);

  // driver discovery publisher
  mock_driver_node_.addPub(driver_discovery_pub_ptr_);
  mock_driver_node_.setSpinCallback(std::bind(&MockRoadwaySensorDriver::spinCallback, this));

  mock_driver_node_.spin(20);

  return 0;
}

}  // namespace mock_drivers