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

#include <gmock/gmock.h>
#include "rosbag_mock_drivers/MockRoadwaySensorDriver.h"
#include <cav_simulation_msgs/BagData.h>

namespace mock_drivers
{
TEST(MockRoadwaySensorDriver, Constructor)
{
  MockRoadwaySensorDriver d1;
  MockRoadwaySensorDriver d2(false);
  MockRoadwaySensorDriver d3(true);

  ASSERT_EQ(d1.getMockDriverNode().isDummy(), false);
  ASSERT_EQ(d2.getMockDriverNode().isDummy(), false);
  ASSERT_EQ(d3.getMockDriverNode().isDummy(), true);
}

TEST(MockRoadwaySensorDriver, pubCallbacks)
{
  MockRoadwaySensorDriver d(true);

  // Check for no data behavior
  ros::Time::setNow(ros::Time(0.0));
  cav_simulation_msgs::BagData::ConstPtr test_msg_ptr(new cav_simulation_msgs::BagData());
  ASSERT_TRUE(test_msg_ptr->header.stamp.isZero());

  d.parserCB(test_msg_ptr);

  std::vector<std::string> test_str_vector = d.getMockDriverNode().getTopics();
  std::vector<ros::Time> test_time_vector = d.getMockDriverNode().getTimeStamps();

  ASSERT_EQ(test_str_vector.size(), 0);
  ASSERT_EQ(test_time_vector.size(), 0);

  // Check for nominal data behavior

  ros::Time::setNow(ros::Time(1.0));
  cav_simulation_msgs::BagData msg;
  msg.header.stamp = ros::Time::now();
  msg.detected_objects_flag = true;
  msg.lane_models_flag = true;
  test_msg_ptr = cav_simulation_msgs::BagData::ConstPtr(new cav_simulation_msgs::BagData(msg));
  ASSERT_EQ(test_msg_ptr->header.stamp, ros::Time(1.0));

  d.parserCB(test_msg_ptr);

  test_str_vector = d.getMockDriverNode().getTopics();
  test_time_vector = d.getMockDriverNode().getTimeStamps();

  ASSERT_EQ(test_str_vector.size(), 2);
  ASSERT_EQ(test_str_vector[0], "roadway_sensor/detected_objects");
  ASSERT_EQ(test_str_vector[1], "roadway_sensor/lane_models");

  ASSERT_EQ(test_time_vector.size(), 2);
  ASSERT_EQ(test_time_vector[0], ros::Time::now());
  ASSERT_EQ(test_time_vector[1], ros::Time::now());
}

TEST(MockRoadwaySensorDriver, driver_discovery)
{
  MockRoadwaySensorDriver d(true);

  ASSERT_TRUE(d.driverDiscovery());
}

TEST(MockRoadwaySensorDriver, run)
{
  MockRoadwaySensorDriver d(true);
  ASSERT_EQ(d.run(), 0);
}
}  // namespace mock_drivers