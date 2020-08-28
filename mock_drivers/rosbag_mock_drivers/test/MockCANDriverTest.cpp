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
#include "rosbag_mock_drivers/MockCANDriver.h"
#include <cav_simulation_msgs/BagData.h>

namespace mock_drivers
{
TEST(MockCANDriver, Constructor)
{
  MockCANDriver d1;
  MockCANDriver d2(false);
  MockCANDriver d3(true);

  ASSERT_EQ(d1.getMockDriverNode().isDummy(), false);
  ASSERT_EQ(d2.getMockDriverNode().isDummy(), false);
  ASSERT_EQ(d3.getMockDriverNode().isDummy(), true);
}

TEST(MockCANDriver, pubCallbacks)
{
  MockCANDriver d(true);

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
  msg.acc_engaged_flag = true;
  msg.acceleration_flag = true;
  msg.antilock_brakes_active_flag = true;
  msg.brake_lights_flag = true;
  msg.brake_position_flag = true;
  msg.engine_speed_flag = true;
  msg.fuel_flow_flag = true;
  msg.odometer_flag = true;
  msg.parking_brake_flag = true;
  msg.speed_flag = true;
  msg.stability_ctrl_active_flag = true;
  msg.stability_ctrl_enabled_flag = true;
  msg.steering_wheel_angle_flag = true;
  msg.throttle_position_flag = true;
  msg.traction_ctrl_active_flag = true;
  msg.traction_ctrl_enabled_flag = true;
  msg.transmission_state_flag = true;
  msg.turn_signal_state_flag = true;
  msg.vehicle_twist_flag = true;
  msg.vehicle_status_flag = true;
  msg.velocity_accel_flag = true;
  test_msg_ptr = cav_simulation_msgs::BagData::ConstPtr(new cav_simulation_msgs::BagData(msg));
  ASSERT_EQ(test_msg_ptr->header.stamp, ros::Time(1.0));

  d.parserCB(test_msg_ptr);

  test_str_vector = d.getMockDriverNode().getTopics();
  test_time_vector = d.getMockDriverNode().getTimeStamps();

  ASSERT_EQ(test_str_vector.size(), 21);
  ASSERT_EQ(test_str_vector[0], "can/acc_engaged");
  ASSERT_EQ(test_str_vector[1], "can/acceleration");
  ASSERT_EQ(test_str_vector[2], "can/antilock_brakes_active");
  ASSERT_EQ(test_str_vector[3], "can/brake_lights");
  ASSERT_EQ(test_str_vector[4], "can/brake_position");
  ASSERT_EQ(test_str_vector[5], "can/engine_speed");
  ASSERT_EQ(test_str_vector[6], "can/fuel_flow");
  ASSERT_EQ(test_str_vector[7], "can/odometer");
  ASSERT_EQ(test_str_vector[8], "can/parking_brake");
  ASSERT_EQ(test_str_vector[9], "can/speed");
  ASSERT_EQ(test_str_vector[10], "can/stability_ctrl_active");
  ASSERT_EQ(test_str_vector[11], "can/stability_ctrl_enabled");
  ASSERT_EQ(test_str_vector[12], "can/steering_wheel_angle");
  ASSERT_EQ(test_str_vector[13], "can/throttle_position");
  ASSERT_EQ(test_str_vector[14], "can/traction_ctrl_active");
  ASSERT_EQ(test_str_vector[15], "can/traction_ctrl_enabled");
  ASSERT_EQ(test_str_vector[16], "can/transmission_state");
  ASSERT_EQ(test_str_vector[17], "can/turn_signal_state");
  ASSERT_EQ(test_str_vector[18], "can/vehicle/twist");
  ASSERT_EQ(test_str_vector[19], "can/vehicle_status");
  ASSERT_EQ(test_str_vector[20], "can/velocity_accel");

  ASSERT_EQ(test_time_vector.size(), 3);  // Check the 3 can data types that have headers
  ASSERT_EQ(test_time_vector[0], ros::Time::now());
  ASSERT_EQ(test_time_vector[1], ros::Time::now());
  ASSERT_EQ(test_time_vector[2], ros::Time::now());
}

TEST(MockCANDriver, driver_discovery)
{
  MockCANDriver d(true);

  ASSERT_TRUE(d.driverDiscovery());
}

TEST(MockCANDriver, run)
{
  MockCANDriver d(true);

  ASSERT_EQ(d.run(), 0);
}
}  // namespace mock_drivers