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

#include "rosbag_mock_drivers/MockControllerDriver.h"

namespace mock_drivers
{
bool MockControllerDriver::driverDiscovery()
{
  cav_msgs::DriverStatus discovery_msg;  // TODO only publish driver discovery every second.

  discovery_msg.name = "MockControllerDriver";
  discovery_msg.status = 1;

  discovery_msg.can = false;
  discovery_msg.radar = false;
  discovery_msg.gnss = false;
  discovery_msg.lidar = false;
  discovery_msg.roadway_sensor = false;
  discovery_msg.comms = false;
  discovery_msg.controller = true;
  discovery_msg.camera = false;
  discovery_msg.imu = false;
  discovery_msg.trailer_angle_sensor = false;
  discovery_msg.lightbar = false;

  mock_driver_node_.publishDataNoHeader<cav_msgs::DriverStatus>("driver_discovery", discovery_msg);

  cav_msgs::RobotEnabled robot_status;
  robot_status.robot_active = robot_active_;
  robot_status.robot_enabled = robot_enabled_;

  mock_driver_node_.publishDataNoHeader<cav_msgs::RobotEnabled>(robot_status_topic_, robot_status);

  return true;
}

void MockControllerDriver::parserCB(const cav_simulation_msgs::BagData::ConstPtr& msg)
{
  // generate messages from bag data
  if (msg->robot_status_flag)
  {
    cav_msgs::RobotEnabled robot_status = msg->robot_status;
    // publish the data
    mock_driver_node_.publishDataNoHeader<cav_msgs::RobotEnabled>("controller/robot_status", robot_status);
  }
}

void MockControllerDriver::vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& msg)
{
  robot_enabled_ = true;  // If a command was received set the robot enabled status to true;
}

bool MockControllerDriver::enableRoboticSrv(cav_srvs::SetEnableRobotic::Request& req,
                                            cav_srvs::SetEnableRobotic::Response& res)
{
  if (robot_enabled_)
  {
    robot_active_ = true;
  }

  return true;
}

MockControllerDriver::MockControllerDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);

  robot_status_ptr_ =
      boost::make_shared<ROSComms<cav_msgs::RobotEnabled>>(CommTypes::pub, false, 10, robot_status_topic_);

  vehicle_cmd_ptr_ = boost::make_shared<ConstPtrRefROSComms<autoware_msgs::VehicleCmd>>(
      std::bind(&MockControllerDriver::vehicleCmdCallback, this, std::placeholders::_1), CommTypes::sub, false, 10,
      vehicle_cmd_topic_);

  enable_robotic_ptr_ =
      boost::make_shared<ROSComms<cav_srvs::SetEnableRobotic::Request&, cav_srvs::SetEnableRobotic::Response&>>(
          std::bind(&MockControllerDriver::enableRoboticSrv, this, std::placeholders::_1, std::placeholders::_2),
          CommTypes::srv, enable_robotic_srv_);
}

int MockControllerDriver::run()
{
  mock_driver_node_.init();

  // driver publisher, subscriber, and service
  mock_driver_node_.addPub(robot_status_ptr_);
  mock_driver_node_.addSub(vehicle_cmd_ptr_);
  mock_driver_node_.addSrv(enable_robotic_ptr_);

  // driver discovery publisher
  mock_driver_node_.addPub(driver_discovery_pub_ptr_);
  mock_driver_node_.setSpinCallback(std::bind(&MockControllerDriver::driverDiscovery, this));

  mock_driver_node_.spin(20);

  return 0;
}

}  // namespace mock_drivers