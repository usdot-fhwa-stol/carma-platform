/*
 * Copyright (C) 2020-2021 LEIDOS.
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

//head files required by USF controller driver
#include <ros/ros.h>
#include "rosbag_mock_drivers/VehicleCmd.h"

#include <iostream>
#include <string>
#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
//head files required by USF controller driver

#include "rosbag_mock_drivers/MockControllerDriver.h"

namespace mock_drivers
{
std::vector<DriverType> MockControllerDriver::getDriverTypes()
{
  return { DriverType::CONTROLLER };
}

uint8_t MockControllerDriver::getDriverStatus()
{
  return cav_msgs::DriverStatus::OPERATIONAL;
}

//have a new vehicle command callback function below
/*void MockControllerDriver::vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& msg)
{
  robot_enabled_ = true;  // If a command was received set the robot enabled status to true
}*/

bool MockControllerDriver::enableRoboticSrv(const cav_srvs::SetEnableRobotic::Request& req,
                                            cav_srvs::SetEnableRobotic::Response& res)
{
  if (robot_enabled_ && req.set == cav_srvs::SetEnableRobotic::Request::ENABLE)
  {
    robot_active_ = true;
  }
  else
  {
    robot_active_ = false;
  }

  return true;
}

MockControllerDriver::MockControllerDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);

  robot_status_ptr_ =
      boost::make_shared<ROSComms<cav_msgs::RobotEnabled>>(CommTypes::pub, false, 10, robot_status_topic_);
  
  ////vehicle command topic is subscribed by the vehiclesender node.

  /*vehicle_cmd_ptr_ = boost::make_shared<ConstPtrRefROSComms<autoware_msgs::VehicleCmd>>(
      std::bind(&MockControllerDriver::vehicleCmdCallback, this, std::placeholders::_1), CommTypes::sub, false, 10,
      vehicle_cmd_topic_);*/

  enable_robotic_ptr_ =
      boost::make_shared<ROSComms<cav_srvs::SetEnableRobotic::Request&, cav_srvs::SetEnableRobotic::Response&>>(
          std::bind(&MockControllerDriver::enableRoboticSrv, this, std::placeholders::_1, std::placeholders::_2),
          CommTypes::srv, enable_robotic_srv_);
}

bool MockControllerDriver::onSpin()
{
  cav_msgs::RobotEnabled robot_status;
  robot_status.robot_active = robot_active_;
  robot_status.robot_enabled = robot_enabled_;

  mock_driver_node_.publishDataNoHeader<cav_msgs::RobotEnabled>(robot_status_topic_, robot_status);

  return true;
}

unsigned int MockControllerDriver::getRate()
{
  return 20;  // 20 Hz as default spin rate to match expected controller data rate
}

//USF controller message structure
struct CommandData {
  double linear_x;
  double angular_z;

  void reset();
};

void CommandData::reset()
{
  linear_x      = 0;
  angular_z     = 0;
}

static CommandData command_data;

static void vehicleCmdCallback(const autoware_msgs::VehicleCmd& msg)
{
  command_data.linear_x = msg.twist_cmd.twist.linear.x;
  command_data.angular_z = msg.twist_cmd.twist.angular.z;
}

static void *sendCommand(void *arg)
{
  int *client_sockp = static_cast<int*>(arg);
  int client_sock = *client_sockp;
  delete client_sockp;

  std::ostringstream oss;
  oss << command_data.linear_x << ",";
  oss << command_data.angular_z;

  std::string cmd(oss.str());
  ssize_t n = write(client_sock, cmd.c_str(), cmd.size());
  if(n < 0){
    std::perror("write");
    return nullptr;
  }

  if(close(client_sock) == -1){
    std::perror("close");
    return nullptr;
  }

  std::cout << "cmd: " << cmd << ", size: " << cmd.size() << std::endl;
  return nullptr;
}

static void* receiverCaller(void *unused)
{
  constexpr int listen_port = 10001;

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if(sock == -1){
    std::perror("socket");
    return nullptr;
  }

  sockaddr_in addr;
  sockaddr_in client;
  socklen_t len = sizeof(client);

  std::memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = PF_INET;
  addr.sin_port = htons(listen_port);
  addr.sin_addr.s_addr = INADDR_ANY;

  int ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
  if(ret == -1){
    std::perror("bind");
    goto error;
  }

  ret = listen(sock, 20);
  if(ret == -1){
    std::perror("listen");
    goto error;
  }
  while(true){
    std::cout << "Waiting access..." << std::endl;

    int *client_sock = new int();
    *client_sock = accept(sock, reinterpret_cast<sockaddr*>(&client), &len);
    if(*client_sock == -1){
      std::perror("accept");
      break;
    }

    std::cout << "get connect." << std::endl;

    pthread_t th;
    if(pthread_create(&th, nullptr, sendCommand, static_cast<void*>(client_sock)) != 0){
      std::perror("pthread_create");
      break;
    }

    if(pthread_detach(th) != 0){
      std::perror("pthread_detach");
      break;
    }
  }

  error:
  close(sock);
  return nullptr;
}

int MockControllerDriver::onRun()
{
  // driver publisher, subscriber, and service
  mock_driver_node_.addPub(robot_status_ptr_);
  //mock_driver_node_.addSub(vehicle_cmd_ptr_); //vehicle command topic will be subscribed later.
  mock_driver_node_.addSrv(enable_robotic_ptr_);
  MockControllerDriver::robot_enabled_ = true; //always set robot_enabled to be true
  
  
  //USF customized DBW control codes
  int argc;
  char **argv;

  ros::init(argc, argv, "vehicle_sender"); //define a node to receive control command and send it to DBW using TCP/IP
  ros::NodeHandle nh;

  std::cout << "vehicle sender" << std::endl;


  ros::Subscriber sub = nh.subscribe("/hardware_interface/vehicle_cmd", 1, vehicleCmdCallback);
  

  command_data.reset();

  pthread_t th;
  if(pthread_create(&th, nullptr, receiverCaller, nullptr) != 0){
    std::perror("pthread_create");
    std::exit(1);
  }

  if (pthread_detach(th) != 0){
    std::perror("pthread_detach");
    std::exit(1);
  }
  ros::spin();
  //USF customized DBW control codes

  return 0;
}

}  // namespace mock_drivers