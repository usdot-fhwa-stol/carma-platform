/*
 * Copyright (C) 2018-2019 LEIDOS.
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

#include "new_guidance_commands/NewGuidanceCommands.hpp"

namespace new_guidance_commands {

NewGuidanceCommands::NewGuidanceCommands(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
  }

  InitTimeTracker();

  speedAccelsubscriber_ = nodeHandle_.subscribe("/cmd_speed", 1,
                                      &NewGuidanceCommands::speedAccel_SubscriberCallback, this);
  wrenchEffortsubscriber_ = nodeHandle_.subscribe("/cmd_longitudinal_effort", 1,
                                      &NewGuidanceCommands::wrenchEffort_SubscriberCallback, this);
  lateralControlsubscriber_ = nodeHandle_.subscribe("/cmd_lateral", 1,
                                      &NewGuidanceCommands::lateralControl_SubscriberCallback, this);

  speedAccel_publisher_ = nodeHandle_.advertise<cav_msgs::SpeedAccel>(speedAccel_publisherTopic_, 1);
  wrenchEffort_publisher_ = nodeHandle_.advertise<std_msgs::Float32>(wrenchEffort_publisherTopic_, 1);
  lateralControl_publisher_ = nodeHandle_.advertise<cav_msgs::LateralControl>(lateralControl_publisherTopic_, 1);

  ROS_INFO("Successfully launched node.");
}

NewGuidanceCommands::~NewGuidanceCommands()
{
}

void NewGuidanceCommands::publisher(){
  ROS_DEBUG("Calling publisher functions");
  speedAccel_Publisher();
  wrenchEffort_Publisher();
  lateralControl_Publisher();
} 

bool NewGuidanceCommands::readParameters()
{
  nodeHandle_.param<std::string>("speedAccel_publisher_topic", speedAccel_publisherTopic_, "/republish/cmd_speed");
  nodeHandle_.param<std::string>("wrenchEffort_publisher_topic", wrenchEffort_publisherTopic_, "/republish/cmd_longitudinal_effort");
  nodeHandle_.param<std::string>("lateralControl_publisher_topic", lateralControl_publisherTopic_, "/republish/cmd_lateral");
  nodeHandle_.param("publish_rate", rate, 10);
  nodeHandle_.param("timeout_thresh", timeout, 0.5);

  return true;
}


void NewGuidanceCommands::speedAccel_SubscriberCallback(const cav_msgs::SpeedAccel::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(SpeedAccel_msg_mutex);
    SpeedAccelTimeTracker = ros::Time::now().toSec();
    SpeedAccel_msg = msg;
    ROS_DEBUG("I heard SpeedAccel");
};

void NewGuidanceCommands::wrenchEffort_SubscriberCallback(const std_msgs::Float32::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(WrenchEffort_msg_mutex);
    WrenchEffortTimeTracker = ros::Time::now().toSec();
    WrenchEffort_msg = msg;
    ROS_DEBUG("I heard wrenchEffort");
};

void NewGuidanceCommands::lateralControl_SubscriberCallback(const cav_msgs::LateralControl::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(LateralControl_msg_mutex);
    LateralControlTimeTracker = ros::Time::now().toSec();
    LateralControl_msg = msg;
    ROS_DEBUG("I heard lateralControl");
};

void NewGuidanceCommands::speedAccel_Publisher(){
    std::lock_guard<std::mutex> lock(SpeedAccel_msg_mutex);
    if(SpeedAccel_msg != nullptr && (ros::Time::now().toSec() - SpeedAccelTimeTracker) < TimeoutThresh.toSec()) {
      speedAccel_publisher_.publish(SpeedAccel_msg);
      ROS_DEBUG("I publish SpeedAccel");
    }
};

void NewGuidanceCommands::wrenchEffort_Publisher(){
    std::lock_guard<std::mutex> lock(WrenchEffort_msg_mutex);
    if(WrenchEffort_msg != nullptr && (ros::Time::now().toSec() - WrenchEffortTimeTracker) < TimeoutThresh.toSec()) {
      wrenchEffort_publisher_.publish(WrenchEffort_msg);
      ROS_DEBUG("I publish wrenchEffort");
    }
};

void NewGuidanceCommands::lateralControl_Publisher(){
    std::lock_guard<std::mutex> lock(LateralControl_msg_mutex);
    if(LateralControl_msg != nullptr && (ros::Time::now().toSec() - LateralControlTimeTracker) < TimeoutThresh.toSec()) {
      lateralControl_publisher_.publish(LateralControl_msg);
      ROS_DEBUG("I publish lateralControl");
    }
};

void NewGuidanceCommands::InitTimeTracker(){
  SpeedAccelTimeTracker = 0;
  WrenchEffortTimeTracker = 0;
  LateralControlTimeTracker = 0;
  TimeoutThresh = ros::Duration(timeout);
}

} // namespace new_guidance_commands