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

  SpeedAccelSubscriber_ = nodeHandle_.subscribe("/cmd_speed", 1,
                                      &NewGuidanceCommands::SpeedAccelSubscriberCallback, this);
  WrenchEffortSubscriber_ = nodeHandle_.subscribe("/cmd_longitudinal_effort", 1,
                                      &NewGuidanceCommands::WrenchEffortSubscriberCallback, this);
  LateralControlSubscriber_ = nodeHandle_.subscribe("/cmd_lateral", 1,
                                      &NewGuidanceCommands::LateralControlSubscriberCallback, this);

  SpeedAccelPublisher_ = nodeHandle_.advertise<cav_msgs::SpeedAccel>(SpeedAccelPublisherTopic_, 1);
  WrenchEffortPublisher_ = nodeHandle_.advertise<std_msgs::Float32>(WrenchEffortPublisherTopic_, 1);
  LateralControlPublisher_ = nodeHandle_.advertise<cav_msgs::LateralControl>(LateralControlPublisherTopic_, 1);

  ROS_INFO("Successfully launched node.");
}

NewGuidanceCommands::~NewGuidanceCommands()
{
}

void NewGuidanceCommands::publisher(){
  ROS_DEBUG("Calling publisher functions");
  SpeedAccelPublisher();
  WrenchEffortPublisher();
  LateralControlPublisher();
} 

bool NewGuidanceCommands::readParameters()
{
  nodeHandle_.param<std::string>("SpeedAccelPublisher_topic", SpeedAccelPublisherTopic_, "/republish/cmd_speed");
  nodeHandle_.param<std::string>("WrenchEffortPublisher_topic", WrenchEffortPublisherTopic_, "/republish/cmd_longitudinal_effort");
  nodeHandle_.param<std::string>("LateralControlPublisher_topic", LateralControlPublisherTopic_, "/republish/cmd_lateral");
  nodeHandle_.param("publish_rate", rate, 10);
  nodeHandle_.param("timeout_thresh", timeout, 0.5);

  return true;
}


void NewGuidanceCommands::SpeedAccelSubscriberCallback(const cav_msgs::SpeedAccel::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(SpeedAccelMsgMutex);
    SpeedAccelTimeTracker = ros::Time::now().toSec();
    SpeedAccelMsg = msg;
    ROS_DEBUG("I heard SpeedAccel");
};

void NewGuidanceCommands::WrenchEffortSubscriberCallback(const std_msgs::Float32::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(WrenchEffortMsgMutex);
    WrenchEffortTimeTracker = ros::Time::now().toSec();
    WrenchEffortMsg = msg;
    ROS_DEBUG("I heard wrenchEffort");
};

void NewGuidanceCommands::LateralControlSubscriberCallback(const cav_msgs::LateralControl::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(LateralControlMsgMutex);
    LateralControlTimeTracker = ros::Time::now().toSec();
    LateralControlMsg = msg;
    ROS_DEBUG("I heard lateralControl");
};

void NewGuidanceCommands::SpeedAccelPublisher(){
    std::lock_guard<std::mutex> lock(SpeedAccelMsgMutex);
    if(SpeedAccelMsg != nullptr && (ros::Time::now().toSec() - SpeedAccelTimeTracker) < TimeoutThresh.toSec()) {
      SpeedAccelPublisher_.publish(SpeedAccelMsg);
      ROS_DEBUG("I publish SpeedAccel");
    }
};

void NewGuidanceCommands::WrenchEffortPublisher(){
    std::lock_guard<std::mutex> lock(WrenchEffortMsgMutex);
    if(WrenchEffortMsg != nullptr && (ros::Time::now().toSec() - WrenchEffortTimeTracker) < TimeoutThresh.toSec()) {
      WrenchEffortPublisher_.publish(WrenchEffortMsg);
      ROS_DEBUG("I publish wrenchEffort");
    }
};

void NewGuidanceCommands::LateralControlPublisher(){
    std::lock_guard<std::mutex> lock(LateralControlMsgMutex);
    if(LateralControlMsg != nullptr && (ros::Time::now().toSec() - LateralControlTimeTracker) < TimeoutThresh.toSec()) {
      LateralControlPublisher_.publish(LateralControlMsg);
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