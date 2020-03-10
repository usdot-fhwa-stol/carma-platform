/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#include "mpc_follower_wrapper/mpc_follower_wrapper.hpp"

namespace mpc_follower_wrapper {

MPCFollowerWrapper::MPCFollowerWrapper(ros::NodeHandle &nodeHandle): nh_(nodeHandle)
{
  if (!ReadParameters())
  {
    ROS_ERROR("Could not read parameters.");
  }

  Initialize();

  ROS_INFO("Successfully launched node.");
}

MPCFollowerWrapper::~MPCFollowerWrapper() {
}

void MPCFollowerWrapper::Initialize() {
  // SystemAlert Subscriber
  system_alert_sub_ = nh_.subscribe("system_alert", 10, &MPCFollowerWrapper::SystemAlertHandler, this);
  // SystemAlert Publisher
  system_alert_pub_ = nh_.advertise<cav_msgs::SystemAlert>("system_alert", 10, true);

  // Pose Subscriber
  pose_sub.subscribe(nh_, "current_pose", 1);
  // Trajectory Plan Subscriber
  trajectory_plan_sub.subscribe(nh_, "trajectory_plan", 1);

  // WayPoints Publisher
  way_points_pub_ = nh_.advertise<autoware_msgs::Lane>("final_waypoints", 10, true);
}

bool MPCFollowerWrapper::ReadParameters() {
  return true;
}

void MPCFollowerWrapper::TrajectoryPlanPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& pose, const cav_msgs::TrajectoryPlan::ConstPtr& tp){
  ROS_DEBUG_STREAM("Received TrajectoryPlanCurrentPosecallback message");
    try {
      autoware_msgs::Lane lane;
      lane.header = tp->header;
      std::vector <autoware_msgs::Waypoint> waypoints;
      double current_time = ros::Time::now().toSec();
      for(int i = 0; i < tp->trajectory_points.size() - 1; i++ ) {

        cav_msgs::TrajectoryPlanPoint t1 = tp->trajectory_points[i];
        cav_msgs::TrajectoryPlanPoint t2 = tp->trajectory_points[i + 1];
        autoware_msgs::Waypoint waypoint = ppww.TrajectoryPlanPointToWaypointConverter(current_time, *pose,t1, t2);
        waypoints.push_back(waypoint);
      }

      lane.waypoints = waypoints;
      PublisherForWayPoints(lane);
    }
    catch(const std::exception& e) {
      HandleException(e);
    }

};

void MPCFollowerWrapper::SystemAlertHandler(const cav_msgs::SystemAlert::ConstPtr& msg) {
    try {
      ROS_INFO_STREAM("Received SystemAlert message of type: " << msg->type);
      switch(msg->type) {
        case cav_msgs::SystemAlert::SHUTDOWN: 
          Shutdown(); 
          break;
      }
    }
    catch(const std::exception& e) {
      HandleException(e);
    }
};

void MPCFollowerWrapper::PublisherForWayPoints(autoware_msgs::Lane& msg){
  try {
    ROS_DEBUG_STREAM("Sending WayPoints message.");
    way_points_pub_.publish(msg);
  }
  catch(const std::exception& e) {
    HandleException(e);
  }
};

void MPCFollowerWrapper::HandleException(const std::exception& e) {
  ROS_DEBUG("Sending SystemAlert Message");
  cav_msgs::SystemAlert alert_msg;
  alert_msg.type = cav_msgs::SystemAlert::FATAL;
  alert_msg.description = "Uncaught Exception in " + ros::this_node::getName() + " exception: " + e.what();
 
  ROS_ERROR_STREAM(alert_msg.description);
  system_alert_pub_.publish(alert_msg);

  ros::Duration(0.05).sleep();
  Shutdown();
}

void MPCFollowerWrapper::Shutdown() {
  std::lock_guard<std::mutex> lock(shutdown_mutex_);
  ROS_WARN_STREAM("Node shutting down");
  shutting_down_ = true;
}

}  // namespace mpc_follower_wrapper
