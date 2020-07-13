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

#include "pure_pursuit_wrapper/pure_pursuit_wrapper.hpp"

namespace pure_pursuit_wrapper {

PurePursuitWrapper::PurePursuitWrapper(ros::NodeHandle &nodeHandle): nh_(nodeHandle)
{
  if (!ReadParameters())
  {
    ROS_ERROR("Could not read parameters.");
  }

  Initialize();

  ROS_INFO("Successfully launched node.");
}

PurePursuitWrapper::~PurePursuitWrapper() {
}

void PurePursuitWrapper::Initialize() {
  // SystemAlert Subscriber
  system_alert_sub_ = nh_.subscribe("system_alert", 10, &PurePursuitWrapper::SystemAlertHandler, this);
  // SystemAlert Publisher
  system_alert_pub_ = nh_.advertise<cav_msgs::SystemAlert>("system_alert", 10, true);

  // Pose Subscriber
  pose_sub.subscribe(nh_, "current_pose", 1);
  // Trajectory Plan Subscriber
  trajectory_plan_sub.subscribe(nh_, "trajectory_plan", 1);

  // WayPoints Publisher
  way_points_pub_ = nh_.advertise<autoware_msgs::Lane>("final_waypoints", 10, true);
}

bool PurePursuitWrapper::ReadParameters() {
  return true;
}

void PurePursuitWrapper::TrajectoryPlanPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& pose, const cav_msgs::TrajectoryPlan::ConstPtr& tp){
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

void PurePursuitWrapper::SystemAlertHandler(const cav_msgs::SystemAlert::ConstPtr& msg) {
    try {
      ROS_INFO_STREAM("Received SystemAlert message of type: " << msg->type);
      switch(msg->type) {
        case cav_msgs::SystemAlert::SHUTDOWN: 
          Shutdown(); 
          break;
        default:
          break;
      }
    }
    catch(const std::exception& e) {
      HandleException(e);
    }
};

void PurePursuitWrapper::PublisherForWayPoints(autoware_msgs::Lane& msg){
  try {
    ROS_DEBUG_STREAM("Sending WayPoints message.");
    way_points_pub_.publish(msg);
  }
  catch(const std::exception& e) {
    HandleException(e);
  }
};

void PurePursuitWrapper::HandleException(const std::exception& e) {
  ROS_DEBUG("Sending SystemAlert Message");
  cav_msgs::SystemAlert alert_msg;
  alert_msg.type = cav_msgs::SystemAlert::FATAL;
  alert_msg.description = "Uncaught Exception in " + ros::this_node::getName() + " exception: " + e.what();
 
  ROS_ERROR_STREAM(alert_msg.description);
  system_alert_pub_.publish(alert_msg);

  ros::Duration(0.05).sleep();
  Shutdown();
}

void PurePursuitWrapper::Shutdown() {
  std::lock_guard<std::mutex> lock(shutdown_mutex_);
  ROS_WARN_STREAM("Node shutting down");
  shutting_down_ = true;
}

}  // namespace pure_pursuit_wrapper
