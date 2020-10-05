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
#include <carma_wm/Geometry.h>

namespace mpc_follower_wrapper {

MPCFollowerWrapper::MPCFollowerWrapper(ros::CARMANodeHandle &nodeHandle): nh_(nodeHandle)
{

  Initialize();

  ROS_INFO("Successfully launched node.");
}

MPCFollowerWrapper::~MPCFollowerWrapper() {
}

std::vector<geometry_msgs::Quaternion>
compute_orientations(const cav_msgs::TrajectoryPlan::ConstPtr& tp) const
{
  std::vector<geometry_msgs::Quaternion> out;

  if (tp->trajectory_points.size() == 0) {
      return out;
  }

  lanelet::BasicLineString2d centerline;
  for (auto traj_point : tp->trajectory_points) {
    lanelet::BasicPoint2d p(traj_point.x, traj_point.y);
    centerline.push_back(p);
  }

  std::vector<Eigen::Vector2d> tangents = carma_wm::geometry::compute_finite_differences(centerline);

  Eigen::Vector2d x_axis = { 1, 0 };
  for (int i = 0; i < tangents.size(); i++)
  {
    geometry_msgs::Quaternion q;

    // Derive angle by cos theta = (u . v)/(||u| * ||v||)
    double yaw = carma_wm::geometry::safeAcos(tangents[i].dot(x_axis) / (tangents[i].norm() * x_axis.norm()));

    q = tf::createQuaternionMsgFromYaw(yaw);
    out.push_back(q);
  }

  return out;
}

void MPCFollowerWrapper::Initialize() {


  // Trajectory Plan Subscriber
  
  trajectory_plan_sub = nh_.subscribe<cav_msgs::TrajectoryPlan>("mpc_follower/trajectory", 1, &MPCFollowerWrapper::TrajectoryPlanPoseHandler, this);

  // WayPoints Publisher
  way_points_pub_ = nh_.advertise<autoware_msgs::Lane>("final_waypoints", 10, true);

  mpc_plugin_discovery_pub_ = nh_.advertise<cav_msgs::Plugin>("plugin_discovery", 1);
  plugin_discovery_msg_.name = "MPC";
  plugin_discovery_msg_.versionId = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = true;
  plugin_discovery_msg_.type = cav_msgs::Plugin::CONTROL;
  plugin_discovery_msg_.capability = "control_mpc_plan/plan_controls";

  ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
  mpc_plugin_discovery_pub_.publish(plugin_discovery_msg_);
  return true;
  });
}


void MPCFollowerWrapper::TrajectoryPlanPoseHandler(const cav_msgs::TrajectoryPlan::ConstPtr& tp){
  ROS_DEBUG_STREAM("Received TrajectoryPlanCurrentPosecallback message");
    try {
      autoware_msgs::Lane lane;
      lane.header = tp->header;
      std::vector <autoware_msgs::Waypoint> waypoints;
      std::vector<geometry_msgs::Quaternion> quats = compute_orientations(tp);
      if (quats.size() != tp->trajectory_points.size()) {
        ROS_ERROR_STREAM("Quat list size mismatch");
      }
      for(int i = 0; i < tp->trajectory_points.size() - 1; i++ ) {

        cav_msgs::TrajectoryPlanPoint t1 = tp->trajectory_points[i];
        cav_msgs::TrajectoryPlanPoint t2 = tp->trajectory_points[i + 1];
        autoware_msgs::Waypoint waypoint = mpcww.TrajectoryPlanPointToWaypointConverter(t1, t2);
        waypoint.pose.pose.orientation = quats[i];
        waypoints.push_back(waypoint);
      }

      lane.waypoints = waypoints;
      PublisherForWayPoints(lane);
    }
    catch(const std::exception& e) {
      ros::CARMANodeHandle::handleException(e);
    }

};


void MPCFollowerWrapper::PublisherForWayPoints(const autoware_msgs::Lane& msg){
  way_points_pub_.publish(msg);
};

}  // namespace mpc_follower_wrapper
