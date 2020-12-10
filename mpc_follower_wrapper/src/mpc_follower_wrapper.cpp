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

MPCFollowerWrapper::MPCFollowerWrapper(ros::CARMANodeHandle &nodeHandle): nh_(nodeHandle)
{

  Initialize();

  ROS_INFO("Successfully launched node.");
}

MPCFollowerWrapper::~MPCFollowerWrapper() {
}

void MPCFollowerWrapper::Initialize() {


  // Trajectory Plan Subscriber
  
  trajectory_plan_sub = nh_.subscribe<cav_msgs::TrajectoryPlan>("mpc_follower/trajectory", 1, &MPCFollowerWrapper::TrajectoryPlanPoseHandler, this);

  // WayPoints Publisher
  way_points_pub_ = nh_.advertise<autoware_msgs::Lane>("final_waypoints", 10, true);
}


void MPCFollowerWrapper::TrajectoryPlanPoseHandler(const cav_msgs::TrajectoryPlan::ConstPtr& tp){
  ROS_DEBUG_STREAM("Received TrajectoryPlanCurrentPosecallback message");
    try {
      autoware_msgs::Lane lane;
      lane.header = tp->header;
      std::vector <autoware_msgs::Waypoint> waypoints;
      for(int i = 0; i < tp->trajectory_points.size() - 1; i++ ) {

        cav_msgs::TrajectoryPlanPoint t1 = tp->trajectory_points[i];
        cav_msgs::TrajectoryPlanPoint t2 = tp->trajectory_points[i + 1];
        autoware_msgs::Waypoint waypoint = mpcww.TrajectoryPlanPointToWaypointConverter(t1, t2);
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
