/*
 * Copyright (C) 2018-2021 LEIDOS.
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
#include <trajectory_utils/conversions/conversions.h>
#include <carma_wm/Geometry.h>
#include <algorithm>


namespace pure_pursuit_wrapper
{
PurePursuitWrapper::PurePursuitWrapper(PurePursuitWrapperConfig config, WaypointPub waypoint_pub, PluginDiscoveryPub plugin_discovery_pub)
  : config_(config), waypoint_pub_(waypoint_pub), plugin_discovery_pub_(plugin_discovery_pub)
{
  plugin_discovery_msg_.name = "Pure Pursuit";
  plugin_discovery_msg_.version_id = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = true;
  plugin_discovery_msg_.type = cav_msgs::Plugin::CONTROL;
  plugin_discovery_msg_.capability = "control_pure_pursuit_plan/plan_controls";
}

bool PurePursuitWrapper::onSpin()
{
  plugin_discovery_pub_(plugin_discovery_msg_);
  return true;
}


void PurePursuitWrapper::trajectoryPlanHandler(const cav_msgs::TrajectoryPlan::ConstPtr& tp)
{
  ROS_DEBUG_STREAM("Received TrajectoryPlanCurrentPosecallback message");

  std::vector<double> times;
  std::vector<double> downtracks;

  std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points = tp->trajectory_points;
  ROS_DEBUG_STREAM("Original Trajectory size:"<<trajectory_points.size());

  trajectory_utils::conversions::trajectory_to_downtrack_time(trajectory_points, &downtracks, &times);

  //detect stopping case
  size_t stopping_index = 0;
  for (size_t i = 1; i < times.size(); i++)
  {
    if (times[i] == times[i - 1]) //if exactly same, it is stopping case
    {
      ROS_DEBUG_STREAM("Detected a stopping case where times is exactly equal: " << times[i-1]);
      ROS_DEBUG_STREAM("And index of that is: " << i << ", where size is: " << times.size());
      stopping_index = i;
      break;
    }
  }

  std::vector<double> speeds;
  trajectory_utils::conversions::time_to_speed(downtracks, times, tp->initial_longitudinal_velocity, &speeds);

  if (speeds.size() != trajectory_points.size())
  {
    throw std::invalid_argument("Speeds and trajectory points sizes do not match");
  }

  for (size_t i = 0; i < speeds.size(); i++) { // Ensure 0 is min speed
    if (stopping_index != 0 && i >= stopping_index - 1)
    {
      speeds[i] = 0.0;  //stopping case
    }
    else
    {
      speeds[i] = std::max(0.0, speeds[i]);
    }
  }

  std::vector<double> lag_speeds = apply_response_lag(speeds, downtracks, config_.vehicle_response_lag); // This call requires that the first speed point be current speed to work as expected

  autoware_msgs::Lane lane;
  lane.header = tp->header;
  std::vector<autoware_msgs::Waypoint> waypoints;
  waypoints.reserve(trajectory_points.size());

  for (int i = 0; i < trajectory_points.size(); i++)
  {
    autoware_msgs::Waypoint wp;

    wp.pose.pose.position.x = trajectory_points[i].x;
    wp.pose.pose.position.y = trajectory_points[i].y;
    wp.twist.twist.linear.x = lag_speeds[i];
    ROS_DEBUG_STREAM("Setting waypoint idx: " << i <<", with planner: << " << trajectory_points[i].planner_plugin_name << ", x: " << trajectory_points[i].x << 
                            ", y: " << trajectory_points[i].y <<
                            ", speed: " << lag_speeds[i]* 2.23694 << "mph");
    waypoints.push_back(wp);
  }

  lane.waypoints = waypoints;
  waypoint_pub_(lane);
};

std::vector<double> PurePursuitWrapper::apply_response_lag(const std::vector<double>& speeds, const std::vector<double> downtracks, double response_lag) const { // Note first speed is assumed to be vehicle speed
  if (speeds.size() != downtracks.size()) {
    throw std::invalid_argument("Speed list and downtrack list are not the same size.");
  }

  std::vector<double> output;
  if (speeds.empty()) {
    return output;
  }

  double lookahead_distance = speeds[0] * response_lag;

  double downtrack_cutoff = downtracks[0] + lookahead_distance;
  size_t lookahead_count = std::lower_bound(downtracks.begin(),downtracks.end(), downtrack_cutoff) - downtracks.begin(); // Use binary search to find lower bound cutoff point
  output = trajectory_utils::shift_by_lookahead(speeds, (unsigned int) lookahead_count);
  return output;
}

std::vector<cav_msgs::TrajectoryPlanPoint> PurePursuitWrapper::remove_repeated_timestamps(const std::vector<cav_msgs::TrajectoryPlanPoint>& traj_points){
  
  std::vector<cav_msgs::TrajectoryPlanPoint> new_traj_points;

  cav_msgs::TrajectoryPlanPoint prev_point;
  bool first = true;

  for(auto point : traj_points){

    if(first){
      first = false;
      prev_point = point;
      new_traj_points.push_back(point);
      continue;
    }

    if(point.target_time != prev_point.target_time){
      new_traj_points.push_back(point);
      prev_point = point;
    }
    else{
      ROS_DEBUG_STREAM("Duplicate point found");
    }
  }

  return new_traj_points;

}

}  // namespace pure_pursuit_wrapper
