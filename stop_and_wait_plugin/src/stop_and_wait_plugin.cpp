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

#include <ros/ros.h>
#include <string>
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <sstream>
#include <carma_utils/containers/containers.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <unordered_set>
#include "stop_and_wait_plugin.h"
#include <vector>
#include <cav_msgs/Trajectory.h>
#include <cav_msgs/StopAndWaitManeuver.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_utils/containers/containers.h>
#include <carma_wm/Geometry.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <math.h>
#include <std_msgs/Float64.h>

using oss = std::ostringstream;

namespace stop_and_wait_plugin
{
StopandWait::StopandWait(carma_wm::WorldModelConstPtr wm, StopandWaitConfig config,
                         PublishPluginDiscoveryCB plugin_discovery_publisher)
  : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher){};

bool StopandWait::spinCallback()
{
  cav_msgs::Plugin plugin_discovery_msg_;
  plugin_discovery_msg_.name = "StopandWaitPlugin";
  plugin_discovery_msg_.versionId = "v1.1";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = false;
  plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
  plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";

  plugin_discovery_publisher_(plugin_discovery_msg_);
  return true;
}

bool StopandWait::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
{
  lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
  ROS_DEBUG_STREAM("planning state x:" << req.vehicle_state.X_pos_global << ", y: " << req.vehicle_state.Y_pos_global);
  double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;
  ROS_DEBUG_STREAM("Starting stop&wait planning");
  ROS_DEBUG_STREAM("Current_downtrack" << current_downtrack);
  std::vector<cav_msgs::Maneuver> maneuver_plan;
  for (const auto maneuver : req.maneuver_plan.maneuvers)
  {
    if (maneuver.type == cav_msgs::Maneuver::STOP_AND_WAIT &&
        maneuver.stop_and_wait_maneuver.end_dist > current_downtrack)  // Only process maneuvers beyond our starting
                                                                       // point
    {
      maneuver_plan.push_back(maneuver);
    }
  }

  if (maneuver_plan.size() == 0)
  {
    throw std::invalid_argument("No valid maneuvers in plan provided to stop and wait plugin.");
  }

  std::vector<PointSpeedPair> points_and_target_speeds = maneuvers_to_points(maneuver_plan, wm_, req.vehicle_state;);

  // Trajectory plan
  cav_msgs::TrajectoryPlan trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = ros::Time::now();
  trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

  trajectory.trajectory_points =
      compose_trajectory_from_centerline(points_and_target_speeds, req.vehicle_state, req.header.stamp);
  ROS_DEBUG_STREAM("Trajectory points size:" << trajectory.trajectory_points.size());
  trajectory.initial_longitudinal_velocity = req.vehicle_state.longitudinal_vel;
  resp.trajectory_plan = trajectory;
  resp.related_maneuvers.push_back(cav_msgs::Maneuver::STOP_AND_WAIT);
  resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

  return true;
}

// Returns the centerline points and speed limits for the provided maneuver
std::vector<PointSpeedPair> StopandWait::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                             const carma_wm::WorldModelConstPtr& wm,
                                                             const cav_msgs::VehicleState& state)
{
  std::vector<PointSpeedPair> points_and_target_speeds;
  std::unordered_set<lanelet::Id> visited_lanelets;

  if (maneuvers.size() > 1)
  {
    ROS_WARN_STREAM("StopAndWait plugin can only plan one stopping maenuver at a time");
  }

  if (maneuvers[0].type != cav_msgs::Maneuver::STOP_AND_WAIT)
  {
    throw std::invalid_argument("Stop and Wait Maneuver Plugin doesn't support this maneuver type");
  }

  cav_msgs::StopAndWaitManeuver stop_and_wait_maneuver = maneuvers[0].stop_and_wait_maneuver;

  lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
  double starting_downtrack = wm_->routeTrackPos(veh_pos).downtrack;  // The vehicle position
  double starting_speed = req.vehicle_state.longitudinal_vel;

  ROS_DEBUG_STREAM("Used starting downtrack: " << starting_downtrack);

  auto lanelets = wm->getLaneletsBetween(starting_downtrack, stop_and_wait_maneuver.end_dist, true);

  lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm->getTrafficRules();

  if (!traffic_rules)
  {
    throw std::invalid_argument("Traffic rules could not be optained from carma_wm");
  }

  lanelet::BasicLineString2d downsampled_centerline;
  downsampled_centerline.reserve(400);

  for (const auto& l : lanelets)
  {
    ROS_DEBUG_STREAM("Lanelet ID: " << l.id());

    lanelet::BasicLineString2d centerline = l.centerline2d().basicLineString();
    lanelet::BasicLineString2d downsampled_points;
    downsampled_points.reserve((centerline.size() / 4) + 2) downsampled_points =
        carma_utils::containers::downsample_vector(centerline, downsample_ratio_);  // Downsample centerline

    downsampled_centerline = carma_wm::geometry::concatenate_line_strings(downsampled_centerline, downsampled_points);

    bool loop_first = true;
    for (auto p : downsampled_centerline)
    {
      if (loop_first && points_and_target_speeds.size() != 0)
      {
        loop_first = false;
        continue;  // Skip the first point if we have already added points from a previous maneuver to avoid duplicates
      }
      PointSpeedPair pair;
      pair.point = p;
      pair.speed = (*traffic_rules)->speedLimit(llt).speedLimit.value();
      pair.lanelet_id = l.id();
      points_and_target_speeds.push_back(pair);
    }
  }

  return points_and_target_speeds;
}

std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_from_points_times_orientations(
    const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& times, const std::vector<double>& yaws,
    ros::Time startTime)
{
  if (points.size() != times.size() || points.size() != yaws.size())
  {
    throw std::invalid_argument("All input vectors must have the same size");
  }

  std::vector<cav_msgs::TrajectoryPlanPoint> traj;
  traj.reserve(points.size());

  for (int i = 0; i < points.size(); i++)
  {
    cav_msgs::TrajectoryPlanPoint tpp;
    ros::Duration relative_time(times[i]);
    tpp.target_time = startTime + relative_time;
    tpp.x = points[i].x();
    tpp.y = points[i].y();
    tpp.yaw = yaws[i];

    tpp.controller_plugin_name = "default";
    tpp.planner_plugin_name = plugin_discovery_msg_.name;

    traj.push_back(tpp);
  }

  return traj;
}
// TODO how are we going to handle crawling to the stop line if stopped before?
std::vector<cav_msgs::TrajectoryPlanPoint>
StopandWait::compose_trajectory_from_centerline(const std::vector<PointSpeedPair>& points, double starting_downtrack,
                                                double starting_speed, double stop_location, ros::Time start_time)
{
  std::vector<cav_msgs::TrajectoryPlanPoint> plan;
  if (points.size() == 0)
  {
    ROS_WARN_STREAM("No points to use as trajectory in stop and wait plugin");
    return plan;
  }

  std::vector<PointSpeedPair> final_points;

  double remaining_distance = stop_location - starting_downtrack;
  double target_accel = config_.accel_limit_multiplier * config_.accel_limit;
  double req_dist =
      starting_speed / (2.0 * target_accel);  // Distance needed to go from current speed to 0 at target accel

  if (req_dist > remaining_distance)
  {
    ROS_DEBUG_STREAM("Target Accel Update From: " << target_accel);
    target_accel = starting_speed / (2.0 * remaining_distance);  // If we cannot reach the end point it the required
                                                                 // distance update the accel target
    ROS_DEBUG_STREAM("Target Accel Update To: " << target_accel);
  }

  final_points.reserve(points.size());

  PointSpeedPair prev_pair = points.back();
  prev_pair.speed = 0.0;
  final_points.push_back(prev_pair);  // Store the points in reverse

  bool reached_end = false;
  for (int i = points.size() - 2; i > 0; i--)
  {  // NOTE: Do not use size_t for i type here as -- with > 0 will result in overflow
     //       First point's speed is left unchanged as it is current speed of the vehicle
    double v_i = prev_pair.speed;

    if (v_i >= starting_speed)
    {  // We are walking backward, so if the prev speed is greater than or equal to the starting speed then we are done
       // backtracking
      reached_end = true;
      PointSpeedPair pair = points[i];
      pair.speed = starting_speed;
      final_points.push_back(pair);  // Store the points in reverse
      prev_pair = pair;
      continue;  // continue until loop end
    }

    double dx = lanelet::geometry::distance2d(prev_pair.point, points[i].point);

    double v_f = sqrt(v_i * v_i + 2 * target_accel * dx);

    PointSpeedPair pair = points[i];
    pair.speed = std::min(v_f, starting_speed);
    final_points.push_back(pair);  // Store the points in reverse

    prev_pair = pair;
  }

  // Now we have a trajectory that decelerates from our end point to somewhere in the maneuver
  std::reverse(final_points.begin(), final_points.end());

  std::vector<double> speeds;
  std::vector<lanelet::BasicPoint2d> raw_points;
  splitPointSpeedPairs(points, &raw_points, &speeds);

  std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(raw_points);

  std::vector<double> times;
  trajectory_utils::conversions::speed_to_time(downtracks, speeds, &times);

  std::vector<double> yaws = carma_wm::geometry::compute_tangent_orientations(raw_points);

  auto traj = trajectory_from_points_times_orientations(raw_points, times, yaws, start_time);

  for (size_t i = 0; i < traj.size(); i++)
  {
    traj[i].lane_id = std::to_string(points[i].lanelet_id);
  }

  while (traj.back().target_time - traj.front().target_time < ros::Duration(config_.minimal_trajectory_duration))
  {
    cav_msgs::TrajectoryPlanPoint new_point = traj.back();
    new_point.target_time = new_point.target_time + ros::Duration(config_.stop_timestep);
    traj.push_back(new_point);
  }

  return traj;
}

void StopandWait::splitPointSpeedPairs(const std::vector<PointSpeedPair>& points,
                                       std::vector<lanelet::BasicPoint2d>* basic_points,
                                       std::vector<double>* speeds) const
{
  basic_points->reserve(points.size());
  speeds->reserve(points.size());

  for (const auto& p : points)
  {
    basic_points->push_back(p.point);
    speeds->push_back(p.speed);
  }
}

}  // namespace stop_and_wait_plugin