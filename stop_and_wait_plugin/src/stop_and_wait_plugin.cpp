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

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.hpp>
#include <trajectory_utils/conversions/conversions.hpp>
#include <sstream>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <unordered_set>
#include "stop_and_wait_plugin.hpp"
#include <vector>
#include <carma_planning_msgs/msg/stop_and_wait_maneuver.hpp>
#include <carma_wm_ros2/Geometry.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <math.h>
#include <std_msgs/msg/float64.hpp>
#include <math.h>

using oss = std::ostringstream;

namespace stop_and_wait_plugin
{

StopandWait::StopandWait(std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh, 
                                          carma_wm::WorldModelConstPtr wm, 
                                          const StopandWaitConfig& config, 
                                          const std::string& plugin_name,
                                          const std::string& version_id)
  : version_id_ (version_id),plugin_name_(plugin_name),config_(config),nh_(nh), wm_(wm)
{};

bool StopandWait::plan_trajectory_cb(carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
{

  RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Starting stop&wait planning");

  if (req->maneuver_index_to_plan >= req->maneuver_plan.maneuvers.size())
  {
    throw std::invalid_argument(
        "StopAndWait plugin asked to plan invalid maneuver index: " + std::to_string(req->maneuver_index_to_plan) +
        " for plan of size: " + std::to_string(req->maneuver_plan.maneuvers.size()));
  }

  if (req->maneuver_plan.maneuvers[req->maneuver_index_to_plan].type != carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT)
  {
    throw std::invalid_argument("StopAndWait plugin asked to plan non STOP_AND_WAIT maneuver");
  }

  lanelet::BasicPoint2d veh_pos(req->vehicle_state.x_pos_global, req->vehicle_state.y_pos_global);

  RCLCPP_DEBUG_STREAM(nh_->get_logger(),"planning state x:" << req->vehicle_state.x_pos_global << ", y: " << req->vehicle_state.y_pos_global << ", speed: " << req->vehicle_state.longitudinal_vel);

  if (req->vehicle_state.longitudinal_vel < epsilon_)
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Detected that car is already stopped! Ignoring the request to plan Stop&Wait");
     
    return true;
  }

  double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

  RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Current_downtrack" << current_downtrack);

  if (req->maneuver_plan.maneuvers[req->maneuver_index_to_plan].stop_and_wait_maneuver.end_dist < current_downtrack)
  {
    throw std::invalid_argument("StopAndWait plugin asked to plan maneuver that ends earlier than the current state.");
  }

  resp->related_maneuvers.push_back(req->maneuver_index_to_plan);
  resp->maneuver_status.push_back(carma_planning_msgs::srv::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

  std::string maneuver_id = req->maneuver_plan.maneuvers[req->maneuver_index_to_plan].stop_and_wait_maneuver.parameters.maneuver_id;

  RCLCPP_INFO_STREAM(nh_->get_logger(),"Maneuver not yet planned, planning new trajectory");

  // Maneuver input is valid so continue with execution
  std::vector<carma_planning_msgs::msg::Maneuver> maneuver_plan = { req->maneuver_plan.maneuvers[req->maneuver_index_to_plan] };

  std::vector<PointSpeedPair> points_and_target_speeds = maneuvers_to_points(
      maneuver_plan, wm_, req->vehicle_state);  // Now have 1m downsampled points from cur to endpoint

  // Trajectory plan
  carma_planning_msgs::msg::TrajectoryPlan trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = req->header.stamp;
  trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

  // Extract the stopping buffer used to consider a stopping behavior complete
  double stop_location_buffer = config_.default_stopping_buffer;  // If no maneuver meta data is provided we will use the default buffer
  
  double stopping_accel = 0.0;
  if (maneuver_plan[0].stop_and_wait_maneuver.parameters.presence_vector &
      carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA)
  {
    if(maneuver_plan[0].stop_and_wait_maneuver.parameters.float_valued_meta_data.size() < 2){
      throw std::invalid_argument("stop and wait maneuver message missing required meta data");
    }
    stop_location_buffer = maneuver_plan[0].stop_and_wait_maneuver.parameters.float_valued_meta_data[0];
    stopping_accel = maneuver_plan[0].stop_and_wait_maneuver.parameters.float_valued_meta_data[1];

    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Using stop buffer from meta data: " << stop_location_buffer);
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Using stopping acceleration from meta data: "<< stopping_accel);
  }
  else{
    throw std::invalid_argument("stop and wait maneuver message missing required float meta data");
  }

  double initial_speed = req->vehicle_state.longitudinal_vel; //will be modified after compose_trajectory_from_centerline 

  trajectory.trajectory_points = compose_trajectory_from_centerline(
      points_and_target_speeds, current_downtrack, req->vehicle_state.longitudinal_vel,
      maneuver_plan[0].stop_and_wait_maneuver.end_dist, stop_location_buffer, req->header.stamp, stopping_accel, initial_speed);

  RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Trajectory points size:" << trajectory.trajectory_points.size());

  trajectory.initial_longitudinal_velocity = initial_speed;

  resp->trajectory_plan = trajectory;

  return true;
}

// Returns the centerline points and speed limits for the provided maneuver
std::vector<PointSpeedPair> StopandWait::maneuvers_to_points(const std::vector<carma_planning_msgs::msg::Maneuver>& maneuvers,
                                                             const carma_wm::WorldModelConstPtr& wm,
                                                             const carma_planning_msgs::msg::VehicleState& state)
{
  std::vector<PointSpeedPair> points_and_target_speeds;
  std::unordered_set<lanelet::Id> visited_lanelets;

  carma_planning_msgs::msg::StopAndWaitManeuver stop_and_wait_maneuver = maneuvers[0].stop_and_wait_maneuver;

  lanelet::BasicPoint2d veh_pos(state.x_pos_global, state.y_pos_global);
  double starting_downtrack = wm_->routeTrackPos(veh_pos).downtrack;  // The vehicle position
  double starting_speed = state.longitudinal_vel;

  // Sample the lanelet centerline at fixed increments.
  // std::min call here is a guard against starting_downtrack being within 1m of the maneuver end_dist
  // in this case the sampleRoutePoints method will return a single point allowing execution to continue
  std::vector<lanelet::BasicPoint2d> route_points = wm->sampleRoutePoints(
      std::min(starting_downtrack + config_.centerline_sampling_spacing, stop_and_wait_maneuver.end_dist),
      stop_and_wait_maneuver.end_dist, config_.centerline_sampling_spacing);


  route_points.insert(route_points.begin(), veh_pos);


  for (const auto& p : route_points)
  {
    PointSpeedPair pair;
    pair.point = p;
    pair.speed = starting_speed; // NOTE: Since the vehicle is trying to stop the assumption made is that the speed limit is irrelevant. 
    points_and_target_speeds.push_back(pair);
  }

  return points_and_target_speeds;
}

std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> StopandWait::trajectory_from_points_times_orientations(
    const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& times, const std::vector<double>& yaws,
    rclcpp::Time startTime)
{
  if (points.size() != times.size() || points.size() != yaws.size())
  {
    throw std::invalid_argument("All input vectors must have the same size");
  }

  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> traj;
  traj.reserve(points.size());

  for (size_t i = 0; i < points.size(); i++)
  {
    carma_planning_msgs::msg::TrajectoryPlanPoint tpp;
    rclcpp::Duration relative_time(times[i] * 1e9);
    tpp.target_time = startTime + relative_time;
    tpp.x = points[i].x();
    tpp.y = points[i].y();
    tpp.yaw = yaws[i];
    tpp.planner_plugin_name = plugin_name_;
    tpp.controller_plugin_name = "default";
 
    traj.push_back(tpp);
  }

  return traj;
}

std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> StopandWait::compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, double starting_downtrack, double starting_speed, double stop_location,
    double stop_location_buffer, rclcpp::Time start_time, double stopping_acceleration, double& initial_speed)
{
  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> plan;
  if (points.size() == 0)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(),"No points to use as trajectory in stop and wait plugin");
    return plan;
  }

  std::vector<PointSpeedPair> final_points;

  double half_stopping_buffer = stop_location_buffer * 0.5;
  double remaining_distance = stop_location - half_stopping_buffer - starting_downtrack; // Target to stop in the middle of the buffer

  double target_accel = stopping_acceleration * config_.accel_limit_multiplier;
  double req_dist = (starting_speed * starting_speed) /
                    (2.0 * target_accel);  // Distance needed to go from current speed to 0 at target accel

  // In cases where the vehicle is not able to stop before the half_stopping_buffer the remaining distance becomes negative
  // which will cause the target accel in this loop to be negative and make the vehicle speed up
  while (remaining_distance <= 0.0)
  {
    if(remaining_distance <= (stop_location - starting_downtrack))
    {
    //Add additional distance to remaining to allow vehicle to stop within buffer
      remaining_distance += 0.2;
    }
    else{
     break;
   }
  }
  
  if (req_dist > remaining_distance)
  {

    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Target Accel Update From: " << target_accel);
    target_accel =
        (starting_speed * starting_speed) / (2.0 * remaining_distance);  // If we cannot reach the end point it the
                                                                         // required distance update the accel target
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Target Accel Update To: " << target_accel);
  }


  std::vector<double> inverse_downtracks; // Store downtracks in reverse
  inverse_downtracks.reserve(points.size());
  final_points.reserve(points.size());

  PointSpeedPair prev_pair = points.back();
  prev_pair.speed = 0.0;
  final_points.push_back(prev_pair);  // Store the points in reverse
  inverse_downtracks.push_back(0);

  bool reached_end = false;
  for (int i = points.size() - 2; i >= 0; i--)
  {  // NOTE: Do not use size_t for i type here as -- with > 0 will result in overflow

    double v_i = prev_pair.speed;
    double dx = lanelet::geometry::distance2d(prev_pair.point, points[i].point);
    double new_downtrack = inverse_downtracks.back() + dx;

    if (new_downtrack < half_stopping_buffer) { // Points after (when viewed not in reverse) the midpoint of the buffer should be 0 speed
      PointSpeedPair pair = points[i];
      pair.speed = 0;
      final_points.push_back(pair);  // Store the points in reverse
      inverse_downtracks.push_back(new_downtrack);
      prev_pair = pair;
      continue;
    }
    
    if (reached_end || v_i >= starting_speed)
    {  // We are walking backward, so if the prev speed is greater than or equal to the starting speed then we are done
       // backtracking
      reached_end = true;
      PointSpeedPair pair = points[i];
      pair.speed = starting_speed;
      final_points.push_back(pair);  // Store the points in reverse
      inverse_downtracks.push_back(new_downtrack);
      prev_pair = pair;
      continue;  // continue until loop end
    }
   
    double v_f = sqrt(v_i * v_i + 2 * target_accel * dx);

    PointSpeedPair pair = points[i];
    pair.speed = std::min(v_f, starting_speed);
    final_points.push_back(pair);  // Store the points in reverse
    inverse_downtracks.push_back(new_downtrack);

    prev_pair = pair;
  }

  // Now we have a trajectory that decelerates from our end point to somewhere in the maneuver
  std::reverse(final_points.begin(),
               final_points.end());  

  std::vector<double> speeds;
  std::vector<lanelet::BasicPoint2d> raw_points;
  splitPointSpeedPairs(final_points, &raw_points, &speeds);

  // Convert the inverted downtracks back to regular downtracks
  double max_downtrack = inverse_downtracks.back();
  std::vector<double> downtracks = lanelet::utils::transform(inverse_downtracks, [max_downtrack](const auto& d) { return max_downtrack - d; });
  std::reverse(downtracks.begin(),
               downtracks.end()); 

  bool in_range = false;
  double stopped_downtrack = 0;
  lanelet::BasicPoint2d stopped_point;

  bool vehicle_in_buffer = downtracks.back() < stop_location_buffer;

  std::vector<double> filtered_speeds = basic_autonomy::smoothing::moving_average_filter(speeds, config_.moving_average_window_size);

  for (size_t i = 0; i < filtered_speeds.size(); i++)
  {  // Apply minimum speed constraint
    double downtrack = downtracks[i];

    constexpr double one_mph_in_mps = 0.44704;

    if (downtracks.back() - downtrack < stop_location_buffer && filtered_speeds[i] < config_.crawl_speed + one_mph_in_mps)
    {  // if we are within the stopping buffer and going at near crawl speed then command stop
      
      // To avoid any issues in control plugin behavior we only command 0 if the vehicle is inside the buffer
      if (vehicle_in_buffer || (i == filtered_speeds.size() - 1)) { // Vehicle is in the buffer
        filtered_speeds[i] = 0.0;
      } else { // Vehicle is not in the buffer so fill buffer with crawl speed
        filtered_speeds[i] = std::max(filtered_speeds[i], config_.crawl_speed);
      }

    }
    else
    {
      filtered_speeds[i] = std::max(filtered_speeds[i], config_.crawl_speed);
    }
  }

  std::vector<double> times;
  trajectory_utils::conversions::speed_to_time(downtracks, filtered_speeds, &times);

  for (size_t i = 0; i < times.size(); i++)
  {
    if (times[i] != 0 && !std::isnormal(times[i]) && i != 0)
    {  // If the time
      RCLCPP_WARN_STREAM(nh_->get_logger(),"Detected non-normal (nan, inf, etc.) time. Making it same as before: " << times[i-1]);
      // NOTE: overriding the timestamps in assumption that pure_pursuit_wrapper will detect it as stopping case
      times[i] = times[i - 1];
    }
  }

  std::vector<double> yaws = carma_wm::geometry::compute_tangent_orientations(raw_points);

  for (size_t i = 0; i < points.size(); i++)
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"1d: " << downtracks[i] << " t: " << times[i] << " v: " << filtered_speeds[i]);
  }

  auto traj = trajectory_from_points_times_orientations(raw_points, times, yaws, start_time);

  while (rclcpp::Time(traj.back().target_time) - rclcpp::Time(traj.front().target_time) < rclcpp::Duration(config_.minimal_trajectory_duration * 1e9))
  {
    carma_planning_msgs::msg::TrajectoryPlanPoint new_point = traj.back();
    new_point.target_time = rclcpp::Time(new_point.target_time) + rclcpp::Duration(config_.stop_timestep * 1e9);
    new_point.planner_plugin_name = plugin_name_;
    traj.push_back(new_point);
  }

  if (!filtered_speeds.empty())
    initial_speed = filtered_speeds.front(); //modify initial_speed variable passed by reference

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