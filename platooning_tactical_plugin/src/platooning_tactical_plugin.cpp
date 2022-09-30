/*
 * Copyright (C) 2018-2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "platooning_tactical_plugin/platooning_tactical_plugin.h"

#include <string>
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.hpp>
#include <trajectory_utils/conversions/conversions.hpp>
#include <sstream>
#include <chrono>
#include <carma_ros2_utils/containers/containers.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <unordered_set>

using oss = std::ostringstream;

namespace platooning_tactical_plugin
{
PlatooningTacticalPlugin::PlatooningTacticalPlugin(carma_wm::WorldModelConstPtr wm, PlatooningTacticalPluginConfig config,
                                           std::shared_ptr<carma_ros2_utils::timers::TimerFactory> timer_factory)
  : wm_(wm), config_(config), timer_factory_(timer_factory)
{}

bool PlatooningTacticalPlugin::plan_trajectory_cb(carma_planning_msgs::srv::PlanTrajectory::Request& req,
                                                  carma_planning_msgs::srv::PlanTrajectory::Response& resp)
{
  auto start_time = std::chrono::high_resolution_clock::now(); // Start timing the execution time for planning so it can be logged

  lanelet::BasicPoint2d veh_pos(req.vehicle_state.x_pos_global, req.vehicle_state.y_pos_global);
  double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

  // Only plan the trajectory for the initial LANE_FOLLOWING maneuver and any immediately sequential maneuvers of the same type
  std::vector<carma_planning_msgs::msg::Maneuver> maneuver_plan;
  for(size_t i = req.maneuver_index_to_plan; i < req.maneuver_plan.maneuvers.size(); i++)
  {
    if(req.maneuver_plan.maneuvers[i].type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING)
    {
      if (req.maneuver_plan.maneuvers[i].lane_following_maneuver.parameters.negotiation_type != carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION)
      {
        maneuver_plan.push_back(req.maneuver_plan.maneuvers[i]);
        resp.related_maneuvers.push_back(i);
      }
    }
    else
    {
      break;
    }
  }

  basic_autonomy:: waypoint_generation::DetailedTrajConfig wpg_detail_config;
  basic_autonomy:: waypoint_generation::GeneralTrajConfig wpg_general_config;

  wpg_general_config = basic_autonomy:: waypoint_generation::compose_general_trajectory_config("platooning",
                                                                              config_.default_downsample_ratio,
                                                                              config_.turn_downsample_ratio);
  
  wpg_detail_config = basic_autonomy:: waypoint_generation::compose_detailed_trajectory_config(config_.trajectory_time_length, 
                                                                            config_.curve_resample_step_size, config_.minimum_speed, 
                                                                            config_.max_accel * config_.max_accel_multiplier,
                                                                            config_.lateral_accel_limit * config_.lat_accel_multiplier, 
                                                                            config_.speed_moving_average_window_size, 
                                                                            config_.curvature_moving_average_window_size, config_.back_distance,
                                                                            config_.buffer_ending_downtrack,
                                                                            config_.desired_controller_plugin);
  
  auto points_and_target_speeds = basic_autonomy::waypoint_generation::create_geometry_profile(maneuver_plan, std::max((double)0, current_downtrack - config_.back_distance),
                                                                         wm_, ending_state_before_buffer_, req.vehicle_state, wpg_general_config, wpg_detail_config);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_tactical_plugin"), "points_and_target_speeds: " << points_and_target_speeds.size());

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_tactical_plugin"), "PlanTrajectory");

  carma_planning_msgs::msg::TrajectoryPlan original_trajectory;
  original_trajectory.header.frame_id = "map";
  original_trajectory.header.stamp = timer_factory_->now();
  original_trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());
  original_trajectory.trajectory_points = basic_autonomy:: waypoint_generation::compose_lanefollow_trajectory_from_path(points_and_target_speeds, 
                                                                                req.vehicle_state, req.header.stamp, wm_, ending_state_before_buffer_, debug_msg_, 
                                                                                wpg_detail_config); // Compute the trajectory
  original_trajectory.initial_longitudinal_velocity = std::max(req.vehicle_state.longitudinal_vel, config_.minimum_speed);

  resp.trajectory_plan = original_trajectory;

  if (config_.publish_debug) { // Publish the debug message if in debug logging mode
    debug_msg_.trajectory_plan = resp.trajectory_plan;
    debug_publisher_(debug_msg_); 
  }
  
  resp.maneuver_status.push_back(carma_planning_msgs::srv::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

  auto end_time = std::chrono::high_resolution_clock::now(); // Planning complete

  auto nano_s = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_tactical_plugin"), "ExecutionTime: " << ((double)nano_s.count() * 1e9));

  return true;

}

void PlatooningTacticalPlugin::set_config(PlatooningTacticalPluginConfig config)
{
  config_ = config;
}

}  // namespace platooning_tactical_plugin
