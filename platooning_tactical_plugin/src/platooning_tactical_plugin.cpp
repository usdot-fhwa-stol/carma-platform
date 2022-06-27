/*
 * Copyright (C) 2018-2020 LEIDOS.
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

using oss = std::ostringstream;

namespace platooning_tactical_plugin
{
PlatooningTacticalPlugin::PlatooningTacticalPlugin(carma_wm::WorldModelConstPtr wm, PlatooningTacticalPluginConfig config,
                                           PublishPluginDiscoveryCB plugin_discovery_publisher)
  : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher)
{
  plugin_discovery_msg_.name = "PlatooningTacticalPlugin";
  plugin_discovery_msg_.version_id = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = false;
  plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
  plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
}

bool PlatooningTacticalPlugin::onSpin()
{
  plugin_discovery_publisher_(plugin_discovery_msg_);
  return true;
}

bool PlatooningTacticalPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req,
                                              cav_srvs::PlanTrajectoryResponse& resp)
{
  ros::WallTime start_time = ros::WallTime::now(); // Start timeing the execution time for planning so it can be logged

  lanelet::BasicPoint2d veh_pos(req.vehicle_state.x_pos_global, req.vehicle_state.y_pos_global);
  double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

  // Only plan the trajectory for the initial LANE_FOLLOWING maneuver and any immediately sequential maneuvers of the same type
  std::vector<cav_msgs::Maneuver> maneuver_plan;
  for(size_t i = req.maneuver_index_to_plan; i < req.maneuver_plan.maneuvers.size(); i++)
  {
    if(req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::LANE_FOLLOWING)
    {
      if (req.maneuver_plan.maneuvers[i].lane_following_maneuver.parameters.negotiation_type != cav_msgs::ManeuverParameters::NO_NEGOTIATION)
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

  ROS_DEBUG_STREAM("points_and_target_speeds: " << points_and_target_speeds.size());

  ROS_DEBUG_STREAM("PlanTrajectory");

  cav_msgs::TrajectoryPlan original_trajectory;
  original_trajectory.header.frame_id = "map";
  original_trajectory.header.stamp = ros::Time::now();
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
  
  resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

  ros::WallTime end_time = ros::WallTime::now();  // Planning complete

  ros::WallDuration duration = end_time - start_time;
  ROS_DEBUG_STREAM("ExecutionTime: " << duration.toSec());

  return true;

}

}  // namespace platooning_tactical_plugin
