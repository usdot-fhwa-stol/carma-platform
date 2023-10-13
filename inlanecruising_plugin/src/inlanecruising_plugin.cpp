/*
 * Copyright (C) 2022 LEIDOS.
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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <inlanecruising_plugin/inlanecruising_plugin.hpp>




using oss = std::ostringstream;

namespace inlanecruising_plugin
{
InLaneCruisingPlugin::InLaneCruisingPlugin(std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh, 
                                          carma_wm::WorldModelConstPtr wm, 
                                          const InLaneCruisingPluginConfig& config, 
                                          const DebugPublisher& debug_publisher,
                                          const std::string& plugin_name,
                                          const std::string& version_id)
  : nh_(nh), wm_(wm), config_(config), debug_publisher_(debug_publisher), plugin_name_(plugin_name), version_id_ (version_id)
{}

void InLaneCruisingPlugin::plan_trajectory_callback(
  carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
  carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
{
  std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();  // Start timing the execution time for planning so it can be logged

  lanelet::BasicPoint2d veh_pos(req->vehicle_state.x_pos_global, req->vehicle_state.y_pos_global);
  double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

  // Only plan the trajectory for the initial LANE_FOLLOWING maneuver and any immediately sequential maneuvers of the same type
  std::vector<carma_planning_msgs::msg::Maneuver> maneuver_plan;
  for(size_t i = req->maneuver_index_to_plan; i < req->maneuver_plan.maneuvers.size(); i++)
  {
    if(req->maneuver_plan.maneuvers[i].type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING)
    {
      maneuver_plan.push_back(req->maneuver_plan.maneuvers[i]);
      resp->related_maneuvers.push_back((uint8_t)i);
    }
    else
    {
      break;
    }
  }

  basic_autonomy:: waypoint_generation::DetailedTrajConfig wpg_detail_config;
  basic_autonomy:: waypoint_generation::GeneralTrajConfig wpg_general_config;

  wpg_general_config = basic_autonomy:: waypoint_generation::compose_general_trajectory_config("inlanecruising",
                                                                              config_.default_downsample_ratio,
                                                                              config_.turn_downsample_ratio);

  wpg_detail_config = basic_autonomy:: waypoint_generation::compose_detailed_trajectory_config(config_.trajectory_time_length, 
                                                                            config_.curve_resample_step_size, config_.minimum_speed, 
                                                                            config_.max_accel * config_.max_accel_multiplier,
                                                                            config_.lateral_accel_limit * config_.lat_accel_multiplier, 
                                                                            config_.speed_moving_average_window_size, 
                                                                            config_.curvature_moving_average_window_size, config_.back_distance,
                                                                            config_.buffer_ending_downtrack);
  
  auto points_and_target_speeds = basic_autonomy::waypoint_generation::create_geometry_profile(maneuver_plan, std::max((double)0, current_downtrack - config_.back_distance),
                                                                         wm_, ending_state_before_buffer_, req->vehicle_state, wpg_general_config, wpg_detail_config);

  RCLCPP_DEBUG_STREAM(nh_->get_logger(), "points_and_target_speeds: " << points_and_target_speeds.size());

  RCLCPP_DEBUG_STREAM(nh_->get_logger(), "PlanTrajectory");

  carma_planning_msgs::msg::TrajectoryPlan original_trajectory;
  original_trajectory.header.frame_id = "map";
  original_trajectory.header.stamp = nh_->now();
  original_trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

  original_trajectory.trajectory_points = basic_autonomy:: waypoint_generation::compose_lanefollow_trajectory_from_path(points_and_target_speeds, 
                                                                                req->vehicle_state, req->header.stamp, wm_, ending_state_before_buffer_, debug_msg_, 
                                                                                wpg_detail_config); // Compute the trajectory
  original_trajectory.initial_longitudinal_velocity = std::max(req->vehicle_state.longitudinal_vel, config_.minimum_speed);

  // Set the planning plugin field name
  for (auto& p : original_trajectory.trajectory_points) {
    p.planner_plugin_name = plugin_name_;
  }
  
  // Aside from the flag, ILC should not call yield_plugin on invalid trajectories
  if (config_.enable_object_avoidance && original_trajectory.trajectory_points.size() >= 2)
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Activate Object Avoidance");

    if (yield_client_ && yield_client_->service_is_ready())
    {
      RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Yield Client is valid");
      
      auto yield_srv = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Request>();
      yield_srv->initial_trajectory_plan = original_trajectory;
      yield_srv->vehicle_state = req->vehicle_state;

      auto yield_resp = yield_client_->async_send_request(yield_srv);

      auto future_status = yield_resp.wait_for(std::chrono::milliseconds(100));

      if (future_status == std::future_status::ready)
      {
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Received Traj from Yield");
        carma_planning_msgs::msg::TrajectoryPlan yield_plan = yield_resp.get()->trajectory_plan;
        if (validate_yield_plan(yield_plan))
        {
          RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Yield trajectory validated");
          resp->trajectory_plan = yield_plan;
        }
        else
        {
          throw std::invalid_argument("Invalid Yield Trajectory");
        }
      }
      else
      {
        throw std::invalid_argument("Unable to Call Yield Plugin");
      }
    }
    else
    {
      throw std::invalid_argument("Yield Client is unavailable");
    }
    
  }
  else
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Ignored Object Avoidance");
    resp->trajectory_plan = original_trajectory;
  }

  if (config_.publish_debug) { // Publish the debug message if in debug logging mode
    debug_msg_.trajectory_plan = resp->trajectory_plan;
    debug_publisher_(debug_msg_); 
  }
  
  resp->maneuver_status.push_back(carma_planning_msgs::srv::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

  std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now();  // Planning complete

  auto duration = end_time - start_time;
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), "ExecutionTime: " << std::chrono::duration<double>(duration).count());
}

void InLaneCruisingPlugin::set_yield_client(carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> client)
{
  yield_client_ = client;
}

bool InLaneCruisingPlugin::validate_yield_plan(const carma_planning_msgs::msg::TrajectoryPlan& yield_plan) const
{
  if (yield_plan.trajectory_points.size()>= 2)
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Yield Trajectory Time" << rclcpp::Time(yield_plan.trajectory_points[0].target_time).seconds());
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Now:" << nh_->now().seconds());
    if (rclcpp::Time(yield_plan.trajectory_points[0].target_time) + rclcpp::Duration(5.0, 0) > nh_->now())
    {
      return true;
    }
    else
    {
      RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Old Yield Trajectory");
    }
  }
  else
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Invalid Yield Trajectory"); 
  }
  return false;
}


}  // namespace inlanecruising_plugin