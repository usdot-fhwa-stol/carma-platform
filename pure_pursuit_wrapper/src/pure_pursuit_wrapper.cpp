/*
 * Copyright (C) 2018-2022 LEIDOS.
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

#include <pure_pursuit_wrapper/pure_pursuit_wrapper.hpp>
#include <trajectory_utils/conversions/conversions.hpp>
#include <carma_wm_ros2/Geometry.hpp>
#include <algorithm>


namespace pure_pursuit_wrapper
{
namespace std_ph = std::placeholders;

PurePursuitWrapperNode::PurePursuitWrapperNode(const rclcpp::NodeOptions& options)
  : carma_guidance_plugins::ControlPlugin(options)
{
  config_ = PurePursuitWrapperConfig();
  config_.vehicle_response_lag = declare_parameter<double>("vehicle_response_lag", config_.vehicle_response_lag);
  config_.minimum_lookahead_distance = declare_parameter<double>("minimum_lookahead_distance", config_.minimum_lookahead_distance);
  config_.maximum_lookahead_distance = declare_parameter<double>("maximum_lookahead_distance", config_.maximum_lookahead_distance);
  config_.speed_to_lookahead_ratio = declare_parameter<double>("speed_to_lookahead_ratio", config_.speed_to_lookahead_ratio);
  config_.is_interpolate_lookahead_point = declare_parameter<bool>("is_interpolate_lookahead_point", config_.is_interpolate_lookahead_point);
  config_.is_delay_compensation = declare_parameter<bool>("is_delay_compensation", config_.is_delay_compensation);
  config_.emergency_stop_distance = declare_parameter<double>("emergency_stop_distance", config_.emergency_stop_distance);
  config_.speed_thres_traveling_direction = declare_parameter<double>("speed_thres_traveling_direction", config_.speed_thres_traveling_direction);
  config_.dist_front_rear_wheels = declare_parameter<double>("dist_front_rear_wheels", config_.dist_front_rear_wheels);

  // integrator part
  config_.dt = declare_parameter<double>("dt", config_.dt);
  config_.integrator_max_pp = declare_parameter<double>("integrator_max_pp", config_.integrator_max_pp);
  config_.integrator_min_pp = declare_parameter<double>("integrator_min_pp", config_.integrator_min_pp);
  config_.Ki_pp = declare_parameter<double>("Ki_pp", config_.Ki_pp);
  config_.integral = declare_parameter<double>("integral", config_.integral);
  config_.is_integrator_enabled = declare_parameter<bool>("is_integrator_enabled", config_.is_integrator_enabled);
}

carma_ros2_utils::CallbackReturn PurePursuitWrapperNode::on_configure_plugin()
{
  config_ = PurePursuitWrapperConfig();
  get_parameter<double>("vehicle_response_lag", config_.vehicle_response_lag);
  get_parameter<double>("minimum_lookahead_distance", config_.minimum_lookahead_distance);
  get_parameter<double>("maximum_lookahead_distance", config_.maximum_lookahead_distance);
  get_parameter<double>("speed_to_lookahead_ratio", config_.speed_to_lookahead_ratio);
  get_parameter<bool>("is_interpolate_lookahead_point", config_.is_interpolate_lookahead_point);
  get_parameter<bool>("is_delay_compensation", config_.is_delay_compensation);
  get_parameter<double>("emergency_stop_distance", config_.emergency_stop_distance);
  get_parameter<double>("speed_thres_traveling_direction", config_.speed_thres_traveling_direction);
  get_parameter<double>("dist_front_rear_wheels", config_.dist_front_rear_wheels);
  
  // integrator configs
  get_parameter<double>("dt", config_.dt);
  get_parameter<double>("integrator_max_pp", config_.integrator_max_pp);
  get_parameter<double>("integrator_min_pp", config_.integrator_min_pp);
  get_parameter<double>("Ki_pp", config_.Ki_pp);
  get_parameter<double>("integral", config_.integral);
  get_parameter<bool>("is_integrator_enabled", config_.is_integrator_enabled);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "Loaded Params: " << config_);

  // Register runtime parameter update callback
  add_on_set_parameters_callback(std::bind(&PurePursuitWrapperNode::parameter_update_callback, this, std_ph::_1));
  
  // create config for pure_pursuit worker
  pure_pursuit::Config cfg{
    config_.minimum_lookahead_distance,
    config_.maximum_lookahead_distance,
    config_.speed_to_lookahead_ratio,
    config_.is_interpolate_lookahead_point,
    config_.is_delay_compensation,
    config_.emergency_stop_distance,
    config_.speed_thres_traveling_direction,
    config_.dist_front_rear_wheels,
  };

  pure_pursuit::IntegratorConfig i_cfg;
  i_cfg.dt = config_.dt; 
  i_cfg.integrator_max_pp = config_.integrator_max_pp; 
  i_cfg.integrator_min_pp = config_.integrator_min_pp; 
  i_cfg.Ki_pp = config_.Ki_pp; 
  i_cfg.integral = config_.integral; 
  i_cfg.is_integrator_enabled = config_.is_integrator_enabled; 
  
  pp_ = std::make_shared<pure_pursuit::PurePursuit>(cfg, i_cfg);

  // Return success if everything initialized successfully
  return CallbackReturn::SUCCESS;
}

motion::motion_common::State PurePursuitWrapperNode::convert_state(geometry_msgs::msg::PoseStamped pose, geometry_msgs::msg::TwistStamped twist)
{
  motion::motion_common::State state; //todo check if correct
  state.header = pose.header;
  state.state.x = pose.pose.position.x;
  state.state.y = pose.pose.position.y;
  state.state.z = pose.pose.position.z;
  state.state.heading.real = pose.pose.orientation.w;
  state.state.heading.imag = pose.pose.orientation.z;

  state.state.longitudinal_velocity_mps = twist.twist.linear.x;
  return state;
}

autoware_msgs::msg::ControlCommandStamped PurePursuitWrapperNode::convert_cmd(motion::motion_common::Command cmd)
{
  autoware_msgs::msg::ControlCommandStamped return_cmd; //todo check if correct
  return_cmd.header.stamp = cmd.stamp;

  return_cmd.cmd.linear_acceleration = cmd.long_accel_mps2;
  return_cmd.cmd.linear_velocity = cmd.velocity_mps;
  return_cmd.cmd.steering_angle = cmd.front_wheel_angle_rad;

  RCLCPP_ERROR_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "generate_command() cmd.stamp: " << std::to_string(rclcpp::Time(cmd.stamp).seconds()));
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "generate_command() cmd.long_accel_mps2: " << cmd.long_accel_mps2);
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "generate_command() cmd.velocity_mps: " << cmd.velocity_mps);
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "generate_command() cmd.rear_wheel_angle_rad: " << cmd.rear_wheel_angle_rad);
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "generate_command() cmd.front_wheel_angle_rad: " << cmd.front_wheel_angle_rad);

  //todo front_wheel_angle_rad is omitted
  return return_cmd;
}

autoware_msgs::msg::ControlCommandStamped PurePursuitWrapperNode::generate_command()
{
  // process and save the trajectory inside pure_pursuit
  autoware_msgs::msg::ControlCommandStamped converted_cmd;

  RCLCPP_ERROR_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "generate_command() is called");

  if (!current_trajectory_ || !current_pose_ || !current_twist_)
    return converted_cmd;


  motion::control::controller_common::State state_tf = convert_state(current_pose_.get(), current_twist_.get());

  RCLCPP_ERROR_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "Forced from frame_id: " << state_tf.header.frame_id << ", into: " << current_trajectory_.get().header.frame_id);

  current_trajectory_.get().header.frame_id = state_tf.header.frame_id;

  auto autoware_traj_plan = basic_autonomy::waypoint_generation::process_trajectory_plan(current_trajectory_.get());

  pp_->set_trajectory(autoware_traj_plan);

  const auto cmd{pp_->compute_command(state_tf)};
  
  converted_cmd = convert_cmd(cmd);


  return converted_cmd;
}

rcl_interfaces::msg::SetParametersResult PurePursuitWrapperNode::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  auto error_double = update_params<double>({
    {"/vehicle_response_lag", config_.vehicle_response_lag}}, parameters);
  //todo this needs to be updated with new ones, and also update the worker's config parameters while at it
  rcl_interfaces::msg::SetParametersResult result;

  result.successful = !error_double;

  return result;
}


bool PurePursuitWrapperNode::get_availability() 
{
  return true;
}

std::string PurePursuitWrapperNode::get_version_id() 
{
  return "v1.0";
}

std::vector<double> PurePursuitWrapperNode::apply_response_lag(const std::vector<double>& speeds, const std::vector<double> downtracks, double response_lag) const 
{ // Note first speed is assumed to be vehicle speed
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

std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> PurePursuitWrapperNode::remove_repeated_timestamps(const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& traj_points) //todo this had not been used??
{
  
  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> new_traj_points;

  carma_planning_msgs::msg::TrajectoryPlanPoint prev_point;
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
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "Duplicate point found");
    }
  }

  return new_traj_points;

}

}  // namespace pure_pursuit_wrapper

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(pure_pursuit_wrapper::PurePursuitWrapperNode)
