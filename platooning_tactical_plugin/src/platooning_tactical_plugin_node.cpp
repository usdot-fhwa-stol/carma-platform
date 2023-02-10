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
#include "platooning_tactical_plugin/platooning_tactical_plugin_node.h"
#include <carma_ros2_utils/timers/ROSTimerFactory.hpp>

namespace platooning_tactical_plugin
{
  namespace std_ph = std::placeholders;

  Node::Node(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::TacticalPlugin(options)
  {
    // Create initial config
    config_ = PlatooningTacticalPluginConfig();

    // Declare parameters
    config_.trajectory_time_length = declare_parameter<double>("trajectory_time_length", config_.trajectory_time_length);
    config_.curve_resample_step_size = declare_parameter<double>("curve_resample_step_size", config_.curve_resample_step_size);
    config_.default_downsample_ratio = declare_parameter<int>("default_downsample_ratio", config_.default_downsample_ratio);
    config_.turn_downsample_ratio = declare_parameter<int>("turn_downsample_ratio", config_.turn_downsample_ratio);
    config_.minimum_speed = declare_parameter<double>("minimum_speed", config_.minimum_speed);
    config_.max_accel_multiplier = declare_parameter<double>("max_accel_multiplier", config_.max_accel_multiplier);
    config_.lat_accel_multiplier = declare_parameter<double>("lat_accel_multiplier", config_.lat_accel_multiplier);
    config_.back_distance = declare_parameter<double>("back_distance", config_.back_distance);
    config_.speed_moving_average_window_size = declare_parameter<int>("speed_moving_average_window_size", config_.speed_moving_average_window_size);
    config_.curvature_moving_average_window_size = declare_parameter<int>("curvature_moving_average_window_size", config_.curvature_moving_average_window_size);
    config_.max_accel = declare_parameter<double>("vehicle_acceleration_limit", config_.max_accel);
    config_.lateral_accel_limit = declare_parameter<double>("vehicle_lateral_accel_limit", config_.lateral_accel_limit);
    config_.enable_object_avoidance = declare_parameter<bool>("enable_object_avoidance", config_.enable_object_avoidance);
    config_.buffer_ending_downtrack = declare_parameter<double>("buffer_ending_downtrack", config_.buffer_ending_downtrack);

    config_.lateral_accel_limit = config_.lateral_accel_limit * config_.lat_accel_multiplier;
    config_.max_accel = config_.max_accel *  config_.max_accel_multiplier;

    RCLCPP_INFO_STREAM(get_logger(), "PlatooningTacticalPlugin Params" << config_);

  }

  rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error = update_params<bool>({
      {"enable_object_avoidance", config_.enable_object_avoidance}
    }, parameters);

    auto error2 = update_params<int>({
      {"default_downsample_ratio", config_.default_downsample_ratio},
      {"turn_downsample_ratio", config_.turn_downsample_ratio},
      {"speed_moving_average_window_size", config_.speed_moving_average_window_size},
      {"curvature_moving_average_window_size", config_.curvature_moving_average_window_size}
    }, parameters);

    auto error3 = update_params<double>({
      {"trajectory_time_length", config_.trajectory_time_length},
      {"curve_resample_step_size", config_.curve_resample_step_size},
      {"minimum_speed", config_.minimum_speed},
      {"max_accel_multiplier", config_.max_accel_multiplier},
      {"lat_accel_multiplier", config_.lat_accel_multiplier},
      {"back_distance", config_.back_distance},
      {"buffer_ending_downtrack", config_.buffer_ending_downtrack}
    }, parameters); // Accel limits system wide and not allowed to be updated per node

    config_.lateral_accel_limit = config_.lateral_accel_limit * config_.lat_accel_multiplier;
    config_.max_accel = config_.max_accel *  config_.max_accel_multiplier;

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error2 && !error3;

    if (result.successful && worker_)
    {
      worker_->set_config(config_);
    }

    return result;
  }

  carma_ros2_utils::CallbackReturn Node::on_configure_plugin()
  {
    // Reset config
    config_ = PlatooningTacticalPluginConfig();

    // Load parameters
    get_parameter<double>("trajectory_time_length", config_.trajectory_time_length);
    get_parameter<double>("curve_resample_step_size", config_.curve_resample_step_size);
    get_parameter<int>("default_downsample_ratio", config_.default_downsample_ratio);
    get_parameter<int>("turn_downsample_ratio", config_.turn_downsample_ratio);
    get_parameter<double>("minimum_speed", config_.minimum_speed);
    get_parameter<double>("max_accel_multiplier", config_.max_accel_multiplier);
    get_parameter<double>("lat_accel_multiplier", config_.lat_accel_multiplier);
    get_parameter<double>("back_distance", config_.back_distance);
    get_parameter<int>("speed_moving_average_window_size", config_.speed_moving_average_window_size);
    get_parameter<int>("curvature_moving_average_window_size", config_.curvature_moving_average_window_size);
    get_parameter<double>("vehicle_acceleration_limit", config_.max_accel);
    get_parameter<double>("vehicle_lateral_accel_limit", config_.lateral_accel_limit);
    get_parameter<bool>("enable_object_avoidance", config_.enable_object_avoidance);
    get_parameter<double>("buffer_ending_downtrack", config_.buffer_ending_downtrack);

    config_.lateral_accel_limit = config_.lateral_accel_limit * config_.lat_accel_multiplier;
    config_.max_accel = config_.max_accel *  config_.max_accel_multiplier;

    RCLCPP_INFO_STREAM(get_logger(), "PlatooningTacticalPlugin Params" << config_);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&Node::parameter_update_callback, this, std_ph::_1));

    worker_ = std::make_shared<PlatooningTacticalPlugin>(get_world_model(), config_, 
      std::make_shared<carma_ros2_utils::timers::ROSTimerFactory>(shared_from_this()));

    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }

  void Node::plan_trajectory_callback(
    std::shared_ptr<rmw_request_id_t>, 
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
  {
    if (!worker_)
      return;
    
    worker_->plan_trajectory_cb(*req, *resp);
  }

  bool Node::get_availability() {
    return true;
  }

  std::string Node::get_version_id() {
    return "v4.0";
  }

} // platooning_tactical_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(platooning_tactical_plugin::Node)
