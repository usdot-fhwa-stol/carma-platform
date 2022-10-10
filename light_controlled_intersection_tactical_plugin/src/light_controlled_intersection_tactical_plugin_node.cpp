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
#include "light_controlled_intersection_tactical_plugin/light_controlled_intersection_tactical_plugin_node.hpp"

namespace light_controlled_intersection_tactical_plugin
{
  namespace std_ph = std::placeholders;

  LightControlledIntersectionTransitPluginNode::LightControlledIntersectionTransitPluginNode(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::TacticalPlugin(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.centerline_sampling_spacing = declare_parameter<double>("centerline_sampling_spacing", config_.centerline_sampling_spacing);
    config_.trajectory_time_length = declare_parameter<double>("trajectory_time_length", config_.trajectory_time_length);
    config_.default_downsample_ratio = declare_parameter<int>("default_downsample_ratio", config_.default_downsample_ratio);
    config_.turn_downsample_ratio = declare_parameter<int>("turn_downsample_ratio", config_.turn_downsample_ratio);
    config_.curve_resample_step_size = declare_parameter<double>("curve_resample_step_size", config_.curve_resample_step_size);
    config_.curvature_moving_average_window_size = declare_parameter<int>("curvature_moving_average_window_size", config_.curvature_moving_average_window_size);
    config_.speed_moving_average_window_size = declare_parameter<int>("speed_moving_average_window_size", config_.speed_moving_average_window_size);
    config_.back_distance = declare_parameter<double>("back_distance", config_.back_distance);
    config_.buffer_ending_downtrack = declare_parameter<double>("buffer_ending_downtrack", config_.buffer_ending_downtrack);
    config_.vehicle_decel_limit_multiplier = declare_parameter<double>("vehicle_decel_limit_multiplier", config_.vehicle_decel_limit_multiplier);
    config_.vehicle_accel_limit_multiplier = declare_parameter<double>("vehicle_accel_limit_multiplier", config_.vehicle_accel_limit_multiplier);
    config_.lat_accel_multiplier = declare_parameter<double>("lat_accel_multiplier", config_.lat_accel_multiplier);
    config_.stop_line_buffer = declare_parameter<double>("stop_line_buffer", config_.stop_line_buffer);
    config_.minimum_speed = declare_parameter<double>("minimum_speed", config_.minimum_speed);
    config_.algorithm_evaluation_distance = declare_parameter<double>("algorithm_evaluation_distance", config_.algorithm_evaluation_distance);
    config_.algorithm_evaluation_period = declare_parameter<double>("algorithm_evaluation_period", config_.algorithm_evaluation_period);

    config_.lateral_accel_limit = declare_parameter<double>("vehicle_lateral_accel_limit", config_.lateral_accel_limit);
    config_.vehicle_accel_limit = declare_parameter<double>("vehicle_acceleration_limit", config_.vehicle_accel_limit);
    config_.vehicle_decel_limit = declare_parameter<double>("vehicle_deceleration_limit", config_.vehicle_decel_limit);
  }

  rcl_interfaces::msg::SetParametersResult LightControlledIntersectionTransitPluginNode::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
     auto error = update_params<double>(
       {{"centerline_sampling_spacing", config_.centerline_sampling_spacing},
       {"trajectory_time_length", config_.trajectory_time_length},
       {"curve_resample_step_size", config_.curve_resample_step_size},
       {"back_distance", config_.back_distance},
       {"buffer_ending_downtrack", config_.buffer_ending_downtrack},
       {"vehicle_decel_limit_multiplier", config_.vehicle_decel_limit_multiplier},
       {"vehicle_accel_limit_multiplier", config_.vehicle_accel_limit_multiplier},
       {"lat_accel_multiplier", config_.lat_accel_multiplier},      
       {"stop_line_buffer", config_.stop_line_buffer},      
       {"minimum_speed", config_.minimum_speed},      
       {"algorithm_evaluation_distance", config_.algorithm_evaluation_distance},      
       {"algorithm_evaluation_period", config_.algorithm_evaluation_period}}, parameters); // Global acceleration limits not allowed to dynamically update
    auto error_2 = update_params<int>(
      {{"default_downsample_ratio", config_.default_downsample_ratio},
      {"turn_downsample_ratio", config_.turn_downsample_ratio},
      {"curvature_moving_average_window_size", config_.curvature_moving_average_window_size},
      {"speed_moving_average_window_size", config_.speed_moving_average_window_size}}, parameters);

    if (worker_)
    {
      worker_->setConfig(config_);
    }

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error_2;

    return result;
  }

  carma_ros2_utils::CallbackReturn LightControlledIntersectionTransitPluginNode::on_configure_plugin()
  {
    RCLCPP_INFO_STREAM(get_logger(), "LightControlledIntersectionTransitPluginNode trying to configure");

    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<double>("centerline_sampling_spacing", config_.centerline_sampling_spacing);
    get_parameter<double>("trajectory_time_length", config_.trajectory_time_length);
    get_parameter<int>("default_downsample_ratio", config_.default_downsample_ratio);
    get_parameter<int>("turn_downsample_ratio", config_.turn_downsample_ratio);
    get_parameter<double>("curve_resample_step_size", config_.curve_resample_step_size);
    get_parameter<int>("curvature_moving_average_window_size", config_.curvature_moving_average_window_size);
    get_parameter<int>("speed_moving_average_window_size", config_.speed_moving_average_window_size);
    get_parameter<double>("back_distance", config_.back_distance);
    get_parameter<double>("buffer_ending_downtrack", config_.buffer_ending_downtrack);
    get_parameter<double>("vehicle_decel_limit_multiplier", config_.vehicle_decel_limit_multiplier);
    get_parameter<double>("vehicle_accel_limit_multiplier", config_.vehicle_accel_limit_multiplier);
    get_parameter<double>("lat_accel_multiplier", config_.lat_accel_multiplier);
    get_parameter<double>("stop_line_buffer", config_.stop_line_buffer);
    get_parameter<double>("minimum_speed", config_.minimum_speed);
    get_parameter<double>("algorithm_evaluation_distance", config_.algorithm_evaluation_distance);
    get_parameter<double>("algorithm_evaluation_period", config_.algorithm_evaluation_period);
    get_parameter<double>("vehicle_lateral_accel_limit", config_.lateral_accel_limit);
    get_parameter<double>("vehicle_acceleration_limit", config_.vehicle_accel_limit);
    get_parameter<double>("vehicle_deceleration_limit", config_.vehicle_decel_limit);

    // Use the configured multipliers to update the accel limits
    config_.lateral_accel_limit = config_.lateral_accel_limit * config_.lat_accel_multiplier;
    config_.vehicle_accel_limit = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
    config_.vehicle_decel_limit = config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&LightControlledIntersectionTransitPluginNode::parameter_update_callback, this, std_ph::_1));

    RCLCPP_INFO_STREAM(get_logger(), "Loaded params: " << config_);

    // Initialize worker object
    worker_ = std::make_shared<LightControlledIntersectionTacticalPlugin>(get_world_model(), config_, get_node_logging_interface());

    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }

  bool LightControlledIntersectionTransitPluginNode::get_availability() {
    return true;
  }

  std::string LightControlledIntersectionTransitPluginNode::get_version_id() {
    return "v4.0"; // Version ID matches the value set in this package's package.xml
  }

  void LightControlledIntersectionTransitPluginNode::plan_trajectory_callback(
    std::shared_ptr<rmw_request_id_t>, 
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
  {
    worker_->planTrajectoryCB(req, resp);
  }

} // light_controlled_intersection_tactical_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(light_controlled_intersection_tactical_plugin::LightControlledIntersectionTransitPluginNode)
