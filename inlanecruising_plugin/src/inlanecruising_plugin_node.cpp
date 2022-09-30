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

#include <inlanecruising_plugin/inlanecruising_plugin_node.hpp>

namespace inlanecruising_plugin
{
  namespace std_ph = std::placeholders;
  
  InLaneCruisingPluginNode::InLaneCruisingPluginNode(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::TacticalPlugin(options), 
        plugin_name_(get_plugin_name_and_ns()), 
        version_id_("v4.0"), 
        config_(InLaneCruisingPluginConfig())
  {
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
    config_.buffer_ending_downtrack = declare_parameter<double>("buffer_ending_downtrack", config_.buffer_ending_downtrack);
    config_.max_accel = declare_parameter<double>("vehicle_acceleration_limit", config_.max_accel);
    config_.lateral_accel_limit = declare_parameter<double>("vehicle_lateral_accel_limit", config_.lateral_accel_limit);
    config_.enable_object_avoidance = declare_parameter<bool>("enable_object_avoidance", config_.enable_object_avoidance);
  }
  
  carma_ros2_utils::CallbackReturn InLaneCruisingPluginNode::on_configure_plugin()
  {
    trajectory_debug_pub_ = create_publisher<carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds>("debug/trajectory_planning", 1);

    config_ = InLaneCruisingPluginConfig();

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
    get_parameter<double>("buffer_ending_downtrack", config_.buffer_ending_downtrack);
    get_parameter<double>("vehicle_acceleration_limit", config_.max_accel);
    get_parameter<double>("vehicle_lateral_accel_limit", config_.lateral_accel_limit);
    get_parameter<bool>("enable_object_avoidance", config_.enable_object_avoidance);


    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&InLaneCruisingPluginNode::parameter_update_callback, this, std_ph::_1));

    RCLCPP_INFO_STREAM(rclcpp::get_logger("inlanecruising_plugin"), "InLaneCruisingPlugin Params" << config_);
    
    config_.lateral_accel_limit = config_.lateral_accel_limit * config_.lat_accel_multiplier;
    config_.max_accel = config_.max_accel *  config_.max_accel_multiplier;
    
    // Determine if we will enable debug publishing by checking the current log level of the node
    auto level = rcutils_logging_get_logger_level(get_logger().get_name());

    config_.publish_debug = level == RCUTILS_LOG_SEVERITY_DEBUG;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("inlanecruising_plugin"), "InLaneCruisingPlugin Params After Accel Change" << config_);
    
    worker_ = std::make_shared<InLaneCruisingPlugin>(shared_from_this(), get_world_model(), config_,
                                                          [this](const carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds& msg) { trajectory_debug_pub_->publish(msg); },
                                                          plugin_name_,
                                                          version_id_);

    //TODO: Update yield client to use the Plugin Manager capabilities query, in case someone else wants to add an alternate yield implementation 
    yield_client_ = create_client<carma_planning_msgs::srv::PlanTrajectory>("yield_plugin/plan_trajectory");
    worker_->set_yield_client(yield_client_);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("inlanecruising_plugin"), "Yield Client Set");

    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;

  }

  rcl_interfaces::msg::SetParametersResult InLaneCruisingPluginNode::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error_double = update_params<double>({
      {"trajectory_time_length", config_.trajectory_time_length},
      {"curve_resample_step_size", config_.curve_resample_step_size},
      {"minimum_speed", config_.minimum_speed},
      {"max_accel_multiplier", config_.max_accel_multiplier},
      {"lat_accel_multiplier", config_.lat_accel_multiplier},
      {"back_distance", config_.back_distance},
      {"buffer_ending_downtrack", config_.buffer_ending_downtrack}}, parameters); // Global acceleration limits not allowed to dynamically update

    auto error_bool = update_params<bool>({
      {"enable_object_avoidance", config_.enable_object_avoidance}}, parameters);

    auto error_int = update_params<int>({
      {"default_downsample_ratio", config_.default_downsample_ratio}, 
      {"turn_downsample_ratio", config_.turn_downsample_ratio}, 
      {"speed_moving_average_window_size", config_.speed_moving_average_window_size}, 
      {"curvature_moving_average_window_size", config_.curvature_moving_average_window_size}}, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error_double && !error_bool && !error_int;

    return result;
  }

  bool InLaneCruisingPluginNode::get_availability()
  {
    return true;
  }

  std::string InLaneCruisingPluginNode::get_version_id()
  {
    return version_id_;
  }

  void InLaneCruisingPluginNode::plan_trajectory_callback(
    std::shared_ptr<rmw_request_id_t> srv_header, 
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
  {
    worker_->plan_trajectory_callback(req, resp);
  }

}  // namespace inlanecruising_plugin


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(inlanecruising_plugin::InLaneCruisingPluginNode)