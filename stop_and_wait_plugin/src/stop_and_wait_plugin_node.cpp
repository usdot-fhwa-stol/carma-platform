/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include "stop_and_wait_node.hpp"

namespace stop_and_wait_plugin
{
  namespace std_ph = std::placeholders;

  StopandWaitNode::StopandWaitNode(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::TacticalPlugin(options),version_id_("v4.0"),plugin_name_(get_plugin_name_and_ns())
  {
    // Create initial config
    config_ = StopandWaitConfig();
 
    // Declare parameters
    config_.minimal_trajectory_duration = declare_parameter<double>("minimal_trajectory_duration", config_.minimal_trajectory_duration);
    config_.stop_timestep = declare_parameter<double>("stop_timestep", config_.stop_timestep);
    config_.trajectory_step_size = declare_parameter<double>("trajectory_step_size", config_.trajectory_step_size);
    config_.accel_limit_multiplier = declare_parameter<double>("accel_limit_multiplier", config_.accel_limit_multiplier);
    config_.accel_limit = declare_parameter<double>("vehicle_acceleration_limit", config_.accel_limit);
    config_.crawl_speed = declare_parameter<double>("crawl_speed", config_.crawl_speed);
    config_.centerline_sampling_spacing = declare_parameter<double>("centerline_sampling_spacing", config_.centerline_sampling_spacing);
    config_.default_stopping_buffer = declare_parameter<double>("default_stopping_buffer", config_.default_stopping_buffer);
  }

  rcl_interfaces::msg::SetParametersResult StopandWaitNode::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {

    auto error = update_params<double>({
      {"minimal_trajectory_duration", config_.minimal_trajectory_duration},
      {"stop_timestep", config_.stop_timestep},
      {"trajectory_step_size", config_.trajectory_step_size},
      {"accel_limit_multiplier", config_.accel_limit_multiplier},
      {"crawl_speed", config_.crawl_speed},
      {"centerline_sampling_spacing", config_.centerline_sampling_spacing},
      {"default_stopping_buffer", config_.default_stopping_buffer}
    }, parameters); // vehicle_acceleration_limit not updated as it's global param

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error;

    return result;
  }

  carma_ros2_utils::CallbackReturn StopandWaitNode::on_configure_plugin()
  {

    get_parameter<double>("minimal_trajectory_duration", config_.minimal_trajectory_duration);
    get_parameter<double>("stop_timestep", config_.stop_timestep);
    get_parameter<double>("trajectory_step_size", config_.trajectory_step_size);
    get_parameter<double>("accel_limit_multiplier", config_.accel_limit_multiplier);
    get_parameter<double>("vehicle_acceleration_limit", config_.accel_limit);
    get_parameter<double>("crawl_speed", config_.crawl_speed);
    get_parameter<double>("centerline_sampling_spacing", config_.centerline_sampling_spacing);
    get_parameter<double>("default_stopping_buffer", config_.default_stopping_buffer);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&StopandWaitNode::parameter_update_callback, this, std_ph::_1));
  
    plugin_ = std::make_shared<StopandWait>(shared_from_this(), get_world_model(), config_,plugin_name_,version_id_);
                                        
    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }

    void StopandWaitNode::plan_trajectory_callback(
    std::shared_ptr<rmw_request_id_t> srv_header, 
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
  {
    plugin_->plan_trajectory_cb(req, resp);
  }

    bool StopandWaitNode::get_availability()
  {
    return true;
  }

  std::string StopandWaitNode::get_version_id()
  {
    return version_id_;
  }

} // stop_and_wait_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(stop_and_wait_plugin::StopandWaitNode)
