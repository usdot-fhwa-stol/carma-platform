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

#include "stop_and_wait_plugin_node.hpp"

namespace stop_and_wait_plugin
{
  namespace std_ph = std::placeholders;

  StopandWaitNode::StopandWaitNode(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::TacticalPlugin(options)
  {
    // Create initial config
    config = StopandWaitConfig();
 
    // Declare parameters
    config.config.minimal_trajectory_duration = declare_parameter<std::string>("minimal_trajectory_duration", config.minimal_trajectory_duration);
    config.stop_timestep = declare_parameter<std::string>("stop_timestep", config.stop_timestep);
    config.trajectory_step_size = declare_parameter<std::string>("trajectory_step_size", config.trajectory_step_size);
    config.accel_limit_multiplier = declare_parameter<std::string>("accel_limit_multiplier", config.accel_limit_multiplier);
    config.accel_limit = declare_parameter<std::string>("/vehicle_acceleration_limit", config.accel_limit);
    config.crawl_speed = declare_parameter<std::string>("crawl_speed", config.crawl_speed);
    config.cernterline_sampling_spacing = declare_parameter<std::string>("cernterline_sampling_spacing", config.cernterline_sampling_spacing);
    config.default_stopping_buffer = declare_parameter<std::string>("default_stopping_buffer", config.default_stopping_buffer);
  }

  carma_ros2_utils::CallbackReturn StopandWaitNode::handle_on_configure(const rclcpp_lifecycle::State &)
  {
  
    StopandWait plugin(wm, config);
                                        
    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

    void StopandWaitNode::plan_trajectory_callback(
    std::shared_ptr<rmw_request_id_t> srv_header, 
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
  {
    plugin->plan_trajectory_cb(req, resp);
  }

} // stop_and_wait_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(stop_and_wait_plugin::StopandWaitNode)
