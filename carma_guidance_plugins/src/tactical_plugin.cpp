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

#include <functional>
#include "carma_guidance_plugins/tactical_plugin.hpp"

namespace carma_guidance_plugins
{
  namespace std_ph = std::placeholders;

  TacticalPlugin::TacticalPlugin(const rclcpp::NodeOptions &options)
      : PluginBaseNode(options)
  {}

  std::string TacticalPlugin::get_capability()
  {
    return "tactical_plan/plan_trajectory";
  }

  uint8_t TacticalPlugin::get_type() 
  {
    return carma_planning_msgs::msg::Plugin::TACTICAL; 
  }

  carma_ros2_utils::CallbackReturn TacticalPlugin::handle_on_configure(const rclcpp_lifecycle::State &prev_state)
  {
    // Initialize plan trajectory service
    plan_trajectory_service_ = create_service<carma_planning_msgs::srv::PlanTrajectory>(get_plugin_name() + "/plan_trajectory", 
      [this] (auto header, auto req, auto resp) {
        if (this->get_activation_status()) // Only trigger when activated
        {
          this->plan_trajectory_callback(header, req, resp);
        }
      });
    
    return PluginBaseNode::handle_on_configure(prev_state);
  }

  carma_ros2_utils::CallbackReturn TacticalPlugin::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_activate(prev_state);
  }

  carma_ros2_utils::CallbackReturn TacticalPlugin::handle_on_deactivate(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_deactivate(prev_state);
  }

  carma_ros2_utils::CallbackReturn TacticalPlugin::handle_on_cleanup(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_cleanup(prev_state);
  }

  carma_ros2_utils::CallbackReturn TacticalPlugin::handle_on_shutdown(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_shutdown(prev_state);
  }

  carma_ros2_utils::CallbackReturn TacticalPlugin::handle_on_error(const rclcpp_lifecycle::State &prev_state, const std::string &exception_string)
  {
    return PluginBaseNode::handle_on_error(prev_state, exception_string);
  }

} // carma_guidance_plugins

