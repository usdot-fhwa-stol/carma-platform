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

#include "carma_guidance_plugin/strategic_plugin.hpp"

namespace carma_guidance_plugin
{
  namespace std_ph = std::placeholders;

  StrategicPlugin::StrategicPlugin(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::PluginBaseNode(options)
  {}

  std::string StrategicPlugin::get_capability()
  {
    return "strategic_plan/plan_maneuvers";
  }

  uint8_t StrategicPlugin::get_type() 
  {
    return carma_planning_msgs::msg::Plugin::STRATEGIC; 
  }

  carma_ros2_utils::CallbackReturn StrategicPlugin::handle_on_configure(const rclcpp_lifecycle::State &prev_state)
  {
    // Initialize plan maneuvers service
    plan_maneuvers_service_ = create_service<carma_planning_msgs::msg::PlanManeuvers>(get_name() + "/plan_maneuvers", 
      std::bind(&StrategicPlugin::plan_maneuvers_callback, this, std_ph::_1, std_ph::_2, std_ph::_3));
    
    return PluginBaseNode::handle_on_configure(prev_state);
  }

  carma_ros2_utils::CallbackReturn StrategicPlugin::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_activate(prev_state);
  }

  carma_ros2_utils::CallbackReturn StrategicPlugin::handle_on_deactivate(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_deactivate(prev_state);
  }

  carma_ros2_utils::CallbackReturn StrategicPlugin::handle_on_shutdown(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_shutdown(prev_state);
  }

  carma_ros2_utils::CallbackReturn StrategicPlugin::handle_on_error(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_error(prev_state);
  }

} // carma_guidance_plugin

