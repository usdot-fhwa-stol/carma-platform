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

#include "carma_guidance_plugin/tactical_plugin.hpp"

namespace carma_guidance_plugin
{
  namespace std_ph = std::placeholders;

  ControlPlugin::ControlPlugin(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::PluginBaseNode(options)
  {}

  std::string ControlPlugin::get_capability()
  {
    return "control/trajectory_control";
  }

  uint8_t ControlPlugin::get_type() 
  {
    return carma_planning_msgs::msg::Plugin::CONTROL; 
  }

  void current_pose_callback(geometry_msgs::msg::PoseStamped::UniquePtr msg)
  {
    current_pose_ = *msg;
  }

  void current_twist_callback(geometry_msgs::msg::TwistStamped::UniquePtr msg)
  {
    current_twist_ = *msg;
  }

  void current_trajectory_callback(carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg)
  {
    current_trajectory_ = *msg;
  }

  carma_ros2_utils::CallbackReturn ControlPlugin::handle_on_configure(const rclcpp_lifecycle::State &prev_state)
  {
    // Initialize plan maneuvers service
    current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 
      std::bind(&ControlPlugin::current_pose_callback, this, std_ph::_1));

    current_velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("vehicle/twist", 
      std::bind(&ControlPlugin::current_velocity_callback, this, std_ph::_1));

    trajectory_plan_sub_ = create_subscription<carma_planning_msgs::msg::TrajectoryPlan>(get_name() + "/plan_trajectory", 
      std::bind(&ControlPlugin::current_trajectory_callback, this, std_ph::_1));

    command_timer_ = create_timer(
        get_clock(),
        std::chrono::milliseconds(33), // Spin at 30 Hz per plugin API
        std::bind(&ControlPlugin::generate_command, this));
    
    return PluginBaseNode::handle_on_configure(prev_state);
  }

  carma_ros2_utils::CallbackReturn ControlPlugin::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_activate(prev_state);
  }

  carma_ros2_utils::CallbackReturn ControlPlugin::handle_on_deactivate(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_deactivate(prev_state);
  }

  carma_ros2_utils::CallbackReturn ControlPlugin::handle_on_shutdown(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_shutdown(prev_state);
  }

  carma_ros2_utils::CallbackReturn ControlPlugin::handle_on_error(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_error(prev_state);
  }

} // carma_guidance_plugin

