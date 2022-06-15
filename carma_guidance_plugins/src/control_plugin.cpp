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
#include "carma_guidance_plugins/control_plugin.hpp"


namespace carma_guidance_plugins
{
  namespace std_ph = std::placeholders;

  ControlPlugin::ControlPlugin(const rclcpp::NodeOptions &options)
      : PluginBaseNode(options)
  {}

  std::string ControlPlugin::get_capability()
  {
    return "control/trajectory_control";
  }

  uint8_t ControlPlugin::get_type() 
  {
    return carma_planning_msgs::msg::Plugin::CONTROL; 
  }

  void ControlPlugin::current_pose_callback(geometry_msgs::msg::PoseStamped::UniquePtr msg)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("carma_guidance_plugins"), "Received pose message");
    current_pose_ = *msg;
  }

  void ControlPlugin::current_twist_callback(geometry_msgs::msg::TwistStamped::UniquePtr msg)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("carma_guidance_plugins"), "Received twist message");
    current_twist_ = *msg;
  }

  void ControlPlugin::current_trajectory_callback(carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("carma_guidance_plugins"), "Received trajectory message");
    current_trajectory_ = *msg;
  }

  carma_ros2_utils::CallbackReturn ControlPlugin::handle_on_configure(const rclcpp_lifecycle::State &prev_state)
  {
    // Initialize subscribers and publishers
    current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 1,
      std::bind(&ControlPlugin::current_pose_callback, this, std_ph::_1));

    current_velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("vehicle/twist", 1,
      std::bind(&ControlPlugin::current_twist_callback, this, std_ph::_1));

    trajectory_plan_sub_ = create_subscription<carma_planning_msgs::msg::TrajectoryPlan>(get_plugin_name() + "/plan_trajectory", 1,
      std::bind(&ControlPlugin::current_trajectory_callback, this, std_ph::_1));

    vehicle_cmd_pub_ = create_publisher<autoware_msgs::msg::ControlCommandStamped>("ctrl_raw", 1);

    command_timer_ = create_timer(
        get_clock(),
        std::chrono::milliseconds(33), // Spin at 30 Hz per plugin API
        [this]() {
          if (this->get_activation_status()) // Only trigger when activated
          {
            this->vehicle_cmd_pub_->publish(this->generate_command());
          }
        });
    
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

  carma_ros2_utils::CallbackReturn ControlPlugin::handle_on_cleanup(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_cleanup(prev_state);
  }

  carma_ros2_utils::CallbackReturn ControlPlugin::handle_on_shutdown(const rclcpp_lifecycle::State &prev_state)
  {
    return PluginBaseNode::handle_on_shutdown(prev_state);
  }

  carma_ros2_utils::CallbackReturn ControlPlugin::handle_on_error(const rclcpp_lifecycle::State &prev_state, const std::string &exception_string)
  {
    return PluginBaseNode::handle_on_error(prev_state, exception_string);
  }

} // carma_guidance_plugins

