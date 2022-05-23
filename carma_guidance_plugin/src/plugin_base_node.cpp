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
#include <rclcpp/create_publisher.hpp>

#include "carma_guidance_plugin/plugin_base_node.hpp"

namespace carma_guidance_plugin
{
  namespace std_ph = std::placeholders;

  PluginBaseNode::PluginBaseNode(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Setup plugin publisher discovery which should bypass lifecycle behavior to ensure plugins are found
    // prior to them needing to be activated.
    // NOTE: Any other topics which need to be setup in the future should use handle_on_configure and the default this->create_publisher method to get a lifecycle publisher instead
    plugin_discovery_pub_ = rclcpp::create_publisher<carma_planning_msgs::msg::Plugin>(this, "plugin_discovery",1);

    // Setup discovery timer to publish onto the plugin_discovery_pub
    discovert_timer_ = create_timer(
        get_clock(),
        std::chrono::milliseconds(1000), // TODO double check frequency of update. Seems like it should be at least 500ms to hit nyquist frequency for arbitration calls
        std::bind(&PluginBaseNode::discovery_timer_callback, this));
  }

  bool PluginBaseNode::get_activation_status() {
    // Determine the plugin activation state by checking which lifecycle state we are in. 
    // If we are active then the plugin is active otherwise the plugin is inactive
    return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  carma_ros2_utils::CallbackReturn PluginBaseNode::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    return configure_plugin();
  }
  carma_ros2_utils::CallbackReturn PluginBaseNode::handle_on_activate(const rclcpp_lifecycle::State &)
  {
    return activate_plugin();
  }
  carma_ros2_utils::CallbackReturn PluginBaseNode::handle_on_deactivate(const rclcpp_lifecycle::State &)
  {
    return deactivate_plugin();
  }
  carma_ros2_utils::CallbackReturn PluginBaseNode::handle_on_shutdown(const rclcpp_lifecycle::State &)
  {
    if (get_activation_status())
      deactivate_plugin(); // Try to deactivate first on shutdown

    return CallbackReturn::SUCCESS;
  }
  carma_ros2_utils::CallbackReturn PluginBaseNode::handle_on_error(const rclcpp_lifecycle::State &)
  {
    if (get_activation_status())
      deactivate_plugin(); // Try to deactivate first on error

    return CallbackReturn::SUCCESS;
  }

  uint8_t PluginBaseNode::get_type() 
  {
    // Base class returns unknown. 
    // Its expected that 2nd level extending classes (strategic, tactical, control) will return correct type by overriding
    return carma_planning_msgs::msg::Plugin::UNKNOWN;
  }

  void PluginBaseNode::discovery_timer_callback()
  {
    carma_planning_msgs::msg::Plugin msg;
    msg.name = get_name();
    msg.version_id = get_version_id();
    msg.type = get_type();
    msg.available = get_availability();
    msg.activated = get_activation_status();
    msg.capability = get_capability();

    plugin_discovery_pub_->publish(msg);
  }

} // carma_guidance_plugin

