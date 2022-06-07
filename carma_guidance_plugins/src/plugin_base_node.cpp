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
#include <rclcpp/create_publisher.hpp>

#include "carma_guidance_plugins/plugin_base_node.hpp"

namespace carma_guidance_plugins
{
  namespace std_ph = std::placeholders;

  PluginBaseNode::PluginBaseNode(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {

    // Setup discovery timer to publish onto the plugin_discovery_pub
    discovery_timer_ = create_timer(
        get_clock(),
        std::chrono::milliseconds(500), // 2 Hz frequency to account for 1Hz maneuver planning frequency
        std::bind(&PluginBaseNode::discovery_timer_callback, this));
  }

  void PluginBaseNode::lazy_wm_initialization()
  {
    if (wm_listener_)
      return; // Already initialized
    

    wm_listener_ = std::make_shared<carma_wm::WMListener>(
        this->get_node_base_interface(), this->get_node_logging_interface(),
       this->get_node_topics_interface(), this->get_node_parameters_interface()
      );

    wm_ = wm_listener_->getWorldModel();
  }

  std::shared_ptr<carma_wm::WMListener> PluginBaseNode::get_world_model_listener()
  {
    lazy_wm_initialization();
    return wm_listener_;
  }

  carma_wm::WorldModelConstPtr PluginBaseNode::get_world_model()
  {
    lazy_wm_initialization();
    return wm_;
  }

  bool PluginBaseNode::get_activation_status() {
    // Determine the plugin activation state by checking which lifecycle state we are in. 
    // If we are active then the plugin is active otherwise the plugin is inactive
    return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  carma_ros2_utils::CallbackReturn PluginBaseNode::on_activate_plugin()
  {
    return carma_ros2_utils::CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn PluginBaseNode::on_deactivate_plugin()
  {
    return carma_ros2_utils::CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn PluginBaseNode::on_cleanup_plugin()
  {
    return carma_ros2_utils::CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn PluginBaseNode::on_shutdown_plugin()
  {
    return carma_ros2_utils::CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn PluginBaseNode::on_error_plugin(const std::string &)
  {
    // On error should default to failure so user must explicitly implement error handling to get any other behavior
    return carma_ros2_utils::CallbackReturn::FAILURE; 
  }

  carma_ros2_utils::CallbackReturn PluginBaseNode::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    return on_configure_plugin();
  }
  
  carma_ros2_utils::CallbackReturn PluginBaseNode::handle_on_activate(const rclcpp_lifecycle::State &)
  {
    return on_activate_plugin();
  }
  
  carma_ros2_utils::CallbackReturn PluginBaseNode::handle_on_deactivate(const rclcpp_lifecycle::State &)
  {
    return on_deactivate_plugin();
  }
  
  carma_ros2_utils::CallbackReturn PluginBaseNode::handle_on_cleanup(const rclcpp_lifecycle::State &)
  {
    return on_cleanup_plugin();
  }
  
  carma_ros2_utils::CallbackReturn PluginBaseNode::handle_on_shutdown(const rclcpp_lifecycle::State &)
  {
    return on_shutdown_plugin();
  }
  
  carma_ros2_utils::CallbackReturn PluginBaseNode::handle_on_error(const rclcpp_lifecycle::State &, const std::string &exception_string)
  {
    return on_error_plugin(exception_string);
  }

  uint8_t PluginBaseNode::get_type() 
  {
    // Base class returns unknown. 
    // Its expected that 2nd level extending classes (strategic, tactical, control) will return correct type by overriding
    return carma_planning_msgs::msg::Plugin::UNKNOWN;
  }

  void PluginBaseNode::discovery_timer_callback()
  {

    if (!plugin_discovery_pub_) // If the publisher has not been initalized then initialize it. 
    {
      // Setup plugin publisher discovery which should bypass lifecycle behavior to ensure plugins are found
      // prior to them needing to be activated.
      // NOTE: Any other topics which need to be setup in the future should use handle_on_configure and the default this->create_publisher method to get a lifecycle publisher instead
      auto this_shared = shared_from_this(); // Usage of shared_from_this() means this cannot be done in the constructor thus it is delegated to the timer callback
      plugin_discovery_pub_ = rclcpp::create_publisher<carma_planning_msgs::msg::Plugin>(this_shared, "plugin_discovery", 1);

    }

    carma_planning_msgs::msg::Plugin msg;
    msg.name = get_plugin_name();
    msg.version_id = get_version_id();
    msg.type = get_type();
    msg.available = get_availability();
    msg.activated = get_activation_status();
    msg.capability = get_capability();

    plugin_discovery_pub_->publish(msg);
  }

} // carma_guidance_plugins

