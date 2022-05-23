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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <carma_planning_msgs/msg/plugin.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace carma_guidance_plugin
{

  /**
   * \brief PluginBaseNode provides default functionality for all carma guidance plugins.
   *        This includes basic state machine management (largely delegated to lifecycle behavior), required interfaces, and plugin discovery
   * 
   */
  class PluginBaseNode : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Publishers
    carma_ros2_utils::PubPtr<carma_planning_msgs::msg::Plugin> plugin_discovery_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr discovert_timer_;

  public:
    /**
     * \brief PluginBaseNode constructor 
     */
    explicit PluginBaseNode(const rclcpp::NodeOptions &);

    // TODO do I need the virtual destructor?

    /**
     * \brief Callback for the plugin discovery timer which will publish the plugin discovery message
     */
    void discovery_timer_callback();

    // TODO comments
    bool get_activation_status();

    virtual uint8_t get_type();

    virtual bool get_availability() = 0;
    virtual std::string get_capability() = 0;
    virtual std::string get_name() = 0;
    virtual std::string get_version_id() = 0;

    virtual carma_ros2_utils::CallbackReturn configure_plugin() = 0;
    virtual carma_ros2_utils::CallbackReturn activate_plugin() = 0;
    virtual carma_ros2_utils::CallbackReturn deactivate_plugin() = 0;

    ////
    // Overrides
    ////
    // For simplicity of managing the plugin state machine all lifecycle callbacks are implemented as final by the core extending classes (strategic, tactical, control)
    // Plugins will use the simplified plugin callbacks via the template pattern 
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &) override;
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &) override;
    carma_ros2_utils::CallbackReturn handle_on_deactivate(const rclcpp_lifecycle::State &) override;
    carma_ros2_utils::CallbackReturn handle_on_shutdown(const rclcpp_lifecycle::State &) override; 
    carma_ros2_utils::CallbackReturn handle_on_error(const rclcpp_lifecycle::State &) override;
    
  };

} // carma_guidance_plugin
