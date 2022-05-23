
/*
 * Copyright (C) 2021 LEIDOS.
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

#include "subsystem_controllers/guidance_controller/guidance_controller.hpp"

namespace subsystem_controllers
{
  GuidanceControllerNode::GuidanceControllerNode(const rclcpp::NodeOptions &options)
      : BaseSubsystemController(options)
  {
  }

  cr2::CallbackReturn GuidanceControllerNode::handle_on_configure(const rclcpp_lifecycle::State &) {
    auto base_return = BaseSubsystemController::handle_on_configure(prev_state);

    if (base_return != cr2::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Guidance Controller could not configure");
      return base_return;
    }

    // TODO load parameters

    plugin_manager_ = std::make_shared<PluginManager>(required_plugins_, plugin_service_prefix_, strategic_plugin_service_suffix_, tactical_plugin_service_suffix_);

    plugin_discovery_sub_ = create_subscription<carma_planning_msgs::msg::Plugin>(
      "plugin_discovery", 50,
      std::bind(&PluginManager::update_plugin_status, plugin_manager_, std::placeholders::_1));

    get_registered_plugins_server_ = create_service(
        "plugins/get_registered_plugins",
        std::bind(&PluginManager::get_registered_plugins, plugin_manager_, std::placeholders::_1, std::placeholders::_2));

    get_active_plugins_server_ = create_service(
        "plugins/get_active_plugins",
        std::bind(&PluginManager::get_active_plugins, plugin_manager_, std::placeholders::_1, std::placeholders::_2));

    activate_plugin_server_ = create_service(
        "plugins/get_active_plugins",
        std::bind(&PluginManager::get_active_plugins, plugin_manager_, std::placeholders::_1, std::placeholders::_2));

    get_strategic_plugin_by_capability_server_ = create_service(
        "plugins/get_strategic_plugin_by_capability",
        std::bind(&PluginManager::get_strategic_plugin_by_capability, plugin_manager_, std::placeholders::_1, std::placeholders::_2));

    get_tactical_plugin_by_capability_server_ = create_service(
        "plugins/get_tactical_plugin_by_capability",
        std::bind(&PluginManager::get_tactical_plugin_by_capability, plugin_manager_, std::placeholders::_1, std::placeholders::_2));


    return cr2::CallbackReturn::SUCCESS;
  }

  cr2::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &)
  {
    auto base_return = BaseSubsystemController::handle_on_activate(prev_state); // This will activate all nodes in the namespace TODO what about the plugins?

    if (base_return != cr2::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Guidance Controller could not activate");
      return base_return;
    }

    // In activate we want to activate all plugins which are required if they haven't already been activated by base class
    // We want to skip all plugins which are not required
    // TODO it seems like we may want some sort of start activated configuration for plugins as well

    plugin_manager_->

  }

  cr2::CallbackReturn handle_on_deactivate(const rclcpp_lifecycle::State &)
  {
    auto base_return = BaseSubsystemController::handle_on_deactivate(prev_state);

    if (base_return != cr2::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Guidance Controller could not deactivate");
      return base_return;
    }

    // Iterate over all plugins to deactivate them
    plugin_manager_.get_registered_plugins();
  }


} // namespace subsystem_controllers

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(subsystem_controllers::GuidanceControllerNode)
