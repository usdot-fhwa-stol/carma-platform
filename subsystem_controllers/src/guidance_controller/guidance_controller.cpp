
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
      : BaseSubsystemController(options),
      // Don't automatically trigger state transitions from base class on configure
      // In this class the managed nodes list first needs to be modified then the transition will be triggered manually
      trigger_managed_nodes_configure_from_base_class_ = false;
  {}

  cr2::CallbackReturn GuidanceControllerNode::handle_on_configure(const rclcpp_lifecycle::State &) {
    auto base_return = BaseSubsystemController::handle_on_configure(prev_state);

    if (base_return != cr2::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Guidance Controller could not configure");
      return base_return;
    }

    config_ = Config();

    auto base_managed_nodes = lifecycle_mgr_.get_managed_nodes();

    std::string plugin_namespace = base_config_.subsystem_namespace + "/plugins/"


    std::vector<std::string> guidance_plugin_nodes;
    // Extract the nodes under the plugin namespaces (ie. /guidance/plugins/)
    std::copy_if(base_managed_nodes.begin(), base_managed_nodes.end(),
      std::back_inserter(guidance_plugin_nodes),
      [](const std::string& s) { return s.rfind(plugin_namespace, 0) == 0; });

    std::vector<std::string> non_plugin_guidance_nodes = get_non_intersecting_set(base_managed_nodes, guidance_plugin_nodes);

    lifecycle_mgr_.set_managed_nodes(non_plugin_guidance_nodes);

    // Load required plugins and default enabled plugins
    get_parameter<std::vector<std::string>>("required_plugins", config_.required_plugins); 
    get_parameter<std::vector<std::string>>("auto_activated_plugins", config_.auto_activated_plugins); 

    
    // The core need is that plugins need to be managed separately from guidance nodes
    // Specifically every time a plugin is added it needs to be brought to the inactive state
    // If the plugin is inactive it needs to be made active
    plugin_lifecycle_mgr_.set_managed_nodes();

    // Bringup method
    // On deactivate we don't want to cause failure if its already deactivated



    // With all of our non-plugin managed nodes now being tracked we can execute their configure operations
    bool success = lifecycle_mgr_.configure(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms)).empty();

    if (success)
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem able to configure");
      return CallbackReturn::SUCCESS;
    }
    else
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem unable to configure");
      return CallbackReturn::FAILURE;
    }


    // The core process for the guidance controller should be as follows
    // On startup identify all nodes in the guidance namespace
    // All nodes which are in guidance but NOT under /guidance/plugins will be managed by base_subsystem_controller
    // All nodes under the /guidance/plugins/ namespace will be managed by the plugin manager 
    // - Only plugins listed as required or auto-start will be activated along with the controller
    // - All other plugins will be identified via the plugin_discovery and tracked  

    // TODO also in the system_controller or base_subsystem_controller we need to publish a system alert on startup failure so we can see the exception in the UI
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
        std::bind(&PluginManager::get_active_plugins, plugin_manager_, std::placeholders::_1, std::placeholders::_2)); // TODO this points to the wrong function

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
