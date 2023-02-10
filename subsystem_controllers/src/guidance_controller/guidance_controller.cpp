
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

#include <chrono>
#include "subsystem_controllers/guidance_controller/guidance_controller.hpp"

using std_msec = std::chrono::milliseconds;

namespace subsystem_controllers
{
  GuidanceControllerNode::GuidanceControllerNode(const rclcpp::NodeOptions &options)
      : BaseSubsystemController(options)
  {
      // Don't automatically trigger state transitions from base class on configure
      // In this class the managed nodes list first needs to be modified then the transition will be triggered manually
      trigger_managed_nodes_configure_from_base_class_ = false;
      config_.required_plugins = declare_parameter<std::vector<std::string>>("required_plugins", config_.required_plugins); 
      config_.auto_activated_plugins = declare_parameter<std::vector<std::string>>("auto_activated_plugins", config_.auto_activated_plugins); 
      config_.ros2_initial_plugins = declare_parameter<std::vector<std::string>>("ros2_initial_plugins", config_.ros2_initial_plugins); 
  }

  cr2::CallbackReturn GuidanceControllerNode::handle_on_configure(const rclcpp_lifecycle::State &prev_state) {
    auto base_return = BaseSubsystemController::handle_on_configure(prev_state);

    if (base_return != cr2::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Guidance Controller could not configure");
      return base_return;
    }

    config_ = GuidanceControllerConfig();

    auto base_managed_nodes = lifecycle_mgr_.get_managed_nodes();

    std::string plugin_namespace = base_config_.subsystem_namespace + "/plugins/";


    std::vector<std::string> guidance_plugin_nodes;
    // Extract the nodes under the plugin namespaces (ie. /guidance/plugins/)
    std::copy_if(base_managed_nodes.begin(), base_managed_nodes.end(),
      std::back_inserter(guidance_plugin_nodes),
      [plugin_namespace](const std::string& s) { return s.rfind(plugin_namespace, 0) == 0; });

    std::vector<std::string> non_plugin_guidance_nodes = get_non_intersecting_set(base_managed_nodes, guidance_plugin_nodes);

    lifecycle_mgr_.set_managed_nodes(non_plugin_guidance_nodes);

    // Load required plugins and default enabled plugins
    get_parameter<std::vector<std::string>>("required_plugins", config_.required_plugins); 
    get_parameter<std::vector<std::string>>("auto_activated_plugins", config_.auto_activated_plugins); 
    get_parameter<std::vector<std::string>>("ros2_initial_plugins", config_.ros2_initial_plugins); 

    RCLCPP_INFO_STREAM(get_logger(), "Config: " << config_);

    // The core need is that plugins need to be managed separately from guidance nodes
    auto plugin_lifecycle_manager = std::make_shared<ros2_lifecycle_manager::Ros2LifecycleManager>(
      get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(), get_node_services_interface());

    plugin_manager_ = std::make_shared<PluginManager>(
      config_.required_plugins, 
      config_.auto_activated_plugins,
      config_.ros2_initial_plugins, 
      plugin_lifecycle_manager, 
      [this](){ return get_current_state().id(); },
      [this](auto node, auto ns){ return get_service_names_and_types_by_node(node, ns); },
      std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms)
    );

    plugin_discovery_sub_ = create_subscription<carma_planning_msgs::msg::Plugin>(
      "plugin_discovery", 50,
      std::bind(&PluginManager::update_plugin_status, plugin_manager_, std::placeholders::_1));

    get_registered_plugins_server_ = create_service<carma_planning_msgs::srv::PluginList>(
        "plugins/get_registered_plugins",
        std::bind(&PluginManager::get_registered_plugins, plugin_manager_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    get_active_plugins_server_ = create_service<carma_planning_msgs::srv::PluginList>(
        "plugins/get_active_plugins",
        std::bind(&PluginManager::get_active_plugins, plugin_manager_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    activate_plugin_server_ = create_service<carma_planning_msgs::srv::PluginActivation>(
        "plugins/activate_plugin", // TODO check topic name
        std::bind(&PluginManager::activate_plugin, plugin_manager_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    get_strategic_plugins_by_capability_server_ = create_service<carma_planning_msgs::srv::GetPluginApi>(
        "plugins/get_strategic_plugins_by_capability",
        std::bind(&PluginManager::get_strategic_plugins_by_capability, plugin_manager_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    get_tactical_plugins_by_capability_server_ = create_service<carma_planning_msgs::srv::GetPluginApi>(
        "plugins/get_tactical_plugins_by_capability",
        std::bind(&PluginManager::get_tactical_plugins_by_capability, plugin_manager_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    get_control_plugins_by_capability_server_ = create_service<carma_planning_msgs::srv::GetPluginApi>(
        "plugins/get_control_plugins_by_capability",
        std::bind(&PluginManager::get_control_plugins_by_capability, plugin_manager_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));



    // With all of our non-plugin managed nodes now being tracked we can execute their configure operations
    bool success = lifecycle_mgr_.configure(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms)).empty();

    // Configure our plugins
    try {
      plugin_manager_->configure(); // Only checking required nodes. Other node failure tracked by activity status
    } catch(const std::runtime_error& e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error configuring plugin manager " << e.what());
      success = false;
    }
    


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

  }

  cr2::CallbackReturn GuidanceControllerNode::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {
    auto base_return = BaseSubsystemController::handle_on_activate(prev_state); // This will activate all nodes in the namespace TODO what about the plugins?

    if (base_return != cr2::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Guidance Controller could not activate");
      return base_return;
    }

    bool success = true;
    try {
      plugin_manager_->activate(); // Only checking required nodes. Other node failure tracked by activity status
    } catch(const std::runtime_error& e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error activating plugin manager " << e.what());
      success = false;
    }

    if (success)
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem able to activate");
      return CallbackReturn::SUCCESS;
    }
    else
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem unable to activate");
      return CallbackReturn::FAILURE;
    }

  }

  cr2::CallbackReturn GuidanceControllerNode::handle_on_deactivate(const rclcpp_lifecycle::State &prev_state)
  {
    auto base_return = BaseSubsystemController::handle_on_deactivate(prev_state);

    if (base_return != cr2::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Guidance Controller could not deactivate");
      return base_return;
    }

    bool success = true;
    try {
      plugin_manager_->deactivate(); // Only checking required nodes. Other node failure tracked by activity status
    } catch(const std::runtime_error& e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error deactivating plugin manager " << e.what());
      success = false;
    }

    if (success)
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem able to deactivate");
      return CallbackReturn::SUCCESS;
    }
    else
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem unable to deactivate");
      return CallbackReturn::FAILURE;
    }
  }

  cr2::CallbackReturn GuidanceControllerNode::handle_on_cleanup(const rclcpp_lifecycle::State &prev_state)
  {
    auto base_return = BaseSubsystemController::handle_on_cleanup(prev_state);

    if (base_return != cr2::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Guidance Controller could not cleanup");
      return base_return;
    }

    bool success = true;
    try {
      plugin_manager_->cleanup(); // Only checking required nodes. Other node failure tracked by activity status
    } catch(const std::runtime_error& e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error cleaning up plugin manager " << e.what());
      success = false;
    }

    if (success)
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem able to cleanup");
      return CallbackReturn::SUCCESS;
    }
    else
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem unable to cleanup");
      return CallbackReturn::FAILURE;
    }
  }

  cr2::CallbackReturn GuidanceControllerNode::handle_on_shutdown(const rclcpp_lifecycle::State &prev_state)
  {
    auto base_return = BaseSubsystemController::handle_on_shutdown(prev_state);

    if (base_return != cr2::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Guidance Controller could not shutdown");
      return base_return;
    }

    bool success = plugin_manager_->shutdown();

    if (success)
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem able to shutdown cleanly");
      return CallbackReturn::SUCCESS;
    }
    else
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem unable to shutdown cleanly");
      return CallbackReturn::FAILURE;
    }
  }


} // namespace subsystem_controllers

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(subsystem_controllers::GuidanceControllerNode)
