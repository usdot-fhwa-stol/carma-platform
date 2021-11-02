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

#include <unordered_set>
#include "subsystem_controllers/base_subsystem_controller/base_subsystem_controller.hpp"
#include "subsystem_controllers/base_subsystem_controller/base_subsystem_controller_config.hpp"

using std_msec = std::chrono::milliseconds;

namespace subsystem_controllers
{
  BaseSubsystemController::BaseSubsystemController(const rclcpp::NodeOptions &options)
      : CarmaLifecycleNode(options), 
      lifecycle_mgr_(get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(), get_node_services_interface())
  {
    
  }

  void BaseSubsystemController::set_config(BaseSubSystemControllerConfig config)
  {
    base_config_ = config;
  }

  void BaseSubsystemController::on_system_alert(carma_msgs::msg::SystemAlert::UniquePtr msg)
  {


    RCLCPP_INFO_STREAM(
          get_logger(), "Received SystemAlert message of type: " << static_cast<int>(msg->type) << " with message: " << msg->description);

    // NOTE: Here we check the required nodes not the full managed node set
    if (msg->type == carma_msgs::msg::SystemAlert::FATAL)
    {

      // Required node has failed
      if (std::find(base_config_.required_subsystem_nodes.begin(), base_config_.required_subsystem_nodes.end(), msg->source_node) != base_config_.required_subsystem_nodes.end())
      {
        // Our subsystem has failed so notify the overall system controller

        RCLCPP_ERROR_STREAM(
          get_logger(), "Failure in required node: " << msg->source_node);

        carma_msgs::msg::SystemAlert alert;
        alert.type = carma_msgs::msg::SystemAlert::FATAL;
        alert.description = base_config_.subsystem_namespace + " subsytem has failed with error: " + msg->description;
        alert.source_node =  get_node_base_interface()->get_fully_qualified_name();
        publish_system_alert(alert);

        // TODO: It might be worth trying to deactivate or shutdown after alerting the larger system, 
        //       but not clear on if that will increase instability of shutdown process
      }
      else
      { // Optional node has failed
        RCLCPP_WARN_STREAM(
          get_logger(), "Failure in optional node: " << msg->source_node);
      }
    }
  }

  carma_ros2_utils::CallbackReturn BaseSubsystemController::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Subsystem trying to configure");

    // Load Parameters
    base_config_.service_timeout_ms = this->declare_parameter<int64_t>("service_timeout_ms", base_config_.service_timeout_ms);
    base_config_.call_timeout_ms = this->declare_parameter<int64_t>("call_timeout_ms", base_config_.call_timeout_ms);
    base_config_.required_subsystem_nodes = this->declare_parameter<std::vector<std::string>>("required_subsystem_nodes", base_config_.required_subsystem_nodes);
    base_config_.subsystem_namespace = this->declare_parameter<std::string>("subsystem_namespace", base_config_.subsystem_namespace);
    base_config_.full_subsystem_required = this->declare_parameter<bool>("full_subsystem_required", base_config_.full_subsystem_required);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded config: " << base_config_);

    // Create subscriptions
    system_alert_sub_ = create_subscription<carma_msgs::msg::SystemAlert>(
        system_alert_topic_, 100,
        std::bind(&BaseSubsystemController::on_system_alert, this, std::placeholders::_1));

    // Initialize lifecycle manager
    auto nodes_in_namespace = get_nodes_in_namespace(base_config_.subsystem_namespace);

    std::vector<std::string> managed_nodes = nodes_in_namespace;
    managed_nodes.reserve(nodes_in_namespace.size() + base_config_.required_subsystem_nodes.size());

    // Collect non-namespace nodes if any and add them to our set of managed nodes
    auto additional_nodes = get_non_intersecting_set(base_config_.required_subsystem_nodes, nodes_in_namespace);

    managed_nodes.insert(managed_nodes.end(), additional_nodes.begin(), additional_nodes.end());

    lifecycle_mgr_.set_managed_nodes(managed_nodes);

    // Check if all nodes are required
    if (base_config_.full_subsystem_required) {
      RCLCPP_INFO_STREAM(get_logger(), "full_subsystem_required is True. Setting all namespace nodes as required");

      base_config_.required_subsystem_nodes = managed_nodes;

      RCLCPP_INFO_STREAM(get_logger(), "New config: " << base_config_);
    }
    

    // With all of our managed nodes now being tracked we can execute their configure operations
    bool success = lifecycle_mgr_.configure(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms));

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

  carma_ros2_utils::CallbackReturn BaseSubsystemController::handle_on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Subsystem trying to activate");

    bool success = lifecycle_mgr_.activate(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms));

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

  carma_ros2_utils::CallbackReturn BaseSubsystemController::handle_on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Subsystem trying to deactivate");

    bool success = lifecycle_mgr_.deactivate(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms));

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

  carma_ros2_utils::CallbackReturn BaseSubsystemController::handle_on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Subsystem trying to cleanup");

    bool success = lifecycle_mgr_.cleanup(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms));

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

  carma_ros2_utils::CallbackReturn BaseSubsystemController::handle_on_error(const rclcpp_lifecycle::State &, const std::string &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Subsystem trying to shutdown due to error");

    bool success = lifecycle_mgr_.shutdown(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms));

    if (success)
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem able to shutdown");
      return CallbackReturn::SUCCESS;
    }
    else
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem unable to shutdown");
      return CallbackReturn::FAILURE;
    }
  }

  carma_ros2_utils::CallbackReturn BaseSubsystemController::handle_on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Subsystem trying to shutdown");

    bool success = lifecycle_mgr_.shutdown(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms), false);

    if (success)
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem able to shutdown");
      return CallbackReturn::SUCCESS;
    }
    else
    {

      RCLCPP_INFO_STREAM(get_logger(), "Subsystem unable to shutdown");
      return CallbackReturn::FAILURE;
    }
  }

  std::vector<std::string> BaseSubsystemController::get_nodes_in_namespace(const std::string &node_namespace) const
  {

    auto all_nodes = this->get_node_names();

    std::vector<std::string> nodes_in_namspace;
    nodes_in_namspace.reserve(all_nodes.size());

    for (const auto &node : all_nodes)
    {
      if (node.find(node_namespace) == 0)
      { // The node is in the provided namespace
        nodes_in_namspace.emplace_back(node);
      }
    }

    return nodes_in_namspace;
  }

  std::vector<std::string> BaseSubsystemController::get_non_intersecting_set(const std::vector<std::string> &set_a, const std::vector<std::string> &set_b) const
  {

    std::vector<std::string> non_intersecting_set; // Returned set
    non_intersecting_set.reserve(set_a.size());

    std::unordered_set<std::string> set_b_lookup; // Quick lookup map of set_b
    set_b_lookup.reserve(set_b.size());

    set_b_lookup.insert(set_b.begin(), set_b.end());

    for (const auto &string : set_a)
    {
      if (set_b_lookup.find(string) == set_b_lookup.end())
      { // No intersection so store
        non_intersecting_set.emplace_back(string);
      }
    }

    return non_intersecting_set;
  }

} // namespace subsystem_controllers
