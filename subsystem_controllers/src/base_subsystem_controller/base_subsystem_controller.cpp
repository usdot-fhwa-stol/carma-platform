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
    system_alert_sub_ = create_subscription<carma_msgs::msg::SystemAlert>(
        system_alert_topic_, 100,
        std::bind(&BaseSubsystemController::on_system_alert, this, std::placeholders::_1));

    // TODO add validation that all required nodes are in namespace
    // TODO set the config here by loading parameters
    auto nodes_in_namespace = get_nodes_in_namespace(base_config_.subsystem_namespace);

    lifecycle_mgr_.set_managed_nodes(nodes_in_namespace);
  }

  void BaseSubsystemController::set_config(BaseSubSystemControllerConfig config)
  {
    base_config_ = config;
  }

  void BaseSubsystemController::on_system_alert(const carma_msgs::msg::SystemAlert::UniquePtr msg)
  {

    RCLCPP_INFO(
        get_logger(), "Received SystemAlert message of type: %u, msg: %s",
        msg->type, msg->description.c_str());

    // NOTE: Here we check the required nodes not the full node set
    if (msg->type == carma_msgs::msg::SystemAlert::FATAL)
    {

      // Required node has failed
      if (std::find(base_config_.required_subsystem_nodes.begin(), base_config_.required_subsystem_nodes.end(), msg->source_node) != base_config_.required_subsystem_nodes.end())
      {
        lifecycle_mgr_.shutdown(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms), false);
        // TODO publish new FATAL system alert describing that the subsystem has failed
      }
      else
      { // Optional node has failed
        // TODO publish a WARNING message
      }
    }
  }

  carma_ros2_utils::CallbackReturn BaseSubsystemController::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Subsystem trying to configure");

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

  std::vector<std::string> BaseSubsystemController::get_non_intersecting_set(const std::vector<std::string> &superset, const std::vector<std::string> &subset) const
  {
    // Super set is out namespace
    // Subset is our managed nodes
    std::vector<std::string> non_intersecting_set;
    non_intersecting_set.reserve(superset.size());

    std::unordered_set<std::string> subset_lookup;
    subset_lookup.reserve(subset.size());

    subset_lookup.insert(subset.begin(), subset.end());

    for (const auto &string : superset)
    {
      if (subset_lookup.find(string) == subset_lookup.end())
      { // No intersection so store
        non_intersecting_set.emplace_back(string);
      }
    }

    return non_intersecting_set;
  }

} // namespace subsystem_controllers
