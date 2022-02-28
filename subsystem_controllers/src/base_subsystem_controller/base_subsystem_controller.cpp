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
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>

using std_msec = std::chrono::milliseconds;

namespace subsystem_controllers
{
  BaseSubsystemController::BaseSubsystemController(const rclcpp::NodeOptions &options)
      : CarmaLifecycleNode(options), 
      lifecycle_mgr_(get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(), get_node_services_interface())
  {
    // Declare parameters
    base_config_.service_timeout_ms = this->declare_parameter<int>("service_timeout_ms", base_config_.service_timeout_ms);
    base_config_.call_timeout_ms = this->declare_parameter<int>("call_timeout_ms", base_config_.call_timeout_ms);
    base_config_.required_subsystem_nodes = this->declare_parameter<std::vector<std::string>>("required_subsystem_nodes", base_config_.required_subsystem_nodes);
    base_config_.subsystem_namespace = this->declare_parameter<std::string>("subsystem_namespace", base_config_.subsystem_namespace);
    base_config_.full_subsystem_required = this->declare_parameter<bool>("full_subsystem_required", base_config_.full_subsystem_required);
    base_config_.unmanaged_required_nodes = this->declare_parameter<std::vector<std::string>>("unmanaged_required_nodes", base_config_.unmanaged_required_nodes);
  
    // Handle fact that parameter vectors cannot be empty
    if (base_config_.required_subsystem_nodes.size() == 1 && base_config_.required_subsystem_nodes[0].empty()) {
      base_config_.required_subsystem_nodes.clear();
    }

    if (base_config_.unmanaged_required_nodes.size() == 1 && base_config_.unmanaged_required_nodes[0].empty()) {
      base_config_.unmanaged_required_nodes.clear();
    }

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
      if (std::find(base_config_.required_subsystem_nodes.begin(), base_config_.required_subsystem_nodes.end(), msg->source_node) != base_config_.required_subsystem_nodes.end()
          || std::find(base_config_.unmanaged_required_nodes.begin(), base_config_.unmanaged_required_nodes.end(), msg->source_node) != base_config_.unmanaged_required_nodes.end())
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

    // Reset config 
    base_config_ = BaseSubSystemControllerConfig();

    // Load Parameters
    get_parameter<int>("service_timeout_ms", base_config_.service_timeout_ms);
    get_parameter<int>("call_timeout_ms", base_config_.call_timeout_ms);
    get_parameter<std::vector<std::string>>("required_subsystem_nodes", base_config_.required_subsystem_nodes);
    get_parameter<std::string>("subsystem_namespace", base_config_.subsystem_namespace);
    get_parameter<bool>("full_subsystem_required", base_config_.full_subsystem_required);
    get_parameter<std::vector<std::string>>("unmanaged_required_nodes", base_config_.unmanaged_required_nodes);

    // Handle fact that parameter vectors cannot be empty
    if (base_config_.required_subsystem_nodes.size() == 1 && base_config_.required_subsystem_nodes[0].empty()) {
      base_config_.required_subsystem_nodes.clear();
    }

    if (base_config_.unmanaged_required_nodes.size() == 1 && base_config_.unmanaged_required_nodes[0].empty()) {
      base_config_.unmanaged_required_nodes.clear();
    }

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
  }

  carma_ros2_utils::CallbackReturn BaseSubsystemController::handle_on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Subsystem trying to activate");

    bool success = lifecycle_mgr_.activate(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms)).empty();

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

    bool success = lifecycle_mgr_.deactivate(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms)).empty();

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

    bool success = lifecycle_mgr_.cleanup(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms)).empty();

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

    bool success = lifecycle_mgr_.shutdown(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms)).empty();

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

    bool success = lifecycle_mgr_.shutdown(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms), false).empty();

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

    // These two services are exposed by lifecycle nodes. 
    static const std::string CHANGE_STATE_SRV = "/change_state";
    static const std::string GET_STATE_SRV = "/get_state";

    static const std::string CHANGE_STATE_TYPE = "lifecycle_msgs/srv/ChangeState";
    static const std::string GET_STATE_TYPE = "lifecycle_msgs/srv/GetState";

    static const std::string this_full_name =  std::string(get_namespace()) + "/" + std::string(get_name());


    auto all_nodes = this->get_node_names();

    std::vector<std::string> nodes_in_namspace;
    nodes_in_namspace.reserve(all_nodes.size());


    for (const auto &node : all_nodes)
    {

      if (node == this_full_name)
        continue; // no need for this node to try managing itself

      if (node.find(node_namespace) == 0)
      { // The node is in the provided namespace


        ////
        // In the following section of code we check if the current node in the target namespace is actually a lifecycle node
        // This check is done by evaluating if that nodes exposes the change_state and get_state lifecycle services. 
        // If the node does not expose these services then it cannot be managed by this component. 
        // However, this does not result in an error as the node could be wrapped by a lifecycle component wrapper
        ////

        // First extract the service names and types for the current node. This requires seperating the node base name and namespace
        std::vector<std::string> result;
        boost::split(result, node, boost::is_any_of("/"));

        if (result.empty()) {
          throw std::runtime_error("Fully specified node name does not contain '/' seems to suggest internal ROS error.");
        }

        std::string base_name = result.back();
        result.pop_back();
        std::string namespace_joined = boost::algorithm::join(result, "/");

        auto services_and_types = get_service_names_and_types_by_node(base_name, namespace_joined);


        // Next we check if both services are available with the correct type
        // Short variable names used here to make conditional more readable
        const std::string cs_srv = node + CHANGE_STATE_SRV;
        const std::string gs_srv = node + GET_STATE_SRV;

        if (services_and_types.find(cs_srv) != services_and_types.end() 
          && services_and_types.find(gs_srv) != services_and_types.end()
          && std::find(services_and_types.at(cs_srv).begin(), services_and_types.at(cs_srv).end(), CHANGE_STATE_TYPE) != services_and_types.at(cs_srv).end()
          && std::find(services_and_types.at(gs_srv).begin(), services_and_types.at(gs_srv).end(), GET_STATE_TYPE) != services_and_types.at(gs_srv).end())
        {

          nodes_in_namspace.emplace_back(node);

        } else {
          // Current node is not a lifecycle node so log a warning
          RCLCPP_WARN_STREAM(get_logger(), "Failed to find lifecycle services for node: " << node << " this node will not be managed. NOTE: If this node is wrapped by a lifecycle component that is managed then this is not an issue.");
          
          continue;

        }
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
