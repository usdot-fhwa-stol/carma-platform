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

#include "ros2_lifecycle_manager/ros2_lifecycle_manager.hpp"

namespace ros2_lifecycle_manager
{

  Ros2LifecycleManager::Ros2LifecycleManager(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
      rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services
    ) : node_base_(node_base), node_graph_(node_graph), node_logging_(node_logging), node_services_(node_services) {
      service_callback_group_ = node_base_->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
    }

  void Ros2LifecycleManager::set_managed_nodes(const std::vector<std::string> &nodes)
  {

    managed_node_names_ = nodes; // Store node names

    // Recreate and store ManagedNodes instances
    managed_nodes_.clear();
    managed_nodes_.reserve(nodes.size());
    node_map_.clear();
    node_map_.reserve(nodes.size());

    size_t i = 0;
    for (const auto &node : nodes)
    {

      ManagedNode managed_node(node,
                               create_client<lifecycle_msgs::srv::ChangeState>(node + change_state_topic_),
                               create_client<lifecycle_msgs::srv::GetState>(node + get_state_topic_));

      managed_nodes_.push_back(managed_node);
      node_map_.emplace(node, i);
      i++;
    }
  }

  std::vector<std::string> Ros2LifecycleManager::get_managed_nodes()
  {
    return managed_node_names_;
  }

  uint8_t Ros2LifecycleManager::get_managed_node_state(const std::string &node_name)
  {
    auto it = node_map_.find(node_name);
    if (it == node_map_.end()) // Check if the requested node is being managed
    {

      RCLCPP_ERROR_STREAM(
          node_logging_->get_logger(), "State for node: " << node_name << " could not be provided as that node was not being managed. ");

      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto node = managed_nodes_.at((*it).second);

    // Check for service. If not read return unknown
    if (!node.get_state_client->service_is_ready()) 
    {
      RCLCPP_ERROR_STREAM(
          node_logging_->get_logger(), "State for node: " << node_name << " could not be provided as that node's service is not ready ");
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    // Send request
    auto future_result = node.get_state_client->async_send_request(request);

    auto future_status = future_result.wait_for(std_nanosec(10000000L)); // 10 millisecond delay

    if (future_status != std::future_status::ready)
    {
      RCLCPP_ERROR_STREAM(
          node_logging_->get_logger(), "Server time out while getting current state for node with name: " << node_name);
      return false;
    }

    return future_result.get()->current_state.id;
  }

  bool Ros2LifecycleManager::configure(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered)
  {
    return transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, ordered, connection_timeout, call_timeout);
  }

  bool Ros2LifecycleManager::cleanup(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered)
  {
    return transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, ordered, connection_timeout, call_timeout);
  }

  bool Ros2LifecycleManager::activate(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered)
  {
    return transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, ordered, connection_timeout, call_timeout);
  }

  bool Ros2LifecycleManager::deactivate(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered)
  {
    return transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, ordered, connection_timeout, call_timeout);
  }

  bool Ros2LifecycleManager::shutdown(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered)
  {
    // To avoid having to track all the node sates and pick the appropriate shutdown we will simply shut down in order of most dangeorus
    // Active is shutdown first to avoid more data being published, this is followed by inactive and finally unconfigured
    bool success = false;
    success = success || transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN, ordered, connection_timeout, call_timeout);
    success = success || transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN, ordered, connection_timeout, call_timeout);
    success = success || transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, ordered, connection_timeout, call_timeout);
    return success;
    // transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_DESTROY, ordered, connection_timeout, call_timeout); // TODO not clear if this is needed yet
  }

  bool Ros2LifecycleManager::transition_multiplex(uint8_t transition, bool ordered, const std_nanosec &connection_timeout, const std_nanosec &call_timeout)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    lifecycle_msgs::msg::Transition t_msg;
    t_msg.id = transition;
    request->transition = t_msg;
    bool clean_result = true;

    // If ordered argument was true then we will transition the nodes in sequence 
    if (ordered)
    {
      // Iterate over each node and transition them
      for (auto node : managed_nodes_)
      {

        // Wait for service
        if (!waitForService<lifecycle_msgs::srv::ChangeState>(node.change_state_client, connection_timeout))
        {
          clean_result = false;
          continue; 
        }

        RCLCPP_INFO_STREAM(
          node_logging_->get_logger(), "Calling node: " << node.node_name);

        // Call service
        ChangeStateSharedFutureWithRequest future_result = node.change_state_client->async_send_request(request, [](ChangeStateSharedFutureWithRequest) {});

        // Wait for response
        if (!wait_on_change_state_future(future_result, call_timeout))
        { 
          clean_result = false;
        }
      }
    }
    else // If ordered was not true then we shall call all the nodes first and then wait for their responses
    {
      std::vector<ChangeStateSharedFutureWithRequest> futures;
      futures.reserve(managed_nodes_.size());

      for (auto node : managed_nodes_)
      {
        // Wait for service
        if (!waitForService<lifecycle_msgs::srv::ChangeState>(node.change_state_client, connection_timeout))
        {
          clean_result = false;
          continue; 
        }

        RCLCPP_INFO_STREAM(
          node_logging_->get_logger(), "Calling node a-sync: " << node.node_name);

        // Call service
        futures.emplace_back(node.change_state_client->async_send_request(request, [](ChangeStateSharedFutureWithRequest) {}));
      }
      for (auto future : futures)
      {
        if (!wait_on_change_state_future(future, call_timeout))
        { 
          clean_result = false;
        }
      }
    }

    return clean_result;
  }

  bool Ros2LifecycleManager::wait_on_change_state_future(const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFutureWithRequest &future,
                                   const std_nanosec &timeout)
  {
    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    RCLCPP_ERROR(
          node_logging_->get_logger(), "Waiting for future");
    auto future_status = future.wait_for(timeout);

    if (future_status != std::future_status::ready)
    {
      RCLCPP_ERROR(
          node_logging_->get_logger(), "Server time out while getting current state for node");
      return false;
    }

    // We have an answer, let's print our success.
    if (future.get().second->success)
    {
      RCLCPP_INFO(
          node_logging_->get_logger(), "Transition %d successfully triggered.", static_cast<int>(future.get().first->transition.id));
      return true;
    }
    else
    {
      RCLCPP_WARN(
          node_logging_->get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(future.get().first->transition.id));
      return false;
    }

    return true;
  }

} // namespace ros2_lifecycle_manager
