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

  void Ros2LifecycleManager::set_managed_nodes(const std::vector<std::string> &nodes) = 0
  {

    managed_node_names_ = nodes;

    managed_nodes_.clear();
    managed_nodes_.reserve(nodes.size());

    for (const auto &node : nodes)
    {

      ManagedNode managed_node(node,
                               this->create_client<lifecycle_msgs::srv::ChangeState>(node + change_state_topic_),
                               this->create_client<lifecycle_msgs::srv::ChangeState>(node + get_state_topic_));

      managed_nodes_.push_back(managed_node);
    }
  }

  std::vector<std::stirng> Ros2LifecycleManager::get_managed_nodes()
  {
    return managed_node_names_;
  }

  uint8_t get_managed_node_state(const std::string &node)
  {

    if (managed_nodes_.find(node) == managed_nodes_.end())
    {

      RCLCPP_ERROR_STREAM(
          get_logger(), "State for node: " << node << " could not be provided as that node was not being managed. ");

      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if (!waitForService<lifecycle_msgs::srv::GetState>(node.change_state_client, service_timeout))
    {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We send the request with the transition we want to invoke.
    auto future_result = client->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future_result, call_timeout);

    if (future_status != std::future_status::ready)
    {
      RCLCPP_ERROR(
          get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
      return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success)
    {
      RCLCPP_INFO(
          get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
      return true;
    }
    else
    {
      RCLCPP_WARN(
          get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
      return false;
    }
  }

  // TODO cleanup timeout usage in API
  // If ordered is true then the calls will execute in sequence as defined by set_managed_nodes. If false then no ordering is enforced (though it still may be)
  bool Ros2LifecycleManager::configure(const std::chrono::nanoseconds &timeout, bool ordered = true)
  {
    transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, ordered, timeout);
  }

  bool Ros2LifecycleManager::cleanup(const std::chrono::nanoseconds &timeout, bool ordered = true)
  {
    transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, ordered, timeout);
  }

  bool Ros2LifecycleManager::activate(const std::chrono::nanoseconds &timeout, bool ordered = true)
  {
    transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, ordered, timeout);
  }

  bool Ros2LifecycleManager::deactivate(const std::chrono::nanoseconds &timeout, bool ordered = true)
  {
    transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, ordered, timeout);
  }

  bool Ros2LifecycleManager::shutdown(const std::chrono::nanoseconds &timeout, bool ordered = true)
  {
    // To avoid having to track all the node sates and pick the appropriate shutdown we will simply shut down in order of most dangeorus
    // Active is shutdown first to avoid more data being published, this is followed by inactive and finally unconfigured
    transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN, ordered, timeout);
    transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN, ordered, timeout);
    transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, ordered, timeout);
    // transition_multiplex(lifecycle_msgs::msg::Transition::TRANSITION_DESTROY, ordered, service_timeout_, call_timeout_); // TODO not clear if this is needed yet
  }

  bool transition_multiplex(uint8_t transition, bool ordered, const std::chrono::nanoseconds &timeout)
  {
    lifecycle_msgs::srv::ChangeState request;
    request.transition = transition;
    bool clean_result = true;
    rclcpp::Duration allowed_duration(timeout);
    auto start_time = rclcpp::Time::now();
    if (ordered)
    {
      // When operating
      for (auto node : managed_nodes_)
      {
        // Check if the timeout has expired
        auto elapsed_time = rclcpp::Time::now() - start_time;
        if (elapsed_time >= allowed_duration)
        {
          clean_result = false;
          break;
        }

        // Wait for service
        if (!waitForService<lifecycle_msgs::srv::ChangeState>(node.change_state_client, (allowed_duration - elapsed_time).nanoseconds())
        {
          clean_result = false;
          continue; 
        }
        // Call service
        auto future_result = node.change_state_client->async_send_request(request);

        // Wait for response
        elapsed_time = rclcpp::Time::now() - start_time;
        if (!wait_on_change_state_future(future_result, (allowed_duration - elapsed_time).nanoseconds())) { 
        }
      }
    }
    else
    {
      std::vector<rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFutureWithRequest> futures;
      futures.reserve(managed_nodes_.size());

      for (auto node : managed_nodes_)
      {
        // Check if the timeout has expired
        auto elapsed_time = rclcpp::Time::now() - start_time;
        if (elapsed_time >= allowed_duration)
        {
          clean_result = false;
          break;
        }

        // Wait for service
        if (!waitForService<lifecycle_msgs::srv::ChangeState>(node.change_state_client, (allowed_duration - elapsed_time).nanoseconds()))
        {
          clean_result = false;
          continue;
        }

        // Call service
        futures.emplace_back(node.change_state_client->async_send_request(request));
      }
      for (auto future : futures)
      {
        // Check if the timeout has expired
        auto elapsed_time = rclcpp::Time::now() - start_time;
        if (elapsed_time >= allowed_duration)
        {
          clean_result = false;
          break;
        }

        if (!wait_on_change_state_future(future, (allowed_duration - elapsed_time).nanoseconds()))
        {
          clean_result = false;
          break;
        }
      }
    }

    return clean_result;
  }

  template <class T>
  bool waitForService(const rclcpp::Client<T> &client, const std::chrono::nanoseconds &timeout)
  {
    if (!client->wait_for_service(timeout))
    {
      RCLCPP_ERROR(
          get_logger(),
          "Service %s is not available.",
          client->get_service_name());
      return false;
    }

    return true;
  }

  bool wait_on_change_state_future(const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFutureWithRequest &future,
                                   const std::chrono::milliseconds &timeout)
  {
    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future, timeout);

    if (future_status != std::future_status::ready)
    {
      RCLCPP_ERROR(
          get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
      return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success)
    {
      RCLCPP_INFO(
          get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
      return true;
    }
    else
    {
      RCLCPP_WARN(
          get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
      return false;
    }
  }

} // namespace ros2_lifecycle_manager
