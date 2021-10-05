// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_lifecycle_manager/lifecycle_manager_client.hpp"

#include <memory>
#include <string>

namespace ros2_lifecycle_manager
{

// TODO(mjeronimo): Use the node interfaces in the constructor (or add another constructor)
LifecycleManagerClient::LifecycleManagerClient(
  const std::string & managed_node_name,
  std::shared_ptr<rclcpp::Node> parent_node)
{
  manage_service_name_ = managed_node_name + std::string("/manage_nodes");

  // Use the provided node for service calls and logging
  node_ = parent_node;

  // Create the service clients
  lifecycle_manager_client_ = std::make_shared<ros2_utils::ServiceClient<ManageLifecycleNodes>>(
    manage_service_name_, node_);
}

bool
LifecycleManagerClient::startup(const std::chrono::nanoseconds timeout)
{
  return call_service(ManageLifecycleNodes::Request::STARTUP, timeout);
}

bool
LifecycleManagerClient::shutdown(const std::chrono::nanoseconds timeout)
{
  return call_service(ManageLifecycleNodes::Request::SHUTDOWN, timeout);
}

bool
LifecycleManagerClient::pause(const std::chrono::nanoseconds timeout)
{
  return call_service(ManageLifecycleNodes::Request::PAUSE, timeout);
}

bool
LifecycleManagerClient::resume(const std::chrono::nanoseconds timeout)
{
  return call_service(ManageLifecycleNodes::Request::RESUME, timeout);
}

bool
LifecycleManagerClient::reset(const std::chrono::nanoseconds timeout)
{
  return call_service(ManageLifecycleNodes::Request::RESET, timeout);
}

bool
LifecycleManagerClient::call_service(uint8_t command, const std::chrono::nanoseconds timeout)
{
  auto request = std::make_shared<ManageLifecycleNodes::Request>();
  request->command = command;

  RCLCPP_DEBUG(
    node_->get_logger(), "Waiting for the %s service...",
    manage_service_name_.c_str());

  // TODO(mjeronimo): should timeout overall and return rather than print and loop
  while (!lifecycle_manager_client_->wait_for_service(timeout)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Client interrupted while waiting for service to appear");
      return false;
    }
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for service to appear...");
  }

  RCLCPP_DEBUG(node_->get_logger(), "Sending %s request", manage_service_name_.c_str());
  try {
    auto future_result = lifecycle_manager_client_->invoke(request, timeout);
    return future_result->success;
  } catch (std::runtime_error &) {
    return false;
  }
}

}  // namespace ros2_lifecycle_manager
