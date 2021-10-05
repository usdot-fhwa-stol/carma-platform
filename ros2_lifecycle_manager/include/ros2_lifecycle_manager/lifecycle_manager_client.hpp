// Copyright (c) 2019 Intel Corporation
// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
#define ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_lifecycle_manager_msgs/srv/manage_lifecycle_nodes.hpp"
#include "ros2_utils/service_client.hpp"

namespace ros2_lifecycle_manager
{

class LifecycleManagerClient
{
public:
  explicit LifecycleManagerClient(
    const std::string & managed_node_name,
    std::shared_ptr<rclcpp::Node> parent_node);  // The node that executes the service calls
  // TODO(@mjeronimo): node_logging_interface, node_services_interface
  // TODO(@mjeronimo): LifecycleNode ctor
  // TODO(@mjeronimo): Raw interfaces ctor; delegating constructors

  // Client interface to the lifecycle manager
  bool startup(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  bool shutdown(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  bool pause(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  bool resume(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  bool reset(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

protected:
  using ManageLifecycleNodes = ros2_lifecycle_manager_msgs::srv::ManageLifecycleNodes;

  // A generic method used by startup, shutdown, etc.
  bool call_service(
    uint8_t command,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  // The node to use for the service call
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<ros2_utils::ServiceClient<ManageLifecycleNodes>> lifecycle_manager_client_;

  std::string manage_service_name_;
};

}  // namespace ros2_lifecycle_manager

#endif  // ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
