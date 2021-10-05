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

#ifndef ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_INTERFACE_HPP_
#define ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_INTERFACE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_lifecycle_manager_msgs/srv/manage_lifecycle_nodes.hpp"
#include "ros2_utils/service_client.hpp"

namespace ros2_lifecycle_manager
{

// TODO this interface defines the interface to be used for lifecycle manages to call the relevent services for their nodes
class LifecycleManagerInterface
{
public:

  virtual void set_managed_nodes(const std::vector<std::string>& nodes) = 0;
  virtual std::vector<std::stirng> get_managed_nodes() = 0;

  virtual uint8_t get_managed_node_state(const std::string& node) = 0;


  // If ordered is true then the calls will execute in sequence as defined by set_managed_nodes. If false then no ordering is enforced (though it still may be)
  virtual bool configure(const std::chrono::nanoseconds& timeout, bool ordered=true) = 0;
  virtual bool cleanup(const std::chrono::nanoseconds& timeout, bool ordered=true) = 0;
  virtual bool activate(const std::chrono::nanoseconds& timeout, bool ordered=true) = 0;
  virtual bool deactivate(const std::chrono::nanoseconds& timeout, bool ordered=true) = 0;
  virtual bool shutdown(const std::chrono::nanoseconds& timeout, bool ordered=true) = 0;

};

}  // namespace ros2_lifecycle_manager

#endif  // ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_INTERFACE_HPP_
