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

#ifndef ROS2_LIFECYCLE_MANAGER__ROS2_LIFECYCLE_MANAGER_HPP_
#define ROS2_LIFECYCLE_MANAGER__ROS2_LIFECYCLE_MANAGER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_lifecycle_manager_msgs/srv/manage_lifecycle_nodes.hpp"
#include "ros2_utils/service_client.hpp"
#include "lifecycle_manager_interface.hpp"

namespace ros2_lifecycle_manager
{
  using ChangeStateClient = rclcpp::Client<lifecycle_msgs::srv::ChangeState>;
  using GetStateClient = rclcpp::Client<lifecycle_msgs::srv::GetState>;

// Struct combining the node name and relevant clients together
struct ManagedNode {

  ManagedNode(const std::string& node_name, const ChangeStateClient& change_state_client, const GetStateClient& get_state_client) :
    node_name(node_name), change_state_client(change_state_client), get_state_client(get_state_client) {};

  std::string node_name;
  ChangeStateClient change_state_client;
  GetStateClient get_state_client;

}
// TODO this interface defines the interface to be used for lifecycle manages to call the relevent services for their nodes
class Ros2LifecycleManager : public LifecycleManagerInterface
{

public:

  void set_managed_nodes(const std::vector<std::string>& nodes) override;
  std::vector<ManagedNode> get_managed_nodes() override;

  uint8_t get_managed_node_state(const std::string& node) override;

  // If ordered is true then the calls will execute in sequence as defined by set_managed_nodes. If false then no ordering is enforced (though it still may be)
  bool configure(const std::chrono::nanoseconds& timeout, bool ordered=true) override;
  bool cleanup(const std::chrono::nanoseconds& timeout, bool ordered=true) override;
  bool activate(const std::chrono::nanoseconds& timeout, bool ordered=true) override;
  bool deactivate(const std::chrono::nanoseconds& timeout, bool ordered=true) override;
  bool shutdown(const std::chrono::nanoseconds& timeout, bool ordered=true) override;

protected:

  bool transition_multiplex(uint8_t transition, bool ordered);

  const std::string change_state_topic_ = "/change_state";
  const std::string get_state_topic_ = "/get_state";
  std::vector<ManagedNode> managed_nodes_;
  std::vector<std::string> managed_node_names_;

};

}  // namespace ros2_lifecycle_manager

#endif  // ROS2_LIFECYCLE_MANAGER__ROS2_LIFECYCLE_MANAGER_HPP_
