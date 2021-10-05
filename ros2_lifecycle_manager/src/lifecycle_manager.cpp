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

#include "ros2_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::msg::State;

namespace ros2_lifecycle_manager
{

LifecycleManager::LifecycleManager(rclcpp::Node::SharedPtr node)
: LifecycleManager(
    node->get_node_base_interface(),
    node->get_node_parameters_interface(),
    node->get_node_logging_interface(),
    node->get_node_timers_interface(),
    node->get_node_services_interface())
{
}

LifecycleManager::LifecycleManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: LifecycleManager(
    node->get_node_base_interface(),
    node->get_node_parameters_interface(),
    node->get_node_logging_interface(),
    node->get_node_timers_interface(),
    node->get_node_services_interface())
{
}

LifecycleManager::LifecycleManager(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services)
: node_base_(node_base), node_params_(node_params), node_logging_(node_logging), node_timers_(
    node_timers), node_services_(node_services)
{
  // The list of names is parameterized, allowing this module to be used with a different set
  // of managed nodes. By default the node name list is empty.
  std::vector<std::string> default_node_names;
  node_params_->declare_parameter("node_names", rclcpp::ParameterValue(default_node_names));
  node_params_->declare_parameter("autostart", rclcpp::ParameterValue(false));

  node_names_ = node_params_->get_parameter("node_names").as_string_array();
  autostart_ = node_params_->get_parameter("autostart").as_bool();

  manager_srv_ = create_service<ManageLifecycleNodes>(
    node_base_->get_name() + std::string("/manage_nodes"),
    std::bind(&LifecycleManager::manager_callback, this, _1, _2, _3));

  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "-r", std::string("__node:=") + node_base_->get_name() + "_service_client",
      "--"});
  service_client_node_ = std::make_shared<rclcpp::Node>("_", options);

  transition_state_map_[Transition::TRANSITION_CONFIGURE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_CLEANUP] = State::PRIMARY_STATE_UNCONFIGURED;
  transition_state_map_[Transition::TRANSITION_ACTIVATE] = State::PRIMARY_STATE_ACTIVE;
  transition_state_map_[Transition::TRANSITION_DEACTIVATE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    State::PRIMARY_STATE_FINALIZED;

  transition_label_map_[Transition::TRANSITION_CONFIGURE] = std::string("Configuring ");
  transition_label_map_[Transition::TRANSITION_CLEANUP] = std::string("Cleaning up ");
  transition_label_map_[Transition::TRANSITION_ACTIVATE] = std::string("Activating ");
  transition_label_map_[Transition::TRANSITION_DEACTIVATE] = std::string("Deactivating ");
  transition_label_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    std::string("Shutting down ");

  // TODO(mjeronimo): Get rid of this timer callback
  // and do it a different way (avoid race condition)
  // Can't use shared_from_this() during construction
  init_timer_ = rclcpp::create_wall_timer(
    // std::chrono::nanoseconds(10),
    std::chrono::seconds(1),
    [this]() -> void {
      init_timer_->cancel();
      create_lifecycle_service_clients();
      if (autostart_) {
        startup();
      }
    },
    nullptr,
    node_base_.get(),
    node_timers_.get()
  );
}

void
LifecycleManager::manager_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ManageLifecycleNodes::Request> request,
  std::shared_ptr<ManageLifecycleNodes::Response> response)
{
  switch (request->command) {
    case ManageLifecycleNodes::Request::STARTUP:
      response->success = startup();
      break;
    case ManageLifecycleNodes::Request::RESET:
      response->success = reset();
      break;
    case ManageLifecycleNodes::Request::SHUTDOWN:
      response->success = shutdown();
      break;
    case ManageLifecycleNodes::Request::PAUSE:
      response->success = pause();
      break;
    case ManageLifecycleNodes::Request::RESUME:
      response->success = resume();
      break;
  }
}

void
LifecycleManager::create_lifecycle_service_clients()
{
  RCLCPP_INFO(node_logging_->get_logger(), "Creating and initializing lifecycle service clients");
  for (auto & node_name : node_names_) {
    node_map_[node_name] =
      std::make_shared<ros2_utils::LifecycleServiceClient>(node_name, service_client_node_);
  }
}

void
LifecycleManager::destroy_lifecycle_service_clients()
{
  RCLCPP_INFO(node_logging_->get_logger(), "Destroying lifecycle service clients");
  for (auto & kv : node_map_) {
    kv.second.reset();
  }
}

bool
LifecycleManager::change_state_for_node(const std::string & node_name, std::uint8_t transition)
{
  std::string msg = transition_label_map_[transition] + node_name;
  RCLCPP_INFO(node_logging_->get_logger(), msg.c_str());

  try {
    if (!node_map_[node_name]->change_state(transition, 1s) ||
      !(node_map_[node_name]->get_state() == transition_state_map_[transition]))
    {
      RCLCPP_ERROR(
        node_logging_->get_logger(), "Failed to change lifecycle node state: %s",
        node_name.c_str());
      return false;
    }
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(
      node_logging_->get_logger(), "Failed to change lifecycle node state: %s",
      node_name.c_str());
    return false;
  }

  return true;
}

bool
LifecycleManager::change_state_for_all_nodes(std::uint8_t transition)
{
  if (transition == Transition::TRANSITION_CONFIGURE ||
    transition == Transition::TRANSITION_ACTIVATE)
  {
    for (auto & node_name : node_names_) {
      if (!change_state_for_node(node_name, transition)) {
        return false;
      }
    }
  } else {
    std::vector<std::string>::reverse_iterator rit;
    for (rit = node_names_.rbegin(); rit != node_names_.rend(); ++rit) {
      if (!change_state_for_node(*rit, transition)) {
        return false;
      }
    }
  }
  return true;
}

bool
LifecycleManager::startup()
{
  RCLCPP_INFO(node_logging_->get_logger(), "Starting managed nodes bringup");

  // TODO(mjeronimo): enable this here instead of in the constructor:
  // create_lifecycle_service_clients();

  if (!change_state_for_all_nodes(Transition::TRANSITION_CONFIGURE) ||
    !change_state_for_all_nodes(Transition::TRANSITION_ACTIVATE))
  {
    RCLCPP_ERROR(
      node_logging_->get_logger(), "Failed to bring up all requested nodes. Aborting bringup.");
    return false;
  }

  RCLCPP_INFO(node_logging_->get_logger(), "Managed nodes are active");
  return true;
}

bool
LifecycleManager::shutdown()
{
  RCLCPP_INFO(node_logging_->get_logger(), "Shutting down managed nodes...");

  change_state_for_all_nodes(Transition::TRANSITION_DEACTIVATE);
  change_state_for_all_nodes(Transition::TRANSITION_CLEANUP);
  change_state_for_all_nodes(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);

  destroy_lifecycle_service_clients();

  RCLCPP_INFO(node_logging_->get_logger(), "Managed nodes have been shut down");
  return true;
}

bool
LifecycleManager::reset()
{
  // Should transition in reverse order
  RCLCPP_INFO(node_logging_->get_logger(), "Resetting managed nodes...");
  if (!change_state_for_all_nodes(Transition::TRANSITION_DEACTIVATE) ||
    !change_state_for_all_nodes(Transition::TRANSITION_CLEANUP))
  {
    RCLCPP_ERROR(node_logging_->get_logger(), "Failed to reset nodes: aborting reset");
    return false;
  }

  RCLCPP_INFO(node_logging_->get_logger(), "Managed nodes have been reset");
  return true;
}

bool
LifecycleManager::pause()
{
  RCLCPP_INFO(node_logging_->get_logger(), "Pausing managed nodes...");
  if (!change_state_for_all_nodes(Transition::TRANSITION_DEACTIVATE)) {
    RCLCPP_ERROR(node_logging_->get_logger(), "Failed to pause nodes: aborting pause");
    return false;
  }

  RCLCPP_INFO(node_logging_->get_logger(), "Managed nodes have been paused");
  return true;
}

bool
LifecycleManager::resume()
{
  RCLCPP_INFO(node_logging_->get_logger(), "Resuming managed nodes...");
  if (!change_state_for_all_nodes(Transition::TRANSITION_ACTIVATE)) {
    RCLCPP_ERROR(node_logging_->get_logger(), "Failed to resume nodes: aborting resume");
    return false;
  }

  RCLCPP_INFO(node_logging_->get_logger(), "Managed nodes are active");
  return true;
}

}  // namespace ros2_lifecycle_manager
