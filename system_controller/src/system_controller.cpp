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

#include "system_controller/system_controller.hpp"

#include <memory>

namespace system_controller
{

  SystemController(const SystemControllerConfig &config, const LifecycleManagerInterface& lifecycle_interface) : config_(config), lifecycle_interface_(lifecycle_interface)
  {

    required_subsystem_nodes_.insert(required_subsystem_nodes_.back(), config.required_subsystem_nodes.begin(), config.required_subsystem_nodes.end());
  
    lifecycle_interface_.set_managed_nodes(required_subsystem_nodes_);
  }

  void startup_timeout() {
    state_transition_table_.signal(SystemEvent::STARTUP_DELAY_EXCEEDED);
  }

  void SystemController::change_state(const SystemState& new_state) {
    
    RCLCPP_INFO_STREAM(get_logger(), "Transitioning from state: " << state_ << " to state: " << new_state); // TODO add stream operator
    auto prev_state = state_;
    state_ = new_state;
    switch (prev_state) {
      case SystemState::STARTING_UP:
          if (state_ == ) {

          }
        break
      case SystemState::CONFIGURING: // TODO only call once
        lifecycle_interface_->configure();
        break
      case SystemState::INACTIVE:
        lifecycle_interface_->activate(); // TODO only call once
        break
      case SystemState::ACTIVE:
        // NO-OP
        break
      case SystemState::SHUTTING_DOWN:
        lifecycle_interface_->shutdown();
        break
      case SystemState::FINALIZED:
        break
    }
  }

  void SystemController::on_loop() {
    if (!start_time_) {
      start_time_ = rclcpp::Time::now();
    }

    /*
        STARTING_UP,
    CONFIGURING,
    INACTIVE,
    ACTIVE,
    SHUTTING_DOWN,
    FINALIZED
    */

    switch (state_) {
      case SystemState::STARTING_UP:
          if (rclcpp::Time::now() - start_time > rclcpp::Duration(config_.signal_configure_delay)) {
            state_ = SystemState::CONFIGURING;
          }
        break
      case SystemState::CONFIGURING: // TODO only call once
        lifecycle_interface_->configure();
        break
      case SystemState::INACTIVE:
        lifecycle_interface_->activate(); // TODO only call once
        break
      case SystemState::ACTIVE:
        // NO-OP
        break
      case SystemState::SHUTTING_DOWN:
        lifecycle_interface_->shutdown();
        break
      case SystemState::FINALIZED:
        break
    }

    
  }



  void
  SystemController::on_system_alert(const carma_msgs::msg::SystemAlert::SharedPtr msg)
  {
    RCLCPP_INFO(
        get_logger(), "Received SystemAlert message of type: %u, msg: %s, source_node: %s",
        msg->type, msg->description.c_str(), msg->source_node.c_str());

    switch (msg->type)
    {
    case carma_msgs::msg::SystemAlert::FATAL:
      if (required_subsystem_nodes_.find(msg->source_node) != required_subsystem_nodes_.end()) {
        RCLCPP_INFO_STREAM(
          get_logger(), "Required node: " << msg->source_node << " has reported a fault in its subsystem. Shutting down system.");
      }
      state_ = SystemState::SHUTTING_DOWN;
      on_loop();
      break;
    default:
      break;
    }
  }

} // namespace system_controller
