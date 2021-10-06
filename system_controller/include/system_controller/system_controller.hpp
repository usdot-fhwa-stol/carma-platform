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

#ifndef SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_HPP_
#define SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_HPP_

#include <memory>
#include <unordered_map>

#include "carma_msgs/msg/system_alert.hpp"
#include "system_controller_config.hpp"
namespace system_controller
{

  enum class SystemState {
    STARTING_UP,
    CONFIGURING,
    INACTIVE,
    ACTIVE,
    SHUTTING_DOWN,
    FINALIZED
  }

class SystemController
{
public:
  SystemController(const SystemControllerConfig& config, std::shared_ptr<LifecycleManagerInterface> lifecycle_interface);

  void on_system_alert(const carma_msgs::msg::SystemAlert::SharedPtr msg);

  void on_loop();

protected:

  SystemControllerConfig config_;

  std::unordered_map<std::string> required_subsystem_nodes_;

  std::shared_ptr<LifecycleManagerInterface> lifecycle_interface_;

  boost::optional<rclcpp::Time> start_time_;

  SystemState state_ = SystemState::STARTING_UP;

  SystemControllerStateTransitionTable state_transition_table_;
};

}  // namespace system_controller

#endif  // SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_HPP_
