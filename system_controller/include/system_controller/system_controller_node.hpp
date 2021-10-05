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

#ifndef SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_NODE_HPP_
#define SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_NODE_HPP_

#include <memory>

#include "carma_msgs/msg/system_alert.hpp"
#include "ros2_lifecycle_manager/lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace system_controller
{

class SystemControllerNode : public rclcpp::Node
{
public:
  SystemControllerNode() = delete;
  explicit SystemControllerNode(const rclcpp::NodeOptions & options);

protected:
  void on_system_alert(const carma_msgs::msg::SystemAlert::SharedPtr msg) override;

  const std::string system_alert_topic_{"/system_alert"};
  rclcpp::Subscription<carma_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;
  std::shared_ptr<rclcpp::Publisher<carma_msgs::msg::SystemAlert>> system_alert_pub_;

  std::shared_ptr<ros2_lifecycle_manager::LifecycleManager> lifecycle_mgr_;
};

}  // namespace system_controller

#endif  // SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_NODE_HPP_
