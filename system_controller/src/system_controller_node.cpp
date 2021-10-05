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

#include "system_controller/system_controller_node.hpp"
#include "system_controller/system_controller.hpp"

#include <memory>

namespace system_controller
{

SystemControllerNode::SystemControllerNode(const rclcpp::NodeOptions & options)
: CarmaNode(options)
{
  lifecycle_mgr_ = std::make_shared<ros2_lifecycle_manager::LifecycleManager>(
    get_node_base_interface(),
    get_node_parameters_interface(),
    get_node_logging_interface(),
    get_node_timers_interface(),
    get_node_services_interface()
  );

  system_alert_pub_ = create_publisher<carma_msgs::msg::SystemAlert>(
    system_alert_topic_, 10);

  system_alert_sub_ = create_subscription<carma_msgs::msg::SystemAlert>(
    system_alert_topic_, 1,
    std::bind(&CarmaNode::on_system_alert, this, std::placeholders::_1));
}

void
SystemControllerNode::on_system_alert(const carma_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Received SystemAlert message of type: %u, msg: %s",
    msg->type, msg->description.c_str());

  switch (msg->type) {
    case carma_msgs::msg::SystemAlert::CAUTION:
      lifecycle_mgr_->pause();
      break;

    case carma_msgs::msg::SystemAlert::SHUTDOWN:
    case carma_msgs::msg::SystemAlert::FATAL:
      lifecycle_mgr_->shutdown();
      break;
  }
}

}  // namespace system_controller
