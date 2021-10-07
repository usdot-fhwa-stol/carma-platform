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

#include "system_controller/system_controller_node.hpp"
#include "system_controller/system_controller.hpp"

#include <memory>

namespace system_controller
{

  SystemControllerNode::SystemControllerNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node(options)
  {

    // Create subscriptions. History size of 100 is used here to ensure no missed alerts
    system_alert_sub_ = create_subscription<carma_msgs::msg::SystemAlert>(
        system_alert_topic_, 100,
        std::bind(&SystemControllerNode::on_system_alert, this, std::placeholders::_1));
  }

  void on_error(const std::exception &e)
  {
    std::string reason = "Uncaught exception: " + e.what();
    RCLCPP_ERROR_STREAM(
        get_logger(), reason);

    lifecycle_mgr_.shutdown(); // Trigger shutdown when internal error occurs

    rclcpp::shutdown(nullptr, reason); // Fully shutdown this node
  }

  void timer_callback(const ros::TimerEvent &)
  {
    try
    {
      // Walk the managed nodes through their lifecycle
      // First we configure the nodes
      if (!lifecycle_mgr_.configure())
      {
        // If some nodes failed to configure then we will shutdown the system
        RCLCPP_ERROR_STREAM(
            get_logger(), "System could not be configured on startup. Shutting down.");

        lifecycle_mgr_.shutdown();
        return;
      }

      // Second we activate the nodes
      if (!lifecycle_mgr_.activate())
      {
        RCLCPP_ERROR_STREAM(
            get_logger(), "System could not be activated. Shutting down.");
        lifecycle_mgr_.shutdown();
        return;
      }
    }
    catch (const std::exception &e)
    {
      on_error(e);
    }
  }

  void on_system_alert(const carma_msgs::msg::SystemAlert::UniquePtr msg)
  {

    try
    {

      RCLCPP_INFO(
          get_logger(), "Received SystemAlert message of type: %u, msg: %s",
          msg->type, msg->description.c_str());

      if ((managed_nodes_.find(msg.source_node) != managed_nodes_.end() && msg.type == FATAL) || msg.type == SHUTDOWN)
      { // TODO might make more sense for external shutdown to be a service call
        lifecycle_mgr_.shutdown();
      }
    }
    catch (const std::exception &e)
    {
      on_error(e);
    }
  }

} // namespace system_controller
