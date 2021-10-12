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

namespace system_controller
{
  using std_msec = std::chrono::milliseconds;
  SystemControllerNode::SystemControllerNode(const rclcpp::NodeOptions &options, bool auto_init)
      : rclcpp::Node("system_controller", options), 
      lifecycle_mgr_(get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(), get_node_services_interface())
  {

    if (auto_init)
    {
      initialize();
    }
  }

  void SystemControllerNode::initialize()
  {
    try
    {

      RCLCPP_INFO(get_logger(), "Initializing SystemControllerNode");

      // Create subscriptions. History size of 100 is used here to ensure no missed alerts
      system_alert_sub_ = create_subscription<carma_msgs::msg::SystemAlert>(
          system_alert_topic_, 100,
          std::bind(&SystemControllerNode::on_system_alert, this, std::placeholders::_1));

      // Create startup timer
      startup_timer_ = rclcpp::create_timer(
          this,
          get_clock(),
          std::chrono::milliseconds(static_cast<long>(config_.signal_configure_delay * 1000)),
          std::bind(&SystemControllerNode::startup_delay_callback, this));

      this->declare_parameter<double>("signal_configure_delay", config_.signal_configure_delay);
      this->declare_parameter<int64_t>("my_parameter", config_.service_timeout_ms);
      this->declare_parameter<int64_t>("my_parameter", config_.call_timeout_ms);
      //this->declare_parameter<std::string>("my_parameter", config_.required_subsystem_nodes); TODO


    }
    catch (const std::exception &e)
    {
      on_error(e);
    }
  }

  void SystemControllerNode::set_config(SystemControllerConfig config)
  {
    config_ = config;
  }

  void SystemControllerNode::on_error(const std::exception &e)
  {
    std::string reason = "Uncaught exception: " + std::string(e.what());
    RCLCPP_ERROR_STREAM(
        get_logger(), reason);

    lifecycle_mgr_.shutdown(std_msec(config_.service_timeout_ms), std_msec(config_.call_timeout_ms), false); // Trigger shutdown when internal error occurs

    rclcpp::shutdown(nullptr, reason); // Fully shutdown this node
  }

  void SystemControllerNode::startup_delay_callback()
  {
    try
    {
      // Walk the managed nodes through their lifecycle
      // First we configure the nodes
      if (!lifecycle_mgr_.configure(std_msec(config_.service_timeout_ms), std_msec(config_.call_timeout_ms)))
      {
        // If some nodes failed to configure then we will shutdown the system
        RCLCPP_ERROR_STREAM(
            get_logger(), "System could not be configured on startup. Shutting down.");

        lifecycle_mgr_.shutdown(std_msec(config_.service_timeout_ms), std_msec(config_.call_timeout_ms), false);
        return;
      }

      // Second we activate the nodes
      if (!lifecycle_mgr_.activate(std_msec(config_.service_timeout_ms), std_msec(config_.call_timeout_ms)))
      {
        RCLCPP_ERROR_STREAM(
            get_logger(), "System could not be activated. Shutting down.");
        lifecycle_mgr_.shutdown(std_msec(config_.service_timeout_ms), std_msec(config_.call_timeout_ms), false);
        return;
      }

      startup_timer_->cancel();
    }
    catch (const std::exception &e)
    {
      on_error(e);
    }
  }

  void SystemControllerNode::on_system_alert(const carma_msgs::msg::SystemAlert::UniquePtr msg)
  {

    try
    {

      RCLCPP_INFO(
          get_logger(), "Received SystemAlert message of type: %u, msg: %s",
          msg->type, msg->description.c_str());

      if ((std::find(config_.required_subsystem_nodes.begin(), config_.required_subsystem_nodes.end(), msg->source_node) != config_.required_subsystem_nodes.end() && msg->type == carma_msgs::msg::SystemAlert::FATAL) || msg->type == carma_msgs::msg::SystemAlert::SHUTDOWN)
      { // TODO might make more sense for external shutdown to be a service call
        lifecycle_mgr_.shutdown(std_msec(config_.service_timeout_ms), std_msec(config_.call_timeout_ms), false);
      }
    }
    catch (const std::exception &e)
    {
      on_error(e);
    }
  }

} // namespace system_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(system_controller::SystemControllerNode)
