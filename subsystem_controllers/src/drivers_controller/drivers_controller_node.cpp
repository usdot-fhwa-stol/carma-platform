
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

#include "subsystem_controllers/drivers_controller/drivers_controller_node.hpp"

namespace subsystem_controllers
{
DriversControllerNode::DriversControllerNode(const rclcpp::NodeOptions & options)
: BaseSubsystemController(options)
{
  // Don't automatically trigger state transitions from base class on configure
  // In this class the managed nodes list first needs to be modified then the transition will be
  // triggered manually
  trigger_managed_nodes_configure_from_base_class_ = false;

  config_.startup_duration_ = declare_parameter<int>("startup_duration", config_.startup_duration_);
  config_.driver_timeout_ =
    declare_parameter<double>("ros1_ssc_driver_timeout ", config_.driver_timeout_);

  // carma-config parameters
  config_.ros1_ssc_driver_name_ =
    declare_parameter<std::string>("ros1_ssc_driver_name", config_.ros1_ssc_driver_name_);
  config_.excluded_namespace_nodes_ = declare_parameter<std::vector<std::string>>(
    "excluded_namespace_nodes", config_.excluded_namespace_nodes_);
}

carma_ros2_utils::CallbackReturn DriversControllerNode::handle_on_configure(
  const rclcpp_lifecycle::State & prev_state)
{
  auto base_return = BaseSubsystemController::handle_on_configure(prev_state);

  if (base_return != carma_ros2_utils::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Drivers Controller could not configure");
    return base_return;
  }

  config_ = DriversControllerConfig();

  // Load required plugins and default enabled plugins
  get_parameter<std::string>("ros1_ssc_driver_name", config_.ros1_ssc_driver_name_);
  get_parameter<int>("startup_duration", config_.startup_duration_);
  get_parameter<double>("ros1_ssc_driver_timeout ", config_.driver_timeout_);
  get_parameter<std::vector<std::string>>(
    "excluded_namespace_nodes", config_.excluded_namespace_nodes_);

  RCLCPP_INFO_STREAM(get_logger(), "Config: " << config_);

  // Handle fact that parameter vectors cannot be empty
  if (
    config_.excluded_namespace_nodes_.size() == 1 && config_.excluded_namespace_nodes_[0].empty()) {
    config_.excluded_namespace_nodes_.clear();
  }

  auto base_managed_nodes = lifecycle_mgr_.get_managed_nodes();
  // Update managed nodes
  // Collect namespace nodes not managed by other subsystem controllers - manually specified in
  // carma-config
  auto updated_managed_nodes =
    get_non_intersecting_set(base_managed_nodes, config_.excluded_namespace_nodes_);

  lifecycle_mgr_.set_managed_nodes(updated_managed_nodes);

  ssc_driver_manager_ =
    std::make_shared<SSCDriverManager>(config_.ros1_ssc_driver_name_, config_.driver_timeout_);

  // record starup time
  start_up_timestamp_ = this->now().nanoseconds() / 1e6;

  driver_status_sub_ = create_subscription<carma_driver_msgs::msg::DriverStatus>(
    "driver_discovery", 20,
    std::bind(&DriversControllerNode::driver_discovery_cb, this, std::placeholders::_1));

  ssc_status_check_timer_ = create_timer(
    get_clock(), std::chrono::milliseconds(1000),
    std::bind(&DriversControllerNode::critical_drivers_check_callback, this));

  // Configure our drivers
  bool success = lifecycle_mgr_
                   .configure(
                     std::chrono::milliseconds(base_config_.service_timeout_ms),
                     std::chrono::milliseconds(base_config_.call_timeout_ms))
                   .empty();

  if (success) {
    RCLCPP_INFO_STREAM(get_logger(), "Subsystem able to configure");
    return CallbackReturn::SUCCESS;
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "Subsystem unable to configure");
    return CallbackReturn::FAILURE;
  }
}

carma_ros2_utils::CallbackReturn DriversControllerNode::handle_on_activate(
  const rclcpp_lifecycle::State & prev_state)
{
  // Reset to automatically trigger state transitions from base class
  trigger_managed_nodes_configure_from_base_class_ = true;

  // return only either SUCCESS or FAILURE
  auto base_return = BaseSubsystemController::handle_on_activate(
    prev_state);  // This will activate all base_managed_nodes

  if (base_return == CallbackReturn::SUCCESS) {
    RCLCPP_INFO(get_logger(), "Driver Controller activated!");
  } else if (base_return == CallbackReturn::FAILURE) {
    RCLCPP_ERROR(
      get_logger(),
      "Driver Controller encountered FAILURE when trying to activate the subsystems... please "
      "check which driver failed to activate...");
  }

  return base_return;
}

void DriversControllerNode::critical_drivers_check_callback()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;  // Exit early if node is not active
  }

  long time_now = this->now().nanoseconds() / 1e6;
  long start_duration = rclcpp::Duration(config_.startup_duration_, 0).nanoseconds() / 1e6;

  auto sys_alert_msg_from_ssc =
    ssc_driver_manager_->get_latest_system_alert(time_now, start_up_timestamp_, start_duration);

  // Only publish if it is first time publishing
  // or if its type or description changed
  if (
    !prev_alert || prev_alert->type != sys_alert_msg_from_ssc.type ||
    prev_alert->description != sys_alert_msg_from_ssc.description) {
    prev_alert = sys_alert_msg_from_ssc;
    publish_system_alert(sys_alert_msg_from_ssc);
  } else {
    RCLCPP_DEBUG_STREAM(get_logger(), "No change to alert status");
  }
}

void DriversControllerNode::driver_discovery_cb(
  const carma_driver_msgs::msg::DriverStatus::SharedPtr msg)
{
  // Driver discovery only published by ros1 drivers
  ssc_driver_manager_->update_driver_status(msg, this->now().nanoseconds() / 1e6);
}

}  // namespace subsystem_controllers

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(subsystem_controllers::DriversControllerNode)
