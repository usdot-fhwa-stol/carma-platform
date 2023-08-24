
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
  DriversControllerNode::DriversControllerNode(const rclcpp::NodeOptions &options)
      : BaseSubsystemController(options)
  {
    // Don't automatically trigger state transitions from base class on configure
      // In this class the managed nodes list first needs to be modified then the transition will be triggered manually
    trigger_managed_nodes_configure_from_base_class_ = false;

    config_.startup_duration_ = declare_parameter<double>("startup_duration", config_.startup_duration_);
    config_.driver_timeout_ = declare_parameter<double>("required_driver_timeout", config_.driver_timeout_);
    
    // carma-config parameters
    config_.required_drivers_ = declare_parameter<std::vector<std::string>>("required_drivers", config_.required_drivers_); 
    config_.camera_drivers_ = declare_parameter<std::vector<std::string>>("camera_drivers", config_.camera_drivers_);

  }

  carma_ros2_utils::CallbackReturn DriversControllerNode::handle_on_configure(const rclcpp_lifecycle::State &prev_state)
  {
    auto base_return = BaseSubsystemController::handle_on_configure(prev_state);

    if (base_return != carma_ros2_utils::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Drivers Controller could not configure");
      return base_return;
    }

    config_ = DriversControllerConfig();

    // Load required plugins and default enabled plugins
    get_parameter<std::vector<std::string>>("required_drivers", config_.required_drivers_);  
    get_parameter<std::vector<std::string>>("camera_drivers", config_.camera_drivers_); 
    get_parameter<double>("startup_duration", config_.startup_duration_);
    get_parameter<double>("required_driver_timeout", config_.driver_timeout_);

    RCLCPP_INFO_STREAM(get_logger(), "Config: " << config_);


    auto base_managed_nodes = lifecycle_mgr_.get_managed_nodes();

    driver_manager_ = std::make_shared<DriverManager>(
      config_.required_drivers_, 
      config_.camera_drivers_, 
      config_.driver_timeout_
    );

    // record starup time
    start_up_timestamp_ = this->now().nanoseconds() / 1e6;

    driver_status_sub_ = create_subscription<carma_driver_msgs::msg::DriverStatus>("driver_discovery", 1, std::bind(&DriversControllerNode::driver_discovery_cb, this, std::placeholders::_1));
    

    timer_ = create_timer(get_clock(), std::chrono::milliseconds(100), std::bind(&DriversControllerNode::timer_callback,this));

    // Configure our drivers
    bool success = lifecycle_mgr_.configure(std::chrono::milliseconds(base_config_.service_timeout_ms), std::chrono::milliseconds(base_config_.call_timeout_ms)).empty();
    
    if (success)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Subsystem able to configure");
      return CallbackReturn::SUCCESS;
    }
    else
    {
      RCLCPP_INFO_STREAM(get_logger(), "Subsystem unable to configure");
      return CallbackReturn::FAILURE;
    }

  }

  carma_ros2_utils::CallbackReturn DriversControllerNode::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {

    // Reset to automatically trigger state transitions from base class 
    trigger_managed_nodes_configure_from_base_class_ = true;

    auto base_return = BaseSubsystemController::handle_on_activate(prev_state); // This will activate all base_managed_nodes

    if (base_return != carma_ros2_utils::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Driver Controller could not activate");
      return base_return;
    }

  }

  void DriversControllerNode::timer_callback()
  {
    
    long time_now = this->now().nanoseconds() / 1e6;
    rclcpp::Duration sd(config_.startup_duration_);
    long start_duration = sd.nanoseconds()/1e6;

    auto dm = driver_manager_->handle_spin(time_now, start_up_timestamp_, config_.startup_duration_);
    if (!prev_alert) {
      prev_alert = dm;
      publish_system_alert(dm);
    } 
    else if ( prev_alert->type == dm.type && prev_alert->description.compare(dm.description) == 0) { // Do not publish duplicate alerts
      RCLCPP_DEBUG_STREAM(get_logger(), "No change to alert status");
    }
    else{
      prev_alert = dm;
      publish_system_alert(dm);
    }

  }


  void DriversControllerNode::driver_discovery_cb(const carma_driver_msgs::msg::DriverStatus::SharedPtr msg)
  {
    // Driver discovery only published by ros1 drivers
    driver_manager_->update_driver_status(msg, this->now().nanoseconds()/1e6);
  }

  void DriversControllerNode::setDriverManager(DriverManager dm)
  {
      driver_manager_ = std::make_shared<DriverManager>(dm);
  }


} // namespace subsystem_controllers

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(subsystem_controllers::DriversControllerNode)
