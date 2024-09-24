/*
 * Copyright (C) 2024 LEIDOS.
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
#include "mock_controller_driver/mock_controller_driver_node.hpp"

namespace mock_controller_driver
{
  namespace std_ph = std::placeholders;

  MockControllerDriver::MockControllerDriver(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {

  }


  carma_ros2_utils::CallbackReturn MockControllerDriver::handle_on_configure(const rclcpp_lifecycle::State &)
  {

    // Setup subscribers
    vehicle_cmd_sub_ = create_subscription<autoware_msgs::msg::VehicleCmd>(vehicle_cmd_topic_, 10,
                                                              std::bind(&MockControllerDriver::vehicle_cmd_callback, this, std_ph::_1));

    // Setup publishers
    robot_status_pub_ = create_publisher<carma_driver_msgs::msg::RobotEnabled>(robot_status_topic_, 10);

    // Setup service servers
    enable_robotic_srvice_ = create_service<carma_driver_msgs::srv::SetEnableRobotic>(enable_robotic_srv_,
                                                            std::bind(&MockControllerDriver::enable_robotic_srv, this, std_ph::_1, std_ph::_2, std_ph::_3));


    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn MockControllerDriver::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {
    // Setup timers
    spin_timer_ = create_timer(
        get_clock(),
        std::chrono::milliseconds(50), // 20 Hz
        std::bind(&MockControllerDriver::timer_callback, this));

    return CallbackReturn::SUCCESS;
  }


  void MockControllerDriver::vehicle_cmd_callback(const autoware_msgs::msg::VehicleCmd::UniquePtr msg)
  {
    robot_enabled_ = true;  // If a command was received set the robot enabled status to true
  }

  bool MockControllerDriver::enable_robotic_srv(std::shared_ptr<rmw_request_id_t>,
                            const std::shared_ptr<carma_driver_msgs::srv::SetEnableRobotic::Request> request,
                            std::shared_ptr<carma_driver_msgs::srv::SetEnableRobotic::Response> response)
  {
    if (robot_enabled_ && request->set == carma_driver_msgs::srv::SetEnableRobotic::Request::ENABLE)
    {
      robot_active_ = true;
    }
    else
    {
      robot_active_ = false;
    }

    return true;
  }

  void MockControllerDriver::timer_callback()
  {
    RCLCPP_DEBUG(get_logger(), "Example timer callback");
    carma_driver_msgs::msg::RobotEnabled robot_status;
    robot_status.robot_active = robot_active_;
    robot_status.robot_enabled = robot_enabled_;
    robot_status_pub_->publish(robot_status);
  }

} // mock_controller_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(mock_controller_driver::MockControllerDriver)
