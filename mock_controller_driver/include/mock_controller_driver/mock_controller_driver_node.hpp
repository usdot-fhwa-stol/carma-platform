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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <carma_driver_msgs/srv/set_enable_robotic.hpp>
#include <carma_driver_msgs/msg/robot_enabled.hpp>
#include <autoware_msgs/msg/vehicle_cmd.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace mock_controller_driver
{

  /**
   * \brief TODO for USER: Add class description
   *
   */
  class MockControllerDriver : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<autoware_msgs::msg::VehicleCmd> vehicle_cmd_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_driver_msgs::msg::RobotEnabled> robot_status_pub_;


    // Service Servers
    carma_ros2_utils::ServicePtr<carma_driver_msgs::srv::SetEnableRobotic> enable_robotic_srvice_;

    // Timers
    rclcpp::TimerBase::SharedPtr spin_timer_;


  public:

      // Pub
    const std::string robot_status_topic_ = "controller/robot_status";

    // Sub
    const std::string vehicle_cmd_topic_ = "vehicle_cmd";

    // Service
    const std::string enable_robotic_srv_ = "controller/enable_robotic";

    // Robot Status flags
    bool robot_active_ = false;
    bool robot_enabled_ = false;

    /**
     * \brief MockControllerDriver constructor
     */
    explicit MockControllerDriver(const rclcpp::NodeOptions &);

    /**
     * \brief Callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * \brief Timer callback
     */
    void timer_callback();


    /**
    * \brief Callback for the enable robotic service. Triggering this callback will modify the RobotEnabled message output by this driver.
    *
    * \param request The service request in message
    * \param response The service response out message
    *
    * \return Flag idicating if the service was processed successfully.
    */
    bool enable_robotic_srv(std::shared_ptr<rmw_request_id_t>,
                            const std::shared_ptr<carma_driver_msgs::srv::SetEnableRobotic::Request> request,
                            std::shared_ptr<carma_driver_msgs::srv::SetEnableRobotic::Response> response);

    /**
    * \brief Callback for the vehicle command topic. This callback must be triggered at least once before the enable robotic service is called.
    *
    * \param msg The vehicle command to receive
    */
    void vehicle_cmd_callback(const autoware_msgs::msg::VehicleCmd::UniquePtr msg);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);

    /**
     * TODO for USER: The following lifecycle overrides are also available if needed
     * handle_on_activate
     * handle_on_deactivate
     * handle_on_cleanup
     * handle_on_shutdown
     * handle_on_error
     */
  };

} // mock_controller_driver
