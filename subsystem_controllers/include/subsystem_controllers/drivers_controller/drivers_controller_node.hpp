#pragma once

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



#include <memory>

#include "carma_msgs/msg/system_alert.hpp"
#include "ros2_lifecycle_manager/ros2_lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include <carma_driver_msgs/msg/driver_status.hpp>
#include "subsystem_controllers/base_subsystem_controller/base_subsystem_controller.hpp"
#include "drivers_controller_config.hpp"
#include "driver_manager.hpp"
#include <boost/algorithm/string.hpp>

namespace subsystem_controllers
{

  class DriversControllerNode : public BaseSubsystemController
  {
  public:
    
    DriversControllerNode() = delete;

    ~DriversControllerNode() = default;

    /**
     * \brief Constructor. Set explicitly to support node composition.
     * 
     * \param options The node options to use for configuring this node
     */
    explicit DriversControllerNode(const rclcpp::NodeOptions &options);


  private:

    // DriverManager to handle all the driver specific discovery and reporting
    std::shared_ptr<DriverManager> driver_manager_;

    //! Config for user provided parameters
    DriversControllerConfig config_;

    //! ROS handles
    carma_ros2_utils::SubPtr<carma_driver_msgs::msg::DriverStatus> driver_status_sub_;

    // message/service callbacks
    void driver_discovery_cb(const carma_driver_msgs::msg::DriverStatus::SharedPtr msg);

    //! Timer callback function to check status of required ros1 drivers 
    void timer_callback();

    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &prev_state);

    //! ROS parameters
    std::vector<std::string> ros1_required_drivers_;
    std::vector<std::string> ros1_camera_drivers_;

    // record of startup timestamp
    long start_up_timestamp_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Previously published alert message
    boost::optional<carma_msgs::msg::SystemAlert> prev_alert;

  };

} // namespace v2x_controller

