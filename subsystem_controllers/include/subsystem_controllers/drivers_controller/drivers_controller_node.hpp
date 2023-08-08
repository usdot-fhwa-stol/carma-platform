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
#include "subsystem_controllers/base_subsystem_controller/base_subsystem_controller.hpp"
#include "subsystem_controllers/drivers_controllers/drivers_controller_config.hpp"

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

    // TODO integrate driver_discovery/health_monitor behavior into this node
    // https://github.com/usdot-fhwa-stol/carma-platform/issues/1500

    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp::NodeOptions &options);
    
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);

    carma_ros2_utils::CallbackReturn handle_on_deactivate(const rclcpp_lifecycle::State &);

    carma_ros2_utils::CallbackReturn handle_on_cleanup(const rclcpp_lifecycle::State &);

    carma_ros2_utils::CallbackReturn handle_on_shutdown(const rclcpp_lifecycle::State &);


    /*!
      * \brief Begin normal execution of health monitor node. Will take over control flow of program and exit from here.
      * 
      * \return The exit status of this program
      */
    void run();

    // spin callback function
    bool spin_cb();

  private:

    // DriverManager to handle all the driver specific discovery and reporting
    // DriverManager driver_manager_;

    //! Config for user provided parameters
    DriverControllerConfig config_;

    //! ROS handles
    carma_ros2_utils<carma_driver_msgs::msg::DriverStatus> driver_status_sub_;

    // message/service callbacks
    void driver_discovery_cb(const carma_driver_msgs::msg::DriverStatus::SharedPtr msg);

    // initialize method
    void initialize();

    //! ROS parameters
    std::vector<std::string> required_drivers_;
    std::vector<std::string> lidar_gps_drivers_;
    std::vector<std::string> camera_drivers_;

    bool truck_;
    bool car_;

    // Previously published alert message
    boost::optional<carma_msgs::msg::SystemAlert> prev_alert;
  };

} // namespace v2x_controller

