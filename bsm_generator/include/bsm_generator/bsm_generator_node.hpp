/*
 * Copyright (C) 2019-2022 LEIDOS.
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
#include <std_msgs/msg/float64.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <automotive_platform_msgs/msg/velocity_accel_cov.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <j2735_v2x_msgs/msg/transmission_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <gps_msgs/msg/gps_fix.hpp>
#include <vector>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "bsm_generator/bsm_generator_worker.hpp"
#include "bsm_generator/bsm_generator_config.hpp"

namespace bsm_generator
{

  /**
   * \class BSMGenerator
   * \brief The class responsible for publishing BSM messages
   */
  class BSMGenerator : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> pose_sub_;
    carma_ros2_utils::SubPtr<automotive_platform_msgs::msg::VelocityAccelCov> accel_sub_;
    carma_ros2_utils::SubPtr<sensor_msgs::msg::Imu> yaw_sub_;
    carma_ros2_utils::SubPtr<j2735_v2x_msgs::msg::TransmissionState> gear_sub_;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> speed_sub_;
    carma_ros2_utils::SubPtr<std_msgs::msg::Float64> steer_wheel_angle_sub_;
    carma_ros2_utils::SubPtr<std_msgs::msg::Float64> brake_sub_;
    carma_ros2_utils::SubPtr<gps_msgs::msg::GPSFix> heading_sub_;
    carma_ros2_utils::SubPtr<std_msgs::msg::String> georeference_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::BSM> bsm_pub_;

    // Timer to run the BSM Generation task
    rclcpp::TimerBase::SharedPtr timer_;

    // Node configuration
    Config config_;

    // Worker class
    std::shared_ptr<BSMGeneratorWorker> worker;

    // The BSM object that all subscribers make updates to
    carma_v2x_msgs::msg::BSM bsm_;
  
    std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

    std::vector<uint8_t> bsm_message_id_;

    /**
     * \brief Function to fill the BSM message with initial default data
     */ 
    void initializeBSM();

    /**
     * \brief Callback to populate BSM message with longitude, latitude, and elevation data
     * \param msg Latest pose message
     */ 
    void poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg);

    /**
     * \brief Callback to populate BSM message with longitudinal acceleration data
     * \param msg Latest acceleration message
     */ 
    void accelCallback(const automotive_platform_msgs::msg::VelocityAccelCov::UniquePtr msg);

    /**
     * \brief Callback to populate BSM message with yaw rate data
     * \param msg Latest IMU message
     */ 
    void yawCallback(const sensor_msgs::msg::Imu::UniquePtr msg);

    /**
     * \brief Callback to populate BSM message with transmission state data
     * \param msg Latest transmissio state message
     */ 
    void gearCallback(const j2735_v2x_msgs::msg::TransmissionState::UniquePtr msg);

    /**
     * \brief Callback to populate BSM message with vehicle speed data
     * \param msg Latest speed message
     */ 
    void speedCallback(const geometry_msgs::msg::TwistStamped::UniquePtr msg);

    /**
     * \brief Callback to populate BSM message with vehicle steering wheel angle data
     * \param msg Latest steering wheel angle message
     */ 
    void steerWheelAngleCallback(const std_msgs::msg::Float64::UniquePtr msg);

    /**
     * \brief Callback to populate BSM message with vehicle applied brake status
     * \param msg Latest brake status message
     */ 
    void brakeCallback(const std_msgs::msg::Float64::UniquePtr msg);

    /**
     * \brief Callback to populate BSM message with vehicle heading data
     * \param msg Latest GNSS message
     */ 
    void headingCallback(const gps_msgs::msg::GPSFix::UniquePtr msg);

    /**
     * \brief Callback for map projection string to define lat/lon -> map conversion
     * \param msg The proj string defining the projection.
     */ 
    void georeferenceCallback(const std_msgs::msg::String::UniquePtr msg);

    /**
     * \brief Timer callback, which publishes a BSM
     */ 
    void generateBSM();

  public:
  
    /**
     * \brief BSMGenerator constructor 
     */
    explicit BSMGenerator(const rclcpp::NodeOptions &);

    /**
     * \brief Function callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);
  };

} // namespace bsm_generator
