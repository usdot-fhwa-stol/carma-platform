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
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_guidance_plugins/control_plugin.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/ackermann_control_command.hpp>
#include <gtest/gtest_prod.h>
#include <basic_autonomy/basic_autonomy.hpp>
#include <carma_guidance_plugins/control_plugin.hpp>
#include <optional>
#include "trajectory_follower_wrapper/trajectory_follower_wrapper_config.hpp"

#define DEG2RAD 3.1415926535 / 180.0
#define RAD2DEG 180.0 / 3.1415926535

namespace trajectory_follower_wrapper
{

  class TrajectoryFollowerWrapperNode : public carma_guidance_plugins::ControlPlugin
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<autoware_auto_msgs::msg::AckermannControlCommand> control_cmd_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<autoware_auto_msgs::msg::Trajectory> autoware_traj_pub_;
    carma_ros2_utils::PubPtr<autoware_auto_msgs::msg::VehicleKinematicState> autoware_state_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr autoware_info_timer_;

    // Received Control Command
    std::optional<autoware_auto_msgs::msg::AckermannControlCommand> received_ctrl_command_;

  public:

    /**
     * \brief Node constructor
     */
    explicit TrajectoryFollowerWrapperNode(const rclcpp::NodeOptions& options);

    /**
     * \brief Callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * \brief Timer callback to spin at 30 hz and frequently publish autoware kinematic state and trajectory
     */
    void autoware_info_timer_callback();

    /**
     * \brief autoware's control subscription callback
     */
    void ackermann_control_cb(const autoware_auto_msgs::msg::AckermannControlCommand::SharedPtr msg);

    /**
     * \brief Check to see if the received control command recent or old
     */
    bool isControlCommandOld(const autoware_auto_msgs::msg::AckermannControlCommand& cmd) const;

    // Node configuration
    TrajectoryFollowerWrapperConfig config_;

    //CONVERSIONS

    /**
     * \brief convert vehicle's pose and twist messages to autoware kinematic state
     */
    autoware_auto_msgs::msg::VehicleKinematicState convert_state(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::TwistStamped& twist) const;

    /**
     * \brief convert autoware Ackermann control command to autoware stamped control command
     */
    autoware_msgs::msg::ControlCommandStamped convert_cmd(const autoware_auto_msgs::msg::AckermannControlCommand& cmd) const;

    /**
     * \brief calculate wheel angle from angular velocity in twist message
     */
    double get_wheel_angle_from_twist(const geometry_msgs::msg::TwistStamped& twist) const;



    ////
    // Overrides
    ////
    autoware_msgs::msg::ControlCommandStamped generate_command() override;

    bool get_availability() override;

    std::string get_version_id() override;

    /**
     * \brief This method should be used to load parameters and will be called on the configure state transition.
     */
    carma_ros2_utils::CallbackReturn on_configure_plugin() override;

    FRIEND_TEST(TesttrajectoryFollowerWrapper, TestControlCallback);
    FRIEND_TEST(TesttrajectoryFollowerWrapper, TestThreshold);

  };

} // trajectory_follower_wrapper
