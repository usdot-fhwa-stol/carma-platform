#pragma once
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Modifications:
 * - Added limiting for longitudinal velocity per CARMA specifications. Refactored
 *   namespacing and header as needed to support unit testing.
 *   - Kyle Rush 
 *   - 9/11/2020
 * - Refactored for ros2
 *    - 11/16/2022
 */

#include <iostream>
#include <boost/optional.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <autoware_msgs/msg/control_command_stamped.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "autoware_config_msgs/msg/config_waypoint_follower.hpp"
#include "autoware_config_msgs/msg/config_twist_filter.hpp"
#include <gtest/gtest_prod.h>
#include "accel_limiter.hpp"

namespace twist_filter
{
//const values
constexpr double MIN_LINEAR_X = 1e-3;
constexpr double MIN_LENGTH = 1e-3;
constexpr double MIN_DURATION = 1e-3;

struct StampedValue
{
    rclcpp::Time time;
    double dt;
    double val;
    StampedValue() : time(0.0), dt(0.0), val(0.0) {}
    void reset()
    {
        time = rclcpp::Time(0,0);
        val = 0.0;
    }
};

class TwistFilter : public carma_ros2_utils::CarmaLifecycleNode
{
public:
    explicit TwistFilter(const rclcpp::NodeOptions &);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);

private:

    //publishers
    carma_ros2_utils::PubPtr<geometry_msgs::msg::TwistStamped> twist_pub_;
    carma_ros2_utils::PubPtr<autoware_msgs::msg::ControlCommandStamped> ctrl_pub_;
    carma_ros2_utils::PubPtr<std_msgs::msg::Float32> twist_lacc_limit_debug_pub_, twist_ljerk_limit_debug_pub_;
    carma_ros2_utils::PubPtr<std_msgs::msg::Float32> ctrl_lacc_limit_debug_pub_, ctrl_ljerk_limit_debug_pub_;
    carma_ros2_utils::PubPtr<std_msgs::msg::Float32> twist_lacc_result_pub_, twist_ljerk_result_pub_;
    carma_ros2_utils::PubPtr<std_msgs::msg::Float32> ctrl_lacc_result_pub_, ctrl_ljerk_result_pub_;
    
    //subscribers
    carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> twist_sub_;
    carma_ros2_utils::SubPtr<autoware_msgs::msg::ControlCommandStamped> ctrl_sub_;
    carma_ros2_utils::SubPtr<autoware_config_msgs::msg::ConfigTwistFilter> config_sub_;
    
    //ros params
    double wheel_base_;
    double longitudinal_velocity_limit_;
    double longitudinal_accel_limit_;
    double lateral_accel_limit_;
    double lateral_jerk_limit_;
    double lowpass_gain_linear_x_;
    double lowpass_gain_angular_z_;
    double lowpass_gain_steering_angle_;

    //dataset
    StampedValue az_prev_;
    StampedValue sa_prev_;

    LongitudinalAccelLimiter _lon_accel_limiter;

    boost::optional<double>
        calcLaccWithAngularZ(const double& lv, const double& az) const;
    boost::optional<double>
        calcLjerkWithAngularZ(const double& lv, const double& az) const;
    boost::optional<double>
        calcLaccWithSteeringAngle(const double& lv, const double& sa) const;
    boost::optional<double>
        calcLjerkWithSteeringAngle(const double& lv, const double& sa) const;
    void publishLateralResultsWithTwist(
        const geometry_msgs::msg::TwistStamped& msg) const;
    void publishLateralResultsWithCtrl(
        const autoware_msgs::msg::ControlCommandStamped& msg) const;
    void checkTwist(const geometry_msgs::msg::TwistStamped& msg);
    void checkCtrl(const autoware_msgs::msg::ControlCommandStamped& msg);
    geometry_msgs::msg::TwistStamped
        lateralLimitTwist(const geometry_msgs::msg::TwistStamped& msg);
    geometry_msgs::msg::TwistStamped
        smoothTwist(const geometry_msgs::msg::TwistStamped& msg);
    autoware_msgs::msg::ControlCommandStamped
    lateralLimitCtrl(const autoware_msgs::msg::ControlCommandStamped& msg);
    autoware_msgs::msg::ControlCommandStamped
        smoothCtrl(const autoware_msgs::msg::ControlCommandStamped& msg);
    void updatePrevTwist(const geometry_msgs::msg::TwistStamped& msg);
    void updatePrevCtrl(const autoware_msgs::msg::ControlCommandStamped& msg);
    void configCallback(
        const autoware_config_msgs::msg::ConfigTwistFilter::UniquePtr config);
    void TwistCmdCallback(const geometry_msgs::msg::TwistStamped::UniquePtr msg);
    void CtrlCmdCallback(const autoware_msgs::msg::ControlCommandStamped::UniquePtr msg);

    // Friend test setup for unit testing of private methods
    FRIEND_TEST(TwistFilterTest, test_longitudinal_twist_filter);
    FRIEND_TEST(TwistFilterTest, test_longitudinal_ctrl_filter);

};
}
