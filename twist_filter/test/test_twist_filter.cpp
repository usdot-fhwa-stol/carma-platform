/*
 * Copyright (C) 2022 LEIDOS.
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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include "velocity_limit.hpp"
#include "accel_limiter.hpp"

namespace twist_filter
{
    TEST(TwistFilterTest, test_longitudinal_twist_filter) {
        geometry_msgs::msg::TwistStamped twist;
        twist.twist.linear.x = 7.0;

        geometry_msgs::msg::TwistStamped out = twist_filter::longitudinalLimitTwist(twist, 5.0);

        ASSERT_DOUBLE_EQ(5.0, out.twist.linear.x);

        geometry_msgs::msg::TwistStamped twist2;
        twist2.twist.linear.x = 50.0;

        geometry_msgs::msg::TwistStamped out2 = twist_filter::longitudinalLimitTwist(twist2, 100.0);
        std::cout<<"Finished"<<std::endl;

        ASSERT_LT(out2.twist.linear.x, 36.0);
    }

    TEST(TwistFilterTest, test_longitudinal_ctrl_filter) {
        autoware_msgs::msg::ControlCommandStamped ccs;
        ccs.cmd.linear_velocity = 7.0;

        autoware_msgs::msg::ControlCommandStamped out = twist_filter::longitudinalLimitCtrl(ccs, 5.0);

        ASSERT_DOUBLE_EQ(5.0, out.cmd.linear_velocity);

        autoware_msgs::msg::ControlCommandStamped ccs2;
        ccs2.cmd.linear_velocity = 50.0;

        autoware_msgs::msg::ControlCommandStamped out2 = twist_filter::longitudinalLimitCtrl(ccs2, 100.0);

        ASSERT_LT(out2.cmd.linear_velocity, 36.0);
    }

    TEST(TwistFilterTest, test_longitudinal_accel_limiter1) {
        LongitudinalAccelLimiter limiter{5.0};

        geometry_msgs::msg::TwistStamped msg1;
        msg1.header.stamp = rclcpp::Time(0.0, 0.0);
        msg1.twist.linear.x = 0.0;

        geometry_msgs::msg::TwistStamped out1 = limiter.longitudinalAccelLimitTwist(msg1);
        ASSERT_DOUBLE_EQ(0.0, out1.twist.linear.x);

        geometry_msgs::msg::TwistStamped msg2;
        msg2.header.stamp = rclcpp::Time(1.0, 0.0);
        msg2.twist.linear.x = 6.0;

        geometry_msgs::msg::TwistStamped out2 = limiter.longitudinalAccelLimitTwist(msg2);
        ASSERT_DOUBLE_EQ(5.0, out2.twist.linear.x);

        geometry_msgs::msg::TwistStamped msg3;
        msg3.header.stamp = rclcpp::Time(3.0, 0.0);
        msg3.twist.linear.x = 9.0;

        geometry_msgs::msg::TwistStamped out3 = limiter.longitudinalAccelLimitTwist(msg3);
        ASSERT_DOUBLE_EQ(9.0, out3.twist.linear.x);

        geometry_msgs::msg::TwistStamped msg4;
        msg4.header.stamp = rclcpp::Time(4.0, 0.0);
        msg4.twist.linear.x = 6.0;

        geometry_msgs::msg::TwistStamped out4 = limiter.longitudinalAccelLimitTwist(msg4);
        ASSERT_DOUBLE_EQ(6.0, out4.twist.linear.x);

        geometry_msgs::msg::TwistStamped msg5;
        msg5.header.stamp = rclcpp::Time(5.0, 0.0);
        msg5.twist.linear.x = 0.0;

        geometry_msgs::msg::TwistStamped out5 = limiter.longitudinalAccelLimitTwist(msg5);
        ASSERT_DOUBLE_EQ(1.0, out5.twist.linear.x);
    }

    TEST(TwistFilterTest, test_longitudinal_accel_limiter2) {
        LongitudinalAccelLimiter limiter{5.0};

        autoware_msgs::msg::ControlCommandStamped msg1;
        msg1.cmd.linear_velocity = 0.0;
        msg1.header.stamp = rclcpp::Time(0.0, 0.0);

        autoware_msgs::msg::ControlCommandStamped out1 = limiter.longitudinalAccelLimitCtrl(msg1);
        ASSERT_DOUBLE_EQ(0.0, out1.cmd.linear_velocity);

        autoware_msgs::msg::ControlCommandStamped msg2;
        msg2.header.stamp = rclcpp::Time(1.0, 0.0);
        msg2.cmd.linear_velocity = 6.0;

        autoware_msgs::msg::ControlCommandStamped out2 = limiter.longitudinalAccelLimitCtrl(msg2);
        ASSERT_DOUBLE_EQ(5.0, out2.cmd.linear_velocity);

        autoware_msgs::msg::ControlCommandStamped msg3;
        msg3.header.stamp = rclcpp::Time(3.0, 0.0);
        msg3.cmd.linear_velocity = 9.0;

        autoware_msgs::msg::ControlCommandStamped out3 = limiter.longitudinalAccelLimitCtrl(msg3);
        ASSERT_DOUBLE_EQ(9.0, out3.cmd.linear_velocity);

        autoware_msgs::msg::ControlCommandStamped msg4;
        msg4.header.stamp = rclcpp::Time(4.0, 0.0);
        msg4.cmd.linear_velocity = 6.0;

        autoware_msgs::msg::ControlCommandStamped out4 = limiter.longitudinalAccelLimitCtrl(msg4);
        ASSERT_DOUBLE_EQ(6.0, out4.cmd.linear_velocity);

        autoware_msgs::msg::ControlCommandStamped msg5;
        msg5.header.stamp = rclcpp::Time(5.0, 0.0);
        msg5.cmd.linear_velocity = 0.0;

        autoware_msgs::msg::ControlCommandStamped out5 = limiter.longitudinalAccelLimitCtrl(msg5);
        ASSERT_DOUBLE_EQ(1.0, out5.cmd.linear_velocity);
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}