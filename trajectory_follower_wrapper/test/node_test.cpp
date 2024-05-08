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

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include "trajectory_follower_wrapper/trajectory_follower_wrapper_node.hpp"


TEST(Testtrajectory_follower_wrapper, test_time_threshold){

    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<trajectory_follower_wrapper::TrajectoryFollowerWrapperNode>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    std::shared_ptr<autoware_auto_msgs::msg::AckermannControlCommand> aw_cmd = std::make_shared<autoware_auto_msgs::msg::AckermannControlCommand>();
    aw_cmd->stamp = worker_node->now() - rclcpp::Duration::from_seconds(5);
    aw_cmd->longitudinal.acceleration = 1.0;
    aw_cmd->longitudinal.speed = 2.0;

    worker_node->ackermann_control_cb(move(aw_cmd));

    auto res = worker_node->generate_command();

    ASSERT_NEAR(res.cmd.linear_acceleration, 0.0, 0.0001);
}

TEST(Testtrajectory_follower_wrapper, test_conversion_state)
{
    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<trajectory_follower_wrapper::TrajectoryFollowerWrapperNode>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    auto converted_time_now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = rclcpp::Time(converted_time_now*1e9);
    pose.pose.position.x = 1;
    pose.pose.position.y = 2;
    pose.pose.position.z = 3;


    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = rclcpp::Time(converted_time_now*1e9);
    twist.twist.linear.x = 4;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 0;


    autoware_auto_msgs::msg::VehicleKinematicState state_tf = worker_node->convert_state(pose, twist);

    ASSERT_NEAR(state_tf.state.x, pose.pose.position.x, 0.001);
    ASSERT_NEAR(state_tf.state.longitudinal_velocity_mps , twist.twist.linear.x, 0.001);

}

TEST(Testtrajectory_follower_wrapper, test_conversion_ctrl)
{
    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<trajectory_follower_wrapper::TrajectoryFollowerWrapperNode>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    auto converted_time_now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());



    autoware_auto_msgs::msg::AckermannControlCommand autoware_cmd;
    autoware_cmd.longitudinal.acceleration = 1.0;
    autoware_cmd.longitudinal.speed = 2.0;
    autoware_cmd.lateral.steering_tire_angle;

    autoware_msgs::msg::ControlCommandStamped carma_cmd = worker_node->convert_cmd(autoware_cmd);

    ASSERT_NEAR(carma_cmd.cmd.linear_acceleration, autoware_cmd.longitudinal.acceleration, 0.001);
    ASSERT_NEAR(carma_cmd.cmd.linear_velocity , autoware_cmd.longitudinal.speed, 0.001);
    ASSERT_NEAR(carma_cmd.cmd.steering_angle , autoware_cmd.lateral.steering_tire_angle, 0.001);

}



int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
}
