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


TEST(Testtrajectory_follower_wrapper, TestControlCallback){
// tetsing the overall control callback as well as combination of the conversions and time checks
    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<trajectory_follower_wrapper::TrajectoryFollowerWrapperNode>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    std::shared_ptr<autoware_auto_msgs::msg::AckermannControlCommand> aw_cmd = std::make_shared<autoware_auto_msgs::msg::AckermannControlCommand>();
    aw_cmd->stamp = worker_node->now() - rclcpp::Duration::from_seconds(5);
    aw_cmd->longitudinal.acceleration = 1.0;
    aw_cmd->longitudinal.speed = 2.0;
    aw_cmd->lateral.steering_tire_angle = 0.5;

    worker_node->ackermann_control_cb(move(aw_cmd));

    auto res = worker_node->generate_command();
    // resulting control command is empty because trajectory and twist member objects were not set
    ASSERT_NEAR(res.cmd.linear_acceleration, 0.0, 0.0001);

    std::unique_ptr<carma_planning_msgs::msg::TrajectoryPlan> traj_plan = std::make_unique<carma_planning_msgs::msg::TrajectoryPlan>();

    carma_planning_msgs::msg::TrajectoryPlanPoint tpp, tpp2, tpp3;
    tpp.x = 100;
    tpp.y = 100;
    tpp.target_time = rclcpp::Time(1.0*1e9);  // 14.14 m/s

    tpp2.x = 110;
    tpp2.y = 110;
    tpp2.target_time = rclcpp::Time(2.0*1e9);  // 14.14 m/s

    tpp3.x = 120;
    tpp3.y = 120;
    tpp3.target_time = rclcpp::Time(3.0*1e9);  // 14.14 m/s

    carma_planning_msgs::msg::TrajectoryPlan plan;
    traj_plan->initial_longitudinal_velocity = 14.14;
    traj_plan->header.stamp = worker_node->now();
    traj_plan->trajectory_points = { tpp, tpp2, tpp3 };

    worker_node->current_trajectory_callback(move(traj_plan));

    std::unique_ptr<geometry_msgs::msg::TwistStamped> twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    twist_msg->header.stamp = worker_node->now();
    twist_msg->twist.linear.x = 4;
    twist_msg->twist.linear.y = 0;
    twist_msg->twist.linear.z = 0;
    worker_node->current_twist_callback(move(twist_msg));

    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();

    pose_msg->header.stamp = worker_node->now();
    pose_msg->pose.position.x = 1;
    pose_msg->pose.position.y = 2;
    pose_msg->pose.position.z = 3;
    worker_node->current_pose_callback(move(pose_msg));

    worker_node->config_.incoming_cmd_time_threshold = 10.0;

    // control successfully set since time threshold is increased and trajectory, pose and twist objects are set
    auto res2 = worker_node->generate_command();

    ASSERT_NEAR(res2.cmd.linear_acceleration, 1.0, 0.0001);
    ASSERT_NEAR(res2.cmd.linear_velocity, 2.0, 0.0001);
    ASSERT_NEAR(res2.cmd.steering_angle, 0.5, 0.0001);
}

TEST(Testtrajectory_follower_wrapper, TestThreshold){

    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<trajectory_follower_wrapper::TrajectoryFollowerWrapperNode>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    autoware_auto_msgs::msg::AckermannControlCommand aw_cmd;
    aw_cmd.stamp = worker_node->now() - rclcpp::Duration::from_seconds(5);
    aw_cmd.longitudinal.acceleration = 1.0;
    aw_cmd.longitudinal.speed = 2.0;

    bool res1 = worker_node->isControlCommandOld(aw_cmd);
    ASSERT_TRUE(res1);

    aw_cmd.stamp = worker_node->now();
    bool res2 = worker_node->isControlCommandOld(aw_cmd);
    ASSERT_FALSE(res2);

    std::string version = worker_node->get_version_id();
    ASSERT_EQ(version, "1.0");

    bool available = worker_node->get_availability();
    ASSERT_TRUE(available);
}

TEST(Testtrajectory_follower_wrapper, TestStateConversion)
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

TEST(Testtrajectory_follower_wrapper, TestControlConversion)
{
    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<trajectory_follower_wrapper::TrajectoryFollowerWrapperNode>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    auto converted_time_now = worker_node->now();

    autoware_auto_msgs::msg::AckermannControlCommand autoware_cmd;
    autoware_cmd.longitudinal.acceleration = 1.0;
    autoware_cmd.longitudinal.speed = 2.0;
    autoware_cmd.lateral.steering_tire_angle = 1.0;

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
