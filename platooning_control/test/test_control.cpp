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

#include "platoon_control/platoon_control.hpp"


TEST(PlatoonControlPluginTest, test1)
{
    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<platoon_control::PlatoonControlPlugin>(options);

    worker_node->configure();
    worker_node->activate();

    carma_planning_msgs::msg::TrajectoryPlan tp;
    carma_planning_msgs::msg::TrajectoryPlanPoint point1;
    point1.x = 1.0;
    point1.y = 1.0;

    carma_planning_msgs::msg::TrajectoryPlanPoint point2;
    point2.x = 10.0;
    point2.y = 10.0;

    carma_planning_msgs::msg::TrajectoryPlanPoint point3;
    point3.x = 20.0;
    point3.y = 20.0;

    tp.trajectory_points = {point1, point2, point3};

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.twist.linear.x = 0.0;

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;

    carma_planning_msgs::msg::TrajectoryPlanPoint out = worker_node->get_lookahead_trajectory_point(tp, pose_msg, twist_msg);
    EXPECT_EQ(out.x, 10.0);

}

TEST(PlatoonControlPluginTest, test_2)
{
    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<platoon_control::PlatoonControlPlugin>(options);

    EXPECT_TRUE(worker_node->get_availability());
    EXPECT_EQ(worker_node->get_version_id(), "v1.0");

    EXPECT_EQ(worker_node->generate_command().cmd.linear_velocity, 0.0);

}

TEST(PlatoonControlPluginTest, test_convert_state)
{
    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<platoon_control::PlatoonControlPlugin>(options);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;
    pose_msg.pose.orientation.z = 0.0;

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.twist.linear.x = 0.0;

    auto converted_state = worker_node->convert_state(pose_msg, twist_msg);
    EXPECT_EQ(converted_state.state.x, pose_msg.pose.position.x);
    EXPECT_EQ(converted_state.state.heading.imag, pose_msg.pose.orientation.z);

}

TEST(PlatoonControlPluginTest, test_compose_twist_cmd)
{
    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<platoon_control::PlatoonControlPlugin>(options);

    auto linear_vel = 1.0;
    auto angular_vel = 2.0;

    auto twist_cmd = worker_node->compose_twist_cmd(linear_vel,angular_vel);

    EXPECT_EQ(twist_cmd.twist.linear.x, linear_vel);
    EXPECT_EQ(twist_cmd.twist.angular.z, angular_vel);
}

TEST(PlatoonControlPluginTest, test_current_trajectory_callback)
{
    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<platoon_control::PlatoonControlPlugin>(options);

    carma_planning_msgs::msg::TrajectoryPlan traj_plan;

    carma_planning_msgs::msg::TrajectoryPlanPoint point1;
    point1.x = 30.0;
    point1.y = 20.0;

    traj_plan.trajectory_points = {point1};
    worker_node->current_trajectory_callback(std::make_unique<carma_planning_msgs::msg::TrajectoryPlan>(traj_plan));
    EXPECT_EQ(worker_node->latest_trajectory_.trajectory_points.size(), 0);

    carma_planning_msgs::msg::TrajectoryPlanPoint point2;
    point2.x = 50.0;
    point2.y = 60.0;


    traj_plan.trajectory_points = {point1, point2};

    worker_node->current_trajectory_callback(std::make_unique<carma_planning_msgs::msg::TrajectoryPlan>(traj_plan));
    EXPECT_EQ(worker_node->latest_trajectory_.trajectory_points.size(), 2);

}

namespace platoon_control
{

    TEST(PlatoonControlPluginTest, test_platoon_info_cb)
    {
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<platoon_control::PlatoonControlPlugin>(options);

        worker_node->configure();
        worker_node->activate();

        carma_planning_msgs::msg::PlatooningInfo msg;
        msg.leader_id = "id";
        msg.leader_downtrack_distance = 2.0;
        msg.leader_cmd_speed = 1.0;
        msg.host_platoon_position=1;

        worker_node->platoon_info_cb(std::make_shared<carma_planning_msgs::msg::PlatooningInfo>(msg));

    }

    TEST(PlatoonControlPluginTest, test_get_trajectory_speed)
    {

        carma_planning_msgs::msg::TrajectoryPlanPoint point;
        point.x = 30.0;
        point.y = 15.0;
        point.target_time.sec = 0;

        carma_planning_msgs::msg::TrajectoryPlanPoint point2;
        point2.x = 50.0;
        point2.y = 60.0;
        point2.target_time.sec = 1.0;

        carma_planning_msgs::msg::TrajectoryPlanPoint point3;
        point3.x = 55.0;
        point3.y = 60.0;
        point3.target_time.sec = 10.0;

        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<platoon_control::PlatoonControlPlugin>(options);


        EXPECT_NEAR(worker_node->get_trajectory_speed({point,point2,point3}), 5.1, 0.1);
    }
}