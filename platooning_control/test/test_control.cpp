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

namespace platoon_control
{
    TEST(PlatoonControlPluginTest, test_current_trajectory_callback)
    {
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<platoon_control::PlatoonControlPlugin>(options);

        carma_planning_msgs::msg::TrajectoryPlan traj_plan;

        carma_planning_msgs::msg::TrajectoryPlanPoint point1;
        point1.x = 30.0;
        point1.y = 20.0;

        carma_planning_msgs::msg::TrajectoryPlanPoint point2;
        point2.x = 50.0;
        point2.y = 60.0;


        traj_plan.trajectory_points = {point1, point2};

        worker_node->current_trajectory_callback(std::make_unique<carma_planning_msgs::msg::TrajectoryPlan>(traj_plan));
        EXPECT_EQ(worker_node->current_trajectory_.get().trajectory_points.size(), 2);

    }

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

        // Test parameter update
        rclcpp::ParameterValue bool_val(false);
        rclcpp::Parameter param("enable_max_accel_filter", bool_val);
        std::vector<rclcpp::Parameter> param_vec = {param};

        worker_node->parameter_update_callback(param_vec);
        EXPECT_FALSE(worker_node->config_.enable_max_accel_filter);
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

    TEST(PlatoonControlPluginTest, test_generate_controls)
    {
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<platoon_control::PlatoonControlPlugin>(options);

        worker_node->configure();
        worker_node->activate();

        platoon_control::PlatoonLeaderInfo msg;
        msg.staticId = "id";
        msg.commandSpeed = 2.0;
        msg.vehicleSpeed = 2.0;
        msg.vehiclePosition = 0.5;
        msg.leaderIndex = 0;
        msg.NumberOfVehicleInFront=0;

        worker_node->platoon_leader_ = msg;

        worker_node->trajectory_speed_ = 4.47;

        carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point;
        trajectory_point.x = 50.0;
        trajectory_point.y = 60.0;

        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.pose.position.x = 0.0;
        current_pose.pose.position.y = 0.0;

        geometry_msgs::msg::TwistStamped current_twist;
        current_twist.twist.linear.x = 0.0;

        carma_planning_msgs::msg::TrajectoryPlanPoint point1;
        point1.x = 50.0;
        point1.y = 60.0;
        carma_planning_msgs::msg::TrajectoryPlanPoint point2;
        point2.x = 52.0;
        point2.y = 60.0;

        carma_planning_msgs::msg::TrajectoryPlan traj_plan;
        traj_plan.trajectory_points = {point1, point2};

        worker_node->current_trajectory_callback(std::make_unique<carma_planning_msgs::msg::TrajectoryPlan>(traj_plan));
        auto control_cmd = worker_node->generate_control_signals(trajectory_point, current_pose, current_twist);
        EXPECT_NEAR(4.47, control_cmd.cmd.linear_velocity, 0.5);


    }
}