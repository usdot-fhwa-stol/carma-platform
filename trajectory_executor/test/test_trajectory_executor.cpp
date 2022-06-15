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

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>

#include "trajectory_executor/trajectory_executor_node.hpp"
#include "trajectory_executor_test_suite.cpp"

namespace trajectory_executor
{
    /*!
    * \brief Test that the trajectory executor will output the trajectory on 
    * the desired topic after receiving a trajectory plan for the Pure Pursuit controller plugin
    */
    TEST(TrajectoryExecutorTest, test_emit_multiple)
    {
        // Create and activate TrajectoryExecutor node
        rclcpp::NodeOptions options;
        auto traj_executor_node = std::make_shared<trajectory_executor::TrajectoryExecutor>(options);
        traj_executor_node->configure(); //Call configure state transition
        traj_executor_node->activate();  //Call activate state transition to get not read for runtime

        // Create and activate TrajectoryExecutorTestSuite node
        rclcpp::NodeOptions options2;
        auto test_suite_node = std::make_shared<trajectory_executor_test_suite::TrajectoryExecutorTestSuite>(options2);
        test_suite_node->configure();
        test_suite_node->activate();

        // Add these nodes to an executor to spin them and trigger callbacks
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(traj_executor_node->get_node_base_interface());
        executor.add_node(test_suite_node->get_node_base_interface());

        // Generate a 'Pure Pursuit' trajectory plan and publish it to the /trajectory topic
        carma_planning_msgs::msg::TrajectoryPlan plan = trajectory_executor_test_suite::buildSampleTraj();
        test_suite_node->traj_pub_->publish(plan);

        // Spin executor for 2 seconds
        auto end_time = std::chrono::system_clock::now() + std::chrono::seconds(2);
        while(std::chrono::system_clock::now() < end_time){
            executor.spin_once();
        }

        ASSERT_LE(10, test_suite_node->msg_count) << "Failed to receive whole trajectory from TrajectoryExecutor node.";
    }

    /*!
    * \brief Test that the trajectory executor will output the trajectory on 
    * the desired topic after receiving a trajectory plan for the PlatooningControlPlugin controller plugin
    */
    TEST(TrajectoryExecutorTestSuite, test_emit_traj) {
        // Create and activate TrajectoryExecutor node
        rclcpp::NodeOptions options;
        auto traj_executor_node = std::make_shared<trajectory_executor::TrajectoryExecutor>(options);
        traj_executor_node->configure(); //Call configure state transition
        traj_executor_node->activate();  //Call activate state transition to get not read for runtime

        // Create and activate TrajectoryExecutorTestSuite node
        rclcpp::NodeOptions options2;
        auto test_suite_node = std::make_shared<trajectory_executor_test_suite::TrajectoryExecutorTestSuite>(options2);
        test_suite_node->configure();
        test_suite_node->activate();

        // Add these nodes to an executor to spin them and trigger callbacks
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(traj_executor_node->get_node_base_interface());
        executor.add_node(test_suite_node->get_node_base_interface());

        // Generate a 'PlatooningControlPlugin' trajectory plan and publish it to the /trajectory topic
        carma_planning_msgs::msg::TrajectoryPlan plan = trajectory_executor_test_suite::buildSampleTraj2();
        test_suite_node->traj_pub_->publish(plan);

        // Spin executor for 2 seconds
        auto end_time = std::chrono::system_clock::now() + std::chrono::seconds(2);
        while(std::chrono::system_clock::now() < end_time){
            executor.spin_once();
        }

        ASSERT_GT(test_suite_node->msg_count, 0) << "Failed to receive whole trajectory from TrajectoryExecutor node.";
    } 

    /*!
    * \brief Test that the Trajectory Executor properly shuts down if it encounters a trajectory
    * containing a reference to a control plugin which was not discovered.
    */
    TEST(TrajectoryExecutorTestSuite, test_control_plugin_not_found) {
        // Create and activate TrajectoryExecutor node
        rclcpp::NodeOptions options;
        auto traj_executor_node = std::make_shared<trajectory_executor::TrajectoryExecutor>(options);
        traj_executor_node->configure(); //Call configure state transition
        traj_executor_node->activate();  //Call activate state transition to get not read for runtime

        // Create and activate TrajectoryExecutorTestSuite node
        rclcpp::NodeOptions options2;
        auto test_suite_node = std::make_shared<trajectory_executor_test_suite::TrajectoryExecutorTestSuite>(options2);
        test_suite_node->configure();
        test_suite_node->activate();

        // Add these nodes to an executor to spin them and trigger callbacks
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(traj_executor_node->get_node_base_interface());
        executor.add_node(test_suite_node->get_node_base_interface());

        // Generate a 'Pure Pursuit' trajectory plan
        carma_planning_msgs::msg::TrajectoryPlan plan = trajectory_executor_test_suite::buildSampleTraj();

        // Assign controller plugin name NULL to first trajectory plan point; this is a plugin that trajectory_executor will not recognize
        plan.trajectory_points[0].controller_plugin_name = "NULL";

        // Publish trajectory plan from the test suite node
        test_suite_node->traj_pub_->publish(plan);

        try {
            // Spin executor for 2 seconds
            auto end_time = std::chrono::system_clock::now() + std::chrono::seconds(2);
            while(std::chrono::system_clock::now() < end_time){
                executor.spin_some();
            }
        }
        catch (const std::exception& e)
        {
            ASSERT_EQ(test_suite_node->msg_count, 0) << "TrajectoryExecutor published plans that began with an unknown control plugin";
        }
    }

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