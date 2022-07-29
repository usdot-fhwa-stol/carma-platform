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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace trajectory_executor_test_suite
{

    namespace std_ph = std::placeholders;

    /**
     * TrajectoryExecutorTestSuite: Test fixture for TrajectoryExecutor testing
     * Maintains publishers, subscribers, and message tracking for all tests.
     * State is reset between tests to ensure clean results.
     */
    class TrajectoryExecutorTestSuite : public carma_ros2_utils::CarmaLifecycleNode
    {

    public:
        /*!
        * \brief Constructor for TrajectoryExecutorTestSuite
        */
        explicit TrajectoryExecutorTestSuite(const rclcpp::NodeOptions &options) 
            : carma_ros2_utils::CarmaLifecycleNode(options) {}

        // Publisher
        carma_ros2_utils::PubPtr<carma_planning_msgs::msg::TrajectoryPlan> traj_pub_; 

        // Subscribers
        carma_ros2_utils::SubPtr<carma_planning_msgs::msg::TrajectoryPlan> traj_sub_;
        carma_ros2_utils::SubPtr<carma_planning_msgs::msg::TrajectoryPlan> traj_sub2_;

        int msg_count = 0;

        void trajEmitCallback(carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg) {
            msg_count++;
        }

        ////
        // Overrides
        ////
        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &) {
            // Setup Publisher
            traj_pub_ = create_publisher<carma_planning_msgs::msg::TrajectoryPlan>("trajectory", 5);

            // Setup Subscribers
            traj_sub_ = create_subscription<carma_planning_msgs::msg::TrajectoryPlan>("/guidance/pure_pursuit/plan_trajectory", 100,
                                                                    std::bind(&TrajectoryExecutorTestSuite::trajEmitCallback, this, std_ph::_1));
            traj_sub2_ = create_subscription<carma_planning_msgs::msg::TrajectoryPlan>("/guidance/PlatooningControlPlugin/plan_trajectory", 100,
                                                                    std::bind(&TrajectoryExecutorTestSuite::trajEmitCallback, this, std_ph::_1));
            
            return CallbackReturn::SUCCESS;
        }

    };

    /*!
    * \brief Helper Function: Builds a small sample TrajectoryPlan message for the Pure Pursuit 
    *                         controller plugin
    * 
    * \return A 10-point TrajectoryPlan message containing sample data
    */
    carma_planning_msgs::msg::TrajectoryPlan buildSampleTraj() {
        carma_planning_msgs::msg::TrajectoryPlan plan;

        plan.header.stamp = rclcpp::Time(0,0);
        plan.trajectory_id = "TEST TRAJECTORY 1";

        rclcpp::Time cur_time = rclcpp::Time(0,0);
        for (int i = 0; i < 10; i++) {
            carma_planning_msgs::msg::TrajectoryPlanPoint p;
            p.controller_plugin_name = "Pure Pursuit";
            p.lane_id = "0";
            p.planner_plugin_name = "cruising";
            rclcpp::Duration dur((i * 0.13)*1e9); // Convert seconds to nanoseconds
            p.target_time = cur_time + dur;
            p.x = 10 * i;
            p.y = 10 * i;
            plan.trajectory_points.push_back(p);
        }

        return plan;
    }

    /*!
    * \brief Helper Function: Builds a small sample TrajectoryPlan message for the PlatooningControlPlugin 
    *                         controller plugin
    * 
    * \return A 10-point TrajectoryPlan message containing sample data
    */
    carma_planning_msgs::msg::TrajectoryPlan buildSampleTraj2() {
        carma_planning_msgs::msg::TrajectoryPlan plan;

        plan.header.stamp = rclcpp::Time(0,0);
        plan.trajectory_id = "TEST TRAJECTORY 2";

        rclcpp::Time cur_time = rclcpp::Time(0,0);
        for (int i = 0; i < 10; i++) {
            carma_planning_msgs::msg::TrajectoryPlanPoint p;
            p.controller_plugin_name = "PlatooningControlPlugin";
            p.lane_id = "0";
            p.planner_plugin_name = "cruising";
            rclcpp::Duration dur((i * 0.13)*1e9); // Convert seconds to nanoseconds
            p.target_time = cur_time + dur;
            p.x = 10 * i;
            p.y = 10 * i;
            plan.trajectory_points.push_back(p);
        }

        return plan;
    }

} // trajectory_executor_test_suite