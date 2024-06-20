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
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <basic_travel_simulator/basic_travel_simulator_config.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace basic_travel_simulator
{
    /**
     * \brief Core execution node for this package
     */

    class Node : public carma_ros2_utils::CarmaLifecycleNode
    {
    private:
        // Subscribers
        carma_ros2_utils::SubPtr<carma_planning_msgs::msg::TrajectoryPlan> trajectory_sub_;
        carma_planning_msgs::msg::TrajectoryPlan current_trajectory_;

        // Publishers
        carma_ros2_utils::PubPtr<geometry_msgs::msg::PoseStamped> pose_pub_;
        carma_ros2_utils::PubPtr<geometry_msgs::msg::TwistStamped> current_speed_pub_;

        // Timers
        rclcpp::TimerBase::SharedPtr all_publisher_timer_;
        
        // Node configuration
        BasicTravelSimulatorConfig config_;

    public:
        /**
         * \brief Node constructor
         */
        explicit Node(const rclcpp::NodeOptions &);

        void currentTrajectoryCallback(carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg);
        void publishCurrentVelocity();
        void publishCurrentPose();
        void statusTick();
        ////
        // Overrides
        ////
        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
        carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);
        /**
         * \brief Callback for dynamic parameter updates
         */
        rcl_interfaces::msg::SetParametersResult 
        parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    };
}