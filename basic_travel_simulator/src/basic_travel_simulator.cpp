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
#include "basic_travel_simulator/basic_travel_simulator.hpp"
#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <carma_ros2_utils/timers/ROSTimerFactory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>


namespace basic_travel_simulator
{
    namespace std_ph = std::placeholders;

    Node::Node(const rclcpp::NodeOptions &options)
        : carma_ros2_utils::CarmaLifecycleNode(options)

    {
        // Create initial config
        config_ = BasicTravelSimulatorConfig();
        config_.pose_pub_rate = declare_parameter<double>("pose_pub_rate", config_.pose_pub_rate);
        config_.nth_point = declare_parameter<int>("nth_point", config_.nth_point);
    }

    rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        auto error = update_params<double>(
        {{"pose_pub_rate", config_.pose_pub_rate}}, parameters);
        
        auto error_2 = update_params<int>(
        {{"nth_point", config_.nth_point}}, parameters);

        rcl_interfaces::msg::SetParametersResult result;

        result.successful = !error && !error_2;

        return result;
    }

    carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        // Reset config
        config_ = BasicTravelSimulatorConfig();

        get_parameter<double>("pose_pub_rate", config_.pose_pub_rate);
        get_parameter<int>("nth_point", config_.nth_point);

        // Register runtime parameter update callback
        add_on_set_parameters_callback(std::bind(&Node::parameter_update_callback, this, std_ph::_1));

        // Setup publishers
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
        current_speed_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("vehicle/twist", 10);

        // Setup subscribers
        trajectory_sub_ = create_subscription<carma_planning_msgs::msg::TrajectoryPlan>("plan_trajectory", 5,
                                                                               std::bind(&Node::currentTrajectoryCallback, this, std_ph::_1));

        // Return success if everything initialized successfully
        return CallbackReturn::SUCCESS;
    }

    carma_ros2_utils::CallbackReturn Node::handle_on_activate(const rclcpp_lifecycle::State &)
    {
        // Setup timer
        all_publisher_timer_ = create_timer(get_clock(), std::chrono::milliseconds(int(1 / config_.pose_pub_rate * 1000)),
                                   std::bind(&Node::statusTick, this));
        return CallbackReturn::SUCCESS;
    }

    
    void Node::statusTick() {
        // Use the latest trajectory to generate pose and speed
        // current_trajectory_ is the TrajectoryPlan
        if (current_trajectory_.trajectory_points.size() <= config_.nth_point) {
            RCLCPP_WARN(this->get_logger(), "Not enough points in current trajectory.");
            return;
        }

        geometry_msgs::msg::PoseStamped current_pose;
        geometry_msgs::msg::TwistStamped current_speed;

        // Extract the n-th point
        const auto& point_n = current_trajectory_.trajectory_points[config_.nth_point];
        const auto& point_n_minus_1 = current_trajectory_.trajectory_points[config_.nth_point - 1];

        // Fill in the PoseStamped message
        current_pose.header = current_trajectory_.header;
        current_pose.pose.position.x = point_n.x;
        current_pose.pose.position.y = point_n.y;
        current_pose.pose.position.z = 0.0; // Assuming 2D, z is 0

        tf2::Quaternion q;
        q.setRPY(0, 0, point_n.yaw);
        current_pose.pose.orientation = tf2::toMsg(q);

        // Calculate the speed
        double dx = point_n.x - point_n_minus_1.x;
        double dy = point_n.y - point_n_minus_1.y;
        double dt = (rclcpp::Time(point_n.target_time) - rclcpp::Time(point_n_minus_1.target_time)).seconds();
        double speed = std::sqrt(dx * dx + dy * dy) / dt;

        // Fill in the TwistStamped message
        current_speed.header = current_trajectory_.header;
        current_speed.twist.linear.x = speed;
        current_speed.twist.linear.y = 0.0;
        current_speed.twist.linear.z = 0.0;
        current_speed.twist.angular.x = 0.0;
        current_speed.twist.angular.y = 0.0;
        current_speed.twist.angular.z = 0.0;

        // Publish the messages
        pose_pub_->publish(current_pose);
        current_speed_pub_->publish(current_speed);
    }

    void Node::currentTrajectoryCallback(carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("basic_travel_simulator"), "Received trajectory message");
        current_trajectory_ = *msg;
    }


} // namespace basic_travel_simulator

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(basic_travel_simulator::Node)
