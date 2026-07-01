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

#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <basic_travel_simulator/basic_travel_simulator_config.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

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
  double last_linear_speed_ = 0.0;
  double last_yaw_ = 0.0;
  int last_idx_ = 1;
  std::optional<carma_planning_msgs::msg::TrajectoryPlanPoint> last_traj_first_point_;

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
  double getCurrentLinearSpeedFromTraj(
    int input_idx, const carma_planning_msgs::msg::TrajectoryPlan & traj);
  double getCurrentAngularSpeedFromTraj(
    double last_yaw, double time_elapsed_since_last_s, int input_idx,
    const carma_planning_msgs::msg::TrajectoryPlan & traj);
  int getCurrentPoseIndexFromTraj(
    double last_linear_speed, double time_elapsed_since_last_s,
    const carma_planning_msgs::msg::TrajectoryPlan & traj);
  geometry_msgs::msg::PoseStamped composePoseStamped(
    int idx, const carma_planning_msgs::msg::TrajectoryPlan & traj);
  geometry_msgs::msg::TwistStamped composeTwistStamped(
    double current_linear_speed, double current_angular_speed,
    const carma_planning_msgs::msg::TrajectoryPlan & traj);
  void statusTick();

  ////
  // Overrides
  ////
  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
  carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);
  /**
   * \brief Callback for dynamic parameter updates
   */
  rcl_interfaces::msg::SetParametersResult parameter_update_callback(
    const std::vector<rclcpp::Parameter> & parameters);
};
}  // namespace basic_travel_simulator
