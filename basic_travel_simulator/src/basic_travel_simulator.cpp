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
#include <rclcpp/rclcpp.hpp>

#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/header.hpp>

namespace basic_travel_simulator
{
namespace std_ph = std::placeholders;

Node::Node(const rclcpp::NodeOptions & options) : carma_ros2_utils::CarmaLifecycleNode(options)

{
  // Create initial config
  config_ = BasicTravelSimulatorConfig();
  config_.pose_pub_rate = declare_parameter<double>("pose_pub_rate", config_.pose_pub_rate);
  config_.traj_idx_buffer = declare_parameter<int>("traj_idx_buffer", config_.traj_idx_buffer);

}

rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto error = update_params<double>({{"pose_pub_rate", config_.pose_pub_rate}}, parameters);
  auto error2 = update_params<int>({{"traj_idx_buffer", config_.traj_idx_buffer}}, parameters);


  rcl_interfaces::msg::SetParametersResult result;

  result.successful = !error && !error2;

  return result;
}

carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
{
  // Reset config
  config_ = BasicTravelSimulatorConfig();

  get_parameter<double>("pose_pub_rate", config_.pose_pub_rate);
  get_parameter<int>("traj_idx_buffer", config_.traj_idx_buffer);

  // Register runtime parameter update callback
  add_on_set_parameters_callback(std::bind(&Node::parameter_update_callback, this, std_ph::_1));

  // Setup publishers
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
  current_speed_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("vehicle/twist", 10);

  // Setup subscribers
  trajectory_sub_ = create_subscription<carma_planning_msgs::msg::TrajectoryPlan>(
    "plan_trajectory", 5, std::bind(&Node::currentTrajectoryCallback, this, std_ph::_1));

  // Return success if everything initialized successfully
  return CallbackReturn::SUCCESS;
}

carma_ros2_utils::CallbackReturn Node::handle_on_activate(const rclcpp_lifecycle::State &)
{
  // Setup timer
  all_publisher_timer_ = create_timer(
    get_clock(), std::chrono::milliseconds(int(1 / config_.pose_pub_rate * 1000)),
    std::bind(&Node::statusTick, this));
  return CallbackReturn::SUCCESS;
}
////////////////////
// NON-MEMBER PRIVATE FUNCTIONS
////////////////////

int validateIdx(int idx, int max_idx_allowed)
{
  auto validated_idx = idx;
  // In case there is invalid input,
  // just force some value, which is sufficient for estimation
  if (idx <= 0) {
    validated_idx = 1;
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("basic_travel_simulator"),
      "Forcing current pose idx to be 1 to get a valid speed.");
  } else if (idx > max_idx_allowed) {
    validated_idx = max_idx_allowed - 1;
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("basic_travel_simulator"),
      "Forcing current pose idx to be the last point: " << validated_idx
                                                        << ", to get a valid speed.");
  }
  return validated_idx;
}

double get_2d_distance(
  const carma_planning_msgs::msg::TrajectoryPlanPoint & point_n,
  const carma_planning_msgs::msg::TrajectoryPlanPoint & point_n_minus_1)
{
  double dx = point_n.x - point_n_minus_1.x;
  double dy = point_n.y - point_n_minus_1.y;
  return std::sqrt(dx * dx + dy * dy);
}

int Node::getCurrentPoseIndexFromTraj(
  double last_speed, double time_elapsed_since_last_s,
  const carma_planning_msgs::msg::TrajectoryPlan & traj)
{
  // Distance actually traveled
  double distance_traveled = last_speed * time_elapsed_since_last_s;

  // Get corresponding idx to the distance traveled from the traj
  // start from last_idx_ which is the last idx the pose was in this traj
  auto idx = last_idx_;
  double dist_accumulator = 0.0;
  for (auto i = last_idx_; i < traj.trajectory_points.size(); i++) {
    // Extract the i-th point
    const auto & point_i = traj.trajectory_points[i];
    const auto & point_i_minus_1 = traj.trajectory_points[i - 1];

    // Estimate with 2d distance since points are usually close
    dist_accumulator += get_2d_distance(point_i, point_i_minus_1);

    // If traveled enough distance or iterated until the last point available
    if (dist_accumulator >= distance_traveled || i == traj.trajectory_points.size() - 1) {
      idx = i;
      break;
    }
  }
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("basic_travel_simulator"), "Calculated idx from trajectory with buffer: " << idx);
  return idx;
}

geometry_msgs::msg::PoseStamped Node::composePoseStamped(
  int idx, const carma_planning_msgs::msg::TrajectoryPlan & traj)
{
  geometry_msgs::msg::PoseStamped pose_msg;
  // Get point_idx
  const auto & point_n = traj.trajectory_points[idx];

  // Fill in the PoseStamped message
  pose_msg.header = traj.header;
  pose_msg.pose.position.x = point_n.x;
  pose_msg.pose.position.y = point_n.y;
  pose_msg.pose.position.z = 0.0;  // Assuming 2D, z is 0

  tf2::Quaternion q;
  q.setRPY(0, 0, point_n.yaw);
  pose_msg.pose.orientation = tf2::toMsg(q);
  return pose_msg;
}

geometry_msgs::msg::TwistStamped Node::composeTwistStamped(
  double current_linear_speed, double current_angular_speed,
  const carma_planning_msgs::msg::TrajectoryPlan & traj)
{
  geometry_msgs::msg::TwistStamped twist_msg;
  // Fill in the TwistStamped message
  twist_msg.header = traj.header;
  twist_msg.twist.linear.x = current_linear_speed;
  twist_msg.twist.linear.y = 0.0;
  twist_msg.twist.linear.z = 0.0;
  twist_msg.twist.angular.x = 0.0;
  twist_msg.twist.angular.y = 0.0;
  // anuglar speed is not working currently
  twist_msg.twist.angular.z = 0.0;
  return twist_msg;
}

double Node::getCurrentLinearSpeedFromTraj(
  int input_idx, const carma_planning_msgs::msg::TrajectoryPlan & traj)
{
  auto idx = validateIdx(input_idx, traj.trajectory_points.size() - 1);
  const auto & point_n = traj.trajectory_points[idx];
  const auto & point_n_minus_1 = traj.trajectory_points[idx - 1];

  // Calculate the speed
  double dx = point_n.x - point_n_minus_1.x;
  double dy = point_n.y - point_n_minus_1.y;
  double dt =
    (rclcpp::Time(point_n.target_time) - rclcpp::Time(point_n_minus_1.target_time)).seconds();
  double speed = std::sqrt(dx * dx + dy * dy) / dt;
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("basic_travel_simulator"), "Calculated current_speed: " << speed);
  return speed;
}

double Node::getCurrentAngularSpeedFromTraj(
  double last_yaw, double time_elapsed_since_last_s, int input_idx,
  const carma_planning_msgs::msg::TrajectoryPlan & traj)
{
  auto idx = validateIdx(input_idx, traj.trajectory_points.size() - 1);
  const auto & point_n = traj.trajectory_points[idx];
  double current_yaw_rate = (point_n.yaw - last_yaw) / time_elapsed_since_last_s;

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("basic_travel_simulator"),
    "Calculated current yaw_rate from trajectory: " << current_yaw_rate);

  // Calculate the angular speed
  return current_yaw_rate;
}

void Node::statusTick()
{
  double operation_period_s = 1 / config_.pose_pub_rate;
  if (current_trajectory_.trajectory_points.size() < 2) {
    return;
  }

  auto current_pose_idx =
    getCurrentPoseIndexFromTraj(last_linear_speed_, operation_period_s, current_trajectory_);

  double current_linear_speed =
    getCurrentLinearSpeedFromTraj(current_pose_idx, current_trajectory_);

  double current_angular_speed = getCurrentAngularSpeedFromTraj(
    last_yaw_, operation_period_s, current_pose_idx, current_trajectory_);

  const auto & current_pose_msg = composePoseStamped(current_pose_idx, current_trajectory_);

  const auto & current_speed_msg =
    composeTwistStamped(current_linear_speed, current_angular_speed, current_trajectory_);

  // Publish the messages
  pose_pub_->publish(current_pose_msg);
  current_speed_pub_->publish(current_speed_msg);

  last_yaw_ = current_trajectory_.trajectory_points[current_pose_idx].yaw;
  last_linear_speed_ = current_linear_speed;
  last_idx_ = current_pose_idx;
  last_traj_first_point_ = current_trajectory_.trajectory_points.front();
}

void Node::currentTrajectoryCallback(carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg)
{
  RCLCPP_DEBUG(rclcpp::get_logger("basic_travel_simulator"), "Received trajectory message");

  // Check if the new trajectory is different from the current one
  if (current_trajectory_.trajectory_points.empty() || 
      get_2d_distance(current_trajectory_.trajectory_points.front(), msg->trajectory_points.front()) > 0.1) 
  {
    current_trajectory_ = *msg;
    last_idx_ = 1 + config_.traj_idx_buffer;  // Reset the index for a new trajectory
  }
}

}  // namespace basic_travel_simulator

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(basic_travel_simulator::Node)
