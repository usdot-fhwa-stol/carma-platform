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
#include "trajectory_follower_wrapper/trajectory_follower_wrapper_node.hpp"

namespace trajectory_follower_wrapper
{
  namespace std_ph = std::placeholders;

  TrajectoryFollowerWrapperNode::TrajectoryFollowerWrapperNode(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::ControlPlugin(options)
  {
    // Create initial config
    config_ = TrajectoryFollowerWrapperConfig();
    config_.vehicle_response_lag = declare_parameter<double>("vehicle_response_lag", config_.vehicle_response_lag);
    config_.incoming_cmd_time_threshold = declare_parameter<double>("incoming_cmd_time_threshold", config_.incoming_cmd_time_threshold);

  }

  rcl_interfaces::msg::SetParametersResult TrajectoryFollowerWrapperNode::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error_double = update_params<double>({
      {"vehicle_response_lag", config_.vehicle_response_lag},
      {"incoming_cmd_time_threshold", config_.incoming_cmd_time_threshold}
      }, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error_double;

    return result;
  }

  carma_ros2_utils::CallbackReturn TrajectoryFollowerWrapperNode::on_configure_plugin()
  {
    // Reset config
    config_ = TrajectoryFollowerWrapperConfig();

    // Load parameters
    get_parameter<double>("vehicle_response_lag", config_.vehicle_response_lag);
    get_parameter<double>("incoming_cmd_time_threshold", config_.incoming_cmd_time_threshold);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("trajectory_follower_wrapper"), "Loaded Params: " << config_);
    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&TrajectoryFollowerWrapperNode::parameter_update_callback, this, std_ph::_1));

    // NOTE: Currently, intra-process comms must be disabled for the following subscriber that is transient_local: https://github.com/ros2/rclcpp/issues/1753

    rclcpp::SubscriptionOptions intra_proc_disabled;
    intra_proc_disabled.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

    auto sub_qos_transient_local = rclcpp::QoS(rclcpp::KeepAll()); // A subscriber with this QoS will store all messages that it has sent on the topic
    sub_qos_transient_local.transient_local();
    // Setup subscriber
    control_cmd_sub_ = create_subscription<autoware_auto_msgs::msg::AckermannControlCommand>("output/control_cmd", sub_qos_transient_local,
      std::bind(&TrajectoryFollowerWrapperNode::ackermann_control_cb, this, std::placeholders::_1),intra_proc_disabled);

    // Setup publishers
    autoware_traj_pub_ = create_publisher<autoware_auto_msgs::msg::Trajectory>("input/reference_trajectory", 10);
    autoware_state_pub_ = create_publisher<autoware_auto_msgs::msg::VehicleKinematicState>("input/current_kinematic_state", 10);

    // Setup timers to publish autoware compatible info (trajectory and state)
    autoware_info_timer_ = create_timer(
        get_clock(),
        std::chrono::milliseconds(33), // Spin at 30 Hz per plugin API
        std::bind(&TrajectoryFollowerWrapperNode::autoware_info_timer_callback, this));

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }


  void TrajectoryFollowerWrapperNode::autoware_info_timer_callback()
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("trajectory_follower_wrapper"), "In autoware info timer callback");

    if (current_trajectory_ && current_pose_ && current_twist_)
    {
      // generate and publish autoware kinematic state
      auto autoware_state = convert_state(current_pose_.get(), current_twist_.get());
      autoware_state_pub_->publish(autoware_state);

      // generate and publish autoware trajectory
      current_trajectory_.get().header.frame_id = autoware_state.header.frame_id;
      auto autoware_traj_plan = basic_autonomy::waypoint_generation::process_trajectory_plan(current_trajectory_.get(), config_.vehicle_response_lag);
      // autoware_traj_plan.points[0].heading.real = current_pose_.get().pose.orientation.w;
      // autoware_traj_plan.points[0].heading.imag = current_pose_.get().pose.orientation.z;

      for (size_t i=1; i< autoware_traj_plan.points.size(); i++ )
      {

        double yaw = calc_point_to_point_yaw(autoware_traj_plan.points[i-1], autoware_traj_plan.points[i]);
        autoware_traj_plan.points[i-1].heading.real = std::cos(yaw/2);
        autoware_traj_plan.points[i-1].heading.imag = std::sin(yaw/2);

      }
      autoware_traj_pub_->publish(autoware_traj_plan);

    }
  }

  double TrajectoryFollowerWrapperNode::calc_point_to_point_yaw(const autoware_auto_msgs::msg::TrajectoryPoint& cur_point,
                                                                const autoware_auto_msgs::msg::TrajectoryPoint& next_point)
  {
    double dx = next_point.x - cur_point.x;
    double dy = next_point.y - cur_point.y;
    double yaw = std::atan2(dy, dx);
    return yaw;
  }

  void TrajectoryFollowerWrapperNode::ackermann_control_cb(const autoware_auto_msgs::msg::AckermannControlCommand::SharedPtr msg)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("trajectory_follower_wrapper"), "In ackermann control callback");
    received_ctrl_command_ = *msg;
  }


  autoware_auto_msgs::msg::VehicleKinematicState TrajectoryFollowerWrapperNode::convert_state(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::TwistStamped& twist) const
  {
    autoware_auto_msgs::msg::VehicleKinematicState state;
    state.header = pose.header;
    state.state.x = pose.pose.position.x;
    state.state.y = pose.pose.position.y;
    state.state.z = pose.pose.position.z;
    state.state.heading.real = pose.pose.orientation.w;
    state.state.heading.imag = pose.pose.orientation.z;
    state.state.front_wheel_angle_rad = steering_angle_;

    state.state.longitudinal_velocity_mps = twist.twist.linear.x;
    return state;
  }

  autoware_msgs::msg::ControlCommandStamped TrajectoryFollowerWrapperNode::convert_cmd(const autoware_auto_msgs::msg::AckermannControlCommand& cmd)
  {
    autoware_msgs::msg::ControlCommandStamped return_cmd;
    return_cmd.header.stamp = cmd.stamp;

    return_cmd.cmd.linear_acceleration = cmd.longitudinal.acceleration;
    return_cmd.cmd.linear_velocity = cmd.longitudinal.speed;
    return_cmd.cmd.steering_angle = cmd.lateral.steering_tire_angle/15.0;
    steering_angle_ = return_cmd.cmd.steering_angle;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("trajectory_follower_wrapper"), "generated command cmd.stamp: " << std::to_string(rclcpp::Time(cmd.stamp).seconds()));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("trajectory_follower_wrapper"), "generated command cmd.longitudinal.acceleration: " << cmd.longitudinal.acceleration);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("trajectory_follower_wrapper"), "generated command cmd.longitudinal.speed: " << cmd.longitudinal.speed);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("trajectory_follower_wrapper"), "generated command cmd.lateral.steering_tire_angle: " << cmd.lateral.steering_tire_angle);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("trajectory_follower_wrapper"), "generated command cmd.lateral.steering_tire_rotation_rate: " << cmd.lateral.steering_tire_rotation_rate);

    return return_cmd;
  }

  autoware_msgs::msg::ControlCommandStamped TrajectoryFollowerWrapperNode::generate_command()
  {
    // process and save the trajectory
    autoware_msgs::msg::ControlCommandStamped converted_cmd;

    if (!current_trajectory_ || !current_pose_ || !current_twist_ || !received_ctrl_command_.has_value())
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("trajectory_follower_wrapper"), "Insufficient data, empty control command generated");
      return converted_cmd;
    }


    if (isControlCommandOld(received_ctrl_command_.value()))
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("trajectory_follower_wrapper"), "Control Command is old, empty control command generated");
      return converted_cmd;
    }

    converted_cmd = convert_cmd(received_ctrl_command_.value());

    return converted_cmd;
  }

  bool TrajectoryFollowerWrapperNode::isControlCommandOld(const autoware_auto_msgs::msg::AckermannControlCommand& cmd) const
  {
    double difference = std::abs(this->now().seconds() - rclcpp::Time(cmd.stamp).seconds());

    if (difference >= config_.incoming_cmd_time_threshold)
    {
      return true;
    }

    return false;
  }

  bool TrajectoryFollowerWrapperNode::get_availability()
  {
    return true;
  }

  std::string TrajectoryFollowerWrapperNode::get_version_id()
  {
    return "1.0";
  }


} // trajectory_follower_wrapper

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_follower_wrapper::TrajectoryFollowerWrapperNode)
