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

#include "platooning_control/platooning_control.hpp"


TEST(PurePursuitTest, sanity_check)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<platooning_control::PlatooningControlPlugin>(options);
  node->configure();
  node->activate();

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
  plan.initial_longitudinal_velocity = 14.14;

  motion::control::controller_common::State state_tf;
  auto converted_time_now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

  state_tf.header.stamp = rclcpp::Time(converted_time_now*1e9);

  state_tf.state.heading.real = 3.14 / 2;
  state_tf.state.heading.imag = 1.0;

  state_tf.state.x = 0;
  state_tf.state.y = 0;
  state_tf.state.longitudinal_velocity_mps = 4.0; //arbitrary speed for first point
  plan.header.frame_id = state_tf.header.frame_id;
  plan.header.stamp = rclcpp::Time(converted_time_now*1e9) + rclcpp::Duration::from_nanoseconds(1.0*1e9);

  plan.trajectory_points = { tpp, tpp2, tpp3 };

  auto traj = basic_autonomy::waypoint_generation::process_trajectory_plan(plan, 0.0);
  node->pp_->set_trajectory(traj);

  const auto cmd{node->pp_->compute_command(state_tf)};

  ASSERT_NEAR(cmd.front_wheel_angle_rad, -0.294355, 0.005);
  ASSERT_NEAR(cmd.long_accel_mps2, 0.311803, 0.005);
  ASSERT_NEAR(cmd.rear_wheel_angle_rad, 0, 0.001);
  ASSERT_NEAR(cmd.velocity_mps, 14.14, 0.01);

  auto steer_cmd = cmd.front_wheel_angle_rad;

  auto speed_cmd = 5.0;
  auto converted_cmd = node->compose_ctrl_cmd(speed_cmd, steer_cmd);

  ASSERT_NEAR(converted_cmd.cmd.linear_velocity, speed_cmd, 0.01);
  ASSERT_NEAR(converted_cmd.cmd.steering_angle, cmd.front_wheel_angle_rad, 0.001);
}
