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
#include <pure_pursuit_wrapper/pure_pursuit_wrapper.hpp>
#include <gtest/gtest.h>
#include <motion_testing/motion_testing.hpp>
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#include <time_utils/time_utils.hpp>
#include <common/types.hpp>

#include <algorithm>

using motion::motion_testing::constant_velocity_trajectory;
using motion::motion_testing::make_state;
using autoware::motion::control::pure_pursuit::Config;
using autoware::motion::control::pure_pursuit::PurePursuit;
using autoware::motion::control::pure_pursuit::VehicleControlCommand;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace pure_pursuit_wrapper {

TEST(PurePursuitTest, sanity_check)
{
  auto node = std::make_shared<pure_pursuit_wrapper::PurePursuitWrapperNode>(rclcpp::NodeOptions());
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
  plan.header.stamp = rclcpp::Time(converted_time_now*1e9) + rclcpp::Duration(1.0*1e9);

  plan.trajectory_points = { tpp, tpp2, tpp3 };

  auto traj = basic_autonomy::waypoint_generation::process_trajectory_plan(plan, 0.0);
  node->pp_->set_trajectory(traj);

  const auto cmd{node->get_pure_pursuit_worker()->compute_command(state_tf)};

  ASSERT_NEAR(cmd.front_wheel_angle_rad, -0.294355, 0.005);
  ASSERT_NEAR(cmd.long_accel_mps2, 0.311803, 0.005);
  ASSERT_NEAR(cmd.rear_wheel_angle_rad, 0, 0.001);
  ASSERT_NEAR(cmd.velocity_mps, 14.14, 0.01);

  auto converted_cmd = node->convert_cmd(cmd);

  ASSERT_NEAR(converted_cmd.cmd.linear_acceleration, cmd.long_accel_mps2, 0.001);
  ASSERT_NEAR(converted_cmd.cmd.linear_velocity, cmd.velocity_mps, 0.01);
  ASSERT_NEAR(converted_cmd.cmd.steering_angle, cmd.front_wheel_angle_rad, 0.001);
}
} // namespace pure_pursuit_wrapper


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); 
  testing::InitGoogleTest(&argc, argv);
  int ret = 0;
  try {
    ret = RUN_ALL_TESTS();
  } catch (...) {
  }
  return ret;
}