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
  tpp.x = 10;
  tpp.y = 10;
  tpp.target_time = rclcpp::Time(0.1*1e9);  // 8.5m/s

  tpp2.x = 12;
  tpp2.y = 12;
  tpp2.target_time = rclcpp::Time(0.2*1e9);  // 48.068542495 m/s

  tpp3.x = 14;
  tpp3.y = 14;
  tpp3.target_time = rclcpp::Time(0.3*1e9);  // 8.5m/s

  carma_planning_msgs::msg::TrajectoryPlan plan;

  motion::control::controller_common::State state_tf;
  auto converted_time_now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::cerr << "Before processing: "  << std::to_string(static_cast<double>(converted_time_now)) << std::endl;

  state_tf.header.stamp = rclcpp::Time(converted_time_now*1e9);
  state_tf.state.x = 0;
  state_tf.state.y = 0;
  state_tf.state.longitudinal_velocity_mps = 1.0; //TODO arbitrary speed for first point

  plan.header.frame_id = state_tf.header.frame_id;
  plan.header.stamp = rclcpp::Time(converted_time_now*1e9) + rclcpp::Duration(1.0*1e9);

  plan.trajectory_points = { tpp, tpp2, tpp3 };

  std::cerr << "Heree" <<std::endl;
  node->process_trajectory_plan(plan);

  std::cerr << "Here" <<std::endl;
  const auto cmd{node->get_pure_pursuit_worker()->compute_command(state_tf)};
  std::cerr << "Here1" <<std::endl;
  
  auto converted_cmd = node->convert_cmd(cmd);
  std::cerr << "Here2" <<std::endl;

  ASSERT_TRUE(false);
  /*
  node->process_trajectory_plan

  ASSERT_TRUE(!!wp_msg);

  autoware_msgs::Lane lane = wp_msg.get();

  ASSERT_EQ(3, lane.waypoints.size());
  ASSERT_NEAR(8.5, lane.waypoints[0].twist.twist.linear.x, 0.0000001);
  ASSERT_NEAR(10.0, lane.waypoints[0].pose.pose.position.x, 0.0000001);
  ASSERT_NEAR(10.0, lane.waypoints[0].pose.pose.position.y, 0.0000001);

  ASSERT_NEAR(48.068542495, lane.waypoints[1].twist.twist.linear.x, 0.0000001);
  ASSERT_NEAR(12.0, lane.waypoints[1].pose.pose.position.x, 0.0000001);
  ASSERT_NEAR(12.0, lane.waypoints[1].pose.pose.position.y, 0.0000001);

  ASSERT_NEAR(8.5, lane.waypoints[2].twist.twist.linear.x, 0.0000001);
  ASSERT_NEAR(14.0, lane.waypoints[2].pose.pose.position.x, 0.0000001);
  ASSERT_NEAR(14.0, lane.waypoints[2].pose.pose.position.y, 0.0000001);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  */

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