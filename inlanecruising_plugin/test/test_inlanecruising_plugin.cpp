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

#include <inlanecruising_plugin/inlanecruising_plugin.hpp>
#include <inlanecruising_plugin/inlanecruising_plugin_node.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <math.h>
#include <tf2/LinearMath/Transform.h>

using namespace inlanecruising_plugin;
// Test to ensure Eigen::Isometry2d behaves like tf2::Transform


TEST(InLaneCruisingPluginTest, validate_eigen)
{
  Eigen::Rotation2Dd frame_rot(M_PI_2);
  lanelet::BasicPoint2d origin(1, 1);
  Eigen::Isometry2d B_in_A = carma_wm::geometry::build2dEigenTransform(origin, frame_rot);

  Eigen::Rotation2Dd new_rot(B_in_A.rotation());

  ASSERT_EQ(2, B_in_A.translation().size());
  ASSERT_NEAR(1.0, B_in_A.translation()[0], 0.000000001);
  ASSERT_NEAR(1.0, B_in_A.translation()[1], 0.000000001);
  ASSERT_NEAR(M_PI_2, new_rot.smallestAngle(), 0.000000001);

  lanelet::BasicPoint2d p_in_B(0.5, -1);
  lanelet::BasicPoint2d p_in_A = B_in_A * p_in_B;

  ASSERT_NEAR(2.0, p_in_A.x(), 0.000000001);
  ASSERT_NEAR(1.5, p_in_A.y(), 0.000000001);

  Eigen::Rotation2Dd zero_rot(0.0);
  Eigen::Isometry2d P_in_B_as_tf = carma_wm::geometry::build2dEigenTransform(p_in_B, zero_rot);
  Eigen::Isometry2d P_in_A = B_in_A * P_in_B_as_tf;
  Eigen::Rotation2Dd P_in_A_rot(P_in_A.rotation());

  ASSERT_EQ(2, P_in_A.translation().size());
  ASSERT_NEAR(2.0, P_in_A.translation()[0], 0.000000001);
  ASSERT_NEAR(1.5, P_in_A.translation()[1], 0.000000001);
  ASSERT_NEAR(M_PI_2, P_in_A_rot.smallestAngle(), 0.000000001);
}

TEST(InLaneCruisingPluginTest, test_verify_yield)
{
  InLaneCruisingPluginConfig config;
  config.enable_object_avoidance = true;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto node = std::make_shared<inlanecruising_plugin::InLaneCruisingPluginNode>(rclcpp::NodeOptions());

  InLaneCruisingPlugin plugin(node, wm, config, [&](auto msg) {});

  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory_points;

  rclcpp::Time startTime = node->now();

  carma_planning_msgs::msg::TrajectoryPlanPoint point_2;
  point_2.x = 5.0;
  point_2.y = 0.0;
  point_2.target_time = startTime + rclcpp::Duration(1, 0);
  point_2.lane_id = "1";
  trajectory_points.push_back(point_2);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_3;
  point_3.x = 10.0;
  point_3.y = 0.0;
  point_3.target_time = startTime + rclcpp::Duration(2, 0);
  point_3.lane_id = "1";
  trajectory_points.push_back(point_3);


  carma_planning_msgs::msg::TrajectoryPlan tp;
  tp.trajectory_points = trajectory_points;

  bool res = plugin.validate_yield_plan(tp);
  ASSERT_TRUE(plugin.validate_yield_plan(tp));

  carma_planning_msgs::msg::TrajectoryPlan tp2;

  carma_planning_msgs::msg::TrajectoryPlanPoint point_4;
  point_4.x = 5.0;
  point_4.y = 0.0;
  point_4.target_time = startTime + rclcpp::Duration(1, 0);
  point_4.lane_id = "1";
  tp2.trajectory_points.push_back(point_4);
  
  ASSERT_FALSE(plugin.validate_yield_plan(tp2));

  carma_planning_msgs::msg::TrajectoryPlan tp3;

  carma_planning_msgs::msg::TrajectoryPlanPoint point_5;
  point_5.x = 5.0;
  point_5.y = 0.0;
  point_5.target_time = startTime;
  point_5.lane_id = "1";
  tp3.trajectory_points.push_back(point_5);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_6;
  point_6.x = 10.0;
  point_6.y = 0.0;
  point_6.target_time = startTime + rclcpp::Duration(1, 0);
  point_6.lane_id = "1";
  tp3.trajectory_points.push_back(point_6);

  ASSERT_FALSE(plugin.validate_yield_plan(tp2));
    
}
