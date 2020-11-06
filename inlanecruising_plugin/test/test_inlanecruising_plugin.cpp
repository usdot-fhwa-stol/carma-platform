/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include <inlanecruising_plugin/inlanecruising_plugin.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/CARMAWorldModel.h>
#include <math.h>

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

TEST(InLaneCruisingPluginTest, curvePointInMapTF)
{
  InLaneCruisingPluginConfig config;
  config.downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

  Eigen::Rotation2Dd frame_rot(M_PI_2);
  lanelet::BasicPoint2d origin(1, 1);
  Eigen::Isometry2d C_in_M = carma_wm::geometry::build2dEigenTransform(origin, frame_rot);

  lanelet::BasicPoint2d p_in_B(0.5, -1);
  Eigen::Isometry2d P_in_M = plugin.curvePointInMapTF(C_in_M, p_in_B, M_PI_2);

  Eigen::Rotation2Dd P_in_M_rot(P_in_M.rotation());

  ASSERT_EQ(2, P_in_M.translation().size());
  ASSERT_NEAR(2.0, P_in_M.translation()[0], 0.000000001);
  ASSERT_NEAR(1.5, P_in_M.translation()[1], 0.000000001);
  ASSERT_NEAR(M_PI, fabs(P_in_M_rot.smallestAngle()), 0.000000001);
}

TEST(InLaneCruisingPluginTest, trajectory_from_points_times_orientations)
{
  InLaneCruisingPluginConfig config;
  config.downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

  lanelet::BasicPoint2d p1(0.0, 0.0);
  lanelet::BasicPoint2d p2(2.0, 0.0);
  lanelet::BasicPoint2d p3(4.5, 0.0);
  lanelet::BasicPoint2d p4(7.0, 3.0);

  std::vector<lanelet::BasicPoint2d> points = { p1, p2, p3, p4 };

  std::vector<double> times = { 0, 2, 4, 8 };
  std::vector<double> yaws = { 0.2, 0.5, 0.6, 1.0 };
  ros::Time startTime(1.0);
  std::vector<cav_msgs::TrajectoryPlanPoint> traj_points =
      plugin.trajectory_from_points_times_orientations(points, times, yaws, startTime);

  ASSERT_EQ(4, traj_points.size());
  ASSERT_NEAR(1.0, traj_points[0].target_time.toSec(), 0.0000001);
  ASSERT_NEAR(3.0, traj_points[1].target_time.toSec(), 0.0000001);
  ASSERT_NEAR(5.0, traj_points[2].target_time.toSec(), 0.0000001);
  ASSERT_NEAR(9.0, traj_points[3].target_time.toSec(), 0.0000001);

  ASSERT_NEAR(0.0, traj_points[0].x, 0.0000001);
  ASSERT_NEAR(2.0, traj_points[1].x, 0.0000001);
  ASSERT_NEAR(4.5, traj_points[2].x, 0.0000001);
  ASSERT_NEAR(7.0, traj_points[3].x, 0.0000001);

  ASSERT_NEAR(0.0, traj_points[0].y, 0.0000001);
  ASSERT_NEAR(0.0, traj_points[1].y, 0.0000001);
  ASSERT_NEAR(0.0, traj_points[2].y, 0.0000001);
  ASSERT_NEAR(3.0, traj_points[3].y, 0.0000001);

  ASSERT_NEAR(0.2, traj_points[0].yaw, 0.0000001);
  ASSERT_NEAR(0.5, traj_points[1].yaw, 0.0000001);
  ASSERT_NEAR(0.6, traj_points[2].yaw, 0.0000001);
  ASSERT_NEAR(1.0, traj_points[3].yaw, 0.0000001);

  std::string controller_plugin = "default";
  ASSERT_EQ(0, traj_points[0].controller_plugin_name.compare(controller_plugin));
  ASSERT_EQ(0, traj_points[1].controller_plugin_name.compare(controller_plugin));
  ASSERT_EQ(0, traj_points[2].controller_plugin_name.compare(controller_plugin));
  ASSERT_EQ(0, traj_points[3].controller_plugin_name.compare(controller_plugin));

  std::string expected_plugin_name = "InLaneCruisingPlugin";
  ASSERT_EQ(0, traj_points[0].planner_plugin_name.compare(expected_plugin_name));
  ASSERT_EQ(0, traj_points[1].planner_plugin_name.compare(expected_plugin_name));
  ASSERT_EQ(0, traj_points[2].planner_plugin_name.compare(expected_plugin_name));
  ASSERT_EQ(0, traj_points[3].planner_plugin_name.compare(expected_plugin_name));
}

TEST(InLaneCruisingPluginTest, compose_trajectory_from_centerline)
{
  cav_msgs::VehicleState state;
  state.X_pos_global = -191.098;
  state.Y_pos_global = 475.945;
  state.orientation = -2.65651;
  state.longitudinal_vel = 0.0;

  std::vector<PointSpeedPair> points;

  PointSpeedPair p;
  p.point = lanelet::BasicPoint2d(-159.597, 522.468);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-159.855, 520.124);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-160.128, 518.048);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-160.433, 515.979);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-160.776, 513.918);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-161.16, 511.873);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-161.544, 509.892);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-161.926, 507.912);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-162.304, 505.956);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-162.725, 504.058);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-163.225, 502.179);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-163.802, 500.323);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-164.457, 498.491);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-165.187, 496.688);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-165.992, 494.917);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-166.871, 493.179);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-167.827, 491.404);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-168.773, 489.651);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-169.735, 488.065);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-170.849, 486.579);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-172.104, 485.21);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-173.53, 483.916);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-174.992, 482.746);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-176.554, 481.712);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-178.237, 480.805);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-179.97, 479.997);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-181.749, 479.317);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-183.704, 478.671);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-185.672, 477.911);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-187.55, 477.073);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-189.386, 476.233);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-191.223, 475.393);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-193.069, 474.539);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-194.906, 473.67);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-196.736, 472.787);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-198.55, 471.9);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-200.364, 471.012);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-202.178, 470.125);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-203.992, 469.237);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-205.805, 468.349);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-207.619, 467.462);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-209.433, 466.574);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-211.247, 465.686);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-213.086, 464.757);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-214.897, 463.79);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-216.678, 462.79);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-218.435, 461.752);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-220.169, 460.676);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-221.881, 459.563);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-223.592, 458.484);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-225.327, 457.587);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-227.165, 456.806);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-229.238, 455.682);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-231.09, 454.213);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-232.68, 452.626);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-234.16, 451.058);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-235.524, 449.544);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-236.861, 448.008);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-238.171, 446.451);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-239.455, 444.871);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-240.711, 443.272);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-241.937, 441.682);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-243.162, 440.092);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-244.404, 438.483);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-245.683, 436.82);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-246.94, 435.12);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-248.16, 433.394);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-249.341, 431.64);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-250.452, 429.355);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-250.341, 426.332);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-248.912, 423.938);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-247.105, 422.081);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-244.199, 421.018);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-240.226, 421.725);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-237.814, 422.299);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-235.781, 422.702);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-233.719, 423.039);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-231.614, 423.315);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-229.604, 423.523);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-227.622, 423.675);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-225.637, 423.78);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-223.655, 423.841);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-221.696, 423.829);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-219.738, 423.737);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-217.772, 423.555);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-215.816, 423.281);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-213.874, 422.925);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-211.947, 422.5);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-210.04, 422.008);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-208.133, 421.45);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-206.237, 420.8);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-204.374, 420.063);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-202.502, 419.244);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-200.714, 418.395);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-198.982, 417.449);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-197.143, 416.305);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-195.366, 415.054);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-193.763, 413.857);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-192.267, 412.618);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-190.843, 411.296);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-189.435, 409.889);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-188.079, 408.469);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-186.768, 406.997);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-185.511, 405.485);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-184.254, 403.866);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-183.029, 402.204);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-181.842, 400.553);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-180.841, 398.889);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-179.91, 397.043);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-178.861, 395.173);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-177.713, 393.414);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-176.499, 391.765);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-175.3, 390.158);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-174.101, 388.552);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-172.901, 386.946);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-171.702, 385.34);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-170.484, 383.718);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-169.201, 382.123);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-167.93, 380.572);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-166.628, 379.013);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-165.297, 377.479);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-163.939, 375.968);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-162.562, 374.489);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-161.182, 373.022);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-159.781, 371.563);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-158.362, 370.122);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-156.925, 368.699);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-155.467, 367.291);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-153.977, 365.905);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-152.458, 364.55);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-150.911, 363.227);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-149.338, 361.937);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-147.766, 360.678);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-146.157, 359.43);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-144.522, 358.217);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-142.861, 357.039);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-141.176, 355.896);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-139.476, 354.792);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-137.757, 353.711);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-136.019, 352.661);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-134.264, 351.641);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-132.49, 350.652);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-130.7, 349.694);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-128.895, 348.767);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-127.074, 347.87);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-125.238, 347.004);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-123.398, 346.17);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-121.564, 345.347);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-119.71, 344.537);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-117.845, 343.75);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-115.97, 342.987);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-114.087, 342.249);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-112.217, 341.528);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-110.35, 340.806);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-108.486, 340.079);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-106.619, 339.349);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-104.752, 338.619);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-102.885, 337.889);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-101.017, 337.16);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-99.1503, 336.43);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-97.2831, 335.7);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-95.411, 334.947);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-93.7624, 334.157);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-92.3424, 333.134);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-91.151, 331.908);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-90.4156, 330.613);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-90.0448, 329.058);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-90.1217, 327.612);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-90.4319, 326.032);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-91.0001, 324.545);
  p.speed = 6.7056;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(-91.8756, 322.892);
  p.speed = 6.7056;
  points.push_back(p);

  InLaneCruisingPluginConfig config;
  config.downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

  auto plan = plugin.compose_trajectory_from_centerline(points, state);
  // std::vector<cav_msgs::TrajectoryPlanPoint> InLaneCruisingPlugin::compose_trajectory_from_centerline(
  //   const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state)
}

// TEST(InLaneCruisingPluginTest, setWaypointsTest)
// {
//     // compose a list of waypoints spanning 8 seconds
//     std::vector<autoware_msgs::Waypoint> waypoints;
//     autoware_msgs::Waypoint wp_1;
//     wp_1.twist.twist.linear.x = 2.0;
//     wp_1.pose.pose.position.x = 0.0;
//     autoware_msgs::Waypoint wp_2;
//     wp_2.twist.twist.linear.x = 4.0;
//     wp_2.pose.pose.position.x = 6.0;
//     autoware_msgs::Waypoint wp_3;
//     wp_3.twist.twist.linear.x = 8.0;
//     wp_3.pose.pose.position.x = 24.0;
//     autoware_msgs::Waypoint wp_4;
//     wp_4.twist.twist.linear.x = 8.0;
//     wp_4.pose.pose.position.x = 40.0;
//     autoware_msgs::Waypoint wp_5;
//     wp_5.twist.twist.linear.x = 8.0;
//     wp_5.pose.pose.position.x = 48.0;
//     waypoints.push_back(wp_1);
//     waypoints.push_back(wp_2);
//     waypoints.push_back(wp_3);
//     waypoints.push_back(wp_4);
//     waypoints.push_back(wp_5);
//     inlanecruising_plugin::InLaneCruisingPlugin ip;
//     inlanecruising_plugin::Point2DRTree rTree = ip.set_waypoints(waypoints);
//     ip.pose_msg_.reset(new geometry_msgs::PoseStamped());

//     inlanecruising_plugin::Boost2DPoint vehicle_point(0.0, 0.0);
//     std::vector<inlanecruising_plugin::PointIndexPair> nearest_points;
//     rTree.query(boost::geometry::index::nearest(vehicle_point, 1), std::back_inserter(nearest_points));
//     ASSERT_EQ(0, std::get<1>(nearest_points[0]));
//     vehicle_point = inlanecruising_plugin::Boost2DPoint(2.0, 0.0);
//     nearest_points = {};
//     rTree.query(boost::geometry::index::nearest(vehicle_point, 1), std::back_inserter(nearest_points));
//     ASSERT_EQ(0, std::get<1>(nearest_points[0]));
//     vehicle_point = inlanecruising_plugin::Boost2DPoint(24.0, 0.0);
//     nearest_points = {};
//     rTree.query(boost::geometry::index::nearest(vehicle_point, 1), std::back_inserter(nearest_points));
//     ASSERT_EQ(2, std::get<1>(nearest_points[0]));
//     vehicle_point = inlanecruising_plugin::Boost2DPoint(50.0, 0.0);
//     nearest_points = {};
//     rTree.query(boost::geometry::index::nearest(vehicle_point, 1), std::back_inserter(nearest_points));
//     ASSERT_EQ(4, std::get<1>(nearest_points[0]));
// }

// TEST(InLaneCruisingPluginTest, testGetWaypointsInTimeBoundary1)
// {
//     // compose a list of waypoints spanning 8 seconds
//     std::vector<autoware_msgs::Waypoint> waypoints;
//     autoware_msgs::Waypoint wp_1;
//     wp_1.twist.twist.linear.x = 2.0;
//     wp_1.pose.pose.position.x = 0.0;
//     autoware_msgs::Waypoint wp_2;
//     wp_2.twist.twist.linear.x = 4.0;
//     wp_2.pose.pose.position.x = 6.0;
//     autoware_msgs::Waypoint wp_3;
//     wp_3.twist.twist.linear.x = 8.0;
//     wp_3.pose.pose.position.x = 24.0;
//     autoware_msgs::Waypoint wp_4;
//     wp_4.twist.twist.linear.x = 8.0;
//     wp_4.pose.pose.position.x = 40.0;
//     autoware_msgs::Waypoint wp_5;
//     wp_5.twist.twist.linear.x = 8.0;
//     wp_5.pose.pose.position.x = 48.0;
//     waypoints.push_back(wp_1);
//     waypoints.push_back(wp_2);
//     waypoints.push_back(wp_3);
//     waypoints.push_back(wp_4);
//     waypoints.push_back(wp_5);
//     inlanecruising_plugin::InLaneCruisingPlugin ip;
//     ip.set_waypoints(waypoints);
//     ip.pose_msg_.reset(new geometry_msgs::PoseStamped());
//     std::vector<autoware_msgs::Waypoint> res = ip.get_waypoints_in_time_boundary(waypoints, 6.0);
//     EXPECT_EQ(3, res.size());
//     EXPECT_NEAR(4.0, res[0].twist.twist.linear.x, 0.01);
//     EXPECT_NEAR(6.0, res[0].pose.pose.position.x, 0.01);
//     EXPECT_NEAR(8.0, res[1].twist.twist.linear.x, 0.01);
//     EXPECT_NEAR(24.0, res[1].pose.pose.position.x, 0.01);
//     EXPECT_NEAR(8.0, res.back().twist.twist.linear.x, 0.01);
//     EXPECT_NEAR(40.0, res.back().pose.pose.position.x, 0.01);
//     // test if plugin returns sublist if the point is on the last waypoint
//     geometry_msgs::PoseStamped pose;
//     pose.pose.position.x = 48.0;
//     ip.pose_msg_.reset(new geometry_msgs::PoseStamped(pose));
//     res = ip.get_waypoints_in_time_boundary(waypoints, 6.0);
//     EXPECT_EQ(0, res.size());
// }

// TEST(InLaneCruisingPluginTest, testGetWaypointsInTimeBoundary2)
// {
//     // compose a list of waypoints spaning less than 6 seconds
//     std::vector<autoware_msgs::Waypoint> waypoints;
//     autoware_msgs::Waypoint wp_1;
//     wp_1.twist.twist.linear.x = 2.0;
//     wp_1.pose.pose.position.x = 0.0;
//     autoware_msgs::Waypoint wp_2;
//     wp_2.twist.twist.linear.x = 4.0;
//     wp_2.pose.pose.position.x = 6.0;
//     waypoints.push_back(wp_1);
//     waypoints.push_back(wp_2);
//     inlanecruising_plugin::InLaneCruisingPlugin ip;
//     ip.set_waypoints(waypoints);
//     ip.pose_msg_.reset(new geometry_msgs::PoseStamped());
//     std::vector<autoware_msgs::Waypoint> res = ip.get_waypoints_in_time_boundary(waypoints, 6.0);
//     EXPECT_EQ(1, res.size());
//     EXPECT_NEAR(4.0, res[0].twist.twist.linear.x, 0.01);
//     EXPECT_NEAR(6.0, res[0].pose.pose.position.x, 0.01);
// }

// TEST(InLaneCruisingPluginTest, testGetWaypointsInTimeBoundary3)
// {
//     // compose a list of waypoints spanning exactly 5 seconds
//     std::vector<autoware_msgs::Waypoint> waypoints;
//     autoware_msgs::Waypoint wp_1;
//     wp_1.twist.twist.linear.x = 2.0;
//     wp_1.pose.pose.position.x = 0.0;
//     autoware_msgs::Waypoint wp_2;
//     wp_2.twist.twist.linear.x = 4.0;
//     wp_2.pose.pose.position.x = 6.0;
//     autoware_msgs::Waypoint wp_3;
//     wp_3.twist.twist.linear.x = 8.0;
//     wp_3.pose.pose.position.x = 24.0;
//     waypoints.push_back(wp_1);
//     waypoints.push_back(wp_2);
//     waypoints.push_back(wp_3);
//     inlanecruising_plugin::InLaneCruisingPlugin ip;
//     ip.set_waypoints(waypoints);
//     ip.pose_msg_.reset(new geometry_msgs::PoseStamped());
//     std::vector<autoware_msgs::Waypoint> res = ip.get_waypoints_in_time_boundary(waypoints, 5.0);
//     EXPECT_EQ(2, res.size());
//     EXPECT_NEAR(4.0, res[0].twist.twist.linear.x, 0.01);
//     EXPECT_NEAR(6.0, res[0].pose.pose.position.x, 0.01);
//     EXPECT_NEAR(8.0, res[1].twist.twist.linear.x, 0.01);
//     EXPECT_NEAR(24.0, res[1].pose.pose.position.x, 0.01);
// }

// TEST(InLaneCruisingPluginTest, testCreateUnevenTrajectory1)
// {
//     // compose a list of waypoints, uneven spaced
//     std::vector<autoware_msgs::Waypoint> waypoints;
//     autoware_msgs::Waypoint wp_1;
//     wp_1.twist.twist.linear.x = 2.0;
//     wp_1.pose.pose.position.x = 0.0;
//     autoware_msgs::Waypoint wp_2;
//     wp_2.twist.twist.linear.x = 4.0;
//     wp_2.pose.pose.position.x = 0.5;
//     autoware_msgs::Waypoint wp_3;
//     wp_3.twist.twist.linear.x = 2.0;
//     wp_3.pose.pose.position.x = 1.3;
//     autoware_msgs::Waypoint wp_4;
//     wp_4.twist.twist.linear.x = 4.0;
//     wp_4.pose.pose.position.x = 1.4;
//     autoware_msgs::Waypoint wp_5;
//     wp_5.twist.twist.linear.x = 4.0;
//     wp_5.pose.pose.position.x = 2.0;
//     waypoints.push_back(wp_1);
//     waypoints.push_back(wp_2);
//     waypoints.push_back(wp_3);
//     waypoints.push_back(wp_4);
//     waypoints.push_back(wp_5);
//     inlanecruising_plugin::InLaneCruisingPlugin ip;
//     ip.set_waypoints(waypoints);
//     // create pose message to indicate that the current location is on top of the starting waypoint
//     ip.pose_msg_.reset(new geometry_msgs::PoseStamped());
//     std::vector<cav_msgs::TrajectoryPlanPoint> traj = ip.create_uneven_trajectory_from_waypoints(waypoints);
//     EXPECT_EQ(5, traj.size());
//     EXPECT_NEAR(0.0, traj[0].target_time, 0.01);
//     EXPECT_NEAR(0.0, traj[0].x, 0.01);
//     EXPECT_NEAR(0.25, traj[1].target_time / 1e9, 0.01);
//     EXPECT_NEAR(0.5, traj[1].x, 0.01);
//     EXPECT_NEAR(0.45, traj[2].target_time / 1e9, 0.01);
//     EXPECT_NEAR(1.3, traj[2].x, 0.01);
//     EXPECT_NEAR(0.5, traj[3].target_time / 1e9, 0.01);
//     EXPECT_NEAR(1.4, traj[3].x, 0.01);
//     EXPECT_NEAR(0.65, traj[4].target_time / 1e9, 0.01);
//     EXPECT_NEAR(2.0, traj[4].x, 0.01);
// }

// TEST(InLaneCruisingPluginTest, testCreateUnevenTrajectory2)
// {
//     // compose a list of waypoints, uneven spaced
//     std::vector<autoware_msgs::Waypoint> waypoints;
//     autoware_msgs::Waypoint wp_1;
//     wp_1.twist.twist.linear.x = 2.0;
//     wp_1.pose.pose.position.x = 0.0;
//     autoware_msgs::Waypoint wp_2;
//     wp_2.twist.twist.linear.x = 4.0;
//     wp_2.pose.pose.position.x = 0.5;
//     autoware_msgs::Waypoint wp_3;
//     wp_3.twist.twist.linear.x = 2.0;
//     wp_3.pose.pose.position.x = 1.3;
//     autoware_msgs::Waypoint wp_4;
//     wp_4.twist.twist.linear.x = 4.0;
//     wp_4.pose.pose.position.x = 1.4;
//     autoware_msgs::Waypoint wp_5;
//     wp_5.twist.twist.linear.x = 4.0;
//     wp_5.pose.pose.position.x = 2.0;
//     waypoints.push_back(wp_1);
//     waypoints.push_back(wp_2);
//     waypoints.push_back(wp_3);
//     waypoints.push_back(wp_4);
//     waypoints.push_back(wp_5);
//     inlanecruising_plugin::InLaneCruisingPlugin ip;
//     ip.set_waypoints(waypoints);
//     // create pose message to indicate that the current location is not near the starting waypoint
//     geometry_msgs::PoseStamped pose;
//     pose.pose.position.x = -1.0;
//     ip.pose_msg_.reset(new geometry_msgs::PoseStamped(pose));
//     std::vector<cav_msgs::TrajectoryPlanPoint> traj = ip.create_uneven_trajectory_from_waypoints(waypoints);
//     EXPECT_EQ(6, traj.size());
//     EXPECT_NEAR(0.0, traj[0].target_time / 1e9, 0.01);
//     EXPECT_NEAR(-1.0, traj[0].x, 0.01);
//     EXPECT_NEAR(0.5, traj[1].target_time / 1e9, 0.01);
//     EXPECT_NEAR(0.0, traj[1].x, 0.01);
//     EXPECT_NEAR(0.75, traj[2].target_time / 1e9, 0.001);
//     EXPECT_NEAR(0.5, traj[2].x, 0.01);
//     EXPECT_NEAR(0.95, traj[3].target_time / 1e9, 0.001);
//     EXPECT_NEAR(1.3, traj[3].x, 0.01);
//     EXPECT_NEAR(1.0, traj[4].target_time / 1e9, 0.001);
//     EXPECT_NEAR(1.4, traj[4].x, 0.01);
//     EXPECT_NEAR(1.15, traj[5].target_time / 1e9, 0.001);
//     EXPECT_NEAR(2.0, traj[5].x, 0.01);
// }
