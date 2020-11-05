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

    std::vector<lanelet::BasicPoint2d> points = {p1, p2, p3, p4};


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
