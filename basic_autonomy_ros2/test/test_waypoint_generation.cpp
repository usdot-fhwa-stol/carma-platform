/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include <basic_autonomy_ros2/basic_autonomy.hpp>
#include <basic_autonomy_ros2/helper_functions.hpp>
#include <gtest/gtest.h>
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <math.h>
#include <tf2/LinearMath/Transform.h>
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <string>
#include <sstream>
#include <carma_planning_msgs/msg/maneuver.hpp>
#include <carma_planning_msgs/msg/vehicle_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace basic_autonomy
{

    // Test to ensure Eigen::Isometry2d behaves like tf2::Transform
    TEST(BasicAutonomyTest, validate_eigen)
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

    TEST(BasicAutonomyTest, test_name)
    {

        lanelet::BasicPoint2d p1(0.0, 0.0);
        lanelet::BasicPoint2d p2(2.0, 0.0);
        lanelet::BasicPoint2d p3(4.5, 0.0);
        lanelet::BasicPoint2d p4(7.0, 3.0);

        std::vector<lanelet::BasicPoint2d> points = {p1, p2, p3, p4};

        std::vector<double> times = {0, 2, 4, 8};
        std::vector<double> yaws = {0.2, 0.5, 0.6, 1.0};
        rclcpp::Time startTime(1.0 * 1e9); // Arbitrarily selected 1 second, converted to nanoseconds
        std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> traj_points =
            basic_autonomy::waypoint_generation::trajectory_from_points_times_orientations(points, times, yaws, startTime, "default");

        ASSERT_EQ(4, traj_points.size());
        ASSERT_NEAR(1.0, rclcpp::Time(traj_points[0].target_time).seconds(), 0.0000001);
        ASSERT_NEAR(3.0, rclcpp::Time(traj_points[1].target_time).seconds(), 0.0000001);
        ASSERT_NEAR(5.0, rclcpp::Time(traj_points[2].target_time).seconds(), 0.0000001);
        ASSERT_NEAR(9.0, rclcpp::Time(traj_points[3].target_time).seconds(), 0.0000001);

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

    }

     TEST(BasicAutonomyTest, constrain_to_time_boundary)
    {

        std::vector<waypoint_generation::PointSpeedPair> points;

        waypoint_generation::PointSpeedPair p;
        p.point = lanelet::BasicPoint2d(0, 0);
        p.speed = 1.0;
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(1, 0);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(2, 0);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(3, 0);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(4, 0);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(5, 0);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(6, 0);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(7, 0);
        points.push_back(p);

        std::vector<waypoint_generation::PointSpeedPair> time_bound_points = waypoint_generation::constrain_to_time_boundary(points, 6.0);

        ASSERT_EQ(6, time_bound_points.size());
        ASSERT_NEAR(0.0, time_bound_points[0].point.x(), 0.0000001);
        ASSERT_NEAR(1.0, time_bound_points[1].point.x(), 0.0000001);
        ASSERT_NEAR(2.0, time_bound_points[2].point.x(), 0.0000001);
        ASSERT_NEAR(3.0, time_bound_points[3].point.x(), 0.0000001);
        ASSERT_NEAR(4.0, time_bound_points[4].point.x(), 0.0000001);
        ASSERT_NEAR(5.0, time_bound_points[5].point.x(), 0.0000001);

        ASSERT_NEAR(0.0, time_bound_points[0].point.y(), 0.0000001);
        ASSERT_NEAR(0.0, time_bound_points[1].point.y(), 0.0000001);
        ASSERT_NEAR(0.0, time_bound_points[2].point.y(), 0.0000001);
        ASSERT_NEAR(0.0, time_bound_points[3].point.y(), 0.0000001);
        ASSERT_NEAR(0.0, time_bound_points[4].point.y(), 0.0000001);
        ASSERT_NEAR(0.0, time_bound_points[5].point.y(), 0.0000001);

        ASSERT_NEAR(1.0, time_bound_points[0].speed, 0.0000001);
        ASSERT_NEAR(1.0, time_bound_points[1].speed, 0.0000001);
        ASSERT_NEAR(1.0, time_bound_points[2].speed, 0.0000001);
        ASSERT_NEAR(1.0, time_bound_points[3].speed, 0.0000001);
        ASSERT_NEAR(1.0, time_bound_points[4].speed, 0.0000001);
        ASSERT_NEAR(1.0, time_bound_points[5].speed, 0.0000001);
    }

    TEST(BasicAutonomyTest, get_nearest_point_index)
    {
        std::vector<waypoint_generation::PointSpeedPair> points;
        std::vector<lanelet::BasicPoint2d> basic_points;
        waypoint_generation::PointSpeedPair p;
        p.point = lanelet::BasicPoint2d(0, 0);
        p.speed = 1.0;
        points.push_back(p);
        basic_points.push_back(p.point);
        p.point = lanelet::BasicPoint2d(1, 1);
        points.push_back(p);
        basic_points.push_back(p.point);
        p.point = lanelet::BasicPoint2d(2, 2);
        points.push_back(p);
        basic_points.push_back(p.point);
        p.point = lanelet::BasicPoint2d(3, 3);
        points.push_back(p);
        basic_points.push_back(p.point);
        p.point = lanelet::BasicPoint2d(4, 4);
        points.push_back(p);
        basic_points.push_back(p.point);
        p.point = lanelet::BasicPoint2d(5, 5);
        points.push_back(p);
        basic_points.push_back(p.point);
        p.point = lanelet::BasicPoint2d(6, 6);
        points.push_back(p);
        basic_points.push_back(p.point);
        p.point = lanelet::BasicPoint2d(7, 7);
        points.push_back(p);
        basic_points.push_back(p.point);

        carma_planning_msgs::msg::VehicleState state;
        state.x_pos_global = 3.3;
        state.y_pos_global = 3.3;

        ASSERT_EQ(3, waypoint_generation::get_nearest_point_index(basic_points, state));
        ASSERT_EQ(3, waypoint_generation::get_nearest_point_index(points, state));
    }

TEST(BasicAutonomyTest, get_nearest_basic_point_index)
    {
        std::vector<waypoint_generation::PointSpeedPair> points;

        waypoint_generation::PointSpeedPair p;
        p.point = lanelet::BasicPoint2d(0, 0);
        p.speed = 1.0;
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(1, 1);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(2, 2);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(3, 3);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(4, 4);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(5, 5);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(6, 6);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(7, 7);
        points.push_back(p);

        carma_planning_msgs::msg::VehicleState state;
        state.x_pos_global = 3.3;
        state.y_pos_global = 3.3;

        ASSERT_EQ(3, waypoint_generation::get_nearest_point_index(points, state));
    }

    TEST(BasicAutonomyTest, split_point_speed_pairs)
    {
        std::vector<waypoint_generation::PointSpeedPair> points;

        waypoint_generation::PointSpeedPair p;
        p.point = lanelet::BasicPoint2d(0, 1);
        p.speed = 1.0;
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(1, 2);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(2, 3);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(3, 4);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(4, 5);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(5, 6);
        points.push_back(p);

        std::vector<lanelet::BasicPoint2d> basic_points;
        std::vector<double> speeds;

        waypoint_generation::split_point_speed_pairs(points, &basic_points, &speeds);
        ASSERT_EQ(points.size(), basic_points.size());
        ASSERT_NEAR(0.0, basic_points[0].x(), 0.0000001);
        ASSERT_NEAR(1.0, basic_points[1].x(), 0.0000001);
        ASSERT_NEAR(2.0, basic_points[2].x(), 0.0000001);
        ASSERT_NEAR(3.0, basic_points[3].x(), 0.0000001);
        ASSERT_NEAR(4.0, basic_points[4].x(), 0.0000001);
        ASSERT_NEAR(5.0, basic_points[5].x(), 0.0000001);

        ASSERT_NEAR(1.0, basic_points[0].y(), 0.0000001);
        ASSERT_NEAR(2.0, basic_points[1].y(), 0.0000001);
        ASSERT_NEAR(3.0, basic_points[2].y(), 0.0000001);
        ASSERT_NEAR(4.0, basic_points[3].y(), 0.0000001);
        ASSERT_NEAR(5.0, basic_points[4].y(), 0.0000001);
        ASSERT_NEAR(6.0, basic_points[5].y(), 0.0000001);

        ASSERT_NEAR(1.0, speeds[0], 0.0000001);
        ASSERT_NEAR(1.0, speeds[1], 0.0000001);
        ASSERT_NEAR(1.0, speeds[2], 0.0000001);
        ASSERT_NEAR(1.0, speeds[3], 0.0000001);
        ASSERT_NEAR(1.0, speeds[4], 0.0000001);
        ASSERT_NEAR(1.0, speeds[5], 0.0000001);
    }

    TEST(BasicAutonomyTest, compute_fit)
    {
        ///////////////////////
        // Check straight line
        ///////////////////////
        std::vector<lanelet::BasicPoint2d> points;
        auto p = lanelet::BasicPoint2d(20, 30);
        points.push_back(p);
        p = lanelet::BasicPoint2d(21, 30);
        points.push_back(p);
        p = lanelet::BasicPoint2d(22, 30);
        points.push_back(p);
        p = lanelet::BasicPoint2d(23, 30);
        points.push_back(p);

        std::unique_ptr<smoothing::SplineI> fit_curve = waypoint_generation::compute_fit(points);
        std::vector<lanelet::BasicPoint2d> spline_points;
        // Following logic is written for BSpline library. Switch with appropriate call of the new library if different.
        double parameter = 0.0;

        for (size_t i = 0; i < points.size(); i++)
        {
            auto values = (*fit_curve)(parameter);

            // Uncomment to print and check if this generated map matches with the original one above
            // ROS_INFO_STREAM("BSpline point: x: " << values.x() << "y: " << values.y());
            spline_points.push_back({values.x(), values.y()});
            parameter += 1.0 / (points.size() * 1.0);
        }

        ASSERT_EQ(spline_points.size(), points.size());

        tf2::Vector3 original_vector_1(points[1].x() - points[0].x(),
                                      points[1].y() - points[0].y(), 0);
        original_vector_1.setZ(0);
        tf2::Vector3 spline_vector_1(spline_points[1].x() - spline_points[0].x(),
                                    spline_points[1].y() - spline_points[0].y(), 0);
        spline_vector_1.setZ(0);
        tf2::Vector3 original_vector_2(points[2].x() - points[1].x(),
                                      points[2].y() - points[1].y(), 0);
        original_vector_2.setZ(0);
        tf2::Vector3 spline_vector_2(spline_points[2].x() - spline_points[1].x(),
                                    spline_points[2].y() - spline_points[1].y(), 0);
        spline_vector_2.setZ(0);
        double angle_in_rad_1 = std::fabs(tf2::tf2Angle(original_vector_1, spline_vector_1));
        double angle_in_rad_2 = std::fabs(tf2::tf2Angle(original_vector_2, spline_vector_2));

        ASSERT_NEAR(angle_in_rad_1, 0.0, 0.0001);
        ASSERT_NEAR(angle_in_rad_2, 0.0, 0.0001);

        ///////////////////////
        // S curve
        ///////////////////////
        points = {};
        lanelet::BasicPoint2d po1(3, 4);
        points.push_back(po1);
        lanelet::BasicPoint2d po2(5, 4);
        points.push_back(po2);
        lanelet::BasicPoint2d po3(8, 9);
        points.push_back(po3);
        lanelet::BasicPoint2d po4(8, 23);
        points.push_back(po4);
        lanelet::BasicPoint2d po5(3.5, 25);
        points.push_back(po5);
        lanelet::BasicPoint2d po6(3, 25);
        points.push_back(po6);
        lanelet::BasicPoint2d po7(2.5, 26);
        points.push_back(po7);
        lanelet::BasicPoint2d po8(2.25, 27);
        points.push_back(po8);
        lanelet::BasicPoint2d po9(2.0, 28);
        points.push_back(po9);
        lanelet::BasicPoint2d po10(1.5, 30);
        points.push_back(po10);
        lanelet::BasicPoint2d po11(1.0, 32);
        points.push_back(po11);
        lanelet::BasicPoint2d po12(1.25, 34);
        points.push_back(po12);
        lanelet::BasicPoint2d po13(2.0, 35);
        points.push_back(po13);
        lanelet::BasicPoint2d po14(4.0, 35);
        points.push_back(po14);
        lanelet::BasicPoint2d po15(5.0, 35.5);
        points.push_back(po15);
        lanelet::BasicPoint2d po16(6.0, 36);
        points.push_back(po16);
        lanelet::BasicPoint2d po17(7.0, 50);
        points.push_back(po17);
        lanelet::BasicPoint2d po18(6.5, 48);
        points.push_back(po18);
        lanelet::BasicPoint2d po19(4.0, 43);
        points.push_back(po19);

        // As different libraries may fit S curves differently, we are only checking if we can get any fit here.
        ASSERT_NO_THROW(waypoint_generation::compute_fit(points));

        std::unique_ptr<smoothing::SplineI> fit_s_curve = waypoint_generation::compute_fit(points);

        ASSERT_TRUE(!!fit_s_curve);
    }

    TEST(BasicAutonomyTest, optimize_speed)
    {
        std::vector<double> downtracks, curv_speeds;
        downtracks.push_back(0);
        downtracks.push_back(2);
        downtracks.push_back(4);
        downtracks.push_back(6);
        downtracks.push_back(8);
        downtracks.push_back(10);
        downtracks.push_back(12);
        downtracks.push_back(14);
        downtracks.push_back(16);

        double max_accel = 2.0;

        ASSERT_THROW(waypoint_generation::optimize_speed(downtracks, curv_speeds, max_accel), std::invalid_argument);

        curv_speeds.push_back(1);
        curv_speeds.push_back(3);
        curv_speeds.push_back(4);
        curv_speeds.push_back(4);
        curv_speeds.push_back(1);
        curv_speeds.push_back(0);
        curv_speeds.push_back(3);
        curv_speeds.push_back(3);
        curv_speeds.push_back(6);

        ASSERT_THROW(waypoint_generation::optimize_speed(downtracks, curv_speeds, -10), std::invalid_argument);

        std::vector<double> expected_results;
        expected_results.push_back(1);
        expected_results.push_back(3);
        expected_results.push_back(4);
        expected_results.push_back(3);
        expected_results.push_back(1);
        expected_results.push_back(0);
        expected_results.push_back(2.82843);
        expected_results.push_back(3);
        expected_results.push_back(4.12311);
        auto test_results = waypoint_generation::optimize_speed(downtracks, curv_speeds, max_accel);

        ASSERT_NEAR(expected_results[0], test_results[0], 0.001);
        ASSERT_NEAR(expected_results[1], test_results[1], 0.001);
        ASSERT_NEAR(expected_results[2], test_results[2], 0.001);
        ASSERT_NEAR(expected_results[3], test_results[3], 0.001);
        ASSERT_NEAR(expected_results[4], test_results[4], 0.001);
        ASSERT_NEAR(expected_results[5], test_results[5], 0.001);
        ASSERT_NEAR(expected_results[6], test_results[6], 0.001);
        ASSERT_NEAR(expected_results[7], test_results[7], 0.001);
        ASSERT_NEAR(expected_results[8], test_results[8], 0.001);

        // Check if the first speed is same
        curv_speeds = {};
        curv_speeds.push_back(4);
        curv_speeds.push_back(1);
        curv_speeds.push_back(3);
        curv_speeds.push_back(4);
        curv_speeds.push_back(1);
        curv_speeds.push_back(0);
        curv_speeds.push_back(3);
        curv_speeds.push_back(3);
        curv_speeds.push_back(6);

        expected_results = {};
        expected_results.push_back(4);
        expected_results.push_back(2.82847);
        expected_results.push_back(3);
        expected_results.push_back(3);
        expected_results.push_back(1);
        expected_results.push_back(0);
        expected_results.push_back(2.82843);
        expected_results.push_back(3);
        expected_results.push_back(4.12311);

        test_results = waypoint_generation::optimize_speed(downtracks, curv_speeds, max_accel);

        ASSERT_NEAR(expected_results[0], test_results[0], 0.001);
        ASSERT_NEAR(expected_results[1], test_results[1], 0.001);
        ASSERT_NEAR(expected_results[2], test_results[2], 0.001);
        ASSERT_NEAR(expected_results[3], test_results[3], 0.001);
        ASSERT_NEAR(expected_results[4], test_results[4], 0.001);
        ASSERT_NEAR(expected_results[5], test_results[5], 0.001);
        ASSERT_NEAR(expected_results[6], test_results[6], 0.001);
        ASSERT_NEAR(expected_results[7], test_results[7], 0.001);
        ASSERT_NEAR(expected_results[8], test_results[8], 0.001);
    }

    TEST(BasicAutonomyTest, compute_curvature_at)
    {
        ///////////////////////
        // Check straight line
        ///////////////////////
        std::vector<lanelet::BasicPoint2d> points;
        auto p = lanelet::BasicPoint2d(20, 30);
        points.push_back(p);
        p = lanelet::BasicPoint2d(21, 30);
        points.push_back(p);
        p = lanelet::BasicPoint2d(22, 30);
        points.push_back(p);
        p = lanelet::BasicPoint2d(23, 30);
        points.push_back(p);
        std::unique_ptr<smoothing::SplineI> fit_curve = waypoint_generation::compute_fit(points);

        ASSERT_NEAR(waypoint_generation::compute_curvature_at((*fit_curve), 0.0), 0, 0.001);  // check start
        ASSERT_NEAR(waypoint_generation::compute_curvature_at((*fit_curve), 1.0), 0, 0.001);  // check end
        ASSERT_NEAR(waypoint_generation::compute_curvature_at((*fit_curve), 0.23), 0, 0.001); // check random 1
        ASSERT_NEAR(waypoint_generation::compute_curvature_at((*fit_curve), 0.97), 0, 0.001); // check random 2

        ///////////////////////
        // Circle (0,0 centered, R radius)
        ///////////////////////
        points = {};
        std::vector<double> x, y;
        double x_ = 0.0;
        double radius = 20;
        for (int i = 0; i < 10; i++)
        {
            x.push_back(x_);
            y.push_back(-sqrt(pow(radius, 2) - pow(x_, 2))); //y-
            x_ += radius / (double)10;
        }
        for (int i = 0; i < 10; i++)
        {
            x.push_back(x_);
            y.push_back(sqrt(pow(radius, 2) - pow(x_, 2))); //y+
            x_ -= radius / (double)10;
        }
        for (int i = 0; i < 10; i++)
        {
            x.push_back(x_);
            y.push_back(sqrt(pow(radius, 2) - pow(x_, 2))); //y+
            x_ -= radius / (double)10;
        }
        for (int i = 0; i < 10; i++)
        {
            x.push_back(x_);
            y.push_back(-sqrt(pow(radius, 2) - pow(x_, 2))); //y-
            x_ += radius / (double)10;
        }
        y.push_back(-sqrt(pow(radius, 2) - pow(x_, 2))); // to close the loop with redundant first point

        for (size_t i = 0; i < y.size(); i++)
        {
            points.push_back({x[i], y[i]});
        }

        std::unique_ptr<smoothing::SplineI> fit_circle = waypoint_generation::compute_fit(points);
        double param = 0.0;
        for (int i = 0; i < 40; i++)
        {
            (*fit_circle)(param);
            param += 1.0 / 40.0;
        }
        (*fit_circle)(param);

        //TODO: what is supposed to be happening here?  The results of this loop are never used (circle_param)
        double circle_param = 0.0;
        for (auto i = 0; i < 50; i++)
        {
            circle_param += 0.02;
        }

        ASSERT_NEAR(waypoint_generation::compute_curvature_at((*fit_circle), 0.0), 1.0 / radius, 0.005); // check start curvature 1/r
        // check curvature is consistent ??????????
        ASSERT_NEAR(waypoint_generation::compute_curvature_at((*fit_circle), 0.42), waypoint_generation::compute_curvature_at((*fit_circle), 0.85), 0.005);
        ASSERT_NEAR(waypoint_generation::compute_curvature_at((*fit_circle), 0.0), waypoint_generation::compute_curvature_at((*fit_circle), 1.0), 0.005);
        ASSERT_NEAR(waypoint_generation::compute_curvature_at((*fit_circle), 0.23), waypoint_generation::compute_curvature_at((*fit_circle), 0.99), 0.005);
        ASSERT_NEAR(waypoint_generation::compute_curvature_at((*fit_circle), 0.12), waypoint_generation::compute_curvature_at((*fit_circle), 0.76), 0.005);
    }

    TEST(BasicAutonomyTest, attach_back_points)
    {
        std::vector<waypoint_generation::PointSpeedPair> points;
        std::vector<waypoint_generation::PointSpeedPair> future_points;

        waypoint_generation::PointSpeedPair p;
        p.point = lanelet::BasicPoint2d(0, 1);
        p.speed = 1.0;
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(1, 2);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(2, 3);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(3, 4);
        future_points.push_back(p);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(4, 5);
        future_points.push_back(p);
        points.push_back(p);
        p.point = lanelet::BasicPoint2d(5, 6);
        future_points.push_back(p);
        points.push_back(p);

        int nearest_pt_index = 2;

        auto result = waypoint_generation::attach_past_points(points, future_points, nearest_pt_index, 1.5);

        ASSERT_EQ(points.size() - 1, result.size());
        ASSERT_NEAR(1.0, result[0].point.x(), 0.0000001);
        ASSERT_NEAR(2.0, result[1].point.x(), 0.0000001);
        ASSERT_NEAR(3.0, result[2].point.x(), 0.0000001);
        ASSERT_NEAR(4.0, result[3].point.x(), 0.0000001);
        ASSERT_NEAR(5.0, result[4].point.x(), 0.0000001);

        ASSERT_NEAR(2.0, result[0].point.y(), 0.0000001);
        ASSERT_NEAR(3.0, result[1].point.y(), 0.0000001);
        ASSERT_NEAR(4.0, result[2].point.y(), 0.0000001);
        ASSERT_NEAR(5.0, result[3].point.y(), 0.0000001);
        ASSERT_NEAR(6.0, result[4].point.y(), 0.0000001);
    }

    TEST(BasicAutonomyTest, test_lanechange_trajectory)
    {
        //Case 1: Lane change from start to end of adjacent lanelets
        std::string path = ament_index_cpp::get_package_share_directory("basic_autonomy_ros2");
        std::string file = "/resource/map/town01_vector_map_lane_change.osm";
        file = path.append(file);
        int projector_type = 0;
        std::string target_frame;
        lanelet::ErrorMessages load_errors;
        lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
        lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
        lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);
        if (map->laneletLayer.size() == 0)
        {
            FAIL() << "Input map does not contain any lanelets";
        }
        std::shared_ptr<carma_wm::CARMAWorldModel> cwm = std::make_shared<carma_wm::CARMAWorldModel>();
        cwm->carma_wm::CARMAWorldModel::setMap(map);

        //Set Route
        lanelet::Id start_id = 106;
        lanelet::Id end_id = 111;

        carma_wm::test::setRouteByIds({start_id,end_id},cwm);
        //get starting position
        auto shortest_path = cwm->getRoute()->shortestPath();
        lanelet::BasicPoint2d veh_pos = shortest_path[0].centerline2d().front();
        double starting_downtrack =cwm->routeTrackPos(veh_pos).downtrack;
        double ending_downtrack = cwm->routeTrackPos(shortest_path.back().centerline2d().back()).downtrack;

        //Arguments for create geometry profile function-
        carma_planning_msgs::msg::Maneuver maneuver;
        maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
        maneuver.lane_change_maneuver.start_dist = starting_downtrack;
        maneuver.lane_change_maneuver.end_dist = ending_downtrack;
        maneuver.lane_change_maneuver.start_speed = 5.0;
        maneuver.lane_change_maneuver.start_time = rclcpp::Time(10.0 * 1e9); // Arbitrarily selected 10 seconds, converted to nanoseconds
        //calculate end_time assuming constant acceleration
        double acc = pow(maneuver.lane_change_maneuver.start_speed, 2) / (2 * (ending_downtrack - starting_downtrack));
        double end_time = maneuver.lane_change_maneuver.start_speed / acc;
        maneuver.lane_change_maneuver.end_speed = 25.0;
        maneuver.lane_change_maneuver.end_time = rclcpp::Time((end_time + 10.0) * 1e9); // Add 10 seconds and convert to nanoseconds
        maneuver.lane_change_maneuver.starting_lane_id = std::to_string(start_id);
        maneuver.lane_change_maneuver.ending_lane_id = std::to_string(end_id);

        std::vector<carma_planning_msgs::msg::Maneuver> maneuvers;
        maneuvers.push_back(maneuver);
        carma_planning_msgs::msg::VehicleState state;
        state.x_pos_global = veh_pos.x();
        state.y_pos_global = veh_pos.y();
        state.longitudinal_vel = 8.0;

        std::string trajectory_type = "cooperative_lanechange";
        waypoint_generation::GeneralTrajConfig general_config = waypoint_generation::compose_general_trajectory_config(trajectory_type, 1, 1);
        const waypoint_generation::DetailedTrajConfig config = waypoint_generation::compose_detailed_trajectory_config(0, 0, 0, 0, 0, 5, 0, 0, 20);
        carma_planning_msgs::msg::VehicleState ending_state;
        
        std::vector<basic_autonomy::waypoint_generation::PointSpeedPair> points = basic_autonomy::waypoint_generation::create_geometry_profile(maneuvers, 
                                                                                    starting_downtrack, cwm, ending_state, state, general_config, config);
        
        EXPECT_EQ(points.back().speed, state.longitudinal_vel);


        //Case 2: Complete lane change before end of lanelet
        int centerline_size = shortest_path.back().centerline2d().size();
        maneuver.lane_change_maneuver.end_dist = cwm->routeTrackPos(shortest_path.back().centerline2d()[centerline_size/2]).downtrack;
        maneuvers[0] = maneuver;
        points = basic_autonomy::waypoint_generation::create_geometry_profile(maneuvers, 
                                                                                    starting_downtrack, cwm, ending_state, state, general_config, config);
        
        EXPECT_TRUE(points.size() > 4);

        //Test resample linestring
        lanelet::BasicLineString2d line_one = shortest_path.front().centerline2d().basicLineString();
        std::vector<lanelet::BasicPoint2d> line_1;
        for(size_t i = 0;i<line_one.size();i++){
            line_1.push_back(line_one[i]);
        }
        lanelet::BasicLineString2d line_two = shortest_path.back().centerline2d().basicLineString();
        std::vector<lanelet::BasicPoint2d> line_2;
        for(size_t i = 0;i<line_two.size();i++){
            line_2.push_back(line_two[i]);
        }
        std::vector<std::vector<lanelet::BasicPoint2d>> linestrings = basic_autonomy::waypoint_generation::resample_linestring_pair_to_same_size(line_1, line_2);

    }

    TEST(BasicAutonomyTest, maneuvers_to_lanechange_points)
    {

        std::string path = ament_index_cpp::get_package_share_directory("basic_autonomy_ros2");
        std::string file = "/resource/map/town01_vector_map_lane_change.osm";
        file = path.append(file);
        int projector_type = 0;
        std::string target_frame;
        lanelet::ErrorMessages load_errors;
        lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
        lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
        lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);
        if (map->laneletLayer.size() == 0)
        {
            FAIL() << "Input map does not contain any lanelets";
        }
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        //Set Route
        lanelet::Id start_id = 106;
        lanelet::Id end_id = 111;
        carma_wm::test::setRouteByIds({start_id, end_id}, cmw);
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        //get starting position
        auto shortest_path = cmw->getRoute()->shortestPath();
        lanelet::BasicPoint2d veh_pos = shortest_path[0].centerline2d().front();
        double starting_downtrack = cmw->routeTrackPos(veh_pos).downtrack;
        double ending_downtrack = cmw->routeTrackPos(shortest_path.back().centerline2d().back()).downtrack;

        //Testing maneuvers to points lanechange
        //Arguments for function-
        carma_planning_msgs::msg::Maneuver maneuver;
        maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
        maneuver.lane_change_maneuver.start_dist = starting_downtrack;
        maneuver.lane_change_maneuver.end_dist = ending_downtrack;
        maneuver.lane_change_maneuver.start_speed = 5.0;
        maneuver.lane_change_maneuver.start_time = rclcpp::Time(10.0 * 1e9); // 10 seconds converted to nanoseconds
        //calculate end_time assuming constant acceleration
        double acc = pow(maneuver.lane_change_maneuver.start_speed, 2) / (2 * (ending_downtrack - starting_downtrack));
        double end_time = maneuver.lane_change_maneuver.start_speed / acc;
        maneuver.lane_change_maneuver.end_speed = 25.0;
        maneuver.lane_change_maneuver.end_time = rclcpp::Time((end_time + 10.0) * 1e9); // Add 10 seconds and convert to nanoseconds
        maneuver.lane_change_maneuver.starting_lane_id = std::to_string(start_id);
        maneuver.lane_change_maneuver.ending_lane_id = std::to_string(end_id);

        std::vector<carma_planning_msgs::msg::Maneuver> maneuvers;
        maneuvers.push_back(maneuver);
        carma_planning_msgs::msg::VehicleState state;
        state.x_pos_global = veh_pos.x();
        state.y_pos_global = veh_pos.y();
        state.longitudinal_vel = 8.0;

        std::string trajectory_type = "cooperative_lanechange";
        waypoint_generation::GeneralTrajConfig general_config = waypoint_generation::compose_general_trajectory_config(trajectory_type, 1, 1);
        const waypoint_generation::DetailedTrajConfig config = waypoint_generation::compose_detailed_trajectory_config(0, 1, 0, 0, 0, 5, 0, 0, 20);
        carma_planning_msgs::msg::VehicleState ending_state;
        
        std::vector<basic_autonomy::waypoint_generation::PointSpeedPair> points = basic_autonomy::waypoint_generation::create_geometry_profile(maneuvers, 
                                                                                    starting_downtrack, cmw, ending_state, state, general_config, config);
        rclcpp::Time state_time(10 * 1e9); // Arbitrarily selected 10 seconds (converted to nanoseconds)
        EXPECT_EQ(points.back().speed, state.longitudinal_vel);
        std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory_points = basic_autonomy::waypoint_generation::compose_lanechange_trajectory_from_path(points,
                                                                                                                                                          state, state_time, cmw, ending_state, config);
        EXPECT_TRUE(trajectory_points.size() > 2);
        basic_autonomy::waypoint_generation::create_lanechange_geometry(start_id, end_id,starting_downtrack, ending_downtrack, cmw, 1, 5);
    }

    TEST(BasicAutonomyTest, lanefollow_geometry_visited_lanelets)
    {

        double starting_downtrack = 0;
        carma_planning_msgs::msg::VehicleState ending_state;
        std::string trajectory_type = "lane_follow";
        waypoint_generation::GeneralTrajConfig general_config = waypoint_generation::compose_general_trajectory_config(trajectory_type, 0, 0);
        waypoint_generation::DetailedTrajConfig detailed_config = waypoint_generation::compose_detailed_trajectory_config(0, 0, 0, 0, 0, 5, 0, 0, 20);

        lanelet::Id id1 = 1100;
        std::unordered_set<lanelet::Id> visited_lanelets;
        visited_lanelets.insert(id1);
        std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
        auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);
        wm->setMap(map);
        carma_wm::test::setSpeedLimit(15_mph, wm);

        carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

        carma_planning_msgs::msg::Maneuver maneuver;
        maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        maneuver.lane_following_maneuver.lane_ids.push_back(std::to_string(1200));
        maneuver.lane_following_maneuver.start_dist = 5.0;
        maneuver.lane_following_maneuver.start_time = rclcpp::Time(0.0);
        maneuver.lane_following_maneuver.start_speed = 0.0;

        maneuver.lane_following_maneuver.end_dist = 14.98835712;
        maneuver.lane_following_maneuver.end_speed = 6.7056;
        maneuver.lane_following_maneuver.end_time = rclcpp::Time(4.4704*1e9); // 4.4704 seconds converted to nanoseconds

        

        std::vector<basic_autonomy::waypoint_generation::PointSpeedPair> points = basic_autonomy::waypoint_generation::create_lanefollow_geometry(maneuver, 
                                                                                    starting_downtrack, wm, general_config, detailed_config, visited_lanelets);

        EXPECT_EQ(visited_lanelets.size(), 3);

        EXPECT_TRUE(visited_lanelets.find(id1) != visited_lanelets.end()); // the lanelet previously in the visited lanelet set
        EXPECT_TRUE(visited_lanelets.find(1200) != visited_lanelets.end()); // new lanelets added to the set with the new maneuver
    }

    TEST(BasicAutonomyTest, lanefollow_defined_lanelets)
    {

        double starting_downtrack = 0;
        carma_planning_msgs::msg::VehicleState ending_state;
        std::string trajectory_type = "lane_follow";
        waypoint_generation::GeneralTrajConfig general_config = waypoint_generation::compose_general_trajectory_config(trajectory_type, 0, 0);
        waypoint_generation::DetailedTrajConfig detailed_config = waypoint_generation::compose_detailed_trajectory_config(0, 0, 0, 0, 0, 5, 0, 0, 20);

        lanelet::Id id1 = 1100;
        std::unordered_set<lanelet::Id> visited_lanelets;
        visited_lanelets.insert(id1);
        std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
        auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);
        wm->setMap(map);
        carma_wm::test::setSpeedLimit(15_mph, wm);

        carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

        carma_planning_msgs::msg::Maneuver maneuver;
        maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        maneuver.lane_following_maneuver.lane_ids = {};
        maneuver.lane_following_maneuver.start_dist = 5.0;
        maneuver.lane_following_maneuver.start_time = rclcpp::Time(0.0);
        maneuver.lane_following_maneuver.start_speed = 0.0;

        maneuver.lane_following_maneuver.end_dist = 14.98835712;
        maneuver.lane_following_maneuver.end_speed = 6.7056;
        maneuver.lane_following_maneuver.end_time = rclcpp::Time(4.4704*1e9); // 4.4704 seconds converted to nanoseconds

        // No defined lanelet ids in the maneuver msg
        EXPECT_THROW(basic_autonomy::waypoint_generation::create_lanefollow_geometry(maneuver, 
                                                                                    starting_downtrack, wm, general_config, detailed_config, visited_lanelets), std::invalid_argument);
        
        // Defined lanelet ids in the maneuver msg                                                                            
        maneuver.lane_following_maneuver.lane_ids.push_back(std::to_string(1200));
        std::vector<basic_autonomy::waypoint_generation::PointSpeedPair> points = basic_autonomy::waypoint_generation::create_lanefollow_geometry(maneuver, 
                                                                                    starting_downtrack, wm, general_config, detailed_config, visited_lanelets);
        EXPECT_EQ(visited_lanelets.size(), 3);
        EXPECT_TRUE(visited_lanelets.find(id1) != visited_lanelets.end()); // the lanelet previously in the visited lanelet set
        EXPECT_TRUE(visited_lanelets.find(1200) != visited_lanelets.end()); // new lanelets added to the set with the new maneuver

        // Non-following lanelet id in the maneuver msg
        maneuver.lane_following_maneuver.lane_ids.push_back(std::to_string(1202));
        EXPECT_THROW(basic_autonomy::waypoint_generation::create_lanefollow_geometry(maneuver, 
                                                                                    starting_downtrack, wm, general_config, 
                                                                                    detailed_config, visited_lanelets), std::invalid_argument);
    }

} // namespace basic_autonomy

// Run all the tests
int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
} 