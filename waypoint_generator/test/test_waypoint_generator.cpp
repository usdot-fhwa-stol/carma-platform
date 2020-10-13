/*
 * Copyright (C) 2020 LEIDOS.
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

#include <gmock/gmock.h>
#include <iostream>
#include <waypoint_generator/waypoint_generator.hpp>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_core/Attribute.h>
#include <carma_wm/Geometry.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_wm/WMTestLibForGuidance.h>
#include <math.h>
#include "TestHelpers.h"

using namespace waypoint_generator;

TEST(WaypointGeneratorTest, TestComputeConstantCurvatureRegions)
{
    WaypointGeneratorConfig config;
    std::shared_ptr<carma_wm::WorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    WaypointGenerator wpg(wm, config, [&](auto msg) {});

    std::vector<double> case_a = {0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<int> out_a = wpg.compute_constant_curvature_regions(case_a, 0.1, 1);

    ASSERT_EQ(1, out_a.size());
    ASSERT_EQ(9, out_a[0]);

    std::vector<double> case_b = {0.0, 0.0, 0.0, 0.0, 0.0,
                                  1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<int> out_b = wpg.compute_constant_curvature_regions(case_b, 0.1, 1);

    ASSERT_EQ(2, out_b.size());
    ASSERT_EQ(4, out_b[0]);
    ASSERT_EQ(9, out_b[1]);

    std::vector<double> case_c = {0.0, 0.0, 1.0, 1.0, 2.0,
                                  2.0, 3.0, 3.0, 4.0, 4.0};
    std::vector<int> out_c = wpg.compute_constant_curvature_regions(case_c, 0.1, 1);

    ASSERT_EQ(5, out_c.size());
    ASSERT_EQ(1, out_c[0]);
    ASSERT_EQ(3, out_c[1]);
    ASSERT_EQ(5, out_c[2]);
    ASSERT_EQ(7, out_c[3]);
    ASSERT_EQ(9, out_c[4]);

    std::vector<double> case_d = {0.0, 0.5, 1.0, 1.5, 2.0,
                                  2.5, 3.0, 3.5, 4.0, 4.5};
    std::vector<int> out_d = wpg.compute_constant_curvature_regions(case_d, 0.7, 1);

    ASSERT_EQ(5, out_d.size());
    ASSERT_EQ(1, out_d[0]);
    ASSERT_EQ(3, out_d[1]);
    ASSERT_EQ(5, out_d[2]);
    ASSERT_EQ(7, out_d[3]);
    ASSERT_EQ(9, out_d[4]);

    std::vector<double> case_e = {0.0, 0.5, 1.0, 1.5, 2.0,
                                  2.5, 3.0, 3.5, 4.0, 4.5};
    std::vector<int> out_e = wpg.compute_constant_curvature_regions(case_e, 0.25, 2);

    ASSERT_EQ(1, out_e.size());
    ASSERT_EQ(9, out_e[0]);
}

TEST(WaypointGeneratorTest, TestComputeIdealSpeeds)
{
    WaypointGeneratorConfig config;
    std::shared_ptr<carma_wm::WorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    WaypointGenerator wpg(wm, config, [&](auto msg) {});

    std::vector<double> curvatures_1 = {1.0, 1.0, 1.0, 1.0, 1.0,
                                        1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<double> out_1 = wpg.compute_ideal_speeds(curvatures_1, 1.0);

    ASSERT_EQ(10, out_1.size());
    ASSERT_NEAR(1.0, out_1[0], 0.005);
    ASSERT_NEAR(1.0, out_1[1], 0.005);
    ASSERT_NEAR(1.0, out_1[2], 0.005);
    ASSERT_NEAR(1.0, out_1[3], 0.005);
    ASSERT_NEAR(1.0, out_1[4], 0.005);
    ASSERT_NEAR(1.0, out_1[5], 0.005);
    ASSERT_NEAR(1.0, out_1[6], 0.005);
    ASSERT_NEAR(1.0, out_1[7], 0.005);
    ASSERT_NEAR(1.0, out_1[8], 0.005);
    ASSERT_NEAR(1.0, out_1[9], 0.005);

    std::vector<double> curvatures_2 = {1.0, 2.0, 3.0, 4.0, 5.0,
                                        6.0, 7.0, 8.0, 9.0, 10.0};
    std::vector<double> out_2 = wpg.compute_ideal_speeds(curvatures_2, 1.0);

    ASSERT_EQ(10, out_2.size());
    ASSERT_NEAR(1.0, out_2[0], 0.005);
    ASSERT_NEAR(std::sqrt(1.0 / 2.0), out_2[1], 0.005);
    ASSERT_NEAR(std::sqrt(1.0 / 3.0), out_2[2], 0.005);
    ASSERT_NEAR(std::sqrt(1.0 / 4.0), out_2[3], 0.005);
    ASSERT_NEAR(std::sqrt(1.0 / 5.0), out_2[4], 0.005);
    ASSERT_NEAR(std::sqrt(1.0 / 6.0), out_2[5], 0.005);
    ASSERT_NEAR(std::sqrt(1.0 / 7.0), out_2[6], 0.005);
    ASSERT_NEAR(std::sqrt(1.0 / 8.0), out_2[7], 0.005);
    ASSERT_NEAR(std::sqrt(1.0 / 9.0), out_2[8], 0.005);
    ASSERT_NEAR(std::sqrt(1.0 / 10.0), out_2[9], 0.005);
}

TEST(WaypointGeneratorTest, TestComputeSpeedForCurvature)
{
    WaypointGeneratorConfig config;
    std::shared_ptr<carma_wm::WorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    WaypointGenerator wpg(wm, config, [&](auto msg) {});

    double speed1 = wpg.compute_speed_for_curvature(1.0, 1.0);
    ASSERT_NEAR(1.0, speed1, 0.005);

    double speed2 = wpg.compute_speed_for_curvature(10.0, 3.33);
    ASSERT_NEAR(std::sqrt(3.33 / 10.0), speed2, 0.005);

    double speed3 = wpg.compute_speed_for_curvature(5.0, 10.0);
    ASSERT_NEAR(std::sqrt(10.0 / 5.0), speed3, 0.005);

    double speed4 = wpg.compute_speed_for_curvature(100.0, 1.0);
    ASSERT_NEAR(std::sqrt(1.0 / 100.0), speed4, 0.005);

    double speed5 = wpg.compute_speed_for_curvature(0.0, 0.0);
    ASSERT_TRUE(std::isinf(speed5));
}

TEST(WaypointGeneratorTest, TestNormalizeCurvatureRegions)
{

    WaypointGeneratorConfig config;
    std::shared_ptr<carma_wm::WorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    WaypointGenerator wpg(wm, config, [&](auto msg) {});

    std::vector<double> case_a = {0.0, 1.0, 0.0, 1.0, 0.0,
                                  0.0, 1.0, 0.0, 1.0, 0.0};
    std::vector<int> regions_a = {9};
    std::vector<double> out_a = wpg.normalize_curvature_regions(case_a, regions_a);

    ASSERT_EQ(10, out_a.size());
    ASSERT_NEAR(0.0, out_a[0], 0.001);
    ASSERT_NEAR(0.0, out_a[1], 0.001);
    ASSERT_NEAR(0.0, out_a[2], 0.001);
    ASSERT_NEAR(0.0, out_a[3], 0.001);
    ASSERT_NEAR(0.0, out_a[4], 0.001);
    ASSERT_NEAR(0.0, out_a[5], 0.001);
    ASSERT_NEAR(0.0, out_a[6], 0.001);
    ASSERT_NEAR(0.0, out_a[7], 0.001);
    ASSERT_NEAR(0.0, out_a[8], 0.001);
    ASSERT_NEAR(0.0, out_a[9], 0.001);

    std::vector<double> case_b = {1.5, 1.0, 1.5, 1.1, 1.3,
                                  2.3, 2.2, 2.0, 2.0, 3.0};
    std::vector<int> regions_b = {4, 9};
    std::vector<double> out_b = wpg.normalize_curvature_regions(case_b, regions_b);

    ASSERT_EQ(10, out_b.size());
    ASSERT_NEAR(1.0, out_b[0], 0.001);
    ASSERT_NEAR(1.0, out_b[1], 0.001);
    ASSERT_NEAR(1.0, out_b[2], 0.001);
    ASSERT_NEAR(1.0, out_b[3], 0.001);
    ASSERT_NEAR(1.0, out_b[4], 0.001);
    ASSERT_NEAR(2.0, out_b[5], 0.001);
    ASSERT_NEAR(2.0, out_b[6], 0.001);
    ASSERT_NEAR(2.0, out_b[7], 0.001);
    ASSERT_NEAR(2.0, out_b[8], 0.001);
    ASSERT_NEAR(2.0, out_b[9], 0.001);
}

TEST(WaypointGeneratorTest, TestApplyAccelLimits)
{

    WaypointGeneratorConfig config;
    std::shared_ptr<carma_wm::WorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    WaypointGenerator wpg(wm, config, [&](auto msg) {});

    std::vector<double> speeds_a = {0.0, 0.0, 0.0, 0.0, 0.0,
                                    5.0, 5.0, 5.0, 5.0, 5.0};
    std::vector<int> regions_a = {4, 9};

    lanelet::BasicPoint2d point1{0, 0};
    lanelet::BasicPoint2d point2{1, 0};
    lanelet::BasicPoint2d point3{2, 0};
    lanelet::BasicPoint2d point4{3, 0};
    lanelet::BasicPoint2d point5{4, 0};
    lanelet::BasicPoint2d point6{5, 0};
    lanelet::BasicPoint2d point7{6, 0};
    lanelet::BasicPoint2d point8{7, 0};
    lanelet::BasicPoint2d point9{8, 0};
    lanelet::BasicPoint2d point10{9, 0};
    lanelet::BasicLineString2d centerline_a{
        point1, point2, point3, point4,
        point5, point6, point7, point8,
        point9, point10};

    std::vector<double> limited_a;
    limited_a = wpg.apply_accel_limits(speeds_a,
                                       regions_a, centerline_a, 3.0, 3.0);

    ASSERT_EQ(10, limited_a.size());
    ASSERT_NEAR(0.0, limited_a[0], 0.01);
    ASSERT_NEAR(0.0, limited_a[1], 0.01);
    ASSERT_NEAR(0.0, limited_a[2], 0.01);
    ASSERT_NEAR(0.0, limited_a[3], 0.01);
    ASSERT_NEAR(0.0, limited_a[4], 0.01);
    ASSERT_NEAR(1.0, limited_a[5], 0.01);
    ASSERT_NEAR(2.0, limited_a[6], 0.01);
    ASSERT_NEAR(3.0, limited_a[7], 0.01);
    ASSERT_NEAR(4.0, limited_a[8], 0.01);
    ASSERT_NEAR(5.0, limited_a[9], 0.01);


    // Test slowdown case
    std::vector<double> speeds_b = {5.0, 5.0, 5.0, 5.0, 5.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> limited_b;
    limited_b = wpg.apply_accel_limits(speeds_b,
                                       regions_a, centerline_a, 3.0, 3.0);

    ASSERT_EQ(10, limited_b.size());
    ASSERT_NEAR(5.0, limited_b[0], 0.01);
    ASSERT_NEAR(4.0, limited_b[1], 0.01);
    ASSERT_NEAR(3.0, limited_b[2], 0.01);
    ASSERT_NEAR(2.0, limited_b[3], 0.01);
    ASSERT_NEAR(1.0, limited_b[4], 0.01);
    ASSERT_NEAR(0.0, limited_b[5], 0.01);
    ASSERT_NEAR(0.0, limited_b[6], 0.01);
    ASSERT_NEAR(0.0, limited_b[7], 0.01);
    ASSERT_NEAR(0.0, limited_b[8], 0.01);
    ASSERT_NEAR(0.0, limited_b[9], 0.01);


    // Test varying speed case
    std::vector<double> speeds_c = {0.0, 0.0, 5.0, 5.0, 5.0,
                                    10.0, 10.0, 10.0, 3.0, 3.0};
    std::vector<int> regions_c = {1, 4, 7, 9};

    std::vector<double> limited_c;
    limited_c = wpg.apply_accel_limits(speeds_c,
                                       regions_c, centerline_a, 4.0, 4.0);

    ASSERT_EQ(10, limited_c.size());
    ASSERT_NEAR(0.0, limited_c[0], 0.01);
    ASSERT_NEAR(0.0, limited_c[1], 0.01);
    ASSERT_NEAR(1.63, limited_c[2], 0.01);
    ASSERT_NEAR(3.26, limited_c[3], 0.01);
    ASSERT_NEAR(4.89, limited_c[4], 0.01);
    ASSERT_NEAR(6.89, limited_c[5], 0.01);
    ASSERT_NEAR(6.89, limited_c[6], 0.01);
    ASSERT_NEAR(5.89, limited_c[7], 0.01);
    ASSERT_NEAR(3.0, limited_c[8], 0.01);
    ASSERT_NEAR(3.0, limited_c[9], 0.01);

    // Test large speeds case
    std::vector<double> speeds_d = {15.0, 15.0, 15.0, 15.0, 15.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> limited_d;
    limited_b = wpg.apply_accel_limits(speeds_b,
                                       regions_a, centerline_a, 3.0, 3.0);

    ASSERT_EQ(10, limited_b.size());
    ASSERT_NEAR(5.0, limited_b[0], 0.01);
    ASSERT_NEAR(4.0, limited_b[1], 0.01);
    ASSERT_NEAR(3.0, limited_b[2], 0.01);
    ASSERT_NEAR(2.0, limited_b[3], 0.01);
    ASSERT_NEAR(1.0, limited_b[4], 0.01);
    ASSERT_NEAR(0.0, limited_b[5], 0.01);
    ASSERT_NEAR(0.0, limited_b[6], 0.01);
    ASSERT_NEAR(0.0, limited_b[7], 0.01);
    ASSERT_NEAR(0.0, limited_b[8], 0.01);
    ASSERT_NEAR(0.0, limited_b[9], 0.01);
}

TEST(WaypointGeneratorTest, TestApplySpeedLimits)
{

    WaypointGeneratorConfig config;
    std::shared_ptr<carma_wm::WorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    WaypointGenerator wpg(wm, config, [&](auto msg) {});

    std::vector<double> speeds_a = {0.0, 0.0, 0.0, 0.0, 0.0,
                                    5.0, 5.0, 5.0, 5.0, 5.0};
    std::vector<double> limits_a = {3.0, 3.0, 3.0, 3.0, 3.0,
                                    3.0, 3.0, 3.0, 3.0, 3.0};
    std::vector<double> out_a = wpg.apply_speed_limits(speeds_a, limits_a);

    ASSERT_EQ(10, out_a.size());
    for (int i = 0; i < speeds_a.size(); i++)
    {
        ASSERT_LE(out_a[i], limits_a[i]);
    }
}

TEST(WaypointGeneratorTest, TestGetSpeedLimits)
{

    WaypointGeneratorConfig config;
    std::shared_ptr<carma_wm::WorldModel> wm;
    WaypointGenerator wpg(wm, config, [&](auto msg) {});

    std::shared_ptr<carma_wm::CARMAWorldModel> cmw = carma_wm::test::getGuidanceTestMap();
    std::shared_ptr<carma_wm::WorldModel> wm1 = cmw;
    std::vector<lanelet::ConstLanelet> input;
    input.push_back(cmw->getMap()->laneletLayer.get(1200));

    ROS_INFO_STREAM("Following get_speed_limit: Invalid WM error is expected");
    ASSERT_THROW(wpg.get_speed_limits(input), std::invalid_argument);

    WaypointGenerator wpg1(cmw, config, [&](auto msg) {});
    std::vector<lanelet::ConstLanelet> input_err;
    ROS_INFO_STREAM("Following get_speed_limit: Invalid lanelets error is expected");
    ASSERT_THROW(wpg1.get_speed_limits(input_err), std::invalid_argument);

    // create a copy lanelet with no regems inside it
    lanelet::ConstLanelet llt = carma_wm::test::getLanelet(1200, cmw->getMutableMap()->laneletLayer.get(1200).leftBound3d(), cmw->getMutableMap()->laneletLayer.get(1200).rightBound3d());
    input_err.push_back(llt);
    ROS_INFO_STREAM("Following get_speed_limit: error is expected");
    ASSERT_THROW(wpg1.get_speed_limits(input_err), std::invalid_argument);

    std::vector<double> result = wpg1.get_speed_limits(input);
    ASSERT_EQ(result.size(), 3);
    ASSERT_NE(result[0], 0);
}

TEST(WaypointGeneratorTest, TestGenerateLaneArray)
{
    ros::Time::init(); // required here by ros::Time::now() in the function
    // following values in the test are dummy values
    // and does not represent valid values for any testing except unit testing
    WaypointGeneratorConfig config;
    std::shared_ptr<carma_wm::WorldModel> wm = carma_wm::test::getGuidanceTestMap();
    WaypointGenerator wpg(wm, config, [&](auto msg) {});

    std::vector<double> speeds = {1, 2, 3, 4, 5, 6};

    geometry_msgs::Quaternion rot1 = tf::createQuaternionMsgFromYaw(1);
    geometry_msgs::Quaternion rot2 = tf::createQuaternionMsgFromYaw(2);
    geometry_msgs::Quaternion rot3 = tf::createQuaternionMsgFromYaw(3);
    geometry_msgs::Quaternion rot4 = tf::createQuaternionMsgFromYaw(4);
    geometry_msgs::Quaternion rot5 = tf::createQuaternionMsgFromYaw(5);
    geometry_msgs::Quaternion rot6 = tf::createQuaternionMsgFromYaw(6);

    std::vector<geometry_msgs::Quaternion> orientations = {rot1, rot2, rot3, rot4, rot5, rot6};
    std::vector<lanelet::ConstLanelet> lanelets;
    lanelets.push_back(wm->getMap()->laneletLayer.get(1200));
    lanelets.push_back(wm->getMap()->laneletLayer.get(1201));

    autoware_msgs::LaneArray msg = wpg.generate_lane_array_message(
        speeds,
        orientations,
        lanelets);
    ASSERT_EQ(msg.lanes.size(), 2);
    // check lane_id of lane
    ASSERT_EQ(msg.lanes[0].lane_id, 0);
    // check lane_id of waypoint
    ASSERT_EQ(msg.lanes[0].waypoints[0].lane_id, 0);
    // check orientation of waypoint
    ASSERT_EQ(msg.lanes[0].waypoints[0].pose.pose.orientation.x, rot1.x); //just checking one random side of 4 possible
    ASSERT_EQ(msg.lanes[0].waypoints[1].pose.pose.orientation.y, rot2.y);
    ASSERT_EQ(msg.lanes[0].waypoints[2].pose.pose.orientation.z, rot3.z);
    ASSERT_EQ(msg.lanes[1].waypoints[0].pose.pose.orientation.w, rot4.w);
    ASSERT_EQ(msg.lanes[1].waypoints[1].pose.pose.orientation.x, rot5.x);
    ASSERT_EQ(msg.lanes[1].waypoints[2].pose.pose.orientation.x, rot6.x);
    // check twist (speeds)
    ASSERT_EQ(msg.lanes[0].waypoints[0].twist.twist.linear.x, 1);
    ASSERT_EQ(msg.lanes[0].waypoints[1].twist.twist.linear.x, 2);
    ASSERT_EQ(msg.lanes[0].waypoints[2].twist.twist.linear.x, 3);
    ASSERT_EQ(msg.lanes[1].waypoints[0].twist.twist.linear.x, 4);
    ASSERT_EQ(msg.lanes[1].waypoints[1].twist.twist.linear.x, 5);
    ASSERT_EQ(msg.lanes[1].waypoints[2].twist.twist.linear.x, 6);
}
}
