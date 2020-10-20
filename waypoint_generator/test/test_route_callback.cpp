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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tuple>
#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

using namespace waypoint_generator;
using namespace lanelet::units::literals;

// void rpyFromQuatMsg(const geometry_msgs::Quaternion& q_msg, double& roll, double& pitch, double& yaw)
// {
//   tf2::Quaternion quat;
//   tf2::convert(q_msg, quat);
//   tf2::Matrix3x3 mat(quat);
//   mat.getRPY(roll, pitch, yaw);
// }

void assert_waypoint_state_eq(const autoware_msgs::WaypointState& expected, const autoware_msgs::WaypointState& actual) {
  ASSERT_EQ(expected.aid, actual.aid);
  ASSERT_EQ(expected.lanechange_state, actual.lanechange_state);
  ASSERT_EQ(expected.steering_state, actual.steering_state);
  ASSERT_EQ(expected.accel_state, actual.accel_state);
  ASSERT_EQ(expected.stop_state, actual.stop_state);
  ASSERT_EQ(expected.event_state, actual.event_state);
}

void assert_pose_eq(const geometry_msgs::Pose& expected, const geometry_msgs::Pose& actual) {
  // Position
  ASSERT_DOUBLE_EQ(expected.position.x, actual.position.x);
  ASSERT_DOUBLE_EQ(expected.position.y, actual.position.y);
  ASSERT_DOUBLE_EQ(expected.position.z, actual.position.z);
  // Orientation
  ASSERT_DOUBLE_EQ(expected.orientation.x, actual.orientation.x);
  ASSERT_DOUBLE_EQ(expected.orientation.y, actual.orientation.y);
  ASSERT_DOUBLE_EQ(expected.orientation.z, actual.orientation.z);
  ASSERT_DOUBLE_EQ(expected.orientation.w, actual.orientation.w);
}

void assert_twist_eq(const geometry_msgs::Twist& expected, const geometry_msgs::Twist& actual) {
  // Linear
  ASSERT_DOUBLE_EQ(expected.linear.x, actual.linear.x);
  ASSERT_DOUBLE_EQ(expected.linear.y, actual.linear.y);
  ASSERT_DOUBLE_EQ(expected.linear.z, actual.linear.z);
  // Angular
  ASSERT_DOUBLE_EQ(expected.angular.x, actual.angular.x);
  ASSERT_DOUBLE_EQ(expected.angular.y, actual.angular.y);
  ASSERT_DOUBLE_EQ(expected.angular.z, actual.angular.z);
}

void assert_dtlane_eq(const autoware_msgs::DTLane& expected, const autoware_msgs::DTLane& actual) {
  ASSERT_DOUBLE_EQ(expected.dist, actual.dist);
  ASSERT_DOUBLE_EQ(expected.apara, actual.apara);
  ASSERT_DOUBLE_EQ(expected.r, actual.r);
  ASSERT_DOUBLE_EQ(expected.slope, actual.slope);
  ASSERT_DOUBLE_EQ(expected.cant, actual.cant);
  ASSERT_DOUBLE_EQ(expected.lw, actual.lw);
  ASSERT_DOUBLE_EQ(expected.rw, actual.rw);  
}


void assert_waypoint_eq(const autoware_msgs::Waypoint& expected, const autoware_msgs::Waypoint& actual) {
  ASSERT_EQ(expected.gid, actual.gid);
  ASSERT_EQ(expected.lid, actual.lid);
  ASSERT_STREQ(expected.pose.header.frame_id.c_str(), actual.pose.header.frame_id.c_str()); // Time stamp is ignored
  assert_pose_eq(expected.pose.pose, actual.pose.pose);
  assert_twist_eq(expected.twist.twist, actual.twist.twist);
  assert_dtlane_eq(expected.dtlane, actual.dtlane);
  ASSERT_EQ(expected.change_flag, actual.change_flag);
  assert_waypoint_state_eq(expected.wpstate, actual.wpstate);
  ASSERT_EQ(expected.lane_id, actual.lane_id);
  ASSERT_EQ(expected.left_lane_id, actual.left_lane_id);
  ASSERT_EQ(expected.right_lane_id, actual.right_lane_id);
  ASSERT_EQ(expected.stop_line_id, actual.stop_line_id);
  ASSERT_FLOAT_EQ(expected.cost, actual.cost);
  ASSERT_FLOAT_EQ(expected.time_cost, actual.time_cost);
  ASSERT_EQ(expected.direction, actual.direction);
}

void assert_lane_eq(const autoware_msgs::Lane& expected, const autoware_msgs::Lane& actual) {
  ASSERT_STREQ(expected.header.frame_id.c_str(), actual.header.frame_id.c_str()); // Time stamp is ignored
  ASSERT_EQ(expected.increment, actual.increment);
  ASSERT_EQ(expected.lane_id, actual.lane_id);

  ASSERT_EQ(expected.waypoints.size(), actual.waypoints.size());
  for (int i = 0; i < expected.waypoints.size(); i++) {
    assert_waypoint_eq(expected.waypoints[i], actual.waypoints[i]);
  }

  ASSERT_EQ(expected.lane_index, actual.lane_index);
  ASSERT_FLOAT_EQ(expected.cost, actual.cost);
  ASSERT_FLOAT_EQ(expected.closest_object_distance, actual.closest_object_distance);
  ASSERT_FLOAT_EQ(expected.closest_object_velocity, actual.closest_object_velocity);
  ASSERT_EQ(expected.is_blocked, actual.is_blocked);
}



void assert_lane_array_eq(const autoware_msgs::LaneArray& expected, const autoware_msgs::LaneArray& actual) {
  ASSERT_EQ(expected.id, actual.id);
  ASSERT_EQ(expected.lanes.size(), actual.lanes.size());
  for (int i = 0; i < expected.lanes.size(); i++) {
    assert_lane_eq(expected.lanes[i], actual.lanes[i]);
  } 
}


/**
 * NOTE: This test depends on the test for generate_lane_array_message to be valid and passing
 */ 
TEST(WaypointGeneratorTest, basic_route)
{
  WaypointGeneratorConfig config;
  std::shared_ptr<CARMAWorldModel> wm = std::make_shared<CARMAWorldModel>();
  bool wp_published;
  autoware_msgs::LaneArray published_wps;
  WaypointGenerator wpg(wm, config, [&](auto msg) {published_wps = msg; wp_published = true;});

  auto map = carma_wm::test::buildGuidanceTestMap(3.7, 25);

  wm->setMap(map);
  carma_wm::test::setSpeedLimit(25_mph, wm);

  /**
   *
   *
   *        |1203|1213|1223|
   *        | _  _  _  _  _|
   *        |1202| Ob |1222|
   *        | _  _  _  _  _|
   *        |1201|1211|1221|    num   = lanelet id hardcoded for easier testing
   *        | _  _  _  _  _|    |     = lane lines
   *        |1200|1210|1220|    - - - = Lanelet boundary
   *        |              |    O     = Default Obstacle
   *        ****************
   *           START_LINE
   */

  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);


  wpg.new_route_callback();

  ASSERT_TRUE(wp_published);
  wp_published = false; // Reset flag
  
  std::cerr << "Number Of Lanes: " << published_wps.lanes.size() << std::endl;

  auto route_lanelets = wm->getRoute()->shortestPath();

  lanelet::ConstLanelets lanelets_as_vec;
  std::vector<double> speeds;
  geometry_msgs::Quaternion vert_rot = tf::createQuaternionMsgFromYaw(M_PI_2);

  std::vector<geometry_msgs::Quaternion> orientations;
  size_t num_points = 0;
  for (lanelet::ConstLanelet ll : route_lanelets)
  {
    lanelets_as_vec.push_back(ll);
    for (auto p : ll.centerline2d()) {
      speeds.push_back((25_mph).value());
      orientations.push_back(vert_rot);
      num_points++;
    }    
  }

  autoware_msgs::LaneArray expected_wp = wpg.generate_lane_array_message(
                speeds, 
                orientations, 
                lanelets_as_vec);

  assert_lane_array_eq(expected_wp, published_wps); // Verify output data
}

TEST(WaypointGeneratorTest, following_lanelet)
{
  WaypointGeneratorConfig config;
  std::shared_ptr<CARMAWorldModel> wm = std::make_shared<CARMAWorldModel>();
  bool wp_published;
  autoware_msgs::LaneArray published_wps;
  WaypointGenerator wpg(wm, config, [&](auto msg) {published_wps = msg; wp_published = true;});

  auto map = carma_wm::test::buildGuidanceTestMap(3.7, 25);

  wm->setMap(map);
  carma_wm::test::setSpeedLimit(25_mph, wm);

  /**
   *
   *
   *        |1203|1213|1223|
   *        | _  _  _  _  _|
   *        |1202| Ob |1222|
   *        | _  _  _  _  _|
   *        |1201|1211|1221|    num   = lanelet id hardcoded for easier testing
   *        | _  _  _  _  _|    |     = lane lines
   *        |1200|1210|1220|    - - - = Lanelet boundary
   *        |              |    O     = Default Obstacle
   *        ****************
   *           START_LINE
   */

  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

  std::vector<lanelet::ConstLanelet> res1 =  wpg.findSuccessingLanelets();
  ASSERT_EQ(res1.size(), 4);
  ASSERT_EQ(res1[0].id(), 1200);
  ASSERT_EQ(res1[1].id(), 1201);
  ASSERT_EQ(res1[2].id(), 1202);
  ASSERT_EQ(res1[3].id(), 1203);

  carma_wm::test::setRouteByIds({ 1200, 1211, 1212, 1213 }, wm);
  ASSERT_THROW( wpg.findSuccessingLanelets(), std::invalid_argument);


  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1223 }, wm);
  ASSERT_THROW( wpg.findSuccessingLanelets(), std::invalid_argument);

  auto p1 = carma_wm::test::getPoint(0, 0, 0);
  auto p2 = carma_wm::test::getPoint(1, 0, 0);
  auto p3 = carma_wm::test::getPoint(0, 1, 0);
  auto p4 = carma_wm::test::getPoint(1, 1, 0);
  auto p5 = carma_wm::test::getPoint(-1, 3, 0);
  auto p6 = carma_wm::test::getPoint(0, 3, 0);
  auto p7 = carma_wm::test::getPoint(2, 3, 0);
  auto p8 = carma_wm::test::getPoint(3, 3, 0);

  std::vector<lanelet::Point3d> left_1 = {p1, p3};
  std::vector<lanelet::Point3d> right_1 = {p2, p4};

  std::vector<lanelet::Point3d> left_2 = {p3, p5};
  std::vector<lanelet::Point3d> right_2 = {p4, p6};

  std::vector<lanelet::Point3d> left_3 = {p3, p7};
  std::vector<lanelet::Point3d> right_3 = {p4, p8};

  lanelet::Lanelet ll_1 = carma_wm::test::getLanelet(left_1, right_1);
  lanelet::Lanelet ll_2 = carma_wm::test::getLanelet(left_2, right_2);
  lanelet::Lanelet ll_3 = carma_wm::test::getLanelet(left_3, right_3);

  map = lanelet::utils::createMap({ ll_1, ll_2, ll_3 }, {});
  lanelet::MapConformer::ensureCompliance(map);

  wm->setMap(map);

  carma_wm::test::setRouteByIds({ ll_1.id(), ll_3.id() }, wm);

  std::vector<lanelet::ConstLanelet> res3 =  wpg.findSuccessingLanelets();
  ASSERT_EQ(res3.size(), 2);
  ASSERT_EQ(res3[0].id(), ll_1.id());
  ASSERT_EQ(res3[1].id(), ll_3.id());

}

}