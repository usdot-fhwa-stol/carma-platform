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

void rpyFromQuatMsg(const geometry_msgs::Quaternion& q_msg, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion quat;
  tf2::convert(q_msg, quat);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch, yaw);
}

TEST(WaypointGeneratorTest, compute_orientations_straight)
{
  WaypointGeneratorConfig config;
  std::shared_ptr<CARMAWorldModel> wm = std::make_shared<CARMAWorldModel>();
  WaypointGenerator wpg(wm, config, [&](auto msg) {});

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

  auto route_lanelets = wm->getRoute()->shortestPath();

  lanelet::ConstLanelets lanelets_as_vec;

  size_t num_points = 0;
  for (lanelet::ConstLanelet ll : route_lanelets)
  {
    lanelets_as_vec.push_back(ll);
    num_points += ll.centerline2d().size();
  }

  std::vector<geometry_msgs::Quaternion> result;
  result = wpg.compute_orientations(lanelets_as_vec);

  ASSERT_EQ(num_points, result.size());

  for (geometry_msgs::Quaternion q_msg : result)
  {
    double roll, pitch, yaw;
    rpyFromQuatMsg(q_msg, roll, pitch, yaw);
    ASSERT_NEAR(0.0, roll, 0.000001);
    ASSERT_NEAR(0.0, pitch, 0.000001);
    ASSERT_NEAR(M_PI_2, yaw, 0.000001);
  }
}

TEST(WaypointGeneratorTest, compute_orientations_curved)
{

  // This tests creates a 90 deg left turn from x = 0 that is followed by a short straight away
  // The inner turn radius is 30m and the outer radius is 34m
  // The straight away is 4m long. The turn is one lanelet the straight section is another
  WaypointGeneratorConfig config;
  std::shared_ptr<CARMAWorldModel> wm = std::make_shared<CARMAWorldModel>();
  WaypointGenerator wpg(wm, config, [&](auto msg) {});

  int segments = 6;
  double rad_increment = M_PI_2 / (double) segments;
  double inner_radius = 30;
  double outer_radius = 34;
  std::vector<lanelet::Point3d> left_points_1, right_points_1, left_points_2, right_points_2;
  double angle = 0;
  for (int i = 0; i <= segments; i++) {
    left_points_1.push_back(carma_wm::test::getPoint(inner_radius * cos(angle), inner_radius * sin(angle), 0));
    right_points_1.push_back(carma_wm::test::getPoint(outer_radius * cos(angle), outer_radius * sin(angle), 0));
    angle += rad_increment;
  }

  left_points_2.push_back(left_points_1.back());
  left_points_2.push_back(carma_wm::test::getPoint(-4, 30, 0));

  right_points_2.push_back(right_points_1.back());
  right_points_2.push_back(carma_wm::test::getPoint(-4, 34, 0));

  lanelet::Lanelet ll_1 = carma_wm::test::getLanelet(left_points_1, right_points_1);
  lanelet::Lanelet ll_2 = carma_wm::test::getLanelet(left_points_2, right_points_2);

  lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1, ll_2 }, {});
  lanelet::MapConformer::ensureCompliance(map);

  wm->setMap(map);
  carma_wm::test::setSpeedLimit(25_mph, wm);

  carma_wm::test::setRouteByLanelets({ ll_1, ll_2 }, wm);

  auto route_lanelets = wm->getRoute()->shortestPath();

  lanelet::ConstLanelets lanelets_as_vec;

  std::vector<geometry_msgs::Quaternion> result;
  result = wpg.compute_orientations({ lanelet::traits::toConst(ll_1), lanelet::traits::toConst(ll_2) });

  size_t num_points = 0;
  for (lanelet::ConstLanelet ll : route_lanelets)
  {
    lanelets_as_vec.push_back(ll);
    num_points += ll.centerline2d().size();
  }

  ASSERT_EQ(num_points, result.size());

  double roll, pitch, yaw;

  for (int i = 0; i < result.size(); i++){
    rpyFromQuatMsg(result[i], roll, pitch, yaw);
    std::cerr << i << " Yaw: " << yaw << std::endl;
  }
  rpyFromQuatMsg(result[0], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(1.7017, yaw, 0.00001); // First point has some error which is allowable due to mathemtical constraints on calculating the tangent

  rpyFromQuatMsg(result[1], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(1.7017, yaw, 0.00001);

  rpyFromQuatMsg(result[2], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(1.82437, yaw, 0.00001);

  rpyFromQuatMsg(result[3], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(1.9635, yaw, 0.00001);

  rpyFromQuatMsg(result[4], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(2.0944, yaw, 0.00001);

  rpyFromQuatMsg(result[5], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(2.22529, yaw, 0.00001);

  rpyFromQuatMsg(result[6], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(2.35619, yaw, 0.00001);

  rpyFromQuatMsg(result[7], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(2.48709, yaw, 0.00001);

  rpyFromQuatMsg(result[8], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(2.60977, yaw, 0.00001);

  rpyFromQuatMsg(result[9], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(2.74889, yaw, 0.00001);

  rpyFromQuatMsg(result[10], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(2.87157, yaw, 0.00001);

  rpyFromQuatMsg(result[11], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(3.01069, yaw, 0.00001);

  rpyFromQuatMsg(result[12], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(3.01069, yaw, 0.00001);

  rpyFromQuatMsg(result[13], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(3.14159, yaw, 0.00001);

  rpyFromQuatMsg(result[14], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(3.14159, yaw, 0.00001);

  rpyFromQuatMsg(result[15], roll, pitch, yaw);
  ASSERT_NEAR(0.0, roll, 0.00001);
  ASSERT_NEAR(0.0, pitch, 0.00001);
  ASSERT_NEAR(3.14159, yaw, 0.00001);
}


}