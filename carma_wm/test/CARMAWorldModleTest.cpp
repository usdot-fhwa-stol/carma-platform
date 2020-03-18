/*
 * Copyright (C) 2019 LEIDOS.
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
#include <../src/CARMAWorldModel.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_core/Attribute.h>
#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_wm
{
TEST(CARMAWorldModelTest, computeCurvature)
{
  CARMAWorldModel cmw;

  // Check curvature of overlapping points
  lanelet::Point2d p1(lanelet::utils::getId(), 0, 0);
  lanelet::Point2d p2(lanelet::utils::getId(), 0, 0);
  lanelet::Point2d p3(lanelet::utils::getId(), 0, 0);

  // ASSERT_NEAR(ex, act, buffer)
  ASSERT_NEAR(0.0, cmw.computeCurvature(p1, p2, p3), 0.0000001);

  // Check flat line
  p1 = lanelet::Point2d(lanelet::utils::getId(), 0, 0);
  p2 = lanelet::Point2d(lanelet::utils::getId(), 1, 0);
  p3 = lanelet::Point2d(lanelet::utils::getId(), 2, 0);

  ASSERT_NEAR(0.0, cmw.computeCurvature(p1, p2, p3), 0.0000001);

  // Check concave down curve
  p1 = lanelet::Point2d(lanelet::utils::getId(), 0, 0);
  p2 = lanelet::Point2d(lanelet::utils::getId(), 1, 1);
  p3 = lanelet::Point2d(lanelet::utils::getId(), 2, 0);

  ASSERT_NEAR(-2.0, cmw.computeCurvature(p1, p2, p3), 0.0000001);

  // Check concave up curve
  p1 = lanelet::Point2d(lanelet::utils::getId(), 0, 0);
  p2 = lanelet::Point2d(lanelet::utils::getId(), 1, -1);
  p3 = lanelet::Point2d(lanelet::utils::getId(), 2, 0);

  ASSERT_NEAR(2.0, cmw.computeCurvature(p1, p2, p3), 0.0000001);
}

TEST(CARMAWorldModelTest, getSetMap)
{
  CARMAWorldModel cmw;

  std::vector<lanelet::Point3d> left = {
    getPoint(0, 0, 0),
    getPoint(0, 1, 0),
  };
  std::vector<lanelet::Point3d> right = {
    getPoint(1, 0, 0),
    getPoint(1, 1, 0),
  };

  // Ensure that none of the returned pointers are valid if the map has not been set
  ASSERT_FALSE((bool)cmw.getMap());
  ASSERT_FALSE((bool)cmw.getRoute());
  ASSERT_FALSE((bool)cmw.getMapRoutingGraph());

  // Create basic map and verify that the map and routing graph can be build, but the route remains false
  auto ll = getLanelet(left, right);
  auto map = lanelet::utils::createMap({ ll }, {});
  cmw.setMap(std::move(map));

  ASSERT_TRUE((bool)cmw.getMap());
  ASSERT_FALSE((bool)cmw.getRoute());
  ASSERT_TRUE((bool)cmw.getMapRoutingGraph());
}

TEST(CARMAWorldModelTest, getSetRoute)
{
  CARMAWorldModel cmw;

  auto pl1 = getPoint(0, 0, 0);
  auto pl2 = getPoint(0, 1, 0);
  auto pl3 = getPoint(0, 2, 0);
  auto pr1 = getPoint(1, 0, 0);
  auto pr2 = getPoint(1, 1, 0);
  auto pr3 = getPoint(1, 2, 0);
  std::vector<lanelet::Point3d> left_1 = { pl1, pl2 };
  std::vector<lanelet::Point3d> right_1 = { pr1, pr2 };
  auto ll_1 = getLanelet(left_1, right_1);

  std::vector<lanelet::Point3d> left_2 = { pl2, pl3 };
  std::vector<lanelet::Point3d> right_2 = { pr2, pr3 };
  auto ll_2 = getLanelet(left_2, right_2);

  // 1. Confirm all pointers are false (done above)
  // Ensure that none of the returned pointers are valid if the map has not been set
  ASSERT_FALSE((bool)cmw.getMap());
  ASSERT_FALSE((bool)cmw.getRoute());
  ASSERT_FALSE((bool)cmw.getMapRoutingGraph());

  // 2. Build map but do not assign
  // Create basic map and verify that the map and routing graph can be build, but the route remains false
  lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1, ll_2 }, {});

  // 3. Build routing graph but do not assign
  // Build routing graph from map
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
  lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

  // 4. Generate route
  auto optional_route = map_graph->getRoute(ll_1, ll_2);
  ASSERT_TRUE((bool)optional_route);
  lanelet::routing::Route route = std::move(*optional_route);
  LaneletRoutePtr route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
  // 5. Try to set route without map and ensure it passes
  cmw.setRoute(route_ptr);
  // 6. getRoute is true but other pointers are false
  ASSERT_FALSE((bool)cmw.getMap());
  ASSERT_TRUE((bool)cmw.getRoute());
  ASSERT_FALSE((bool)cmw.getMapRoutingGraph());
  // 7. Set map
  cmw.setMap(map);
  // 8. All pointers exist
  ASSERT_TRUE((bool)cmw.getMap());
  ASSERT_TRUE((bool)cmw.getRoute());
  ASSERT_TRUE((bool)cmw.getMapRoutingGraph());
  // 9. Call setRoute again to confirm no errors
  cmw.setRoute(route_ptr);
  // 10. All pointers exist
  ASSERT_TRUE((bool)cmw.getMap());
  ASSERT_TRUE((bool)cmw.getRoute());
  ASSERT_TRUE((bool)cmw.getMapRoutingGraph());
}

/*!
 * \brief Test utility function to generate a lanelet of fixed curvature
 * 
 * \param center_of_curvature The center of the circle that will generate the lanelet
 * \param radius_of_curvature The radius size of the circle that describes
 * \param arc_radians The number of radians to traverse the sweep of the circle
 * \param sample_count  How many points to sample evenly spaced across the arc
 * \return A lanelet matching the input geometry description
 */
std::vector<lanelet::Point3d> 
generate_const_curvature_linestring(
  lanelet::BasicPoint2d center_of_curvature,
  double radius_of_curvature,
  double arc_radians,
  double sample_count
)
{
  std::vector<lanelet::Point3d> out;

  double x, y;
  for (int i = 0; i < sample_count; i++) {
    x = center_of_curvature[0] 
      + radius_of_curvature 
      * std::cos(arc_radians / sample_count * i);
    y = center_of_curvature[1]
      + radius_of_curvature
      * std::sin(arc_radians / sample_count * i);
    
    out.push_back(getPoint(x, y, 0.0));
  }

  return out;
}

TEST(CARMAWorldModelTest, getLocalCurvatures)
{
  CARMAWorldModel cmw;

  auto pl1 = getPoint(-1, 0, 0);
  auto pl2 = getPoint(-1, 1, 0);
  auto pl3 = getPoint(-1, 2, 0);
  auto pr1 = getPoint(1, 0, 0);
  auto pr2 = getPoint(1, 1, 0);
  auto pr3 = getPoint(1, 2, 0);
  std::vector<lanelet::Point3d> left_1 = { pl1, pl2, pl3 };
  std::vector<lanelet::Point3d> right_1 = { pr1, pr2, pr3 };
  auto ll_1 = getLanelet(left_1, right_1);
  std::vector<lanelet::ConstLanelet> lanelets = { lanelet::utils::toConst(ll_1) };

  ///// Compute single lanelet 0 curvature
  std::vector<double> curvatures = cmw.getLocalCurvatures(lanelets);
  ASSERT_EQ(lanelets[0].centerline2d().size(), curvatures.size());
  ASSERT_NEAR(0.0, curvatures[0], 0.0000001);
  ASSERT_NEAR(0.0, curvatures[1], 0.0000001);
  ASSERT_NEAR(0.0, curvatures[2], 0.0000001);

  // Test single lanelet constant curvature
  lanelet::BasicPoint2d origin = getBasicPoint(0.0, 0.0);
  // Curve with centerline at radius 10, curvature 1/10
  std::vector<lanelet::Point3d> left_2 = generate_const_curvature_linestring(origin, 5, M_PI / 2.0, 20); 
  std::vector<lanelet::Point3d> right_2 = generate_const_curvature_linestring(origin, 15, M_PI / 2.0, 20);
  auto ll_2 = getLanelet(left_2, right_2);
  std::vector<lanelet::ConstLanelet> lanelets2 = { lanelet::utils::toConst(ll_2) };

  std::vector<double> curvatures2 = cmw.getLocalCurvatures(lanelets2);
  ASSERT_EQ(lanelets2[0].centerline2d().size(), curvatures2.size());

  double total = 0;
  for (double k : curvatures2) {
    total += k;
  }
  double avg = total/curvatures2.size();
  ASSERT_NEAR(0.1, avg, 0.05);

  // Test single lanelet constant curvature 2
  // Curve with centerline at radius 20, curvature 1/20
  std::vector<lanelet::Point3d> left_3 = generate_const_curvature_linestring(origin, 10, M_PI / 2.0, 10); 
  std::vector<lanelet::Point3d> right_3 = generate_const_curvature_linestring(origin, 30, M_PI / 2.0, 10);
  auto ll_3 = getLanelet(left_3, right_3);
  //lanelet::LineString3d center_ls3(lanelet::utils::getId(), center_3);

  lanelet::LaneletMapPtr map = std::move(lanelet::utils::createMap({ ll_3 }, {}));
  lanelet::utils::overwriteLaneletsCenterline(map);

  std::vector<lanelet::ConstLanelet> lanelets3 = { lanelet::utils::toConst(ll_3) };
  std::vector<double> curvatures3 = cmw.getLocalCurvatures(lanelets3);
  ASSERT_EQ(lanelets3[0].centerline2d().size(), curvatures3.size());

  total = 0;
  for (double k : curvatures3) {
    total += k;
  }
  avg = total/(curvatures3.size());
  ASSERT_NEAR(0.05, avg, 0.05);

  // Test single lanelet constant curvature 3
  // Curve with centerline at radius 1, curvature 1
  std::vector<lanelet::Point3d> left_4 = generate_const_curvature_linestring(origin, 0.5, M_PI / 2.0, 5); 
  std::vector<lanelet::Point3d> right_4 = generate_const_curvature_linestring(origin, 1.5, M_PI / 2.0, 5);
  auto ll_4 = getLanelet(left_4, right_4);

  lanelet::LaneletMapPtr map2 = std::move(lanelet::utils::createMap({ ll_4 }, {}));
  std::cout << "LEFT" << std::endl;
  for (auto pt : ll_4.leftBound3d()) {
    std::cout << pt << std::endl;
  }
  std::cout << "RIGHT" << std::endl;
  for (auto pt : ll_4.rightBound3d()) {
    std::cout << pt << std::endl;
  }
  std::cout << "CENTER" << std::endl;
  for (auto pt : ll_4.centerline2d()) {
    std::cout << pt << std::endl;
  }
  lanelet::utils::overwriteLaneletsCenterline(map2);
  std::cout << "AFTER" << std::endl;
  for (auto pt : ll_4.centerline2d()) {
    std::cout << pt << std::endl;
  }

  std::vector<lanelet::ConstLanelet> lanelets4 = { lanelet::utils::toConst(ll_4) };
  std::vector<double> curvatures4 = cmw.getLocalCurvatures(lanelets4);
  ASSERT_EQ(lanelets4[0].centerline2d().size(), curvatures4.size());

  // Values calculated by hand using same method
  ASSERT_NEAR(0.501551, curvatures4[0], 0.00001);
  ASSERT_NEAR(0.746148, curvatures4[1], 0.00001);
  ASSERT_NEAR(0.987687, curvatures4[2], 0.00001);
  ASSERT_NEAR(0.746139, curvatures4[3], 0.00001);
  ASSERT_NEAR(0.501544, curvatures4[4], 0.00001);

  ///// Test exception
  lanelet::Lanelet ll_empty;
  std::vector<lanelet::ConstLanelet> lanelets_5 = { lanelet::utils::toConst(ll_empty) };
  ASSERT_THROW(cmw.getLocalCurvatures(lanelets_5), std::invalid_argument);
}

TEST(CARMAWorldModelTest, trackPos)
{
  CARMAWorldModel cmw;

  auto pl1 = getPoint(-1, 0, 0);
  auto pl2 = getPoint(-1, 1, 0);
  auto pl3 = getPoint(-1, 2, 0);
  auto pr1 = getPoint(1, 0, 0);
  auto pr2 = getPoint(1, 1, 0);
  auto pr3 = getPoint(1, 2, 0);
  std::vector<lanelet::Point3d> left_1 = { pl1, pl2, pl3 };
  std::vector<lanelet::Point3d> right_1 = { pr1, pr2, pr3 };
  auto ll_1 = getLanelet(left_1, right_1);

  ///// Test start point
  auto p = lanelet::utils::to2D(ll_1.centerline()[0]).basicPoint();
  TrackPos result = cmw.trackPos(ll_1, p);
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Test end point
  p = lanelet::utils::to2D(ll_1.centerline().back()).basicPoint();
  result = cmw.trackPos(ll_1, p);
  ASSERT_NEAR(2.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Test mid point on line
  p = getBasicPoint(0, 1);
  result = cmw.trackPos(ll_1, p);
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Test mid point off line to the right
  p = getBasicPoint(0.5, 1);
  result = cmw.trackPos(ll_1, p);
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Test mid point off line to the left
  p = getBasicPoint(-0.5, 1);
  result = cmw.trackPos(ll_1, p);
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Test point before start
  p = getBasicPoint(-0.5, -0.5);
  result = cmw.trackPos(ll_1, p);
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Test point after end
  p = getBasicPoint(-0.5, 2.5);
  result = cmw.trackPos(ll_1, p);
  ASSERT_NEAR(2.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Test exception throw on empty lanelet
  lanelet::Lanelet empty_ll;
  ASSERT_THROW(cmw.trackPos(empty_ll, p);, std::invalid_argument);
}

TEST(CARMAWorldModelTest, trackPos_point_segment)
{
  CARMAWorldModel cmw;

  ///// Point at start
  TrackPos result = cmw.trackPos(getBasicPoint(0, 0), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point at end
  result = cmw.trackPos(getBasicPoint(0, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point in middle on line
  result = cmw.trackPos(getBasicPoint(0, 0.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point before start but on line
  result = cmw.trackPos(getBasicPoint(0, -0.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point after end but on line
  result = cmw.trackPos(getBasicPoint(0, 1.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point left of start
  result = cmw.trackPos(getBasicPoint(-0.5, 0), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Point right of start
  result = cmw.trackPos(getBasicPoint(0.5, 0), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point left of end
  result = cmw.trackPos(getBasicPoint(-0.5, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Point right of end
  result = cmw.trackPos(getBasicPoint(0.5, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point left of middle
  result = cmw.trackPos(getBasicPoint(-0.5, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Point right of middle
  result = cmw.trackPos(getBasicPoint(0.5, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point right of line before start
  result = cmw.trackPos(getBasicPoint(0.5, -0.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point left of line before start
  result = cmw.trackPos(getBasicPoint(-0.5, -0.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Point right of line after end
  result = cmw.trackPos(getBasicPoint(0.5, 1.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point left of line after end
  result = cmw.trackPos(getBasicPoint(-0.5, 1.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);
}

TEST(CARMAWorldModelTest, trackPos_line_string)
{
  CARMAWorldModel cmw;

  auto p1_ = getPoint(0, 0, 0);
  auto p2_ = getPoint(0, 1, 0);
  auto p3_ = getPoint(0, 2, 0);
  auto p1 = lanelet::utils::to2D(p1_);
  auto p2 = lanelet::utils::to2D(p2_);
  auto p3 = lanelet::utils::to2D(p3_);
  lanelet::Id ls_id = lanelet::utils::getId();
  lanelet::LineString3d ls3d(ls_id, { p1_, p2_, p3_ });
  auto ls2d = lanelet::utils::to2D(ls3d);
  auto ls = ls2d.basicLineString();

  ///// Point at start
  auto result = cmw.matchSegment(p1.basicPoint(), ls);
  ASSERT_NEAR(0.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point at end
  result = cmw.matchSegment(p3.basicPoint(), ls);
  ASSERT_NEAR(2.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point on first segment
  result = cmw.matchSegment(getBasicPoint(0, 0.5), ls);
  ASSERT_NEAR(0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point on second segment
  result = cmw.matchSegment(getBasicPoint(0, 1.5), ls);
  ASSERT_NEAR(1.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point on mid point
  result = cmw.matchSegment(getBasicPoint(0, 1.0), ls);
  ASSERT_NEAR(1.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point just passed mid point
  result = cmw.matchSegment(getBasicPoint(0, 1.01), ls);
  ASSERT_NEAR(1.01, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point before start
  result = cmw.matchSegment(getBasicPoint(0, -0.5), ls);
  ASSERT_NEAR(-0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point just passed end
  result = cmw.matchSegment(getBasicPoint(0, 2.01), ls);
  ASSERT_NEAR(2.01, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to right of start
  result = cmw.matchSegment(getBasicPoint(0.5, 0.0), ls);
  ASSERT_NEAR(0.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point to left of start
  result = cmw.matchSegment(getBasicPoint(-0.5, 0.0), ls);
  ASSERT_NEAR(0.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point to right of mid segment
  result = cmw.matchSegment(getBasicPoint(0.5, 1.01), ls);
  ASSERT_NEAR(1.01, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to left of mid segment
  result = cmw.matchSegment(getBasicPoint(-0.5, 1.01), ls);
  ASSERT_NEAR(1.01, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to right of end
  result = cmw.matchSegment(getBasicPoint(0.5, 2.0), ls);
  ASSERT_NEAR(2.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to left of end
  result = cmw.matchSegment(getBasicPoint(-0.5, 2.0), ls);
  ASSERT_NEAR(2.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to right of beyond end
  result = cmw.matchSegment(getBasicPoint(0.5, 2.4), ls);
  ASSERT_NEAR(2.4, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to left of beyond end
  result = cmw.matchSegment(getBasicPoint(-0.5, 2.4), ls);
  ASSERT_NEAR(2.4, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to right of before start
  result = cmw.matchSegment(getBasicPoint(0.5, -0.5), ls);
  ASSERT_NEAR(-0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point to left of before start
  result = cmw.matchSegment(getBasicPoint(-0.5, -0.5), ls);
  ASSERT_NEAR(-0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point far to right in middle of second segment
  result = cmw.matchSegment(getBasicPoint(5.5, 1.5), ls);
  ASSERT_NEAR(1.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(5.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point far to right in middle of first segment
  result = cmw.matchSegment(getBasicPoint(1.0, 0.5), ls);
  ASSERT_NEAR(0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(1.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point to beyond end of second segment
  result = cmw.matchSegment(getBasicPoint(1.0, 2.5), ls);
  ASSERT_NEAR(2.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(1.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Test exception throw on empty linestring
  lanelet::LineString2d empty_ls;
  ASSERT_THROW(cmw.matchSegment(getBasicPoint(1.0, 2.5), empty_ls.basicLineString()), std::invalid_argument);

  ///// Create concave down triangle
  auto pa_ = getPoint(0, 0, 0);
  auto pb_ = getPoint(0.5, 1, 0);
  auto pc_ = getPoint(1, 0, 0);
  auto pa = lanelet::utils::to2D(pa_).basicPoint();
  auto pb = lanelet::utils::to2D(pb_).basicPoint();
  auto pc = lanelet::utils::to2D(pc_).basicPoint();
  lanelet::LineString3d ls_3d(lanelet::utils::getId(), { pa_, pb_, pc_ });
  auto ls_2d = lanelet::utils::to2D(ls_3d);
  auto ls_ = ls_2d.basicLineString();

  ///// Point before start of triangle
  result = cmw.matchSegment(getBasicPoint(-0.5, -1.0), ls_);
  ASSERT_NEAR(-1.11803398875, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pa, std::get<1>(result).first);
  ASSERT_EQ(pb, std::get<1>(result).second);

  ///// Point above midpoint of triangle
  result = cmw.matchSegment(getBasicPoint(0.5, 1.5), ls_);
  ASSERT_NEAR(1.565247, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.2236067, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pa, std::get<1>(result).first);
  ASSERT_EQ(pb, std::get<1>(result).second);

  ///// Point below midpoint of triangle
  result = cmw.matchSegment(getBasicPoint(0.5, 0.25), ls_);
  ASSERT_NEAR(0.44721359, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.335410, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pa, std::get<1>(result).first);
  ASSERT_EQ(pb, std::get<1>(result).second);

  ///// Point after end of triangle
  result = cmw.matchSegment(getBasicPoint(1.5, -1.0), ls_);
  ASSERT_NEAR(3.35410196625, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pb, std::get<1>(result).first);
  ASSERT_EQ(pc, std::get<1>(result).second);

  ///// Point below midpoint of triangle to left
  result = cmw.matchSegment(getBasicPoint(0.4, 0.25), ls_);
  ASSERT_NEAR(0.402492, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.245967, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pa, std::get<1>(result).first);
  ASSERT_EQ(pb, std::get<1>(result).second);

  ///// Point below midpoint of triangle to right
  result = cmw.matchSegment(getBasicPoint(0.6, 0.25), ls_);
  ASSERT_NEAR(1.83357598875, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.245967, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pb, std::get<1>(result).first);
  ASSERT_EQ(pc, std::get<1>(result).second);
}

TEST(CARMAWorldModelTest, routeTrackPos_point)
{
  CARMAWorldModel cmw;

  ///// Test route exception
  auto p = getBasicPoint(0.5, 0);
  ASSERT_THROW(cmw.routeTrackPos(p), std::invalid_argument);

  ///// Test straight routes
  addStraightRoute(cmw);

  ASSERT_TRUE((bool)cmw.getMap());
  ASSERT_TRUE((bool)cmw.getRoute());
  ASSERT_TRUE((bool)cmw.getMapRoutingGraph());

  ///// Point on route start
  p = getBasicPoint(0.5, 0);
  TrackPos result = cmw.routeTrackPos(p);
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point on route end
  p = getBasicPoint(0.5, 2.0);
  result = cmw.routeTrackPos(p);
  ASSERT_NEAR(2.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point in middle of route
  p = getBasicPoint(0.5, 1.0);
  result = cmw.routeTrackPos(p);
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point before route start
  p = getBasicPoint(0.0, -0.5);
  result = cmw.routeTrackPos(p);
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  // Test disjoint route
  addDisjointRoute(cmw);

  ASSERT_TRUE((bool)cmw.getMap());
  ASSERT_TRUE((bool)cmw.getRoute());
  ASSERT_TRUE((bool)cmw.getMapRoutingGraph());

  ///// Point on route start
  p = getBasicPoint(0.5, 0);
  result = cmw.routeTrackPos(p);
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point end of first lanelet
  p = getBasicPoint(0.5, 1.0);
  result = cmw.routeTrackPos(p);
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-1.0, result.crosstrack, 0.000001);

  ///// Point in middle of second lanelet
  p = getBasicPoint(1.5, 1.5);
  result = cmw.routeTrackPos(p);
  ASSERT_NEAR(1.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point in middle of lane-change lanelet
  p = getBasicPoint(1.5, 0.5);
  result = cmw.routeTrackPos(p);
  ASSERT_NEAR(0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(1.0, result.crosstrack, 0.000001);

  ///// Point to far left of final lanelet
  p = getBasicPoint(0.5, 1.5);
  result = cmw.routeTrackPos(p);
  ASSERT_NEAR(1.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-1.0, result.crosstrack, 0.000001);

  ///// Point at end of final lanelet
  p = getBasicPoint(1.5, 2.0);
  result = cmw.routeTrackPos(p);
  ASSERT_NEAR(2.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point at past end of final lanelet on right
  p = getBasicPoint(2.0, 2.5);
  result = cmw.routeTrackPos(p);
  ASSERT_NEAR(2.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point before middle lanelet
  p = getBasicPoint(1.5, -1.0);
  result = cmw.routeTrackPos(p);
  ASSERT_NEAR(-1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(1.0, result.crosstrack, 0.000001);
}

TEST(CARMAWorldModelTest, routeTrackPos_lanelet)
{
  CARMAWorldModel cmw;

  ///// Test route exception
  lanelet::Lanelet ll;
  ASSERT_THROW(cmw.routeTrackPos(ll), std::invalid_argument);

  ///// Test disjoint routes
  addDisjointRoute(cmw);

  ASSERT_TRUE((bool)cmw.getMap());
  ASSERT_TRUE((bool)cmw.getRoute());
  ASSERT_TRUE((bool)cmw.getMapRoutingGraph());

  auto first_ll = cmw.getRoute()->shortestPath()[0];
  auto second_ll = cmw.getRoute()->shortestPath()[1];
  auto third_ll = cmw.getRoute()->shortestPath()[2];

  ///// First lanelet
  TrackPos result = cmw.routeTrackPos(first_ll);
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Check second lanelet
  result = cmw.routeTrackPos(second_ll);
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(1.0, result.crosstrack, 0.000001);

  ///// Check third lanelet
  result = cmw.routeTrackPos(third_ll);
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);
}

TEST(CARMAWorldModelTest, routeTrackPos_area)
{
  CARMAWorldModel cmw;

  ///// Test route exception
  lanelet::Area a;
  ASSERT_THROW(cmw.routeTrackPos(a), std::invalid_argument);

  ///// Test disjoint routes
  addDisjointRoute(cmw);

  ASSERT_TRUE((bool)cmw.getMap());
  ASSERT_TRUE((bool)cmw.getRoute());
  ASSERT_TRUE((bool)cmw.getMapRoutingGraph());

  auto p1 = getPoint(0.5, 1.5, 0);
  auto p2 = getPoint(0.25, 2.5, 0);
  auto p3 = getPoint(0.75, 2.5, 0);
  lanelet::LineString3d triangle_ls(lanelet::utils::getId(), { p1, p2, p3 });
  lanelet::Area area(lanelet::utils::getId(), { triangle_ls });

  ///// Area
  auto result = cmw.routeTrackPos(area);
  ASSERT_NEAR(1.5, result.first.downtrack, 0.000001);
  ASSERT_NEAR(-1.0, result.first.crosstrack, 0.000001);
  ASSERT_NEAR(2.5, result.second.downtrack, 0.000001);
  ASSERT_NEAR(-1.25, result.second.crosstrack, 0.000001);

  ///// Test exception on empty area
  ASSERT_THROW(cmw.routeTrackPos(a), std::invalid_argument);
}

TEST(CARMAWorldModelTest, getLaneletsBetween)
{
  CARMAWorldModel cmw;

  ///// Test route exception
  ASSERT_THROW(cmw.getLaneletsBetween(0, 1), std::invalid_argument);

  ///// Test straight route
  addStraightRoute(cmw);

  ASSERT_TRUE((bool)cmw.getMap());
  ASSERT_TRUE((bool)cmw.getRoute());
  ASSERT_TRUE((bool)cmw.getMapRoutingGraph());

  ASSERT_EQ(2, cmw.getMap()->laneletLayer.size());
  ASSERT_EQ(2, cmw.getRoute()->laneletMap()->laneletLayer.size());

  ///// Test 0 range
  ASSERT_THROW(cmw.getLaneletsBetween(0, 0), std::invalid_argument);

  ///// Test negative range
  ASSERT_THROW(cmw.getLaneletsBetween(1, 0), std::invalid_argument);

  ///// Test lanelet after range
  auto result = cmw.getLaneletsBetween(2.5, 3.1);
  ASSERT_EQ(0, result.size());

  ///// Test lanelet before range
  result = cmw.getLaneletsBetween(-1.0, -0.1);
  ASSERT_EQ(0, result.size());

  ///// Test 1 lanelet in range
  result = cmw.getLaneletsBetween(-1.0, 0.5);
  ASSERT_EQ(1, result.size());
  ASSERT_NEAR(result[0].id(), cmw.getRoute()->shortestPath().begin()->id(), 0.000001);

  ///// Test both lanelets in range
  result = cmw.getLaneletsBetween(-1.0, 1.5);
  ASSERT_EQ(2, result.size());
  ASSERT_NEAR(result[0].id(), cmw.getRoute()->shortestPath().begin()->id(), 0.000001);
  ASSERT_NEAR(result[1].id(), (cmw.getRoute()->shortestPath().begin() + 1)->id(), 0.000001);

  ///// Test 1 point overlap front
  result = cmw.getLaneletsBetween(-1.0, 0.0);
  ASSERT_EQ(1, result.size());
  ASSERT_NEAR(result[0].id(), cmw.getRoute()->shortestPath().begin()->id(), 0.000001);

  ///// Test 1 point overlap back
  result = cmw.getLaneletsBetween(2.0, 2.5);
  ASSERT_EQ(1, result.size());
  ASSERT_NEAR(result[0].id(), (cmw.getRoute()->shortestPath().begin() + 1)->id(), 0.000001);
}

TEST(CARMAWorldModelTest, getTrafficRules)
{
  CARMAWorldModel cmw;

  ///// Test straight route
  addStraightRoute(cmw);

  auto default_participant = cmw.getTrafficRules();
  ASSERT_TRUE(!!default_participant);  // Verify traffic rules object was returned
  ASSERT_EQ(lanelet::Participants::Vehicle, (*default_participant)->participant());

  default_participant = cmw.getTrafficRules(lanelet::Participants::VehicleCar);
  ASSERT_TRUE(!!default_participant);
  ASSERT_EQ(lanelet::Participants::VehicleCar, (*default_participant)->participant());

  default_participant = cmw.getTrafficRules(lanelet::Participants::VehicleTruck);
  ASSERT_TRUE(!!default_participant);
  ASSERT_EQ(lanelet::Participants::VehicleTruck, (*default_participant)->participant());

  default_participant = cmw.getTrafficRules(lanelet::Participants::Pedestrian);
  ASSERT_TRUE(!!default_participant);
  ASSERT_EQ(lanelet::Participants::Pedestrian, (*default_participant)->participant());

  default_participant = cmw.getTrafficRules(lanelet::Participants::Bicycle);
  ASSERT_TRUE(!!default_participant);
  ASSERT_EQ(lanelet::Participants::Bicycle, (*default_participant)->participant());

  default_participant = cmw.getTrafficRules("fake_person");
  ASSERT_FALSE(!!default_participant);
}
}  // namespace carma_wm
