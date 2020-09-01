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
#include <carma_wm/Geometry.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/Attribute.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_wm
{
TEST(GeometryTest, computeCurvature)
{
  // Check curvature of overlapping points
  lanelet::Point2d p1(lanelet::utils::getId(), 0, 0);
  lanelet::Point2d p2(lanelet::utils::getId(), 0, 0);
  lanelet::Point2d p3(lanelet::utils::getId(), 0, 0);

  // ASSERT_NEAR(ex, act, buffer)
  ASSERT_NEAR(0.0, geometry::computeCurvature(p1, p2, p3), 0.0000001);

  // Check flat line
  p1 = lanelet::Point2d(lanelet::utils::getId(), 0, 0);
  p2 = lanelet::Point2d(lanelet::utils::getId(), 1, 0);
  p3 = lanelet::Point2d(lanelet::utils::getId(), 2, 0);

  ASSERT_NEAR(0.0, geometry::computeCurvature(p1, p2, p3), 0.0000001);

  // Check concave down curve
  p1 = lanelet::Point2d(lanelet::utils::getId(), 0, 0);
  p2 = lanelet::Point2d(lanelet::utils::getId(), 1, 1);
  p3 = lanelet::Point2d(lanelet::utils::getId(), 2, 0);

  ASSERT_NEAR(-2.0, geometry::computeCurvature(p1, p2, p3), 0.0000001);

  // Check concave up curve
  p1 = lanelet::Point2d(lanelet::utils::getId(), 0, 0);
  p2 = lanelet::Point2d(lanelet::utils::getId(), 1, -1);
  p3 = lanelet::Point2d(lanelet::utils::getId(), 2, 0);

  ASSERT_NEAR(2.0, geometry::computeCurvature(p1, p2, p3), 0.0000001);
}

TEST(GeometryTest, getLocalCurvatures)
{

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

  std::vector<std::tuple<size_t, std::vector<double>>> curvatures = geometry::getLocalCurvatures(lanelets);
  ASSERT_EQ(1, curvatures.size());
  ASSERT_EQ(0, std::get<0>(curvatures[0]));
  ASSERT_EQ(ll_1.centerline().size() - 2, std::get<1>(curvatures[0]).size());
  ASSERT_NEAR(0.0, std::get<1>(curvatures[0])[0], 0.0000001);
  ASSERT_NEAR(0.0, std::get<1>(curvatures[0])[1], 0.0000001);
  ASSERT_NEAR(0.0, std::get<1>(curvatures[0])[2], 0.0000001);

  ///// Compute two continuos lanelets with 0 curvature

  std::vector<lanelet::Point3d> left_2 = { pl1, pl2 };
  std::vector<lanelet::Point3d> right_2 = { pr1, pr2 };
  auto ll_2 = getLanelet(left_2, right_2);

  std::vector<lanelet::Point3d> left_3 = { pl2, pl3 };
  std::vector<lanelet::Point3d> right_3 = { pr2, pr3 };
  auto ll_3 = getLanelet(left_3, right_3);

  std::vector<lanelet::ConstLanelet> lanelets_2 = { lanelet::utils::toConst(ll_2), lanelet::utils::toConst(ll_3) };

  curvatures = geometry::getLocalCurvatures(lanelets_2);
  ASSERT_EQ(1, curvatures.size());
  ASSERT_EQ(0, std::get<0>(curvatures[0]));
  ASSERT_EQ(ll_2.centerline().size() + ll_3.centerline().size() - 3, std::get<1>(curvatures[0]).size());
  ASSERT_NEAR(0.0, std::get<1>(curvatures[0])[0], 0.0000001);
  ASSERT_NEAR(0.0, std::get<1>(curvatures[0])[1], 0.0000001);
  ASSERT_NEAR(0.0, std::get<1>(curvatures[0])[2], 0.0000001);

  ///// Compute two disjoint lanelets with 0 curvature

  auto p_l1 = getPoint(-1, 0, 0);
  auto p_l2 = getPoint(-1, 1, 0);
  auto p_l3 = getPoint(-1, 2, 0);

  auto p_l4 = getPoint(1, 2, 0);
  auto p_l5 = getPoint(1, 3, 0);
  auto p_l6 = getPoint(1, 4, 0);

  auto p_r1 = getPoint(1, 0, 0);
  auto p_r2 = getPoint(1, 1, 0);
  auto p_r3 = getPoint(1, 2, 0);

  auto p_r4 = getPoint(2, 2, 0);
  auto p_r5 = getPoint(2, 3, 0);
  auto p_r6 = getPoint(2, 4, 0);
  std::vector<lanelet::Point3d> left_4 = { p_l1, p_l2, p_l3 };
  std::vector<lanelet::Point3d> right_4 = { p_r1, p_r2, p_r3 };

  std::vector<lanelet::Point3d> left_5 = { p_l4, p_l5, p_l6 };
  std::vector<lanelet::Point3d> right_5 = { p_r4, p_r5, p_r6 };
  auto ll_4 = getLanelet(left_4, right_4);
  auto ll_5 = getLanelet(left_5, right_5);

  std::vector<lanelet::ConstLanelet> lanelets_3 = { lanelet::utils::toConst(ll_4), lanelet::utils::toConst(ll_5) };

  curvatures = geometry::getLocalCurvatures(lanelets_3);

  ASSERT_EQ(2, curvatures.size());
  ASSERT_EQ(0, std::get<0>(curvatures[0]));
  ASSERT_EQ(1, std::get<0>(curvatures[1]));
  ASSERT_EQ(ll_4.centerline().size() - 2, std::get<1>(curvatures[0]).size());
  ASSERT_EQ(ll_5.centerline().size() - 2, std::get<1>(curvatures[1]).size());
  ASSERT_NEAR(0.0, std::get<1>(curvatures[0])[0], 0.0000001);
  ASSERT_NEAR(0.0, std::get<1>(curvatures[0])[1], 0.0000001);
  ASSERT_NEAR(0.0, std::get<1>(curvatures[0])[2], 0.0000001);
  ASSERT_NEAR(0.0, std::get<1>(curvatures[1])[0], 0.0000001);
  ASSERT_NEAR(0.0, std::get<1>(curvatures[1])[1], 0.0000001);
  ASSERT_NEAR(0.0, std::get<1>(curvatures[1])[2], 0.0000001);

  ///// curved lanelets

  auto pl6_1 = getPoint(0, 0, 0);
  auto pl6_2 = getPoint(-1, 2, 0);
  auto pl6_3 = getPoint(0, 5, 0);
  auto pr6_1 = getPoint(2, 0, 0);
  auto pr6_2 = getPoint(1, 2, 0);
  auto pr6_3 = getPoint(2, 5, 0);
  std::vector<lanelet::Point3d> left_6 = { pl6_1, pl6_2, pl6_3 };
  std::vector<lanelet::Point3d> right_6 = { pr6_1, pr6_2, pr6_3 };
  auto ll_6 = getLanelet(left_6, right_6);
  std::vector<lanelet::ConstLanelet> lanelets_4 = { lanelet::utils::toConst(ll_6) };

  curvatures = geometry::getLocalCurvatures(lanelets_4);

  ASSERT_EQ(1, curvatures.size());
  ASSERT_EQ(0, std::get<0>(curvatures[0]));
  ASSERT_EQ(ll_6.centerline().size() - 2, std::get<1>(curvatures[0]).size());
  ASSERT_NEAR(0.0, std::get<1>(curvatures[0])[0], 0.0000001);
  ASSERT_NEAR(-0.64, std::get<1>(curvatures[0])[1], 0.0000001);
  ASSERT_NEAR(0.0, std::get<1>(curvatures[0])[2], 0.0000001);

  ///// Test exception
  lanelet::Lanelet ll_empty;
  std::vector<lanelet::ConstLanelet> lanelets_5 = { lanelet::utils::toConst(ll_empty) };
  ASSERT_THROW(geometry::getLocalCurvatures(lanelets_5), std::invalid_argument);
}

TEST(GeometryTest, trackPos)
{

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
  TrackPos result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Test end point
  p = lanelet::utils::to2D(ll_1.centerline().back()).basicPoint();
  result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(2.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Test mid point on line
  p = getBasicPoint(0, 1);
  result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Test mid point off line to the right
  p = getBasicPoint(0.5, 1);
  result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Test mid point off line to the left
  p = getBasicPoint(-0.5, 1);
  result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Test point before start
  p = getBasicPoint(-0.5, -0.5);
  result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Test point after end
  p = getBasicPoint(-0.5, 2.5);
  result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(2.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Test exception throw on empty lanelet
  lanelet::Lanelet empty_ll;
  ASSERT_THROW(geometry::trackPos(empty_ll, p);, std::invalid_argument);
}

TEST(GeometryTest, trackPos_point_segment)
{
  ///// Point at start
  TrackPos result = geometry::trackPos(getBasicPoint(0, 0), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point at end
  result = geometry::trackPos(getBasicPoint(0, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point in middle on line
  result = geometry::trackPos(getBasicPoint(0, 0.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point before start but on line
  result = geometry::trackPos(getBasicPoint(0, -0.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point after end but on line
  result = geometry::trackPos(getBasicPoint(0, 1.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point left of start
  result = geometry::trackPos(getBasicPoint(-0.5, 0), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Point right of start
  result = geometry::trackPos(getBasicPoint(0.5, 0), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point left of end
  result = geometry::trackPos(getBasicPoint(-0.5, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Point right of end
  result = geometry::trackPos(getBasicPoint(0.5, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point left of middle
  result = geometry::trackPos(getBasicPoint(-0.5, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Point right of middle
  result = geometry::trackPos(getBasicPoint(0.5, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point right of line before start
  result = geometry::trackPos(getBasicPoint(0.5, -0.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point left of line before start
  result = geometry::trackPos(getBasicPoint(-0.5, -0.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Point right of line after end
  result = geometry::trackPos(getBasicPoint(0.5, 1.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point left of line after end
  result = geometry::trackPos(getBasicPoint(-0.5, 1.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);
}

TEST(GeometryTest, trackPos_line_string)
{
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
  auto result = geometry::matchSegment(p1.basicPoint(), ls);
  ASSERT_NEAR(0.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point at end
  result = geometry::matchSegment(p3.basicPoint(), ls);
  ASSERT_NEAR(2.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point on first segment
  result = geometry::matchSegment(getBasicPoint(0, 0.5), ls);
  ASSERT_NEAR(0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point on second segment
  result = geometry::matchSegment(getBasicPoint(0, 1.5), ls);
  ASSERT_NEAR(1.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point on mid point
  result = geometry::matchSegment(getBasicPoint(0, 1.0), ls);
  ASSERT_NEAR(1.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point just passed mid point
  result = geometry::matchSegment(getBasicPoint(0, 1.01), ls);
  ASSERT_NEAR(1.01, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point before start
  result = geometry::matchSegment(getBasicPoint(0, -0.5), ls);
  ASSERT_NEAR(-0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point just passed end
  result = geometry::matchSegment(getBasicPoint(0, 2.01), ls);
  ASSERT_NEAR(2.01, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to right of start
  result = geometry::matchSegment(getBasicPoint(0.5, 0.0), ls);
  ASSERT_NEAR(0.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point to left of start
  result = geometry::matchSegment(getBasicPoint(-0.5, 0.0), ls);
  ASSERT_NEAR(0.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point to right of mid segment
  result = geometry::matchSegment(getBasicPoint(0.5, 1.01), ls);
  ASSERT_NEAR(1.01, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to left of mid segment
  result = geometry::matchSegment(getBasicPoint(-0.5, 1.01), ls);
  ASSERT_NEAR(1.01, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to right of end
  result = geometry::matchSegment(getBasicPoint(0.5, 2.0), ls);
  ASSERT_NEAR(2.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to left of end
  result = geometry::matchSegment(getBasicPoint(-0.5, 2.0), ls);
  ASSERT_NEAR(2.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to right of beyond end
  result = geometry::matchSegment(getBasicPoint(0.5, 2.4), ls);
  ASSERT_NEAR(2.4, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to left of beyond end
  result = geometry::matchSegment(getBasicPoint(-0.5, 2.4), ls);
  ASSERT_NEAR(2.4, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to right of before start
  result = geometry::matchSegment(getBasicPoint(0.5, -0.5), ls);
  ASSERT_NEAR(-0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point to left of before start
  result = geometry::matchSegment(getBasicPoint(-0.5, -0.5), ls);
  ASSERT_NEAR(-0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point far to right in middle of second segment
  result = geometry::matchSegment(getBasicPoint(5.5, 1.5), ls);
  ASSERT_NEAR(1.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(5.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point far to right in middle of first segment
  result = geometry::matchSegment(getBasicPoint(1.0, 0.5), ls);
  ASSERT_NEAR(0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(1.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point to beyond end of second segment
  result = geometry::matchSegment(getBasicPoint(1.0, 2.5), ls);
  ASSERT_NEAR(2.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(1.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Test exception throw on empty linestring
  lanelet::LineString2d empty_ls;
  ASSERT_THROW(geometry::matchSegment(getBasicPoint(1.0, 2.5), empty_ls.basicLineString()), std::invalid_argument);

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
  result = geometry::matchSegment(getBasicPoint(-0.5, -1.0), ls_);
  ASSERT_NEAR(-1.11803398875, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pa, std::get<1>(result).first);
  ASSERT_EQ(pb, std::get<1>(result).second);

  ///// Point above midpoint of triangle
  result = geometry::matchSegment(getBasicPoint(0.5, 1.5), ls_);
  ASSERT_NEAR(1.565247, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.2236067, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pa, std::get<1>(result).first);
  ASSERT_EQ(pb, std::get<1>(result).second);

  ///// Point below midpoint of triangle
  result = geometry::matchSegment(getBasicPoint(0.5, 0.25), ls_);
  ASSERT_NEAR(0.44721359, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.335410, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pa, std::get<1>(result).first);
  ASSERT_EQ(pb, std::get<1>(result).second);

  ///// Point after end of triangle
  result = geometry::matchSegment(getBasicPoint(1.5, -1.0), ls_);
  ASSERT_NEAR(3.35410196625, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pb, std::get<1>(result).first);
  ASSERT_EQ(pc, std::get<1>(result).second);

  ///// Point below midpoint of triangle to left
  result = geometry::matchSegment(getBasicPoint(0.4, 0.25), ls_);
  ASSERT_NEAR(0.402492, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.245967, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pa, std::get<1>(result).first);
  ASSERT_EQ(pb, std::get<1>(result).second);

  ///// Point below midpoint of triangle to right
  result = geometry::matchSegment(getBasicPoint(0.6, 0.25), ls_);
  ASSERT_NEAR(1.83357598875, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.245967, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pb, std::get<1>(result).first);
  ASSERT_EQ(pc, std::get<1>(result).second);
}

TEST(CARMAWorldModelTest, objectToMapPolygon)
{
  geometry_msgs::Pose pose;
  pose.position.x = 6;
  pose.position.y = 5;
  pose.position.z = 0;

  tf2::Quaternion tf_orientation;
  tf_orientation.setRPY(0, 0, 1.5708);

  pose.orientation.x = tf_orientation.getX();
  pose.orientation.y = tf_orientation.getY();
  pose.orientation.z = tf_orientation.getZ();
  pose.orientation.w = tf_orientation.getW();

  geometry_msgs::Vector3 size;
  size.x = 4;
  size.y = 2;
  size.z = 1;

  lanelet::BasicPolygon2d result = geometry::objectToMapPolygon(pose, size);

  ASSERT_NEAR(result[0][0], 5.0, 0.00001);
  ASSERT_NEAR(result[0][1], 7.0, 0.00001);

  ASSERT_NEAR(result[1][0], 7.0, 0.00001);
  ASSERT_NEAR(result[1][1], 7.0, 0.00001);

  ASSERT_NEAR(result[2][0], 7.0, 0.00001);
  ASSERT_NEAR(result[2][1], 3.0, 0.00001);

  ASSERT_NEAR(result[3][0], 5.0, 0.00001);
  ASSERT_NEAR(result[3][1], 3.0, 0.00001);
}
}  // namespace carma_wm
