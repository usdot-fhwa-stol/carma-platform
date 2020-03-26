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
#include <carma_wm/CARMAWorldModel.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <tf2/LinearMath/Quaternion.h>
#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_wm
{

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

TEST(CARMAWorldModelTest, objectToMapPolygon)
{
  CARMAWorldModel cmw;

  geometry_msgs::Pose pose;
  pose.position.x = 6;
  pose.position.y = 5;
  pose.position.z = 0;

  tf2::Quaternion tf_orientation;
  tf_orientation.setRPY( 0, 0, 1.5708 );

  pose.orientation.x = tf_orientation.getX();
  pose.orientation.y = tf_orientation.getY();
  pose.orientation.z = tf_orientation.getZ();
  pose.orientation.w = tf_orientation.getW();

  geometry_msgs::Vector3 size;
  size.x = 4;
  size.y = 2;
  size.z = 1;

  lanelet::BasicPolygon2d result = cmw.objectToMapPolygon(pose, size);

  ASSERT_NEAR(result[0][0], 5.0, 0.00001);
  ASSERT_NEAR(result[0][1], 7.0, 0.00001);

  ASSERT_NEAR(result[1][0], 7.0, 0.00001);
  ASSERT_NEAR(result[1][1], 7.0, 0.00001);

  ASSERT_NEAR(result[2][0], 7.0, 0.00001);
  ASSERT_NEAR(result[2][1], 3.0, 0.00001);

  ASSERT_NEAR(result[3][0], 5.0, 0.00001);
  ASSERT_NEAR(result[3][1], 3.0, 0.00001);
}
/*
 virtual lanelet::Optional<cav_msgs::RoadwayObstacle>
  toRoadwayObstacle(const cav_msgs::ExternalObject& object) const = 0;
*/
TEST(CARMAWorldModelTest, toRoadwayObstacle)
{
  CARMAWorldModel cmw;
  // Build map
  auto p1 = getPoint(9, 0, 0);
  auto p2 = getPoint(9, 9, 0);
  auto p3 = getPoint(2, 0, 0);
  auto p4 = getPoint(2, 9, 0);
  lanelet::LineString3d right_ls_1(lanelet::utils::getId(), { p1, p2 });
  lanelet::LineString3d left_ls_1(lanelet::utils::getId(), { p3, p4 });
  auto ll_1 = getLanelet(left_ls_1, right_ls_1);
  lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1 }, {});

  geometry_msgs::Pose pose;
  pose.position.x = 6;
  pose.position.y = 5;
  pose.position.z = 0;

  tf2::Quaternion tf_orientation;
  tf_orientation.setRPY( 0, 0, 1.5708 );

  pose.orientation.x = tf_orientation.getX();
  pose.orientation.y = tf_orientation.getY();
  pose.orientation.z = tf_orientation.getZ();
  pose.orientation.w = tf_orientation.getW();

  geometry_msgs::Vector3 size;
  size.x = 4;
  size.y = 2;
  size.z = 1;

  cav_msgs::ExternalObject obj;
  obj.id = 1;
  obj.object_type = cav_msgs::ExternalObject::SMALL_VEHICLE;
  obj.pose.pose = pose;
  obj.velocity.twist.linear.x = 1.0;

  cav_msgs::PredictedState pred;
  auto pred_pose = obj.pose.pose;
  pred_pose.position.y += 1;
  pred.predicted_position = pred_pose;
  pred.predicted_position_confidence = 1.0;

  obj.predictions.push_back(pred);

  // Test with no map set
  ASSERT_THROW(cmw.toRoadwayObstacle(obj), std::invalid_argument);

  // Set map
  cmw.setMap(map);

  // Test object on lanelet
  lanelet::Optional<cav_msgs::RoadwayObstacle> result = cmw.toRoadwayObstacle(obj);

  ASSERT_TRUE(!!result);
  cav_msgs::RoadwayObstacle obs = result.get();

  ASSERT_EQ(obs.object.id, obj.id); // Check that the object was coppied

  ASSERT_EQ(obs.lanelet_id, ll_1.id());

  ASSERT_EQ(obs.connected_vehicle_type.type, cav_msgs::ConnectedVehicleType::NOT_CONNECTED);

  ASSERT_NEAR(obs.cross_track, 0.5, 0.00001);

  ASSERT_NEAR(obs.down_track, 5.0, 0.00001);

  ASSERT_EQ(obs.predicted_lanelet_ids.size(), 1);
  ASSERT_EQ(obs.predicted_lanelet_ids[0], ll_1.id());

  ASSERT_EQ(obs.predicted_lanelet_id_confidences.size(), 1);
  ASSERT_NEAR(obs.predicted_lanelet_id_confidences[0], 0.9, 0.00001);

  ASSERT_EQ(obs.predicted_cross_tracks.size(), 1);
  ASSERT_NEAR(obs.predicted_cross_tracks[0], 0.5, 0.00001);

  ASSERT_EQ(obs.predicted_cross_track_confidences.size(), 1);
  ASSERT_NEAR(obs.predicted_cross_track_confidences[0], 0.9, 0.00001);

  ASSERT_EQ(obs.predicted_down_tracks.size(), 1);
  ASSERT_NEAR(obs.predicted_down_tracks[0], 6.0, 0.00001);

  ASSERT_EQ(obs.predicted_down_track_confidences.size(), 1);
  ASSERT_NEAR(obs.predicted_down_track_confidences[0], 0.9, 0.00001);

  // Alternate object off the roadway
  obj.pose.pose.position.x = 6;
  obj.pose.pose.position.y = 20;
  obj.pose.pose.position.z = 0;

  result = cmw.toRoadwayObstacle(obj);

  ASSERT_FALSE(!!result);

}
}  // namespace carma_wm
