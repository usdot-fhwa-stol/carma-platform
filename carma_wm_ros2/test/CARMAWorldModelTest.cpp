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

#include <gtest/gtest.h>
#include <iostream>
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <tf2/LinearMath/Quaternion.h>
#include "TestHelpers.hpp"
#include <lanelet2_extension/regulatory_elements/PassingControlLine.h>
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>
#include <rclcpp/rclcpp.hpp>


namespace carma_wm
{

void createTestingWorld(std::vector<lanelet::Lanelet>& llts, lanelet::LaneletMapPtr& map, std::vector<carma_perception_msgs::msg::ExternalObject>& obstacles)
{
  //Note: all params are outputs provided by this function.
  
  /*
  * Create 2x2 lanelets map by hand
  */

  // Linestring points
  std::vector<lanelet::Point3d> pl, pm, pr;
  pl.push_back (carma_wm::getPoint(1, 0 , 0));
  pl.push_back (carma_wm::getPoint(1, 9, 0));
  pl.push_back (carma_wm::getPoint(1, 18, 0));
  pl.push_back (carma_wm::getPoint(10, 27, 0)); //45deg diag ll
  pm.push_back (carma_wm::getPoint(9, 0, 0));
  pm.push_back (carma_wm::getPoint(9, 9, 0));
  pm.push_back (carma_wm::getPoint(9, 18, 0));
  pm.push_back (carma_wm::getPoint(18, 27, 0)); //45deg diag ll
  pr.push_back (carma_wm::getPoint(17, 0, 0));
  pr.push_back (carma_wm::getPoint(17, 9 , 0));
  pr.push_back (carma_wm::getPoint(17, 18, 0));
  pr.push_back (carma_wm::getPoint(26, 27, 0)); //45deg diag ll

  // Unique ids for line strings
  std::vector<lanelet::Id> unique_ids;
  for (int i = 0; i < 9; i ++)
      unique_ids.push_back(lanelet::utils::getId());

  // Create linestrings
  lanelet::LineString3d left_1(unique_ids[0], { pl[0], pl[1] });
  lanelet::LineString3d left_2(unique_ids[2], { pl[1], pl[2] });
  lanelet::LineString3d left_3(unique_ids[6], { pl[2], pl[3] });
  lanelet::LineString3d mid_1(unique_ids[1], { pm[0], pm[1]});
  lanelet::LineString3d mid_2(unique_ids[3], { pm[1], pm[2]});
  lanelet::LineString3d mid_3(unique_ids[7], { pm[2], pm[3]});
  lanelet::LineString3d right_1(unique_ids[4], { pr[0], pr[1]});
  lanelet::LineString3d right_2(unique_ids[5], { pr[1], pr[2]});
  lanelet::LineString3d right_3(unique_ids[8], { pr[2], pr[3]});

  // Create Lanelets
  llts.push_back(carma_wm::getLanelet(left_1, mid_1, lanelet::AttributeValueString::SolidSolid, lanelet::AttributeValueString::Dashed));
  llts.push_back(carma_wm::getLanelet(left_2, mid_2, lanelet::AttributeValueString::SolidSolid, lanelet::AttributeValueString::Dashed));
  llts.push_back(carma_wm::getLanelet(left_3, mid_3, lanelet::AttributeValueString::SolidSolid, lanelet::AttributeValueString::Dashed));
  llts.push_back(carma_wm::getLanelet(mid_1, right_1, lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::SolidSolid));
  llts.push_back(carma_wm::getLanelet(mid_2, right_2, lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::SolidSolid));
  llts.push_back(carma_wm::getLanelet(mid_3, right_3, lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::SolidSolid));

  // Add regualtory element - Passing Control Line
  for (int i = 0; i < 3; i++)
  {
      std::shared_ptr<lanelet::PassingControlLine> pcl(new lanelet::PassingControlLine(lanelet::PassingControlLine::buildData(
          lanelet::utils::getId(), { llts.back().rightBound() }, {lanelet::Participants::Vehicle}, { lanelet::Participants::Vehicle })));
      llts.back().addRegulatoryElement(pcl);
  }

  for (int i = 0; i < 3; i ++)
  {
      std::shared_ptr<lanelet::PassingControlLine> pcl(new lanelet::PassingControlLine(lanelet::PassingControlLine::buildData(
          lanelet::utils::getId(), { llts.back().leftBound() }, {lanelet::Participants::Vehicle}, { lanelet::Participants::Vehicle })));
      llts.back().addRegulatoryElement(pcl);
  }

  // Create lanelet map
  map = lanelet::utils::createMap(llts, {});

  /*
  * Populate the map with external obstacles
  */

  // Offsets are added to x,y = 0,0 origin currently
  // obstacles[0][1][3] are in lane left, obstacles[1][2][3][4] are in lane right:
  std::vector<double> x_offsets = {5, 10.3033, 12.6, 10.01, 17};
  std::vector<double> y_offsets = {9,  9.3033,    4,    14, 20};
  std::vector<double> orients = {0, 0.785, 0, 1.5708, 0}; // 0, 45, 0, 90 degs
  // all objects are same sizes
  geometry_msgs::msg::Vector3 size;
  size.x = 6;
  size.y = 2;
  size.z = 1;

  for (size_t i = 0; i < x_offsets.size(); i++)
  {
      geometry_msgs::msg::Pose pose;
      pose.position.x = x_offsets[i];
      pose.position.y = y_offsets[i];
      pose.position.z = 0;

      tf2::Quaternion tf_orientation;
      tf_orientation.setRPY(0, 0, orients[i]);

      pose.orientation.x = tf_orientation.getX();
      pose.orientation.y = tf_orientation.getY();
      pose.orientation.z = tf_orientation.getZ();
      pose.orientation.w = tf_orientation.getW();

      carma_perception_msgs::msg::ExternalObject obj;
      obj.id = i;
      obj.object_type = carma_perception_msgs::msg::ExternalObject::SMALL_VEHICLE;
      obj.pose.pose = pose;
      obj.velocity.twist.linear.x = 1.0;
      obj.size = size;

      carma_perception_msgs::msg::PredictedState pred;
      auto pred_pose = obj.pose.pose;
      pred_pose.position.y += 1;
      pred.predicted_position = pred_pose;
      pred.predicted_position_confidence = 1.0;

      obj.predictions.push_back(pred);

      obstacles.push_back(obj);
  }

}

TEST(CARMAWorldModelTest, getLane)
{
  /*
  * PREPARE THE TESTING ENVIRONMENT
  */

  carma_wm::CARMAWorldModel cmw;
  std::vector<lanelet::Lanelet> llts;
  lanelet::LaneletMapPtr map;
  std::vector<carma_perception_msgs::msg::ExternalObject> obstacles;

  createTestingWorld(llts, map, obstacles);

  /*
  * TEST
  */
  // Test no map set
  ASSERT_THROW(cmw.getLane(llts[0]), std::invalid_argument);

  // Set map
  cmw.setMap(map); 

  // Convert to RoadwayObstacle format
  std::vector<carma_perception_msgs::msg::RoadwayObstacle> roadway_objects;
  for (auto obj : obstacles)
  {
    roadway_objects.push_back(cmw.toRoadwayObstacle(obj).get());
  }

  // Test left lane
  ASSERT_EQ(cmw.getLane(llts[0], LANE_FULL).size(), 3u);

  // Test right lane
  ASSERT_EQ(cmw.getLane(llts[3], LANE_FULL).size(), 3u);

  // Test lane ahead (always inclde itself)
  ASSERT_EQ(cmw.getLane(llts[2], LANE_AHEAD).size(), 1u);

  // Test lane behind
  ASSERT_EQ(cmw.getLane(llts[5], LANE_BEHIND).size(), 3u);

}

TEST(CARMAWorldModelTest, getNearestObjInLane)
{
  /*
  * PREPARE THE TESTING ENVIRONMENT
  */

  carma_wm::CARMAWorldModel cmw;
  std::vector<lanelet::Lanelet> llts;
  lanelet::LaneletMapPtr map;
  std::vector<carma_perception_msgs::msg::ExternalObject> obstacles;

  createTestingWorld(llts, map, obstacles);

  /*
  * TEST
  */
  // Test no map set
  ASSERT_THROW(cmw.getNearestObjInLane({5,4}), std::invalid_argument);

  // Set map
  cmw.setMap(map); 

  // Convert to RoadwayObstacle format
  std::vector<carma_perception_msgs::msg::RoadwayObstacle> roadway_objects;
  for (auto obj : obstacles)
  {
    roadway_objects.push_back(cmw.toRoadwayObstacle(obj).get());
  }

  // Test with no roadway objects set
  ASSERT_FALSE(!!cmw.getNearestObjInLane({5,4}));

  // Set roadway objects
  cmw.setRoadwayObjects(roadway_objects);

 // Test with a point that is not on the map
  ASSERT_THROW(cmw.getNearestObjInLane({100,205}, LANE_FULL), std::invalid_argument);

  // get closest object's downtrack on left lane, where object is front
  auto tp = cmw.getNearestObjInLane({5,4}, LANE_FULL).get();
  ASSERT_EQ(std::get<0>(tp).downtrack, 5);
  ASSERT_EQ(std::get<1>(tp).lanelet_id, roadway_objects[0].lanelet_id);

  // get closest object's downtrack on left lane, where object is lane changing
  tp = cmw.getNearestObjInLane({5,9.2}, LANE_FULL).get();
  ASSERT_NEAR(std::get<0>(tp).downtrack, 0.1033, 0.001);
  ASSERT_EQ(std::get<1>(tp).lanelet_id, roadway_objects[1].lanelet_id);

  // get closest object's downtrack on right lane, but behind
  tp = cmw.getNearestObjInLane({10.01,16}).get();
  ASSERT_EQ(std::get<0>(tp).downtrack, -2);
  ASSERT_EQ(std::get<1>(tp).lanelet_id, roadway_objects[3].lanelet_id);

  // get closest object's downtrack that is on a lane with an angle
  cmw.setRoadwayObjects({roadway_objects[roadway_objects.size() - 1]});
  tp = cmw.getNearestObjInLane({15,17}, LANE_FULL).get();
  ASSERT_NEAR(std::get<0>(tp).downtrack, 5.242, 0.001);
  ASSERT_EQ(std::get<1>(tp).lanelet_id, roadway_objects[4].lanelet_id);
  
}

TEST(CARMAWorldModelTest, nearestObjectBehindInLane)
{
  /*
  * PREPARE THE TESTING ENVIRONMENT
  */

  carma_wm::CARMAWorldModel cmw;
  std::vector<lanelet::Lanelet> llts;
  lanelet::LaneletMapPtr map;
  std::vector<carma_perception_msgs::msg::ExternalObject> obstacles;
  createTestingWorld(llts, map, obstacles);

  /*
  * TEST
  */
  // Test no map set
  ASSERT_THROW(cmw.nearestObjectBehindInLane({5,4}), std::invalid_argument);

  // Set map
  cmw.setMap(map); 

  // Convert to RoadwayObstacle format
  std::vector<carma_perception_msgs::msg::RoadwayObstacle> roadway_objects;
  for (auto obj : obstacles)
  {
    roadway_objects.push_back(cmw.toRoadwayObstacle(obj).get());
  }

  // Test with no roadway objects set
  ASSERT_FALSE(!!cmw.nearestObjectBehindInLane({5,4}));

  // Set roadway objects
  cmw.setRoadwayObjects(roadway_objects);

  // Test with a point that is not on the map
  ASSERT_THROW(cmw.nearestObjectBehindInLane({100,205}), std::invalid_argument);

  auto tp = cmw.nearestObjectBehindInLane({10.01,16}).get();

  // get closest object's downtrack on right lane, but behind
  ASSERT_EQ(std::get<0>(tp).downtrack, -2);
  ASSERT_EQ(std::get<1>(tp).lanelet_id, roadway_objects[3].lanelet_id);

}

TEST(CARMAWorldModelTest, nearestObjectAheadInLane)
{
  /*
  * PREPARE THE TESTING ENVIRONMENT
  */

  carma_wm::CARMAWorldModel cmw;
  std::vector<lanelet::Lanelet> llts;
  lanelet::LaneletMapPtr map;
  std::vector<carma_perception_msgs::msg::ExternalObject> obstacles;

  createTestingWorld(llts, map, obstacles);

  /*
  * TEST
  */
  // Test no map set
  ASSERT_THROW(cmw.nearestObjectAheadInLane({5,4}), std::invalid_argument);

  // Set map
  cmw.setMap(map); 

  // Convert to RoadwayObstacle format
  std::vector<carma_perception_msgs::msg::RoadwayObstacle> roadway_objects;
  for (auto obj : obstacles)
  {
    roadway_objects.push_back(cmw.toRoadwayObstacle(obj).get());
  }

  // Test with no roadway objects set
  ASSERT_FALSE(!!cmw.nearestObjectAheadInLane({5,4}));

  // Set roadway objects
  cmw.setRoadwayObjects(roadway_objects);

  // Test with a point that is not on the map
  ASSERT_THROW(cmw.nearestObjectAheadInLane({100,205}), std::invalid_argument);

  // get closest object's downtrack on left lane, where object is front
  auto tp = cmw.nearestObjectAheadInLane({5,4}).get();
  ASSERT_EQ(std::get<0>(tp).downtrack, 5);
  ASSERT_EQ(std::get<1>(tp).lanelet_id, roadway_objects[0].lanelet_id);

  // get closest object's downtrack on left lane, where object is lane changing
  tp = cmw.nearestObjectAheadInLane({5,9.2}).get();
  ASSERT_NEAR(std::get<0>(tp).downtrack, 0.1033, 0.001);
  ASSERT_EQ(std::get<1>(tp).lanelet_id, roadway_objects[1].lanelet_id);
}

TEST(CARMAWorldModelTest, getInLaneObjects)
{
  /*
  * PREPARE THE TESTING ENVIRONMENT
  */

  carma_wm::CARMAWorldModel cmw;
  std::vector<lanelet::Lanelet> llts;
  lanelet::LaneletMapPtr map;
  std::vector<carma_perception_msgs::msg::ExternalObject> obstacles;

  createTestingWorld(llts, map, obstacles);

  /*
  * TEST
  */

  // Test with no map set
  ASSERT_THROW(cmw.getInLaneObjects(llts[0]), std::invalid_argument);

  // Set map
  cmw.setMap(map);

  // Convert to RoadwayObstacle format
  std::vector<carma_perception_msgs::msg::RoadwayObstacle> roadway_objects;
  for (auto obj : obstacles)
  {
    roadway_objects.push_back(cmw.toRoadwayObstacle(obj).get());
  }

  // Test with no roadway objects set
  std::vector<carma_perception_msgs::msg::RoadwayObstacle> in_lane_objects;
  in_lane_objects = cmw.getInLaneObjects(llts[0], LANE_FULL);

  ASSERT_EQ(in_lane_objects.size(), 0u);

  // Set roadway objects
  cmw.setRoadwayObjects(roadway_objects);
  
  // Test with lanelet that is not on the map
  auto p1 = getPoint(25, 0, 0);
  auto p2 = getPoint(25, 25, 0);
  auto p3 = getPoint(22, 0, 0);
  auto p4 = getPoint(22, 25, 0);
  lanelet::LineString3d right_ls(lanelet::utils::getId(), { p1, p2 });
  lanelet::LineString3d left_ls(lanelet::utils::getId(), { p3, p4 });
  auto ll = getLanelet(left_ls, right_ls);

  ASSERT_THROW(cmw.getInLaneObjects(ll), std::invalid_argument);

  // check left lane, all should return 2 objects as they are on one lane
  for (int i = 0; i < 3; i++)
  {
    ASSERT_EQ(cmw.getInLaneObjects(llts[i], LANE_FULL).size(), 2u);
  }
  // check right lane, all should return 3 objects as 1 is across linestring
  for (int i = 3; i < 6; i++)
  {
    ASSERT_EQ(cmw.getInLaneObjects(llts[i], LANE_FULL).size(), 4u);
  }
  // check right lane ahead of middle section
  ASSERT_EQ(cmw.getInLaneObjects(llts[4], LANE_AHEAD).size(), 3u);

  // check right lane behind of middle section
  ASSERT_EQ(cmw.getInLaneObjects(llts[4], LANE_BEHIND).size(), 3u);

}

TEST(CARMAWorldModelTest, distToNearestObjInLane)
{
   /*
  * PREPARE THE TESTING ENVIRONMENT
  */

  carma_wm::CARMAWorldModel cmw;
  std::vector<lanelet::Lanelet> llts;
  lanelet::LaneletMapPtr map;
  std::vector<carma_perception_msgs::msg::ExternalObject> obstacles;

  createTestingWorld(llts, map, obstacles);

  /*
  * TEST
  */

  lanelet::BasicPoint2d car_pose = {5,4};

  // Test with no map set
  ASSERT_THROW(cmw.distToNearestObjInLane(car_pose), std::invalid_argument);

  // Set map
  cmw.setMap(map);

  // Convert to RoadwayObstacle format
  std::vector<carma_perception_msgs::msg::RoadwayObstacle> roadway_objects;
  for (auto obj : obstacles)
  {
    roadway_objects.push_back(cmw.toRoadwayObstacle(obj).get());
  }

  // Test with no roadway objects set
  ASSERT_FALSE(!!cmw.distToNearestObjInLane(car_pose));
 
  // Set roadway objects
  cmw.setRoadwayObjects(roadway_objects);
  
  // Test with pose that is not on any lane
  ASSERT_THROW(cmw.distToNearestObjInLane({20, 1}), std::invalid_argument);
  
  // Test closest object
  double result = cmw.distToNearestObjInLane(car_pose).get();
  ASSERT_EQ(result, 4);

  // remove the closest point to get a new closest dist, the 45deg obj
  cmw.setRoadwayObjects(std::vector<carma_perception_msgs::msg::RoadwayObstacle>{roadway_objects.begin() + 1, roadway_objects.end()});
  result = cmw.distToNearestObjInLane(car_pose).get();
  ASSERT_NEAR(result, 4.5, 0.00001);

  // Check an object on a different lane
  car_pose = {12.6, 7}; 
  result = cmw.distToNearestObjInLane(car_pose).get();
  ASSERT_EQ(result, 2);

  // Check if no object is on current lane
  cmw.setRoadwayObjects(std::vector<carma_perception_msgs::msg::RoadwayObstacle>{roadway_objects[roadway_objects.size() - 1]});
  car_pose = {5,4};
  ASSERT_FALSE(!! cmw.distToNearestObjInLane(car_pose));

}

TEST(CARMAWorldModelTest, getIntersectingLanelet)
{
  CARMAWorldModel cmw;
  // Build map
  auto p1 = getPoint(9, 0, 0);
  auto p2 = getPoint(9, 9, 0);
  auto p3 = getPoint(1, 0, 0);
  auto p4 = getPoint(1, 9, 0);
  lanelet::LineString3d right_ls_1(lanelet::utils::getId(), { p1, p2 });
  lanelet::LineString3d left_ls_1(lanelet::utils::getId(), { p3, p4 });
  auto ll_1 = getLanelet(left_ls_1, right_ls_1);
  lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1 }, {});

  geometry_msgs::msg::Pose pose;
  pose.position.x = 9.9;
  pose.position.y = 5;
  pose.position.z = 0;

  tf2::Quaternion tf_orientation;
  tf_orientation.setRPY(0, 0,1.5708); //90 deg

  pose.orientation.x = tf_orientation.getX();
  pose.orientation.y = tf_orientation.getY();
  pose.orientation.z = tf_orientation.getZ();
  pose.orientation.w = tf_orientation.getW();

  // very wide car that is little off the road to right
  geometry_msgs::msg::Vector3 size;
  size.x = 4; 
  size.y = 1.8; // Right on the edge, touching the linestring
  size.z = 1;

  carma_perception_msgs::msg::ExternalObject obj;
  obj.id = 1;
  obj.object_type = carma_perception_msgs::msg::ExternalObject::SMALL_VEHICLE;
  obj.pose.pose = pose;
  obj.velocity.twist.linear.x = 1.0;
  obj.size = size;

  carma_perception_msgs::msg::PredictedState pred;
  auto pred_pose = obj.pose.pose;
  pred_pose.position.y += 1;
  pred.predicted_position = pred_pose;
  pred.predicted_position_confidence = 1.0;

  obj.predictions.push_back(pred);

  // Test with no map set
  ASSERT_THROW(cmw.getIntersectingLanelet(obj), std::invalid_argument);

  // Set map
  cmw.setMap(map);

  // Test object on lanelet
  lanelet::Optional<lanelet::Lanelet> result = cmw.getIntersectingLanelet(obj);

  ASSERT_TRUE(!!result);
  lanelet::Lanelet lanelet = result.get();

  ASSERT_EQ(lanelet, ll_1);

  // Alternate object off the roadway
  obj.pose.pose.position.x = 35;
  obj.pose.pose.position.y = 4;
  obj.pose.pose.position.z = 0;

  result = cmw.getIntersectingLanelet(obj);

  ASSERT_FALSE(!!result);
  
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

TEST(CARMAWorldModelTest, getLaneletsBetween) // HERE
{
  CARMAWorldModel cmw;

  ///// Test route exception
  ASSERT_THROW(cmw.getLaneletsBetween(0, 1), std::invalid_argument);

  ///// Test straight route
  addStraightRoute(cmw);

  ASSERT_TRUE((bool)cmw.getMap());
  ASSERT_TRUE((bool)cmw.getRoute());
  ASSERT_TRUE((bool)cmw.getMapRoutingGraph());

  ASSERT_EQ(2u, cmw.getMap()->laneletLayer.size());
  ASSERT_EQ(2u, cmw.getRoute()->laneletMap()->laneletLayer.size());

  ///// Test 0 range
  auto result = cmw.getLaneletsBetween(0, 0);
  ASSERT_EQ(1u, result.size());
  ASSERT_EQ(result[0].id(), cmw.getRoute()->shortestPath().begin()->id());

  result = cmw.getLaneletsBetween(0, 0, true, true);
  ASSERT_EQ(1u, result.size());
  ASSERT_EQ(result[0].id(), cmw.getRoute()->shortestPath().begin()->id());

  result = cmw.getLaneletsBetween(0, 0, false, true);
  ASSERT_EQ(1u, result.size());
  ASSERT_EQ(result[0].id(), cmw.getRoute()->shortestPath().begin()->id());

  result = cmw.getLaneletsBetween(1.0, 1.0, false, true);
  ASSERT_EQ(2u, result.size());
  ASSERT_EQ(result[0].id(), cmw.getRoute()->shortestPath().begin()->id());
  ASSERT_EQ(result[1].id(),(cmw.getRoute()->shortestPath().begin() + 1)->id());

  result = cmw.getLaneletsBetween(1.0, 1.0, true, false);
  ASSERT_EQ(0u, result.size());
  
  result = cmw.getLaneletsBetween(0.1, 0.2, false, false); // FAIL This fails when the bounds are not inclusive
  ASSERT_EQ(1u, result.size());
  ASSERT_EQ(result[0].id(), cmw.getRoute()->shortestPath().begin()->id());

  result = cmw.getLaneletsBetween(0.1, 0.2, true, false); // FAIL
  ASSERT_EQ(1u, result.size());
  ASSERT_EQ(result[0].id(), cmw.getRoute()->shortestPath().begin()->id());

  ///// Test negative range
  ASSERT_THROW(cmw.getLaneletsBetween(1, 0), std::invalid_argument);

  ///// Test lanelet after range
  result = cmw.getLaneletsBetween(2.5, 3.1);
  ASSERT_EQ(0u, result.size());

  ///// Test lanelet before range
  result = cmw.getLaneletsBetween(-1.0, -0.1);
  ASSERT_EQ(0u, result.size());

  ///// Test 1 lanelet in range
  result = cmw.getLaneletsBetween(-1.0, 0.5);
  ASSERT_EQ(1u, result.size());
  ASSERT_NEAR(result[0].id(), cmw.getRoute()->shortestPath().begin()->id(), 0.000001);

  ///// Test both lanelets in range
  result = cmw.getLaneletsBetween(-1.0, 1.5);
  ASSERT_EQ(2u, result.size());
  ASSERT_NEAR(result[0].id(), cmw.getRoute()->shortestPath().begin()->id(), 0.000001);
  ASSERT_NEAR(result[1].id(), (cmw.getRoute()->shortestPath().begin() + 1)->id(), 0.000001);

  ///// Test 1 point overlap front
  result = cmw.getLaneletsBetween(-1.0, 0.0);
  ASSERT_EQ(1u, result.size());
  ASSERT_NEAR(result[0].id(), cmw.getRoute()->shortestPath().begin()->id(), 0.000001);

  ///// Test 1 point overlap back
  result = cmw.getLaneletsBetween(2.0, 2.5);
  ASSERT_EQ(1u, result.size());
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

TEST(CARMAWorldModelTest, getTrafficRules2)
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

  geometry_msgs::msg::Pose pose;
  pose.position.x = 6;
  pose.position.y = 5;
  pose.position.z = 0;

  tf2::Quaternion tf_orientation;
  tf_orientation.setRPY(0, 0, 1.5708);

  pose.orientation.x = tf_orientation.getX();
  pose.orientation.y = tf_orientation.getY();
  pose.orientation.z = tf_orientation.getZ();
  pose.orientation.w = tf_orientation.getW();

  geometry_msgs::msg::Vector3 size;
  size.x = 4;
  size.y = 2;
  size.z = 1;

  carma_perception_msgs::msg::ExternalObject obj;
  obj.id = 1;
  obj.object_type = carma_perception_msgs::msg::ExternalObject::SMALL_VEHICLE;
  obj.pose.pose = pose;
  obj.velocity.twist.linear.x = 1.0;
  obj.size = size;

  carma_perception_msgs::msg::PredictedState pred;
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
  lanelet::Optional<carma_perception_msgs::msg::RoadwayObstacle> result = cmw.toRoadwayObstacle(obj);

  ASSERT_TRUE(!!result);
  carma_perception_msgs::msg::RoadwayObstacle obs = result.get();

  ASSERT_EQ(obs.object.id, obj.id);  // Check that the object was coppied

  ASSERT_EQ(obs.lanelet_id, ll_1.id());

  ASSERT_EQ(obs.connected_vehicle_type.type, carma_perception_msgs::msg::ConnectedVehicleType::NOT_CONNECTED);

  ASSERT_NEAR(obs.cross_track, 0.5, 0.00001);

  ASSERT_NEAR(obs.down_track, 5.0, 0.00001);

  ASSERT_EQ(obs.predicted_lanelet_ids.size(), 1u);
  ASSERT_EQ(obs.predicted_lanelet_ids[0], ll_1.id());

  ASSERT_EQ(obs.predicted_lanelet_id_confidences.size(), 1u);
  ASSERT_NEAR(obs.predicted_lanelet_id_confidences[0], 0.9, 0.00001);

  ASSERT_EQ(obs.predicted_cross_tracks.size(), 1u);
  ASSERT_NEAR(obs.predicted_cross_tracks[0], 0.5, 0.00001);

  ASSERT_EQ(obs.predicted_cross_track_confidences.size(), 1u);
  ASSERT_NEAR(obs.predicted_cross_track_confidences[0], 0.9, 0.00001);

  ASSERT_EQ(obs.predicted_down_tracks.size(), 1u);
  ASSERT_NEAR(obs.predicted_down_tracks[0], 6.0, 0.00001);

  ASSERT_EQ(obs.predicted_down_track_confidences.size(), 1u);
  ASSERT_NEAR(obs.predicted_down_track_confidences[0], 0.9, 0.00001);

  // Alternate object off the roadway
  obj.pose.pose.position.x = 6;
  obj.pose.pose.position.y = 20;
  obj.pose.pose.position.z = 0;

  result = cmw.toRoadwayObstacle(obj);

  ASSERT_FALSE(!!result);
}

TEST(CARMAWorldModelTest, getLaneletsFromPoint)
{
  carma_wm::CARMAWorldModel cmw;
  std::vector<lanelet::Lanelet> llts;
  lanelet::LaneletMapPtr map;
  std::vector<carma_perception_msgs::msg::ExternalObject> obstacles;

  createTestingWorld(llts, map, obstacles);
  // Test no map set
  ASSERT_THROW(cmw.getLaneletsFromPoint({1,1}), std::invalid_argument);
  // Create a complete map
  test::MapOptions mp(1,1);
  auto cmw_ptr = test::getGuidanceTestMap(mp);
  auto underlyings = cmw_ptr->getLaneletsFromPoint({0.5,0.5});
  ASSERT_EQ(underlyings.size(), 1u);
  ASSERT_EQ(underlyings.front().id(), 1200);

  auto ll_1500 = test::getLanelet(1500, {getPoint(0.0,0.1, 0),getPoint(0.0,1.1, 0)}, 
                         {getPoint(1.0,0.1, 0),getPoint(1.0,1.1, 0)}); // another lanelet the point is in
  cmw_ptr->getMutableMap()->add(ll_1500);
  underlyings = cmw_ptr->getLaneletsFromPoint({0.5,0.5});
  ASSERT_EQ(underlyings.size(), 2u);
  ASSERT_EQ(underlyings.front().id(), 1500);
  ASSERT_EQ(underlyings.back().id(), 1200);
}

TEST(CARMAWorldModelTest, setConfigSpeedLimitTest)
{
  CARMAWorldModel cmw;

  bool flag = false;
  double cL = 24.0;
  ///// Test without user defined config limit
  cmw.setConfigSpeedLimit(cL);
  cmw.setVehicleParticipationType("vehicle:car");

  ASSERT_FALSE(flag);

}

TEST(CARMAWorldModelTest, pointFromRouteTrackPos)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

  auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);

  wm->setMap(map);
  carma_wm::test::setSpeedLimit(20_mph, wm);

  /**
   * Total route length should be 100m
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
  carma_wm::TrackPos pos(0,0);

  auto point = wm->pointFromRouteTrackPos(pos); // test start point
  if (!point) {
    FAIL() << "No point returned";
  }
  auto lanelet = map->laneletLayer.get(1200);
  ASSERT_NEAR((*point).x(), lanelet.centerline().front().x(), 0.001);
  ASSERT_NEAR((*point).y(), lanelet.centerline().front().y(), 0.001);

  pos.downtrack = 40.0;
  point = wm->pointFromRouteTrackPos(pos); // Test end point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1203);
  ASSERT_NEAR((*point).x(), lanelet.centerline().back().x(), 0.001);
  ASSERT_NEAR((*point).y(), lanelet.centerline().back().y(), 0.001);

  pos.downtrack = 12.5;
  point = wm->pointFromRouteTrackPos(pos); // Test mid point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1200);
  ASSERT_NEAR((*point).x(), lanelet.centerline().front().x(), 0.001);
  ASSERT_NEAR((*point).y(), 12.5, 0.001);

  pos.downtrack = 10.0;
  point = wm->pointFromRouteTrackPos(pos); // Test lanelet connection point
  if (!point) {
    FAIL() << "No point returned";
  }
  auto alt_point = wm->routeTrackPos(*point); // Verify the output is equivalent to the inverse function
  lanelet = map->laneletLayer.get(1200);
  ASSERT_NEAR((*point).x(), lanelet.centerline().back().x(), 0.001);
  ASSERT_NEAR((*point).y(), lanelet.centerline().back().y(), 0.001);

  ASSERT_NEAR(alt_point.downtrack, pos.downtrack, 0.001);
  ASSERT_NEAR(alt_point.crosstrack, pos.crosstrack, 0.001);
  
  pos.downtrack = 20.0;
  point = wm->pointFromRouteTrackPos(pos); // Test lanelet connection point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1201);
  ASSERT_NEAR((*point).x(), lanelet.centerline().back().x(), 0.001);
  ASSERT_NEAR((*point).y(), lanelet.centerline().back().y(), 0.001);

  ////////////
  // Evaluate with positive crosstrack
  ////////////
  pos.downtrack = 0;
  pos.crosstrack = 1.0;
  point = wm->pointFromRouteTrackPos(pos); // test start point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1200);
  ASSERT_NEAR((*point).x(), lanelet.centerline().front().x() + 1.0, 0.001);
  ASSERT_NEAR((*point).y(), lanelet.centerline().front().y(), 0.001);

  pos.downtrack = 40.0;
  point = wm->pointFromRouteTrackPos(pos); // Test end point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1203);
  ASSERT_NEAR((*point).x(), lanelet.centerline().back().x() + 1.0, 0.001);
  ASSERT_NEAR((*point).y(), lanelet.centerline().back().y(), 0.001);

  pos.downtrack = 12.5;
  point = wm->pointFromRouteTrackPos(pos); // Test mid point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1200);
  ASSERT_NEAR((*point).x(), lanelet.centerline().front().x() + 1.0, 0.001);
  ASSERT_NEAR((*point).y(), 12.5, 0.001);

  pos.downtrack = 10.0;
  point = wm->pointFromRouteTrackPos(pos); // Test lanelet connection point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1200);
  ASSERT_NEAR((*point).x(), lanelet.centerline().back().x() + 1.0, 0.001);
  ASSERT_NEAR((*point).y(), lanelet.centerline().back().y(), 0.001);
  
  pos.downtrack = 20.0;
  point = wm->pointFromRouteTrackPos(pos); // Test lanelet connection point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1201);
  ASSERT_NEAR((*point).x(), lanelet.centerline().back().x() + 1.0, 0.001);
  ASSERT_NEAR((*point).y(), lanelet.centerline().back().y(), 0.001);

  ////////////
  // Evaluate with negative crosstrack
  ////////////
  pos.downtrack = 0;
  pos.crosstrack = -1.0;
  point = wm->pointFromRouteTrackPos(pos); // test start point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1200);
  ASSERT_NEAR((*point).x(), lanelet.centerline().front().x() - 1.0, 0.001);
  ASSERT_NEAR((*point).y(), lanelet.centerline().front().y(), 0.001);

  pos.downtrack = 40.0;
  point = wm->pointFromRouteTrackPos(pos); // Test end point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1203);
  ASSERT_NEAR((*point).x(), lanelet.centerline().back().x() - 1.0, 0.001);
  ASSERT_NEAR((*point).y(), lanelet.centerline().back().y(), 0.001);

  pos.downtrack = 12.5;
  point = wm->pointFromRouteTrackPos(pos); // Test mid point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1200);
  ASSERT_NEAR((*point).x(), lanelet.centerline().front().x() - 1.0, 0.001);
  ASSERT_NEAR((*point).y(), 12.5, 0.001);

  pos.downtrack = 10.0;
  point = wm->pointFromRouteTrackPos(pos); // Test lanelet connection point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1200);
  ASSERT_NEAR((*point).x(), lanelet.centerline().back().x() - 1.0, 0.001);
  ASSERT_NEAR((*point).y(), lanelet.centerline().back().y(), 0.001);
  
  pos.downtrack = 20.0;
  point = wm->pointFromRouteTrackPos(pos); // Test lanelet connection point
  if (!point) {
    FAIL() << "No point returned";
  }
  lanelet = map->laneletLayer.get(1201);
  ASSERT_NEAR((*point).x(), lanelet.centerline().back().x() - 1.0, 0.001);
  ASSERT_NEAR((*point).y(), lanelet.centerline().back().y(), 0.001);
}

TEST(CARMAWorldModelTest, sampleRoutePoints)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

  auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);

  wm->setMap(map);
  carma_wm::test::setSpeedLimit(20_mph, wm);

  /**
   * Total route length should be 100m
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
  std::vector<lanelet::BasicPoint2d> points = wm->sampleRoutePoints(0, 10.5, 1);

  size_t i = 0;
  for (auto p : points) {
    ASSERT_NEAR(p.x(), 1.85, 0.001);
    if (i != points.size() - 1) {
      ASSERT_NEAR(p.y(), (double)i, 0.001);   
    }
    else {
      ASSERT_NEAR(p.y(), 10.5, 0.001);   
    }
    i++; 
  }
}

TEST(CARMAWorldModelTest, getTrafficSignalId)
{
  CARMAWorldModel cmw;
  uint16_t intersection_id=1;
  uint8_t signal_group_id=1;
  cmw.sim_.intersection_id_to_regem_id_[intersection_id] = 1001;
  cmw.sim_.signal_group_to_traffic_light_id_[signal_group_id] = 1000;

  EXPECT_EQ(cmw.getTrafficSignalId(intersection_id, signal_group_id), 1000); 
}

TEST(CARMAWorldModelTest, processSpatFromMsg)
{
  CARMAWorldModel cmw;
  auto pl1 = carma_wm::getPoint(0, 0, 0);
  auto pl2 = carma_wm::getPoint(0, 1, 0);
  auto pl3 = carma_wm::getPoint(0, 2, 0);
  auto pr1 = carma_wm::getPoint(1, 0, 0);
  auto pr2 = carma_wm::getPoint(1, 1, 0);
  auto pr3 = carma_wm::getPoint(1, 2, 0);
  std::vector<lanelet::Point3d> left_1 = { pl1, pl2, pl3 };
  std::vector<lanelet::Point3d> right_1 = { pr1, pr2, pr3 };
  auto ll_1 = carma_wm::getLanelet(left_1, right_1, lanelet::AttributeValueString::SolidDashed,lanelet::AttributeValueString::Dashed);
  lanelet::Id traffic_light_id = lanelet::utils::getId();
  lanelet::LineString3d virtual_stop_line(lanelet::utils::getId(), {pl2, pr2});
  // Creat passing control line for solid dashed line
  std::shared_ptr<lanelet::CarmaTrafficSignal> traffic_light(new lanelet::CarmaTrafficSignal(lanelet::CarmaTrafficSignal::buildData(traffic_light_id, { virtual_stop_line }, { ll_1 }, { ll_1 })));
  traffic_light->revision_ = 0;
  ll_1.addRegulatoryElement(traffic_light);
  auto map = lanelet::utils::createMap({ ll_1 }, {});
  map->add(traffic_light);
  cmw.setMap(std::move(map));

  uint16_t intersection_id=1;
  uint8_t signal_group_id=1;
  cmw.sim_.intersection_id_to_regem_id_[intersection_id] = 1001;
  cmw.sim_.signal_group_to_traffic_light_id_[signal_group_id] = traffic_light_id;

  // create sample SPAT.msg and fill its entries
  carma_v2x_msgs::msg::SPAT spat;
  carma_v2x_msgs::msg::IntersectionState state;
  state.id.id = 1;
  state.revision = 0;
  carma_v2x_msgs::msg::MovementState movement;
  movement.signal_group = 1;
  carma_v2x_msgs::msg::MovementEvent event;

  // call the processSpatFromMsg with that msg 1
  event.event_state.movement_phase_state = 5;
  event.timing.min_end_time = 20;
  movement.movement_event_list.push_back(event);
  state.movement_list.push_back(movement);
  spat.intersection_state_list.push_back(state);
  cmw.processSpatFromMsg(spat);
  auto lights1 = cmw.getMutableMap()->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
  // default
  EXPECT_EQ(lanelet::time::durationFromSec(43), lights1[0]->fixed_cycle_duration);

  // call the processSpatFromMsg with that msg 2
  event.event_state.movement_phase_state = 7;
  event.timing.min_end_time = 24;
  movement.movement_event_list[0] = event;
  state.movement_list[0] = movement;
  spat.intersection_state_list[0] = state;
  cmw.processSpatFromMsg(spat);
  lights1 = cmw.getMutableMap()->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
  // nothing changed
  EXPECT_EQ(lanelet::time::durationFromSec(43), lights1[0]->fixed_cycle_duration);

  // call the processSpatFromMsg with that msg 3a
  event.event_state.movement_phase_state = 3;
  event.timing.min_end_time = 44.5;
  movement.movement_event_list[0] = event;
  state.movement_list[0] = movement;
  spat.intersection_state_list[0] = state;
  cmw.processSpatFromMsg(spat);
  lights1 = cmw.getMutableMap()->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
  // partial state 7
  EXPECT_EQ(lanelet::time::durationFromSec(44.0), lights1[0]->fixed_cycle_duration);

    // call the processSpatFromMsg with that msg 3b
  event.event_state.movement_phase_state = 3;
  event.timing.min_end_time = 45.0;
  movement.movement_event_list[0] = event;
  state.movement_list[0] = movement;
  spat.intersection_state_list[0] = state;
  cmw.processSpatFromMsg(spat);
  lights1 = cmw.getMutableMap()->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
  // partial state 7
  EXPECT_EQ(lanelet::time::durationFromSec(44.0), lights1[0]->fixed_cycle_duration);

  // call the processSpatFromMsg with that msg 4.a
  event.event_state.movement_phase_state = 5;
  event.timing.min_end_time = 65;
  movement.movement_event_list[0] = event;
  state.movement_list[0] = movement;
  spat.intersection_state_list[0] = state;
  cmw.processSpatFromMsg(spat);
  lights1 = cmw.getMutableMap()->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
  // partial state 3
  EXPECT_EQ(lanelet::time::durationFromSec(44.5), lights1[0]->fixed_cycle_duration); // NOTE: 44.5 here instead of 45 because of the intentional subtraction of 0.5s inside the implementation
  
  // call the processSpatFromMsg with that msg 4.b
  event.event_state.movement_phase_state = 5;
  event.timing.min_end_time = 65.001;
  movement.movement_event_list[0] = event;
  state.movement_list[0] = movement;
  spat.intersection_state_list[0] = state;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ros2"), "Input: Duplicate, so ignore");
  cmw.processSpatFromMsg(spat);
  lights1 = cmw.getMutableMap()->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
  // and query the regem again to check if its entries are updated, by checking revision or getState or predictState etc
  EXPECT_EQ(lanelet::time::durationFromSec(44.5), lights1[0]->fixed_cycle_duration); // NOTE: 44.5 here instead of 45 because of the intentional subtraction of 0.5s inside the implementation

  // call the processSpatFromMsg with that msg 5
  event.event_state.movement_phase_state = 3;
  event.timing.min_end_time = 66;
  movement.movement_event_list[0] = event;
  state.movement_list[0] = movement;
  spat.intersection_state_list[0] = state;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ros2"), "Input: First cycle set. This is technically new cycle, but this info is not counted towards it due to inconvenience");
  cmw.processSpatFromMsg(spat);
  lights1 = cmw.getMutableMap()->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
  // same duration, but counter set to 0
  EXPECT_EQ(lanelet::time::durationFromSec(45), lights1[0]->fixed_cycle_duration);

  // call the processSpatFromMsg with that msg 6
  // new cycle, but will be counted as old cycle due to same light
  event.event_state.movement_phase_state = 5;
  event.timing.min_end_time = 67;
  movement.movement_event_list[0] = event;
  state.movement_list[0] = movement;
  spat.intersection_state_list[0] = state;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ros2"), "Input: New cycle, but cycle duration is same due to shifting");
  cmw.processSpatFromMsg(spat);
  lights1 = cmw.getMutableMap()->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();

  EXPECT_EQ(lanelet::time::durationFromSec(45), lights1[0]->fixed_cycle_duration);

  // call the processSpatFromMsg with that msg 7
  event.event_state.movement_phase_state = 7;
  event.timing.min_end_time = 68;
  movement.movement_event_list[0] = event;
  state.movement_list[0] = movement;
  spat.intersection_state_list[0] = state;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ros2"),"Input: New partial cycle, yellow reduced");
  cmw.processSpatFromMsg(spat);
  lights1 = cmw.getMutableMap()->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
  // and query the regem again to check if its entries are updated, by checking revision or getState or predictState etc
  EXPECT_EQ(lanelet::time::durationFromSec(43), lights1[0]->fixed_cycle_duration);

  // call the processSpatFromMsg with that msg 8
  event.event_state.movement_phase_state = 3;
  event.timing.min_end_time = 69.000;
  movement.movement_event_list[0] = event;
  state.movement_list[0] = movement;
  spat.intersection_state_list[0] = state;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ros2"), "Input: New partial cycle, green reduced");
  cmw.processSpatFromMsg(spat);
  lights1 = cmw.getMutableMap()->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
  // and query the regem again to check if its entries are updated, by checking revision or getState or predictState etc
  EXPECT_EQ(lanelet::time::durationFromSec(41), lights1[0]->fixed_cycle_duration);

  // call the processSpatFromMsg with that msg 9
  event.event_state.movement_phase_state = 5;
  event.timing.min_end_time = 70;
  movement.movement_event_list[0] = event;
  state.movement_list[0] = movement;
  spat.intersection_state_list[0] = state;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ros2"), "Input: New full cycle, yellow reduced");
  cmw.processSpatFromMsg(spat);
  lights1 = cmw.getMutableMap()->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
  // and query the regem again to check if its entries are updated, by checking revision or getState or predictState etc
  EXPECT_EQ(lanelet::time::durationFromSec(22), lights1[0]->fixed_cycle_duration);

  // call the processSpatFromMsg with that msg 10
  event.event_state.movement_phase_state = 7;
  event.timing.min_end_time = 71;
  movement.movement_event_list[0] = event;
  state.movement_list[0] = movement;
  spat.intersection_state_list[0] = state;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ros2"), "Input: New full cycle, red reduced");
  cmw.processSpatFromMsg(spat);
  lights1 = cmw.getMutableMap()->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
  // and query the regem again to check if its entries are updated, by checking revision or getState or predictState etc
  EXPECT_EQ(lanelet::time::durationFromSec(3), lights1[0]->fixed_cycle_duration);

}

TEST(CARMAWorldModelTest, getSignalsAlongRoute)
{
  carma_wm::CARMAWorldModel cmw;
  lanelet::LaneletMapPtr map;
  // Create a complete map
  test::MapOptions mp(1,1);
  auto cmw_ptr = test::getGuidanceTestMap(mp);
  
  auto pl2 = carma_wm::getPoint(0, 1, 0);
  auto pl3 = carma_wm::getPoint(0, 2, 0);
  auto pr2 = carma_wm::getPoint(1, 1, 0);
  auto pr3 = carma_wm::getPoint(1, 2, 0);

  lanelet::Id traffic_light_id1 = lanelet::utils::getId();
  lanelet::Id traffic_light_id2 = lanelet::utils::getId();
  lanelet::LineString3d virtual_stop_line1(lanelet::utils::getId(), {pl2, pr2});
  std::shared_ptr<lanelet::CarmaTrafficSignal> traffic_light1(new lanelet::CarmaTrafficSignal(lanelet::CarmaTrafficSignal::buildData(traffic_light_id1, { virtual_stop_line1 }, { cmw_ptr->getMutableMap()->laneletLayer.get(1200) },  { cmw_ptr->getMutableMap()->laneletLayer.get(1200) })));
  lanelet::LineString3d virtual_stop_line2(lanelet::utils::getId(), {pl3, pr3});
  std::shared_ptr<lanelet::CarmaTrafficSignal> traffic_light2(new lanelet::CarmaTrafficSignal(lanelet::CarmaTrafficSignal::buildData(traffic_light_id2, { virtual_stop_line2 }, { cmw_ptr->getMutableMap()->laneletLayer.get(1201) },  { cmw_ptr->getMutableMap()->laneletLayer.get(1201) })));

  cmw_ptr->getMutableMap()->update(cmw_ptr->getMutableMap()->laneletLayer.get(1200), traffic_light1);
  cmw_ptr->getMutableMap()->update(cmw_ptr->getMutableMap()->laneletLayer.get(1201), traffic_light2);
  carma_wm::test::setRouteByIds({ 1200, 1201, 1202}, cmw_ptr);

  auto lights = cmw_ptr->getSignalsAlongRoute({0.5, 0});
  
  EXPECT_EQ(lights.size(), 2u);
  EXPECT_EQ(lights[0]->id(), traffic_light_id1);
  EXPECT_EQ(lights[1]->id(), traffic_light_id2);

}

TEST(CARMAWorldModelTest, getIntersectionAlongRoute)
{
  lanelet::Id id{1200};
  // intersection id
  lanelet::Id int_id{1};
  lanelet::Point3d p1, p2, p3, p4, p5, p6;
  lanelet::LineString3d ls1, ls2, ls3, ls4, ls5, ls6;
  lanelet::Lanelet ll1, ll2, ll3;

  p1 = lanelet::Point3d(++id, 0., 1., 1.);
  p2 = lanelet::Point3d(++id, 1., 1., 1.);
  p3 = lanelet::Point3d(++id, 0., 0., 0.);
  p4 = lanelet::Point3d(++id, 1., 0., 0.);

  ls1 = lanelet::LineString3d(++id, lanelet::Points3d{p1, p2});
  ls2 = lanelet::LineString3d(++id, lanelet::Points3d{p3, p4});
  ls3 = lanelet::LineString3d(++id, lanelet::Points3d{p3, p1});


  ll1 = lanelet::Lanelet(++id, ls1, ls2);
  ll2 = lanelet::Lanelet(++id, ls2, ls3);

  
  carma_wm::CARMAWorldModel cmw;
  lanelet::LaneletMapPtr map;
  // Create a complete map
  test::MapOptions mp(1,1);
  auto cmw_ptr = test::getGuidanceTestMap(mp);

  std::shared_ptr<lanelet::AllWayStop> row = lanelet::AllWayStop::make(int_id, lanelet::AttributeMap(), {{ll1, ls1}});
  cmw_ptr->getMutableMap()->update(cmw_ptr->getMutableMap()->laneletLayer.get(1200), row);

  carma_wm::test::setRouteByIds({ 1200, 1201, 1202}, cmw_ptr);

  auto ints = cmw_ptr->getIntersectionsAlongRoute({0.5, 0});
  
  EXPECT_EQ(ints.size(), 1u);
  EXPECT_EQ(ints[0]->id(), int_id);

}

TEST(CARMAWorldModelTest, checkIfSeenBeforeMovementState)
{
  carma_wm::CARMAWorldModel cmw;
  auto system_now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count() + 1.0;

  cmw.sim_.traffic_signal_states_[13][15].push_back(std::make_pair(lanelet::time::timeFromSec(system_now + 1.0), lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN));
  cmw.sim_.traffic_signal_start_times_[13][15].push_back(lanelet::time::timeFromSec(system_now));

  boost::posix_time::ptime min_end_time_dynamic = lanelet::time::timeFromSec(system_now + 1.0);
  auto received_state_dynamic=lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN;
  int mov_id=13;
  int mov_signal_group=15;

  ASSERT_EQ(cmw.check_if_seen_before_movement_state(min_end_time_dynamic,received_state_dynamic,mov_id,mov_signal_group), 1);

  min_end_time_dynamic=lanelet::time::timeFromSec(system_now);
  received_state_dynamic= lanelet::CarmaTrafficSignalState::PROTECTED_CLEARANCE;
  mov_id=13;
  mov_signal_group=15;

  ASSERT_EQ(cmw.check_if_seen_before_movement_state(min_end_time_dynamic,received_state_dynamic,mov_id,mov_signal_group), 0);
}
}  // namespace carma_wm
