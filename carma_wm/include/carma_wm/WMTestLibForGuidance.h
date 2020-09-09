#pragma once
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
#include <carma_wm/CARMAWorldModel.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <cav_msgs/RoadwayObstacle.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <tf2/LinearMath/Transform.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <carma_wm/Geometry.h>
/**
 * This is a test library made for guidance unit tests. It includes following in general:
 * - Helper functions to create the world from scratch or extend the world in getGuidanceTestMap()
 * - addObstacle at a specified Cartesian or Trackpos point relative to specified lanelet Id
 * - set route by giving series of lanelet Id in the map (setRouteById)
 * - set speed of entire road (setSpeedLimit)
 * - getGuidanceTestMap gives a simple one way, 3 lane map (25mph speed limit) with one static prebaked obstacle and 
 *      4 lanelets in a lane (if 2 stripes make up one lanelet):
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
 * - NOTE: please look at unit test on example how to use them
 */
namespace carma_wm
{
namespace test
{
using namespace lanelet::units::literals;

enum TestMapOption {DEFAULT_OBSTACLE, NO_OBSTACLE, DEFAULT_SPEED_LIMIT, NO_SPEED_LIMIT};
inline lanelet::Point3d getPoint(double x, double y, double z)
{
  return lanelet::Point3d(lanelet::utils::getId(), x, y, z);
}

inline lanelet::BasicPoint2d getBasicPoint(double x, double y)
{
  return lanelet::utils::to2D(getPoint(x, y, 0.0)).basicPoint();
}

// Defaults to double solid line on left and a solid line on right
inline lanelet::Lanelet getLanelet(lanelet::LineString3d& left_ls, lanelet::LineString3d& right_ls,
                                   const lanelet::Attribute& left_sub_type = lanelet::AttributeValueString::SolidSolid,
                                   const lanelet::Attribute& right_sub_type = lanelet::AttributeValueString::Solid)
{
  left_ls.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
  left_ls.attributes()[lanelet::AttributeName::Subtype] = left_sub_type;

  right_ls.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
  right_ls.attributes()[lanelet::AttributeName::Subtype] = right_sub_type;

  lanelet::Lanelet ll;
  ll.setId(lanelet::utils::getId());
  ll.setLeftBound(left_ls);
  ll.setRightBound(right_ls);

  ll.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::Lanelet;
  ll.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  ll.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
  ll.attributes()[lanelet::AttributeName::OneWay] = "yes";
  ll.attributes()[lanelet::AttributeName::Dynamic] = "no";

  return ll;
}

inline lanelet::Lanelet getLanelet(std::vector<lanelet::Point3d> left, std::vector<lanelet::Point3d> right,
                                   const lanelet::Attribute& left_sub_type = lanelet::AttributeValueString::SolidSolid,
                                   const lanelet::Attribute& right_sub_type = lanelet::AttributeValueString::Solid)
{
  lanelet::LineString3d left_ls(lanelet::utils::getId(), left);

  lanelet::LineString3d right_ls(lanelet::utils::getId(), right);

  return getLanelet(left_ls, right_ls, left_sub_type, right_sub_type);
}

inline lanelet::Lanelet getLanelet(lanelet::Id id, lanelet::LineString3d left, lanelet::LineString3d right,
                                   const lanelet::Attribute& left_sub_type = lanelet::AttributeValueString::SolidSolid,
                                   const lanelet::Attribute& right_sub_type = lanelet::AttributeValueString::Solid)
{
  auto ll = getLanelet(left, right, left_sub_type, right_sub_type);
  ll.setId(id);
  return ll;
}

/**
 * \brief helper function for creating lanelet map for getGuidanceTestMap
 * \param width width of single lanelet, default is 3.7 meters which is US standard
 * \param length length of a single lanelet, default is 25 meters to accomplish 100 meters of full lane
 */
inline lanelet::LaneletMapPtr buildGuidanceTestMap(double width, double length)
{
  std::vector<lanelet::Lanelet> all_lanelets;
  auto pt00 = carma_wm::test::getPoint(0.0,0, 0);
  auto pt01 = carma_wm::test::getPoint(0.0,1 * length, 0);
  auto pt02 = carma_wm::test::getPoint(0.0,2 * length, 0);
  auto pt03 = carma_wm::test::getPoint(0.0,3 * length, 0);
  auto pt04 = carma_wm::test::getPoint(0.0,4 * length, 0);
  auto pt10 = carma_wm::test::getPoint(1 * width,0, 0);
  auto pt11 = carma_wm::test::getPoint(1 * width,1 * length, 0);
  auto pt12 = carma_wm::test::getPoint(1 * width,2 * length, 0);
  auto pt13 = carma_wm::test::getPoint(1 * width,3 * length, 0);
  auto pt14 = carma_wm::test::getPoint(1 * width,4 * length, 0);
  auto pt20 = carma_wm::test::getPoint(2 * width,0, 0);
  auto pt21 = carma_wm::test::getPoint(2 * width,1 * length, 0);
  auto pt22 = carma_wm::test::getPoint(2 * width,2 * length, 0);
  auto pt23 = carma_wm::test::getPoint(2 * width,3 * length, 0);
  auto pt24 = carma_wm::test::getPoint(2 * width,4 * length, 0);
  auto pt30 = carma_wm::test::getPoint(3 * width,0, 0);
  auto pt31 = carma_wm::test::getPoint(3 * width,1 * length, 0);
  auto pt32 = carma_wm::test::getPoint(3 * width,2 * length, 0);
  auto pt33 = carma_wm::test::getPoint(3 * width,3 * length, 0);
  auto pt34 = carma_wm::test::getPoint(3 * width,4 * length, 0);

  lanelet::LineString3d ls00(lanelet::utils::getId(), {pt00,pt01} );
  lanelet::LineString3d ls01(lanelet::utils::getId(), {pt01,pt02} );
  lanelet::LineString3d ls02(lanelet::utils::getId(), {pt02,pt03} );
  lanelet::LineString3d ls03(lanelet::utils::getId(), {pt03,pt04} );

  lanelet::LineString3d ls10(lanelet::utils::getId(), {pt10,pt11} );
  lanelet::LineString3d ls11(lanelet::utils::getId(), {pt11,pt12} );
  lanelet::LineString3d ls12(lanelet::utils::getId(), {pt12,pt13} );
  lanelet::LineString3d ls13(lanelet::utils::getId(), {pt13,pt14} );

  lanelet::LineString3d ls20(lanelet::utils::getId(), {pt20,pt21} );
  lanelet::LineString3d ls21(lanelet::utils::getId(), {pt21,pt22} );
  lanelet::LineString3d ls22(lanelet::utils::getId(), {pt22,pt23} );
  lanelet::LineString3d ls23(lanelet::utils::getId(), {pt23,pt24} );

  lanelet::LineString3d ls30(lanelet::utils::getId(), {pt30,pt31} );
  lanelet::LineString3d ls31(lanelet::utils::getId(), {pt31,pt32} );
  lanelet::LineString3d ls32(lanelet::utils::getId(), {pt32,pt33} );
  lanelet::LineString3d ls33(lanelet::utils::getId(), {pt33,pt34} );

  all_lanelets.push_back(getLanelet(1200, ls00,ls10,lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed));
  all_lanelets.push_back(getLanelet(1201, ls01,ls11,lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed));
  all_lanelets.push_back(getLanelet(1202, ls02,ls12,lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed));
  all_lanelets.push_back(getLanelet(1203, ls03,ls13,lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed));

  all_lanelets.push_back(getLanelet(1210, ls10,ls20,lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Dashed));
  all_lanelets.push_back(getLanelet(1211, ls11,ls21,lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Dashed));
  all_lanelets.push_back(getLanelet(1212, ls12,ls22,lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Dashed));
  all_lanelets.push_back(getLanelet(1213, ls13,ls23,lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Dashed));
  
  all_lanelets.push_back(getLanelet(1220, ls20,ls30,lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Solid));
  all_lanelets.push_back(getLanelet(1221, ls21,ls31,lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Solid));
  all_lanelets.push_back(getLanelet(1222, ls22,ls32,lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Solid));
  all_lanelets.push_back(getLanelet(1223, ls23,ls33,lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Solid));
  
  // Create basic map and verify that the map and routing graph can be build, but the route remains false	
  lanelet::LaneletMapPtr map = lanelet::utils::createMap(all_lanelets, {});
  return map;
}

/**
 * \brief adds a roadway object at the specified cartesian coords
 * \param x coord
 * \param y coord
 * \param cmw CARMAWorldModel shared ptr
 * \param pred_coords vector of std::pair(x,y) predicted coords with 1s interval in the future
 * \param width width of roadway object, default 3 meters
 * \param length length of roadway object, default 3 meters
 * NOTE: x,y is the center of your object
 */
inline void addObstacle(double x, double y, std::shared_ptr<carma_wm::CARMAWorldModel> cmw, std::vector<std::pair<double,double>> pred_coords = {}, double width = 3, double length = 3)
{
    cav_msgs::RoadwayObstacle rwo;	

    tf2::Quaternion tf_orientation;	
    tf_orientation.setRPY(0, 0, 1.5708); // 90 deg

    rwo.object.pose.pose.position.x = x;	
    rwo.object.pose.pose.position.y = y;	
    rwo.object.pose.pose.position.z = 0;	

    rwo.object.pose.pose.orientation.x = tf_orientation.getX();	
    rwo.object.pose.pose.orientation.y = tf_orientation.getY();	
    rwo.object.pose.pose.orientation.z = tf_orientation.getZ();	
    rwo.object.pose.pose.orientation.w = tf_orientation.getW();	

    rwo.object.size.x = width;	
    rwo.object.size.y = length;	
    rwo.object.size.z = 1;	

    int time_stamp = 0;
    std::vector<cav_msgs::PredictedState> pred_states;
    for (auto pred : pred_coords)
    {
        cav_msgs::PredictedState ps;
        time_stamp += 1000;
        ps.header.stamp.nsec = time_stamp;

        ps.predicted_position.position.x = pred.first;	
        ps.predicted_position.position.y = pred.second;	
        ps.predicted_position.position.z = 0;	

        ps.predicted_position.orientation.x = tf_orientation.getX();	
        ps.predicted_position.orientation.y = tf_orientation.getY();	
        ps.predicted_position.orientation.z = tf_orientation.getZ();	
        ps.predicted_position.orientation.w = tf_orientation.getW();	
        pred_states.push_back(ps);
    }

    rwo.object.predictions = pred_states;

    // populate current and predicted lanelet_id, cross_track, downtracks 
    rwo = cmw->toRoadwayObstacle(rwo.object).get();

    std::vector<cav_msgs::RoadwayObstacle> rw_objs = cmw->getRoadwayObjects();	

    rw_objs.push_back(rwo);	

    cmw->setRoadwayObjects(rw_objs);	
}

/**
 * \brief adds a roadway object at the specified trackpos relative to the given lanelet id
 * \param tp TrackPos relative to the given lanelet_id
 * \param lanelet_id Lanelet Id to place the roadway object relative to
 * \param cmw CARMAWorldModel shared ptr
 * \param pred_trackpos_list vector of TrackPos predicted coords with 1s interval in the future. This is relative to the given lanelet_id
 * \param width width of roadway object, default 3 meters
 * \param length length of roadway object, default 3 meters
 * NOTE: This assumes a similar simple shape of the GuidanceTestMap and does not populate cartesian components of the roadway object.
 */
inline void addObstacle(carma_wm::TrackPos tp, lanelet::Id lanelet_id, std::shared_ptr<carma_wm::CARMAWorldModel> cmw, std::vector<carma_wm::TrackPos> pred_trackpos_list = {}, double width = 3, double length = 3)
{
  cav_msgs::RoadwayObstacle rwo;	

  if (!cmw->getMap() || cmw->getMap()->laneletLayer.size() == 0)
  {
    throw std::invalid_argument("Map is not set or does not contain lanelets");
  }
  rwo.connected_vehicle_type.type =
      cav_msgs::ConnectedVehicleType::NOT_CONNECTED;  // TODO No clear way to determine automation state at this time
  
  // get id of specified tp lanelet
  // it assumes similar shape map with getGuidanceTestMap
  auto reference_llt = cmw->getMutableMap()->laneletLayer.get(lanelet_id);
  lanelet::BasicPoint2d object_center = {(reference_llt.leftBound()[0].x() + reference_llt.rightBound()[0].x())/2 + tp.crosstrack,
                                        (reference_llt.leftBound()[0].y() + reference_llt.rightBound()[0].y())/2 + tp.downtrack};    

  auto nearestLanelet = cmw->getMap()->laneletLayer.nearest(object_center, 1)[0]; 
  if (!boost::geometry::within(object_center, nearestLanelet.polygon2d()))
  {
    throw std::invalid_argument("Given trackpos from given lanelet id does land on any lanelet in the map");
  }
  rwo.lanelet_id = nearestLanelet.id();

  rwo.down_track = tp.downtrack;
  rwo.cross_track = tp.crosstrack;

  int time_stamp = 0;
  std::vector<cav_msgs::PredictedState> pred_states;
  for (auto pred_track_pos : pred_trackpos_list)
  {
    // record time intervals
    cav_msgs::PredictedState ps;
    time_stamp += 1000;
    ps.header.stamp.nsec = time_stamp;

    auto ref_llt_pred = cmw->getMutableMap()->laneletLayer.get(lanelet_id);
    lanelet::BasicPoint2d object_center_pred = {(ref_llt_pred.leftBound()[0].x() + ref_llt_pred.rightBound()[0].x())/2 + pred_track_pos.crosstrack,
                                          (ref_llt_pred.leftBound()[0].y() + ref_llt_pred.rightBound()[0].y())/2 + pred_track_pos.downtrack};    

    auto predNearestLanelet = cmw->getMap()->laneletLayer.nearest(object_center_pred, 1)[0]; 
    if (!boost::geometry::within(object_center_pred, predNearestLanelet.polygon2d()))
    {
      std::cerr << "Given pred trackpos from given lanelet id does land on any lanelet in the map" << std::endl;
    }
    rwo.predicted_lanelet_ids.emplace_back(predNearestLanelet.id());
    auto tp_pred = carma_wm::geometry::trackPos(predNearestLanelet,object_center_pred);
    rwo.predicted_cross_tracks.emplace_back(tp_pred.crosstrack);
    rwo.predicted_down_tracks.emplace_back(tp_pred.downtrack);

    pred_states.push_back(ps);
  }
  // add time intervals
  rwo.object.predictions = pred_states;
  std::vector<cav_msgs::RoadwayObstacle> rw_objs = cmw->getRoadwayObjects();	

  rw_objs.push_back(rwo);	

  cmw->setRoadwayObjects(rw_objs);	
}
/**
 * \brief Sets all lanelets in the map the given speed limit
 *
 * \param speed_limit speed limit value to set
 * \param cmw CARMAWorldModel
 *
 * \throw std::invalid_argument if the map is not set, contains no lanelets
 * NOTE: this overwrites existing speed limit regulatory elements in the map
 */
inline void setSpeedLimit (lanelet::Velocity speed_limit, std::shared_ptr<carma_wm::CARMAWorldModel> cmw)
{
  if (!cmw->getMap() || cmw->getMap()->laneletLayer.size() == 0)
  {
    throw std::invalid_argument("setSpeedLimit: Map is not set or does not contain lanelets");
  }
  // remove all speed limit regulatory element and set the new speed limit
  for (auto llt : cmw->getMutableMap()->laneletLayer)
  {
    for (auto regem : llt.regulatoryElements())
    {
      if (regem->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0)
      {
        cmw->getMutableMap()->remove(llt, regem);
      }
    }
    lanelet::DigitalSpeedLimitPtr sl = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::utils::getId(), speed_limit, {llt}, {},
                                                     { lanelet::Participants::VehicleCar }));
    cmw->getMutableMap()->update(llt, sl);
  }
  std::cout << "Set the new speed limit!" << std::endl;
}

/**
 * \brief Sets the route by series of lanelets. 
 *
 * \param lanelets lanelets that you want the route to follow
 * \param cmw CARMAWorldModel shared_ptr
 *
 * \throw std::invalid_argument if fewer than 2 lanelets passed, or the given lanelets are not routable
 */
inline void setRouteByLanelets (std::vector<lanelet::ConstLanelet> lanelets, std::shared_ptr<carma_wm::CARMAWorldModel> cmw)
{
  // Build routing graph from map	
  //auto map_graph = cmw->getMapRoutingGraph();
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
  lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*cmw->getMap(), *traffic_rules);

  // Generate route
  lanelet::Optional<lanelet::routing::Route> optional_route;
  std::vector<lanelet::ConstLanelet> middle_lanelets;
  if (lanelets.size() > 2)
  {
    middle_lanelets = std::vector<lanelet::ConstLanelet>(lanelets.begin() + 1, lanelets.end() - 1);
    optional_route = map_graph->getRouteVia(lanelets[0], middle_lanelets, lanelets[lanelets.size() - 1]);	
  }
  else if (lanelets.size() == 2)
  {
    optional_route = map_graph->getRoute(lanelets[0], lanelets[1]);
  }
  else
  {
    throw lanelet::InvalidInputError("Fewer than 2 lanelets have been passed to setRouteByLanelets, which cannot be a valid route");
  }
  
  if (!optional_route)
  {
    throw lanelet::InvalidInputError("There was an error in routing using the given lanelets in setRouteByLanelets");
  }

  lanelet::routing::Route route = std::move(*optional_route);
  carma_wm::LaneletRoutePtr route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
  cmw->setRoute(route_ptr);

  std::cout << "New route has been set successfully!" << std::endl;
}

/**
 * \brief Sets the route by series of lanelets id. 
 *
 * \param lanelets lanelets that you want the route to follow
 * \param cmw CARMAWorldModel shared_ptr
 *
 * \throw std::invalid_argument if fewer than 2 lanelets passed, or the given lanelets are not routable
 */
inline void setRouteByIds (std::vector<lanelet::Id> lanelet_ids, std::shared_ptr<carma_wm::CARMAWorldModel> cmw)
{
  std::vector<lanelet::ConstLanelet> lanelets;
  for (auto id : lanelet_ids)
  {
      lanelets.push_back(cmw->getMap()->laneletLayer.get(id));
  }
  setRouteByLanelets(lanelets, cmw);
}

/**
 * \brief Gets the CARMAWorldModel for the guidance test map
 *
 * \param width width of single lanelet, default is 3.7 meters which is US standard
 * \param length length of a single lanelet, default is 25 meters to accomplish 100 meters of full lane
 * \param map_options vector of enum map_options. No input is DEFAULT options, but empty vector is NO_OBSTACLE, NO_SPEED_LIMIT options.
 *                    if mix and match, provide each one explicitly e.g. NO_OBSTACLE, DEFAULT_SPEED_LIMIT
 * NOTE: Input 1/4th of full lane length you want to accomplish in length. 
 */
inline std::shared_ptr<carma_wm::CARMAWorldModel> getGuidanceTestMap(double width = 3.7, double length = 25.0, std::vector<TestMapOption> map_options = {DEFAULT_OBSTACLE, DEFAULT_SPEED_LIMIT})
{
  std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();	
  // create the semantic map
  auto map = buildGuidanceTestMap(width, length);

  // set the map, with default routingGraph
  cmw->setMap(map);

  // set default route, from 1200 to 1203 (it will automatically pick the shortest)
  setRouteByIds({1200,1203}, cmw);

  // set the obstacle
  for (auto option : map_options)
  {
    if (option == DEFAULT_OBSTACLE)
    {
      addObstacle(1.5*width, 2.5*length, cmw);  // lanelet_id at 1212
    }
    else if (option == DEFAULT_SPEED_LIMIT)
    {
      setSpeedLimit(25_mph, cmw);   // change speed limit of all lanelets in the map to 25mph
    }
  }
  
  return cmw;
}
}
  //namespace test
} //namespace carma_wm