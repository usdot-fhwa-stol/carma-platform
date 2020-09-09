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
/**
 * This is a test library made for guidance unit tests. It includes following in general:
 * - Helper functions to create the world from scratch or extend the world in getGuidanceTestMap()
 * - addObstacle at a specified Cartesian or Trackpos point relative to specified lanelet Id
 * - set route by giving series of lanelet Id in the map (setRouteById)
 * - getGuidanceTestMap gives a simple one way, 3 lane map with one prebaked obstacle and 
 *      4 lanelets in a lane (if 2 stripes make up one lanelet):
 *
 *        |1203|1213|1223|
 *        | _  _  _  _  _|
 *        |1202| Ob |1222|
 *        | _  _  _  _  _|
 *        |1201|1211|1221|    num   = lanelet id hardcoded for easier testing
 *        | _  _  _  _  _|    |     = lane lines
 *        |1200|1210|1220|    - - - = Lanelet boundary
 *        |              |    O     = Obstacle
 *        ****************
 *           START_LINE
 */
namespace carma_wm
{
namespace test
{
enum ObstacleOption {DEFAULT_OBSTACLE, NO_OBSTACLE};
inline lanelet::Point3d getPoint(double x, double y, double z)
{
  return lanelet::Point3d(lanelet::utils::getId(), x, y, z);
}

inline lanelet::BasicPoint2d getBasicPoint(double x, double y)
{
  return lanelet::utils::to2D(getPoint(x, y, 0.0)).basicPoint();
}

// Defaults to double solid line on left and double solid line on right
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

inline lanelet::Lanelet getLanelet(lanelet::Id id, std::vector<lanelet::Point3d> left, std::vector<lanelet::Point3d> right,
                                   const lanelet::Attribute& left_sub_type = lanelet::AttributeValueString::SolidSolid,
                                   const lanelet::Attribute& right_sub_type = lanelet::AttributeValueString::Solid)
{
  auto ll = getLanelet(left, right, left_sub_type, right_sub_type);
  ll.setId(id);
  return ll;
}

inline std::shared_ptr<carma_wm::CARMAWorldModel> getGuidanceTestMap(double width = 1.0, double length = 1.0, ObstacleOption ob_option = DEFAULT_OBSTACLE)
{
    std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();	
    // create the lanelets
    std::vector<lanelet::Lanelet> all_lanelets;
    for (int i = 0; i < 4; i ++)
    {
        
        lanelet::Id left_id = 1200 + i;
        lanelet::Id mid_id = 1210 + i;
        lanelet::Id right_id = 1220 + i;

        auto ll_left = carma_wm::test::getLanelet(left_id, {carma_wm::test::getPoint(0.0,i * length, 0), carma_wm::test::getPoint(0.0, (i + 1)* length, 0)},        //left_ls
                                                {carma_wm::test::getPoint(width, i* length, 0), carma_wm::test::getPoint( width, (i + 1)* length, 0)},              //right_ls
                                                lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed);           
        auto ll_middle = carma_wm::test::getLanelet(mid_id, {carma_wm::test::getPoint(width,i * length, 0), carma_wm::test::getPoint(width, (i + 1)* length, 0)},          //left_ls            
                                                {carma_wm::test::getPoint(2.0 * width, i* length, 0), carma_wm::test::getPoint( 2.0 * width, (i + 1)* length, 0)},        //right_ls   
                                                lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Dashed);
        auto ll_right = carma_wm::test::getLanelet(right_id, {carma_wm::test::getPoint(2* width,i * length, 0), carma_wm::test::getPoint( 2* width, (i + 1)* length, 0)}, //left_ls
                                                {carma_wm::test::getPoint(3.0 * width, i* length, 0), carma_wm::test::getPoint( 3.0 * width, (i + 1)* length, 0)},        //right_ls
                                                lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Solid);
        all_lanelets.push_back(ll_left);
        all_lanelets.push_back(ll_middle);
        all_lanelets.push_back(ll_right);

    }
    
    // Create basic map and verify that the map and routing graph can be build, but the route remains false	
    lanelet::LaneletMapPtr map = lanelet::utils::createMap(all_lanelets, {});

    // set the map, with default routing and routingGraph
    cmw->setMap(map);

    // set the obstacle
    if (ob_option == DEFAULT_OBSTACLE)
    {
        cav_msgs::RoadwayObstacle rwo_static;	

        tf2::Quaternion tf_orientation;	
        tf_orientation.setRPY(0, 0, 1.5708);	// 90 deg facing forward

        rwo_static.object.pose.pose.position.x = 1.5 * width;	
        rwo_static.object.pose.pose.position.y = 2.5 * length;	
        rwo_static.object.pose.pose.position.z = 0;	

        rwo_static.object.pose.pose.orientation.x = tf_orientation.getX();	
        rwo_static.object.pose.pose.orientation.y = tf_orientation.getY();	
        rwo_static.object.pose.pose.orientation.z = tf_orientation.getZ();	
        rwo_static.object.pose.pose.orientation.w = tf_orientation.getW();	

        rwo_static.object.size.x = width;	
        rwo_static.object.size.y = width;	
        rwo_static.object.size.z = width;

        rwo_static.down_track = 0.5 * length;

        std::vector<cav_msgs::RoadwayObstacle> rw_objs;
        rw_objs.push_back(rwo_static);	
        cmw->setRoadwayObjects(rw_objs);
    }

    
}

inline void setRouteByLanelets (std::vector<lanelet::Lanelet> lanelets, std::shared_ptr<carma_wm::CARMAWorldModel> cmw)
{
    // Build routing graph from map	
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
inline void setRouteByIds (std::vector<lanelet::Id> lanelet_ids, std::shared_ptr<carma_wm::CARMAWorldModel> cmw)
{
    std::vector<lanelet::Lanelet> lanelets;
    for (auto id : lanelet_ids)
    {
        lanelets.push_back(cmw->getMutableMap()->laneletLayer.get(id));
    }
    setRouteByLanelets(lanelets, cmw);
}

void addObstacle(double x, double y, std::shared_ptr<carma_wm::CARMAWorldModel> cmw, std::vector<std::pair<double,double>> pred_coords = {}, double width = 1, double length = 1);

// Assumes on the road, and assumes that it is the map returned by getGuidanceTestMap
void addObstacle(carma_wm::TrackPos tp, lanelet::Id lanelet_id, std::shared_ptr<carma_wm::CARMAWorldModel> cmw, std::vector<carma_wm::TrackPos> pred_trackpos_list = {}, double width = 1, double length = 1);
}
  //namespace test
} //namespace carma_wm