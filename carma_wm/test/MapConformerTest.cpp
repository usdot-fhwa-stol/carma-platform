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
#include <carma_wm/MapConformer.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_traffic_rules/GermanTrafficRules.h>
#include <lanelet2_extension/regulatory_elements/RegionAccessRule.h>
#include <lanelet2_extension/regulatory_elements/PassingControlLine.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <lanelet2_core/utility/Units.h>
#include <boost/algorithm/string.hpp>
#include "TestHelpers.h"
using namespace lanelet::units::literals;

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_wm
{
/**
 * @brief Function modifies an existing map to make a best effort attempt at ensuring the map confroms to the
 * expectations of CarmaUSTrafficRules
 *
 * Map is updated by ensuring all lanelet and area bounds are marked with PassingControlLines
 * In addition, lanelets and areas are updated to have their accessability marked with a RegionAccessRule.
 * At the moment the creation of DigitalSpeedLimits for all lanelets/areas is not performed. This is because
 * CarmaUSTrafficRules supports the existing SpeedLimit definition and allows DigitalSpeedLimits to be overlayed on
 * that.
 *
 * @param map A pointer to the map which will be modified in place
 */
// void ensureCompliance(lanelet::LaneletMapPtr map);
TEST(MapConformer, ensureCompliance)
{
  auto map = carma_wm::getDisjointRouteMap();

  // Create carma traffic rules object
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::traffic_rules::CarmaUSTrafficRules::Location, lanelet::Participants::VehicleCar);

  
  lanelet::Velocity con_lim = 0_mph;

  ASSERT_EQ(4, map->laneletLayer.size());
  ASSERT_EQ(1, map->areaLayer.size());
  ASSERT_EQ(0, map->regulatoryElementLayer.size()) << "There should be no regulations in the map at this point";

  lanelet::MapConformer::ensureCompliance(map, con_lim);

  // First verify that each lanelet has a left and right control line and a region access rule
  for (auto ll : map->laneletLayer)
  {
    auto control_lines = ll.regulatoryElementsAs<lanelet::PassingControlLine>();
    ASSERT_EQ(2, control_lines.size());

    auto access_rules = ll.regulatoryElementsAs<lanelet::RegionAccessRule>();
    ASSERT_EQ(1, access_rules.size());

    if (ll.id() == 10000)
    {  // First lanelet in disjoint route
      ASSERT_FALSE(lanelet::PassingControlLine::boundPassable(ll.leftBound(), control_lines, false,
                                                              lanelet::Participants::Vehicle));

      ASSERT_TRUE(lanelet::PassingControlLine::boundPassable(ll.rightBound(), control_lines, true,
                                                             lanelet::Participants::Vehicle));

      ASSERT_TRUE(access_rules[0]->accessable(lanelet::Participants::Vehicle));

      ASSERT_TRUE(traffic_rules->isOneWay(ll));
    }
    else if (ll.id() == 10001)
    {
      ASSERT_TRUE(lanelet::PassingControlLine::boundPassable(ll.leftBound(), control_lines, false,
                                                             lanelet::Participants::Vehicle));

      ASSERT_FALSE(lanelet::PassingControlLine::boundPassable(ll.rightBound(), control_lines, true,
                                                              lanelet::Participants::Vehicle));

      ASSERT_TRUE(access_rules[0]->accessable(lanelet::Participants::Vehicle));

      ASSERT_TRUE(traffic_rules->isOneWay(ll));
    }
    else if (ll.id() == 10002)
    {
      ASSERT_FALSE(lanelet::PassingControlLine::boundPassable(ll.leftBound(), control_lines, false,
                                                              lanelet::Participants::Vehicle));

      ASSERT_FALSE(lanelet::PassingControlLine::boundPassable(ll.rightBound(), control_lines, true,
                                                              lanelet::Participants::Vehicle));

      ASSERT_TRUE(access_rules[0]->accessable(lanelet::Participants::Vehicle));

      ASSERT_TRUE(traffic_rules->isOneWay(ll));
    }
    else if (ll.id() == 10003)
    {  // Two way lanelet
      ASSERT_FALSE(lanelet::PassingControlLine::boundPassable(ll.leftBound(), control_lines, false,
                                                              lanelet::Participants::Vehicle));

      ASSERT_FALSE(lanelet::PassingControlLine::boundPassable(ll.rightBound(), control_lines, true,
                                                              lanelet::Participants::Vehicle));

      ASSERT_TRUE(access_rules[0]->accessable(lanelet::Participants::Vehicle));

      ASSERT_FALSE(traffic_rules->isOneWay(ll));
    }
    else
    {
      FAIL() << "The base map used in TEST(MapConformer, ensureCompliance) has changed. The unit test must be updated. "
                "Lanelet Check";
    }
  }

  // Check areas
  for (auto area : map->areaLayer)
  {
    auto control_lines = area.regulatoryElementsAs<lanelet::PassingControlLine>();
    ASSERT_EQ(1, control_lines.size());

    auto access_rules = area.regulatoryElementsAs<lanelet::RegionAccessRule>();
    ASSERT_EQ(1, access_rules.size());

    if (area.id() == 10004)
    {  // First lanelet in disjoint route
      ASSERT_EQ(1, area.outerBound().size());
      ASSERT_TRUE(lanelet::PassingControlLine::boundPassable(area.outerBound()[0], control_lines, false,
                                                             lanelet::Participants::Vehicle));

      ASSERT_TRUE(lanelet::PassingControlLine::boundPassable(area.outerBound()[0], control_lines, true,
                                                             lanelet::Participants::Vehicle));

      ASSERT_FALSE(access_rules[0]->accessable(lanelet::Participants::Vehicle));
    }
    else
    {
      FAIL() << "The base map used in TEST(MapConformer, ensureCompliance) has changed. The unit test must be updated. "
                "Area Check";
    }
  }

  ASSERT_EQ(1, map->areaLayer.size());
  ASSERT_EQ(4, map->laneletLayer.size());
  ASSERT_EQ(18, map->regulatoryElementLayer.size());  // New map should contain 7 passing control lines and 4 region
                                                      // access rules and 1 direction of travel rule
                                                      // 4 DigitalSpeedLimits
  // Then verify that routing can still be done properly over this map
  // Build routing graph from map

  lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

  // Try Generating a route
  auto ll_1 = map->laneletLayer.find(10000);
  auto ll_3 = map->laneletLayer.find(10002);

  auto optional_route = map_graph->getRoute(*ll_1, *ll_3);

  ASSERT_TRUE((bool)optional_route);  // Routing is possible

  auto shortest_path = (*optional_route).shortestPath();

  // Verify resulting route
  ASSERT_EQ(3, shortest_path.size());

  lanelet::Id count = 10000;
  for (auto ll : shortest_path)
  {
    ASSERT_EQ(count, ll.id());
    count++;
  }

//Test Speed Limits
/*
  * Create 2x2 lanelets map by hand
  */
std::vector<lanelet::Lanelet> llts;
lanelet::Areas areas;
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

  llts.push_back(carma_wm::getLanelet(mid_1, right_1, lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::SolidSolid));

// Add regualtory element - Passing Control Line
  
      std::shared_ptr<lanelet::DigitalSpeedLimit> dsl(new lanelet::DigitalSpeedLimit(lanelet::DigitalSpeedLimit::buildData(
        lanelet::utils::getId(), 80_mph, llts, areas, {lanelet::Participants::Vehicle})));
        llts.back().addRegulatoryElement(dsl);
      

      std::shared_ptr<lanelet::DigitalSpeedLimit> dsl2(new lanelet::DigitalSpeedLimit(lanelet::DigitalSpeedLimit::buildData(
        lanelet::utils::getId(), 39_mph, llts, areas, {lanelet::Participants::Vehicle})));
        llts.back().addRegulatoryElement(dsl2);
  



  // Create lanelet map
  lanelet::LaneletMapPtr map2;
  map2 = lanelet::utils::createMap(llts, {});

 //Test speed limits within ensureCompliance
 lanelet::MapConformer::ensureCompliance(map2, 0_mph);//config_limit not in use

 for (auto ll: map2->laneletLayer)
 {
   auto speed_limit = ll.regulatoryElementsAs<lanelet::DigitalSpeedLimit>();
   
   ASSERT_LT(speed_limit.back().get()->speed_limit_ , 80_mph); 

   ASSERT_EQ(2, llts.size());

 //Since the value is larger than the max speed limit, the speed limit should be reset to 80_mph
  // ASSERT_EQ(speed_limit.front().get()->speed_limit_, 80_mph);

 }

ASSERT_EQ(7, map2->regulatoryElementLayer.size());//Test that there are only 7 regulatory elements: 4 access rules
                                                  //+ 1 direction of travel route + 2 digital speed limits







}
}  // namespace carma_wm