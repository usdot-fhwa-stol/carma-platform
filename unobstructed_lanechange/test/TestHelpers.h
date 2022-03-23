#pragma once
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

/**
 * Helper file containing inline functions used to quickly build lanelet objects in unit tests
 *
 */
namespace carma_wm
{
inline lanelet::Point3d getPoint(double x, double y, double z)
{
  return lanelet::Point3d(lanelet::utils::getId(), x, y, z);
}

inline lanelet::BasicPoint3d getBasicPoint(double x, double y, double z)
{
  return getPoint(x, y, z).basicPoint();
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

inline void addStraightRoute(CARMAWorldModel& cmw)
{
  // 1. Construct map
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
  lanelet::routing::Route route = std::move(*optional_route);
  LaneletRoutePtr route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
  // 5. Set route and map
  cmw.setRoute(route_ptr);
  cmw.setMap(map);
}

inline void addDisjointRoute(CARMAWorldModel& cmw)
{
  // 1. Construct map
  auto p1 = getPoint(0, 0, 0);
  auto p2 = getPoint(0, 1, 0);
  auto p3 = getPoint(1, 1, 0);
  auto p4 = getPoint(1, 2, 0);
  auto p5 = getPoint(1, 0, 0);
  auto p6 = getPoint(2, 0, 0);
  auto p7 = getPoint(2, 1, 0);
  auto p8 = getPoint(2, 2, 0);
  lanelet::LineString3d left_ls_1(lanelet::utils::getId(), { p1, p2 });
  lanelet::LineString3d right_ls_1(lanelet::utils::getId(), { p5, p3 });
  auto ll_1 = getLanelet(left_ls_1, right_ls_1, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);

  lanelet::LineString3d right_ls_2(lanelet::utils::getId(), { p6, p7 });
  auto ll_2 =
      getLanelet(right_ls_1, right_ls_2, lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Solid);

  lanelet::LineString3d left_ls_3(lanelet::utils::getId(), { p3, p4 });
  lanelet::LineString3d right_ls_3(lanelet::utils::getId(), { p7, p8 });
  auto ll_3 =
      getLanelet(left_ls_3, right_ls_3, lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Solid);

  // 2. Build map but do not assign
  // Create basic map and verify that the map and routing graph can be build, but the route remains false
  lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1, ll_2, ll_3 }, {});

  // 3. Build routing graph but do not assign
  // Build routing graph from map
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
  lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

  // 4. Generate route
  auto optional_route = map_graph->getRoute(ll_1, ll_3);
  // map_graph->exportGraphViz("my_graph"); // Uncomment to visualize route graph
  lanelet::routing::Route route = std::move(*optional_route);
  LaneletRoutePtr route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
  // 5. Set route and map
  cmw.setRoute(route_ptr);
  cmw.setMap(map);
}
}  // namespace carma_wm