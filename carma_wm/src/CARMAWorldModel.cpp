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

#include <tuple>
#include <algorithm>
#include <assert.h>
#include <carma_wm/CARMAWorldModel.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Traits.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <carma_wm/Geometry.h>

namespace carma_wm
{

CARMAWorldModel::CARMAWorldModel()
{
}

CARMAWorldModel::~CARMAWorldModel()
{
}

std::pair<TrackPos, TrackPos> CARMAWorldModel::routeTrackPos(const lanelet::ConstArea& area) const
{
  // Check if the route was loaded yet
  if (!route_)
  {
    throw std::invalid_argument("Route has not yet been loaded");
  }

  lanelet::ConstLineStrings3d outer_bound = area.outerBound();

  if (outer_bound.empty())
  {
    throw std::invalid_argument("Provided area outer bound is invalid as it contains no points");
  }

  TrackPos minPos(0, 0);
  TrackPos maxPos(0, 0);
  bool first = true;
  for (lanelet::ConstLineString3d sub_bound3d : outer_bound)
  {
    auto sub_bound = lanelet::utils::to2D(sub_bound3d);
    for (lanelet::ConstPoint2d point : sub_bound)
    {
      TrackPos tp = routeTrackPos(point);
      if (first)
      {
        minPos = maxPos = tp;
        first = false;
      }
      else if (tp.downtrack < minPos.downtrack)
      {
        minPos.downtrack = tp.downtrack;
        minPos.crosstrack = tp.crosstrack;
      }
      else if (tp.downtrack > maxPos.downtrack)
      {
        maxPos.downtrack = tp.downtrack;
        maxPos.crosstrack = tp.crosstrack;
      }
    }
  }
  return std::make_pair(minPos, maxPos);
}

TrackPos CARMAWorldModel::routeTrackPos(const lanelet::ConstLanelet& lanelet) const
{
  // Check if the route was loaded yet
  if (!route_)
  {
    throw std::invalid_argument("Route has not yet been loaded");
  }

  lanelet::ConstLineString2d centerline = lanelet::utils::to2D(lanelet.centerline());
  if (centerline.empty())
  {
    throw std::invalid_argument("Provided lanelet has invalid centerline containing no points");
  }
  auto front = centerline.front();
  return routeTrackPos(front);
}

TrackPos CARMAWorldModel::routeTrackPos(const lanelet::BasicPoint2d& point) const
{
  // Check if the route was loaded yet
  if (!route_)
  {
    throw std::invalid_argument("Route has not yet been loaded");
  }

  // Find the nearest continuos shortest path centerline segment using fast map nearest search
  lanelet::Points3d near_points =
      shortest_path_filtered_centerline_view_->pointLayer.nearest(point, 1);  // Find the nearest points

  // Match point with linestring using fast map lookup
  auto lineString_1 =
      lanelet::utils::to2D(shortest_path_filtered_centerline_view_->lineStringLayer.findUsages(near_points[0])
                               .front());  // Only need the first index due to nature of route centerline

  if (lineString_1.size() == 0)
  {
    throw std::invalid_argument("Invalid route loaded. Shortest path does not have proper references");
  }

  // New approach
  // 1. Find nearest point
  // 2. Find linestring associated with nearest point
  // 3. If the nearest point is the first point on the linestring then we need to check the downtrack value
  // 4. -- If donwtrack is negative then iterate over route segments to find preceeding segment
  // 5. -- If downtrack is positive then we are on the correct segment
  // 6. If the nearest point is the last point on the linestring then we need to check downtrack value
  // 7. -- If downtrack is less then seg length then we are on the correct segment
  // 8. -- If downtrack is greater then seg length then iterate over route segments to find succeeding segment
  // 9. With correct segment identified compute segment downtrack distance
  // 10. Accumulate previos segment distances if needed.

  // Find best route segment
  lanelet::Id bestRouteSegId;
  TrackPos tp(0, 0);
  // Check for end cases

  auto indexes = shortest_path_distance_map_.getIndexFromId(near_points[0].id());
  size_t ls_i = indexes.first;
  size_t p_i = indexes.second;

  if (near_points[0].id() == lineString_1.front().id())
  {  // Nearest point is at the start of a line string
    // Get start point of cur segment and add 1
    auto next_point = lineString_1[1];
    TrackPos tp_next = geometry::trackPos(point, lineString_1.front().basicPoint(), next_point.basicPoint());

    if (tp_next.downtrack >= 0 || ls_i == 0)
    {
      bestRouteSegId = lineString_1.id();
      tp = tp_next;
      // If downtrack is positive then we are on the correct segment
    }
    else
    {
      // If downtrack is negative then iterate over route segments to find preceeding segment
      const size_t prev_ls_i = ls_i - 1;

      auto prev_centerline = lanelet::utils::to2D(shortest_path_centerlines_[prev_ls_i]);  // Get prev centerline
      tp = geometry::trackPos(point, prev_centerline[prev_centerline.size() - 2].basicPoint(),
                              prev_centerline[prev_centerline.size() - 1].basicPoint());
      tp.downtrack += shortest_path_distance_map_.distanceToPointAlongElement(prev_ls_i, prev_centerline.size() - 2);
      bestRouteSegId = prev_centerline.id();
    }
  }
  else if (near_points[0].id() == lineString_1.back().id())
  {  // Nearest point is the end of a line string

    // Get end point of cur segment and subtract 1
    auto prev_prev_point = lineString_1[lineString_1.size() - 2];
    auto prev_point = lineString_1[lineString_1.size() - 1];

    TrackPos tp_prev = geometry::trackPos(point, prev_prev_point.basicPoint(), prev_point.basicPoint());

    double last_seg_length =
        shortest_path_distance_map_.distanceBetween(ls_i, lineString_1.size() - 2, lineString_1.size() - 1);

    if (tp_prev.downtrack < last_seg_length || ls_i == shortest_path_centerlines_.size() - 1)
    {
      // If downtrack is less then seg length then we are on the correct segment
      bestRouteSegId = lineString_1.id();
      tp = tp_prev;
      tp.downtrack += shortest_path_distance_map_.distanceToPointAlongElement(ls_i, lineString_1.size() - 2);
    }
    else
    {
      // If downtrack is greater then seg length then we need to find the succeeding segment
      auto next_centerline = lanelet::utils::to2D(shortest_path_centerlines_[ls_i + 1]);  // Get prev centerline
      tp = geometry::trackPos(point, next_centerline[0].basicPoint(), next_centerline[1].basicPoint());
      bestRouteSegId = next_centerline.id();
    }
  }
  else
  {  // The nearest point is in the middle of a line string
    // Graph the two bounding points on the line string and call matchSegment using a 3 element segment
    // There is a guarantee from the earlier if statements that near_points[0] will always be located at an index within
    // the exclusive range (0,lineString_1.size() - 1) so no need for range checks

    lanelet::BasicLineString2d subSegment = lanelet::BasicLineString2d(
        { lineString_1[p_i - 1].basicPoint(), lineString_1[p_i].basicPoint(), lineString_1[p_i + 1].basicPoint() });

    tp = std::get<0>(geometry::matchSegment(point, subSegment));  // Get track pos along centerline

    tp.downtrack += shortest_path_distance_map_.distanceToPointAlongElement(ls_i, p_i - 1);

    bestRouteSegId = lineString_1.id();
  }

  // Accumulate distance
  auto bestRouteSegIndex = shortest_path_distance_map_.getIndexFromId(bestRouteSegId);
  tp.downtrack += shortest_path_distance_map_.distanceToElement(bestRouteSegIndex.first);

  return tp;
}

std::vector<lanelet::ConstLanelet> CARMAWorldModel::getLaneletsBetween(double start, double end) const
{
  // Check if the route was loaded yet
  if (!route_)
  {
    throw std::invalid_argument("Route has not yet been loaded");
  }
  if (start >= end)
  {
    throw std::invalid_argument("Start distance is greater than or equal to end distance");
  }

  std::vector<lanelet::ConstLanelet> vec;

  auto lanelet_map = route_->laneletMap();
  for (lanelet::ConstLanelet lanelet : lanelet_map->laneletLayer)
  {
    lanelet::ConstLineString2d centerline = lanelet::utils::to2D(lanelet.centerline());

    auto front = centerline.front();
    auto back = centerline.back();
    TrackPos min = routeTrackPos(front);
    TrackPos max = routeTrackPos(back);

    if (std::max(min.downtrack, start) > std::min(max.downtrack, end))
    {  // Check for 1d intersection
      // No intersection so continue
      continue;
    }
    // Intersection has occurred so add lanelet to list
    vec.push_back(lanelet);
  }

  return vec;
}

lanelet::LaneletMapConstPtr CARMAWorldModel::getMap() const
{
  return std::static_pointer_cast<lanelet::LaneletMap const>(semantic_map_);  // Cast pointer to const variant
}

LaneletRouteConstPtr CARMAWorldModel::getRoute() const
{
  return std::static_pointer_cast<const lanelet::routing::Route>(route_);  // Cast pointer to const variant
}

void CARMAWorldModel::setMap(lanelet::LaneletMapPtr map)
{
  semantic_map_ = map;
  // Build routing graph from map
  TrafficRulesConstPtr traffic_rules = *(getTrafficRules(lanelet::Participants::Vehicle));

  lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*semantic_map_, *traffic_rules);
  map_routing_graph_ = std::move(map_graph);
}

void CARMAWorldModel::setRoute(LaneletRoutePtr route)
{
  route_ = route;

  lanelet::ConstLanelets path_lanelets(route_->shortestPath().begin(), route_->shortestPath().end());

  shortest_path_view_ = lanelet::utils::createConstMap(path_lanelets, {});

  computeDowntrackReferenceLine();
}

lanelet::LineString3d CARMAWorldModel::copyConstructLineString(const lanelet::ConstLineString3d& line) const
{
  std::vector<lanelet::Point3d> coppied_points;
  coppied_points.reserve(line.size());

  for (auto basic_p : line.basicLineString())
  {
    coppied_points.push_back(lanelet::Point3d(lanelet::utils::getId(), basic_p));
  }

  return lanelet::LineString3d(lanelet::utils::getId(), coppied_points);
}

void CARMAWorldModel::computeDowntrackReferenceLine()
{
  IndexedDistanceMap distance_map;

  lanelet::routing::LaneletPath shortest_path = route_->shortestPath();
  // Build shortest path routing graph
  TrafficRulesConstPtr traffic_rules = *(getTrafficRules(lanelet::Participants::Vehicle));

  lanelet::routing::RoutingGraphUPtr shortest_path_graph =
      lanelet::routing::RoutingGraph::build(*shortest_path_view_, *traffic_rules);

  std::vector<lanelet::LineString3d> lineStrings;  // List of continuos line strings representing segments of the route
                                                   // reference line

  bool first = true;
  size_t next_index = 0;

  // Iterate over each lanelet in the shortest path this loop works by looking one lanelet ahead to detect lane changes
  for (lanelet::ConstLanelet ll : shortest_path)
  {
    next_index++;
    if (first)
    {  // For the first lanelet store its centerline and length
      lineStrings.push_back(copyConstructLineString(ll.centerline()));
      first = false;
    }
    if (next_index < shortest_path.size())
    {  // Check for remaining lanelets
      auto nextLanelet = shortest_path[next_index];
      lanelet::LineString3d nextCenterline = copyConstructLineString(nextLanelet.centerline());

      size_t connectionCount = shortest_path_graph->possiblePaths(ll, (uint32_t)2, false).size();

      if (connectionCount == 1)
      {  // Get list of connected lanelets without lanechanges. On the shortest path this should only return 1 or 0
        // No lane change
        // Append distance to current centerline
        lineStrings.back().insert(lineStrings.back().end(), nextCenterline.begin(), nextCenterline.end());
      }
      else if (connectionCount == 0)
      {
        // Lane change required
        // Break the point chain when a lanechange occurs
        lanelet::LineString3d empty_linestring;
        empty_linestring.setId(lanelet::utils::getId());
        distance_map.pushBack(lanelet::utils::to2D(lineStrings.back()));
        lineStrings.push_back(empty_linestring);
      }
      else
      {
        assert(false);  // It should not be possable to reach this point. Doing so demonstrates a bug
      }
    }
  }
  // Copy values to member variables
  shortest_path_centerlines_ = lineStrings;
  shortest_path_distance_map_ = distance_map;

  // Add length of final sections
  if (shortest_path_centerlines_.size() > shortest_path_distance_map_.size())
  {
    shortest_path_distance_map_.pushBack(lanelet::utils::to2D(lineStrings.back()));  // Record length of last continuous
                                                                                     // segment
  }

  shortest_path_filtered_centerline_view_ = lanelet::utils::createMap(shortest_path_centerlines_);
}

LaneletRoutingGraphConstPtr CARMAWorldModel::getMapRoutingGraph() const
{
  return std::static_pointer_cast<const lanelet::routing::RoutingGraph>(map_routing_graph_);  // Cast pointer to const
                                                                                              // variant
}

lanelet::Optional<TrafficRulesConstPtr> CARMAWorldModel::getTrafficRules(const std::string& participant) const
{
  lanelet::Optional<TrafficRulesConstPtr> optional_ptr;
  // Create carma traffic rules object
  try
  {
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::traffic_rules::CarmaUSTrafficRules::Location, participant);

    optional_ptr = std::static_pointer_cast<const lanelet::traffic_rules::TrafficRules>(
        lanelet::traffic_rules::TrafficRulesPtr(std::move(traffic_rules)));
  }
  catch (const lanelet::InvalidInputError& e)
  {
    return optional_ptr;
  }

  return optional_ptr;
}
}  // namespace carma_wm