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
#include "CARMAWorldModel.h"
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Traits.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>

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
    TrackPos tp_next = trackPos(point, lineString_1.front().basicPoint(), next_point.basicPoint());

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
      tp = trackPos(point, prev_centerline[prev_centerline.size() - 2].basicPoint(),
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

    TrackPos tp_prev = trackPos(point, prev_prev_point.basicPoint(), prev_point.basicPoint());

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
      tp = trackPos(point, next_centerline[0].basicPoint(), next_centerline[1].basicPoint());
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

    tp = std::get<0>(matchSegment(point, subSegment));  // Get track pos along centerline

    tp.downtrack += shortest_path_distance_map_.distanceToPointAlongElement(ls_i, p_i - 1);

    bestRouteSegId = lineString_1.id();
  }

  // Accumulate distance
  auto bestRouteSegIndex = shortest_path_distance_map_.getIndexFromId(bestRouteSegId);
  tp.downtrack += shortest_path_distance_map_.distanceToElement(bestRouteSegIndex.first);

  return tp;
}

TrackPos CARMAWorldModel::trackPos(const lanelet::ConstLanelet& lanelet, const lanelet::BasicPoint2d& point) const
{
  auto center_line = lanelet::utils::to2D(lanelet.centerline());

  if (center_line.numSegments() < 1)
  {
    throw std::invalid_argument("Provided lanelet has invalid centerline containing no points");
  }
  auto tuple = matchSegment(point, center_line.basicLineString());

  return std::get<0>(tuple);
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

std::vector<std::tuple<size_t, std::vector<double>>>
CARMAWorldModel::getLocalCurvatures(const std::vector<lanelet::ConstLanelet>& lanelets) const
{
  std::vector<std::tuple<size_t, std::vector<double>>> vec;

  lanelet::BasicPoint2d prevEndPoint;
  lanelet::BasicPoint2d first_2d, second_2d, third_2d;

  for (size_t n = 0; n < lanelets.size(); n++)
  {
    lanelet::ConstLanelet ll = lanelets[n];
    auto mutableCenterLine = lanelet::utils::to2D(ll.centerline()).basicLineString();

    if (mutableCenterLine.empty())
    {
      throw std::invalid_argument("Provided lanelet contains no centerline");
    }

    for (size_t i = 0; i < mutableCenterLine.size() - 1; i++)
    {
      // If the first lanelet initialize the 3 points needed for computing curvature
      if (n == 0 && i <= 1)
      {
        if (i == 0)
        {
          std::vector<double> curvatures;
          vec.push_back(std::make_tuple(n, curvatures));
          continue;
        }
        else
        {  // i == 1
          first_2d = mutableCenterLine[i - 1];
          second_2d = mutableCenterLine[i];
          third_2d = mutableCenterLine[i + 1];
          std::get<1>(vec.back()).push_back(computeCurvature(first_2d, second_2d, third_2d));
          continue;
        }
      }
      else if (n != 0 && i <= 1)
      {
        bool new_segment_needed = lanelet::geometry::distance2d(prevEndPoint, mutableCenterLine[i]) > 0.1;
        if (new_segment_needed)
        {
          if (i == 0)
          {
            // Consider as disjoint lanelet
            std::vector<double> curvatures;
            vec.push_back(std::make_tuple(n, curvatures));
            continue;
          }
          else
          {  // i == 1
            first_2d = mutableCenterLine[i - 1];
            second_2d = mutableCenterLine[i];
            third_2d = mutableCenterLine[i + 1];
            std::get<1>(vec.back()).push_back(computeCurvature(first_2d, second_2d, third_2d));
            continue;
          }
        }
      }

      first_2d = second_2d;
      second_2d = third_2d;
      third_2d = mutableCenterLine[i + 1];
      std::get<1>(vec.back()).push_back(computeCurvature(first_2d, second_2d, third_2d));
    }

    prevEndPoint = mutableCenterLine.back();
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
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
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
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
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

      size_t connectionCount = shortest_path_graph->possiblePaths(ll, 1, false).size();
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

// NOTE: See WorldModel.h header file for details on source of logic in this function
double CARMAWorldModel::computeCurvature(const lanelet::BasicPoint2d& p1, const lanelet::BasicPoint2d& p2,
                                         const lanelet::BasicPoint2d& p3) const
{
  auto dp = 0.5 * (p3 - p1);
  auto ddp = p3 - 2.0 * p2 + p1;
  auto denom = std::pow(dp.x() * dp.x() + dp.y() * dp.y(), 3.0 / 2.0);
  if (std::fabs(denom) < 1e-20)
  {
    denom = 1e-20;
  }
  return static_cast<double>((ddp.y() * dp.x() - dp.y() * ddp.x()) / denom);
}

double CARMAWorldModel::getAngleBetweenVectors(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) const
{
  double vec1Mag = vec1.norm();
  double vec2Mag = vec2.norm();
  if (vec1Mag == 0 || vec2Mag == 0)
  {
    return 0;
  }
  return std::acos(vec1.dot(vec2) / (vec1Mag * vec2Mag));
}

TrackPos CARMAWorldModel::trackPos(const lanelet::BasicPoint2d& p, const lanelet::BasicPoint2d& seg_start,
                                   const lanelet::BasicPoint2d& seg_end) const
{
  Eigen::Vector2d vec_to_p(p);
  Eigen::Vector2d vec_to_start(seg_start);
  Eigen::Vector2d vec_to_end(seg_end);

  // Get vector from start to external point
  Eigen::Vector2d start_to_p = vec_to_p - vec_to_start;

  // Get vector from start to end point
  Eigen::Vector2d start_to_end = vec_to_end - vec_to_start;

  // Get angle between both vectors
  double interior_angle = getAngleBetweenVectors(start_to_p, start_to_end);

  // Calculate downtrack distance
  double start_to_p_mag = start_to_p.norm();
  double downtrack_dist = start_to_p_mag * std::cos(interior_angle);

  /**
   * Calculate the sign of the crosstrack distance by projecting the points to 2d
   * d = (p_x - s_x)(e_y - s_y) - (p_y - s_y)(e_x - s_x)
   * Equivalent to d = (start_to_p.x * start_to_end.y) - (start_to_p.y * start_to_end.x)
   *
   * Code below based on math equation described at
   * https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
   * Question asked by user
   * Ritvars (https://math.stackexchange.com/users/56723/ritvars)
   * and answered by user
   * Shard (https://math.stackexchange.com/users/55608/shard)
   * Attribution here is in line with Stack Overflow's Attribution policy cc-by-sa found here:
   * https://stackoverflow.blog/2009/06/25/attribution-required/
   */

  double d = (start_to_p[0] * start_to_end[1]) - (start_to_p[1] * start_to_end[0]);
  double sign = d >= 0 ? 1.0 : -1.0;  // If d is positive then the point is to the right if it is negative the point is
                                      // to the left

  double crosstrack = start_to_p_mag * std::sin(interior_angle) * sign;

  return TrackPos(downtrack_dist, crosstrack);
}

bool CARMAWorldModel::selectFirstSegment(const TrackPos& first_seg_trackPos, const TrackPos& second_seg_trackPos,
                                         double first_seg_length, double second_seg_length) const
{
  const bool first_seg_in_downtrack =
      (0 <= first_seg_trackPos.downtrack && first_seg_trackPos.downtrack < first_seg_length);
  const bool second_seg_in_downtrack =
      (0 <= second_seg_trackPos.downtrack && second_seg_trackPos.downtrack < second_seg_length);

  if (first_seg_in_downtrack && !second_seg_in_downtrack)
  {  // If in the first segment but not the last segment

    return true;  // First segment is better
  }
  else if (second_seg_in_downtrack && !first_seg_in_downtrack)
  {
    return false;  // Second segment is better
  }
  else if (first_seg_in_downtrack && second_seg_in_downtrack)
  {
    return first_seg_trackPos.crosstrack <=
           second_seg_trackPos.crosstrack;  // Pick the first segment if the crosstrack values are equal or the first
                                            // segment is closer
  }
  else
  {  // Point lies outside the downtrack bounds of both segments (Remember According to function contract the nearest
     // point to external point is the midpoint of the two segments)
    return true;  // Choose first segment as it will always have positive downtrack in this case while the second
                  // segment will always have negative downtrack
  }
}

std::tuple<TrackPos, lanelet::BasicSegment2d>
CARMAWorldModel::matchSegment(const lanelet::BasicPoint2d& p, const lanelet::BasicLineString2d& line_string) const
{
  if (line_string.size() < 2)
  {
    throw std::invalid_argument("Provided with linestring containing fewer than 2 points");
  }

  lanelet::BasicSegment2d best_segment =
      std::make_pair(line_string[0], line_string[1]);  // Default to starting segment if no match is found
  auto point_2d = lanelet::utils::to2D(p);

  double min_distance = lanelet::geometry::distance2d(point_2d, line_string[0]);
  size_t best_point_index = 0;
  double best_accumulated_length = 0;
  double best_last_accumulated_length = 0;
  double best_seg_length = 0;
  double best_last_seg_length = 0;
  double last_seg_length = 0;

  double accumulated_length = 0;
  double last_accumulated_length = 0;
  for (size_t i = 0; i < line_string.size(); i++)
  {  // Iterate over line string to find nearest point
    double seg_length = 0;
    if (i < line_string.size() - 1)
    {
      seg_length = lanelet::geometry::distance2d(line_string[i], line_string[i + 1]);  // Compute segment length
    }

    double distance = lanelet::geometry::distance2d(p, line_string[i]);  // Compute from current point to external point
    if (distance < min_distance)
    {  // If this distance is below minimum discovered so far then update minimum
      min_distance = distance;
      best_point_index = i;
      best_accumulated_length = accumulated_length;
      best_last_accumulated_length = last_accumulated_length;  // Record accumulated lengths to each segment
      best_seg_length = seg_length;
      best_last_seg_length = last_seg_length;
    }

    last_accumulated_length = accumulated_length;  // Update accumulated lenths
    accumulated_length += seg_length;
    last_seg_length = seg_length;
  }

  // Minimum point has been found next step is to determine which segment it should go with using the following rules.
  // If the minimum point is the first point then use the first segment
  // If the minimum point is the last point then use the last segment
  // If the minimum point is within the downtrack bounds of one segment but not the other then use the one it is within
  // If the minimum point is within the downtrack bounds of both segments then use the one with the smallest crosstrack
  // distance If the minimum point is within the downtrack bounds of both segments and has exactly equal crosstrack
  // bounds with each segment then use the first one
  TrackPos best_pos(0, 0);
  if (best_point_index == 0)
  {
    best_pos = trackPos(p, line_string[0], line_string[1]);
    best_segment = std::make_pair(line_string[0], line_string[1]);
  }
  else if (best_point_index == line_string.size() - 1)
  {
    best_pos = trackPos(p, line_string[line_string.size() - 2], line_string[line_string.size() - 1]);
    best_pos.downtrack += best_last_accumulated_length;
    best_segment = std::make_pair(line_string[line_string.size() - 2], line_string[line_string.size() - 1]);
  }
  else
  {
    TrackPos first_seg_trackPos = trackPos(p, line_string[best_point_index - 1], line_string[best_point_index]);
    TrackPos second_seg_trackPos = trackPos(p, line_string[best_point_index], line_string[best_point_index + 1]);
    if (selectFirstSegment(first_seg_trackPos, second_seg_trackPos, best_last_seg_length, best_seg_length))
    {
      best_pos = first_seg_trackPos;
      best_pos.downtrack += best_last_accumulated_length;
      best_segment = std::make_pair(line_string[best_point_index - 1], line_string[best_point_index]);
    }
    else
    {
      best_pos = second_seg_trackPos;
      best_pos.downtrack += best_accumulated_length;
      best_segment = std::make_pair(line_string[best_point_index], line_string[best_point_index + 1]);
    }
  }

  // couldn't find a matching segment, so use the first segment within the downtrack range of.
  // Or the starting segment assuming we are before the route
  return std::make_tuple(best_pos, best_segment);
}
}  // namespace carma_wm