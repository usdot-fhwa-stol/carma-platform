/*
 * Copyright (C) 2021 LEIDOS.
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

#include <carma_wm/WorldModelUtils.h>

namespace carma_wm
{
namespace query
{

std::vector<lanelet::ConstLanelet> getLaneletsFromPoint(const lanelet::LaneletMapConstPtr& semantic_map, const lanelet::BasicPoint2d& point,
                                                                    const unsigned int n)
{
  // Check if the map is loaded yet
  if (!semantic_map || semantic_map->laneletLayer.size() == 0)
  {
    throw std::invalid_argument("Map is not set or does not contain lanelets");
  }
  std::vector<lanelet::ConstLanelet> possible_lanelets;
  auto nearestLanelets = lanelet::geometry::findNearest(semantic_map->laneletLayer, point, n);
  if (nearestLanelets.size() == 0)
    return {};

  size_t id = 0;  // closest ones are in the back
  // loop through until the point is no longer geometrically in the lanelet

  while (boost::geometry::within(point, nearestLanelets[id].second.polygon2d()))
  {
    possible_lanelets.push_back(nearestLanelets[id].second);
    id++;
    if (id >= nearestLanelets.size())
      break;
  }
  return possible_lanelets;
}

std::vector<lanelet::Lanelet> getLaneletsFromPoint(const lanelet::LaneletMapPtr& semantic_map, const lanelet::BasicPoint2d& point,
                                                                    const unsigned int n)
{
  lanelet::LaneletMapConstPtr const_ptr = semantic_map;
  auto possible_lanelets = getLaneletsFromPoint(const_ptr, point, n);
  std::vector<lanelet::Lanelet> return_lanelets;
  for (auto llt : possible_lanelets)
  {
    return_lanelets.push_back(semantic_map->laneletLayer.get(llt.id()));
  }
  return return_lanelets;
}

std::vector<lanelet::Lanelet> nonConnectedAdjacentLeft(const lanelet::LaneletMapPtr& semantic_map, const lanelet::BasicPoint2d& point,
                                                                    const unsigned int n)
{
  
  lanelet::LaneletMapConstPtr const_ptr = semantic_map;
  auto possible_lanelets = nonConnectedAdjacentLeft(const_ptr, point, n);
  
  
  std::vector<lanelet::Lanelet> return_lanelets;
  for (auto llt : possible_lanelets)
  {
    return_lanelets.push_back(semantic_map->laneletLayer.get(llt.id()));
  }
  return return_lanelets;
}

std::vector<lanelet::ConstLanelet> nonConnectedAdjacentLeft(const lanelet::LaneletMapConstPtr& semantic_map, const lanelet::BasicPoint2d& input_point,
                                                                    const unsigned int n)
{
  
  // Check if the map is loaded yet
  if (!semantic_map || semantic_map->laneletLayer.size() == 0)
  {
    throw std::invalid_argument("Map is not set or does not contain lanelets");
  }
  
  auto input_lanelets = getLaneletsFromPoint(semantic_map, input_point);
  
  if (input_lanelets.empty())
  {
    throw std::invalid_argument("Input point x: " + std::to_string(input_point.x()) + ", y: " + std::to_string(input_point.y()) + " is not in the map");
  }
  
  auto input_lanelet = input_lanelets[0]; 
  
  auto point_downtrack = carma_wm::geometry::trackPos(input_lanelet, input_point).downtrack;
  
  auto point_downtrack_ratio = point_downtrack / carma_wm::geometry::trackPos(input_lanelet, input_lanelet.centerline().back().basicPoint2d()).downtrack;

  auto point_on_ls = input_lanelet.leftBound2d()[std::round((input_lanelet.leftBound2d().size()- 1) * point_downtrack_ratio) ];

  // point_on_opposite_lane coord is acquired by extrapolating the line from input_point to point_on_ls with same distance.
  // threfore point_on_opposite_lane.x() = input_point.x() + 2dx, where dx = point_on_ls.x() - input_point.x(). Here, one of input_point.x() cancels out, results in:
  auto point_on_opposite_lane = lanelet::BasicPoint2d{2 * point_on_ls.x() - input_point.x(), 2 * point_on_ls.y() - input_point.y()};

  auto opposite_lanelets = getLaneletsFromPoint(semantic_map, point_on_opposite_lane, n);

  // TODO: create opposite direction protection throw
  // currently the function doesn't recognize if adjacent lane is opposite direction, but assumes it

  return opposite_lanelets;
}


lanelet::ConstLaneletOrAreas getAffectedLaneletOrAreas(const lanelet::Points3d& gf_pts, const lanelet::LaneletMapPtr& lanelet_map, std::shared_ptr<const lanelet::routing::RoutingGraph> routing_graph, double max_lane_width)
{
  // Logic to detect which part is affected
  ROS_DEBUG_STREAM("Get affected lanelets loop");
  std::unordered_set<lanelet::Lanelet> affected_lanelets;
  for (size_t idx = 0; idx < gf_pts.size(); idx ++)
  {
    ROS_DEBUG_STREAM("Index: " << idx << " Point: " << gf_pts[idx].x() << ", " << gf_pts[idx].y());
    std::unordered_set<lanelet::Lanelet> possible_lanelets;

    // This loop identifes the lanelets which this point lies within that could be impacted by the geofence
    // This loop somewhat inefficiently calls the findNearest method iteratively until all the possible lanelets are identified. 
    // The reason findNearest is used instead of nearestUntil is because that method orders results by bounding box which
    // can give invalid sequences when dealing with large curved lanelets.  
    bool continue_search = true; 
    size_t nearest_count = 0;
    while (continue_search) {
      
      nearest_count += 10; // Increase the index search radius by 10 each loop until all nearby lanelets are found

      for (const auto& ll_pair : lanelet::geometry::findNearest(lanelet_map->laneletLayer, gf_pts[idx].basicPoint2d(), nearest_count)) { // Get the nearest lanelets and iterate over them
        auto ll = std::get<1>(ll_pair);

        if (possible_lanelets.find(ll) != possible_lanelets.end()) { // Skip if already found
          continue;
        }

        double dist = std::get<0>(ll_pair);
        ROS_DEBUG_STREAM("Distance to lanelet " << ll.id() << ": " << dist << " max_lane_width: " << max_lane_width);
        
        if (dist > max_lane_width) { // Only save values closer than max_lane_width. Since we are iterating in distance order when we reach this distance the search can stop
          continue_search = false;
          break;
        }

        // Check if the point is inside this lanelet
        if(dist == 0.0) { // boost geometry uses a distance of 0 to indicate a point is within a polygon
          possible_lanelets.insert(ll);
        }

      }

      if (nearest_count >= lanelet_map->laneletLayer.size()) { // if we are out of lanelets to evaluate then end the search
        continue_search = false;
      }
    }

    // among these llts, filter the ones that are on same direction as the geofence using routing
    if (idx + 1 == gf_pts.size()) // we only check this for the last gf_pt after saving everything
    {
      ROS_DEBUG_STREAM("Last point");
      std::unordered_set<lanelet::Lanelet> filtered = filterSuccessorLanelets(possible_lanelets, affected_lanelets, lanelet_map, routing_graph);
      ROS_DEBUG_STREAM("Got successor lanelets of size: " << filtered.size());
      affected_lanelets.insert(filtered.begin(), filtered.end());

      if (affected_lanelets.empty() && !possible_lanelets.empty())
      {
        ROS_DEBUG_STREAM("Checking if it is the edge case where only last point falls on a valid (correct direction) lanelet");
        for (auto llt: possible_lanelets)
        {
          ROS_DEBUG_STREAM("Evaluating lanelet: " << llt.id());
          lanelet::BasicLineString2d gf_dir_line({gf_pts[idx - 1].basicPoint2d(), gf_pts[idx].basicPoint2d()});
          lanelet::BasicLineString2d llt_boundary({(llt.leftBound2d().begin())->basicPoint2d(), (llt.rightBound2d().begin())->basicPoint2d()});
          
          // record the llts that are on the same dir
          if (boost::geometry::intersects(llt_boundary, gf_dir_line))
          {
            ROS_DEBUG_STREAM("Overlaps starting line... Picking llt: " << llt.id());
            affected_lanelets.insert(llt);
          }  
        }
      }
      break;
    } 

    ROS_DEBUG_STREAM("Checking possible lanelets");
    // check if each lines connecting end points of the llt is crossing with the line connecting current and next gf_pts
    for (auto llt: possible_lanelets)
    {
      ROS_DEBUG_STREAM("Evaluating lanelet: " << llt.id());
      lanelet::BasicLineString2d gf_dir_line({gf_pts[idx].basicPoint2d(), gf_pts[idx+1].basicPoint2d()});
      lanelet::BasicLineString2d llt_boundary({(llt.leftBound2d().end() -1)->basicPoint2d(), (llt.rightBound2d().end() - 1)->basicPoint2d()});
      
      // record the llts that are on the same dir
      if (boost::geometry::intersects(llt_boundary, gf_dir_line))
      {
        ROS_DEBUG_STREAM("Overlaps end line");
        affected_lanelets.insert(llt);
      }
      // check condition if two geofence points are in one lanelet then check matching direction and record it also
      else if (boost::geometry::within(gf_pts[idx+1].basicPoint2d(), llt.polygon2d()) && 
              affected_lanelets.find(llt) == affected_lanelets.end())
      { 
        ROS_DEBUG_STREAM("Within new lanelet");
        lanelet::BasicPoint2d median({((llt.leftBound2d().end() - 1)->basicPoint2d().x() + (llt.rightBound2d().end() - 1)->basicPoint2d().x())/2 , 
                                      ((llt.leftBound2d().end() - 1)->basicPoint2d().y() + (llt.rightBound2d().end() - 1)->basicPoint2d().y())/2});
        // turn into vectors
        Eigen::Vector2d vec_to_median(median);
        Eigen::Vector2d vec_to_gf_start(gf_pts[idx].basicPoint2d());
        Eigen::Vector2d vec_to_gf_end(gf_pts[idx + 1].basicPoint2d());

        // Get vector from start to external point
        Eigen::Vector2d start_to_median = vec_to_median - vec_to_gf_start;

        // Get vector from start to end point
        Eigen::Vector2d start_to_end = vec_to_gf_end - vec_to_gf_start;

        // Get angle between both vectors
        double interior_angle = carma_wm::geometry::getAngleBetweenVectors(start_to_median, start_to_end);

        ROS_DEBUG_STREAM("vec_to_median: " << vec_to_median.x() << ", " << vec_to_median.y());
        ROS_DEBUG_STREAM("vec_to_gf_start: " << vec_to_gf_start.x() << ", " << vec_to_gf_start.y());
        ROS_DEBUG_STREAM("vec_to_gf_end: " << vec_to_gf_end.x() << ", " << vec_to_gf_end.y());
        ROS_DEBUG_STREAM("start_to_median: " << start_to_median.x() << ", " << start_to_median.y());
        ROS_DEBUG_STREAM("start_to_end: " << start_to_end.x() << ", " << start_to_end.y());
        ROS_DEBUG_STREAM("interior_angle: " << interior_angle);
        // Save the lanelet if the direction of two points inside aligns with that of the lanelet

        if (interior_angle < M_PI_2 && interior_angle >= 0) 
          affected_lanelets.insert(llt); 
      }
      else
      {
        ROS_DEBUG_STREAM("------ Did not record anything...");
      }

    }
  
  }
  
  ROS_DEBUG_STREAM("affected_lanelets size: " << affected_lanelets.size());
  // Currently only returning lanelet, but this could be expanded to LanelerOrArea compound object 
  // by implementing non-const version of that LaneletOrArea
  lanelet::ConstLaneletOrAreas affected_parts;
  // return results in ascending downtrack order from the first point of geofence
  std::vector<std::pair<double, lanelet::Lanelet>> sorted_parts;
  for (auto llt : affected_lanelets)
  {
    sorted_parts.push_back(std::make_pair(carma_wm::geometry::trackPos(llt, gf_pts.front().basicPoint2d()).downtrack, llt));
  }
  std::sort(sorted_parts.begin(), sorted_parts.end(), [](const auto& x, const auto& y){return x.first > y.first;});

  for (auto pair : sorted_parts)
  {
    affected_parts.push_back(pair.second);
  }

  return affected_parts;
}

// helper function that filters successor lanelets of root_lanelets from possible_lanelets
std::unordered_set<lanelet::Lanelet> filterSuccessorLanelets(const std::unordered_set<lanelet::Lanelet>& possible_lanelets, const std::unordered_set<lanelet::Lanelet>& root_lanelets,
                                                              const lanelet::LaneletMapPtr& lanelet_map, std::shared_ptr<const lanelet::routing::RoutingGraph> routing_graph)
{
  if (!routing_graph) {
    throw std::invalid_argument("No routing graph available");
  }
  
  std::unordered_set<lanelet::Lanelet> filtered_lanelets;
  // we utilize routes to filter llts that are overlapping but not connected
  // as this is the last lanelet 
  // we have to filter the llts that are only geometrically overlapping yet not connected to prev llts
  for (auto recorded_llt: root_lanelets)
  {
    for (auto following_llt: routing_graph->following(recorded_llt, false))
    {
      auto mutable_llt = lanelet_map->laneletLayer.get(following_llt.id());
      auto it = possible_lanelets.find(mutable_llt);
      if (it != possible_lanelets.end())
      {
        filtered_lanelets.insert(mutable_llt);
      }
    }
  }
  return filtered_lanelets;
}


}  // namespace query

namespace utils
{

uint32_t get32BitId(uint16_t intersection_id, uint8_t signal_group_id)
{
  uint32_t temp = 0;
  temp |= intersection_id;
  temp = temp << 8;
  temp |= signal_group_id;
  return temp;
}


} // namespace utils
}  // namespace carma_wm