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
namespace utils
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
  int id = 0;  // closest ones are in the back
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
  auto possible_lanelets = getLaneletsFromPoint(semantic_map, point, n);
  for (auto llt : possible_lanelets)
  {
    semantic_map->laneletLayer.get(llt.id());
  }
  return possible_lanelets;
}

std::vector<lanelet::Lanelet> nonConnectedAdjacentLeft(const lanelet::LaneletMapPtr& semantic_map, const lanelet::BasicPoint2d& point,
                                                                    const unsigned int n)
{
  auto possible_lanelets = nonConnectedAdjacentLeft(semantic_map, point, n);
  for (auto llt : possible_lanelets)
  {
    semantic_map->laneletLayer.get(llt.id());
  }
  return possible_lanelets;
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

  ROS_DEBUG_STREAM("right ls size: " << input_lanelet.leftBound2d().size());
  
  auto point_downtrack = carma_wm::geometry::trackPos(input_lanelet, input_point).downtrack;
  ROS_DEBUG_STREAM("point_downtrack: " << point_downtrack);
  
  auto point_downtrack_ratio = point_downtrack / carma_wm::geometry::trackPos(input_lanelet, input_lanelet.centerline().back().basicPoint2d()).downtrack;
  ROS_DEBUG_STREAM("point_downtrack_ratio: " << point_downtrack_ratio);

  auto point_on_ls = input_lanelet.leftBound2d()[std::round((input_lanelet.leftBound2d().size()- 1) * point_downtrack_ratio) ];

  ROS_DEBUG_STREAM("x: " << point_on_ls.basicPoint2d().x() <<  ", y: " << point_on_ls.basicPoint2d().y());

  // point_on_opposite_lane coord is acquired by extrapolating the line from input_point to point_on_ls with same distance.
  // threfore point_on_opposite_lane.x() = input_point.x() + 2dx, where dx = point_on_ls.x() - input_point.x(). Here, one of input_point.x() cancels out, results in:
  auto point_on_opposite_lane = lanelet::BasicPoint2d{2 * point_on_ls.x() - input_point.x(), 2 * point_on_ls.y() - input_point.y()};

  ROS_DEBUG_STREAM("x: " << point_on_opposite_lane.x() <<  ", y: " << point_on_opposite_lane.y());

  auto opposite_lanelets = getLaneletsFromPoint(semantic_map, point_on_opposite_lane, n);

  for (auto llt: opposite_lanelets)
  {
      ROS_DEBUG_STREAM("potential_opposite_lanelets llt: "  << llt.id());
  }

  return opposite_lanelets;
}

uint32_t get32BitId(uint16_t intersection_id, uint8_t signal_group_id)
{
  uint32_t temp;
  temp |= intersection_id;
  temp = temp << 8;
  temp |= signal_group_id;
  return temp;
}

}  // namespace utils

}  // namespace carma_wm