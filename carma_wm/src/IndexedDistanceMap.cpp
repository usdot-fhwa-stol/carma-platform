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

#include <carma_wm/IndexedDistanceMap.hpp>

namespace carma_wm
{
void IndexedDistanceMap::pushBack(const lanelet::LineString2d& ls)
{
  if (id_index_map.find(ls.id()) != id_index_map.end())
  {
    throw std::invalid_argument("IndexedDistanceMap already contains this ls");
  }
  std::vector<double> accumulated_dist;
  accumulated_dist.reserve(ls.size());
  accumulated_dist.push_back(0);
  size_t ls_i = accum_lengths.size();
  id_index_map[ls.front().id()] = std::make_pair(ls_i, 0);  // Add first point to id map
  for (size_t i = 0; i < ls.numSegments(); i++)
  {
    auto segment = ls.segment(i);
    double dist = lanelet::geometry::distance2d(segment.first, segment.second);  // length of line string
    accumulated_dist.push_back(dist + accumulated_dist.back());                  // Distance along linestring
    id_index_map[segment.second.id()] = std::make_pair(ls_i, i + 1);             // Add point id and index to map
  }
  auto tuple = std::make_tuple(accumulated_dist, totalLength());
  accum_lengths.push_back(tuple);
  id_index_map[ls.id()] = std::make_pair(ls_i, 0);  // Add linestirng id
}

std::pair<size_t, size_t> IndexedDistanceMap::getElementIndexByDistance(double distance, bool get_point) const {
  if (distance < 0) {
    throw std::invalid_argument("Distance must non-negative: " + std::to_string(distance));
  }
  if (distance > totalLength()) {
    throw std::invalid_argument("Distance cannot be greater than distance map length");
  }
  if (accum_lengths.size() == 0) {
    throw std::invalid_argument("No data available in distance map");
  }

  auto low = std::lower_bound (accum_lengths.begin(), accum_lengths.end(), distance, // Binary search to find the index 
    [](const std::tuple<std::vector<double>, double>& a, const double& b){return std::get<1>(a) < b;});
  
  size_t ls_index = 0;
  if (low == accum_lengths.end()) { // If we reached the end, it means we should pick the final point since we already checked the bounds
    ls_index = accum_lengths.size() - 1;
  } else {
    ls_index = std::max(low - accum_lengths.begin() - 1, (long int)0);
  }

  if (!get_point) { // If we are looking for the linestring index then return
    return std::make_pair(ls_index, (size_t) 0);
  }

  // If we are looking for the point index then calculate the index in the same manner as before
  double dist_to_ls = distanceToElement(ls_index);
  double relative_dist = distance - dist_to_ls;

  auto low_point = std::lower_bound (std::get<0>(accum_lengths[ls_index]).begin(), std::get<0>(accum_lengths[ls_index]).end(), relative_dist, // Binary search to find the point index 
    [](const double& a, const double& b){ return a < b; });
  
  size_t point_idx = std::max(low_point - std::get<0>(accum_lengths[ls_index]).begin() - 1, (long int)0);
  return std::make_pair(ls_index, point_idx);
}

double IndexedDistanceMap::elementLength(size_t index) const
{
  return std::get<0>(accum_lengths[index]).back();
}

double IndexedDistanceMap::distanceToElement(size_t index) const
{
  return std::get<1>(accum_lengths[index]);
}

double IndexedDistanceMap::distanceBetween(size_t index, size_t p1_index, size_t p2_index) const
{
  return fabs(distanceToPointAlongElement(index, p2_index) - distanceToPointAlongElement(index, p1_index));
}

double IndexedDistanceMap::distanceToPointAlongElement(size_t index, size_t point_index) const
{
  return std::get<0>(accum_lengths[index])[point_index];
}

double IndexedDistanceMap::totalLength() const
{
  if (accum_lengths.size() == 0)
  {
    return 0.0;
  }
  return distanceToElement(accum_lengths.size() - 1) + elementLength(accum_lengths.size() - 1);
}

std::pair<size_t, size_t> IndexedDistanceMap::getIndexFromId(const lanelet::Id& id) const
{
  return id_index_map.at(id);
}

size_t IndexedDistanceMap::size() const
{
  return accum_lengths.size();
}

size_t IndexedDistanceMap::size(size_t index) const
{
  return std::get<0>(accum_lengths[index]).size();
}

}  // namespace carma_wm
