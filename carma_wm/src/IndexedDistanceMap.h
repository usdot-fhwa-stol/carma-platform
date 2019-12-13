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

#include <vector>
#include <tuple>
#include <utility>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <unordered_map>
#include <math.h>

namespace carma_wm {
  /*!
  * \brief O(1) distance lookup structure for quickly accessing route distance information
  *
  * This class is meant to be used when a route update occurs to precompute distances along the route to support rapid queries later on
  * Insertion is O(n) where n is the number of points in the linestring
  * This structure does not support element reinsertion
  * Linestrings added to this structure should be fully unique (NO DUPLICATE IDs for linestring or point objects)
  *  
  * 
  * NOTE: Pre-computing route distances make queries much faster but could slow down route loading for large routes and cause them to use more memory.
  */
  class IndexedDistanceMap {
    private:
      // Distance storage structure
      // A vector of tuples where the first vector is indexed by line segment and the second vector is index by point index on that line segment. 
      // The double value in the tuple stores the total along-line distance to the start of that line segment from the first point on the first line segment
      // The double value inside the interior vector store the along-line distance of the point from the start of its line segment 
      std::vector<std::tuple<std::vector<double>, double>> accum_lengths;

      // Id mapping structure
      // Sores the linestring and point index's as values with their lanelet Ids as the key
      std::unordered_map<lanelet::Id, std::pair<size_t, size_t>> id_index_map;

    public:
      
      /*!
      * \brief Add a linestring to this structure. This function will iterate over the line string to compute distances between each segment. 
      *
      * \throws std::invalid_argument If the provided linstring was already added to this structure
      * 
      * \param ls The linestring to add
      */
      void pushBack(const lanelet::LineString2d& ls) {
        if (id_index_map.find(ls.id()) != id_index_map.end()) {
          throw std::invalid_argument("IndexedDistanceMap already contains this ls");
        }
        std::vector<double> accumulated_dist;
        accumulated_dist.reserve(ls.size());
        accumulated_dist.push_back(0);
        size_t ls_i = accum_lengths.size();
        id_index_map[ls.front().id()] = std::make_pair(ls_i, 0); // Add first point to id map
        for (size_t i = 0; i < ls.numSegments(); i++) {
          auto segment = ls.segment(i);
          double dist = lanelet::geometry::distance2d(segment.first, segment.second); // length of line string
          accumulated_dist.push_back(dist + accumulated_dist.back()); // Distance along linestring
          id_index_map[segment.second.id()] = std::make_pair(ls_i, i + 1); // Add point id and index to map
        }
        auto tuple = std::make_tuple(accumulated_dist, totalLength());
        accum_lengths.push_back(tuple);
        id_index_map[ls.id()] = std::make_pair(ls_i, 0); // Add linestirng id
      }
      
      /*! 
      * \brief Get the length of the linestring located at the provided index
      * 
      * NOTE: No bounds checking is performed
      *
      * \param index The index of the linestring
      * 
      * \return The length of the linestring
      */
      double elementLength(size_t index) const {
        return std::get<0>(accum_lengths[index]).back();
      }
      
      /*! 
      * \brief Get the distance to the start of the linestring at the specified index
      * 
      * NOTE: No bounds checking is performed
      *
      * \param index The index of the linestring
      * 
      * \return The along-line distance to the start of the linestring at index
      */
      double distanceToElement(size_t index) const {
        return std::get<1>(accum_lengths[index]);
      }

      /*! 
      * \brief Get the distance between two points on the same linestring
      * 
      * NOTE: No bounds checking is performed
      *
      * \param index The index of the linestring
      * \param p1_index The index of the first point on the linestring
      * \param p2_index The index of the second point
      * 
      * \return The along-line distance between the two provided points on the same linestring
      */
      double distanceBetween(size_t index, size_t p1_index, size_t p2_index) const {
        return fabs(distanceToPointAlongElement(index, p2_index) - distanceToPointAlongElement(index, p1_index));
      }
      
      /*! 
      * \brief Get the along-line distance to the point on the provided linestring
      * 
      * NOTE: No bounds checking is performed
      *
      * \param index The linestring index
      * \param point_index The point index in the linestring at index
      * 
      * \return The along-line distance to the point from the start of the requested linestring
      */
      double distanceToPointAlongElement(size_t index, size_t point_index) const {
        return std::get<0>(accum_lengths[index])[point_index];
      }

      /*! 
      * \brief Returns the total along-line length of this structure
      * 
      * \return The length
      */
      double totalLength() const {
        if (accum_lengths.size() == 0) {
          return 0.0;
        }
        return distanceToElement(accum_lengths.size() - 1) + elementLength(accum_lengths.size() - 1);
      }

      /*! 
      * \brief Returns the indexes of the element identified by the provided Id
      *        NOTE: It is up to the user to know if the id is for a linestring or a point. By default index values will be 0
      * 
      * \throws std::out_of_range if the container does not have an element with the specified id
      * 
      * \return An std::pair of the matched index where the first element is the linestring index and the second is the point index in that linestring
      */
      std::pair<size_t, size_t> getIndexFromId(const lanelet::Id& id) const {
        return id_index_map.at(id);
      }

      /*! 
      * \brief Returns number of linestrings in this structure
      * 
      * \return The element count
      */
      size_t size() const {
        return accum_lengths.size(); 
      }

      /*! 
      * \brief Returns the size of the linestring at the specified index
      * 
      * \return The linestring point count
      */
      size_t size(size_t index) const {
        return std::get<0>(accum_lengths[index]).size(); 
      }
  };
}
