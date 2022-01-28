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

#pragma once

#include <unordered_set>
#include <geometry/spatial_hash.hpp>
#include <geometry/spatial_hash_config.hpp>
#include "approximate_intersection/config.hpp"

namespace approximate_intersection
{

  /**
   * \brief LookupGrid class implements a fast occupancy grid creation and intersection data structure.
   *        The user provides 2d min/max bounds on the grid as well as cell side length (cells are always square).
   *        The user can then add points into the grid. Cells which contain points are marked as occupied.
   *        Once the grid is populated, intersections can be checked against.
   *        If the queried point lands in an occupied cell the intersection is reported as true.
   * 
   * \tparam PointT The type of 2d point which the grid will be built from. Must have publicly accessible .x and .y members. 
   */
  template<class PointT>
  class LookupGrid
  {
  protected:
    using Hasher = autoware::common::geometry::spatial_hash::Config2d;
    using HashIndex = autoware::common::geometry::spatial_hash::Index;
    

    //! Configuration
    Config config_;

    //! Spatial hasher which defines the grid
    Hasher hasher_;

    //! The set of occupied cells identified by their hash index
    std::unordered_set<HashIndex> occupied_cells_;

    //! Static placeholder value for the Autoware.Auto Hasher in use which requires this value but does not utilize it.
    static constexpr size_t PLACE_HOLDER_CAPACITY = 1;

  public:

    /**
     * \brief Default constructor
     *        Note: This constructor is provided for convenience but users should normally use the constructor which takes a Config object.
     *      
     */
    LookupGrid():LookupGrid(Config()) {};
    
    /**
     * \brief Constructor
     * 
     * \param config The configuration for the grid.
     */
    LookupGrid(Config config):
      config_(config),
      hasher_(config.min_x, config.max_x, config.min_y, config.max_y, config.cell_side_length, PLACE_HOLDER_CAPACITY)
    {
      double dx = config.max_x - config.min_x;
      double dy = config.max_y - config.min_y;
      
      double rough_cell_side_count = 0;
      
      if (dx > dy) {
        rough_cell_side_count = dx / config.cell_side_length;
      } else {
        rough_cell_side_count = dy / config.cell_side_length;
      }

      occupied_cells_.reserve(rough_cell_side_count * rough_cell_side_count);
    }

    /**
     * \brief Return the current config
     * 
     * \return the config in use by this object
     */ 
    Config get_config() {
      return config_;
    }

    /**
     * \brief Checks if a point lies within an occupied cell of the grid
     * 
     * \param point The point to check for intersection
     * 
     * \return True if the point lies within an occupied cell, false otherwise
     */ 
    bool intersects(PointT point) const {

      HashIndex cell = hasher_.bin(point.x, point.y, 0);

      if (occupied_cells_.find(cell) != occupied_cells_.end()) {
        return true;
      }

      return false;
    }

    /**
     * \brief Adds a point to the grid. The point must lie within the grid bounds.
     *        The cell which the point lies in is marked as occupied.
     * 
     * \param point The point to add to the grid
     */
    void insert(PointT point) {
      HashIndex cell = hasher_.bin(point.x, point.y, 0);
      occupied_cells_.insert(cell);
    }

  };

} // approximate_intersection
