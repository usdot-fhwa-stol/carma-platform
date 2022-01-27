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
   * \brief TODO for USER: Add class description
   * 
   */
  template<class PointT>
  class LookupGrid
  {
  protected:
    using Hasher = autoware::common::geometry::spatial_hash::Config2d;
    using HashIndex = autoware::common::geometry::spatial_hash::Index;
    

    // Configuration
    Config config_;

    Hasher hasher_;

    std::unordered_set<HashIndex> occupied_cells_;

    static constexpr size_t PLACE_HOLDER_CAPACITY = 1;

  public:

    LookupGrid():LookupGrid(Config()) {};
    
    /**
     * \brief Grid constructor 
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

    Config get_config() {
      return config_;
    }

    bool intersects(PointT point) {

      HashIndex cell = hasher_.bin(point.x, point.y, 0);

      if (occupied_cells_.find(cell) != occupied_cells_.end()) {
        return true;
      }

      return false;
    }

    void insert(PointT point) {
      HashIndex cell = hasher_.bin(point.x, point.y, 0);
      occupied_cells_.insert(cell);
    }

  };

} // approximate_intersection
