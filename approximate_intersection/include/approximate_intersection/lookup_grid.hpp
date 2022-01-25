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

    float32_t compute_side_length(Config config) const {
      float32_t dx = config.max_x - config.min_x;
      float32_t dy = config.max_y - config.min_y;

      if (dx >= dy) {
        return dx / (float32_t) config.resolution;
      } else {
        return dy / (float32_t) config.resolution;
      }
    }

    static constexpr size_t PLACE_HOLDER_CAPACITY = 1;

  public:
    /**
     * \brief Grid constructor 
     */
    LookupGrid(Config config):
      config_(config),
      hasher_(config.min_x, config.max_x, config.min_y, config.max_y, compute_side_length(config), PLACE_HOLDER_CAPACITY)
    {
      occupied_cells_.reserve(config.resolution * config.resolution);
      std::cerr << "Side Length " << compute_side_length(config) << std::endl;
    }

    bool intersects(PointT point) {

      HashIndex cell = hasher_.bin(point.x, point.y, 0);
      std::cerr << "Cell " << cell << std::endl;

      if (occupied_cells_.find(cell) != occupied_cells_.end()) {
        return true;
      }

      return false;
    }

    void insert(PointT point) {
      HashIndex cell = hasher_.bin(point.x, point.y, 0);
      std::cerr << "Occupied Cell " << cell << std::endl;
      occupied_cells_.insert(cell);
    }

  };

} // approximate_intersection
