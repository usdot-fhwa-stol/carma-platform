#pragma once

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

#include <iostream>
#include <vector>

namespace approximate_intersection
{

  /**
   * \brief Stuct containing the algorithm configuration values for approximate_intersection
   */
  struct Config
  {

    //! Maximum x dimension
    float32_t min_x = -1000;
    
    //! Maximum x dimension
    float32_t max_x = 1000;

    //! Maximum y dimension
    float32_t min_y = -1000;

    //! Maximum y dimension
    float32_t max_y = 1000;

    //! Cell size length
    size_t cell_side_length = 100;

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "approximate_intersection::Config { " << std::endl
           << "min_x: " << c.min_x << std::endl
           << "max_x: " << c.max_x << std::endl
           << "min_y: " << c.min_y << std::endl
           << "max_y: " << c.max_y << std::endl
           << "cell_side_length: " << c.cell_side_length << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // approximate_intersection