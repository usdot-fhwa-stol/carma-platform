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

namespace points_map_filter
{

  /**
   * \brief Stuct containing the algorithm configuration values for points_map_filter
   */
  struct Config
  {
    //! The side length of the 2d cells which are used to discretize the filter space
    double cell_side_length = 3.0;

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "points_map_filter::Config { " << std::endl
           << "cell_side_length: " << c.cell_side_length << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // points_map_filter