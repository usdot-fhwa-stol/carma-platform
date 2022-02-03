#pragma once

/*
 * Copyright (C) <SUB><year> LEIDOS.
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

namespace mobilitypath_visualizer
{

  /**
   * \brief Stuct containing the algorithm configuration values for <SUB><package_name>
   */
  struct Config
  { 
    double spin_rate = 10.0;
    double x = 0.5;
    double y = 0.5;
    double z = 1.0;
    double t = 3.0;

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "mobilitypath_visualizer::Config { " << std::endl
           << "spin_rate: " << c.spin_rate << std::endl
           << "x: " << c.x << std::endl
           << "x: " << c.y << std::endl
           << "x: " << c.z << std::endl
           << "x: " << c.t << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // mobilitypath_visualizer