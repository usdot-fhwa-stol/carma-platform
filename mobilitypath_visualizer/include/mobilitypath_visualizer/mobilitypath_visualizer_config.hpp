#pragma once

/*
 * Copyright (C) 2019-2022LEIDOS.
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
    int timer_cb_rate = 10;
    double x = 0.5;
    double y = 0.5;
    double z = 1.0;
    double t = 3.0;
    std::string host_id = "";

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "mobilitypath_visualizer::Config { " << std::endl
           << "timer_cb_rate: " << c.timer_cb_rate << std::endl
           << "x: " << c.x << std::endl
           << "y: " << c.y << std::endl
           << "z: " << c.z << std::endl
           << "t: " << c.t << std::endl
           << "host_id: "<< c.host_id << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // mobilitypath_visualizer