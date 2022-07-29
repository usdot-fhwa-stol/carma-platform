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

namespace route
{

  /**
   * \brief Struct containing the algorithm configuration values for the route node
   */
  struct Config
  {
    double max_crosstrack_error = 2.0; // (Meters) Max cross track tolerance error
    double destination_downtrack_range = 10.0; // (Meters) Tracks how far a stop must be from the end to be accepted as a completed route
    double route_spin_rate = 10.0; // (Hz) Spin rate of the Route node
    int cte_max_count = 4; // Max number of consecutive timesteps outside of the route allowable before triggering a LEFT_ROUTE event
    std::string route_file_path = "NULL"; // Path to the directory that contains the route file(s) for CARMA

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "route::Config { " << std::endl
           << "max_crosstrack_error: " << c.max_crosstrack_error << std::endl
           << "destination_downtrack_range: " << c.destination_downtrack_range << std::endl
           << "route_spin_rate: " << c.route_spin_rate << std::endl
           << "cte_max_count: " << c.cte_max_count << std::endl
           << "route_file_path: " << c.route_file_path << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // route