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

namespace mobilitypath_publisher
{

  /**
   * \brief Stuct containing the algorithm configuration values for mobilitypath_publisher
   */
  struct Config
  {
    double path_pub_rate = 10.0; // Rate (in Hz) at which the MobilityPath message is published
    std::string vehicle_id = "DEFAULT-VEHICLE-ID";  // Sender's static ID, which is its license plate

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "mobilitypath_publisher::Config { " << std::endl
           << "path_pub_rate: " << c.path_pub_rate << std::endl
           << "vehicle_id: " << c.vehicle_id << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // mobilitypath_publisher