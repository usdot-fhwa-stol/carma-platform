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

namespace gnss_to_map_convertor
{

  /**
   * \brief Stuct containing the algorithm configuration values for gnss_to_map_convertor
   */
  struct Config
  {
    std::string base_link_frame = "base_link"; // Frame ID of the base_link frame
    std::string map_frame = "map";             // Frame ID of the map frame
    std::string heading_frame = "ned_heading"; // Frame ID of the frame with +x aligned to vehicle heading and +z into the ground

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "gnss_to_map_convertor::Config { " << std::endl
           << "base_link_frame: " << c.base_link_frame << std::endl
           << "map_frame: " << c.map_frame << std::endl
           << "heading_frame: " << c.heading_frame << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // gnss_to_map_convertor
