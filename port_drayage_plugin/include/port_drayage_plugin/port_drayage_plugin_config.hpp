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

namespace port_drayage_plugin
{

  /**
   * \brief Struct containing the algorithm configuration values for port_drayage_plugin
   */
  struct Config
  {
    std::string cmv_id = ""; // The CMV's static ID, which is its license plate

    std::string cargo_id = ""; // The ID of the cargo that is being carried at system start-up. Empty if CMV is not carrying cargo.

    bool enable_port_drayage = false; // Flag to activate/deactivate port drayage operations. If false, the port drayage state machine will remain INACTIVE

    bool starting_at_staging_area = true; // Flag to indicate CMV's first destination; 'true' indicates Staging Area Entrance, 'false' indicates Port Entrance

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "port_drayage_plugin::Config { " << std::endl
           << "cmv_id: " << c.cmv_id << std::endl
           << "cargo_id: " << c.cargo_id << std::endl
           << "enable_port_drayage: " << c.enable_port_drayage << std::endl
           << "starting_at_staging_area: " << c.starting_at_staging_area << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // port_drayage_plugin