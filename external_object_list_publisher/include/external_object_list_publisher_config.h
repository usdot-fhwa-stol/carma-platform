#pragma once

/*
 * Copyright (C) 2021 LEIDOS.
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

/*
 * Developed by the JFL Solutions LLC.
 * Author: Fang Zhou
 */

#include <iostream>

namespace external_object_list_publisher
{

  /**
   * \brief Stuct containing the algorithm configuration values for the emergency_pullover_strategic
   */
  struct ExternalObjectListPublisherConfig
  {
    double emergency_vehicle_distance   = 5.0; //m

    friend std::ostream& operator<<(std::ostream& output, const ExternalObjectListPublisherConfig& c)
    {
      output << "ExternalObjectListPublisherConfig { " << std::endl
            << "emergency_vehicle_distance: " << c.emergency_vehicle_distance << std::endl
            << "}" << std::endl; 
      return output;
    }
  };

}
