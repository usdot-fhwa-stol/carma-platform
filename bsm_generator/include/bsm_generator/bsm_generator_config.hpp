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

namespace bsm_generator
{

  /**
   * \brief Stuct containing the algorithm configuration values for bsm_generator
   */
  struct Config
  {
    double bsm_generation_frequency = 10.0; // Rate (in Hz) at which a BSM is generated
    double bsm_id_change_period = 300.0; // BSM id change period (in sec)
    bool bsm_id_rotation_enabled = true; // Flag to enable/disable rotation of BSM ID during vehicle operations
    int bsm_message_id = 0; // Value is converted to a 4 element array of uint8_t where each byte of the parameter becomes one element of the array 
    double vehicle_length = 5.0; // Vehicle length (in meters)
    double vehicle_width = 2.0; // Vehicle width (in meters)


    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "bsm_generator::Config { " << std::endl
             << "bsm_generation_frequency: " << c.bsm_generation_frequency << std::endl
             << "bsm_id_change_period: " << c.bsm_id_change_period << std::endl
             << "bsm_id_rotation_enabled: " << c.bsm_id_rotation_enabled << std::endl
             << "bsm_message_id: " << c.bsm_message_id << std::endl
             << "vehicle_length: " << c.vehicle_length << std::endl
             << "vehicle_width: " << c.vehicle_width << std::endl
             << "}" << std::endl;
      return output;
    }
  };

} // bsm_generator