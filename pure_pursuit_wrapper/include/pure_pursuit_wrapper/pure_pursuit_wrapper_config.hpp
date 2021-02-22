#pragma once

/*
 * Copyright (C) 2020 LEIDOS.
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

namespace pure_pursuit_wrapper {
/**
 * \brief Stuct containing the algorithm configuration values for the PurePursuitWrapperConfig
 */
struct PurePursuitWrapperConfig
{
  double vehicle_response_lag = 0.2;       // An approximation of the delay (sec) between sent vehicle commands and the vehicle begining a meaningful acceleration to that command

  friend std::ostream& operator<<(std::ostream& output, const PurePursuitWrapperConfig& c)
  {
    output << "PurePursuitWrapperConfig { " << std::endl
           << "vehicle_response_lag: " << c.vehicle_response_lag << std::endl
           << "}" << std::endl;
    return output;
  }
};
}