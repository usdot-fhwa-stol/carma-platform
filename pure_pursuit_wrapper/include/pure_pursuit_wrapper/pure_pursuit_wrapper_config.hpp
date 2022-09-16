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

namespace pure_pursuit_wrapper {
/**
 * \brief Struct containing the algorithm configuration values for the PurePursuitWrapperConfig
 */
struct PurePursuitWrapperConfig
{
  double vehicle_response_lag = 0.2;       // An approximation of the delay (sec) between sent vehicle commands and the vehicle begining a meaningful acceleration to that command
  double minimum_lookahead_distance = 6.0;
  double maximum_lookahead_distance = 100.0;
  double speed_to_lookahead_ratio = 2.0;
  bool is_interpolate_lookahead_point = true;
  bool is_delay_compensation = true;      // not to be confused with vehicle_response_lag, which is applied nonetheless
  double emergency_stop_distance = 0.1;
  double speed_thres_traveling_direction = 0.3;
  double dist_front_rear_wheels = 3.5;

  // integrator part
  double dt = 0.1;
  double integrator_max_pp = 0.0;
  double integrator_min_pp = 0.0;
  double Ki_pp = 0.0;
  bool is_integrator_enabled = false;
  
  friend std::ostream& operator<<(std::ostream& output, const PurePursuitWrapperConfig& c)
  {
    output << "PurePursuitWrapperConfig { " << std::endl
           << "vehicle_response_lag: " << c.vehicle_response_lag << std::endl
           << "minimum_lookahead_distance: " << c.minimum_lookahead_distance << std::endl
           << "maximum_lookahead_distance: " << c.maximum_lookahead_distance << std::endl
           << "speed_to_lookahead_ratio: " << c.speed_to_lookahead_ratio << std::endl
           << "is_interpolate_lookahead_point: " << c.is_interpolate_lookahead_point << std::endl
           << "is_delay_compensation: " << c.is_delay_compensation << std::endl
           << "emergency_stop_distance: " << c.emergency_stop_distance << std::endl
           << "speed_thres_traveling_direction: " << c.speed_thres_traveling_direction << std::endl
           << "dist_front_rear_wheels: " << c.dist_front_rear_wheels << std::endl
            // integrator part:
           << "dt: " << c.dt << std::endl
           << "integrator_max_pp: " << c.integrator_max_pp << std::endl
           << "integrator_min_pp: " << c.integrator_min_pp << std::endl
           << "Ki_pp: " << c.Ki_pp << std::endl
           << "is_integrator_enabled: " << c.is_integrator_enabled << std::endl
           << "}" << std::endl;
    return output;
  }
};
}