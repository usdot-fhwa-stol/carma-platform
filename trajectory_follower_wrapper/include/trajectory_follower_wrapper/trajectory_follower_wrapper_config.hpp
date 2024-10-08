#pragma once

/*
 * Copyright (C) 2024 LEIDOS.
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

namespace trajectory_follower_wrapper
{

  /**
   * \brief Stuct containing the algorithm configuration values for trajectory_follower_wrapper
   */
  struct TrajectoryFollowerWrapperConfig
  {
    double vehicle_response_lag = 0.2;       // (in seconds) approximation of the delay between sent vehicle commands and the vehicle beginning a meaningful acceleration to that command" - set and modified in carma config
    double incoming_cmd_time_threshold = 1.0; // (in second) acceptable time difference from autoware's contrl command to still use the command
    double vehicle_wheel_base = 2.79; //(in meter) distance between the centers of the front and rear axles - set and modified in carma config


    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const TrajectoryFollowerWrapperConfig &c)
    {
      output << "trajectory_follower_wrapper::TrajectoryFollowerWrapperConfig { " << std::endl
           << "vehicle_response_lag: " << c.vehicle_response_lag << std::endl
           << "vehicle_wheel_base: " << c.vehicle_wheel_base << std::endl
           << "incoming_cmd_time_threshold: " << c.incoming_cmd_time_threshold << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // trajectory_follower_wrapper
