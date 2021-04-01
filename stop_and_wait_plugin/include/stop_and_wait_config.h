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

#include <iostream>

/**
 * \brief Stuct containing the algorithm configuration values for the InLaneCruisingPlugin
 */
struct StopandWaitConfig
{
  double minimal_trajectory_duration = 6.0;    // Trajectory length in seconds
  double stop_timestep = 0.1;                  // Size of timesteps between stopped trajectory points
  int downsample_ratio = 4;                  // Amount to downsample input lanelet centerline data.
  double destination_downtrack_range_ = 10.0;  // Buffer around target end point TODO is this still needed?
  double accel_limit_multiplier = 0.5;         // Multiplier to compine with actual accel limit for target planning
  double accel_limit = 2.0;                    // Longitudinal acceleration limit of the vehicle

  friend std::ostream& operator<<(std::ostream& output, const StopandWaitConfig& c)
  {
    output << "StopandWaitConfig { " << std::endl
           << "minimal_trajectory_duration: " << c.minimal_trajectory_duration << std::endl
           << "stop_timestep: " << c.stop_timestep << std::endl
           << "downsample_ratio: " << c.downsample_ratio << std::endl
           << "destination_downtrack_range_: " << c.destination_downtrack_range_ << std::endl
           << "accel_limit_multiplier: " << c.accel_limit_multiplier << std::endl
           << "accel_limit: " << c.accel_limit << std::endl
           << "}" << std::endl;
    return output;
  }
};