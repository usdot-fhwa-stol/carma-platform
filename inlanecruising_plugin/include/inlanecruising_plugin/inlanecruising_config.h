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

/**
 * \brief Stuct containing the algorithm configuration values for the InLaneCruisingPlugin
 */
struct InLaneCruisingPluginConfig
{
  double trajectory_time_length = 6.0;     // Trajectory length in seconds
  double curve_resample_step_size = 1.0;   // Curve re-sampling step size in m
  int downsample_ratio = 12.0;              // Amount to downsample input lanelet centerline data.
                                           // Corresponds to saving each nth point.
  double minimum_speed = 2.2352;           // Minimum allowable speed in m/s
  double max_accel = 3;                  // Maximum allowable longitudinal acceleration in m/s^2
  double max_accel_multiplier = 0.85;     // Multiplier of max_accel to bring the value under max_accel
  double lat_accel_multiplier = 0.50;      // Multiplier of lat_accel to bring the value under lat_accel TODO: needs to be tuned
  double lateral_accel_limit = 2.5;        // Maximum allowable lateral acceleration m/s^2
  int moving_average_window_size = 5;      // Size of the window used in the moving average filter to smooth both the
                                           // computed curvature and output speeds
  double back_distance = 15;               // Number of meters behind the first maneuver that need to be included in points for curvature calculation
  bool enable_object_avoidance = false;    // Activate object avoidance logic
  
  friend std::ostream& operator<<(std::ostream& output, const InLaneCruisingPluginConfig& c)
  {
    output << "InLaneCruisingPluginConfig { " << std::endl
           << "trajectory_time_length: " << c.trajectory_time_length << std::endl
           << "curve_resample_step_size: " << c.curve_resample_step_size << std::endl
           << "downsample_ratio: " << c.downsample_ratio << std::endl
           << "minimum_speed: " << c.minimum_speed << std::endl
           << "max_accel: " << c.max_accel << std::endl
           << "max_accel_multiplier: " << c.max_accel_multiplier << std::endl
           << "lat_accel_multiplier: " << c.lat_accel_multiplier << std::endl
           << "lateral_accel_limit: " << c.lateral_accel_limit << std::endl
           << "moving_average_window_size: " << c.moving_average_window_size << std::endl
           << "back_distance: " << c.back_distance << std::endl
           << "enable_object_avoidance: " << c.enable_object_avoidance << std::endl
           << "}" << std::endl;
    return output;
  }
};