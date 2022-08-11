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

namespace light_controlled_intersection_tactical_plugin
{

  /**
   * \brief Stuct containing the algorithm configuration values for light_controlled_intersection_tactical_plugin
   */
  struct Config
  {
    double centerline_sampling_spacing;
    double trajectory_time_length;
    int default_downsample_ratio;
    int turn_downsample_ratio;
    double curve_resample_step_size;
    int curvature_moving_average_window_size;
    int speed_moving_average_window_size;
    double back_distance;
    double buffer_ending_downtrack;
    double vehicle_decel_limit_multiplier;
    double vehicle_accel_limit_multiplier;
    double lat_accel_multiplier;
    double stop_line_buffer;
    double minimum_speed;
    double algorithm_evaluation_distance;
    double algorithm_evaluation_period;
    double vehicle_lateral_accel_limit;
    double vehicle_acceleration_limit;
    double vehicle_deceleration_limit; 

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "light_controlled_intersection_tactical_plugin::Config { " << std::endl
           << "centerline_sampling_spacing: " << c.centerline_sampling_spacing << std::endl
           << "trajectory_time_length: " << c.trajectory_time_length << std::endl
           << "default_downsample_ratio: " << c.default_downsample_ratio << std::endl
           << "turn_downsample_ratio: " << c.turn_downsample_ratio << std::endl
           << "curve_resample_step_size: " << c.curve_resample_step_size << std::endl
           << "curvature_moving_average_window_size: " << c.curvature_moving_average_window_size << std::endl
           << "speed_moving_average_window_size: " << c.speed_moving_average_window_size << std::endl
           << "back_distance: " << c.back_distance << std::endl
           << "buffer_ending_downtrack: " << c.buffer_ending_downtrack << std::endl
           << "vehicle_decel_limit_multiplier: " << c.vehicle_decel_limit_multiplier << std::endl
           << "vehicle_accel_limit_multiplier: " << c.vehicle_accel_limit_multiplier << std::endl
           << "lat_accel_multiplier: " << c.lat_accel_multiplier << std::endl
           << "stop_line_buffer: " << c.stop_line_buffer << std::endl
           << "minimum_speed: " << c.minimum_speed << std::endl
           << "algorithm_evaluation_distance: " << c.algorithm_evaluation_distance << std::endl
           << "algorithm_evaluation_period: " << c.algorithm_evaluation_period << std::endl
           << "vehicle_lateral_accel_limit: " << c.vehicle_lateral_accel_limit << std::endl
           << "vehicle_acceleration_limit: " << c.vehicle_acceleration_limit << std::endl
           << "vehicle_deceleration_limit: " << c.vehicle_deceleration_limit << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // light_controlled_intersection_tactical_plugin