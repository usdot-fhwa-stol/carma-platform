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

namespace cooperative_lanechange
{

  /**
   * \brief Stuct containing the algorithm configuration values for cooperative_lanechange
   */
  struct Config
  {
    double trajectory_time_length = 6.0;
    std::string control_plugin_name = "pure_pursuit";
    double minimum_speed = 2.2352;
    double max_accel = 1.5;
    double minimum_lookahead_distance = 5.0;
    double maximum_lookahead_distance = 25.0;
    double minimum_lookahead_speed = 2.8;
    double maximum_lookahead_speed = 13.9;
    double lateral_accel_limit = 1.5;
    int speed_moving_average_window_size = 5; 
    int curvature_moving_average_window_size = 9; 
    int curvature_calc_lookahead_count = 1;
    int downsample_ratio = 1;
    double destination_range = 0.0;
    double lanechange_time_out = 6.0;
    double min_timestep = 0.1;
    double starting_downtrack_range = 5.0;
    double starting_fraction = 0.2;
    double mid_fraction = 0.5;
    double min_desired_gap = 5.0;
    double desired_time_gap = 3.0;
    int turn_downsample_ratio = 0;
    double curve_resample_step_size = 1.0;
    double back_distance = 0.0;
    double buffer_ending_downtrack = 5.0;
    std::string vehicle_id = "DEFAULT_VEHICLE_ID";

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "cooperative_lanechange::Config { " << std::endl
           << "trajectory_time_length: " << c.trajectory_time_length << std::endl
           << "control_plugin_name: " << c.control_plugin_name << std::endl
           << "minimum_speed: " << c.minimum_speed << std::endl
           << "max_accel: " << c.max_accel << std::endl
           << "minimum_lookahead_distance: " << c.minimum_lookahead_distance << std::endl
           << "maximum_lookahead_distance: " << c.maximum_lookahead_distance << std::endl
           << "minimum_lookahead_speed: " << c.minimum_lookahead_speed << std::endl
           << "maximum_lookahead_speed: " << c.maximum_lookahead_speed << std::endl
           << "lateral_accel_limit: " << c.lateral_accel_limit << std::endl
           << "speed_moving_average_window_size: " << c.speed_moving_average_window_size << std::endl
           << "curvature_moving_average_window_size: " << c.curvature_moving_average_window_size << std::endl
           << "curvature_calc_lookahead_count: " << c.curvature_calc_lookahead_count << std::endl
           << "downsample_ratio: " << c.downsample_ratio << std::endl
           << "destination_range: " << c.destination_range << std::endl
           << "lanechange_time_out: " << c.lanechange_time_out << std::endl
           << "min_timestep: " << c.min_timestep << std::endl
           << "starting_downtrack_range: " << c.starting_downtrack_range << std::endl
           << "starting_fraction: " << c.starting_fraction << std::endl
           << "mid_fraction: " << c.mid_fraction << std::endl
           << "min_desired_gap: " << c.min_desired_gap << std::endl
           << "desired_time_gap: " << c.desired_time_gap << std::endl
           << "turn_downsample_ratio: " << c.turn_downsample_ratio << std::endl
           << "curve_resample_step_size: " << c.curve_resample_step_size << std::endl
           << "back_distance: " << c.back_distance << std::endl
           << "buffer_ending_downtrack: " << c.buffer_ending_downtrack << std::endl
           << "vehicle_id: " << c.vehicle_id << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // cooperative_lanechange