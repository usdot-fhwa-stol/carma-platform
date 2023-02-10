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

#include <string>

namespace route_following_plugin
{
    /**
   * \brief Struct containing config values values for route_following_plugin
   */
   
  struct Config
  {
    // Minimal duration of maneuver, loaded from config file
    double min_plan_duration_ = 16.0;
    //Tactical plugin being used for planning lane change
    std::string lane_change_plugin_ =  "cooperative_lanechange";
    std::string stop_and_wait_plugin_ = "stop_and_wait_plugin";
    std::string vehicle_id = "vehicle_id";
    std::string lanefollow_planning_tactical_plugin_ = "inlanecruising_plugin";
    double route_end_point_buffer_ = 10.0;
    double accel_limit_ = 2.0;
    double lateral_accel_limit_ = 2.0;
    double stopping_accel_limit_multiplier_ = 0.5;
    double min_maneuver_length_ = 10.0; // Minimum length to allow for a maneuver when updating it for stop and wait

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "route_following_plugin::Config { " << std::endl
           << "min_plan_duration_: " << c.min_plan_duration_ << std::endl
           << "lane_change_plugin_: " << c.lane_change_plugin_  << std::endl
           << "stop_and_wait_plugin_: " << c.stop_and_wait_plugin_ << std::endl
           << "lanefollow_planning_tactical_plugin_: " << c.lanefollow_planning_tactical_plugin_ << std::endl
           << "route_end_point_buffer_: " << c.route_end_point_buffer_ << std::endl
           << "accel_limit_: " << c.accel_limit_ << std::endl
           << "lateral_accel_limit_: " << c.lateral_accel_limit_ << std::endl
           << "stopping_accel_limit_multiplier_: " << c.stopping_accel_limit_multiplier_ << std::endl
           << "min_maneuver_length_: " << c.min_maneuver_length_ << std::endl
           << "vehicle_id: " << c.vehicle_id << std::endl
           << "}" << std::endl;
      return output;
    }
  };
} // route_following_plugin