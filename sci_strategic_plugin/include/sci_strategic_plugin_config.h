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

#include <string>

namespace sci_strategic_plugin
{
//! @brief Struct to store the configuration settings for the WzStrategicPlugin class
struct SCIStrategicPluginConfig
{
  //! The maximum allowable vehicle deceleration limit in m/s
  double vehicle_decel_limit = 2.0;

  //! A multiplier to apply to the maximum allowable vehicle deceleration limit so we plan under our capabilities
  double vehicle_decel_limit_multiplier = 0.5;

  //! The maximum allowable vehicle acceleration limit in m/s
  double vehicle_accel_limit = 2.0;

  //! A multiplier to apply to the maximum allowable vehicle acceleration limit so we plan under our capabilities
  double vehicle_accel_limit_multiplier = 0.5;

  //! A buffer infront of the stopping location which will still be considered a valid stop
  double stop_line_buffer = 3.0;

  //! The minimum period in seconds which a maneuver plan must cover if the plugin wishes to control the whole plan
  double min_maneuver_planning_period = 15.1;

  // Double: Approximate update time interval of carma streets
  double delta_t = 1.0;

  // Double: Minimum inter-vehicle gap
  double min_gap = 10.0;

  // Double: Length od the vehicle
  double veh_length = 4.0;

  // Double: Vehicle reaction time to a received schedule in seconds (approximate value, only used for communication with the schedule)
  double reaction_time = 5.0;

  // Double: The distance vehicle drives to be considered out of intersections
  double intersection_exit_zone_length = 15.0;

  //! The name to use for this plugin during comminications with the arbitrator
  std::string strategic_plugin_name = "SCIStrategicPlugin";

  //! The name of the tactical plugin to use for Lane Following trajectory planning
  std::string lane_following_plugin_name = "StopControlledIntersectionTacticalPlugin";

  //! The name of the plugin to use for stop and wait trajectory planning
  std::string stop_and_wait_plugin_name = "StopAndWaitPlugin";

  //! The name of the plugin to use for intersection transit trajectory planning
  std::string intersection_transit_plugin_name = "IntersectionTransitPlugin";

  //! License plate of the vehicle.
  std::string vehicle_id = "default_id";
};
}  // namespace localizer