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

namespace wz_strategic_plugin
{
//! @brief Struct to store the configuration settings for the WzStrategicPlugin class
struct WzStrategicPluginConfig
{
  //! The maximum allowable vehicle deceleration limit in m/s
  double vehicle_decel_limit = 2.0;

  //! A multiplier to apply to the maximum allowable vehicle deceleration limit so we plan under our capabilities
  double vehicle_decel_limit_multiplier = 0.75;

  //! The length parameter of the participant vehicle used to help calculate the distance before stopping at a red traffic signal
  double vehicle_length = 0.0;

  //! The minimum distance in meters that the vehicle can be at before requiring a transition to the APPROACH state
  double min_approach_distance = 30.0;

  //! A buffer infront of the stopping location which will still be considered a valid stop. Units in meters
  double stopping_location_buffer = 3.0;

  //! A buffer in seconds around the green phase which will reduce the phase length such that vehicle still considers it non-green
  double green_light_time_buffer = 2.0;

  //! The minimum period in seconds which a maneuver plan must cover if the plugin wishes to control the whole plan
  double min_maneuver_planning_period = 15.1;

  //! The name to use for this plugin during comminications with the arbitrator
  std::string strategic_plugin_name = "WorkZonePlugin";

  //! The name of the tactical plugin to use for Lane Following trajectory planning
  std::string lane_following_plugin_name = "InLaneCruisingPlugin";

  //! The name of the plugin to use for stop and wait trajectory planning
  std::string stop_and_wait_plugin_name = "StopAndWaitPlugin";

  //! The name of the plugin to use for intersection transit trajectory planning
  std::string intersection_transit_plugin_name = "IntersectionTransitPlugin";
};
}  // namespace localizer