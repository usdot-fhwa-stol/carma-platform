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

namespace lci_strategic_plugin
{
//! @brief Struct to store the configuration settings for the LCIStrategicPlugin class
struct LCIStrategicPluginConfig
{
  //! The maximum allowable vehicle deceleration limit in m/s
  double vehicle_decel_limit = 2.0;

  //! A multiplier to apply to the maximum allowable vehicle deceleration limit so we plan under our capabilities
  double vehicle_decel_limit_multiplier = 0.75;

  //! The maximum allowable vehicle acceleration limit in m/s
  double vehicle_accel_limit = 2.0;

  //! A multiplier to apply to the maximum allowable vehicle acceleration limit so we plan under our capabilities
  double vehicle_accel_limit_multiplier = 0.75;

  //! The minimum distance in meters that the vehicle can be at before requiring a transition to the APPROACH state
  double min_approach_distance = 30.0;

  //! Downtrack distance until nearest intersection where the Trajectory Smoothing algorithm should activate
  double trajectory_smoothing_activation_distance = 200.0;

  //! A buffer infront of the stopping location which will still be considered a valid stop. Units in meters
  double stopping_location_buffer = 3.0;

  //! A buffer in seconds around the green phase which will reduce the phase length such that vehicle still considers it non-green
  double green_light_time_buffer = 0.0;

  //! Double: A buffer in seconds starting green phase which is still considered valid stop if safety requires it
  double stop_light_time_buffer = 0.0;

  //! Minimum allowable speed TS algorithm in m/s
  double algo_minimum_speed = 2.2352;

  //! Minimum allowable speed in m/s
  double absolute_minimum_speed = 2.2352;

  //! Double: Safety multiplier of planned allowable vehicle deceleration to use when stopping. This new deceleration makes vehicle decelerate earlier distance.
  //!      NOTE: Stacks on vehicle_decel_limit_multiplier and stopping uses max_decel; this distance is only used for calculating earlier downtrack
  double deceleration_fraction = 0.8;

  //! Double: Desired distance to stop buffer in meters
  double desired_distance_to_stop_buffer = 10.0;

  //! The minimum period in seconds which a maneuver plan must cover if the plugin wishes to control the whole plan
  double min_maneuver_planning_period = 15.1;

  //! Double: Approximate update time interval of carma streets
  double carma_streets_update_interval = 1.0;
  
  //! Double: Vehicle reaction time to a received schedule in seconds (approximate value, only used for communication with the schedule)
  double reaction_time = 2.0;

  //! Double: Minimum inter-vehicle gap
  double min_gap = 10.0;

  //! Bool: Enable carma streets connection
  bool  enable_carma_streets_connection = false;

  //! Double: Mobility operation rate
  double mobility_rate = 10.0;

  //! License plate of the vehicle.
  std::string vehicle_id = "default_id";

  //! The name to use for this plugin during comminications with the arbitrator
  std::string strategic_plugin_name = "LCIStrategicPlugin";

  //! The name of the tactical plugin to use for Lane Following trajectory planning
  //! This plugin is used to apply trajectory smoothing algorithm BEFORE entering the intersection if within activation distance
  std::string lane_following_plugin_name = "LightControlledIntersectionTacticalPlugin";

  //! The name of the plugin to use for stop and wait trajectory planning
  std::string stop_and_wait_plugin_name = "StopAndWaitPlugin";

  //! The name of the plugin to use for intersection transit trajectory planning
  //! This plugin is used to travel INSIDE the intersection where there is no trajectory smoothing algorithm active
  std::string intersection_transit_plugin_name = "IntersectionTransitPlugin";
};
}  // namespace localizer