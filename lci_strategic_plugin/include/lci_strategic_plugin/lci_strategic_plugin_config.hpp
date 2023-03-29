#pragma once
/*
 * Copyright (C) 2023 LEIDOS.
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

  //! Minimum allowable speed TS algorithm in m/s
  double algo_minimum_speed = 2.2352;

  //! Double: Safety multiplier of planned allowable vehicle deceleration to use when stopping. This new deceleration makes vehicle decelerate earlier distance.
  //!      NOTE: Stacks on vehicle_decel_limit_multiplier and stopping uses max_decel; this distance is only used for calculating earlier downtrack
  double deceleration_fraction = 0.8;

  //! Double: Desired distance to stop buffer in meters
  double desired_distance_to_stop_buffer = 10.0;

  //! The minimum period in seconds which a maneuver plan must cover if the plugin wishes to control the whole plan
  double min_maneuver_planning_period = 15.1;

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
  std::string strategic_plugin_name = "lci_strategic_plugin";

  //! The name of the tactical plugin to use for Lane Following trajectory planning
  //! This plugin is used to apply trajectory smoothing algorithm BEFORE entering the intersection if within activation distance
  std::string lane_following_plugin_name = "light_controlled_intersection_tactical_plugin";

  //! The name of the plugin to use for stop and wait trajectory planning
  std::string stop_and_wait_plugin_name = "stop_and_wait_plugin";

  //! The name of the plugin to use for intersection transit trajectory planning
  //! This plugin is used to travel INSIDE the intersection where there is no trajectory smoothing algorithm active
  std::string intersection_transit_plugin_name = "intersection_transit_maneuvering";

  // Stream operator for this config
  friend std::ostream &operator<<(std::ostream &output, const LCIStrategicPluginConfig &c)
  {
    output << "LCIStrategicPluginConfig { " << std::endl
          << "vehicle_decel_limit: " << c.vehicle_decel_limit << std::endl
          << "vehicle_decel_limit_multiplier: " << c.vehicle_decel_limit_multiplier << std::endl
          << "vehicle_accel_limit: " << c.vehicle_accel_limit << std::endl
          << "vehicle_accel_limit_multiplier: " << c.vehicle_accel_limit_multiplier << std::endl
          << "min_approach_distance: " << c.min_approach_distance << std::endl
          << "trajectory_smoothing_activation_distance: " << c.trajectory_smoothing_activation_distance << std::endl
          << "stopping_location_buffer: " << c.stopping_location_buffer << std::endl
          << "green_light_time_buffer: " << c.green_light_time_buffer << std::endl
          << "algo_minimum_speed: " << c.algo_minimum_speed << std::endl
          << "deceleration_fraction: " << c.deceleration_fraction << std::endl
          << "desired_distance_to_stop_buffer: " << c.desired_distance_to_stop_buffer << std::endl
          << "min_maneuver_planning_period: " << c.min_maneuver_planning_period << std::endl
          << "reaction_time: " << c.reaction_time << std::endl
          << "min_gap: " << c.min_gap << std::endl
          << "enable_carma_streets_connection: " << c.enable_carma_streets_connection << std::endl
          << "mobility_rate: " << c.mobility_rate << std::endl
          << "vehicle_id: " << c.vehicle_id << std::endl
          << "strategic_plugin_name: " << c.strategic_plugin_name << std::endl
          << "lane_following_plugin_name: " << c.lane_following_plugin_name << std::endl
          << "stop_and_wait_plugin_name: " << c.stop_and_wait_plugin_name << std::endl
          << "intersection_transit_plugin_name: " << c.intersection_transit_plugin_name << std::endl
          << "}" << std::endl;
    return output;
  }
};
}  // namespace localizer