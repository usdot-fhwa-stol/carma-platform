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

namespace stop_and_dwell_strategic_plugin
{
//! @brief Struct to store the configuration settings for the WzStrategicPlugin class
struct StopAndDwellStrategicPluginConfig
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

  // Double: Length od the vehicle
  double veh_length = 4.0;

  // Double: The distance bus drives to be considered out of bus stop
  double bus_line_exit_zone_length = 15.0;

  //! The name to use for this plugin during comminications with the arbitrator
  std::string strategic_plugin_name = "stop_and_dwell_strategic_plugin";

  //! The name of the tactical plugin to use for Lane Following trajectory planning
  std::string lane_following_plugin_name = "inlanecruising_plugin";

  //! The name of the plugin to use for stop and wait trajectory planning
  std::string stop_and_wait_plugin_name = "stop_and_wait_plugin";

  //! License plate of the vehicle.
  std::string vehicle_id = "default_id";

  //! Activation distance of stop and dwell plugin
  double activation_distance = 200.0;

  //! Dwell time is the time to stop at the bus stop
  double dwell_time = 0.0;

  //! Double: Safety multiplier (must be less than 1.0) of planned allowable vehicle deceleration to use when stopping. This new deceleration makes vehicle decelerate earlier distance.
  double deceleration_fraction = 0.7;

  //! Double: Desired distance to stop buffer in meters
  double desired_distance_to_stop_buffer = 15.0;

  

  // Stream operator for this config
  friend std::ostream &operator<<(std::ostream &output, const StopAndDwellStrategicPluginConfig &c)
  {
    output << "StopAndDwellStrategicPluginConfig { " << std::endl
          << "vehicle_decel_limit: " << c.vehicle_decel_limit << std::endl
          << "vehicle_decel_limit_multiplier: " << c.vehicle_decel_limit_multiplier << std::endl
          << "vehicle_accel_limit: " << c.vehicle_accel_limit << std::endl
          << "vehicle_accel_limit_multiplier: " << c.vehicle_accel_limit_multiplier << std::endl
          << "stop_line_buffer: " << c.stop_line_buffer << std::endl
          << "min_maneuver_planning_period: " << c.min_maneuver_planning_period << std::endl
          << "veh_length: " << c.veh_length << std::endl
          << "bus_line_exit_zone_length: " << c.bus_line_exit_zone_length << std::endl
          << "strategic_plugin_name: " << c.strategic_plugin_name << std::endl
          << "lane_following_plugin_name: " << c.lane_following_plugin_name << std::endl
          << "stop_and_wait_plugin_name: " << c.stop_and_wait_plugin_name << std::endl
          << "vehicle_id: " << c.vehicle_id << std::endl
          << "activation_distance: " << c.activation_distance << std::endl
          << "dwell_time: " << c.dwell_time << std::endl
          << "deceleration_fraction: " << c.deceleration_fraction << std::endl
          << "desired_distance_to_stop_buffer: " << c.desired_distance_to_stop_buffer << std::endl
          << "}" << std::endl;
    return output;
  }
};
}  // namespace stop_and_dwell_strategic_plugin