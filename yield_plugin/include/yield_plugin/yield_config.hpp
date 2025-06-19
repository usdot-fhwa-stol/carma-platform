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
 * \brief Stuct containing the algorithm configuration values for the YieldPluginConfig
 */
struct YieldPluginConfig
{
  double acceleration_adjustment_factor = 4.0;  // Adjustment factor for safe and comfortable acceleration/deceleration
  double time_horizon_until_collision_to_commit_to_stop_in_s = 4.0;  // Time horizon until collision to commit to last stopping trajectory
  double time_horizon_since_obj_clearance_to_start_moving_in_s = 4.0;  // Time horizon since stopping and verified obstacle is cleard to start moving again
  double on_route_vehicle_collision_horizon_in_s = 10.0;        // time horizon for collision detection in s
  double obstacle_zero_speed_threshold_in_ms = 0.25;       // Minimum speed threshold for moving obstacle in m/s to be considered stopped
  double min_obj_avoidance_plan_time_in_s = 2.0;                     // minimum object avoidance planning time in s
  double yield_max_deceleration_in_ms2 = 3.0;  // max deceleration value in m/s^2
  double minimum_safety_gap_in_meters = 2.0;                       // minimum safety gap in m
  double vehicle_length = 5.0;              // Host vehicle length in m
  double vehicle_height = 3.0;              // Host vehicle height in m
  double vehicle_width = 2.5;              // Host vehicle width in m
  double max_stop_speed_in_ms = 1.0;           //  Maximum speed value to consider the ego vehicle stopped in m/s
  bool enable_cooperative_behavior = true;       //parameter to enable cooperative behavior
  bool always_accept_mobility_request = true;       //parameter to always accept mobility request
  std::string vehicle_id = "DEFAULT_VEHICLE_ID";         // Vehicle id is the license plate of the vehicle
  int acceptable_passed_timesteps = 10;              // acceptable number of timesteps to use the latest known mobility request before switching to yield
  double intervehicle_collision_distance_in_m = 10.0;    //Intervehicle distance that is considered a collision
  int consecutive_clearance_count_for_passed_obstacles_threshold = 5; //Number of consecutive times the vehicle detects that the obstacle is behind to confirm the obstacle is actually behind
  double safety_collision_time_gap_in_s = 2.0;          // Time gap to finish planning a yield earlier than collision time
  bool enable_adjustable_gap = true;          // Flag to enable yield plugin to check for adjustable gap for example digital gap from map
  int acceptable_urgency = 5;                 //Minimum urgency value to consider the mobility request
  double speed_moving_average_window_size = 3.0;  //Window size for speed moving average filter
  double collision_check_radius_in_m = 150.0;  //Radius to check for potential collision

  friend std::ostream& operator<<(std::ostream& output, const YieldPluginConfig& c)
  {
    output << "YieldPluginConfig { " << std::endl
          << "acceleration_adjustment_factor: " << c.acceleration_adjustment_factor << std::endl
          << "time_horizon_until_collision_to_commit_to_stop_in_s: " << c.time_horizon_until_collision_to_commit_to_stop_in_s << std::endl
          << "time_horizon_since_obj_clearance_to_start_moving_in_s: " << c.time_horizon_since_obj_clearance_to_start_moving_in_s << std::endl
          << "on_route_vehicle_collision_horizon_in_s: " << c.on_route_vehicle_collision_horizon_in_s << std::endl
          << "obstacle_zero_speed_threshold_in_ms: " << c.obstacle_zero_speed_threshold_in_ms << std::endl
          << "yield_max_deceleration_in_ms2: " << c.yield_max_deceleration_in_ms2 << std::endl
          << "minimum_safety_gap_in_meters: " << c.minimum_safety_gap_in_meters << std::endl
          << "vehicle_length: " << c.vehicle_length << std::endl
          << "vehicle_height: " << c.vehicle_height << std::endl
          << "vehicle_width: " << c.vehicle_width << std::endl
          << "max_stop_speed_in_ms: " << c.max_stop_speed_in_ms << std::endl
          << "enable_cooperative_behavior: " << c.enable_cooperative_behavior << std::endl
          << "always_accept_mobility_request: " << c.always_accept_mobility_request << std::endl
          << "vehicle_id: " << c.vehicle_id << std::endl
          << "acceptable_passed_timesteps: " << c.acceptable_passed_timesteps << std::endl
          << "intervehicle_collision_distance_in_m: " << c.intervehicle_collision_distance_in_m << std::endl
          << "consecutive_clearance_count_for_passed_obstacles_threshold: " << c.consecutive_clearance_count_for_passed_obstacles_threshold << std::endl
          << "safety_collision_time_gap_in_s: " << c.safety_collision_time_gap_in_s << std::endl
          << "enable_adjustable_gap: " << c.enable_adjustable_gap << std::endl
          << "acceptable_urgency: " << c.acceptable_urgency << std::endl
          << "speed_moving_average_window_size: " << c.speed_moving_average_window_size << std::endl
          << "collision_check_radius_in_m: " << c.collision_check_radius_in_m << std::endl
          << "}" << std::endl;
    return output;
  }
};
