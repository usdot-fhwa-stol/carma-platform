#pragma once

/*
 * Copyright (C) 2024 LEIDOS.
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

namespace platoon_control
{

/**
 * \brief Stuct containing the algorithm configuration values for the PlatooningControlPlugin
 */
  struct PlatooningControlPluginConfig
  {
    double stand_still_headway_m = 12.0; // Standstill gap between vehicles (m)
    double max_accel_mps2 = 2.5;  // Maximum acceleration absolute value used in controller filters (m/s^2)
    double kp = 0.5;  // Proportional weight for PID controller
    double kd = -0.5;  // Derivative Weight for PID controller
    double ki = 0.0;  // Integral weight for PID controller
    double max_delta_speed_per_timestep = 2;  // Max value to restrict speed adjustment at one time step (limit on delta_v) (m/s)
    double min_delta_speed_per_timestep = -10; // Min value to restrict speed adjustment at one time step (limit on delta_v) (m/s)

    double adjustment_cap_mps = 10;  // Adjustment cap for speed command (m/s)
    int cmd_timestamp_ms = 100;  // Timestamp to calculate ctrl commands (ms)
    double integrator_max = 100; // Max limit for integrator term
    double integrator_min = -100;  // Max limit for integrator term
    std::string vehicle_id = "DEFAULT_VEHICLE_ID";         // Vehicle id is the license plate of the vehicle
    int     shutdown_timeout = 200;    // Timeout to stop generating ctrl signals after stopped receiving trajectory (ms)
    int     ignore_initial_inputs = 0;  // num inputs to throw away after startup

    //Pure Pursuit configs
    double vehicle_response_lag = 0.2;       // An approximation of the delay (sec) between sent vehicle commands and the vehicle begining a meaningful acceleration to that command
    double max_lookahead_dist = 100.0;
    double min_lookahead_dist = 6.0;  // Min lookahead distance (m)
    double speed_to_lookahead_ratio = 2.0; // Ratio to calculate lookahead distance
    bool is_interpolate_lookahead_point = true;
    bool is_delay_compensation = true;      // not to be confused with vehicle_response_lag, which is applied nonetheless
    double emergency_stop_distance = 0.1;
    double speed_thres_traveling_direction = 0.3;
    double dist_front_rear_wheels = 3.5;

    double dt = 0.1; // Timestep to calculate ctrl commands (s)
    double integrator_max_pp = 0.0; //Max integrator val for pure pursuit integral controller
    double integrator_min_pp = 0.0;   //Min integrator val for pure pursuit integral controller
    double ki_pp = 0.0; // Integral weight for pure pursuit integral controller";
    bool is_integrator_enabled = false;
    bool enable_max_adjustment_filter = true;
    bool enable_max_accel_filter = true;



    // Stream operator for this config
  friend std::ostream& operator<<(std::ostream& output, const PlatooningControlPluginConfig& c)
  {
    output << "PlatooningControlPluginConfig { " << std::endl
           << "stand_still_headway_m: " << c.stand_still_headway_m << std::endl
           << "max_accel_mps2: " << c.max_accel_mps2 << std::endl
           << "kp: " << c.kp << std::endl
           << "kd: " << c.kd << std::endl
           << "ki: " << c.ki << std::endl
           << "max_delta_speed_per_timestep: " << c.max_delta_speed_per_timestep << std::endl
           << "min_delta_speed_per_timestep_: " << c.min_delta_speed_per_timestep << std::endl
           << "adjustment_cap_mps: " << c.adjustment_cap_mps << std::endl
           << "cmd_timestamp_ms: " << c.cmd_timestamp_ms << std::endl
           << "integrator_max: " << c.integrator_max << std::endl
           << "integrator_min: " << c.integrator_min << std::endl
           << "vehicle_id: " << c.vehicle_id << std::endl
           << "shutdown_timeout: " << c.shutdown_timeout << std::endl
           << "ignore_initial_inputs: " << c.ignore_initial_inputs << std::endl
            //Pure Pursuit configs
           << "vehicle_response_lag" << c.vehicle_response_lag << std::endl
           << "max_lookahead_dist: " << c.max_lookahead_dist << std::endl
           << "min_lookahead_dist: " << c.min_lookahead_dist << std::endl
           << "speed_to_lookahead_ratio: " << c.speed_to_lookahead_ratio << std::endl
           << "is_interpolate_lookahead_point: " << c.is_interpolate_lookahead_point << std::endl
           << "is_delay_compensation: " << c.is_delay_compensation << std::endl
           << "emergency_stop_distance: " << c.emergency_stop_distance << std::endl
           << "speed_thres_traveling_direction: "<< c.speed_thres_traveling_direction << std::endl
           << "dist_front_rear_wheels: " << c.dist_front_rear_wheels << std::endl
           << "dt: " << c.dt << std::endl
           << "integrator_max_pp: " << c.integrator_max_pp << std::endl
           << "integrator_min_pp: " << c.integrator_min_pp << std::endl
           << "ki_pp_: " << c.ki_pp << std::endl
           << "is_integrator_enabled" << c.is_integrator_enabled <<std::endl
           << "enable_max_adjustment_filter" << c.enable_max_adjustment_filter << std::endl
           << "enable_max_accel_filter" << c.enable_max_accel_filter << std::endl
           << "}" << std::endl;
    return output;
  }
};

} // platoon_control