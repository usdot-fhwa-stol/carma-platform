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
    double stand_still_headway = 12.0; // Standstill gap between vehicles (m)
    double max_accel = 2.5;  // Maximum acceleration absolute value used in controller filters (m/s^2)
    double kp = 0.5;  // Proportional weight for PID controller
    double kd = -0.5;  // Derivative Weight for PID controller
    double ki = 0.0;  // Integral weight for PID controller
    double max_value = 2;  // Max value to restrict speed adjustment at one time step (limit on delta_v) (m/s)
    double min_value = -10; // Min value to restrict speed adjustment at one time step (limit on delta_v) (m/s)
    double dt = 0.1; // Timestep to calculate ctrl commands (s)
    double adjustment_cap = 10;  // Adjustment cap for speed command (m/s)
    int cmd_timestamp = 100;  // Timestamp to calculate ctrl commands (ms)
    double integrator_max = 100; // Max limit for integrator term
    double integrator_min = -100;  // Max limit for integrator term
    double kdd = 4.5; //coefficient for smooth steering
    double wheel_base = 3.09; //Wheelbase of the vehicle (m)
    double lowpass_gain = 0.5;  // Lowpass filter gain
    double lookahead_ratio = 2.0;  // Ratio to calculate lookahead distance
    double min_lookahead_dist = 6.0;  // Min lookahead distance (m)
    std::string vehicle_id = "DEFAULT_VEHICLE_ID";         // Vehicle id is the license plate of the vehicle
    int     shutdown_timeout = 200;    // Timeout to stop generating ctrl signals after stopped receiving trajectory (ms)
    int     ignore_initial_inputs = 0;  // num inputs to throw away after startup
    double correction_angle = 0.0;  //Correction angle to improve steering accuracy
    double integrator_max_pp = 0.0; //Max integrator val for pure pursuit integral controller
    double integrator_min_pp = 0.0;   //Min integrator val for pure pursuit integral controller
    double ki_pp = 0.0; // Integral weight for pure pursuit integral controller";

    // Stream operator for this config
  friend std::ostream& operator<<(std::ostream& output, const PlatooningControlPluginConfig& c)
  {
    output << "PlatooningControlPluginConfig { " << std::endl
           << "stand_still_headway_: " << c.stand_still_headway << std::endl
           << "max_accel_: " << c.max_accel << std::endl
           << "kp_: " << c.kp << std::endl
           << "kd_: " << c.kd << std::endl
           << "ki_: " << c.ki << std::endl
           << "max_value_: " << c.max_value << std::endl
           << "min_value_: " << c.min_value << std::endl
           << "dt_: " << c.dt << std::endl
           << "adjustment_cap_: " << c.adjustment_cap << std::endl
           << "cmd_timestamp_: " << c.cmd_timestamp << std::endl
           << "integrator_max_: " << c.integrator_max << std::endl
           << "integrator_min_: " << c.integrator_min << std::endl
           << "kdd_: " << c.kdd << std::endl
           << "wheel_base_: " << c.wheel_base << std::endl
           << "lowpass_gain_: " << c.lowpass_gain << std::endl
           << "lookahead_ratio_: " << c.lookahead_ratio << std::endl
           << "min_lookahead_dist_: " << c.min_lookahead_dist << std::endl
           << "vehicle_id_: " << c.vehicle_id << std::endl
           << "shutdown_timeout_: " << c.shutdown_timeout << std::endl
           << "ignore_initial_inputs_: " << c.ignore_initial_inputs << std::endl
           << "correction_angle_: " << c.correction_angle << std::endl
           << "integrator_max_pp_: " << c.integrator_max_pp << std::endl
           << "integrator_min_pp_: " << c.integrator_min_pp << std::endl
           << "ki_pp_: " << c.ki_pp << std::endl
           << "}" << std::endl;
    return output;
  }
};

} // platoon_control