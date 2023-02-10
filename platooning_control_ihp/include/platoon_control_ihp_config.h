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

#include <iostream>

/**
 * \brief Stuct containing the algorithm configuration values for the PlatooningControlPlugin
 */
struct PlatooningControlIHPPluginConfig
{
  double standStillHeadway = 12.0; // Standstill gap between vehicles (m)
  double maxAccel = 2.5;  // Maximum acceleration absolute value used in controller filters (m/s^2)
  double Kp = 0.5;  // Proportional weight for PID controller
  double Kd = -0.5; // Derivative Weight for PID controller
  double Ki = 0.0;  // Integral weight for PID controller
  double maxValue = 2;  // Max value to restrict speed adjustment at one time step (limit on delta_v) (m/s)
  double minValue = -10; // Min value to restrict speed adjustment at one time step (limit on delta_v) (m/s)
  double dt = 0.1; // Timestep to calculate ctrl commands (s)
  double adjustmentCap = 10;  // Adjustment cap for speed command (m/s)
  int cmdTmestamp = 100;  // Timestamp to calculate ctrl commands (ms)
  double integratorMax = 100; // Max limit for integrator term
  double integratorMin = -100;  // Max limit for integrator term
  double Kdd = 4.5;                             //coefficient for smooth steering
  double lowpassGain = 0.5; // Lowpass filter gain
  double lookaheadRatio = 2.0;  // Ratio to calculate lookahead distance
  double minLookaheadDist = 6.0;  // Min lookahead distance (m)
  std::string vehicleID = "DEFAULT_VEHICLE_ID";         // Vehicle id is the license plate of the vehicle
  int     shutdownTimeout = 200;                // ms 
  int     ignoreInitialInputs = 0;              // num inputs to throw away after startup
  double wheelBase = 3.7;    //Wheelbase of the vehicle
  // added for gap regulation
  double vehicleLength = 5.0;     // m
  // UCLA: Added for IHP control 
  // ---------------------- UCLA: parameters for IHP platoon trajectory regulation ----------------
  /**
  * \brief Parameter sets for IHP platoon trajectory regulation algorithm. 
  * Please refer to the updated design doc for detailed parameter description.
  */
  double ss_theta   = 1.5;    // Minimum speed to be considered as moving, in m/s.
  double standstill = 2.0;    // Extra time needed to reacte to traffic sceanrios when vehicle is standstill (not moving), in s.
  double inter_tau  = 1.5;    // Inter-platoon time gap, refer to bumper to bumper gap time, in s.
  double intra_tau  = 0.6;    // Intra-platoon time gao, refer to bumper to bumper gap time, in s.
  double gap_weight = 0.9;    // Weighted ratio for time-gap based calculation, unitless.
  double time_step  = 5;      // The time step ga regulation algorithm uses to calculate desired position, in s.
  bool   test_front_join = false;   //Flag to enable/disable front join functionality with two vehicles.
                                    // Flag can be set to true, to test front join functionality with two vehicles
                                    // But in normal operating conditions it should be set to false
  //------------------------------------------------------------------------------------------------

  friend std::ostream& operator<<(std::ostream& output, const PlatooningControlIHPPluginConfig& c)
  {
    output << "PlatoonControlIHPPluginConfig { " << std::endl
           << "standStillHeadway: " << c.standStillHeadway << std::endl
           << "maxAccel: " << c.maxAccel << std::endl
           << "Kp: " << c.Kp << std::endl
           << "Kd: " << c.Kd << std::endl
           << "Ki: " << c.Ki << std::endl
           << "maxValue: " << c.maxValue << std::endl
           << "minValue: " << c.minValue << std::endl
           << "dt: " << c.dt << std::endl
           << "adjustmentCap: " << c.adjustmentCap << std::endl
           << "cmdTmestamp: " << c.cmdTmestamp << std::endl
           << "integratorMax: " << c.integratorMax << std::endl
           << "integratorMin: " << c.integratorMin << std::endl
           << "Kdd: " << c.Kdd << std::endl
           << "wheelBase: " << c.wheelBase << std::endl
           << "lowpassGain: " << c.lowpassGain << std::endl
           << "lookaheadRatio: " << c.lookaheadRatio << std::endl
           << "minLookaheadDist: " << c.minLookaheadDist << std::endl
           << "vehicleID: " << c.vehicleID << std::endl
           << "shutdownTimeout: " << c.shutdownTimeout << std::endl
           << "ignoreInitialInputs: " << c.ignoreInitialInputs << std::endl
           << "}" << std::endl;
    return output;
  }
};