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
struct PlatooningControlPluginConfig
{
  double standStillHeadway = 12.0; // Standstill gap between vehicles (m)
  double maxAccel = 2.5;  // Maximum acceleration absolute value used in controller filters (m/s^2)
  double Kp = 0.5;  // Proportional weight for PID controller
  double Kd = -0.5;  // Derivative Weight for PID controller
  double Ki = 0.0;  // Integral weight for PID controller
  double maxValue = 2;  // Max value to restrict speed adjustment at one time step (limit on delta_v) (m/s)
  double minValue = -10; // Min value to restrict speed adjustment at one time step (limit on delta_v) (m/s)
  double dt = 0.1; // Timestep to calculate ctrl commands (s)
  double adjustmentCap = 10;  // Adjustment cap for speed command (m/s)
  int cmdTmestamp = 100;  // Timestamp to calculate ctrl commands (ms)
  double integratorMax = 100; // Max limit for integrator term
  double integratorMin = -100;  // Max limit for integrator term
  double Kdd = 4.5; //coefficient for smooth steering
  double wheelBase = 3.09; //Wheelbase of the vehicle (m)
  double lowpassGain = 0.5;  // Lowpass filter gain
  double lookaheadRatio = 2.0;  // Ratio to calculate lookahead distance
  double minLookaheadDist = 6.0;  // Min lookahead distance (m)
  std::string vehicleID = "DEFAULT_VEHICLE_ID";         // Vehicle id is the license plate of the vehicle
  int     shutdownTimeout = 200;    // Timeout to stop generating ctrl signals after stopped receiving trajectory (ms) 
  int     ignoreInitialInputs = 0;  // num inputs to throw away after startup
  double correctionAngle = 0.0;  //Correction angle to improve steering accuracy
  double integratorMax_pp = 0.0; //Max integrator val for pure pursuit integral controller
  double integratorMin_pp = 0.0;   //Min integrator val for pure pursuit integral controller
  double Ki_pp = 0.0; // Integral weight for pure pursuit integral controller
  
  
  friend std::ostream& operator<<(std::ostream& output, const PlatooningControlPluginConfig& c)
  {
    output << "PlatooningControlPluginConfig { " << std::endl
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
           << "correctionAngle: " << c.correctionAngle << std::endl
           << "integratorMax_pp: " << c.integratorMax_pp << std::endl
           << "integratorMin_pp: " << c.integratorMin_pp << std::endl
           << "Ki_pp: " << c.Ki_pp << std::endl
           << "}" << std::endl;
    return output;
  }
};