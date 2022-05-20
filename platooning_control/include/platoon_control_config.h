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
  double timeHeadway = 6.0;
  double standStillHeadway = 12.0;
  double maxAccel = 2.5;
  double Kp = 0.5;
  double Kd = -0.5;
  double Ki = 0.0;
  double maxValue = 100;
  double minValue = -100;
  double dt = 0.1;
  double adjustmentCap = 10;
  int cmdTmestamp = 100;
  double integratorMax = 100;
  double integratorMin = -100;
  double Kdd = 4.5;                             //coefficient for smooth steering
  double wheelBase = 3.09;
  double lowpassGain = 0.5;
  double lookaheadRatio = 2.0;
  double minLookaheadDist = 6.0;
  std::string vehicleID = "DEFAULT_VEHICLE_ID";         // Vehicle id is the license plate of the vehicle
  int     shutdownTimeout = 200;                // ms 
  int     ignoreInitialInputs = 0;              // num inputs to throw away after startup
  double correctionAngle = 0.0;
  
  
  friend std::ostream& operator<<(std::ostream& output, const PlatooningControlPluginConfig& c)
  {
    output << "PlatooningControlPluginConfig { " << std::endl
           << "timeHeadway: " << c.timeHeadway << std::endl
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
           << "}" << std::endl;
    return output;
  }
};