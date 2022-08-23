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

namespace platoon_strategic_ihp
{

/**
 * \brief Stuct containing the algorithm configuration values for the yield_pluginConfig
 */
struct PlatoonPluginConfig
{
  // following parameters are for general platooning plugin
  double vehicleLength         = 5.0;     // m
  int    maxPlatoonSize        = 10;      // 1
  int    algorithmType         = 0;       // N/A
  int    statusMessageInterval = 100;     // ms
  int    infoMessageInterval   = 200;     // ms
  double maneuver_plan_dt      = 15;      // s
  // following parameters are for IHP gap regulation algorithm
  double time_step             = 15;      // s
  double epislon               = 0.001;   // m/s
  
  // following parameters are for platoon forming and operation
  double timeHeadway                    = 2.0;     // s
  double standStillHeadway              = 12.0;    // m
  double maxAllowedJoinTimeGap          = 15.0;    // s
  double maxAllowedJoinGap              = 90.0;    // m
  double minAllowedJoinGap              = 5.0;    // m
  double longitudinalCheckThresold      = 85.0;    // m
  double desiredJoinTimeGap             = 4.0;     // s
  double desiredJoinGap                 = 30.0;    // m
  int    maxLeaderAbortingCalls         = 2;       // counter
  double waitingStateTimeout            = 25.0;    // s
  double cmdSpeedMaxAdjustment          = 10.0;    // m/s
    
  // following parameters are mainly for APF leader selection
  // UCLA: Rename the next four variables for better explainability.
  double minAllowableHeadaway    = 1.6;   // s
  double headawayStableLowerBond = 1.7;   // s
  double maxAllowableHeadaway    = 4.0;   // s
  double headawayStableUpperBond = 3.9;   // s
  
  double minCutinGap             = 22.0;  // m
  double maxCutinGap             = 22.0;  // m
  double maxCrosstrackError      = 2.0;   // m

  // Speed adjuster to slow down platoon memebr to create gap
  double slowDownAdjuster         = 0.75;    // ratio 
  double createGapAdjuster        = 0.3;     // ratio

  std::string vehicleID       = "default_id";

  // min speed to start platooning negotiations
  double minPlatooningSpeed   = 7.0;   // m/s

  // ---------------------- UCLA: parameters for IHP platoon trajectory regulation ----------------
  /**
  * \brief Parameter sets for IHP platoon trajectory regulation algorithm. 
  * Please refer to the updated design doc for detailed parameter description.
  */
  double ss_theta = 4.0; // Stanstill determining threshold, in m/s.
  double standstill = 2.0; // Stanstill reaction time adjuster, in s.
  double inter_tau = 1.5; // Inter-platoon time gap, refer to bumper to bumper gap time, in s.
  double intra_tau = 0.6; // Intra-platoon time gao, refer to bumper to bumper gap time, in s.
  double gap_weight = 0.9; // Weighted ratio for time-gap based calculation, unitless.
  bool test_front_join = false;  //Flag to enable/disable front join functionality with two vehicles.
                                // Flag can be set to true, to test front join functionality with two vehicles
                                // But in normal operating conditions it should be set to false
  bool test_cutin_join = false; //Flag to enable/disable front/rear cutin join functionality with only 2 vehicles
                                //Normal operations it should be false, but true allows a cutin to a single-car platoon
  //------------------------------------------------------------------------------------------------
  bool allowCutinJoin = true;    //Flag to enable/disable cut-in joins
  double significantDTDchange = 0.2;   // Ratio of dtd that is considered a significant change and triggers a new sort of platoon vector
  int join_index = -1;            // target join index for cut-in join - used for testing purposes. -1: front cutin join, target_platoon.size()-1: cut-in rear


  friend std::ostream& operator<<(std::ostream& output, const PlatoonPluginConfig& c)
  {
    output << "PlatoonPluginConfig { " << std::endl
          << "maxPlatoonSize: " << c.maxPlatoonSize << std::endl
          << "algorithmType: " << c.algorithmType << std::endl
          << "statusMessageInterval: " << c.statusMessageInterval << std::endl
          << "infoMessageInterval: " << c.infoMessageInterval << std::endl
          << "timeHeadway: " << c.timeHeadway << std::endl
          << "standStillHeadway: " << c.standStillHeadway << std::endl
          << "maxAllowedJoinTimeGap: " << c.maxAllowedJoinTimeGap << std::endl
          << "desiredJoinTimeGap: " << c.desiredJoinTimeGap << std::endl
          << "desiredJoinGap: " << c.desiredJoinGap << std::endl
          << "maxLeaderAbortingCalls: " << c.maxLeaderAbortingCalls << std::endl
          << "waitingStateTimeout: " << c.waitingStateTimeout << std::endl
          << "cmdSpeedMaxAdjustment: " << c.cmdSpeedMaxAdjustment << std::endl
          << "minAllowableHeadaway: " << c.minAllowableHeadaway << std::endl
          << "maxAllowableHeadaway: " << c.maxAllowableHeadaway << std::endl
          << "headawayStableLowerBond: " << c.headawayStableLowerBond << std::endl
          << "headawayStableUpperBond: " << c.headawayStableUpperBond << std::endl
          << "minCutinGap: " << c.minCutinGap << std::endl
          << "maxCutinGap: " << c.maxCutinGap << std::endl
          << "maxCrosstrackError: " << c.maxCrosstrackError << std::endl
          << "vehicleID: " << c.vehicleID << std::endl
          << "minPlatooningSpeed: " << c.minPlatooningSpeed << std::endl
          << "allowCutinJoin: " << c.allowCutinJoin << std::endl
          << "significantDTDchange: " << c.significantDTDchange << std::endl
          << "join_index: " << c.join_index << std::endl
          << "}" << std::endl; //TODO this is missing some fields
    return output;
  }
};

} //platoon_strategic_ihp
