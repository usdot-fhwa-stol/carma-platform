#pragma once
/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <j2735_msgs/BSM.h>
#include <j2735_msgs/SPAT.h>
#include <j2735_msgs/MapData.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/BSM.h>
#include <cav_msgs/SPAT.h>
#include <cav_msgs/MapData.h>
#include "units.h"

/**
 * @class SPATConvertor
 * @brief Is the class responsible for converting J2735 SPATs to CARMA usable SPATs
 * 
 * Handles conversion between Map messages in the j2735_msgs and cav_msgs packages.
 * Unit conversions are handled
 */
class SPATConvertor 
{
  public:

    /**
     * @brief Convert the contents of a j2735_msgs::MapData into a cav_msgs::MapData
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convert(const j2735_msgs::SPAT& in_msg, cav_msgs::SPAT& out_msg);

  private:
    
    /**
     * @brief Convert the contents of a j2735_msgs::TimeChangeDetails into a cav_msgs::TimeChangeDetails
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertTimeChangeDetails(const j2735_msgs::TimeChangeDetails& in_msg, cav_msgs::TimeChangeDetails& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::AdvisorySpeed into a cav_msgs::AdvisorySpeed
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertAdvisorySpeed(const j2735_msgs::AdvisorySpeed& in_msg, cav_msgs::AdvisorySpeed& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::MovementEvent into a cav_msgs::MovementEvent
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertMovementEvent(const j2735_msgs::MovementEvent& in_msg, cav_msgs::MovementEvent& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::MovementState into a cav_msgs::MovementState
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertMovementState(const j2735_msgs::MovementState& in_msg, cav_msgs::MovementState& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::IntersectionState into a cav_msgs::IntersectionState
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertIntersectionState(const j2735_msgs::IntersectionState& in_msg, cav_msgs::IntersectionState& out_msg);
};  
