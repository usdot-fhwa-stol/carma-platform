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

#include <cstdint>
#include <j2735_msgs/BSM.h>
#include <j2735_msgs/SPAT.h>
#include <j2735_msgs/MapData.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/BSM.h>
#include <cav_msgs/SPAT.h>
#include <cav_msgs/MapData.h>
#include "units.h"
#include "value_convertor.h"



/**
 * @class BSMConvertor
 * @brief Is the class responsible for converting J2735 BSMs to CARMA usable BSMs
 * 
 * Handles conversion between BSMs in the j2735_msgs and cav_msgs packages.
 * Unit conversions and presence flags are also handled
 */
class BSMConvertor 
{
  public:
    /**
     * @brief Convert the contents of a j2735_msgs::BSM into a cav_msgs::BSM
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions and presence flags are handled
     */
    static void convert(const j2735_msgs::BSM& in_msg, cav_msgs::BSM& out_msg);

    /**
     * @brief Convert the contents of a cav_msgs::BSM into a j2735_msgs::BSM
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions and presence flags are handled
     */
    static void convert(const cav_msgs::BSM& in_msg, j2735_msgs::BSM& out_msg);

  private:
    ////
    // Convert j2735_msgs to cav_msgs
    ////

    /**
     * @brief Convert the contents of a j2735_msgs::VehicleSize into a cav_msgs::VehicleSize
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions and presence flags are handled
     */
    static void convert(const j2735_msgs::VehicleSize& in_msg, cav_msgs::VehicleSize& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::AccelerationSet4Way into a cav_msgs::AccelerationSet4Way
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions and presence flags are handled
     */
    static void convert(const j2735_msgs::AccelerationSet4Way& in_msg, cav_msgs::AccelerationSet4Way& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::PositionalAccuracy into a cav_msgs::PositionalAccuracy
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions and presence flags are handled
     */
    static void convert(const j2735_msgs::PositionalAccuracy& in_msg, cav_msgs::PositionalAccuracy& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::BSMCoreData into a cav_msgs::BSMCoreData
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions and presence flags are handled
     */
    static void convert(const j2735_msgs::BSMCoreData& in_msg, cav_msgs::BSMCoreData& out_msg);
    
    ////
    // Convert cav_msgs to j2735_msgs
    ////

    /**
     * @brief Convert the contents of a cav_msgs::VehicleSize into a j2735_msgs::VehicleSize
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions and presence flags are handled
     */
    static void convert(const cav_msgs::VehicleSize& in_msg, j2735_msgs::VehicleSize& out_msg);

    /**
     * @brief Convert the contents of a cav_msgs::AccelerationSet4Way into a j2735_msgs::AccelerationSet4Way
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions and presence flags are handled
     */
    static void convert(const cav_msgs::AccelerationSet4Way& in_msg, j2735_msgs::AccelerationSet4Way& out_msg);

    /**
     * @brief Convert the contents of a cav_msgs::PositionalAccuracy into a j2735_msgs::PositionalAccuracy
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions and presence flags are handled
     */
    static void convert(const cav_msgs::PositionalAccuracy& in_msg, j2735_msgs::PositionalAccuracy& out_msg);

    /**
     * @brief Convert the contents of a cav_msgs::BSMCoreData into a j2735_msgs::BSMCoreData
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions and presence flags are handled
     */
    static void convert(const cav_msgs::BSMCoreData& in_msg, j2735_msgs::BSMCoreData& out_msg);
};