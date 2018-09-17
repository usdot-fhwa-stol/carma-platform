#pragma once
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
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