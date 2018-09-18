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
 * @class MAPConvertor
 * @brief Is the class responsible for converting J2735 Maps to CARMA usable Mapss
 * 
 * Handles conversion between Map messages in the j2735_msgs and cav_msgs packages.
 * Unit conversions are handled
 * Note: The concept of map Zoom is not accounted for in these conversions
 */
class MapConvertor 
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
    static void convert(const j2735_msgs::MapData& in_msg, cav_msgs::MapData& out_msg);

  private:

    /**
     * @brief Convert the contents of a j2735_msgs::OffsetXaxis into a cav_msgs::OffsetAxis
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertOffsetXaxis(const j2735_msgs::OffsetXaxis& in_msg, cav_msgs::OffsetAxis& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::OffsetYaxis into a cav_msgs::OffsetAxis
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertOffsetYaxis(const j2735_msgs::OffsetYaxis& in_msg, cav_msgs::OffsetAxis& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::ComputedLane into a cav_msgs::ComputedLane
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertComputedLane(const j2735_msgs::ComputedLane& in_msg, cav_msgs::ComputedLane& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::NodeOffsetPointXY into a cav_msgs::NodeOffsetPointXY
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertNodeOffsetPointXY(const j2735_msgs::NodeOffsetPointXY& in_msg, cav_msgs::NodeOffsetPointXY& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::LaneDataAttribute into a cav_msgs::LaneDataAttribute
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertLaneDataAttribute(const j2735_msgs::LaneDataAttribute& in_msg, cav_msgs::LaneDataAttribute& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::NodeAttributeSetXY into a cav_msgs::NodeAttributeSetXY
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertNodeAttributeSetXY(const j2735_msgs::NodeAttributeSetXY& in_msg, cav_msgs::NodeAttributeSetXY& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::NodeXY into a cav_msgs::NodeXY
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertNodeXY(const j2735_msgs::NodeXY& in_msg, cav_msgs::NodeXY& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::NodeSetXY into a cav_msgs::NodeSetXY
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertNodeSetXY(const j2735_msgs::NodeSetXY& in_msg, cav_msgs::NodeSetXY& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::NodeListXY into a cav_msgs::NodeListXY
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertNodeListXY(const j2735_msgs::NodeListXY& in_msg, cav_msgs::NodeListXY& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::GenericLane into a cav_msgs::GenericLane
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertGenericLane(const j2735_msgs::GenericLane& in_msg, cav_msgs::GenericLane& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::RegulatorySpeedLimit into a cav_msgs::RegulatorySpeedLimit
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertRegulatorySpeedLimit(const j2735_msgs::RegulatorySpeedLimit& in_msg, cav_msgs::RegulatorySpeedLimit& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::Position3D into a cav_msgs::Position3D
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertPosition3D(const j2735_msgs::Position3D& in_msg, cav_msgs::Position3D& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::IntersectionGeometry into a cav_msgs::IntersectionGeometry
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertIntersectionGeometry(const j2735_msgs::IntersectionGeometry& in_msg, cav_msgs::IntersectionGeometry& out_msg);

    /**
     * @brief Convert the contents of a j2735_msgs::RoadSegment into a cav_msgs::RoadSegment
     * 
     * @param in_msg The message to be converted
     * @param out_msg The message to store the output
     * 
     * Unit conversions are handled
     */
    static void convertRoadSegment(const j2735_msgs::RoadSegment& in_msg, cav_msgs::RoadSegment& out_msg);
};  
