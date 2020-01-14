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
