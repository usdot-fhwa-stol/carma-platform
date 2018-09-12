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

#include "map_convertor.h"

void convertIntersectionGeometry(j2735_msgs::IntersectionGeometry& in_msg, cav_msgs::IntersectionGeometry& out_msg) {
  // TODO
}

void convertComputedLane(j2735_msgs::ComputedLane& in_msg, cav_msgs::ComputedLane& out_msg) {
  out_msg.reference_lane_id = in_msg.reference_lane_id;
// TODO offset axis
// # The offset axis for the computed lane
// cav_msgs/OffsetAxis offset_x_axis

// cav_msgs/OffsetAxis offset_yv_axis

// # Deviates from J2735 standard:
// # DrivenLineOffsetSm in this message are conveyed as deg rather than 1.5 deg units to avoid redundant unessasry conversion by subscribers
// # Angle ::= FLOAT (0..359)
// #  -- Unsigned units of 1 degree, in 1 octet
// #  -- the true north is 0, positive is clockwise
// #  -- the values 360 to 381 shall not be sent
// float32 rotateXY
// bool rotatexy_exists

// # Scale-B12 ::= INTEGER (-2048..2047)
// # A 12-bit signed scaling factor supporting scales from zero (which is not used) to >200%.
// # In this data element, the value zero is taken to represent a value of one (scale 1:1).
// int16 scale_x_axis
// bool scale_x_axis_exists

// int16 scale_y_axis
// bool scale_y_axis_exists

// # regional #TODO: RegionalExtensions are not yet implemented in asn1c
}

void convertNodeSetXY(j2735_msgs::NodeSetXY& in_msg, cav_msgs::NodeSetXY& out_msg) {
  // TODO
}

void convertNodeListXY(j2735_msgs::NodeListXY& in_msg, cav_msgs::NodeListXY& out_msg) {
  out_msg.choice = in_msg.choice;
  // Convert NodeSetXY
  convertNodeSetXY(in_msg.nodes, out_msg.nodes);
  // Convert ComputedLane
  convertComputedLane(in_msg.computed, out_msg.computed);
  // Done Conversion
}

void convertGenericLane(j2735_msgs::GenericLane& in_msg, cav_msgs::GenericLane& out_msg) {
  out_msg.lane_id = in_msg.lane_id;
  out_msg.name = in_msg.name;
  out_msg.name_exists = in_msg.name_exists;
  out_msg.ingress_approach = in_msg.ingress_approach;
  out_msg.ingress_approach_exists = in_msg.ingress_approach_exists;
  out_msg.egress_approach = in_msg.egress_approach;
  out_msg.egress_approach_exists = in_msg.egress_approach_exists;
  out_msg.lane_attributes = in_msg.lane_attributes;
  out_msg.maneuvers = in_msg.maneuvers;
  out_msg.maneuvers_exists = in_msg.maneuvers_exists;

  // Convert NodeListXY
  convertNodeListXY(in_msg.node_list, out_msg.node_list);
  // Done Conversion

  out_msg.connect_to_list = in_msg.connects_to.connect_to_list;
  out_msg.connects_to_exists = in_msg.connects_to_exists;
  out_msg.overlay_lane_list = in_msg.overlay_lane_list.overlay_lane_list;
  out_msg.overlay_lane_list_exists = in_msg.overlay_lane_list_exists;
}

void convertRegulatorySpeedLimit(j2735_msgs::RegulatorySpeedLimit& in_msg, cav_msgs::RegulatorySpeedLimit& out_msg) {
  out_msg.type = in_msg.type;
  // Convert Speed
  out_msg.speed = (double)in_msg.speed / units::FIFTIETH_M_PER_M;
  // Done Conversion
}

void convertPosition3D(j2735_msgs::Position3D& in_msg, cav_msgs::Position3D& out_msg) {
  // Convert lat/lon
  out_msg.latitude = (double)in_msg.latitude / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.longitude = (double)in_msg.longitude / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.elevation = (double)in_msg.elevation / units::DECA_M_PER_M;
  // Done Conversion
  out_msg.elevation_exists = in_msg.elevation_exists;
}

void convertRoadSegment(j2735_msgs::RoadSegment& in_msg, cav_msgs::RoadSegment& out_msg) {
  out_msg.name = in_msg.name;
  out_msg.name_exists = in_msg.name_exists;
  out_msg.id = in_msg.id;
  out_msg.revision = in_msg.revision;
  // Convert Position3D
  convertPosition3D(in_msg.ref_point, out_msg.ref_point);
  // Done Convert
  // Convert LaneWidth
  out_msg.lane_width = (double)in_msg.lane_width / units::CM_PER_M;
  // Done Convertion
  out_msg.lane_width_exists = in_msg.lane_width_exists;
  // Convert SpeedLimitList
  for (j2735_msgs::RegulatorySpeedLimit limit : in_msg.speed_limits.speed_limits) {
    cav_msgs::RegulatorySpeedLimit cav_limit;
    convertRegulatorySpeedLimit(limit, cav_limit);
    out_msg.speed_limits.push_back(cav_limit);
  }
  // Done Convertion
  out_msg.speed_limits_exists = in_msg.speed_limits_exists;
  // Convert RoadLaneSet
  for (j2735_msgs::GenericLane lane : in_msg.road_lane_set.road_lane_set_list) {
    cav_msgs::GenericLane cav_lane;
    convertGenericLane(lane, cav_lane);
    out_msg.road_lane_set_list.push_back(cav_lane);
  }
  // Done Convertion
}

void MapConvertor::convert(j2735_msgs::MapData& in_msg, cav_msgs::MapData& out_msg) {
  out_msg.header = in_msg.header;
  out_msg.time_stamp = in_msg.time_stamp;
  out_msg.time_stamp_exists = in_msg.time_stamp_exists;
  out_msg.msg_issue_revision = in_msg.msg_issue_revision;
  out_msg.layer_type = in_msg.layer_type;
  out_msg.layer_id = in_msg.layer_id;
  out_msg.layer_id_exists = in_msg.layer_id_exists;
  out_msg.intersections_exists = in_msg.intersections_exists;
  
  // Convert IntersectionGeometryList TODO
  for (j2735_msgs::IntersectionGeometry geometry : in_msg.intersections) {
    cav_msgs::IntersectionGeometry cav_geometry;
    convertIntersectionGeometry(geometry, cav_geometry);
    out_msg.intersections.push_back(cav_geometry);
  }
  // Done Conversion
  
  out_msg.road_segments_exists = in_msg.road_segments_exists;
  
  // Convert RoadSegmentList TODO
  for (j2735_msgs::RoadSegment seg : in_msg.road_segments.road_segment_list) {
    cav_msgs::RoadSegment cav_seg;
    convertRoadSegment(seg, cav_seg);
    out_msg.road_segment_list.push_back(cav_seg);
  }
  // Done Conversion

  out_msg.data_parameters = in_msg.data_parameters;
  out_msg.data_parameters_exists = in_msg.data_parameters_exists;
  out_msg.restriction_class_list = in_msg.restriction_list.restriction_class_list;
  out_msg.restriction_list_exists = in_msg.restriction_list_exists;
}

