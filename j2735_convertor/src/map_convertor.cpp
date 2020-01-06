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

/**
 * CPP File containing MapConvertor method definitions
 */

#include "map_convertor.h"

void MapConvertor::convertOffsetXaxis(const j2735_msgs::OffsetXaxis& in_msg, cav_msgs::OffsetAxis& out_msg) {
  out_msg.choice = in_msg.choice;
  out_msg.large = in_msg.large;
  out_msg.small = in_msg.small;
}

void MapConvertor::convertOffsetYaxis(const j2735_msgs::OffsetYaxis& in_msg, cav_msgs::OffsetAxis& out_msg) {
  out_msg.choice = in_msg.choice;
  out_msg.large = in_msg.large;
  out_msg.small = in_msg.small;
}

void MapConvertor::convertComputedLane(const j2735_msgs::ComputedLane& in_msg, cav_msgs::ComputedLane& out_msg) {
  out_msg.reference_lane_id = in_msg.reference_lane_id;
  convertOffsetXaxis(in_msg.offset_x_axis, out_msg.offset_x_axis);
  convertOffsetYaxis(in_msg.offset_y_axis, out_msg.offset_y_axis);

  // Convert DrivenLineOffsetSm
  out_msg.rotateXY = (double)in_msg.rotateXY * units::ONE_AND_A_HALF_DEG;
  // Done Convertion
  out_msg.rotatexy_exists = in_msg.rotatexy_exists;

  // TODO Scaling is currently not applied to other values
  out_msg.scale_x_axis = in_msg.scale_x_axis;
  out_msg.scale_x_axis_exists = in_msg.scale_x_axis_exists;
  out_msg.scale_y_axis = in_msg.scale_y_axis;
  out_msg.scale_y_axis_exists = in_msg.scale_y_axis_exists;
}

void MapConvertor::convertNodeOffsetPointXY(const j2735_msgs::NodeOffsetPointXY& in_msg, cav_msgs::NodeOffsetPointXY& out_msg) {
  out_msg.choice = in_msg.choice;
  // Convert XY or Lat/Lon points based on choice field
  switch(in_msg.choice) {
    case j2735_msgs::NodeOffsetPointXY::NODE_XY1:
      out_msg.x = (double)in_msg.node_xy1.x / units::CM_PER_M;
      out_msg.y = (double)in_msg.node_xy1.y / units::CM_PER_M;
      break;
    case j2735_msgs::NodeOffsetPointXY::NODE_XY2:
      out_msg.x = (double)in_msg.node_xy2.x / units::CM_PER_M;
      out_msg.y = (double)in_msg.node_xy2.y / units::CM_PER_M;
      break;
    case j2735_msgs::NodeOffsetPointXY::NODE_XY3:
      out_msg.x = (double)in_msg.node_xy3.x / units::CM_PER_M;
      out_msg.y = (double)in_msg.node_xy3.y / units::CM_PER_M;
      break;
    case j2735_msgs::NodeOffsetPointXY::NODE_XY4:
      out_msg.x = (double)in_msg.node_xy4.x / units::CM_PER_M;
      out_msg.y = (double)in_msg.node_xy4.y / units::CM_PER_M;
      break;
    case j2735_msgs::NodeOffsetPointXY::NODE_XY5:
      out_msg.x = (double)in_msg.node_xy5.x / units::CM_PER_M;
      out_msg.y = (double)in_msg.node_xy5.y / units::CM_PER_M;
      break;
    case j2735_msgs::NodeOffsetPointXY::NODE_XY6:
      out_msg.x = (double)in_msg.node_xy6.x / units::CM_PER_M;
      out_msg.y = (double)in_msg.node_xy6.y / units::CM_PER_M;
      break;
    case j2735_msgs::NodeOffsetPointXY::NODE_LATLON:
      out_msg.latitude = (double)in_msg.node_latlon.latitude / units::TENTH_MICRO_DEG_PER_DEG;
      out_msg.longitude = (double)in_msg.node_latlon.longitude / units::TENTH_MICRO_DEG_PER_DEG;
      break;
  }
  // Done Conversion
}

void MapConvertor::convertLaneDataAttribute(const j2735_msgs::LaneDataAttribute& in_msg, cav_msgs::LaneDataAttribute& out_msg) {
  out_msg.choice = in_msg.choice;
  // Convert Angles 
  // Do comparison to avoid duplicate math
  switch(in_msg.choice) {
    case j2735_msgs::LaneDataAttribute::PATH_END_POINT_ANGLE:
      out_msg.path_end_point_angle = in_msg.path_end_point_angle;
      break;
    case j2735_msgs::LaneDataAttribute::LANE_CROWN_POINT_CENTER:
      out_msg.lane_crown_point_center = in_msg.lane_crown_point_center * units::THREE_TENTHS_DEG;
      break;
    case j2735_msgs::LaneDataAttribute::LANE_CROWN_POINT_LEFT:
      out_msg.lane_crown_point_left = in_msg.lane_crown_point_left * units::THREE_TENTHS_DEG;
      break;
    case j2735_msgs::LaneDataAttribute::LANE_CROWN_POINT_RIGHT:
      out_msg.lane_crown_point_right = in_msg.lane_crown_point_right * units::THREE_TENTHS_DEG;
      break;
    case j2735_msgs::LaneDataAttribute::LANE_ANGLE:
      out_msg.lane_angle = in_msg.lane_angle * units::ONE_AND_A_HALF_DEG;
      break;
    case j2735_msgs::LaneDataAttribute::SPEED_LIMITS:
        // Convert SpeedLimitsList
        for (j2735_msgs::RegulatorySpeedLimit limit : in_msg.speed_limits.speed_limits) {
          cav_msgs::RegulatorySpeedLimit cav_limit;
          convertRegulatorySpeedLimit(limit, cav_limit);
          out_msg.speed_limits.push_back(cav_limit);
        }
      break;
  }
  // Done Conversion
}

void MapConvertor::convertNodeAttributeSetXY(const j2735_msgs::NodeAttributeSetXY& in_msg, cav_msgs::NodeAttributeSetXY& out_msg) {
  for (j2735_msgs::NodeAttributeXY attribute : in_msg.local_node.node_attribute_xy_List) {
    out_msg.local_node.push_back(attribute);
  }
  out_msg.local_node_exists = in_msg.local_node_exists;

  for (j2735_msgs::SegmentAttributeXY attribute : in_msg.disabled.segment_attribute_xy) {
    out_msg.disabled.push_back(attribute);
  }
  out_msg.disabled_exists = in_msg.disabled_exists;


  for (j2735_msgs::SegmentAttributeXY attribute : in_msg.enabled.segment_attribute_xy) {
    out_msg.enabled.push_back(attribute);
  }
  out_msg.enabled_exists = in_msg.enabled_exists;

  out_msg.data_exists = in_msg.data_exists;
  out_msg.dWitdh_exists = in_msg.dWitdh_exists;
  out_msg.dElevation_exists = in_msg.dElevation_exists;
  
  // Convert LaneDataAttributeList
  for (j2735_msgs::LaneDataAttribute attribute : in_msg.data.lane_attribute_list) {
    cav_msgs::LaneDataAttribute cav_attribute;
    convertLaneDataAttribute(attribute, cav_attribute);
    out_msg.lane_attribute_list.push_back(cav_attribute);
  }
  // Convert dWidth
  out_msg.dWitdh = (double)in_msg.dWitdh / units::CM_PER_M;
  // Convert dElevation
  out_msg.dElevation = (double)in_msg.dElevation / units::CM_PER_M;
  // Done Conversion
}

void MapConvertor::convertNodeXY(const j2735_msgs::NodeXY& in_msg, cav_msgs::NodeXY& out_msg) {
  convertNodeOffsetPointXY(in_msg.delta, out_msg.delta);
  convertNodeAttributeSetXY(in_msg.attributes, out_msg.attributes);
  out_msg.attributes_exists = in_msg.attributes_exists;
}

void MapConvertor::convertNodeSetXY(const j2735_msgs::NodeSetXY& in_msg, cav_msgs::NodeSetXY& out_msg) {
  for (j2735_msgs::NodeXY node: in_msg.node_set_xy) {
    cav_msgs::NodeXY cav_node;
    convertNodeXY(node, cav_node);
    out_msg.node_set_xy.push_back(cav_node);
  }
}

void MapConvertor::convertNodeListXY(const j2735_msgs::NodeListXY& in_msg, cav_msgs::NodeListXY& out_msg) {
  out_msg.choice = in_msg.choice;
  // Convert NodeSetXY
  convertNodeSetXY(in_msg.nodes, out_msg.nodes);
  // Convert ComputedLane
  convertComputedLane(in_msg.computed, out_msg.computed);
  // Done Conversion
}

void MapConvertor::convertGenericLane(const j2735_msgs::GenericLane& in_msg, cav_msgs::GenericLane& out_msg) {
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

void MapConvertor::convertRegulatorySpeedLimit(const j2735_msgs::RegulatorySpeedLimit& in_msg, cav_msgs::RegulatorySpeedLimit& out_msg) {
  out_msg.type = in_msg.type;
  // Convert Speed
  out_msg.speed = (double)in_msg.speed / units::FIFTIETH_M_PER_M;
  // Done Conversion
}

void MapConvertor::convertPosition3D(const j2735_msgs::Position3D& in_msg, cav_msgs::Position3D& out_msg) {
  // Convert lat/lon
  out_msg.latitude = (double)in_msg.latitude / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.longitude = (double)in_msg.longitude / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.elevation = (double)in_msg.elevation / units::DECA_M_PER_M;
  // Done Conversion
  out_msg.elevation_exists = in_msg.elevation_exists;
}

void MapConvertor::convertIntersectionGeometry(const j2735_msgs::IntersectionGeometry& in_msg, cav_msgs::IntersectionGeometry& out_msg) {
  out_msg.name = in_msg.name;
  out_msg.name_exists = in_msg.name_exists;
  out_msg.id = in_msg.id;
  out_msg.revision = in_msg.revision;
  out_msg.lane_width_exists = in_msg.lane_width_exists;
  out_msg.speed_limits_exists = in_msg.speed_limits_exists;
  // Convert Point3D
  convertPosition3D(in_msg.ref_point, out_msg.ref_point);
  // Convert LaneWidth
  out_msg.lane_width = (double)in_msg.lane_width / units::CM_PER_M;
  // Convert SpeedLimitsList
  for (j2735_msgs::RegulatorySpeedLimit limit : in_msg.speed_limits.speed_limits) {
    cav_msgs::RegulatorySpeedLimit cav_limit;
    convertRegulatorySpeedLimit(limit, cav_limit);
    out_msg.speed_limits.push_back(cav_limit);
  }
  // Convert RoadLaneSet
  for (j2735_msgs::GenericLane lane : in_msg.lane_set.lane_list) {
    cav_msgs::GenericLane cav_lane;
    convertGenericLane(lane, cav_lane);
    out_msg.lane_list.push_back(cav_lane);
  }
  // Done Convertion

  out_msg.preempt_priority_list = in_msg.preempt_priority_data.preempt_priority_list;
  out_msg.preempt_priority_data_exists = in_msg.preempt_priority_data_exists;
}

void MapConvertor::convertRoadSegment(const j2735_msgs::RoadSegment& in_msg, cav_msgs::RoadSegment& out_msg) {
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

void MapConvertor::convert(const j2735_msgs::MapData& in_msg, cav_msgs::MapData& out_msg) {
  out_msg.header = in_msg.header;
  out_msg.time_stamp = in_msg.time_stamp;
  out_msg.time_stamp_exists = in_msg.time_stamp_exists;
  out_msg.msg_issue_revision = in_msg.msg_issue_revision;
  out_msg.layer_type = in_msg.layer_type;
  out_msg.layer_id = in_msg.layer_id;
  out_msg.layer_id_exists = in_msg.layer_id_exists;
  out_msg.intersections_exists = in_msg.intersections_exists;
  
  // Convert IntersectionGeometryList
  for (j2735_msgs::IntersectionGeometry geometry : in_msg.intersections) {
    cav_msgs::IntersectionGeometry cav_geometry;
    convertIntersectionGeometry(geometry, cav_geometry);
    out_msg.intersections.push_back(cav_geometry);
  }
  // Done Conversion
  
  out_msg.road_segments_exists = in_msg.road_segments_exists;
  
  // Convert RoadSegmentList
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

