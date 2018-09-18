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

/**
 * CPP File containing SPATConvertor method definitions
 */

#include "spat_convertor.h"

void SPATConvertor::convertTimeChangeDetails(const j2735_msgs::TimeChangeDetails& in_msg, cav_msgs::TimeChangeDetails& out_msg) {
  out_msg.confidence = in_msg.confidence;
  out_msg.confidence_exists = in_msg.confidence_exists;
  out_msg.start_time_exists = in_msg.start_time_exists;
  out_msg.max_end_time_exists = in_msg.max_end_time_exists;
  out_msg.likely_time_exists = in_msg.likely_time_exists;
  out_msg.next_time_exists = in_msg.next_time_exists;
  // Convert Time Marks
  out_msg.start_time = (double)in_msg.start_time / units::DECA_S_PER_S;
  out_msg.min_end_time = (double)in_msg.min_end_time / units::DECA_S_PER_S;
  out_msg.max_end_time = (double)in_msg.max_end_time / units::DECA_S_PER_S;
  out_msg.likely_time_exists = (double)in_msg.likely_time_exists / units::DECA_S_PER_S;
  out_msg.next_time = (double)in_msg.next_time / units::DECA_S_PER_S;
  // Done Conversion
}

void SPATConvertor::convertAdvisorySpeed(const j2735_msgs::AdvisorySpeed& in_msg, cav_msgs::AdvisorySpeed& out_msg) {
  out_msg.type = in_msg.type;
  out_msg.speed_exists = in_msg.speed_exists;
  // Convert Speed
  out_msg.speed = (double)in_msg.speed / units::DECA_MPS_PER_MPS;
  // Done Conversion
  out_msg.confidence = in_msg.confidence;
  out_msg.distance = in_msg.distance;
  out_msg.distance_exists = in_msg.distance_exists;
  out_msg.restriction_class_id = in_msg.restriction_class_id;
  out_msg.restriction_class_id_exists = in_msg.restriction_class_id_exists;
}

void SPATConvertor::convertMovementEvent(const j2735_msgs::MovementEvent& in_msg, cav_msgs::MovementEvent& out_msg) {
  out_msg.event_state = in_msg.event_state;
  out_msg.timing_exists = in_msg.timing_exists;
  // Convert TimeChangeDetails
  convertTimeChangeDetails(in_msg.timing, out_msg.timing);
  // Done Conversion

  out_msg.speeds_exists = in_msg.speeds_exists;
  // Convert AdvisorySpeedList
    for (j2735_msgs::AdvisorySpeed speed : in_msg.speeds.advisory_speed_list) {
    cav_msgs::AdvisorySpeed cav_speed;
    convertAdvisorySpeed(speed, cav_speed);
    out_msg.advisory_speed_list.push_back(cav_speed);
  }
}

void SPATConvertor::convertMovementState(const j2735_msgs::MovementState& in_msg, cav_msgs::MovementState& out_msg) {
  out_msg.movement_name = in_msg.movement_name;
  out_msg.movement_name_exists = in_msg.movement_name_exists;
  out_msg.signal_group = in_msg.signal_group;
  // Convert MovementEvent
  for (j2735_msgs::MovementEvent event : in_msg.state_time_speed.movement_event_list) {
    cav_msgs::MovementEvent cav_event;
    convertMovementEvent(event, cav_event);
    out_msg.movement_event_list.push_back(cav_event);
  }
  // Done Conversion
  out_msg.connection_maneuver_assist_list = in_msg.maneuver_assist_list.connection_maneuver_assist_list;
  out_msg.maneuver_assist_list_exists = in_msg.maneuver_assist_list_exists;
}

void SPATConvertor::convertIntersectionState(const j2735_msgs::IntersectionState& in_msg, cav_msgs::IntersectionState& out_msg) {
  out_msg.name = in_msg.name;
  out_msg.name_exists = in_msg.name_exists;
  out_msg.id = in_msg.id;
  out_msg.revision = in_msg.revision;
  out_msg.status = in_msg.status;
  out_msg.moy = in_msg.moy;
  out_msg.moy_exists = in_msg.moy_exists;
  // Convert units of timestamp from ms to s
  out_msg.time_stamp = (double)in_msg.time_stamp / units::MS_PER_S;
  out_msg.time_stamp_exists = in_msg.time_stamp_exists;
  // Done conversion
  out_msg.lane_id_list = in_msg.enabled_lanes.lane_id_list;
  out_msg.enabled_lanes_exists = in_msg.enabled_lanes_exists;

  // Convert MovementState
  for (j2735_msgs::MovementState state : in_msg.states.movement_list) {
    cav_msgs::MovementState cav_state;
    convertMovementState(state, cav_state);
    out_msg.movement_list.push_back(cav_state);
  }
  // Done Conversion

  out_msg.connection_maneuver_assist_list = in_msg.maneuever_assist_list.connection_maneuver_assist_list;
  out_msg.maneuever_assist_list_exists = in_msg.maneuever_assist_list_exists;
}

void SPATConvertor::convert(const j2735_msgs::SPAT& in_msg, cav_msgs::SPAT& out_msg) {
  out_msg.time_stamp = in_msg.time_stamp;
  out_msg.time_stamp_exists = in_msg.time_stamp_exists;
  out_msg.name = in_msg.name;
  out_msg.name_exists = in_msg.name_exists;

  // Convert Intersection State List
  for (j2735_msgs::IntersectionState state : in_msg.intersections.intersection_state_list) {
    cav_msgs::IntersectionState cav_state;
    convertIntersectionState(state, cav_state);
    out_msg.intersection_state_list.push_back(cav_state);
  }
}

