/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <carma_v2x_msgs/msg/mobility_operation.hpp>
#include <carma_v2x_msgs/msg/traffic_control_message_v01.hpp>
#include <carma_v2x_msgs/msg/traffic_control_message.hpp>
#include <std_msgs/msg/string.hpp>
#include <functional>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <string>
#include <cstring>
#include <vector>
#include <algorithm>
#include <sstream>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <carma_wm_ros2/WorldModel.hpp>
#include <std_msgs/msg/string.h>
#include <carma_wm_ros2/Geometry.hpp>

namespace traffic_incident_parser{

class TrafficIncidentParserWorker
{

 public:

  using PublishTrafficControlCallback = std::function<void(const carma_v2x_msgs::msg::TrafficControlMessage&)>;

  /*!
   * \brief TrafficIncidentParserWorker constructor
   */
  TrafficIncidentParserWorker(carma_wm::WorldModelConstPtr wm, const PublishTrafficControlCallback &traffic_control_pub, 
   rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger, rclcpp::Clock::SharedPtr clock);
    
  /*!
   * \brief Callback for the georeference subscriber used to set the map projection.
   * \param msg The latest georeference.
   */ 
  void georeferenceCallback(std_msgs::msg::String::UniquePtr projection_msg);

  /*! 
   *  \brief Function to receive the incoming mobility operation message from the message node 
   *         and publish the geofence message upon processing the mobility msg. Only incoming mobility operation
   *         messages with strategy "carma3/Incident_Use_Case" are processed.
   *  \param mobility_msg Incoming mobility operation message
   */
  void mobilityOperationCallback(carma_v2x_msgs::msg::MobilityOperation::UniquePtr mobility_msg);

  /*! 
   *  \brief Function to help parse incoming mobility operation messages to required format.
   *  \param  mobility_strategy_params The strategy params associated with an incoming mobility operation message.
   *  \return True if the new message is valid and can be used. False if not new or not valid.
   */
  bool mobilityMessageParser(std::string mobility_strategy_params);

  /*! 
   *  \brief Function to help convert string to double data type.
   *  \param  str String object from which to extract the numeric value (a double).
   *  \param  str_idx The String object's index to start looking at.
   *  \return A string object containing only the numeric value included in the str object.
   */
  std::string stringParserHelper(std::string str, unsigned long str_index) const;
  
  /*! 
   *  \brief Algorithm for extracting the closed lanelet from internally saved mobility message (or geofence) params and assigning it to a traffic contol message. 
   *         Closed lanelets are represent by vector of points, where each point represents the geometric middle point of a closed lanelet
   *  \return A vector of traffic control messages; one for each closed lane.
   */
  std::vector<carma_v2x_msgs::msg::TrafficControlMessageV01> composeTrafficControlMesssages();

  /*! 
   *  \brief Function to convert internally saved incident origin point from lat/lon to local map frame.
   *  \return The internally saved incident origin point within the local map frame.
   */
  lanelet::BasicPoint2d getIncidentOriginPoint() const;

  /*! 
   *  \brief Helper method to compute the concatenated centerlines of the lanes in front of the emergency vehicle point
   *  \param adjacentSet The set of adjacent lanes to start from 
   *  \param start_point Point to start the downtrack calculation from. Should be the emergency vehicle points
   *  \param downtrack downtrack distance to grab centerline points from  
   *  \param forward_lanes Ouput parameter which will be populated with the centerlines for each lane up to the downtrack distance
   */
  void getAdjacentForwardCenterlines(const lanelet::ConstLanelets& adjacentSet,
    const lanelet::BasicPoint2d& start_point, double downtrack, std::vector<std::vector<lanelet::BasicPoint2d>>* forward_lanes) const;

  /*! 
   *  \brief Helper method that is identical to getAdjacentForwardCenterlines except it works in reverse using uptrack distance
   *  \param adjacentSet The set of adjacent lanes to start from 
   *  \param start_point Point to start the downtrack calculation from. Should be the emergency vehicle points
   *  \param downtrack downtrack distance to grab centerline points from  
   *  \param forward_lanes Ouput parameter which will be populated with the centerlines for each lane up to the downtrack distance
   */
  void getAdjacentReverseCenterlines(const lanelet::ConstLanelets& adjacentSet,
    const lanelet::BasicPoint2d& start_point, double uptrack, std::vector<std::vector<lanelet::BasicPoint2d>>* reverse_lanes) const;

  double latitude = 0.0;
  double longitude = 0.0;
  double down_track = 0.0;
  double up_track = 0.0;
  double min_gap = 0.0;
  double speed_advisory = 0.0;
  std::string event_reason;
  std::string event_type;

  std::string previous_strategy_params="";

 private:

  lanelet::BasicPoint2d local_point_;
  std::string projection_msg_;
  PublishTrafficControlCallback traffic_control_pub_;// local copy of external object publihsers
  carma_wm::WorldModelConstPtr wm_;

  // Logger interface
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

  rclcpp::Clock::SharedPtr clock_;
};

} // namespace traffic_incident_parser