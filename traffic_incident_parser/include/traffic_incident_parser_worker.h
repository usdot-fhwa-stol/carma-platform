/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#ifndef TRAFFIC_INCIDENT_PARSER_WORKER_H
#define TRAFFIC_INCIDENT_PARSER_WORKER_H

#include <ros/ros.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/TrafficControlMessageV01.h>
#include <cav_msgs/TrafficControlMessage.h>
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
#include <carma_wm/WorldModel.h>
#include <std_msgs/String.h>
#include <gps_common/GPSFix.h>
#include <carma_wm/Geometry.h>

namespace traffic{

class TrafficIncidentParserWorker
{

 public:

  using PublishTrafficControlCallback = std::function<void(const cav_msgs::TrafficControlMessage&)>;

  /*!
   * \brief Constructor
   */
  TrafficIncidentParserWorker(carma_wm::WorldModelConstPtr wm,const PublishTrafficControlCallback &traffic_control_pub);
    
  /*! \fn projectionCallback(const std_msgs::String &projection_msg)
    \brief projectionCallback stores the projection string needed to convert ECEF to MAP frame
    \param  std_msgs::String
  */
  void projectionCallback(const std_msgs::String &projection_msg);

    /*! \fn mobilityOperationCallback(const cav_msgs::MobilityOperation &mobility_msg)
    \brief mobilityOperationCallback to receive the incoming mobility operation message from message node 
           and also this function publishes the geofence message upon processing the mobility msg.
    \param  cav_msgs::MobilityOperation
  */
  void mobilityOperationCallback(const cav_msgs::MobilityOperation &mobility_msg);

    /*! \fn mobilityMessageParser(std::string mobility_strategy_params)
    \brief mobilityMessageParser helps to parse incoming mobility operation message to required format
    \param  std::string mobility_strategy_params

    \return True if the new message is valid and can be used. False if not new or not valid.
  */
  bool mobilityMessageParser(std::string mobility_strategy_params);

    /*! \fn stringParserHelper(std::string str,int str_index)
    \brief stringParserHelper helps to convert string to double data type.
    \param  std::string to convert from str to double
    \param  unsigned long idx to start looking at 
  */
  std::string stringParserHelper(std::string str, unsigned long str_index) const;
  
    /*! \fn composeTrafficControlMesssage()
    \brief composeTrafficControlMesssage algorithm for extracting the closed lanelet from internally saved mobility message (or geofence) params and assign it to trafic contol message. 
    Closed lanelets are represent by vector of points, where each point represents the geometric middle point of a closed lanelet
  */
  std::vector<cav_msgs::TrafficControlMessageV01> composeTrafficControlMesssages();

    /*! \fngetIncidentOriginPoint()
    \brief getIncidentOriginPoint converts internally saved incident origin point from ECEF to local map frame
  */
  lanelet::BasicPoint2d getIncidentOriginPoint() const;

  /*! \fn getAdjacentForwardCenterlines(const lanelet::ConstLanelets& adjacentSet,
    const lanelet::BasicPoint2d& start_point, double downtrack, std::vector<std::vector<lanelet::BasicPoint2d>>* forward_lanes)
    \brief Helper method to compute the concatenated centerlines of the lanes in front of the emergency vehicle point
    \param adjacentSet The set of adjacent lanes to start from 
    \param start_point Point to start the downtrack calculation from. Should be the emergency vehicle points
    \param The downtrack distance to grab centerline points from  
    \param forward_lanes Ouput parameter which will be populated with the centerlines for each lane up to the downtrack distance
  */
  void getAdjacentForwardCenterlines(const lanelet::ConstLanelets& adjacentSet,
    const lanelet::BasicPoint2d& start_point, double downtrack, std::vector<std::vector<lanelet::BasicPoint2d>>* forward_lanes);

  /*! \fn getAdjacentReverseCenterlines(const lanelet::ConstLanelets& adjacentSet,
    const lanelet::BasicPoint2d& start_point, double uptrack, std::vector<std::vector<lanelet::BasicPoint2d>>* reverse_lanes)

    \brief Helper method that is identical to getAdjacentForwardCenterlines except it works in reverse using uptrack distance
  */
  void getAdjacentReverseCenterlines(const lanelet::ConstLanelets& adjacentSet,
    const lanelet::BasicPoint2d& start_point, double uptrack, std::vector<std::vector<lanelet::BasicPoint2d>>* reverse_lanes);
  
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

};

}//traffic

#endif /* TRAFFIC_INCIDENT_PARSER_WORKER_H*/