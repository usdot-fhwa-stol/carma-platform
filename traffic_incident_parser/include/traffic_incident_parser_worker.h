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

  using PublishTrafficControlCallback = std::function<void(const cav_msgs::TrafficControlMessageV01&)>;

  /*!
   * \brief Constructor
   */
  TrafficIncidentParserWorker(carma_wm::WorldModelConstPtr wm,PublishTrafficControlCallback traffic_control_pub);
    
  /*! \fn projectionCallback(const std_msgs::String &projection_msg)
    \brief projectionCallback to convert ECEF to MAP frame
    \param  std_msgs::String
  */

  void projectionCallback(const std_msgs::String &projection_msg);

    /*! \fn mobilityOperationCallback(const cav_msgs::MobilityOperation &mobility_msg)
    \brief mobilityOperationCallback to receive the incoming mobility operation message from message node
    \param  cav_msgs::MobilityOperation
  */

  void mobilityOperationCallback(const cav_msgs::MobilityOperation &mobility_msg);

    /*! \fn mobilityMessageParser(std::string mobility_strategy_params)
    \brief mobilityMessageParser helps to parse incoming mobility operation message to required format
    \param  std::string mobility_strategy_params
  */

  void mobilityMessageParser(std::string mobility_strategy_params);

    /*! \fn stringParserHelper(std::string str,int str_index)
    \brief stringParserHelper helps in string to required data type conversion
    \param  std::string str,int str_index
  */

  std::string stringParserHelper(std::string str,int str_index);
  
    /*! \fn composeTrafficControlMesssage()
    \brief composeTrafficControlMesssage holds the algorith for extracting the closed lanelet and assign it to traffic control message, which is then published.
  */

  cav_msgs::TrafficControlMessageV01 composeTrafficControlMesssage();

    /*! \fnearthToMapFrame()
    \brief earthToMapFrame converts ECEF to MAP fram using projection string
  */

  lanelet::BasicPoint2d earthToMapFrame();

  double latitude;
  double longitude;
  double closed_lane;
  double down_track;
  double up_track;
  std::string previous_strategy_params={};

 private:

  lanelet::BasicPoint2d local_point_;
  std::string projection_msg_;
  PublishTrafficControlCallback traffic_control_pub_;// local copy of external object publihsers
  carma_wm::WorldModelConstPtr wm_;

};

}//traffic

#endif /* TRAFFIC_INCIDENT_PARSER_WORKER_H*/