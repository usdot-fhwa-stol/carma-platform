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
#include <sstream>

namespace traffic{

class TrafficIncidentParserWorker
{

 public:

  using PublishTrafficControlCallback = std::function<void(const cav_msgs::TrafficControlMessageV01&)>;

  /*!
   * \brief Constructor
   */
  TrafficIncidentParserWorker(PublishTrafficControlCallback traffic_control_pub);
    
  /*! \fn pinpointDriverCallback(const gps_common::GPSFix &pinpoint_msg)
    \brief pinpointDriverCallback populates lat lon heading from pinpoint driver.
    \param  gps_common::GPSFix.
  */

  void projectionCallback(const projection::projection &projection_msg);

    /*! \fn pinpointDriverCallback(const gps_common::GPSFix &pinpoint_msg)
    \brief pinpointDriverCallback populates lat lon heading from pinpoint driver.
    \param  gps_common::GPSFix.
  */

  void mobilityOperationCallback(const cav_msgs::MobilityOperation &mobility_msg);

  // Setters for the prediction parameters
  //void setSenderId(std::string sender_id);
  //void setClosedLane(std::string closed_lane);
  //void setDownTrack(double down_track);
  //void setUpTrack(double up_track);

  // Generate mobility message
  cav_msgs::MobilityOperation mobilityMessageGenerator(const gps_common::GPSFix& msg);

  void mobilityMessageParser(string mobility_strategy_params);

  string stringParserHelper(string str,int str_index);

 private:
  double latitude_;
  double longitude_;
  double closed_lane_;
  double down_track_;
  double up_track_;
  // local copy of external object publihsers

  PublishTrafficControlCallback traffic_control_pub_;
 
 // Prediction parameters
 // std::string sender_id_ = "USDOT-49096";
 // std::string closed_lane_= "[1]";
 // double down_track_= 50.0;
 // double up_track_= 50.0;

};

}//traffic

#endif /* TRAFFIC_INCIDENT_PARSER_WORKER_H*/