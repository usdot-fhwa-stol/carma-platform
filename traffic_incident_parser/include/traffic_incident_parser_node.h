/*
 * Copyright (C) 2020 LEIDOS.
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

#ifndef TRAFFIC_INCIDENT_PARSER_H
#define TRAFFIC_INCIDENT_PARSER_H

#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <functional>
#include <string>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include "traffic_incident_parser_worker.h"

namespace traffic{

class TrafficIncidentParserNode
{

 private:
  
  //node handle
  ros::CARMANodeHandle nh_;
  ros::CARMANodeHandle pnh_{"~"};
   
  //subscriber
  ros::Subscriber projection_sub_;
  ros::Subscriber mobility_operation_sub_;

  // World Model Listener. Must be declared before object_worker_ for proper initialization.
  carma_wm::WMListener wm_listener_;

  //publisher
  ros::Publisher traffic_control_msg_pub_;
  
  //TrafficIncidentParserWorker class object
  TrafficIncidentParserWorker traffic_parser_worker_;
  
    /*!fn initialize()
  \brief initialize this node before running
  */
  void initialize();

 public:
  
  /*! \fn TrafficIncidentParserNode()
    \brief TrafficIncidentParserNode constructor 
  */
  TrafficIncidentParserNode();

  /*! \fn publishTrafficControlMessage()
    \brief Publish traffic control message
  */
  void publishTrafficControlMessage(const cav_msgs::TrafficControlMessage& traffic_control_msg) const;

  /*!fn run()
    \brief General starting point to run this node
  */
  void run();
  
};

}//traffic

#endif /* TRAFFIC_INCIDENT_PARSER_H */