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

  #include "traffic_incident_parser_node.h"

  namespace traffic{

  using std::placeholders::_1;

  TrafficIncidentParserNode::TrafficIncidentParserNode():traffic_parser_worker_(wm_listener_.getWorldModel(), std::bind(&TrafficIncidentParserNode::publishTrafficControlMessage, this, _1)){}

  void TrafficIncidentParserNode::initialize()
  {
    // Setup pub/sub
    projection_sub_=nh_.subscribe("georeference",1,&TrafficIncidentParserWorker::projectionCallback,&traffic_parser_worker_);
    mobility_operation_sub_=nh_.subscribe("incoming_mobility_operation",10,&TrafficIncidentParserWorker::mobilityOperationCallback,&traffic_parser_worker_);
    traffic_control_msg_pub_=nh_.advertise<cav_msgs::TrafficControlMessage>("geofence", 100,true);
  }

  void TrafficIncidentParserNode::publishTrafficControlMessage(const cav_msgs::TrafficControlMessage& traffic_control_msg) const
  {
  	traffic_control_msg_pub_.publish(traffic_control_msg);
  }

  void TrafficIncidentParserNode::run()
  {
    initialize();
    ros::CARMANodeHandle::spin();
  }

}//traffic