/*
 * Copyright (C) 2020-2021 LEIDOS.
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

#include <carma_wm_ctrl/WMBroadcaster.h>
#include <carma_utils/timers/ROSTimerFactory.h>
#include <carma_wm_ctrl/WMBroadcasterNode.h>

namespace carma_wm_ctrl
{
  // @SONAR_STOP@

using std::placeholders::_1;

void WMBroadcasterNode::publishMap(const autoware_lanelet2_msgs::MapBin& map_msg)
{
  map_pub_.publish(map_msg);
}

void WMBroadcasterNode::publishMapUpdate(const autoware_lanelet2_msgs::MapBin& geofence_msg) const
{
  map_update_pub_.publish(geofence_msg);
}

  
void WMBroadcasterNode::publishCtrlReq(const cav_msgs::TrafficControlRequest& ctrlreq_msg) const
{
  control_msg_pub_.publish(ctrlreq_msg);
}

void WMBroadcasterNode::publishActiveGeofence(const cav_msgs::CheckActiveGeofence& active_geof_msg)
{
  active_pub_.publish(active_geof_msg);
}

void WMBroadcasterNode::publishTCMACK(const cav_msgs::MobilityOperation& mom_msg)
{
  tcm_ack_pub_.publish(mom_msg);
}

WMBroadcasterNode::WMBroadcasterNode()
  : wmb_(std::bind(&WMBroadcasterNode::publishMap, this, _1), std::bind(&WMBroadcasterNode::publishMapUpdate, this, _1), 
  std::bind(&WMBroadcasterNode::publishCtrlReq, this, _1), std::bind(&WMBroadcasterNode::publishActiveGeofence, this, _1),
    std::make_unique<carma_utils::timers::ROSTimerFactory>(), std::bind(&WMBroadcasterNode::publishTCMACK, this, _1)){};

int WMBroadcasterNode::run()
{
  int ack_pub_times = 1;
  pnh_.getParam("ack_pub_times", ack_pub_times);
  wmb_.setConfigACKPubTimes(ack_pub_times);
  
  // Map Publisher
  map_pub_ = cnh_.advertise<autoware_lanelet2_msgs::MapBin>("semantic_map", 1, true);
  // Map Update Publisher
  // When a new node connects to this topic that node should be provided with all previous updates for the current map version
  map_update_pub_ = cnh_.advertise<autoware_lanelet2_msgs::MapBin>("map_update", 200, [this](auto& pub){ wmb_.newUpdateSubscriber(pub); });
  //Route Message Publisher
  control_msg_pub_= cnh_.advertise<cav_msgs::TrafficControlRequest>("outgoing_geofence_request", 1, true);
  //Check Active Geofence Publisher
  active_pub_ = cnh_.advertise<cav_msgs::CheckActiveGeofence>("active_geofence", 200, true);
  //publish TCM acknowledgement after processing TCM
  tcm_ack_pub_ = cnh_.advertise<cav_msgs::MobilityOperation>("outgoing_geofence_ack", 2 * ack_pub_times , true);
  // Base Map Sub
  base_map_sub_ = cnh_.subscribe("base_map", 1, &WMBroadcaster::baseMapCallback, &wmb_);
  // Base Map Georeference Sub
  georef_sub_ = cnh_.subscribe("georeference", 1, &WMBroadcaster::geoReferenceCallback, &wmb_);
  // Geofence Sub
  geofence_sub_ = cnh_.subscribe("geofence", 200, &WMBroadcaster::geofenceCallback, &wmb_);
  // External Map Msg Sub
  incoming_map_sub_ = cnh_.subscribe("incoming_map", 20, &WMBroadcaster::externalMapMsgCallback, &wmb_);
  //Route Message Sub
  route_callmsg_sub_ = cnh_.subscribe("route", 1, &WMBroadcaster::routeCallbackMessage, &wmb_);

  //Current Location Sub
  curr_location_sub_ = cnh_.subscribe("current_pose", 1,&WMBroadcaster::currentLocationCallback, &wmb_);
  //TCM Visualizer pub
  tcm_visualizer_pub_= cnh_.advertise<visualization_msgs::MarkerArray>("tcm_visualizer",1,true);
  //TCR Visualizer pub (visualized on UI)
  tcr_visualizer_pub_ = cnh_.advertise<cav_msgs::TrafficControlRequestPolygon>("tcr_bounding_points",1,true);
  //Upcoming intersection and group id of traffic light 
  upcoming_intersection_ids_pub_ = cnh_.advertise<std_msgs::Int32MultiArray>("intersection_signal_group_ids", 1, true);

  double config_limit;
  double lane_max_width;
  std::string vehicle_id;
  double traffic_control_request_period;
  pnh_.getParam("max_lane_width", lane_max_width);
  wmb_.setMaxLaneWidth(lane_max_width);

  pnh_.getParam("traffic_control_request_period", traffic_control_request_period);
  
  std::vector<double> intersection_coord_correction;
  pnh_.getParam("intersection_coord_correction", intersection_coord_correction);
  std::vector<int> intersection_ids_for_correction;
  pnh_.getParam("intersection_ids_for_correction", intersection_ids_for_correction);
 
  wmb_.setIntersectionCoordCorrection(intersection_ids_for_correction, intersection_coord_correction);

  pnh2_.getParam("/config_speed_limit", config_limit);
  wmb_.setConfigSpeedLimit(config_limit);

  pnh2_.getParam("/vehicle_id", vehicle_id);
  wmb_.setConfigVehicleId(vehicle_id);

  std::string participant;
  pnh2_.getParam("/vehicle_participant_type", participant);
  wmb_.setVehicleParticipationType(participant);
  
    timer = cnh_.createTimer(ros::Duration(traffic_control_request_period), [this](auto){
      tcm_visualizer_pub_.publish(wmb_.tcm_marker_array_);
      tcr_visualizer_pub_.publish(wmb_.tcr_polygon_);
      wmb_.publishLightId();
      //updating upcoming traffic signal group id and intersection id
      wmb_.updateUpcomingSGIntersectionIds();
      if (wmb_.upcoming_intersection_ids_.data.size() > 0)
        upcoming_intersection_ids_pub_.publish(wmb_.upcoming_intersection_ids_);
      if(wmb_.getRoute().route_path_lanelet_ids.size() > 0)
        wmb_.routeCallbackMessage(wmb_.getRoute());
      }, false);
      
  // Spin
  cnh_.spin();
  return 0;
}


// @SONAR_START@

}  // namespace carma_wm_ctrl