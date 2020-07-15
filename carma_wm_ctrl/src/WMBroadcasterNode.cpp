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

#include <carma_wm_ctrl/WMBroadcaster.h>
#include <carma_wm_ctrl/ROSTimerFactory.h>
#include <carma_wm_ctrl/WMBroadcasterNode.h>

namespace carma_wm_ctrl
{
using std::placeholders::_1;

void WMBroadcasterNode::publishMap(const autoware_lanelet2_msgs::MapBin& map_msg)
{
  map_pub_.publish(map_msg);
}

void WMBroadcasterNode::publishMapUpdate(const autoware_lanelet2_msgs::MapBin& geofence_msg) const
{
  map_update_pub_.publish(geofence_msg);
}

void WMBroadcasterNode::publishRouteMsg(const cav_msgs::RouteConstPtr route_msg)
{
  route_callmsg_pub_.publish(route_msg);
}
WMBroadcasterNode::WMBroadcasterNode()
  : wmb_(std::bind(&WMBroadcasterNode::publishMap, this, _1), std::bind(&WMBroadcasterNode::publishMapUpdate, this, _1), 
  std::bind(&WMBroadcasterNode::publishRouteMsg, this, _1),
    std::make_unique<ROSTimerFactory>()){};

int WMBroadcasterNode::run()
{
  // Map Publisher
  map_pub_ = cnh_.advertise<autoware_lanelet2_msgs::MapBin>("semantic_map", 1, true);
  // Map Update Publisher
  map_update_pub_ = cnh_.advertise<autoware_lanelet2_msgs::MapBin>("map_update", 1, true);
  //Route Message Publisher
  route_callmsg_pub_= <const cav_msgs::RouteConstPtr>("route", 1, true);
  // Base Map Sub
  base_map_sub_ = cnh_.subscribe("base_map", 1, &WMBroadcaster::baseMapCallback, &wmb_);
  // Base Map Georeference Sub
  georef_sub_ = cnh_.subscribe("georeference", 1, &WMBroadcaster::geoReferenceCallback, &wmb_);
  // Geofence Sub
  geofence_sub_ = cnh_.subscribe("geofence", 1, &WMBroadcaster::geofenceCallback, &wmb_);
  //Route Message Sub
  route_callmsg_sub_ = cnh_.subscribe("route", 1, &WMBroadcaster::routeCallbackMessage, &wmb_);
  
  double lane_max_width;
  pnh_.getParam("max_lane_width", lane_max_width);
  wmb_.setMaxLaneWidth(lane_max_width);
  
  // Spin
  cnh_.setSpinRate(10);
  cnh_.spin();
  return 0;
}

}  // namespace carma_wm_ctrl
