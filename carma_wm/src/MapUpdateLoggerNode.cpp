/*
 * Copyright (C) 2021 LEIDOS.
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

#include <ros/ros.h>
#include <functional>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <cav_msgs/TrafficControlRequest.h>
#include <carma_utils/CARMAUtils.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <carma_wm/TrafficControl.h>
#include <carma_debug_msgs/MapUpdateReadable.h>
#include <carma_debug_msgs/LaneletIdRegulatoryElementPair.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/variant.hpp>
#include <memory>
#include <lanelet2_extension/regulatory_elements/DigitalMinimumGap.h>
#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <lanelet2_extension/regulatory_elements/DirectionOfTravel.h>
#include <lanelet2_extension/regulatory_elements/PassingControlLine.h>
#include <lanelet2_extension/regulatory_elements/RegionAccessRule.h>
#include <lanelet2_extension/regulatory_elements/StopRule.h>
#include <iostream>

carma_debug_msgs::LaneletIdRegulatoryElementPair pairToDebugMessage(const std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>& id_reg_pair) {
  carma_debug_msgs::LaneletIdRegulatoryElementPair pair;
  pair.lanelet_id = std::get<0>(id_reg_pair);
  auto element = std::get<1>(id_reg_pair);
  std::string rule_name = element->attribute(lanelet::AttributeName::Subtype).value();
  pair.element.rule_name = rule_name;

  // TODO
  // It might be worth creating an rviz visualization of the map updates as well. Maybe a transparent polygon with text defining the geofence type and id.
  // This will require that we instantiate a world model instance here, but we will need to make that a configurable parameter so that this node can be prevented from crashing.
  
  if (rule_name.compare("digital_minimum_gap") == 0) {
    
    auto cast = std::static_pointer_cast<lanelet::DigitalMinimumGap>(element);
    pair.element.min_gap = cast->getMinimumGap();
    pair.element.participants.insert(pair.element.participants.begin(), cast->participants_.begin(), cast->participants_.end());

  } else if (rule_name.compare("digital_speed_limit") == 0) {

    auto cast = std::static_pointer_cast<lanelet::DigitalSpeedLimit>(element);
    pair.element.speed_limit = cast->getSpeedLimit().value();
    pair.element.participants.insert(pair.element.participants.begin(), cast->participants_.begin(), cast->participants_.end());

  } else if (rule_name.compare("direction_of_travel") == 0) {

    auto cast = std::static_pointer_cast<lanelet::DirectionOfTravel>(element);
    pair.element.direction = cast->direction_;
    pair.element.participants.insert(pair.element.participants.begin(), cast->participants_.begin(), cast->participants_.end());

  } else if (rule_name.compare("passing_control_line") == 0) {

    auto cast = std::static_pointer_cast<lanelet::PassingControlLine>(element);
    pair.element.left_participants.insert(pair.element.left_participants.begin(), cast->left_participants_.begin(), cast->left_participants_.end());
    pair.element.right_participants.insert(pair.element.right_participants.begin(), cast->right_participants_.begin(), cast->right_participants_.end());

  } else if (rule_name.compare("region_access_rule") == 0) {

    auto cast = std::static_pointer_cast<lanelet::RegionAccessRule>(element);
    pair.element.reason = cast->getReason();
    pair.element.participants.insert(pair.element.participants.begin(), cast->participants_.begin(), cast->participants_.end());

  } else if (rule_name.compare("stop_rule") == 0) {

    auto cast = std::static_pointer_cast<lanelet::StopRule>(element);
    pair.element.participants.insert(pair.element.participants.begin(), cast->participants_.begin(), cast->participants_.end());

  } else {

    ROS_WARN_STREAM("MapUpdateLogger recieved unsupported regulatory element in map update.");
    pair.element.unsupported_type = true; 

  }
  return pair;
}

carma_debug_msgs::MapUpdateReadable mapUpdateCallback(const autoware_lanelet2_msgs::MapBinConstPtr& update) {
  auto control = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  carma_wm::fromBinMsg(*update, control);
  ROS_INFO_STREAM("Recieved map update ");

  carma_debug_msgs::MapUpdateReadable msg;
  msg.header = update->header;
  msg.format_version = update->format_version;
  msg.map_version = update->map_version;
  msg.route_id = update->route_id;
  msg.route_version = update->route_version;
  msg.invalidates_route = update->invalidates_route;

  msg.traffic_control_id = boost::lexical_cast<std::string>(control->id_);

  for (auto id_reg_pair : control->update_list_) {

    msg.update_list.push_back(pairToDebugMessage(id_reg_pair));
  }

  for (auto id_reg_pair : control->remove_list_) {

    msg.remove_list.push_back(pairToDebugMessage(id_reg_pair));
  }

  return msg;
}

class CallbackHolder { // For whatever reason I couldn't get a bound callback with the pub to work so this is the quick fix
  public:
    ros::Publisher pub_;
    void raw_callback(const autoware_lanelet2_msgs::MapBinConstPtr& msg) {
      pub_.publish(mapUpdateCallback(msg));
    }
};


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "map_update_logger_node");
  ros::CARMANodeHandle nh;
  
  //Check Active Geofence Publisher
  ros::Publisher readable_pub = nh.advertise<carma_debug_msgs::MapUpdateReadable>("map_update_debug", 100, true);
  // Base Map Sub
  CallbackHolder ch;
  ch.pub_ = readable_pub;

  ros::Subscriber update_sub = nh.subscribe("/environment/map_update", 100, &CallbackHolder::raw_callback, &ch);

  ros::CARMANodeHandle::spin();
};
