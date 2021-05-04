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
  std::cerr << "A " << std::endl;
  pair.lanelet_id = std::get<0>(id_reg_pair);
  std::cerr << "B" << std::endl;
  auto element = std::get<1>(id_reg_pair);
  std::cerr << "C" << std::endl;
  pair.element.rule_name = element->RuleName;
  std::cerr << "D" << std::endl;

  // TODO
  // It might be worth creating an rviz visualization of the map updates as well. Maybe a transparent polygon with text defining the geofence type and id.
  // This will require that we instantiate a world model instance here, but we will need to make that a configurable parameter so that this node can be prevented from crashing.
  std::string rule_name = std::string(element->RuleName);
  std::cerr << "M" << std::endl;
  
  if (rule_name.compare("digital_minimum_gap") == 0) {
    std::cerr << "N" << std::endl;
    
    auto cast = std::static_pointer_cast<lanelet::DigitalMinimumGap>(element);
    std::cerr << "O" << std::endl;
    pair.element.min_gap = cast->getMinimumGap();
    std::cerr << "P" << std::endl;
    pair.element.participants.insert(pair.element.participants.begin(), cast->participants_.begin(), cast->participants_.end());
    std::cerr << "Q" << std::endl;
  } else if (rule_name.compare("digital_speed_limit") == 0) {
    std::cerr << "R" << std::endl;
    auto cast = std::static_pointer_cast<lanelet::DigitalSpeedLimit>(element);
    std::cerr << "S" << std::endl;
    pair.element.min_gap = cast->getSpeedLimit().value();
    std::cerr << "T" << std::endl;
    pair.element.participants.insert(pair.element.participants.begin(), cast->participants_.begin(), cast->participants_.end());
    std::cerr << "U" << std::endl;
  } else if (rule_name.compare("direction_of_travel") == 0) {
    std::cerr << "V" << std::endl;
    auto cast = std::static_pointer_cast<lanelet::DirectionOfTravel>(element);
    std::cerr << "W" << std::endl;
    pair.element.direction = cast->direction_;
    std::cerr << "X" << std::endl;
    pair.element.participants.insert(pair.element.participants.begin(), cast->participants_.begin(), cast->participants_.end());
    std::cerr << "Y" << std::endl;
  } else if (rule_name.compare("passing_control_line") == 0) {
    std::cerr << "Z" << std::endl;
    auto cast = std::static_pointer_cast<lanelet::PassingControlLine>(element);
    std::cerr << "AA" << std::endl;
    pair.element.left_participants.insert(pair.element.left_participants.begin(), cast->left_participants_.begin(), cast->left_participants_.end());
    std::cerr << "BB" << std::endl;
    pair.element.right_participants.insert(pair.element.right_participants.begin(), cast->right_participants_.begin(), cast->right_participants_.end());
    std::cerr << "CC" << std::endl;
  } else if (rule_name.compare("region_access_rule") == 0) {
    std::cerr << "DD" << std::endl;
    auto cast = std::static_pointer_cast<lanelet::RegionAccessRule>(element);
    std::cerr << "EE" << std::endl;
    pair.element.reason = cast->getReason();
    std::cerr << "FF" << std::endl;
    pair.element.participants.insert(pair.element.participants.begin(), cast->participants_.begin(), cast->participants_.end());
    std::cerr << "GG" << std::endl;
  } else if (rule_name.compare("stop_rule") == 0) {
    std::cerr << "HH" << std::endl;
    auto cast = std::static_pointer_cast<lanelet::StopRule>(element);
    std::cerr << "II" << std::endl;
    pair.element.participants.insert(pair.element.participants.begin(), cast->participants_.begin(), cast->participants_.end());
    std::cerr << "JJ" << std::endl;
  } else {

    ROS_WARN_STREAM("MapUpdateLogger recieved unsupported regulatory element in map update.");
    pair.element.unsupported_type = true; 

  }
  std::cerr << "KK" << std::endl;
  return pair;
}

carma_debug_msgs::MapUpdateReadable mapUpdateCallback(const autoware_lanelet2_msgs::MapBinConstPtr& update) {
  auto control = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  carma_wm::fromBinMsg(*update, control);
  std::cerr << "Recieved map update " << std::endl;

  carma_debug_msgs::MapUpdateReadable msg;
  std::cerr << "1" << std::endl;
  msg.header = update->header;
  std::cerr << "2 " << std::endl;
  msg.format_version = update->format_version;
  std::cerr << "3 " << std::endl;
  msg.map_version = update->map_version;
  std::cerr << "4 " << std::endl;
  msg.route_id = update->route_id;
  std::cerr << "5 " << std::endl;
  msg.route_version = update->route_version;
  std::cerr << "6" << std::endl;
  msg.invalidates_route = update->invalidates_route;
  std::cerr << "7 " << std::endl;

  msg.traffic_control_id = boost::lexical_cast<std::string>(control->id_);
  std::cerr << "8" << std::endl;

  for (auto id_reg_pair : control->update_list_) {
    std::cerr << "9-loop" << std::endl;
    msg.update_list.push_back(pairToDebugMessage(id_reg_pair));
    std::cerr << "10-loop" << std::endl;
  }

  for (auto id_reg_pair : control->remove_list_) {
    std::cerr << "11-loop " << std::endl;
    msg.remove_list.push_back(pairToDebugMessage(id_reg_pair));
    std::cerr << "12-loop " << std::endl;
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
