/*
 * Copyright (C) 2022 LEIDOS.
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

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <autoware_lanelet2_msgs/msg/map_bin.hpp>
#include <carma_v2x_msgs/msg/traffic_control_request.hpp>
#include <carma_ros2_utils/carma_ros2_utils.hpp>
#include <autoware_lanelet2_ros2_interface/utility/message_conversion.hpp>
#include <carma_wm_ros2/TrafficControl.hpp>
#include <carma_debug_ros2_msgs/msg/map_update_readable.hpp>
#include <carma_debug_ros2_msgs/msg/lanelet_id_regulatory_element_pair.hpp>
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

/**
 * This node provides some debugging functionality for carma_wm in order to allow for inspection of map updates.
 * The node can be run with 
 *    rosrun carma_wm map_update_logger_node
 * 
 * The map updates can be inspected with 
 *    rostopic echo /map_update_debug
 */ 

class MapUpdateLogger : public rclcpp::Node
{
  public:
    MapUpdateLogger(const rclcpp::NodeOptions& options);
    
    private:
    rclcpp::Publisher<carma_debug_ros2_msgs::msg::MapUpdateReadable>::SharedPtr readable_pub_;
    rclcpp::Subscription<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr update_sub_;

    /**
     * \brief Converts a TrafficControl pair into the corresponding debug message type
     * 
     * \param id_reg_pair The pair to convert
     * 
     * \return A corresponding carma_debug_ros2_msgs::msg::LaneletIdRegulatoryElementPair
     */
    carma_debug_ros2_msgs::msg::LaneletIdRegulatoryElementPair pairToDebugMessage(const std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>& id_reg_pair);
    
    /**
     * \brief Callback for map updates that converts them to a human readable format
     * 
     * \param update Msg to convert
     * 
     * \return Returned converted message
     */ 
    carma_debug_ros2_msgs::msg::MapUpdateReadable mapUpdateCallback(const autoware_lanelet2_msgs::msg::MapBin& update);

    void raw_callback(const autoware_lanelet2_msgs::msg::MapBin::SharedPtr msg) {
      readable_pub_->publish(mapUpdateCallback(*msg));
    }
};

MapUpdateLogger::MapUpdateLogger(const rclcpp::NodeOptions& options)
  : Node("map_update_logger")
{
    // Setup publishers

    // NOTE: Currently, intra-process comms must be disabled for publishers that are transient_local: https://github.com/ros2/rclcpp/issues/1753
    rclcpp::PublisherOptions readable_pub_options; 
    readable_pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for the map update debug publisher

    auto readable_pub_qos = rclcpp::QoS(rclcpp::KeepAll()); // A publisher with this QoS will store all messages that it has sent on the topic
    readable_pub_qos.transient_local();  // A publisher with this QoS will re-send all (when KeepAll is used) messages to all late-joining subscribers 
                                         // NOTE: The subscriber's QoS must be set to transisent_local() as well for earlier messages to be resent to the later-joiner.

    // Create map update debug publisher, which will send all previously published messages to late-joining subscribers ONLY If the subscriber is transient_local too
    readable_pub_ = this->create_publisher<carma_debug_ros2_msgs::msg::MapUpdateReadable>("map_update_debug", readable_pub_qos, readable_pub_options);

    // Setup subscribers

    // NOTE: Currently, intra-process comms must be disabled for subcribers that are transient_local: https://github.com/ros2/rclcpp/issues/1753
    rclcpp::SubscriptionOptions update_sub_options; 
    update_sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for this SubscriptionOptions object

    auto update_sub_qos = rclcpp::QoS(rclcpp::KeepLast(100)); // Set the queue size for a subscriber with this QoS
    update_sub_qos.transient_local();  // If it is possible that this node is a late-joiner to its topic, it must be set to transient_local to receive earlier messages that were missed.
                                   // NOTE: The publisher's QoS must be set to transisent_local() as well for earlier messages to be resent to this later-joiner.

    // Create map update subscriber that will receive earlier messages that were missed ONLY if the publisher is transient_local too
    update_sub_ = this->create_subscription<autoware_lanelet2_msgs::msg::MapBin>("/environment/map_update", update_sub_qos,
                                                          std::bind(&MapUpdateLogger::raw_callback, this, std::placeholders::_1), update_sub_options);
}

carma_debug_ros2_msgs::msg::LaneletIdRegulatoryElementPair MapUpdateLogger::pairToDebugMessage(const std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>& id_reg_pair) {
  carma_debug_ros2_msgs::msg::LaneletIdRegulatoryElementPair pair;
  pair.lanelet_id = std::get<0>(id_reg_pair);
  auto element = std::get<1>(id_reg_pair);
  std::string rule_name = element->attribute(lanelet::AttributeName::Subtype).value();
  pair.element.rule_name = rule_name;

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

    RCLCPP_WARN_STREAM(rclcpp::get_logger("map_update_debug"), "MapUpdateLogger recieved unsupported regulatory element in map update.");
    pair.element.unsupported_type = true; 

  }
  return pair;
}


carma_debug_ros2_msgs::msg::MapUpdateReadable MapUpdateLogger::mapUpdateCallback(const autoware_lanelet2_msgs::msg::MapBin& update) {
  
  RCLCPP_INFO_STREAM(get_logger(), "Recieved map update ");

  auto control = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  carma_wm::fromBinMsg(update, control);

  carma_debug_ros2_msgs::msg::MapUpdateReadable msg;
  msg.header = update.header;
  msg.format_version = update.format_version;
  msg.map_version = update.map_version;
  msg.route_id = update.route_id;
  msg.route_version = update.route_version;
  msg.invalidates_route = update.invalidates_route;

  msg.traffic_control_id = boost::lexical_cast<std::string>(control->id_);

  for (auto id_reg_pair : control->update_list_) {

    msg.update_list.push_back(pairToDebugMessage(id_reg_pair));
  }

  for (auto id_reg_pair : control->remove_list_) {

    msg.remove_list.push_back(pairToDebugMessage(id_reg_pair));
  }

  return msg;
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(MapUpdateLogger)


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MapUpdateLogger>(rclcpp::NodeOptions()));
  
  rclcpp::shutdown();

  return 0;
}
