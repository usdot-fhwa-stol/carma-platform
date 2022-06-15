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

#include <carma_ros2_utils/timers/ROSTimerFactory.hpp>
#include <carma_wm_ctrl/WMBroadcasterNode.hpp>

namespace carma_wm_ctrl
{
  // @SONAR_STOP@

namespace std_ph = std::placeholders;


void WMBroadcasterNode::publishMap(const autoware_lanelet2_msgs::msg::MapBin& map_msg)
{
  map_pub_->publish(map_msg);
}

void WMBroadcasterNode::publishMapUpdate(const autoware_lanelet2_msgs::msg::MapBin& geofence_msg) const
{
  map_update_pub_->publish(geofence_msg);
}

void WMBroadcasterNode::publishCtrlReq(const carma_v2x_msgs::msg::TrafficControlRequest& ctrlreq_msg) const
{
  control_msg_pub_->publish(ctrlreq_msg);
}

void WMBroadcasterNode::publishActiveGeofence(const carma_perception_msgs::msg::CheckActiveGeofence& active_geof_msg)
{
  active_pub_->publish(active_geof_msg);
}

void WMBroadcasterNode::publishTCMACK(const carma_v2x_msgs::msg::MobilityOperation& mom_msg)
{
  tcm_ack_pub_->publish(mom_msg);
}

WMBroadcasterNode::WMBroadcasterNode(const rclcpp::NodeOptions &options)
  : carma_ros2_utils::CarmaLifecycleNode(options) 
{
  // Create initial config
  config_ = Config();
  
  config_.ack_pub_times = declare_parameter<int>("ack_pub_times", config_.ack_pub_times);
  config_.max_lane_width = declare_parameter<double>("max_lane_width", config_.max_lane_width);
  config_.traffic_control_request_period = declare_parameter<double>("traffic_control_request_period", config_.traffic_control_request_period);
  config_.vehicle_id = declare_parameter<std::string>("/vehicle_id", config_.vehicle_id);
  config_.participant = declare_parameter<std::string>("/vehicle_participant_type", config_.participant);
  
  declare_parameter("intersection_ids_for_correction");
  declare_parameter("intersection_coord_correction");
};

void WMBroadcasterNode::initializeWorker(std::weak_ptr<carma_ros2_utils::CarmaLifecycleNode> weak_node_pointer)
{
  wmb_ = std::make_unique<carma_wm_ctrl::WMBroadcaster>(std::bind(&WMBroadcasterNode::publishMap, this, std_ph::_1), std::bind(&WMBroadcasterNode::publishMapUpdate, this, std_ph::_1), 
  std::bind(&WMBroadcasterNode::publishCtrlReq, this, std_ph::_1), std::bind(&WMBroadcasterNode::publishActiveGeofence, this, std_ph::_1),
    std::make_unique<carma_ros2_utils::timers::ROSTimerFactory>(weak_node_pointer), 
    std::bind(&WMBroadcasterNode::publishTCMACK, this, std_ph::_1));
}

carma_ros2_utils::CallbackReturn WMBroadcasterNode::handle_on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_mw_ctrl"),"Starting configuration!");

  // Reset config
  config_ = Config();

  initializeWorker(shared_from_this());

  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_mw_ctrl"),"Done initializing worker!");

  get_parameter<int>("ack_pub_times", config_.ack_pub_times);
  get_parameter<double>("max_lane_width", config_.max_lane_width);
  get_parameter<double>("traffic_control_request_period", config_.traffic_control_request_period);
  get_parameter<std::string>("/vehicle_id", config_.vehicle_id);
  get_parameter<std::string>("/vehicle_participant_type", config_.participant);
  get_parameter<double>("/config_speed_limit", config_.config_limit);
  
  wmb_->setConfigACKPubTimes(config_.ack_pub_times);
  wmb_->setMaxLaneWidth(config_.max_lane_width);
  wmb_->setConfigSpeedLimit(config_.config_limit);
  wmb_->setConfigVehicleId(config_.vehicle_id);
  wmb_->setVehicleParticipationType(config_.participant);

  rclcpp::Parameter intersection_coord_correction_param = get_parameter("intersection_coord_correction");
  config_.intersection_coord_correction = intersection_coord_correction_param.as_double_array();

  rclcpp::Parameter intersection_ids_for_correction_param = get_parameter("intersection_ids_for_correction");
  config_.intersection_ids_for_correction = intersection_ids_for_correction_param.as_integer_array();
  
  wmb_->setIntersectionCoordCorrection(config_.intersection_ids_for_correction, config_.intersection_coord_correction);
  
  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_mw_ctrl"),"Done loading parameters: " << config_);

  /////////////
  // PUBLISHERS
  /////////////

  // NOTE: Currently, intra-process comms must be disabled for the following two publishers that are transient_local: https://github.com/ros2/rclcpp/issues/1753
  rclcpp::PublisherOptions intra_proc_disabled; 
  intra_proc_disabled.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for this PublisherOptions object

  // Create a publisher that will send all previously published messages to late-joining subscribers ONLY If the subscriber is transient_local too
  auto pub_qos_transient_local = rclcpp::QoS(rclcpp::KeepAll()); // A publisher with this QoS will store all messages that it has sent on the topic
  pub_qos_transient_local.transient_local();  // A publisher with this QoS will re-send all (when KeepAll is used) messages to all late-joining subscribers 
                                         // NOTE: The subscriber's QoS must be set to transient_local() as well for earlier messages to be resent to the later-joiner.
  
  // Map Update Publisher
  map_update_pub_ = create_publisher<autoware_lanelet2_msgs::msg::MapBin>("map_update", pub_qos_transient_local, intra_proc_disabled);
  
  // Map Publisher
  map_pub_ = create_publisher<autoware_lanelet2_msgs::msg::MapBin>("semantic_map", pub_qos_transient_local, intra_proc_disabled);
  
  //Route Message Publisher
  control_msg_pub_= create_publisher<carma_v2x_msgs::msg::TrafficControlRequest>("outgoing_geofence_request", 1);

  //Check Active Geofence Publisher
  active_pub_ = create_publisher<carma_perception_msgs::msg::CheckActiveGeofence>("active_geofence", 200);

  //publish TCM acknowledgement after processing TCM
  tcm_ack_pub_ = create_publisher<carma_v2x_msgs::msg::MobilityOperation>("outgoing_geofence_ack", 2 * config_.ack_pub_times );

  //TCM Visualizer pub
  tcm_visualizer_pub_= create_publisher<visualization_msgs::msg::MarkerArray>("tcm_visualizer",1);

  //TCR Visualizer pub (visualized on UI)
  tcr_visualizer_pub_ = create_publisher<carma_v2x_msgs::msg::TrafficControlRequestPolygon>("tcr_bounding_points",1);

  //Upcoming intersection and group id of traffic light 
  upcoming_intersection_ids_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("intersection_signal_group_ids", 1);
  
  // Return success if everything initialized successfully
  
  return CallbackReturn::SUCCESS;
}

carma_ros2_utils::CallbackReturn WMBroadcasterNode::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
{
  // Timer setup
  timer_ = create_timer(get_clock(),
                          std::chrono::milliseconds((int)(config_.traffic_control_request_period * 1000)),
                          std::bind(&WMBroadcasterNode::spin_callback, this));
  
  /////////////
  //SUBSCRIBERS
  ///////////// NOTE: subscriber declaration delayed until here so that when map is received, publisher is already activated to immediately publish back

  // Base Map Sub
  base_map_sub_ = create_subscription<autoware_lanelet2_msgs::msg::MapBin>("base_map", 1, std::bind(&WMBroadcaster::baseMapCallback, wmb_.get(), std_ph::_1));
  
  // Base Map Georeference Sub
  georef_sub_ = create_subscription<std_msgs::msg::String>("georeference", 1, std::bind(&WMBroadcaster::geoReferenceCallback, wmb_.get(), std_ph::_1));
  
  // Geofence Sub
  geofence_sub_ = create_subscription<carma_v2x_msgs::msg::TrafficControlMessage>("geofence", 200, std::bind(&WMBroadcaster::geofenceCallback, wmb_.get(), std_ph::_1));
  
  // External Map Msg Sub
  incoming_map_sub_ = create_subscription<carma_v2x_msgs::msg::MapData>("incoming_map", 20, std::bind(&WMBroadcaster::externalMapMsgCallback, wmb_.get(), std_ph::_1));
  
  //Route Message Sub
  route_callmsg_sub_ = create_subscription<carma_planning_msgs::msg::Route>("route", 1, std::bind(&WMBroadcaster::routeCallbackMessage, wmb_.get(), std_ph::_1));
  
  //Current Location Sub
  curr_location_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 1, std::bind(&WMBroadcaster::currentLocationCallback, wmb_.get(), std_ph::_1));
  
  
  return CallbackReturn::SUCCESS;
}

bool WMBroadcasterNode::spin_callback()
{
  tcm_visualizer_pub_->publish(wmb_->tcm_marker_array_);
  tcr_visualizer_pub_->publish(wmb_->tcr_polygon_);
  wmb_->publishLightId();
  //updating upcoming traffic signal group id and intersection id
  wmb_->updateUpcomingSGIntersectionIds();
  if (wmb_->upcoming_intersection_ids_.data.size() > 0)
    upcoming_intersection_ids_pub_->publish(wmb_->upcoming_intersection_ids_);
  if(wmb_->getRoute().route_path_lanelet_ids.size() > 0)
    wmb_->routeCallbackMessage(std::make_unique<carma_planning_msgs::msg::Route>(wmb_->getRoute()));

  return true;
}



// @SONAR_START@

}  // namespace carma_wm_ctrl

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(carma_wm_ctrl::WMBroadcasterNode)