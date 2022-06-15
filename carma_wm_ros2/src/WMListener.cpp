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

#include <new>
#include <carma_wm_ros2/WMListener.hpp>
#include "WMListenerWorker.hpp"


namespace carma_wm
{
  // @SONAR_STOP@
WMListener::WMListener(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params_,
  bool multi_thread)
    :node_base_(node_base),node_logging_(node_logging),node_topics_(node_topics),worker_(std::unique_ptr<WMListenerWorker>(new WMListenerWorker)), multi_threaded_(multi_thread)
{

  RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "WMListener: Creating world model listener");

  //Declare parameter if it doesn't exist
  rclcpp::Parameter config_speed_limit_param("config_speed_limit");
  if(!node_params_->get_parameter("config_speed_limit", config_speed_limit_param)){
    rclcpp::ParameterValue config_speed_limit_param_value;
    config_speed_limit_param_value = node_params_->declare_parameter("config_speed_limit", rclcpp::ParameterValue (config_speed_limit_));
  }

  rclcpp::Parameter participant_param("vehicle_participant_type");
  if(!node_params_->get_parameter("vehicle_participant_type", participant_param)){
    rclcpp::ParameterValue participant_param_value;
    participant_param_value = node_params_->declare_parameter("vehicle_participant_type", rclcpp::ParameterValue(participant_));
  }
  
  // Get params
  node_params_->get_parameter("config_speed_limit");
  node_params_->get_parameter("vehicle_participant_type");

  rclcpp::SubscriptionOptions map_update_options;
  rclcpp::SubscriptionOptions map_options;
  rclcpp::SubscriptionOptions route_options;
  rclcpp::SubscriptionOptions roadway_objects_options;
  rclcpp::SubscriptionOptions traffic_spat_options;

  if(multi_threaded_)
  {
    RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "WMListener: Using a multi-threaded subscription");

    auto map_update_cb_group = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    map_update_options.callback_group = map_update_cb_group;

    auto map_cb_group = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    map_options.callback_group = map_cb_group;

    auto route_cb_group = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    route_options.callback_group = route_cb_group;                               

    auto roadway_objects_group = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    roadway_objects_options.callback_group = roadway_objects_group;                                      

    auto traffic_spat_group = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    traffic_spat_options.callback_group = traffic_spat_group;  

  }  

  // Setup subscribers
  route_sub_ = rclcpp::create_subscription<carma_planning_msgs::msg::Route>(node_topics_, "route", 1, 
                                  std::bind(&WMListenerWorker::routeCallback, worker_.get(), std::placeholders::_1), route_options);

  roadway_objects_sub_ = rclcpp::create_subscription<carma_perception_msgs::msg::RoadwayObstacleList>(node_topics_, "roadway_objects", 1,
                                  std::bind(&WMListenerWorker::roadwayObjectListCallback, worker_.get(), std::placeholders::_1), roadway_objects_options);

  traffic_spat_sub_ = rclcpp::create_subscription<carma_v2x_msgs::msg::SPAT>(node_topics_, "incoming_spat", 1,
                                  std::bind(&WMListenerWorker::incomingSpatCallback, worker_.get(), std::placeholders::_1), traffic_spat_options); 

  // NOTE: Currently, intra-process comms must be disabled for subscribers that are transient_local: https://github.com/ros2/rclcpp/issues/1753
  map_update_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for the map update subscriber
  auto map_update_sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)); // Set the queue size for the map update subscriber
  map_update_sub_qos.transient_local();  // If it is possible that this node is a late-joiner to its topic, it must be set to transient_local to receive earlier messages that were missed.
                                         // NOTE: The publisher's QoS must be set to transisent_local() as well for earlier messages to be resent to this later-joiner.

  // Create map update subscriber that will receive earlier messages that were missed ONLY if the publisher is transient_local too
  map_update_sub_ = rclcpp::create_subscription<autoware_lanelet2_msgs::msg::MapBin>(node_topics_, "map_update", map_update_sub_qos, 
                  std::bind(&WMListener::mapUpdateCallback, this, std::placeholders::_1), map_update_options);

  map_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for the semantic map subscriber
  auto map_sub_qos = rclcpp::QoS(rclcpp::KeepLast(2)); // Set the queue size for the semantic map subscriber
  map_sub_qos.transient_local();  // If it is possible that this node is a late-joiner to its topic, it must be set to transient_local to receive earlier messages that were missed.
                                  // NOTE: The publisher's QoS must be set to transisent_local() as well for earlier messages to be resent to this later-joiner.

  // Create semantic mab subscriber that will receive earlier messages that were missed ONLY if the publisher is transient_local too
  map_sub_ = rclcpp::create_subscription<autoware_lanelet2_msgs::msg::MapBin>(node_topics_, "semantic_map", map_sub_qos,
                                  std::bind(&WMListenerWorker::mapCallback, worker_.get(), std::placeholders::_1), map_options);
}

WMListener::~WMListener() {}

void WMListener::enableUpdatesWithoutRouteWL()
{
  const std::lock_guard<std::mutex> lock(mw_mutex_);
  worker_->enableUpdatesWithoutRoute();
}

bool WMListener::checkIfReRoutingNeededWL()
{
  const std::lock_guard<std::mutex> lock(mw_mutex_);
  return worker_->checkIfReRoutingNeeded();
}

WorldModelConstPtr WMListener::getWorldModel()
{
  const std::lock_guard<std::mutex> lock(mw_mutex_);
  return worker_->getWorldModel();
}

void WMListener::mapUpdateCallback(autoware_lanelet2_msgs::msg::MapBin::UniquePtr geofence_msg)
{
  const std::lock_guard<std::mutex> lock(mw_mutex_);

  RCLCPP_INFO_STREAM(node_logging_->get_logger(), "New Map Update Received. SeqNum: " << geofence_msg->seq_id);

  worker_->mapUpdateCallback(std::move(geofence_msg));
}

void WMListener::setMapCallback(std::function<void()> callback)
{
  const std::lock_guard<std::mutex> lock(mw_mutex_);
  worker_->setMapCallback(callback);
}

void WMListener::setRouteCallback(std::function<void()> callback)
{
  const std::lock_guard<std::mutex> lock(mw_mutex_);
  worker_->setRouteCallback(callback);
}

std::unique_lock<std::mutex> WMListener::getLock(bool pre_locked)
{
  if (pre_locked)
  {
    std::unique_lock<std::mutex> lock(mw_mutex_);  // Create movable lock
    return lock;
  }
  std::unique_lock<std::mutex> lock(mw_mutex_, std::defer_lock);  // Create movable deferred lock
  return lock;
}

void WMListener::setConfigSpeedLimit(double config_lim) const
{
  worker_->setConfigSpeedLimit(config_lim);
}

// @SONAR_START@

}