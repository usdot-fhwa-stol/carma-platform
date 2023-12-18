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
#include <carma_wm/WMListener.hpp>
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
    config_speed_limit_param_value = node_params_->declare_parameter("config_speed_limit", rclcpp::ParameterValue (0.0));
  }

  rclcpp::Parameter participant_param("vehicle_participant_type");
  if(!node_params_->get_parameter("vehicle_participant_type", participant_param)){
    rclcpp::ParameterValue participant_param_value;
    participant_param_value = node_params_->declare_parameter("vehicle_participant_type", rclcpp::ParameterValue(""));
  }

  //Declare parameter if it doesn't exist
  rclcpp::Parameter use_sim_time_param("use_sim_time");
  if(!node_params_->get_parameter("use_sim_time", use_sim_time_param)){
    rclcpp::ParameterValue use_sim_time_param_value;
    use_sim_time_param_value = node_params_->declare_parameter("use_sim_time", rclcpp::ParameterValue (false));
  }

  // Get params
  config_speed_limit_param = node_params_->get_parameter("config_speed_limit");
  participant_param = node_params_->get_parameter("vehicle_participant_type");
  use_sim_time_param = node_params_->get_parameter("use_sim_time");


  RCLCPP_INFO_STREAM(node_logging->get_logger(), "Loaded config speed limit: " << config_speed_limit_param.as_double());
  RCLCPP_INFO_STREAM(node_logging->get_logger(), "Loaded vehicle participant type: " << participant_param.as_string());
  RCLCPP_INFO_STREAM(node_logging->get_logger(), "Is using simulation time? : " << use_sim_time_param.as_bool());


  setConfigSpeedLimit(config_speed_limit_param.as_double());
  worker_->setVehicleParticipationType(participant_param.as_string());
  worker_->isUsingSimTime(use_sim_time_param.as_bool());

  rclcpp::SubscriptionOptions map_update_options;
  rclcpp::SubscriptionOptions map_options;
  rclcpp::SubscriptionOptions route_options;
  rclcpp::SubscriptionOptions roadway_objects_options;
  rclcpp::SubscriptionOptions traffic_spat_options;
  rclcpp::SubscriptionOptions ros1_clock_options;
  rclcpp::SubscriptionOptions sim_clock_options;

  if(multi_threaded_)
  {
    RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "WMListener: Using a multi-threaded subscription");

    map_update_options.callback_group = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    map_options.callback_group = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    route_options.callback_group = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    roadway_objects_options.callback_group = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    traffic_spat_options.callback_group = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    ros1_clock_options.callback_group = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    sim_clock_options.callback_group = node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  // Setup subscribers
  route_sub_ = rclcpp::create_subscription<carma_planning_msgs::msg::Route>(node_topics_, "route", 1,
                                  [this](const carma_planning_msgs::msg::Route::SharedPtr msg)
                                  {
                                    this->worker_->routeCallback(msg);
                                  }
                                  , route_options);

  ros1_clock_sub_ = rclcpp::create_subscription<rosgraph_msgs::msg::Clock>(node_topics_, "/clock", 1,
                                  [this](const rosgraph_msgs::msg::Clock::SharedPtr msg)
                                  {
                                    this->worker_->ros1ClockCallback(msg);
                                  }
                                  , ros1_clock_options);

  sim_clock_sub_ = rclcpp::create_subscription<rosgraph_msgs::msg::Clock>(node_topics_, "/sim_clock", 1,
                                  [this](const rosgraph_msgs::msg::Clock::SharedPtr msg)
                                  {
                                    this->worker_->simClockCallback(msg);
                                  }
                                  , sim_clock_options);

  roadway_objects_sub_ = rclcpp::create_subscription<carma_perception_msgs::msg::RoadwayObstacleList>(node_topics_, "roadway_objects", 1,
                                  [this](const carma_perception_msgs::msg::RoadwayObstacleList::SharedPtr msg)
                                  {
                                    this->worker_->roadwayObjectListCallback(msg);
                                  }
                                  , roadway_objects_options);

  traffic_spat_sub_ = rclcpp::create_subscription<carma_v2x_msgs::msg::SPAT>(node_topics_, "incoming_spat", 1,
                                  [this](const carma_v2x_msgs::msg::SPAT::SharedPtr msg)
                                  {
                                    this->worker_->incomingSpatCallback(msg);
                                  }
                                  , traffic_spat_options);

  // NOTE: Currently, intra-process comms must be disabled for subscribers that are transient_local: https://github.com/ros2/rclcpp/issues/1753
  map_update_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for the map update subscriber
  auto map_update_sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)); // Set the queue size for the map update subscriber
  map_update_sub_qos.transient_local();  // If it is possible that this node is a late-joiner to its topic, it must be set to transient_local to receive earlier messages that were missed.
                                         // NOTE: The publisher's QoS must be set to transisent_local() as well for earlier messages to be resent to this later-joiner.

  // Create map update subscriber that will receive earlier messages that were missed ONLY if the publisher is transient_local too
  map_update_sub_ = rclcpp::create_subscription<autoware_lanelet2_msgs::msg::MapBin>(node_topics_, "map_update", map_update_sub_qos,
                  [this](const autoware_lanelet2_msgs::msg::MapBin::SharedPtr msg)
                  {
                    this->mapUpdateCallback(msg);
                  }
                  , map_update_options);

  map_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for the semantic map subscriber
  auto map_sub_qos = rclcpp::QoS(rclcpp::KeepLast(2)); // Set the queue size for the semantic map subscriber
  map_sub_qos.transient_local();  // If it is possible that this node is a late-joiner to its topic, it must be set to transient_local to receive earlier messages that were missed.
                                  // NOTE: The publisher's QoS must be set to transisent_local() as well for earlier messages to be resent to this later-joiner.

  // Create semantic mab subscriber that will receive earlier messages that were missed ONLY if the publisher is transient_local too
  map_sub_ = rclcpp::create_subscription<autoware_lanelet2_msgs::msg::MapBin>(node_topics_, "semantic_map", map_sub_qos,
                  [this](const autoware_lanelet2_msgs::msg::MapBin::SharedPtr msg)
                  {
                    this->worker_->mapCallback(msg);
                  }
                  , map_options);
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

void WMListener::mapUpdateCallback(autoware_lanelet2_msgs::msg::MapBin::SharedPtr geofence_msg)
{
  const std::lock_guard<std::mutex> lock(mw_mutex_);

  RCLCPP_INFO_STREAM(node_logging_->get_logger(), "New Map Update Received. SeqNum: " << geofence_msg->seq_id);

  worker_->mapUpdateCallback(geofence_msg);
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