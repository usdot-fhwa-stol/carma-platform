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
WMListener::WMListener(bool multi_thread)
    : Node("carma_wm"), worker_(std::unique_ptr<WMListenerWorker>(new WMListenerWorker)), multi_threaded_(multi_thread)
{
    RCLCPP_DEBUG_STREAM(get_logger(), "WMListener: Creating world model listener");
    config_speed_limit_ = declare_parameter<double>("config_speed_limit", config_speed_limit_);
    participant_ = declare_parameter<std::string>("vehicle_participant_type", participant_);

    get_parameter<double>("config_speed_limit", config_speed_limit_);
    get_parameter<std::string>("vehicle_participant_type", participant_);

    rclcpp::SubscriptionOptions map_update_options;
    rclcpp::SubscriptionOptions map_options;
    rclcpp::SubscriptionOptions route_options;
    rclcpp::SubscriptionOptions roadway_objects_options;
    rclcpp::SubscriptionOptions traffic_spat_options;

    if(multi_threaded_)
    {

      RCLCPP_DEBUG_STREAM(get_logger(), "WMListener: Using a multi-threaded subscription");
      auto map_update_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      map_update_options.callback_group = map_update_cb_group;

      auto map_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      map_options.callback_group = map_cb_group;

      auto route_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      route_options.callback_group = route_cb_group;                               

      auto roadway_objects_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      roadway_objects_options.callback_group = roadway_objects_group;                                      

      auto traffic_spat_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      traffic_spat_options.callback_group = traffic_spat_group;                                      

    }

    // Setup subscribers
    map_update_sub_ = create_subscription<autoware_lanelet2_msgs::msg::MapBin>("map_update", 200, 
                                    std::bind(&WMListener::mapUpdateCallback, this, std::placeholders::_1), map_update_options);
    map_sub_ = create_subscription<autoware_lanelet2_msgs::msg::MapBin>("semantic_map", 2, 
                                    std::bind(&WMListenerWorker::mapCallback, worker_.get(), std::placeholders::_1), map_options);
    route_sub_ = create_subscription<carma_planning_msgs::msg::Route>("roadway_objects", 1, 
                                    std::bind(&WMListenerWorker::routeCallback, worker_.get(), std::placeholders::_1), route_options);
    roadway_objects_sub_ = create_subscription<carma_perception_msgs::msg::RoadwayObstacleList>("roadway_objects", 1,
                                    std::bind(&WMListenerWorker::roadwayObjectListCallback, worker_.get(), std::placeholders::_1), roadway_objects_options);
    traffic_spat_sub_ = create_subscription<carma_v2x_msgs::msg::SPAT>("incoming_spat", 1,
                                    std::bind(&WMListenerWorker::incomingSpatCallback, worker_.get(), std::placeholders::_1), traffic_spat_options); 
}

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

void WMListener::mapUpdateCallback(const autoware_lanelet2_msgs::msg::MapBin::UniquePtr geofence_msg)
{
  const std::lock_guard<std::mutex> lock(mw_mutex_);

  RCLCPP_INFO_STREAM(get_logger(), "New Map Update Received. SeqNum: " << geofence_msg->seq_id);

  worker_->mapUpdateCallback(*geofence_msg);
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