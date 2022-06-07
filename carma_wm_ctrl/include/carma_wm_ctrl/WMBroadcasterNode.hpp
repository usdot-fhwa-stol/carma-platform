#pragma once

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

#include <functional>
#include <autoware_lanelet2_msgs/msg/map_bin.hpp>
#include <carma_v2x_msgs/msg/traffic_control_request.hpp>
#include <carma_ros2_utils/carma_ros2_utils.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <carma_wm_ctrl/WMBroadcaster.hpp>
#include <carma_wm_ctrl/WMBroadcasterConfig.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

#include <memory>

namespace carma_wm_ctrl
{

class WMBroadcaster;

/*!
 * \brief Node which provies exposes map publication and carma_wm update logic
 *
 * The WMBroadcasterNode handles updating the lanelet2 base map and publishing the new versions to the rest of the CARMA
 * Platform ROS network. The broadcaster also provides functions for adding or removing geofences from the map and
 * notifying the rest of the system.
 *
 */
class WMBroadcasterNode : public carma_ros2_utils::CarmaLifecycleNode 
{
public:
  /**
   * @brief Constructor
   */
  explicit WMBroadcasterNode(const rclcpp::NodeOptions& options);
    
  ////
  // Overrides
  ////
  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
  carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);

  /**
   * @brief Callback to publish a map
   *
   * @param map_msg The map message to publish
   */
  void publishMap(const autoware_lanelet2_msgs::msg::MapBin& map_msg);
  
  /**
   * @brief Initializes the WMBroadcaster worker with reference to the CarmaLifecycleNode itself
   *
   * @param weak_ptr to the node of type CarmaLifecycleNode that owns this worker
   */
  void initializeWorker(std::weak_ptr<carma_ros2_utils::CarmaLifecycleNode> weak_node_pointer);

  /**
   * @brief Callback to publish TrafficControlRequest Messages
   *
   * @param route_msg The TrafficControlRequest message to publish
   */
  void publishCtrlReq(const carma_v2x_msgs::msg::TrafficControlRequest& ctrlreq_msg) const;

  /**
   * @brief Callback to publish map updates (geofences)
   *
   * @param geofence_msg The geofence message to publish
   */
  void publishMapUpdate(const autoware_lanelet2_msgs::msg::MapBin& geofence_msg) const;

   /**
   * @brief Callback to publish active geofence information
   *
   * @param active_geof_msg The geofence information to publish
   */
  void publishActiveGeofence(const carma_perception_msgs::msg::CheckActiveGeofence& active_geof_msg);

  /**
   * @brief Callback to publish traffic control acknowledgement information
   *
   * @param mom_msg The acknowledgement information to publish
   */
  void publishTCMACK(const carma_v2x_msgs::msg::MobilityOperation& mom_msg);

private:

  /**
  * \brief Spin callback, which will be called frequently based on the configured spin rate
  */
  bool spin_callback();

  // Node configuration
  Config config_;

  carma_ros2_utils::PubPtr<autoware_lanelet2_msgs::msg::MapBin> map_pub_;
  carma_ros2_utils::PubPtr<autoware_lanelet2_msgs::msg::MapBin> map_update_pub_;
  carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::TrafficControlRequest> control_msg_pub_;
  carma_ros2_utils::PubPtr<visualization_msgs::msg::MarkerArray> tcm_visualizer_pub_;
  carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::TrafficControlRequestPolygon> tcr_visualizer_pub_;
  carma_ros2_utils::PubPtr<std_msgs::msg::Int32MultiArray> upcoming_intersection_ids_pub_;
  carma_ros2_utils::PubPtr<carma_perception_msgs::msg::CheckActiveGeofence> active_pub_;
  carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityOperation> tcm_ack_pub_;
  
  carma_ros2_utils::SubPtr<autoware_lanelet2_msgs::msg::MapBin> base_map_sub_;
  carma_ros2_utils::SubPtr<carma_planning_msgs::msg::Route> route_callmsg_sub_;
  carma_ros2_utils::SubPtr<std_msgs::msg::String> georef_sub_;
  carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::TrafficControlMessage> geofence_sub_;
  carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MapData> incoming_map_sub_;
  carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> curr_location_sub_;
  
  // Timer for publishing TCR and SignalGroup/IntersectionID
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> ptr_;
  std::unique_ptr<carma_wm_ctrl::WMBroadcaster> wmb_;
};
}  // namespace carma_wm_ctrl
