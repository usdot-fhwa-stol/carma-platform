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
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
#include <carma_ros2_utils/carma_ros2_utils.hpp>
#include <autoware_lanelet2_msgs/msg/map_bin.hpp>
#include <queue>
#include <carma_perception_msgs/msg/roadway_obstacle_list.hpp>
#include <carma_planning_msgs/msg/route.hpp>
#include <carma_v2x_msgs/msg/spat.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace carma_wm
{
class WMListenerWorker;  // Forward declaration of worker class

/*!
 * \brief Class which provies automated subscription and threading support for the world model
 *
 * Users should generally create and cache a single instance of this class within a node.
 * They can then retrieve a pointer to an initialized WorldModel object for doing queries.
 * By default this class follows the threading model of the host node, but it can operate in the background if specified
 * in the constructor. When used in a multi-threading case users can ensure threadsafe operation though usage of the
 * getLock function
 *
 */
class WMListener
{
public:
  /*!
   * \brief Constructor which can be used to specify threading behavior of this class
   *
   * By default this object follows node threading behavior (ie. waiting for ros::spin())
   * If the object is operating in multi-threaded mode a ros::AsyncSpinner is used to implement a background thread.
   * 
   * \param multi_thread If true this object will subscribe using background threads. Defaults to false
   */
  WMListener(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params_,
    bool multi_thread = false);

  /*! \brief Destructor
   */
  ~WMListener();

  /*!
   * \brief Returns a pointer to an intialized world model instance
   *
   * \return Const pointer to a world model object
   */
  WorldModelConstPtr getWorldModel();

  /*!
   * \brief Allows user to set a callback to be triggered when a map update is received
   *        NOTE: If operating in multi-threaded mode the world model will remain locked until the user function
   * completes.
   *
   * \param callback A callback function that will be triggered after the world model receives a new map update
   */
  void setMapCallback(std::function<void()> callback);

  /*!
   * \brief Allows user to set a callback to be triggered when a route update is received
   *        NOTE: If operating in multi-threaded mode the world model will remain locked until the user function
   * completes.
   *
   * \param callback A callback function that will be triggered after the world model is updated with a new route
   */
  void setRouteCallback(std::function<void()> callback);

  /*!
   * \brief Returns a unique_lock which can be used to lock world model updates until the user finishes a desired
   * operation. This function should be used when multiple queries are needed and this object is operating in
   * multi-threaded mode
   *
   * \param pre_locked If true the returned lock will already be locked. If false the lock will be deferred with
   * std::defer_lock Default is true
   *
   * \return std::unique_lock for thread safe access to world model data
   */
  std::unique_lock<std::mutex> getLock(bool pre_locked = true);

  /*!
   * \brief Allows user to set a callback to be triggered when a route update is received
   *        NOTE: If operating in multi-threaded mode the world model will remain locked until the user function
   * completes.
   *
   * \param config_lim A function that populate the configurable speed limit value after the world model is updated with a new route
   */
  void setConfigSpeedLimit(double config_lim) const;

  /*!
   * \brief Use to allow updates to occur even if they invalidate the current route.
   *        This is only meant to be used by components which generate the route
   */  
  void enableUpdatesWithoutRouteWL();

  /*!
   * \brief Returns true if a map update has been processed which requires rerouting.
   *        This method is meant to be used by routing components. 
   * 
   * \return True is rerouting is needed
   */ 
  bool checkIfReRoutingNeededWL();


private:
  // Callback function that uses lock to edit the map
  void mapUpdateCallback(autoware_lanelet2_msgs::msg::MapBin::UniquePtr geofence_msg);
  carma_ros2_utils::SubPtr<carma_perception_msgs::msg::RoadwayObstacleList> roadway_objects_sub_;
  carma_ros2_utils::SubPtr<autoware_lanelet2_msgs::msg::MapBin> map_update_sub_;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params_;

  std::unique_ptr<WMListenerWorker> worker_;
  
  carma_ros2_utils::SubPtr<autoware_lanelet2_msgs::msg::MapBin> map_sub_;
  carma_ros2_utils::SubPtr<carma_planning_msgs::msg::Route> route_sub_;
  carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::SPAT> traffic_spat_sub_;
  const bool multi_threaded_;
  std::mutex mw_mutex_;
  

};
}  // namespace carma_wm