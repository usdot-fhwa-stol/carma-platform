#pragma once

/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include <autoware_lanelet2_msgs/msg/map_bin.hpp>
#include <carma_planning_msgs/msg/route.hpp>
#include <carma_v2x_msgs/msg/spat.hpp>
#include <carma_wm/CARMAWorldModel.hpp>
#include <carma_wm/TrafficControl.hpp>
#include <queue>
#include <carma_wm/SignalizedIntersectionManager.hpp>
#include <utility>
#include <rosgraph_msgs/msg/clock.hpp>

namespace carma_wm
{
/*!
 * \brief Backend logic class for WMListener
 *
 */
class WMListenerWorker
{
public:
  /*!
   * \brief Constructor
   */
  WMListenerWorker();

  /*!
   * \brief Constructor
   */
  WorldModelConstPtr getWorldModel() const;

  /*!
   * \brief Callback for new map messages. Updates the underlying map
   *
   * \param map_msg The new map messages to generate the map from
   */
  void mapCallback(const autoware_lanelet2_msgs::msg::MapBin::SharedPtr map_msg);

  /*!
   * \brief Callback for new map update messages (geofence). Updates the underlying map
   *
   * \param geofence_msg The new map update messages to generate the map edits from
   */
  void mapUpdateCallback(autoware_lanelet2_msgs::msg::MapBin::SharedPtr geofence_msg);

  /*!
   * \brief Callback for route message.
   */
  void routeCallback(const carma_planning_msgs::msg::Route::SharedPtr route_msg);

  /*!
   * \brief Callback for ROS1 clock message (used in Simulation runs)
   */
  void ros1ClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr clock_msg);

  /*!
   * \brief Callback for Simulation clock message (used in Simulation runs)
   */
  void simClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr clock_msg);

  /*!
   * \brief Callback for roadway objects msg
   */
  void roadwayObjectListCallback(const carma_perception_msgs::msg::RoadwayObstacleList::SharedPtr msg);

  /*!
   * \brief Allows user to set a callback to be triggered when a map update is received
   *
   * \param callback A callback function that will be triggered after the world model receives a new map update
   */
  void setMapCallback(std::function<void()> callback);

  /*!
   * \brief Allows user to set a callback to be triggered when a route update is received
   *
   * \param callback A callback function that will be triggered after the world model is updated with a new route
   */
  void setRouteCallback(std::function<void()> callback);

 /*!
   * \brief Allows user to set a callback to be triggered when a map update is received
   *
   * \param config_lim A callback function that will be triggered after the world model receives a new map update
   */
  void setConfigSpeedLimit(double config_lim);

  /**
   *  \brief Returns the current configured speed limit value
   *
   */
  double getConfigSpeedLimit() const;


  /*!
   * \brief Allows user to set a callback to be triggered when a map update is received
   *
   * \param participant A callback function that will be triggered after the world model receives a new map update
   */
  void setVehicleParticipationType(std::string participant);

  /**
   * @brief Returns the Vehicle Participation Type value
   *
   */
  std::string getVehicleParticipationType() const;


  /**
   *  \brief Check if re-routing is needed and returns re-routing flag
   *
   */
  bool checkIfReRoutingNeeded() const;
  /**
   *  \brief Enable updates without route and set route_node_flag_ as true
   *
   */
  void enableUpdatesWithoutRoute();

  /**
   * \brief Helper function to convert a routing graph message into a actual RoutingGraph object
   *
   * \param msg The graph message to convert
   * \param map The base map this graph applies to
   *
   * \return nullptr if the graph could not be constructed or the provided graph does not match the map
   */
  LaneletRoutingGraphPtr routingGraphFromMsg(const autoware_lanelet2_msgs::msg::RoutingGraph& msg, lanelet::LaneletMapPtr map) const;

  /**
   *  \brief incoming spat message
   */
  void incomingSpatCallback(const carma_v2x_msgs::msg::SPAT::SharedPtr spat_msg);

  /**
   *  \brief set true if simulation_mode is on
   */
  void isUsingSimTime(bool use_sim_time) const;

  /**
   *  \brief set true if incoming spat is based on wall clock
   */
  void isSpatWallTime(bool use_real_time_spat_in_sim) const;

  /**
   *  \brief Activate World Model SPAT processor, which is turned off by default,
   * with OFF (0), ON (1), FIXED (2)
   */
  void setWMSpatProcessingState(const SIGNAL_PHASE_PROCESSING& phase_type) const;

  /**
   *  \brief Get World Model SPAT processor state from signalized intersection manager
   *  \return World Model SPAT process state OFF (0), ON(1)
   */
  SIGNAL_PHASE_PROCESSING getWMSpatProcessingState() const;

private:
  std::shared_ptr<CARMAWorldModel> world_model_;
  std::function<void()> map_callback_;
  std::function<void()> route_callback_;
  void newRegemUpdateHelper(lanelet::Lanelet parent_llt, lanelet::RegulatoryElement* regem) const;
  double config_speed_limit_;

  size_t current_map_version_ = 0; // Current map version based on recived map messages
  std::queue<autoware_lanelet2_msgs::msg::MapBin::SharedPtr> map_update_queue_; // Update queue used to cache map updates when they cannot be immeadiatly applied due to waiting for rerouting
  boost::optional<carma_planning_msgs::msg::Route> delayed_route_msg_;

  bool recompute_route_flag_=false; // indicates whether if this node should recompute its route based on invalidated msg
  bool rerouting_flag_=false; //indicates whether if route node is in middle of rerouting
  bool route_node_flag_=false; //indicates whether if this node is route node
  long most_recent_update_msg_seq_ = -1; // Tracks the current sequence number for map update messages. Dropping even a single message would invalidate the map

};
}  // namespace carma_wm
