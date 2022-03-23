#pragma once

/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include <string>
#include <cav_srvs/PlanManeuvers.h>
#include <cav_msgs/Plugin.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_utils/CARMAUtils.h>
#include <carma_wm/Geometry.h>
#include <lanelet2_core/Forward.h>
#include <gtest/gtest_prod.h>

#include <lanelet2_extension/regulatory_elements/CarmaTrafficSignal.h>
#include "wz_strategic_plugin/wz_state_transition_table.h"
#include "wz_strategic_plugin/wz_strategic_plugin_config.h"
#include "wz_strategic_plugin/wz_states.h"

namespace wz_strategic_plugin
{
class WzStrategicPlugin
{
public:
  /**
   * \brief Constructor
   *
   * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
   * \param config The configuration to be used for this object
   */
  WzStrategicPlugin(carma_wm::WorldModelConstPtr wm, WzStrategicPluginConfig config);

  /**
   * \brief Service callback for arbitrator maneuver planning
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \return If service call successed
   */
  bool planManeuverCb(cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp);

  /**
   * \brief Returns the current plugin discovery message reflecting system status
   *
   * \return cav_msgs::Plugin The plugin discovery message
   */
  cav_msgs::Plugin getDiscoveryMsg() const;

private:
  /**
   * \brief Struct representing a vehicle state for the purposes of planning
   */
  struct VehicleState
  {
    ros::Time stamp;      // Timestamp of this state data
    double downtrack;     // The downtrack of the vehicle along the route at time stamp
    double speed;         // The speed of the vehicle at time stamp
    lanelet::Id lane_id;  // The current lane id of the vehicle at time stamp
  };

  /**
   * \brief Method for performing maneuver planning when the current plugin state is TransitState::UNAVAILABLE
   *        Therefor no maneuvers are planned but the system checks for the precense of traffic lights ahead of it
   *
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \param current_state The current state of the vehicle at the start of planning
   * \param traffic_lights A list of traffic lights along the vehicle's route which lie in front of the vehicle
   *
   */
  void planWhenUNAVAILABLE(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp,
                           const VehicleState& current_state,
                           const std::vector<lanelet::CarmaTrafficSignalPtr>& traffic_lights);

  /**
   * \brief Method for performing maneuver planning when the current plugin state is TransitState::APPROACHING
   *        Therefor the planned maneuvers deal with approaching a traffic light
   *
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \param current_state The current state of the vehicle at the start of planning
   * \param traffic_lights A list of traffic lights along the vehicle's route which lie in front of the vehicle
   *
   */
  void planWhenAPPROACHING(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp,
                           const VehicleState& current_state,
                           const std::vector<lanelet::CarmaTrafficSignalPtr>& traffic_lights);

  /**
   * \brief Method for performing maneuver planning when the current plugin state is TransitState::WAITING
   *        Therefor the planned maneuvers deal with waiting at a red light
   *
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \param current_state The current state of the vehicle at the start of planning
   * \param traffic_lights A list of traffic lights along the vehicle's route which lie in front of the vehicle
   *
   */
  void planWhenWAITING(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp,
                       const VehicleState& current_state,
                       const std::vector<lanelet::CarmaTrafficSignalPtr>& traffic_lights);

  /**
   * \brief Method for performing maneuver planning when the current plugin state is TransitState::DEPARTING
   *        Therefor the planned maneuvers deal with the transit of the intersection once the stop bar has been crossed
   *
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \param current_state The current state of the vehicle at the start of planning
   * \param intersection_end_downtrack The ending downtrack in meters of the current intersection
   * \param intersection_speed_limit The speed limit of the current intersection
   *
   */
  void planWhenDEPARTING(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp,
                         const VehicleState& current_state, double intersection_end_downtrack,
                         double intersection_speed_limit);

  /**
   * \brief Compose a lane keeping maneuver message based on input params
   *
   * \param start_dist Start downtrack distance of the current maneuver
   * \param end_dist End downtrack distance of the current maneuver
   * \param start_speed Start speed of the current maneuver
   * \param target_speed Target speed pf the current maneuver, usually it is the lanelet speed limit
   * \param start_time The starting time of the maneuver
   * \param end_time The ending time of the maneuver
   * \param lane_ids List of lanelet IDs that the current maneuver traverses. Message expects these to be contiguous and
   * end to end
   *
   * \return A lane keeping maneuver message which is ready to be published
   */
  cav_msgs::Maneuver composeLaneFollowingManeuverMessage(double start_dist, double end_dist, double start_speed,
                                                         double target_speed, ros::Time start_time, ros::Time end_time,
                                                         std::vector<lanelet::Id> lane_ids) const;

  /**
   * \brief Compose a stop and wait maneuver message based on input params
   *
   * \param current_dist Start downtrack distance of the current maneuver
   * \param end_dist End downtrack distance of the current maneuver
   * \param start_speed Start speed of the current maneuver
   * \param starting_lane_id The starting lanelet id of this maneuver
   * \param ending_lane_id The ending lanelet id of this maneuver
   * \param start_time The starting time of the maneuver
   * \param end_time The ending time of the maneuver
   * \param stopping_accel Acceleration used for calculating the stopping distance
   *
   * \return A stop and wait maneuver message which is ready to be published
   */
  cav_msgs::Maneuver composeStopAndWaitManeuverMessage(double current_dist, double end_dist, double start_speed,
                                                       const lanelet::Id& starting_lane_id,
                                                       const lanelet::Id& ending_lane_id, ros::Time start_time,
                                                       ros::Time end_time, double stopping_accel) const;

  /**
   * \brief Compose a intersection transit maneuver message based on input params
   *
   * \param start_dist Start downtrack distance of the current maneuver
   * \param end_dist End downtrack distance of the current maneuver
   * \param start_speed Start speed of the current maneuver
   * \param target_speed Target speed pf the current maneuver, usually it is the lanelet speed limit
   * \param start_time The starting time of the maneuver
   * \param end_time The ending time of the maneuver
   * \param starting_lane_id The starting lanelet id of this maneuver
   * \param ending_lane_id The ending lanelet id of this maneuver
   *
   * \return A intersection transit maneuver maneuver message which is ready to be published
   */
  cav_msgs::Maneuver composeIntersectionTransitMessage(double start_dist, double end_dist, double start_speed,
                                                       double target_speed, ros::Time start_time, ros::Time end_time,
                                                       const lanelet::Id& starting_lane_id,
                                                       const lanelet::Id& ending_lane_id) const;

  /**
   * \brief Helper method to evaluate if the given traffic light state is supported by this plugin
   *
   * \param state The state to evaluate
   *
   * \return true if the state is supported, flase otherwise
   */
  bool supportedLightState(lanelet::CarmaTrafficSignalState state) const;

  /**
   * \brief Helper method that checks both if the input optional light state is set and if the state it contains is
   * supported via supportedLightState.
   *
   * \param optional_state An optional light state and its min_end_time pair. If this is unset the method will return false
   * \param source_time The time used to optain the optional light state. This is used for logging only
   *
   * \return True if the optional is set and the contained state signal is supported. False otherwise
   */
  bool validLightState(const boost::optional<std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>>& optional_state,
                       const ros::Time& source_time) const;

  /**
   * \brief Helper method to extract the initial vehicle state from the planning request method based on if the
   * prior_plan was set or not.
   *
   * \param req The maneuver planning request to extract the vehicle state from
   *
   * \return The extracted VehicleState
   */
  VehicleState extractInitialState(const cav_srvs::PlanManeuversRequest& req) const;

  /**
   * \brief Helper method which calls carma_wm::WorldModel::getLaneletsBetween(start_downtrack, end_downtrack, shortest_path_only,
   * bounds_inclusive) and throws and exception if the returned list of lanelets is empty. See the referenced method for additional
   * details on parameters.
   */
  std::vector<lanelet::ConstLanelet> getLaneletsBetweenWithException(double start_downtrack, double end_downtrack,
                                                                     bool shortest_path_only = false,
                                                                     bool bounds_inclusive = true) const;

  /**
   * \brief Helper method to use basic kinematics to compute an estimated stopping distance from from the inputs
   *
   * \param v The initial velocity in m/s
   * \param a The deceleration in m/s^2
   *
   * \return The estimated stopping distance in meters
   */
  double estimate_distance_to_stop(double v, double a) const;

  /**
   * \brief Helper method to use basic kinematics to compute an estimated stopping time from from the inputs
   *
   * \param d The distance to travel in meters
   * \param v The initial velocity in m/s
   *
   * \return The estimated stopping time in seconds
   */
  double estimate_time_to_stop(double d, double v) const;

  /**
   * \brief Given a Lanelet, find it's associated Speed Limit
   *
   * \param llt Constant Lanelet object
   *
   * \throw std::invalid_argument if the speed limit could not be retrieved
   *
   * \return value of speed limit in mps
   */
  double findSpeedLimit(const lanelet::ConstLanelet& llt) const;

  ////////// VARIABLES ///////////

  //! World Model pointer
  carma_wm::WorldModelConstPtr wm_;

  //! Config containing configurable algorithm parameters
  WzStrategicPluginConfig config_;

  //! State Machine Transition table
  WorkZoneStateTransitionTable transition_table_;

  //! Plugin discovery message
  cav_msgs::Plugin plugin_discovery_msg_;

  //! Cache variables for storing the current intersection state between state machine transitions
  boost::optional<double> intersection_speed_;
  boost::optional<double> intersection_end_downtrack_;

  //Unit Tests
  FRIEND_TEST(WorkZoneTestFixture, getDiscoveryMsg);
  FRIEND_TEST(WorkZoneTestFixture, supportedLightState);
  FRIEND_TEST(WorkZoneTestFixture, estimate_distance_to_stop);
  FRIEND_TEST(WorkZoneTestFixture, estimate_time_to_stop);
  FRIEND_TEST(WorkZoneTestFixture, extractInitialState);
  FRIEND_TEST(WorkZoneTestFixture, validLightState);
  FRIEND_TEST(WorkZoneTestFixture, getLaneletsBetweenWithException);
  FRIEND_TEST(WorkZoneTestFixture, composeLaneFollowingManeuverMessage);
  FRIEND_TEST(WorkZoneTestFixture, composeStopAndWaitManeuverMessage);
  FRIEND_TEST(WorkZoneTestFixture, composeIntersectionTransitMessage);
  FRIEND_TEST(WorkZoneTestFixture, findSpeedLimit);

};
}  // namespace wz_strategic_plugin