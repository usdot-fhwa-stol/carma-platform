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
#include "lci_strategic_plugin/lci_state_transition_table.h"
#include "lci_strategic_plugin/lci_strategic_plugin_config.h"
#include "lci_strategic_plugin/lci_states.h"

namespace lci_strategic_plugin
{

enum SpeedProfileCase {
  ACCEL_CRUISE_DECEL = 1,
  ACCEL_DECEL = 2,
  DECEL_ACCEL = 3,
  DECEL_CRUISE_ACCEL = 4,
};

/**
 * \brief Struct representing trajectory smoothing algorithm parameters using distance and acceleration
 *        Based on TSMO USE CASE 2. Chapter 2. Trajectory Smoothing
 *        These parameter representation were selected to make it ready to be applied on speed profile.
 */
struct TrajectorySmoothingParameters
{
  double dist_accel = 0.0;      // Acceleration distance (negative if the algorithm failed to meet criteria)
  double dist_decel = 0.0;      // Deceleration distance (negative if the algorithm failed to meet criteria)
  double dist_cruise = 0.0;     // Cruise distance (negative if the algorithm failed to meet criteria)
  double a_accel = 0.0;        // Acceleration rate (negative if the algorithm failed to meet criteria)
  double a_decel = 0.0;        // Deceleration rate (positive if the algorithm failed to meet criteria)
  double speed_before_decel = -1.0; // The highest speed before deceleration starts after accelerating (applies to case 1,2)
  double speed_before_accel = -1.0; // The lowest speed before acceleration starts after decelerating (applies to case 3,4)
  SpeedProfileCase case_num;       // Trajectory Smoothing Case Number.
  bool is_algorithm_successful = true;  // True if the trajectory smoothing algorithm was able to get valid parameters
};

class LCIStrategicPlugin
{
public:
  /**
   * \brief Constructor
   *
   * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
   * \param config The configuration to be used for this object
   */
  LCIStrategicPlugin(carma_wm::WorldModelConstPtr wm, LCIStrategicPluginConfig config);

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
   * \param traffic_light The single traffic light along the vehicle's route in the intersection that is relevant and in front of the vehicle
   * \param entry_lanelet The entry lanelet into the intersection
   * \param exit_lanelet The exit lanelet into the intersection
   * \param current_lanelet The current lanelet the vehicle's state is on
   * \throws if given entry_lanelet doesn't have stop_line
   * NOTE: returns empty if the given traffic light is empty
   */
  void planWhenUNAVAILABLE(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp,
                           const VehicleState& current_state,
                           const lanelet::CarmaTrafficSignalPtr& traffic_light,
                           const lanelet::ConstLanelet& entry_lanelet,
                           const lanelet::ConstLanelet& exit_lanelet,
                           const lanelet::ConstLanelet& current_lanelet);

  /**
   * \brief Method for performing maneuver planning when the current plugin state is TransitState::APPROACHING
   *        Therefore the planned maneuvers deal with approaching a traffic light.
   *        
   *
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \param current_state The current state of the vehicle at the start of planning
   * \param traffic_light The single traffic light along the vehicle's route in the intersection that is relevant and in front of the vehicle
   * \param entry_lanelet The entry lanelet into the intersection
   * \param exit_lanelet The exit lanelet into the intersection
   * \param current_lanelet The current lanelet the vehicle's state is on
   * \throws if given entry_lanelet doesn't have stop_line or speed profile case number was failed to be calculated
   * NOTE: if there is empty traffic_light, it signals that the vehicle has crossed the bar, or if invalid light state is given
   */
  void planWhenAPPROACHING(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp,
                           const VehicleState& current_state,
                           const lanelet::CarmaTrafficSignalPtr& traffic_light,
                           const lanelet::ConstLanelet& entry_lanelet,
                           const lanelet::ConstLanelet& exit_lanelet,
                           const lanelet::ConstLanelet& current_lanelet);

  /**
   * \brief Method for performing maneuver planning when the current plugin state is TransitState::WAITING
   *        Therefor the planned maneuvers deal with waiting at a red light
   *
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \param current_state The current state of the vehicle at the start of planning
   * \param traffic_light The single traffic light along the vehicle's route in the intersection that is relevant and in front of the vehicle
   * \param entry_lanelet The entry lanelet into the intersection
   * \param exit_lanelet The exit lanelet into the intersection
   * \param current_lanelet The current lanelet the vehicle's state is on
   * NOTE: returns empty if the given traffic light is empty
   */
  void planWhenWAITING(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp,
                       const VehicleState& current_state,
                       const lanelet::CarmaTrafficSignalPtr& traffic_light,
                       const lanelet::ConstLanelet& entry_lanelet,
                       const lanelet::ConstLanelet& exit_lanelet,
                       const lanelet::ConstLanelet& current_lanelet);

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
   * \brief Return true if the car can arrive at given arrival time within green light time buffer
   * \param light_arrival_time_by_algo ROS time at green where vehicle arrives
   * \param traffic_light Traffic signal to check time against
   * \return true (bool optional) if can make it at both bounds of error time. 
   * NOTE: internally uses config_.green_light_time_buffer
   * NOTE: boost::none optional if any of the light state was invalid
   */
  boost::optional<bool> canArriveAtGreenWithCertainty(const ros::Time& light_arrival_time_by_algo, const lanelet::CarmaTrafficSignalPtr& traffic_light) const;

  /**
   * \brief Compose a trajectory smoothing maneuver msg (sent as transit maneuver message)
   *
   * \param start_dist Start downtrack distance of the current maneuver
   * \param end_dist End downtrack distance of the current maneuver
   * \param start_speed Start speed of the current maneuver
   * \param target_speed Target speed pf the current maneuver, usually it is the lanelet speed limit
   * \param start_time The starting time of the maneuver
   * \param end_time The ending time of the maneuver
   * \param tsp Trajectory Smoothing parameters needed to modify speed profile
   *
   * \return A transift maneuver message specifically designed for light controlled intersection tactical plugin
   */
  cav_msgs::Maneuver composeTrajectorySmoothingManeuverMessage(double start_dist, double end_dist, double start_speed,
                                                       double target_speed, ros::Time start_time, ros::Time end_time,
                                                       const TrajectorySmoothingParameters& tsp) const;
  
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
   * \brief This function returns stopping maneuver if the vehicle is able to stop at red and in safe stopping distance.
   * 1. Try to stop with max_decel and see if it aligns with red/yellow. If it does, go ahead and stop
   * 2. If stopping with single deceleration falls on green phase, stop at next possible red phase by using 2 different decel_rate (max_decel then custom_decel)
   *
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \param current_state Current state of the vehicle
   * \param traffic_light CarmaTrafficSignalPtr of the relevant signal
   * \param entry_lanelet Entry lanelet in to the intersection along route
   * \param exit_lanelet Exit lanelet in to the intersection along route
   * \param current_lanelet Current lanelet
   * \param nearest_green_entry_time The time when the vehicle was scheduled to enter intersection. 
   *                                 The function utilizes nearest red signal within full_cylce beyond this green entry time 
   * \param traffic_light_down_track Downtrack to the given traffic_light
   *
   * \return void. if successful, resp is non-empty
   */
  void handleStopping(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp, 
                                        const VehicleState& current_state, 
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light,
                                        const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet,
                                        const ros::Time& nearest_green_entry_time,
                                        double traffic_light_down_track);
                                        
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
   * \brief Provides the scheduled entry time for the vehicle in the future. This scheduled time is the earliest possible entry time that 
   *        is during green phase and after timestamps both that is kinematically possible and required times for vehicle in front to pass through first 
   *
   * \param current_time Current state's ROS time
   * \param earliest_entry_time Earliest ROS entry time into the intersection kinematically possible 
   * \param signal CARMATrafficSignal that is responsible for the intersection along route
   * \param minimum_required_green_time Minimum seconds required by other vehicles in front to pass through the intersection first
   *
   * \return Nearest ROS entry time during green phase.
   * NOTE: TSMO UC2: Algorithm 2. Entering time estimation algorithm for EVs in cooperation Class A when the SPaT plan is fixed-time (defined for C-ADS-equipped vehicles)
   */

  ros::Time get_nearest_green_entry_time(const ros::Time& current_time, const ros::Time& earliest_entry_time, lanelet::CarmaTrafficSignalPtr signal, double minimum_required_green_time = 0.0) const;

  /**
   * \brief Gets the earliest entry time into the intersection that is kinematically possible for the vehicle.
   *        Designed to get it for motion that has maximum one acceleration, deceleration, or cruise segments until destination.
   *
   * \param remaining_distance Distance to the intersection in meters
   * \param free_flow_speed Free flow speed along the route until the intersection (usually speed_limit) in m/s
   * \param current_speed Current speed in m/s
   * \param departure_speed Speed into the intersection in m/s. A.k.a target speed at the intersection
   * \param max_accel The acceleration in m/s^2 of the acceleration segment
   * \param max_decel The deceleration in m/s^2 of the deceleration segment
   *
   * \return The estimated stopping distance in meters
   * NOTE: TSMO UC2: Algorithm 1. Earliest entering time estimation algorithm for EVs in all cooperation classes (defined for C-ADS-equipped vehicles)
   */

  ros::Duration get_earliest_entry_time(double remaining_distance, double free_flow_speed, double current_speed, double departure_speed, double max_accel, double max_decel) const;

  /**
   * \brief Gets maximum distance (nearest downtrack) the trajectory smoothing algorithm makes difference than simple lane following
   *        within one fixed cycle time boundary compared to free flow arrival time.
   *
   * \param remaining_time One fixed cycle of the signal before the free flow arrival at the intersection 
   * \param current_speed Current speed in m/s
   * \param speed_limit Speed limit speed in m/s
   * \param departure_speed Speed into the intersection in m/s. A.k.a target speed at the intersection
   * \param max_accel The acceleration in m/s^2 of the acceleration segment
   * \param max_decel The deceleration in m/s^2 of the deceleration segment
   *
   * \return the max distance the plugin should activate to support all speed profile cases of the trajectory smoothing algorithm
   */
  double get_trajectory_smoothing_activation_distance(double remaining_time, double current_speed, double speed_limit, double departure_speed, double max_accel, double max_decel) const;

  /**
   * \brief Get required distance to accel then decel, or vise versa - each at least once - to reach departure_speed with given speed and acceleration parameters 
   *
   * \param free_flow_speed Free flow speed in m/s (usually the speed limit)
   * \param current_speed Current speed in m/s
   * \param departure_speed Speed to enter into the intersection in m/s
   * \param max_accel Acceleration rate during acceleration segment in m/s^2
   * \param max_decel Deceleration rate during acceleration segment in m/s^2
   *
   * \return estimated distance to accel/decel or vice versa exactly twice in meters
   */
  
  double get_distance_to_accel_or_decel_twice (double free_flow_speed, double current_speed, double departure_speed, double max_accel, double max_decel) const;

  /**
   * \brief Get the speed between the acceleration and deceleration pieces. 
   * \param x  Distance remaining in meters
   * \param x1  Required distance to accel then decel, or vise versa-each at least once-to reach departure_speed with given speed and acceleration parameters 
   * \param x2  required distance to accel or decel to reach departure_speed with given speed and acceleration parameters 
   * \param free_flow_speed Free flow speed in m/s (usually the speed limit)
   * \param current_speed Current speed in m/s
   * \param departure_speed Speed to enter into the intersection in m/s
   * \param max_accel Acceleration rate during acceleration segment in m/s^2
   * \param max_decel Deceleration rate during acceleration segment in m/s^2
   * NOTE: note that this returns departure speed if x < x2 since vehicle's trajectory corresponding to its EET 
   *        would contain only one acceleration or deceleration piece
   * \return The estimated stopping distance in meters
   */
  
  double get_inflection_speed_value (double x, double x1, double x2, double free_flow_speed, double current_speed, double departure_speed, double max_accel, double max_decel) const;

  /**
   * \brief Get required distance to accel or decel to reach departure_speed with given speed and acceleration parameters 
   *
   * \param current_speed Current speed in m/s
   * \param departure_speed Speed to enter into the intersection in m/s
   * \param max_accel Acceleration rate during acceleration segment in m/s^2
   * \param max_decel Deceleration rate during acceleration segment in m/s^2
   *
   * \return estimated distance to accel or decel in meters
   */
  
  double get_distance_to_accel_or_decel_once (double current_speed, double departure_speed, double max_accel, double max_decel) const;
  
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

  /**
   * \brief calculate the speed, right before the car starts to decelerate for timed entry into the intersection
    *
   * \param entry_time time the vehicle must stop
   *
   * \param entry_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   * 
   * \param departure_speed speed to get into the intersection
   *
   * \return speed value
   */
  double calc_speed_before_decel(double entry_time, double entry_dist, double current_speed, double departure_speed) const;

  /**
   * \brief calculate the speed, right before the car starts to accelerate for timed entry into the intersection
    *
   * \param entry_time time the vehicle must stop
   *
   * \param entry_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   * 
   * \param departure_speed speed to get into the intersection
   *
   * \return speed value
   */
  double calc_speed_before_accel(double entry_time, double entry_dist, double current_speed, double departure_speed) const;

  /**
   * \brief Determine the speed profile case for approaching an intersection. 
   *        It internally utilizes config_.min_speed and speed_limit_ to determine upper and lower speed bounds
   * 
   * \param estimated_entry_time estimated time to enter the intersection without speed modification
   * 
   * \param scheduled_entry_time scheduled time to enter the intersection
   *
   * \param speed_before_decel speed before starting to decelerate (applicable in case 1, 2)
   * 
   * \param  speed_before_accel speed before starting to accelerate (applicable in case 1, 2)
   *
   * \return integer case number
   */
  SpeedProfileCase determine_speed_profile_case(double estimated_entry_time, double scheduled_entry_time, double speed_before_decel, double speed_before_accel, double speed_limit);

  /**
   * \brief calculate the time vehicle will enter to intersection with optimal deceleration
   *
   * \param entry_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   * 
   * \param departure_speed speed to get into the intersection
   *
   * \return the time vehicle will stop with optimal decelarion
   */
  double calc_estimated_entry_time_left(double entry_dist, double current_speed, double departure_speed) const;

  /**
   * \brief Get parameters for a speed profile according to case one or two of the light controlled intersection, where the vehicle accelerates (then cruises if needed) and decelerates into the intersection. 
   * \param remaining_downtrack distance for the maneuver to be planned (excluding buffer points) in m
   * \param remaining_time remaining time in seconds until scheduled entry into the intersection 
   * \param starting_speed starting speed at the start of the maneuver in m/s
   * \param speed_before_decel highest speed desired between acceleration and deceleration  m/s
   * \param speed_limit speed limit in m/s
   * \param departure_speed intersection speed in m/s
   * \return TrajectorySmoothingParameters that has readily usable parameters to apply speed profile into the trajectory points
   * NOTE: Cruising speed profile is applied (case 1) if speed before deceleration is higher than speed limit. Otherwise Case 2.
   */
  TrajectorySmoothingParameters get_parameters_for_accel_cruise_decel_speed_profile(double remaining_downtrack, double remaining_time, double starting_speed, double speed_before_decel, double speed_limit, double departure_speed);

  /**
   * \brief Creates a speed profile according to case three or four of the light controlled intersection, where the vehicle decelerates (then cruises if needed) and accelerates into the intersection. 
   * \param remaining_downtrack distance for the maneuver to be planned (excluding buffer points) in m
   * \param remaining_time remaining time in seconds until scheduled entry into the intersection 
   * \param starting_speed starting speed at the start of the maneuver in m/s
   * \param speed_before_accel lowest speed desired between deceleration and acceleration in m/s
   * \param speed_limit speed limit in m/s
   * \param departure_speed intersection speed in m/s
   * \return TrajectoySmoothingParameters that has readily usable parameters to apply speed profile into the trajectory points
   * NOTE: Cruising speed profile is applied (case 4) if speed before acceleration is lower than minimum speed allowed. Otherwise Case 3.
   */
  TrajectorySmoothingParameters get_parameters_for_decel_cruise_accel_speed_profile(double remaining_downtrack, double remaining_time, double starting_speed, double speed_before_accel, double minimum_speed, double departure_speed);
  
  ////////// VARIABLES ///////////

  double max_comfort_accel_ = 2.0;  // acceleration rates after applying miltiplier
  double max_comfort_decel_ = -2.0; 
  double max_comfort_decel_norm_ = -1 * max_comfort_decel_;

  //! World Model pointer
  carma_wm::WorldModelConstPtr wm_;

  //! Config containing configurable algorithm parameters
  LCIStrategicPluginConfig config_;

  //! State Machine Transition table
  LCIStrategicStateTransitionTable transition_table_;

  //! Plugin discovery message
  cav_msgs::Plugin plugin_discovery_msg_;

  //! Cache variables for storing the current intersection state between state machine transitions
  boost::optional<double> intersection_speed_;
  boost::optional<double> intersection_end_downtrack_;
  std::string light_controlled_intersection_strategy_ = "Carma/light_controlled_intersection";

  double epsilon_ = 0.001; //Small constant to compare (double) 0.0 with

  //Unit Tests
  FRIEND_TEST(LCIStrategicTestFixture, getDiscoveryMsg);
  FRIEND_TEST(LCIStrategicTestFixture, supportedLightState);
  FRIEND_TEST(LCIStrategicTestFixture, estimate_distance_to_stop);
  FRIEND_TEST(LCIStrategicTestFixture, estimate_time_to_stop);
  FRIEND_TEST(LCIStrategicTestFixture, extractInitialState);
  FRIEND_TEST(LCIStrategicTestFixture, validLightState);
  FRIEND_TEST(LCIStrategicTestFixture, getLaneletsBetweenWithException);
  FRIEND_TEST(LCIStrategicTestFixture, composeStopAndWaitManeuverMessage);
  FRIEND_TEST(LCIStrategicTestFixture, composeIntersectionTransitMessage);
  FRIEND_TEST(LCIStrategicTestFixture, composeTrajectorySmoothingManeuverMessage);
  FRIEND_TEST(LCIStrategicTestFixture, findSpeedLimit);

};
}  // namespace lci_strategic_plugin