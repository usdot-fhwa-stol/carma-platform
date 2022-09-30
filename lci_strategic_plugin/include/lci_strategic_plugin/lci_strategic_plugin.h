#pragma once

/*
 * Copyright (C)2022 LEIDOS.
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
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/BSM.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_utils/CARMAUtils.h>
#include <bsm_helper/bsm_helper.h>
#include <carma_wm/Geometry.h>
#include <lanelet2_core/Forward.h>
#include <gtest/gtest_prod.h>

#include <lanelet2_extension/regulatory_elements/CarmaTrafficSignal.h>
#include "lci_strategic_plugin/lci_state_transition_table.h"
#include "lci_strategic_plugin/lci_strategic_plugin_config.h"
#include "lci_strategic_plugin/lci_states.h"

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <algorithm>

namespace lci_strategic_plugin
{

  enum class TurnDirection {
            Straight,
            Right,
            Left
    };

/**
 * \brief Struct representing trajectory smoothing algorithm parameters using distance and acceleration
 *        Based on TSMO USE CASE 2. Chapter 2. Trajectory Smoothing
 */
enum TSCase {
  CASE_1 = 1,
  CASE_2 = 2,
  CASE_3 = 3,
  CASE_4 = 4,
  CASE_5 = 5,
  CASE_6 = 6,
  CASE_7 = 7,
  CASE_8 = 8,
  STOPPING=9,
  UNAVAILABLE = 10,
};

struct TrajectoryParams
{
  double t0_ = 0;
  double v0_ = 0;
  double x0_ = 0;

  double a1_ = 0;
  double t1_ = 0;
  double v1_ = 0;
  double x1_ = 0;

  double a2_ = 0;
  double t2_ = 0;
  double v2_ = 0;
  double x2_ = 0;

  double a3_ = 0;
  double t3_ = 0;
  double v3_ = 0;
  double x3_ = 0;

  bool is_algorithm_successful = true;
  TSCase case_num;
  double modified_departure_speed = -1.0;  // modified departure speed if algorithm failed
  double modified_remaining_time = -1.0;  // modified departure time if algorithm failed
};

struct BoundaryDistances
{
  double dx1 = 0.0;
  double dx2 = 0.0;
  double dx3 = 0.0;
  double dx4 = 0.0;
  double dx5 = 0.0;
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

  /**
   * \brief Lookup transfrom from front bumper to base link
   */
  void lookupFrontBumperTransform();
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief callback function for mobility operation
   * 
   * \param msg input mobility operation msg
   */
  void mobilityOperationCb(const cav_msgs::MobilityOperationConstPtr& msg);

  /**
   * \brief BSM callback function
   */
  void BSMCb(const cav_msgs::BSMConstPtr& msg);

 /**
   * \brief parse strategy parameters
   * 
   * \param strategy_params input string of strategy params from mob op msg
   */
  void parseStrategyParams(const std::string& strategy_params);

  /**
   * \brief Generates Mobility Operation messages
   *
   * \return mobility operation msg for status and intent
   */
  cav_msgs::MobilityOperation generateMobilityOperation();

  /**
   * \brief Determine the turn direction at intersection
   *
   * \param lanelets_list List of lanelets crossed around the intersection area
   *
   * \return turn direction in format of straight, left, right
   *
   */
  TurnDirection getTurnDirectionAtIntersection(std::vector<lanelet::ConstLanelet> lanelets_list);

  /**
   * \brief Method to call at fixed rate in execution loop. Will publish plugin discovery and mobility operation msgs.
   * 
   * \return True if the node should continue running. False otherwise
   */ 
  bool mobilityPubSpin();


  ros::Publisher mobility_operation_pub;
  
  
  ////////// VARIABLES ///////////

  TurnDirection intersection_turn_direction_ = TurnDirection::Straight;
  bool approaching_light_controlled_interction_ = true; // TODO set to true until carma-street is capable of sending strategy parameters for UC3
                                                        // https://github.com/usdot-fhwa-stol/carma-platform/issues/1947

  // CARMA Streets Variakes
  // timestamp for msg received from carma streets
  unsigned long long street_msg_timestamp_ = 0;
  // scheduled enter time
  unsigned long long scheduled_enter_time_ = 0;
  
  //BSM
  std::string bsm_id_ = "default_bsm_id";
  uint8_t bsm_msg_count_ = 0;
  uint16_t bsm_sec_mark_ = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /**
   * \brief Useful metrics for LCI Plugin
   * \param case_num_ Current speed profile case generated
   * \param distance_remaining_to_tf_ distance_remaining_to_traffic signal meters
   * \param earliest_entry_time_ earliest_entry_time in sec
   * \param scheduled_entry_time_ scheduled_entry_time in sec
   * 
   */
  TSCase case_num_ = TSCase::UNAVAILABLE; 
  TSCase last_case_num_ = TSCase::UNAVAILABLE; 
  double distance_remaining_to_tf_ = 0.0;
  double earliest_entry_time_ = 0.0;
  double scheduled_entry_time_ = 0.0;

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
   * \param check_late check late arrival, default true
   * \param check_early check early arrival, default true
   * \return true (bool optional) if can make it at both bounds of error time.
   *  
   * NOTE: internally uses config_.green_light_time_buffer
   * NOTE: boost::none optional if any of the light state was invalid
   */
  boost::optional<bool> canArriveAtGreenWithCertainty(const ros::Time& light_arrival_time_by_algo, const lanelet::CarmaTrafficSignalPtr& traffic_light, bool check_late, bool check_early) const;

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
  cav_msgs::Maneuver composeTrajectorySmoothingManeuverMessage(double start_dist, double end_dist, const std::vector<lanelet::ConstLanelet>& crossed_lanelets, double start_speed,
                                                       double target_speed, ros::Time start_time, ros::Time end_time,
                                                       const TrajectoryParams& tsp) const;
  
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
   * Given the initial speed, ending speed (which is 0), and remaining_downtrack, this function checks if calculated remaining_time falls on RED
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \param current_state Current state of the vehicle
   * \param traffic_light CarmaTrafficSignalPtr of the relevant signal
   * \param entry_lanelet Entry lanelet in to the intersection along route
   * \param exit_lanelet Exit lanelet in to the intersection along route
   * \param current_lanelet Current lanelet
   * \param traffic_light_down_track Downtrack to the given traffic_light
   *
   * \return void. if successful, resp is non-empty
   */
  void handleStopping(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp, 
                                        const VehicleState& current_state, 
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light,
                                        const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet,
                                        double traffic_light_down_track);

  /**
   * \brief This function returns valid maneuvers if the vehicle is able to utilize trajectory smoothing parameters to go through the intersection with certainty
   *        It utilizes canMakeWithCertainty function to determine if it is able to make it. Corresponds to TSCase 1-7
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \param current_state Current state of the vehicle
   * \param current_state_speed The current speed to use when planning
   * \param traffic_light CarmaTrafficSignalPtr of the relevant signal
   * \param entry_lanelet Entry lanelet in to the intersection along route
   * \param exit_lanelet Exit lanelet in to the intersection along route
   * \param traffic_light_down_track Downtrack to the given traffic_light
   * \param ts_params trajectory smoothing params
   *
   * \return void. if successful, resp is non-empty. Resp is empty if not able to make it with certainty or ts_params.is_successful is false
   */
  void handleGreenSignalScenario(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp, 
                                        const VehicleState& current_state, 
                                        double current_state_speed,
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light,
                                        const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, 
                                        double traffic_light_down_track, const TrajectoryParams& ts_params);
  /**
   * \brief This function returns valid maneuvers if the vehicle is able to utilize trajectory smoothing parameters to go through the intersection with certainty
   *        It utilizes canMakeWithCertainty function to determine if it is able to make it. Corresponds to TSCase 1-7
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \param current_state Current state of the vehicle
   * \param current_state_speed The current speed to use when planning
   * \param traffic_light CarmaTrafficSignalPtr of the relevant signal
   * \param traffic_light_down_track Downtrack to the given traffic_light
   * \param ts_params trajectory smoothing params
   *
   * \return void. if successful, resp is non-empty. Resp is empty if not able to make it with certainty or ts_params.is_successful is false
   */
  void handleCruisingUntilStop(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp, 
                                        const VehicleState& current_state, 
                                        double current_state_speed,
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light,
                                        double traffic_light_down_track, const TrajectoryParams& ts_params);

  /**
   * \brief When the vehicle is not able to successfully run the algorithm or not able to stop, this function modifies 
   *        departure speed as close as possible to the original using max comfortable accelerations with given distance.
   * 
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \param current_state Current state of the vehicle
   * \param current_state_speed The current speed to use when planning
   * \param speed_limit   Speed limit to cap the speed
   * \param remaining_time  Remaining time until scheduled entry
   * \param exit_lanelet_id Id of the exit lanelet
   * \param traffic_light CarmaTrafficSignalPtr of the relevant signal
   * \param traffic_light_down_track Downtrack to the given traffic_light
   * \param ts_params trajectory smoothing params
   *
   * \return void. if successful, resp is non-empty. Resp is empty if not able to make it with certainty or ts_params.is_successful is false
   */
  void handleFailureCase(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp, 
                                        const VehicleState& current_state, 
                                        double current_state_speed,
                                        double speed_limit,
                                        double remaining_time, 
                                        lanelet::Id exit_lanelet_id,
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light, 
                                        double traffic_light_down_track, const TrajectoryParams& ts_params);

  /**
   * \brief Helper function to handleFailureCase that modifies ts_params to desired new parameters. See handleFailureCase
   * \param starting_speed starting speed
   * \param departure_speed departure_speed originally planned
   * \param speed_limit speed_limit
   * \param remaining_downtrack remaining_downtrack until the intersection
   * \param remaining_time  remaining_time when vehicle is scheduled to enter
   * \param traffic_light_downtrack  traffic_light_downtrack when vehicle is scheduled to enter
   *
   * \return TSP with parameters that is best available to pass the intersection. Either cruising with starting_speed or sacrifice departure speed to meet time and distance
   */
  TrajectoryParams handleFailureCaseHelper(double starting_speed, double departure_speed, double speed_limit, double remaining_downtrack, double remaining_time, double traffic_light_downtrack);
                                        
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
   *        within half of one full fixed cycle time boundary compared to free flow arrival time.
   *
   * \param time_remaining_at_free_flow Free flow arrival at the intersection
   * \param full_cycle_duration One fixed cycle of the signal
   * \param current_speed Current speed in m/s
   * \param speed_limit Speed limit speed in m/s
   * \param departure_speed Speed into the intersection in m/s. A.k.a target speed at the intersection
   * \param max_accel The acceleration in m/s^2 of the acceleration segment
   * \param max_decel The deceleration in m/s^2 of the deceleration segment
   *
   * \return the max distance the plugin should activate to support all speed profile cases of the trajectory smoothing algorithm
   *         -1 if it is within the distance already
   */
  double get_trajectory_smoothing_activation_distance(double time_remaining_at_free_flow, double full_cycle_duration, double current_speed, double speed_limit, double departure_speed, double max_accel, double max_decel) const;

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
   * \brief Get boundary distances used to compare against current state in order to create trajectory smoothing parameters
   *
   * \param v0 starting_speed
   * \param v1 departure_speed
   * \param v_max speed_limit
   * \param v_min minimum speed
   * \param a_max max_comfort_acceleration limit
   * \param a_min max_comfort_deceleration limit
   * \return boundary distances used to generate trajectory smoothing segments
   */
  BoundaryDistances get_delta_x(double v0, double v1, double v_max, double v_min, double a_max, double a_min);
  
    /**
   * \brief Get all possible trajectory smoothing parameters for each segments. Later this will be used to generate a single trajectory
   * \param t current time in seconds
   * \param v0 starting_speed
   * \param v1 departure_speed
   * \param v_max speed_limit
   * \param v_min minimum speed
   * \param a_max max_comfort_acceleration limit
   * \param a_min max_comfort_deceleration limit
   * \param x0 current_downtrack
   * \param x_end traffic_light_down_track
   * \param dx distance_remaining_to_traffic_light
   * \param boundary_distances boundary_distances to compare against current state
   * \return all possible trajectory smoothing parameters to later generate single trajectory
   */
  std::vector<TrajectoryParams> get_boundary_traj_params(double t, double v0, double v1, double v_max, double v_min, double a_max, double a_min, double x0, double x_end, double dx, BoundaryDistances boundary_distances);
  
  /**
   * \brief Helper method to print TrajectoryParams
   */
  void print_params(TrajectoryParams params);

  /**
   * \brief Helper method to print TrajectoryParams
   */
  void print_boundary_distances(BoundaryDistances delta_xs);
  
  /**
   * \brief Trajectory Smoothing Algorithm 8 cases.  TSMO UC2, UC3 algorithm equations
   */
  TrajectoryParams ts_case1(double t, double et, double v0, double v1, double v_max, double a_max, double a_min, double x0, double x_end, double dx);
  TrajectoryParams ts_case2(double t, double et, double v0, double v1, double a_max, double a_min, double x0, double x_end, double dx);
  TrajectoryParams ts_case3(double t, double et, double v0, double v1, double a_max, double a_min, double x0, double x_end, double dx);
  TrajectoryParams ts_case4(double t, double et, double v0, double v1, double v_min, double a_max, double a_min, double x0, double x_end, double dx);
  TrajectoryParams ts_case5(double t, double et, double v0, double a_max, double a_min, double x0, double x_end, double dx);
  TrajectoryParams ts_case6(double t, double et, double v0, double v_min, double a_min, double x0, double x_end, double dx, double dx3, TrajectoryParams traj6);
  TrajectoryParams ts_case7(double t, double et, double v0, double v_min, double a_min, double x0, double x_end, double dx);
  TrajectoryParams ts_case8(double dx, double dx5, TrajectoryParams traj8);

  TrajectoryParams boundary_accel_or_decel_incomplete_upper(double t, double v0, double v1, double a_max, double a_min, double x0, double x_end, double dx);
  TrajectoryParams boundary_accel_nocruise_notmaxspeed_decel(double t, double v0, double v1, double a_max, double a_min, double x0, double x_end, double dx); 
  TrajectoryParams boundary_accel_cruise_maxspeed_decel(double t, double v0, double v1, double v_max, double a_max, double a_min, double x0, double x_end, double dx);
  TrajectoryParams boundary_accel_nocruise_maxspeed_decel(double t, double v0, double v1, double v_max, double a_max, double a_min, double x0, double x_end, double dx);
  TrajectoryParams boundary_accel_or_decel_complete_upper(double t, double v0, double v1, double x0, double x_end, double dx);
  TrajectoryParams boundary_decel_nocruise_notminspeed_accel(double t, double v0, double v1, double v_min, double a_max, double a_min, double x0, double x_end, double dx);
  TrajectoryParams boundary_decel_nocruise_minspeed_accel_incomplete(double t, double v0, double v_min, double a_max, double a_min, double x0, double x_end, double dx);
  TrajectoryParams boundary_decel_nocruise_minspeed_accel_complete(double t, double v0, double v1, double v_max, double v_min, double a_max, double a_min, double x0, double x_end, double dx);
  TrajectoryParams boundary_decel_cruise_minspeed_accel(double t, double v0, double v1, double v_min, double a_max, double a_min, double x0, double x_end, double dx);
  TrajectoryParams boundary_decel_cruise_minspeed(double t, double v0, double v_min, double a_min, double x0, double x_end, double dx);
  TrajectoryParams boundary_decel_incomplete_lower(double t, double v0, double a_min, double x0, double x_end, double dx);
  TrajectoryParams boundary_decel_cruise_minspeed_decel(double t, double v0, double v_min, double a_min, double x0, double x_end, double dx);
  
  TrajectoryParams get_ts_case(double t, double et, double v0, double v1, double v_max, double v_min, double a_max, double a_min, double x0, double x_end, double dx, BoundaryDistances boundary_distances, std::vector<TrajectoryParams> params);
  
  ////////// VARIABLES ///////////
  
  std::string previous_strategy_params_ = "";  

  double max_comfort_accel_ = 2.0;  // acceleration rates after applying miltiplier
  double max_comfort_decel_ = -2.0; 
  double max_comfort_decel_norm_ = -1 * max_comfort_decel_;
  boost::optional<ros::Time> nearest_green_entry_time_cached_;

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
  std::string light_controlled_intersection_strategy_ = "signalized"; // Strategy carma-streets is sending. Could be more verbose but needs to be changed on both ends
  
  // TF listenser
  tf2_ros::Buffer tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  tf2::Stamped<tf2::Transform> frontbumper_transform_;
  double length_to_front_bumper_ = 3.0;

  double epsilon_ = 0.001; //Small constant to compare (double) 0.0 with
  double accel_epsilon_ = 0.0001; //Small constant to compare (double) 0.0 with

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
  // Algo Unit Tests
  FRIEND_TEST(LCIStrategicTestFixture, calc_estimated_entry_time_left);
  FRIEND_TEST(LCIStrategicTestFixture, estimate_distance_to_stop);
  FRIEND_TEST(LCIStrategicTestFixture, estimate_time_to_stop);
  FRIEND_TEST(LCIStrategicTestFixture, get_distance_to_accel_or_decel_twice);
  FRIEND_TEST(LCIStrategicTestFixture, get_distance_to_accel_or_decel_once);
  FRIEND_TEST(LCIStrategicTestFixture, get_nearest_green_entry_time);
  FRIEND_TEST(LCIStrategicTestFixture, get_earliest_entry_time);
  FRIEND_TEST(LCIStrategicTestFixture, calc_estimated_entry_time_left);
  FRIEND_TEST(LCIStrategicTestFixture, handleFailureCase);
  FRIEND_TEST(LCIStrategicTestFixture, handleStopping);






};
}  // namespace lci_strategic_plugin