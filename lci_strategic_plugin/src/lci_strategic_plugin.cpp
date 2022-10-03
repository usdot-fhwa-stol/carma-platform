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
#include "lci_strategic_plugin/lci_strategic_plugin.h"
#include "lci_strategic_plugin/lci_states.h"

#define GET_MANEUVER_PROPERTY(mvr, property)                                                                           \
  (((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ?                                                 \
        (mvr).intersection_transit_left_turn_maneuver.property :                                                       \
        ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ?                                           \
             (mvr).intersection_transit_right_turn_maneuver.property :                                                 \
             ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ?                                        \
                  (mvr).intersection_transit_straight_maneuver.property :                                              \
                  ((mvr).type == cav_msgs::Maneuver::LANE_CHANGE    ? (mvr).lane_change_maneuver.property :            \
                   (mvr).type == cav_msgs::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :         \
                                                                      throw new std::invalid_argument("GET_MANEUVER_"  \
                                                                                                      "PROPERTY "      \
                                                                                                      "(property) "    \
                                                                                                      "called on "     \
                                                                                                      "maneuver with " \
                                                                                                      "invalid type "  \
                                                                                                      "id"))))))
   
namespace lci_strategic_plugin
{
LCIStrategicPlugin::LCIStrategicPlugin(carma_wm::WorldModelConstPtr wm, LCIStrategicPluginConfig config)
  : wm_(wm), config_(config)
{
  plugin_discovery_msg_.name = config_.strategic_plugin_name;
  plugin_discovery_msg_.version_id = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = true;
  plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
  plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";

  max_comfort_accel_ = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  max_comfort_decel_ = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
  max_comfort_decel_norm_ = config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
};

cav_msgs::Plugin LCIStrategicPlugin::getDiscoveryMsg() const
{
  return plugin_discovery_msg_;
}

bool LCIStrategicPlugin::supportedLightState(lanelet::CarmaTrafficSignalState state) const
{
  switch (state)
  {
    // NOTE: Following cases are intentional fall through.
    // Supported light states
    case lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN:              // Solid Red
    case lanelet::CarmaTrafficSignalState::PROTECTED_CLEARANCE:          // Yellow Solid no chance of conflicting traffic
    case lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED:   // Solid Green no chance of conflict traffic
                                                                        // (normally used with arrows)
      return true;

    // Unsupported light states
    case lanelet::CarmaTrafficSignalState::PERMISSIVE_MOVEMENT_ALLOWED:  // Solid Green there could be conflict traffic
    case lanelet::CarmaTrafficSignalState::PERMISSIVE_CLEARANCE:         // Yellow Solid there is a chance of conflicting
                                                                        // traffic
    case lanelet::CarmaTrafficSignalState::UNAVAILABLE:                  // No data available
    case lanelet::CarmaTrafficSignalState::DARK:                         // Light is non-functional
    case lanelet::CarmaTrafficSignalState::STOP_THEN_PROCEED:            // Flashing Red

    case lanelet::CarmaTrafficSignalState::PRE_MOVEMENT:                 // Yellow Red transition (Found only in the EU)
                                                                        // (normally used with arrows)
    case lanelet::CarmaTrafficSignalState::CAUTION_CONFLICTING_TRAFFIC:  // Yellow Flashing
    default:
      return false;
  }
}

void LCIStrategicPlugin::lookupFrontBumperTransform() 
{
    tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));
    tf2_buffer_.setUsingDedicatedThread(true);
    try
    {
        geometry_msgs::TransformStamped tf = tf2_buffer_.lookupTransform("base_link", "vehicle_front", ros::Time(0), ros::Duration(20.0)); //save to local copy of transform 20 sec timeout
        length_to_front_bumper_ = tf.transform.translation.x;
        ROS_DEBUG_STREAM("length_to_front_bumper_: " << length_to_front_bumper_);
        
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
}

LCIStrategicPlugin::VehicleState LCIStrategicPlugin::extractInitialState(const cav_srvs::PlanManeuversRequest& req) const
{
  VehicleState state;
  if (!req.prior_plan.maneuvers.empty())
  {
    ROS_DEBUG_STREAM("Provided with initial plan...");
    state.stamp = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_time);
    state.downtrack = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_dist);
    state.speed = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_speed);
    state.lane_id = getLaneletsBetweenWithException(state.downtrack, state.downtrack, true).front().id();
  }
  else
  {
    ROS_DEBUG_STREAM("No initial plan provided...");
    
    state.stamp = req.header.stamp;
    state.downtrack = req.veh_downtrack;
    state.speed = req.veh_logitudinal_velocity;
    state.lane_id = stoi(req.veh_lane_id);
  }
  ROS_DEBUG_STREAM("extractInitialState >>>> state.stamp: " << state.stamp);
  ROS_DEBUG_STREAM("extractInitialState >>>> state.downtrack : " << state.downtrack );
  ROS_DEBUG_STREAM("extractInitialState >>>> state.speed: " << state.speed);
  ROS_DEBUG_STREAM("extractInitialState >>>> state.lane_id: " << state.lane_id);

  return state;
}

double LCIStrategicPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt) const
{
  lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
  if (traffic_rules)
  {
    return (*traffic_rules)->speedLimit(llt).speedLimit.value();
  }
  else
  {
    throw std::invalid_argument("Valid traffic rules object could not be built");
  }
}

bool LCIStrategicPlugin::validLightState(const boost::optional<std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>>& optional_state,
                                        const ros::Time& source_time) const
{
  if (!optional_state)
  {
    ROS_WARN_STREAM("Traffic light data not available for source_time " << std::to_string(source_time.toSec()));
    return false;
  }

  lanelet::CarmaTrafficSignalState light_state = optional_state.get().second;

  if (!supportedLightState(light_state))
  {
    ROS_ERROR_STREAM("LCIStrategic Plugin asked to handle CarmaTrafficSignalState: " << light_state
                                                                                << " which is not supported.");
    return false;
  }

  return true;
}

boost::optional<bool> LCIStrategicPlugin::canArriveAtGreenWithCertainty(const ros::Time& light_arrival_time_by_algo, const lanelet::CarmaTrafficSignalPtr& traffic_light, bool check_late = true, bool check_early = true) const
{
    ros::Time early_arrival_time_by_algo =
        light_arrival_time_by_algo - ros::Duration(config_.green_light_time_buffer);

    ros::Time late_arrival_time_by_algo =
        light_arrival_time_by_algo + ros::Duration(config_.green_light_time_buffer);

    ROS_DEBUG_STREAM("light_arrival_time_by_algo: " << std::to_string(light_arrival_time_by_algo.toSec()));
    ROS_DEBUG_STREAM("early_arrival_time_by_algo: " << std::to_string(early_arrival_time_by_algo.toSec()));
    ROS_DEBUG_STREAM("late_arrival_time_by_algo: " << std::to_string(late_arrival_time_by_algo.toSec()));

    auto early_arrival_state_by_algo_optional = traffic_light->predictState(lanelet::time::timeFromSec(early_arrival_time_by_algo.toSec()));

    if (!validLightState(early_arrival_state_by_algo_optional, early_arrival_time_by_algo))
      return boost::none;

    ROS_DEBUG_STREAM("early_arrival_state_by_algo: " << early_arrival_state_by_algo_optional.get().second);

    auto late_arrival_state_by_algo_optional = traffic_light->predictState(lanelet::time::timeFromSec(late_arrival_time_by_algo.toSec()));

    if (!validLightState(late_arrival_state_by_algo_optional, late_arrival_time_by_algo))
      return boost::none; 

    ROS_DEBUG_STREAM("late_arrival_state_by_algo: " << late_arrival_state_by_algo_optional.get().second);

    bool can_make_early_arrival = true;
    bool can_make_late_arrival = true;

    if (check_early)
      can_make_early_arrival = (early_arrival_state_by_algo_optional.get().second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED);
    
    if (check_late)
      can_make_late_arrival = (late_arrival_state_by_algo_optional.get().second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED);

    // We will cross the light on the green phase even if we arrive early or late
    if (can_make_early_arrival && can_make_late_arrival)  // Green light
      return true;
    else
      return false;

}

std::vector<lanelet::ConstLanelet> LCIStrategicPlugin::getLaneletsBetweenWithException(double start_downtrack,
                                                                                      double end_downtrack,
                                                                                      bool shortest_path_only,
                                                                                      bool bounds_inclusive) const
{
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
      wm_->getLaneletsBetween(start_downtrack, end_downtrack, shortest_path_only, bounds_inclusive);

  if (crossed_lanelets.size() == 0)
  {
    throw std::invalid_argument("getLaneletsBetweenWithException called but inputs do not cross any lanelets going "
                                "from: " +
                                std::to_string(start_downtrack) + " to: " + std::to_string(end_downtrack));
  }

  return crossed_lanelets;
}

void LCIStrategicPlugin::handleStopping(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp, 
                                        const VehicleState& current_state, 
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light,
                                        const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet,
                                        double traffic_light_down_track)
{
  double distance_remaining_to_traffic_light = traffic_light_down_track - current_state.downtrack;

  // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
        getLaneletsBetweenWithException(current_state.downtrack, traffic_light_down_track, true, true);

  double decel_rate =  max_comfort_decel_norm_; // Kinematic |(v_f - v_i) / t = a|
  ROS_DEBUG_STREAM("HANDLE_STOPPING: Planning stop and wait maneuver at decel_rate: " << decel_rate);
  
  case_num_ = TSCase::STOPPING;
  
  resp.new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(
    current_state.downtrack, traffic_light_down_track, current_state.speed, crossed_lanelets.front().id(),
    crossed_lanelets.back().id(), current_state.stamp,
    current_state.stamp + ros::Duration(config_.min_maneuver_planning_period), decel_rate));
}


void LCIStrategicPlugin::handleFailureCase(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp, 
                                        const VehicleState& current_state, 
                                        double current_state_speed,
                                        double speed_limit,
                                        double remaining_time, 
                                        lanelet::Id exit_lanelet_id,
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light, 
                                        double traffic_light_down_track, const TrajectoryParams& ts_params)
{
  double distance_remaining_to_traffic_light = traffic_light_down_track - current_state.downtrack;

  // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
        getLaneletsBetweenWithException(current_state.downtrack, traffic_light_down_track, true, true);

  auto incomplete_traj_params = handleFailureCaseHelper(current_state_speed, intersection_speed_.get(), speed_limit, distance_remaining_to_traffic_light, remaining_time, traffic_light_down_track);

  resp.new_plan.maneuvers.push_back(composeTrajectorySmoothingManeuverMessage(current_state.downtrack, traffic_light_down_track, crossed_lanelets,
                                          current_state_speed, incomplete_traj_params.modified_departure_speed, current_state.stamp, current_state.stamp + ros::Duration(incomplete_traj_params.modified_remaining_time), incomplete_traj_params));

  double intersection_length = intersection_end_downtrack_.get() - traffic_light_down_track;

  ros::Time intersection_exit_time =
      current_state.stamp + ros::Duration(incomplete_traj_params.modified_remaining_time) + ros::Duration(intersection_length / incomplete_traj_params.modified_departure_speed);

  resp.new_plan.maneuvers.push_back(composeIntersectionTransitMessage(
      traffic_light_down_track, intersection_end_downtrack_.get(), intersection_speed_.get(),
      incomplete_traj_params.modified_departure_speed, current_state.stamp + ros::Duration(incomplete_traj_params.modified_remaining_time), intersection_exit_time, crossed_lanelets.back().id(), exit_lanelet_id));

  case_num_ = TSCase::UNAVAILABLE;
}

void LCIStrategicPlugin::handleCruisingUntilStop(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp, 
                                        const VehicleState& current_state, 
                                        double current_state_speed,
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light, 
                                        double traffic_light_down_track, const TrajectoryParams& ts_params)
{
  if (!ts_params.is_algorithm_successful || ts_params.case_num != TSCase::CASE_8)
  {
    ROS_WARN_STREAM("handleCruisingUntilStop is called but it is not case_8");
    return;
  }

  auto new_ts_params = ts_params; 

  double decel_rate = std::fabs(ts_params.a3_);

  ROS_DEBUG_STREAM("CASE_8: Planning cruise and stop with decel_rate: " << decel_rate);
  
  new_ts_params.t3_ = new_ts_params.t2_;
  new_ts_params.x3_ = new_ts_params.x2_;
  new_ts_params.v3_ = new_ts_params.v2_;
  new_ts_params.a3_ = new_ts_params.a2_;

  // Identify the lanelets which will be crossed by approach maneuvers stopping part
  std::vector<lanelet::ConstLanelet> lane_follow_crossed_lanelets =
      getLaneletsBetweenWithException(new_ts_params.x1_, new_ts_params.x2_, true, true);

  resp.new_plan.maneuvers.push_back(composeTrajectorySmoothingManeuverMessage(current_state.downtrack, traffic_light_down_track, lane_follow_crossed_lanelets, 
                                          current_state_speed, new_ts_params.v2_, current_state.stamp, ros::Time(new_ts_params.t2_), new_ts_params));

  // Identify the lanelets which will be crossed by approach maneuvers stopping part
  std::vector<lanelet::ConstLanelet> case_8_crossed_lanelets =
      getLaneletsBetweenWithException(new_ts_params.x2_, traffic_light_down_track, true, true);

  resp.new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(
    new_ts_params.x2_, traffic_light_down_track, new_ts_params.v2_, case_8_crossed_lanelets.front().id(),
    case_8_crossed_lanelets.back().id(), ros::Time(new_ts_params.t2_),
    current_state.stamp + ros::Duration(config_.min_maneuver_planning_period), decel_rate));

  case_num_ = TSCase::CASE_8;

  return;
}

void LCIStrategicPlugin::handleGreenSignalScenario(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp, 
                                        const VehicleState& current_state, 
                                        double current_state_speed,
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light,
                                        const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet,
                                        double traffic_light_down_track, const TrajectoryParams& ts_params)
{
  if (!ts_params.is_algorithm_successful || ts_params.case_num == TSCase::CASE_8) 
  {
    return;
  }

  ros::Time light_arrival_time_by_algo = ros::Time(ts_params.t3_);
  double remaining_time = light_arrival_time_by_algo.toSec() - req.header.stamp.toSec();
  ROS_DEBUG_STREAM("Algo initially successful: New light_arrival_time_by_algo: " << std::to_string(light_arrival_time_by_algo.toSec()) << ", with remaining_time: " << std::to_string(remaining_time));
  auto can_make_green_optional = canArriveAtGreenWithCertainty(light_arrival_time_by_algo, traffic_light);

    // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
        getLaneletsBetweenWithException(current_state.downtrack, traffic_light_down_track, true, true);

  // no change for maneuver if invalid light states
  if (!can_make_green_optional) 
    return;
  
  if (can_make_green_optional.get())
  {
    ROS_DEBUG_STREAM("HANDLE_SUCCESSFULL: Algorithm successful, and able to make it at green with certainty. Planning traj smooth and intersection transit maneuvers");
    
    resp.new_plan.maneuvers.push_back(composeTrajectorySmoothingManeuverMessage(current_state.downtrack, traffic_light_down_track, crossed_lanelets,
                                          current_state_speed, ts_params.v3_, current_state.stamp, light_arrival_time_by_algo, ts_params));

    double intersection_length = intersection_end_downtrack_.get() - traffic_light_down_track;

    ros::Time intersection_exit_time =
        light_arrival_time_by_algo + ros::Duration(intersection_length / intersection_speed_.get());

    resp.new_plan.maneuvers.push_back(composeIntersectionTransitMessage(
        traffic_light_down_track, intersection_end_downtrack_.get(), intersection_speed_.get(),
        intersection_speed_.get(), light_arrival_time_by_algo, intersection_exit_time, entry_lanelet.id(), exit_lanelet.id()));
  }
}


TrajectoryParams LCIStrategicPlugin::handleFailureCaseHelper(double starting_speed, double departure_speed,  double speed_limit, double remaining_downtrack, double remaining_time, double traffic_light_downtrack)
{
  //Requested maneuver needs to be modified to meet remaining_dist req
  //by trying to get close to the target_speed and remaining_time as much as possible
  TrajectoryParams params;

  params.is_algorithm_successful = false;
  params.case_num = CASE_1;

  ROS_DEBUG_STREAM("HANDLE_LAST_RESORT_CASE: Starting...");
  double modified_remaining_time;

  if (starting_speed <= departure_speed)
    modified_remaining_time = (sqrt(pow(starting_speed, 2) + (2 * max_comfort_accel_ * remaining_downtrack)) - starting_speed)/ max_comfort_accel_;
  else
    modified_remaining_time = (sqrt(pow(starting_speed, 2) + (2 * max_comfort_decel_ * remaining_downtrack)) - starting_speed) / max_comfort_decel_;

  if (starting_speed <= departure_speed)
  {
    params.a1_ = max_comfort_accel_;
    params.v1_ = sqrt(pow(starting_speed, 2) + (2 * max_comfort_accel_ * remaining_downtrack));
  }
  else
  {
    params.a1_ = max_comfort_decel_;
    params.v1_ = sqrt(pow(starting_speed, 2) + (2 * max_comfort_decel_ * remaining_downtrack));
  }

  params.x1_ = traffic_light_downtrack;

  params.a2_ = 0;
  params.v2_ = params.v1_;
  params.x2_ = params.x1_;

  params.a3_ = 0;
  params.v3_ = params.v1_;
  params.x3_ = params.x1_;

  params.modified_departure_speed = params.v1_;
  params.modified_remaining_time = modified_remaining_time;

  // handle hard failure case such as nan
  if (!isnan(params.modified_departure_speed) && params.modified_departure_speed > epsilon_ &&
      params.modified_departure_speed <  speed_limit ) //80_mph
  {
    ROS_DEBUG_STREAM("Updated the speed, and using modified_departure_speed: " << params.modified_departure_speed);
    print_params(params);
    return params;
  }

  params.a1_ = 0;
  params.v1_ = starting_speed;
  params.x1_ = traffic_light_downtrack;

  params.a2_ = 0;
  params.v2_ = params.v1_;
  params.x2_ = params.x1_;

  params.a3_ = 0;
  params.v3_ = params.v1_;
  params.x3_ = params.x1_;

  params.modified_departure_speed = params.v1_;
  params.modified_remaining_time = remaining_downtrack / starting_speed;

  ROS_DEBUG_STREAM("Cruising!!! at: " << params.modified_departure_speed <<", for sec: " << params.modified_remaining_time << ", where remaining_time: " << remaining_time);
  
  // handle hard failure case such as nan
  if (isnan(params.modified_departure_speed) || params.modified_departure_speed < - epsilon_ ||
      params.modified_departure_speed > 35.7632 ) //80_mph
  {
    throw std::invalid_argument("Calculated departure speed is invalid: " + std::to_string(params.modified_departure_speed));
  }
  print_params(params);
  
  return params;
}


void LCIStrategicPlugin::planWhenUNAVAILABLE(const cav_srvs::PlanManeuversRequest& req,
                                            cav_srvs::PlanManeuversResponse& resp, const VehicleState& current_state,
                                            const lanelet::CarmaTrafficSignalPtr& traffic_light, const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet)
{
  // Reset intersection state since in this state we are not yet known to be in or approaching an intersection
  intersection_speed_ = boost::none;
  intersection_end_downtrack_ = boost::none;
  double current_state_speed = std::max(current_state.speed, config_.algo_minimum_speed * 1.001);


  if (!traffic_light)
  {
    ROS_DEBUG("No lights found along route. Returning maneuver plan unchanged");
    return;
  }

  double max_comfort_accel = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  double max_comfort_decel = -1.0 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;

  auto stop_line = traffic_light->getStopLine(entry_lanelet);

  if (!stop_line)
  {
    throw std::invalid_argument("Given entry lanelet doesn't have stop_line...");
  }

  double traffic_light_down_track =
      wm_->routeTrackPos(stop_line.get().front().basicPoint2d()).downtrack;

  ROS_DEBUG("traffic_light_down_track %f", traffic_light_down_track);

  double distance_remaining_to_traffic_light = traffic_light_down_track - current_state.downtrack;

  ROS_DEBUG_STREAM("distance_remaining_to_traffic_light: " << distance_remaining_to_traffic_light <<
                    ", current_state.downtrack: " << current_state.downtrack);

  distance_remaining_to_tf_ = distance_remaining_to_traffic_light; // performance metric

  auto speed_limit = findSpeedLimit(current_lanelet);

  current_state_speed = std::min(speed_limit, current_state_speed);

  ROS_DEBUG_STREAM("speed_limit (free flow speed): " << speed_limit);

  ROS_DEBUG_STREAM("trajectory_smoothing_activation_distance: " << config_.trajectory_smoothing_activation_distance);

  double stopping_dist = estimate_distance_to_stop(current_state_speed, config_.vehicle_decel_limit_multiplier  *
                                                                            config_.vehicle_decel_limit); //accepts positive decel

  ROS_DEBUG_STREAM("Stopping distance: " << stopping_dist);

  double plugin_activation_distance = std::max(stopping_dist, config_.min_approach_distance);

  plugin_activation_distance = std::max(plugin_activation_distance, config_.trajectory_smoothing_activation_distance);

  ROS_DEBUG_STREAM("plugin_activation_distance: " << plugin_activation_distance);

  if (distance_remaining_to_traffic_light <= plugin_activation_distance)
  {
    ROS_INFO_STREAM("Within intersection range");
    transition_table_.signal(TransitEvent::IN_STOPPING_RANGE);  // Evaluate Signal and Run Trajectory Smoothing Algorithm
  }
  else
  {
    case_num_ = TSCase::UNAVAILABLE;
    ROS_DEBUG_STREAM("Not within intersection range");
  }

}

void LCIStrategicPlugin::planWhenAPPROACHING(const cav_srvs::PlanManeuversRequest& req,
                                            cav_srvs::PlanManeuversResponse& resp, const VehicleState& current_state,
                                            const lanelet::CarmaTrafficSignalPtr& traffic_light, const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet)
{

  if (!traffic_light)  // If we are in the approaching state and there is no longer any lights ahead of us then
                       // the vehicle must have crossed the stop bar
  {
    transition_table_.signal(TransitEvent::CROSSED_STOP_BAR);
    return;
  }

  double current_state_speed = std::max(current_state.speed, config_.algo_minimum_speed * 1.001);

  auto stop_line = traffic_light->getStopLine(entry_lanelet);

  if (!stop_line)
  {
    throw std::invalid_argument("Given entry lanelet doesn't have stop_line...");
  }

  double traffic_light_down_track =
      wm_->routeTrackPos(stop_line.get().front().basicPoint2d()).downtrack;

  ROS_DEBUG("traffic_light_down_track %f", traffic_light_down_track);

  double distance_remaining_to_traffic_light = traffic_light_down_track - current_state.downtrack;

  distance_remaining_to_tf_ = distance_remaining_to_traffic_light;

  if (distance_remaining_to_traffic_light < 0) // there is small discrepancy between signal's routeTrackPos as well as current
  {                                            // downtrack calculated using veh_x,y from state. Therefore, it may have crossed it 
    ROS_DEBUG("Crossed the bar distance_remaining_to_traffic_light: %f", distance_remaining_to_traffic_light);
    transition_table_.signal(TransitEvent::CROSSED_STOP_BAR);
    return;
  }

  // At this point we know the vehicle is within the activation distance and we know the current and next light phases
  // All we need to now determine is if we should stop or if we should continue
  lanelet::ConstLanelet intersection_lanelet;

  auto route = wm_->getRoute()->shortestPath();
  auto entry_llt_iter = std::find(route.begin(), route.end(), entry_lanelet);
  if (entry_llt_iter != route.end())
  {
    intersection_lanelet = *(entry_llt_iter + 1);
  }
  
  if (intersection_lanelet.id() != lanelet::InvalId)
  {
    intersection_speed_ = findSpeedLimit(intersection_lanelet); 
    intersection_turn_direction_ = getTurnDirectionAtIntersection({intersection_lanelet});
  }
  else
  {
    intersection_speed_ = findSpeedLimit(exit_lanelet); 
  }

  intersection_speed_ = intersection_speed_.get() * 0.999; // so that speed_limit is not exactly same as departure or current speed

  double speed_limit = findSpeedLimit(current_lanelet);

  current_state_speed = std::min(current_state_speed, speed_limit - 5 * epsilon_); // so that current_speed is not exactly same as departure or speed limit

  ROS_DEBUG_STREAM("current_state_speed: " << current_state_speed);
  ROS_DEBUG_STREAM("intersection_speed_: " << intersection_speed_.get());

  intersection_end_downtrack_ =
      wm_->routeTrackPos(exit_lanelet.centerline2d().front().basicPoint2d()).downtrack;

  ROS_DEBUG_STREAM("intersection_end_downtrack_: " << intersection_end_downtrack_.get());

  // If the vehicle is at a stop trigger the stopped state
  constexpr double HALF_MPH_IN_MPS = 0.22352;
  if (current_state.speed < HALF_MPH_IN_MPS &&
      fabs(distance_remaining_to_traffic_light) < config_.stopping_location_buffer)
  {
    transition_table_.signal(TransitEvent::STOPPED);  // The vehicle has come to a stop at the light
    ROS_DEBUG_STREAM("CARMA has detected that the vehicle stopped at the stop bar. Transitioning to WAITING STATE");

    return;
  }

  // Start of TSMO UC2 Algorithm

  ros::Time earliest_entry_time = current_state.stamp + get_earliest_entry_time(distance_remaining_to_traffic_light, speed_limit, 
                                                  current_state_speed, intersection_speed_.get(), max_comfort_accel_, max_comfort_decel_);

  ROS_DEBUG_STREAM("earliest_entry_time: " << std::to_string(earliest_entry_time.toSec()) << ", with : " << earliest_entry_time - current_state.stamp  << " left at: " << std::to_string(current_state.stamp.toSec()));
  ros::Time nearest_green_entry_time;
  bool is_entry_time_within_future_events = false;

  // Following TODO comments are tracked in this issue: https://github.com/usdot-fhwa-stol/carma-platform/issues/1947
  
  if (config_.enable_carma_streets_connection ==false /*|| scheduled_enter_time_ == 0 */) // TODO uncomment when carma-street is capable of sending strategy params
  {
    nearest_green_entry_time = get_nearest_green_entry_time(current_state.stamp, earliest_entry_time, traffic_light) 
                                          + ros::Duration(0.01); //0.01sec more buffer since green_light algorithm's timestamp picks the previous signal - Vehicle Estimation
    is_entry_time_within_future_events = true; //UC2
  }
  else if(config_.enable_carma_streets_connection ==true /*&& scheduled_enter_time_ != 0 */) // TODO uncomment when carma-street is capable of sending strategy params
  {
    //nearest_green_entry_time = ros::Time(std::max(earliest_entry_time.toSec(), (scheduled_enter_time_)/1000.0)) + ros::Duration(0.01); //Carma Street
    
    // below is temporary logic to test UC3 incrementally. Testing if UC3 will act like UC2
    // check if there is any GREEN state in the light since scheduled_enter_time_ is not sent yet
    nearest_green_entry_time = ros::Time::now() + ros::Duration(60*60.0) + ros::Duration(0.01); //Carma Street TESTING TODO revert, should come from street in the future
    for (auto pair : traffic_light->recorded_time_stamps)
    {
      if (pair.second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED && 
        lanelet::time::timeFromSec(ros::Time::now().toSec()) < pair.first ) //if not outdated
      {
        nearest_green_entry_time = get_nearest_green_entry_time(current_state.stamp, earliest_entry_time, traffic_light) 
                                          + ros::Duration(0.01);
        ROS_DEBUG_STREAM("Up to date GREEN signal exists");
        is_entry_time_within_future_events = true;
        break;
      }
    }
  }
  
  ROS_DEBUG_STREAM("nearest_green_entry_time: " << std::to_string(nearest_green_entry_time.toSec()) << ", with : " << nearest_green_entry_time - current_state.stamp  << " seconds left at: " << std::to_string(current_state.stamp.toSec()));
  
  if (!nearest_green_entry_time_cached_ && is_entry_time_within_future_events) 
  {
    ROS_DEBUG_STREAM("Applying green_light_buffer for the first time and caching! nearest_green_entry_time (without buffer):" << std::to_string(nearest_green_entry_time.toSec()) << ", and earliest_entry_time: " << std::to_string(earliest_entry_time.toSec()));
    // save first calculated nearest_green_entry_time + buffer to compare against in the future as nearest_green_entry_time changes with earliest_entry_time
    
    // check if it needs buffer below:
    ros::Time early_arrival_time_green_et =
        nearest_green_entry_time - ros::Duration(config_.green_light_time_buffer);

    ROS_DEBUG_STREAM("early_arrival_time_green_et: " << std::to_string(early_arrival_time_green_et.toSec()));

    auto early_arrival_state_green_et_optional = traffic_light->predictState(lanelet::time::timeFromSec(early_arrival_time_green_et.toSec()));

    if (!validLightState(early_arrival_state_green_et_optional, early_arrival_time_green_et))
    {
      ROS_ERROR_STREAM("Unable to resolve give signal...");
      return;
    }

    ROS_DEBUG_STREAM("early_arrival_state_green_et: " << early_arrival_state_green_et_optional.get().second);

    bool can_make_early_arrival  = (early_arrival_state_green_et_optional.get().second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED);
   
    // nearest_green_entry_time is by definition on green, so only check early_arrival
    if (can_make_early_arrival)  // Green light with Certainty
    {
      nearest_green_entry_time_cached_ = nearest_green_entry_time;  //don't apply buffer if ET is in green
    }  
    else //buffer is needed
    {
      // below logic stores correct buffered timestamp into nearest_green_entry_time_cached_ to be used later
      
      ros::Time nearest_green_signal_start_time;
      if (traffic_light->fixed_cycle_duration.total_milliseconds()/1000.0 > 1.0) // UC2
      {
        ROS_DEBUG_STREAM("UC2 Handling");
        auto normal_arrival_state_green_et_optional = traffic_light->predictState(lanelet::time::timeFromSec(nearest_green_entry_time.toSec()));

        if (!validLightState(normal_arrival_state_green_et_optional, nearest_green_entry_time))
        {
          ROS_ERROR_STREAM("Unable to resolve give signal...");
          return;
        }

        ROS_DEBUG_STREAM("normal_arrival_signal_end_time: " << std::to_string(lanelet::time::toSec(normal_arrival_state_green_et_optional.get().first)));
        
        // nearest_green_signal_start_time = normal_arrival_signal_end_time (green guaranteed) - green_signal_duration
        nearest_green_signal_start_time = ros::Time(lanelet::time::toSec(normal_arrival_state_green_et_optional.get().first - traffic_light->signal_durations[lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED]));
      }
      else  // UC3
      {
        ROS_DEBUG_STREAM("UC3 Handling");
        
        for (size_t i = 0; i < traffic_light->recorded_start_time_stamps.size(); i++)
        {
          if (traffic_light->recorded_time_stamps[i].second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED && 
            lanelet::time::timeFromSec(ros::Time::now().toSec()) < traffic_light->recorded_time_stamps[i].first ) //if not outdated
          {
            nearest_green_signal_start_time = ros::Time(lanelet::time::toSec(traffic_light->recorded_start_time_stamps[i])); // by using this, we are assuming the ET given by streets falls into this green signal time
            break;
          }
        }
      }

      if (early_arrival_time_green_et.toSec() - nearest_green_signal_start_time.toSec() < config_.green_light_time_buffer)
      {
        nearest_green_entry_time_cached_ = nearest_green_signal_start_time + ros::Duration(config_.green_light_time_buffer);
      }
      else
      {
        nearest_green_entry_time_cached_ = nearest_green_entry_time + ros::Duration(config_.green_light_time_buffer);
        // NOTE: around ending time of green signal duration, this buffer addition might push ET out of green phase.
        // However, that is okay because those timestamps that fit this criteria is treated non-green by later late/early arrival checks
      }
    }

    nearest_green_entry_time = nearest_green_entry_time_cached_.get();
  }
  else if (nearest_green_entry_time_cached_) 
  { // always pick later of buffered green entry time, or earliest entry time
    nearest_green_entry_time = ros::Time(std::max(nearest_green_entry_time.toSec(), nearest_green_entry_time_cached_.get().toSec()));
  }
  
  if (nearest_green_entry_time > nearest_green_entry_time_cached_.get())
  {
    ROS_DEBUG_STREAM("Earliest entry time has gone past the cached green light. nearest_green_entry_time_cached_:" << std::to_string(nearest_green_entry_time_cached_.get().toSec()) << ", and earliest_entry_time: " << std::to_string(earliest_entry_time.toSec()));
  }

  ROS_DEBUG_STREAM("Final nearest_green_entry_time: " << std::to_string(nearest_green_entry_time.toSec()));

  double remaining_time = nearest_green_entry_time.toSec() - current_state.stamp.toSec();
  double remaining_time_earliest_entry = earliest_entry_time.toSec() - current_state.stamp.toSec();
  scheduled_entry_time_ = remaining_time; // performance metric
  earliest_entry_time_ = remaining_time_earliest_entry; // performance metric
  
  // CASE SELECTION START
  auto boundary_distances = get_delta_x(current_state_speed, intersection_speed_.get(), speed_limit, config_.algo_minimum_speed, max_comfort_accel_, max_comfort_decel_);
  print_boundary_distances(boundary_distances); //debug

  auto boundary_traj_params = get_boundary_traj_params(req.header.stamp.toSec(), current_state_speed, intersection_speed_.get(), speed_limit, config_.algo_minimum_speed, max_comfort_accel_, max_comfort_decel_, current_state.downtrack, traffic_light_down_track, distance_remaining_to_traffic_light, boundary_distances);

  TrajectoryParams ts_params = get_ts_case(req.header.stamp.toSec(), nearest_green_entry_time.toSec(), current_state_speed, intersection_speed_.get(), speed_limit, config_.algo_minimum_speed, max_comfort_accel_, max_comfort_decel_, current_state.downtrack, traffic_light_down_track, distance_remaining_to_traffic_light, boundary_distances, boundary_traj_params);
  print_params(ts_params);

  ROS_DEBUG_STREAM("SPEED PROFILE CASE:" << ts_params.case_num);

  // CASE SELECTION END

  // Although algorithm determines nearest_green_time is possible, check if the vehicle can arrive with certainty (Case 1-7)
  if (ts_params.is_algorithm_successful && ts_params.case_num != TSCase::CASE_8) 
  {
    handleGreenSignalScenario(req, resp, current_state, current_state_speed, traffic_light, entry_lanelet, exit_lanelet, traffic_light_down_track, ts_params);
    
    if (!resp.new_plan.maneuvers.empty()) // able to pass at green
    {
      last_case_num_ = ts_params.case_num;
      case_num_ = ts_params.case_num; //to print for debugging
      return;
    }
    
    ROS_DEBUG_STREAM("Not able to make it with certainty: TSCase: " << ts_params.case_num << ", changing it to 8");
    ts_params = boundary_traj_params[7];
    ts_params.is_algorithm_successful = true; //false correspond to cases when vehicle is beyond safe_distance to stop for case8
    ts_params.case_num = CASE_8;
    print_params(ts_params);
  }
  
  ROS_DEBUG_STREAM("Not able to make it with certainty: NEW TSCase: " << ts_params.case_num);
  // if algorithm is NOT successful or if the vehicle cannot make the green light with certainty

  double safe_distance_to_stop = pow(current_state.speed, 2)/(2 * max_comfort_decel_norm_);
  ROS_DEBUG_STREAM("safe_distance_to_stop at max_comfort_decel:  " << safe_distance_to_stop << ", max_comfort_decel_norm_: " << max_comfort_decel_norm_);

  double desired_distance_to_stop = pow(current_state.speed, 2)/(2 * max_comfort_decel_norm_ * config_.deceleration_fraction) + config_.desired_distance_to_stop_buffer;
  ROS_DEBUG_STREAM("desired_distance_to_stop at: " << desired_distance_to_stop << ", where effective deceleration rate is: " << max_comfort_decel_norm_ * config_.deceleration_fraction);
  
  desired_distance_to_stop = std::max(desired_distance_to_stop, config_.stopping_location_buffer);

  ROS_DEBUG_STREAM("new desired_distance_to_stop: " << desired_distance_to_stop);

  ROS_DEBUG_STREAM("distance_remaining_to_traffic_light:  " << distance_remaining_to_traffic_light << ", current_state.speed: " << current_state.speed);

  if (desired_distance_to_stop < distance_remaining_to_traffic_light && last_case_num_ != TSCase::STOPPING) // do not switch from STOPPING to case8 again
  {
    ROS_DEBUG_STREAM("Way too early to stop. Cruising at CASE8");
    handleCruisingUntilStop(req, resp, current_state, current_state_speed, traffic_light, traffic_light_down_track, ts_params);
    
    if (!resp.new_plan.maneuvers.empty())
    {
      last_case_num_ = ts_params.case_num; //case 8
      return;
    }
  }

  if ((safe_distance_to_stop <= distance_remaining_to_traffic_light && desired_distance_to_stop >= distance_remaining_to_traffic_light) || 
    last_case_num_ == TSCase::STOPPING) // if stopping continue stopping until transition to planwhenWAITING
  {
    ROS_DEBUG_STREAM("Planning stopping now. last case:" << static_cast<int>(last_case_num_));
    handleStopping(req,resp, current_state, traffic_light, entry_lanelet, exit_lanelet, current_lanelet, traffic_light_down_track); //case_9

    if (!resp.new_plan.maneuvers.empty()) // able to stop
    {
      last_case_num_ = TSCase::STOPPING;
      return;
    }
  }

  if (safe_distance_to_stop > distance_remaining_to_traffic_light)
  {
    ROS_DEBUG_STREAM("No longer within safe distance to stop! Decide to continue stopping or continue into intersection");
    
    if (last_case_num_ != TSCase::STOPPING && last_case_num_ != TSCase::UNAVAILABLE && last_case_num_ != TSCase::CASE_8) //case 1-7
    {
      // 3. if not able to stop nor reach target speed at green, attempt its best to reach the target parameters at the intersection
      ROS_DEBUG_STREAM("HANDLE_LAST_RESORT: The vehicle is not able to stop at red/yellow light nor is able to reach target speed at green. Attempting its best to pass through at green!");
      
      handleFailureCase(req, resp, current_state, current_state_speed, speed_limit, remaining_time, 
                                    exit_lanelet.id(), traffic_light, traffic_light_down_track, ts_params);
    }
    else
    {
      ROS_DEBUG_STREAM("HANDLE_SAFETY: Planning to keep stopping now. last case:" << static_cast<int>(last_case_num_));
      handleStopping(req,resp, current_state, traffic_light, entry_lanelet, exit_lanelet, current_lanelet, traffic_light_down_track); //case_9
    }
  }

 
}

void LCIStrategicPlugin::planWhenWAITING(const cav_srvs::PlanManeuversRequest& req,
                                        cav_srvs::PlanManeuversResponse& resp, const VehicleState& current_state,
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light, const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet)
{
  case_num_ = TSCase::STOPPING;

  if (!traffic_light)
  {
    throw std::invalid_argument("While in WAITING state, the traffic lights disappeared.");
  }

  auto stop_line = traffic_light->getStopLine(entry_lanelet);

  if (!stop_line)
  {
    throw std::invalid_argument("Given entry lanelet doesn't have stop_line...");
  }

  double traffic_light_down_track =
      wm_->routeTrackPos(stop_line.get().front().basicPoint2d()).downtrack;

  ROS_DEBUG("traffic_light_down_track %f", traffic_light_down_track);
  
  auto current_light_state_optional = traffic_light->predictState(lanelet::time::timeFromSec(current_state.stamp.toSec()));
  ROS_DEBUG_STREAM("WAITING STATE: requested time to check: " << std::to_string(req.header.stamp.toSec()));
  ROS_DEBUG_STREAM("WAITING STATE: requested time to CURRENT STATE check: " << std::to_string(current_state.stamp.toSec()));
  
  if (!validLightState(current_light_state_optional, current_state.stamp))
    return;

  auto bool_optional_late_certainty = canArriveAtGreenWithCertainty(current_state.stamp, traffic_light, true, false);
  
  if (!bool_optional_late_certainty)
  {
    ROS_ERROR_STREAM("Unable to resolve green light...");
    return;
  }

  if (current_light_state_optional.get().second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED &&
        bool_optional_late_certainty.get()) // if can make with certainty
  {
    transition_table_.signal(TransitEvent::RED_TO_GREEN_LIGHT);  // If the light is green send the light transition
                                                                 // signal
    return;
  }

  // A fixed buffer to add to stopping maneuvers once the vehicle is already stopped to ensure that they can have their
  // trajectory planned
  constexpr double stop_maneuver_buffer = 10.0;

  // If the light is not green then continue waiting by creating a stop and wait maneuver on top of the vehicle
  double stopping_accel = config_.vehicle_decel_limit_multiplier * config_.vehicle_decel_limit;

  resp.new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(
      current_state.downtrack - stop_maneuver_buffer, traffic_light_down_track, current_state.speed,
      current_state.lane_id, current_state.lane_id, current_state.stamp,
      current_state.stamp + ros::Duration(config_.min_maneuver_planning_period), stopping_accel));
}

void LCIStrategicPlugin::planWhenDEPARTING(const cav_srvs::PlanManeuversRequest& req,
                                          cav_srvs::PlanManeuversResponse& resp, const VehicleState& current_state,
                                          double intersection_end_downtrack, double intersection_speed_limit)
{
  case_num_ = TSCase::UNAVAILABLE;
  
  if (current_state.downtrack > intersection_end_downtrack)
  {
    transition_table_.signal(TransitEvent::INTERSECTION_EXIT);  // If we are past the most recent
    return;
  }

  // Calculate exit time assuming constant acceleration
  ros::Time intersection_exit_time =
      current_state.stamp +
      ros::Duration(2.0 * (intersection_end_downtrack - current_state.downtrack) / (current_state.speed + intersection_speed_limit));

  // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
      getLaneletsBetweenWithException(current_state.downtrack, intersection_end_downtrack, true, true);

  // Compose intersection transit maneuver
  resp.new_plan.maneuvers.push_back(composeIntersectionTransitMessage(
      current_state.downtrack, intersection_end_downtrack, current_state.speed, intersection_speed_limit,
      current_state.stamp, intersection_exit_time, crossed_lanelets.front().id(), crossed_lanelets.back().id()));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LCIStrategicPlugin::mobilityOperationCb(const cav_msgs::MobilityOperationConstPtr& msg)
{
  if (msg->strategy == light_controlled_intersection_strategy_)
  {
    ROS_DEBUG_STREAM("Received Schedule message with id: " << msg->m_header.plan_id);
    approaching_light_controlled_interction_ = true;
    ROS_DEBUG_STREAM("Approaching light Controlled Intersection: " << approaching_light_controlled_interction_);

    if (msg->m_header.recipient_id == config_.vehicle_id)
      {
        street_msg_timestamp_ = msg->m_header.timestamp;
        ROS_DEBUG_STREAM("street_msg_timestamp_: " << street_msg_timestamp_);
        parseStrategyParams(msg->strategy_params);
        previous_strategy_params_ = msg->strategy_params;
      }
    
  }
  
}

void LCIStrategicPlugin::BSMCb(const cav_msgs::BSMConstPtr& msg)
{
  std::vector<uint8_t> bsm_id_vec = msg->core_data.id;
  bsm_id_ = BSMHelper::BSMHelper::bsmIDtoString(bsm_id_vec);
  bsm_msg_count_ = msg->core_data.msg_count;
  bsm_sec_mark_ = msg->core_data.sec_mark;
}

void LCIStrategicPlugin::parseStrategyParams(const std::string& strategy_params)
{
  // sample strategy_params: "et:1634067059"
  std::string params = strategy_params;
  std::vector<std::string> inputsParams;
  boost::algorithm::split(inputsParams, params, boost::is_any_of(","));

  std::vector<std::string> et_parsed;
  boost::algorithm::split(et_parsed, inputsParams[0], boost::is_any_of(":"));
  scheduled_enter_time_ = std::stoull(et_parsed[1]);
  ROS_DEBUG_STREAM("scheduled_enter_time_: " << scheduled_enter_time_);

}

cav_msgs::MobilityOperation LCIStrategicPlugin::generateMobilityOperation()
{
    cav_msgs::MobilityOperation mo_;
    mo_.m_header.timestamp = ros::Time::now().toNSec()/1000000;
    mo_.m_header.sender_id = config_.vehicle_id;
    mo_.m_header.sender_bsm_id = bsm_id_;
    mo_.strategy = light_controlled_intersection_strategy_;

    double vehicle_acceleration_limit_ = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
    double vehicle_deceleration_limit_ = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;

    std::string intersection_turn_direction = "straight";
    if (intersection_turn_direction_ == TurnDirection::Right) intersection_turn_direction = "right";
    if (intersection_turn_direction_ == TurnDirection::Left) intersection_turn_direction = "left";

    mo_.strategy_params = "access: 1, max_accel: " + std::to_string(vehicle_acceleration_limit_) +  // NOTE: Access currently set to 1 at all times since its not specified by streets
                        ", max_decel: " + std::to_string(vehicle_deceleration_limit_) + ", react_time: " + std::to_string(config_.reaction_time) +
                        ", min_gap: " + std::to_string(config_.min_gap) + ", depart_pos: 0" + // NOTE: Departure position set to 0 at all times since it's not specified by streets
                        ", turn_direction: " + intersection_turn_direction + ", msg_count: " + std::to_string(bsm_msg_count_) + ", sec_mark: " + std::to_string(bsm_sec_mark_);
    

    return mo_;
}

TurnDirection LCIStrategicPlugin::getTurnDirectionAtIntersection(std::vector<lanelet::ConstLanelet> lanelets_list)
{
  TurnDirection turn_direction = TurnDirection::Straight;
  for (auto l:lanelets_list)
  {
    if(l.hasAttribute("turn_direction")) {
      std::string direction_attribute = l.attribute("turn_direction").value();
      if (direction_attribute == "right")
      {
        turn_direction = TurnDirection::Right;
        break;
      }
      else if (direction_attribute == "left")
      {
        turn_direction = TurnDirection::Left;
        break;
      }
      else turn_direction = TurnDirection::Straight;
    }
    else
    {
      // if there is no attribute, assumption is straight
      turn_direction = TurnDirection::Straight;
    }

  }
  return turn_direction;
}


bool LCIStrategicPlugin::mobilityPubSpin()
{
  if (approaching_light_controlled_interction_)
  {
    cav_msgs::MobilityOperation status_msg = generateMobilityOperation();
    mobility_operation_pub.publish(status_msg);
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool LCIStrategicPlugin::planManeuverCb(cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp)
{
  ROS_DEBUG("<<<<<<<<<<<<<<<<< STARTING LCI_STRATEGIC_PLAN!!!!!!!!! >>>>>>>>>>>>>>>>");

  if (!wm_->getRoute())
  {
    ROS_ERROR_STREAM("Could not plan maneuvers as route was not available");
    return true;
  }

  if(config_.enable_carma_streets_connection ==true)
  {
  if (!approaching_light_controlled_interction_)
  {
    resp.new_plan.maneuvers = {};
    ROS_WARN_STREAM("Not approaching light-controlled intersection so no maneuvers");
    return true;
  }

  bool is_empty_schedule_msg  = false ;// = (scheduled_enter_time_ == 0); TODO https://github.com/usdot-fhwa-stol/carma-platform/issues/1947
  if (is_empty_schedule_msg)
  {
    resp.new_plan.maneuvers = {};
    ROS_WARN_STREAM("Receiving empty schedule message");
    return true;
  }
  }

  ROS_DEBUG("Finding car information");

  // Extract vehicle data from request
  VehicleState current_state = extractInitialState(req);
  if (transition_table_.getState() != TransitState::UNAVAILABLE && !req.prior_plan.maneuvers.empty())
  {
    ROS_WARN_STREAM("State is NOT UNAVAILABLE AND Maneuvers in request is NOT empty");
    return true;
  }
  // Get current traffic light information
  ROS_DEBUG("\n\nFinding traffic_light information");

  auto traffic_list = wm_->getSignalsAlongRoute({ req.veh_x, req.veh_y });

  ROS_DEBUG_STREAM("Found traffic lights of size: " << traffic_list.size());

  auto current_lanelets = wm_->getLaneletsFromPoint({ req.veh_x, req.veh_y});
  lanelet::ConstLanelet current_lanelet;
  
  if (current_lanelets.empty())
  {
    ROS_ERROR_STREAM("Given vehicle position is not on the road! Returning...");
    return true;
  }

  // get the lanelet that is on the route in case overlapping ones found
  for (auto llt : current_lanelets)
  {
    auto route = wm_->getRoute()->shortestPath();
    if (std::find(route.begin(), route.end(), llt) != route.end())
    {
      current_lanelet = llt;
      break;
    }
  }

  lanelet::CarmaTrafficSignalPtr nearest_traffic_signal = nullptr;
  
  lanelet::ConstLanelet entry_lanelet;
  lanelet::ConstLanelet exit_lanelet;

  for (auto signal : traffic_list)
  {
    auto optional_entry_exit = wm_->getEntryExitOfSignalAlongRoute(signal);
    // if signal is not matching our destination skip
    if (!optional_entry_exit)
    {
      ROS_ERROR_STREAM("Did not find entry.exit along the route");
      continue;
    }
      
    entry_lanelet = optional_entry_exit.get().first;
    exit_lanelet = optional_entry_exit.get().second;

    nearest_traffic_signal = signal;
    break;
  }


  TransitState prev_state;

  do
  {
    // Clear previous maneuvers planned by this plugin as guard against state change since each state generates an
    // independent set of maneuvers
    resp.new_plan = cav_msgs::ManeuverPlan();

    prev_state = transition_table_.getState();  // Cache previous state to check if state has changed after 1 iteration

    ROS_INFO_STREAM("Planning in state: " << transition_table_.getState());

    boost::optional<std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>> current_light_state_optional = boost::none;
    if (nearest_traffic_signal)
    {
      current_light_state_optional = nearest_traffic_signal->predictState(lanelet::time::timeFromSec(current_state.stamp.toSec()));
      ROS_DEBUG_STREAM("Real-time current signal: " << current_light_state_optional.get().second << ", for Id: " << nearest_traffic_signal->id());
    }
    switch (transition_table_.getState())
    {
      case TransitState::UNAVAILABLE:
        planWhenUNAVAILABLE(req, resp, current_state, nearest_traffic_signal, entry_lanelet, exit_lanelet, current_lanelet);
        break;

      case TransitState::APPROACHING:
        planWhenAPPROACHING(req, resp, current_state, nearest_traffic_signal, entry_lanelet, exit_lanelet, current_lanelet);
        break;

      case TransitState::WAITING:
        planWhenWAITING(req, resp, current_state, nearest_traffic_signal, entry_lanelet, exit_lanelet, current_lanelet);
        break;

      case TransitState::DEPARTING:
        // Reset intersection state since we are no longer in an intersection
        if (!intersection_end_downtrack_ || !intersection_speed_)
        {
          throw std::invalid_argument("State is DEPARTING but the intersection variables are not available");
        }
        planWhenDEPARTING(req, resp, current_state, intersection_end_downtrack_.get(), intersection_speed_.get());
        break;

      default:
        throw std::invalid_argument("LCIStrategic Strategic Plugin entered unknown state");
        break;
    }

  } while (transition_table_.getState() != prev_state);  // If the state has changed then continue looping

  return true;
  // We need to evaluate the events so the state transitions can be triggered
}

cav_msgs::Maneuver LCIStrategicPlugin::composeTrajectorySmoothingManeuverMessage(double start_dist, double end_dist, const std::vector<lanelet::ConstLanelet>& crossed_lanelets, double start_speed,
                                                       double target_speed, ros::Time start_time, ros::Time end_time,
                                                       const TrajectoryParams& tsp) const
{
  cav_msgs::Maneuver maneuver_msg;
  maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
  maneuver_msg.lane_following_maneuver.parameters.negotiation_type =
      cav_msgs::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.lane_following_maneuver.parameters.presence_vector =
      cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN | cav_msgs::ManeuverParameters::HAS_FLOAT_META_DATA | cav_msgs::ManeuverParameters::HAS_INT_META_DATA;
  maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin =
      config_.lane_following_plugin_name;
  maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin =
      config_.strategic_plugin_name;
  maneuver_msg.lane_following_maneuver.start_dist = start_dist;
  maneuver_msg.lane_following_maneuver.start_speed = start_speed;
  maneuver_msg.lane_following_maneuver.start_time = start_time;
  maneuver_msg.lane_following_maneuver.end_dist = end_dist;
  maneuver_msg.lane_following_maneuver.end_speed = target_speed;
  maneuver_msg.lane_following_maneuver.end_time = end_time;
  
  maneuver_msg.lane_following_maneuver.parameters.string_valued_meta_data.push_back(light_controlled_intersection_strategy_);

  maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.a1_);
  maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.v1_);
  maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.x1_);

  maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.a2_);
  maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.v2_);
  maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.x2_);

  maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.a3_);
  maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.v3_);
  maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.x3_);

  maneuver_msg.lane_following_maneuver.parameters.int_valued_meta_data.push_back(static_cast<int>(tsp.case_num));
  maneuver_msg.lane_following_maneuver.parameters.int_valued_meta_data.push_back(static_cast<int>(tsp.is_algorithm_successful));

  for (auto llt : crossed_lanelets)
  {
    maneuver_msg.lane_following_maneuver.lane_ids.push_back(std::to_string(llt.id()));
  }
  // Start time and end time for maneuver are assigned in updateTimeProgress

  ROS_INFO_STREAM("Creating TrajectorySmoothingManeuver start dist: " << start_dist << " end dist: " << end_dist
                                                                      << " start_time: " << std::to_string(start_time.toSec())
                                                                      << " end_time: " << std::to_string(end_time.toSec()));

  return maneuver_msg;
}

cav_msgs::Maneuver LCIStrategicPlugin::composeStopAndWaitManeuverMessage(double current_dist, double end_dist,
                                                                        double start_speed,
                                                                        const lanelet::Id& starting_lane_id,
                                                                        const lanelet::Id& ending_lane_id,
                                                                        ros::Time start_time, ros::Time end_time, double stopping_accel) const
{
  cav_msgs::Maneuver maneuver_msg;
  maneuver_msg.type = cav_msgs::Maneuver::STOP_AND_WAIT;
  maneuver_msg.stop_and_wait_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector =
      cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN | cav_msgs::ManeuverParameters::HAS_FLOAT_META_DATA;
  maneuver_msg.stop_and_wait_maneuver.parameters.planning_tactical_plugin = config_.stop_and_wait_plugin_name;
  maneuver_msg.stop_and_wait_maneuver.parameters.planning_strategic_plugin = config_.strategic_plugin_name;
  maneuver_msg.stop_and_wait_maneuver.start_dist = current_dist;
  maneuver_msg.stop_and_wait_maneuver.end_dist = end_dist;
  maneuver_msg.stop_and_wait_maneuver.start_speed = start_speed;
  maneuver_msg.stop_and_wait_maneuver.start_time = start_time;
  maneuver_msg.stop_and_wait_maneuver.end_time = end_time;
  maneuver_msg.stop_and_wait_maneuver.starting_lane_id = std::to_string(starting_lane_id);
  maneuver_msg.stop_and_wait_maneuver.ending_lane_id = std::to_string(ending_lane_id);

  // Set the meta data for the stop location buffer
  maneuver_msg.stop_and_wait_maneuver.parameters.float_valued_meta_data.push_back(config_.stopping_location_buffer);
  maneuver_msg.stop_and_wait_maneuver.parameters.float_valued_meta_data.push_back(stopping_accel);

  ROS_INFO_STREAM("Creating stop and wait start dist: " << current_dist << " end dist: " << end_dist
                                                                        << " start_time: " << std::to_string(start_time.toSec())
                                                                        << " end_time: " << std::to_string(end_time.toSec())
                                                                        << " stopping_accel: " << stopping_accel);

  return maneuver_msg;
}

cav_msgs::Maneuver LCIStrategicPlugin::composeIntersectionTransitMessage(double start_dist, double end_dist,
                                                                        double start_speed, double target_speed,
                                                                        ros::Time start_time, ros::Time end_time,
                                                                        const lanelet::Id& starting_lane_id,
                                                                        const lanelet::Id& ending_lane_id) const
{
  cav_msgs::Maneuver maneuver_msg;
  maneuver_msg.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.negotiation_type =
      cav_msgs::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.presence_vector =
      cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.planning_tactical_plugin =
      config_.intersection_transit_plugin_name;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.planning_strategic_plugin =
      config_.strategic_plugin_name;
  maneuver_msg.intersection_transit_straight_maneuver.start_dist = start_dist;
  maneuver_msg.intersection_transit_straight_maneuver.start_speed = start_speed;
  maneuver_msg.intersection_transit_straight_maneuver.start_time = start_time;
  maneuver_msg.intersection_transit_straight_maneuver.end_dist = end_dist;
  maneuver_msg.intersection_transit_straight_maneuver.end_speed = target_speed;
  maneuver_msg.intersection_transit_straight_maneuver.end_time = end_time;
  maneuver_msg.intersection_transit_straight_maneuver.starting_lane_id = std::to_string(starting_lane_id);
  maneuver_msg.intersection_transit_straight_maneuver.ending_lane_id = std::to_string(ending_lane_id);

  // Start time and end time for maneuver are assigned in updateTimeProgress

  ROS_INFO_STREAM("Creating IntersectionTransitManeuver start dist: " << start_dist << " end dist: " << end_dist
                                                                      << " From lanelet: " << starting_lane_id
                                                                      << " to lanelet: " << ending_lane_id
                                                                      << " From start_time: " << start_time
                                                                      << " to end_time: " << end_time);

  return maneuver_msg;
}

}  // namespace lci_strategic_plugin