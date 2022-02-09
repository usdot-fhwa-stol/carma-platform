/*
 * Copyright (C) 2021 LEIDOS.
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
#include "wz_strategic_plugin/wz_strategic_plugin.h"
#include "wz_strategic_plugin/wz_states.h"

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

namespace wz_strategic_plugin
{
WzStrategicPlugin::WzStrategicPlugin(carma_wm::WorldModelConstPtr wm, WzStrategicPluginConfig config)
  : wm_(wm), config_(config)
{
  plugin_discovery_msg_.name = config_.strategic_plugin_name;
  plugin_discovery_msg_.version_id = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = true;
  plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
  plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";
};

cav_msgs::Plugin WzStrategicPlugin::getDiscoveryMsg() const
{
  return plugin_discovery_msg_;
}

bool WzStrategicPlugin::supportedLightState(lanelet::CarmaTrafficSignalState state) const
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

double WzStrategicPlugin::estimate_distance_to_stop(double v, double a) const
{
  return (v * v) / (2.0 * a);
}

double WzStrategicPlugin::estimate_time_to_stop(double d, double v) const // TODO remove
{
  return 2.0 * d / v;
};

WzStrategicPlugin::VehicleState WzStrategicPlugin::extractInitialState(const cav_srvs::PlanManeuversRequest& req) const
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

bool WzStrategicPlugin::validLightState(const boost::optional<std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>>& optional_state,
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
    ROS_ERROR_STREAM("WorkZone Plugin asked to handle CarmaTrafficSignalState: " << light_state
                                                                                << " which is not supported.");
    return false;
  }

  return true;
}

std::vector<lanelet::ConstLanelet> WzStrategicPlugin::getLaneletsBetweenWithException(double start_downtrack,
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

void WzStrategicPlugin::planWhenUNAVAILABLE(const cav_srvs::PlanManeuversRequest& req,
                                            cav_srvs::PlanManeuversResponse& resp, const VehicleState& current_state,
                                            const std::vector<lanelet::CarmaTrafficSignalPtr>& traffic_lights)
{
  // Reset intersection state since in this state we are not yet known to be in or approaching an intersection
  intersection_speed_ = boost::none;
  intersection_end_downtrack_ = boost::none;

  if (traffic_lights.empty())
  {
    ROS_DEBUG("No lights found along route. Returning maneuver plan unchanged");
    return;
  }

  double traffic_light_down_track =
      wm_->routeTrackPos(traffic_lights.front()->stopLine().front().front().basicPoint2d()).downtrack;

  ROS_DEBUG("traffic_light_down_track %f", traffic_light_down_track);

  double distance_remaining_to_traffic_light = traffic_light_down_track - current_state.downtrack - config_.vehicle_length;

  ROS_DEBUG("distance_remaining_to_traffic_light %f", distance_remaining_to_traffic_light);

  double stopping_dist = estimate_distance_to_stop(current_state.speed, config_.vehicle_decel_limit_multiplier  *
                                                                            config_.vehicle_decel_limit);

  ROS_DEBUG_STREAM("Stopping distance: " << stopping_dist);

  double plugin_activation_distance = std::max(stopping_dist, config_.min_approach_distance);

  ROS_DEBUG_STREAM("plugin_activation_distance: " << plugin_activation_distance);

  if (distance_remaining_to_traffic_light <= plugin_activation_distance)
  {
    ROS_INFO_STREAM("Within intersection range");
    transition_table_.signal(TransitEvent::IN_STOPPING_RANGE);  // Evaluate Signal
  }
  else
  {
    ROS_DEBUG_STREAM("Not within intersection range");
  }
}
// TODO should we handle when the vehicle is not going to make the light but doesn't have space to stop?
void WzStrategicPlugin::planWhenAPPROACHING(const cav_srvs::PlanManeuversRequest& req,
                                            cav_srvs::PlanManeuversResponse& resp, const VehicleState& current_state,
                                            const std::vector<lanelet::CarmaTrafficSignalPtr>& traffic_lights)
{
  if (traffic_lights.empty())  // If we are in the approaching state and there is no longer any lights ahead of us then
                               // the vehicle must have crossed the stop bar
  {
    transition_table_.signal(TransitEvent::CROSSED_STOP_BAR);
    return;
  }

  auto nearest_traffic_light = traffic_lights.front();

  double traffic_light_down_track =
      wm_->routeTrackPos(nearest_traffic_light->stopLine().front().front().basicPoint2d()).downtrack;

  ROS_DEBUG("traffic_light_down_track %f", traffic_light_down_track);

  double distance_remaining_to_traffic_light = traffic_light_down_track - current_state.downtrack;

  // If the vehicle is at a stop trigger the
  constexpr double HALF_MPH_IN_MPS = 0.22352;
  if (current_state.speed < HALF_MPH_IN_MPS &&
      fabs(distance_remaining_to_traffic_light) < config_.stopping_location_buffer)
  {
    transition_table_.signal(TransitEvent::STOPPED);  // The vehicle has come to a stop at the light
    return;
  }

  // At this point we know the vehicle is within the activation distance and we know the current and next light phases
  // All we need to now determine is if we should stop or if we should continue

  intersection_speed_ = findSpeedLimit(nearest_traffic_light->getControlStartLanelets().front());

  ROS_DEBUG_STREAM("intersection_speed_: " << intersection_speed_.get());

  intersection_end_downtrack_ =
      wm_->routeTrackPos(nearest_traffic_light->getControlEndLanelets().back().centerline2d().back()).downtrack;

  ROS_DEBUG_STREAM("intersection_end_downtrack_: " << intersection_end_downtrack_.get());

  // Estimate the time to reach the traffic light assuming we decelerate to the speed of the intersection before arrival
  // In the case of higher accel limits this calculation should always overestimate the arrival time which should be
  // fine as that would hit the following red phase.
  double time_remaining_to_traffic_light =
      (2.0 * (distance_remaining_to_traffic_light - config_.vehicle_length)) /
      (intersection_speed_.get() + current_state.speed);  // Kinematic Equation: 2*d / (vf + vi) = t

  ROS_DEBUG_STREAM("time_remaining_to_traffic_light: " << time_remaining_to_traffic_light);

  ros::Time light_arrival_time_at_freeflow = current_state.stamp + ros::Duration(time_remaining_to_traffic_light);
  
  ros::Time early_arrival_time_at_freeflow =
      light_arrival_time_at_freeflow - ros::Duration(config_.green_light_time_buffer);

  ros::Time late_arrival_time_at_freeflow =
      light_arrival_time_at_freeflow + ros::Duration(config_.green_light_time_buffer);

  ROS_DEBUG_STREAM("light_arrival_time_at_freeflow: " << std::to_string(light_arrival_time_at_freeflow.toSec()));
  ROS_DEBUG_STREAM("early_arrival_time_at_freeflow: " << std::to_string(early_arrival_time_at_freeflow.toSec()));
  ROS_DEBUG_STREAM("late_arrival_time_at_freeflow: " << std::to_string(late_arrival_time_at_freeflow.toSec()));

  auto early_arrival_state_at_freeflow_optional = nearest_traffic_light->predictState(lanelet::time::timeFromSec(early_arrival_time_at_freeflow.toSec()));

  if (!validLightState(early_arrival_state_at_freeflow_optional, early_arrival_time_at_freeflow))
    return;

  ROS_DEBUG_STREAM("early_arrival_state_at_freeflow: " << early_arrival_state_at_freeflow_optional.get().second);

  auto late_arrival_state_at_freeflow_optional = nearest_traffic_light->predictState(lanelet::time::timeFromSec(late_arrival_time_at_freeflow.toSec()));

  if (!validLightState(late_arrival_state_at_freeflow_optional, late_arrival_time_at_freeflow))
    return;

  ROS_DEBUG_STREAM("late_arrival_state_at_freeflow: " << late_arrival_state_at_freeflow_optional.get().second);

  // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
      getLaneletsBetweenWithException(current_state.downtrack, traffic_light_down_track, true, true);

  // We will cross the light on the green phase even if we arrive early or late
  if (early_arrival_state_at_freeflow_optional.get().second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED &&
      late_arrival_state_at_freeflow_optional.get().second ==
          lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED)  // Green light
  {

    ROS_DEBUG_STREAM("Planning lane follow and intersection transit maneuvers");
    // TODO plan lane follow from current speed to intersection speed (does inlane-cruising even support this???)

    // TODO do we need to check for anything before pushing onto the plan?
    resp.new_plan.maneuvers.push_back(composeLaneFollowingManeuverMessage(
        current_state.downtrack, traffic_light_down_track - config_.vehicle_length, current_state.speed, intersection_speed_.get(),
        current_state.stamp, light_arrival_time_at_freeflow,
        lanelet::utils::transform(crossed_lanelets, [](const auto& ll) { return ll.id(); })));

    double intersection_length = intersection_end_downtrack_.get() - traffic_light_down_track;

    ros::Time intersection_exit_time =
        light_arrival_time_at_freeflow + ros::Duration(intersection_length / intersection_speed_.get());

    resp.new_plan.maneuvers.push_back(composeIntersectionTransitMessage(
        traffic_light_down_track - config_.vehicle_length, intersection_end_downtrack_.get(), intersection_speed_.get(),
        intersection_speed_.get(), light_arrival_time_at_freeflow, intersection_exit_time, crossed_lanelets.back().id(),
        nearest_traffic_light->getControlEndLanelets().back().id()));
  }
  else  // Red or Yellow light
  {
    double stopping_accel = config_.vehicle_decel_limit_multiplier * config_.vehicle_decel_limit;

    ROS_DEBUG_STREAM("Planning stop and wait maneuver");
    resp.new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(
        current_state.downtrack, traffic_light_down_track - config_.vehicle_length, current_state.speed, crossed_lanelets.front().id(),
        crossed_lanelets.back().id(), current_state.stamp,
        req.header.stamp + ros::Duration(config_.min_maneuver_planning_period), stopping_accel));
  }
}

void WzStrategicPlugin::planWhenWAITING(const cav_srvs::PlanManeuversRequest& req,
                                        cav_srvs::PlanManeuversResponse& resp, const VehicleState& current_state,
                                        const std::vector<lanelet::CarmaTrafficSignalPtr>& traffic_lights)
{
  if (traffic_lights.empty())
  {
    throw std::invalid_argument("While in WAITING state, the traffic lights disappeared.");
  }

  auto nearest_traffic_light = traffic_lights.front();

  double traffic_light_down_track =
      wm_->routeTrackPos(nearest_traffic_light->stopLine().front().front().basicPoint2d()).downtrack;

  ROS_DEBUG("traffic_light_down_track %f", traffic_light_down_track);

  auto current_light_state_optional = nearest_traffic_light->predictState(lanelet::time::timeFromSec(req.header.stamp.toSec()));

  if (!validLightState(current_light_state_optional, req.header.stamp))
    return;

  if (current_light_state_optional.get().second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED)
  {
    transition_table_.signal(TransitEvent::RED_TO_GREEN_LIGHT);  // If the light is green send the light transition
                                                                 // signal
    return;
  }

  // A fixed buffer to add to stopping maneuvers once the vehicle is already stopped to ensure that they can have their
  // trajectory planned
  constexpr double stop_maneuver_buffer = 10.0;

  // If the light is not green then continue waiting by creating a stop and wait maneuver ontop of the vehicle
  double stopping_accel = config_.vehicle_decel_limit_multiplier * config_.vehicle_decel_limit;

  resp.new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(
      current_state.downtrack - stop_maneuver_buffer, traffic_light_down_track, current_state.speed,
      current_state.lane_id, current_state.lane_id, current_state.stamp,
      current_state.stamp + ros::Duration(config_.min_maneuver_planning_period), stopping_accel));
}

void WzStrategicPlugin::planWhenDEPARTING(const cav_srvs::PlanManeuversRequest& req,
                                          cav_srvs::PlanManeuversResponse& resp, const VehicleState& current_state,
                                          double intersection_end_downtrack, double intersection_speed_limit)
{
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

bool WzStrategicPlugin::planManeuverCb(cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp)
{
  if (!wm_->getRoute())
  {
    ROS_ERROR_STREAM("Could not plan maneuvers as route was not available");
    return true;
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

  TransitState prev_state;

  do
  {
    // Clear previous maneuvers planned by this plugin as guard against state change since each state generates an
    // independant set of maneuvers
    resp.new_plan = cav_msgs::ManeuverPlan();

    prev_state = transition_table_.getState();  // Cache previous state to check if state has changed after 1 iteration

    /* NOTE: Leaving this commented out code is intentional to provide an easy way to monitor light state at runtime. If a better way is implemented then this can be removed
    if (!traffic_list.empty()) { 
      auto nearest_traffic_light = traffic_list.front();
      ROS_ERROR_STREAM("\n\nCurrent Light State: " << nearest_traffic_light->getState().get() 
      << std::endl <<  "                      1: " << nearest_traffic_light->predictState(lanelet::time::timeFromSec((ros::Time::now() + ros::Duration(1.0)).toSec())).get()
      << std::endl <<  "                      2: " << nearest_traffic_light->predictState(lanelet::time::timeFromSec((ros::Time::now() + ros::Duration(2.0)).toSec())).get()
      << std::endl <<  "                      3: " << nearest_traffic_light->predictState(lanelet::time::timeFromSec((ros::Time::now() + ros::Duration(3.0)).toSec())).get()
      << std::endl <<  "                      4: " << nearest_traffic_light->predictState(lanelet::time::timeFromSec((ros::Time::now() + ros::Duration(4.0)).toSec())).get()
      << std::endl <<  "                      5: " << nearest_traffic_light->predictState(lanelet::time::timeFromSec((ros::Time::now() + ros::Duration(5.0)).toSec())).get()
      << std::endl);
    }
    */
    ROS_INFO_STREAM("Planning in state: " << transition_table_.getState());
    switch (transition_table_.getState())
    {
      case TransitState::UNAVAILABLE:
        planWhenUNAVAILABLE(req, resp, current_state, traffic_list);
        break;

      case TransitState::APPROACHING:
        planWhenAPPROACHING(req, resp, current_state, traffic_list);
        break;

      case TransitState::WAITING:
        planWhenWAITING(req, resp, current_state, traffic_list);
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
        throw std::invalid_argument("WorkZone Strategic Plugin entered unknown state");
        break;
    }

  } while (transition_table_.getState() != prev_state);  // If the state has changed then continue looping

  return true;
  // We need to evaluate the events so the state transitions can be triggered
}

cav_msgs::Maneuver WzStrategicPlugin::composeLaneFollowingManeuverMessage(double start_dist, double end_dist,
                                                                          double start_speed, double target_speed,
                                                                          ros::Time start_time, ros::Time end_time,
                                                                          std::vector<lanelet::Id> lane_ids) const
{
  cav_msgs::Maneuver maneuver_msg;
  maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
  maneuver_msg.lane_following_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
  maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = config_.lane_following_plugin_name;
  maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = config_.strategic_plugin_name;
  maneuver_msg.lane_following_maneuver.start_dist = start_dist;
  maneuver_msg.lane_following_maneuver.start_speed = start_speed;
  maneuver_msg.lane_following_maneuver.end_dist = end_dist;
  maneuver_msg.lane_following_maneuver.end_speed = target_speed;
  maneuver_msg.lane_following_maneuver.start_time = start_time;
  maneuver_msg.lane_following_maneuver.end_time = end_time;
  maneuver_msg.lane_following_maneuver.lane_ids =
      lanelet::utils::transform(lane_ids, [](auto id) { return std::to_string(id); });

  ROS_INFO_STREAM("Creating lane follow start dist: " << start_dist << " end dist: " << end_dist);

  return maneuver_msg;
}

cav_msgs::Maneuver WzStrategicPlugin::composeStopAndWaitManeuverMessage(double current_dist, double end_dist,
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

  ROS_INFO_STREAM("Creating stop and wait start dist: " << current_dist << " end dist: " << end_dist);

  return maneuver_msg;
}

cav_msgs::Maneuver WzStrategicPlugin::composeIntersectionTransitMessage(double start_dist, double end_dist,
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
                                                                      << " to lanelet: " << ending_lane_id);

  return maneuver_msg;
}

double WzStrategicPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt) const
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

}  // namespace wz_strategic_plugin