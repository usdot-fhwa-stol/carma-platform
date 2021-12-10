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
#include "sci_strategic_plugin.h"

#define GET_MANEUVER_PROPERTY(mvr, property)                                                                           \
  (((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ?                                                 \
        (mvr).intersection_transit_left_turn_maneuver.property :                                                       \
        ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ?                                           \
             (mvr).intersection_transit_right_turn_maneuver.property :                                                 \
             ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ?                                        \
                  (mvr).intersection_transit_straight_maneuver.property :                                              \
                  ((mvr).type == cav_msgs::Maneuver::LANE_CHANGE    ? (mvr).lane_change_maneuver.property :            \
                  ((mvr).type == cav_msgs::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :         \
                  ((mvr).type == cav_msgs::Maneuver::STOP_AND_WAIT ? (mvr).stop_and_wait_maneuver.property :           \
                                                                      throw new std::invalid_argument("GET_MANEUVER_"  \
                                                                                                      "PROPERTY "      \
                                                                                                      "(property) "    \
                                                                                                      "called on "     \
                                                                                                      "maneuver with " \
                                                                                                      "invalid type "  \
                                                                                                      "id"))))))))

namespace sci_strategic_plugin
{
SCIStrategicPlugin::SCIStrategicPlugin(carma_wm::WorldModelConstPtr wm, SCIStrategicPluginConfig& config)
  : wm_(wm), config_(config)
{
  plugin_discovery_msg_.name = config_.strategic_plugin_name;
  plugin_discovery_msg_.version_id = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = true;
  plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
  plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";
};

cav_msgs::Plugin SCIStrategicPlugin::getDiscoveryMsg() const
{
  return plugin_discovery_msg_;
}



SCIStrategicPlugin::VehicleState SCIStrategicPlugin::extractInitialState(const cav_srvs::PlanManeuversRequest& req) const
{
  VehicleState state;
  if (!req.prior_plan.maneuvers.empty())
  {
    ROS_DEBUG_STREAM("Provided with initial plan...");
    state.stamp = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_time);
    state.speed = getManeuverEndSpeed(req.prior_plan.maneuvers.back());
    state.downtrack = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_dist);
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
  
  ROS_DEBUG_STREAM("state.stamp: " << state.stamp);
  ROS_DEBUG_STREAM("state.downtrack : " << state.downtrack );
  ROS_DEBUG_STREAM("state.speed: " << state.speed);
  ROS_DEBUG_STREAM("state.lane_id: " << state.lane_id);

  return state;
}

void SCIStrategicPlugin::mobilityOperationCb(const cav_msgs::MobilityOperationConstPtr& msg)
{
  if (msg->strategy == stop_controlled_intersection_strategy_)
  {
    approaching_stop_controlled_interction_ = true;
    if (msg->strategy_params != previous_strategy_params_)
    {
      parseStrategyParams(msg->strategy_params); 
    }
    previous_strategy_params_ = msg->strategy_params;

    generateMobilityOperation();
  }
  
}

void SCIStrategicPlugin::BSMCb(const cav_msgs::BSMConstPtr& msg)
{
  bsm_id = msg->core_data.id;
}

void SCIStrategicPlugin::currentPoseCb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::PoseStamped pose_msg = geometry_msgs::PoseStamped(*msg.get());
  lanelet::BasicPoint2d current_loc(pose_msg.pose.position.x, pose_msg.pose.position.y);
  current_downtrack_ = wm_->routeTrackPos(current_loc).downtrack;
  ROS_DEBUG_STREAM("Downtrack from current pose: " << current_downtrack_);
}

void SCIStrategicPlugin::parseStrategyParams(const std::string& strategy_params)
{
  std::istringstream strategy_params_ss(strategy_params);
  boost::property_tree::ptree parser;
  boost::property_tree::ptree child;
  boost::property_tree::json_parser::read_json(strategy_params_ss, parser);
  child = parser.get_child("schedule_plan");
  for(const auto& p : child)
  {
    if (p.first == "metadata")
    {
      street_msg_timestamp_ = p.second.get<uint32_t>("timestamp");
    }
    if (p.first == "payload" && p.second.get<std::string>("veh_id") == config_.vehicle_id)
    {        
      // parse stop time in ms
      scheduled_stop_time_ = p.second.get<uint32_t>("est_stop_t");
      ROS_DEBUG_STREAM("scheduled_stop_time_: " << scheduled_stop_time_);

      scheduled_enter_time_ = p.second.get<uint32_t>("est_enter_t");
      ROS_DEBUG_STREAM("scheduled_enter_time_: " << scheduled_enter_time_);
      
      scheduled_depart_time_ = p.second.get<uint32_t>("est_depart_t");
      ROS_DEBUG_STREAM("scheduled_depart_time_: " << scheduled_depart_time_);

      scheduled_latest_depart_time_ = p.second.get<uint32_t>("latest_depart_p");
      ROS_DEBUG_STREAM("scheduled_latest_depart_time_: " << scheduled_latest_depart_time_);

      is_allowed_int_ = p.second.get<bool>("is_allowed_int");
      ROS_DEBUG_STREAM("is_allowed_int: " << is_allowed_int_);
    }
  }
}

int SCIStrategicPlugin::determineSpeedProfileCase(double stop_dist, double current_speed, double schedule_stop_time, double speed_limit)
{
  int case_num = 0;
  double estimated_stop_time = calcEstimatedStopTime(stop_dist, current_speed);
  
  ROS_DEBUG_STREAM("estimated_stop_time: " << estimated_stop_time);
  if (estimated_stop_time < schedule_stop_time)
  {
    case_num = 3;
  }
  else
  {
    double speed_before_stop = calcSpeedBeforeDecel(estimated_stop_time, stop_dist, current_speed);
    if (speed_before_stop < speed_limit)
    {
      case_num = 1;
    }
    else
    {
      case_num = 2;
    }
  }
  
  return case_num;
}

double SCIStrategicPlugin::calcEstimatedStopTime(double stop_dist, double current_speed) const
{
  
  double t_stop = 0;
  t_stop = 2*stop_dist/current_speed;
  return t_stop;
}

double SCIStrategicPlugin::calcSpeedBeforeDecel(double stop_time, double stop_dist, double current_speed) const
{
  double speed_before_decel = 0;

  double desired_acceleration = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  double desired_deceleration = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;

  double sqr_term = sqrt(pow(1 - (desired_acceleration/desired_deceleration), 2) * pow(stop_dist/stop_time, 2) 
                        + (1 - (desired_acceleration/desired_deceleration))*(current_speed*current_speed - 2*current_speed*stop_dist/stop_time));

  speed_before_decel = (stop_dist/stop_time) + sqr_term/(1 - (desired_acceleration/desired_deceleration));

  return speed_before_decel;
}


std::vector<lanelet::ConstLanelet> SCIStrategicPlugin::getLaneletsBetweenWithException(double start_downtrack,
                                                                                      double end_downtrack,
                                                                                      bool shortest_path_only,
                                                                                      bool bounds_inclusive) const
{
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
      wm_->getLaneletsBetween(start_downtrack, end_downtrack, shortest_path_only, bounds_inclusive);

  if (crossed_lanelets.empty())
  {
    throw std::invalid_argument("getLaneletsBetweenWithException called but inputs do not cross any lanelets going "
                                "from: " +
                                std::to_string(start_downtrack) + " to: " + std::to_string(end_downtrack));
  }

  return crossed_lanelets;
}



bool SCIStrategicPlugin::planManeuverCb(cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp)
{
  if (!wm_->getRoute())
  {
    ROS_ERROR_STREAM("Could not plan maneuvers as route was not available");
    return true;
  }

  if (!approaching_stop_controlled_interction_)
  {
    resp.new_plan.maneuvers = {};
    ROS_WARN_STREAM("Not approaching stop-controlled itnersection so no maneuvers");
    return true;
  }

  ROS_DEBUG("Finding car information");
  // Extract vehicle data from request
  VehicleState current_state = extractInitialState(req);

  // Get current traffic light information
  ROS_DEBUG("\n\nFinding intersecction information");

  auto stop_intersection_list = wm_->getIntersectionsAlongRoute({ req.veh_x, req.veh_y });
  auto nearest_stop_intersection = stop_intersection_list.front();
  double stop_intersection_down_track =
  wm_->routeTrackPos(nearest_stop_intersection->stopLines().front().front().basicPoint2d()).downtrack;

  double distance_to_stopline = stop_intersection_down_track - current_downtrack_;
  std::cout << "distance_to_stopline  " << distance_to_stopline << std::endl;

  if (distance_to_stopline >= 0)
  {
    if (distance_to_stopline > config_.stop_line_buffer)
    {
      // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
      std::vector<lanelet::ConstLanelet> crossed_lanelets =
          getLaneletsBetweenWithException(current_downtrack_, stop_intersection_down_track, true, true);

      // approaching stop line
      speed_limit_ = findSpeedLimit(crossed_lanelets.front());

      // lane following to intersection
      double time_to_schedule_stop = (scheduled_stop_time_ - street_msg_timestamp_)*1000.0;
      int case_num = determineSpeedProfileCase(stop_intersection_down_track, current_state.speed, time_to_schedule_stop, speed_limit_);

      resp.new_plan.maneuvers.push_back(composeLaneFollowingManeuverMessage(
        case_num, current_downtrack_, stop_intersection_down_track, current_state.speed, 0.0,
        current_state.stamp, time_to_schedule_stop,
        lanelet::utils::transform(crossed_lanelets, [](const auto& ll) { return ll.id(); })));
    }
    else
    {
      // at the stop line
      // stop and wait maneuver
      auto stop_line_lanelet = nearest_stop_intersection->lanelets().front();
      double stop_duration = (scheduled_enter_time_ - scheduled_stop_time_)*1000.0;
      ROS_DEBUG_STREAM("Planning stop and wait maneuver");
      resp.new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(
        current_downtrack_, stop_intersection_down_track, current_state.speed, stop_line_lanelet.id(),
        stop_line_lanelet.id(), current_state.stamp,
        current_state.stamp + ros::Duration(stop_duration)));

    }

  }
  else
  {
    // Passed the stop line
    // Intersection transit maneuver

    
    // Compose intersection transit maneuver
    double intersection_transit_time = (scheduled_depart_time_ - scheduled_enter_time_)*1000.0;

    double intersection_end_downtrack =
      wm_->routeTrackPos(nearest_stop_intersection->lanelets().back().centerline2d().back()).downtrack;

    // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
    std::vector<lanelet::ConstLanelet> crossed_lanelets =
          getLaneletsBetweenWithException(current_downtrack_, intersection_end_downtrack, true, true);

    double intersection_speed_limit = findSpeedLimit(nearest_stop_intersection->lanelets().front());

    resp.new_plan.maneuvers.push_back(composeIntersectionTransitMessage(
      current_downtrack_, intersection_end_downtrack, current_state.speed, intersection_speed_limit,
      current_state.stamp, req.header.stamp + ros::Duration(intersection_transit_time), crossed_lanelets.front().id(), crossed_lanelets.back().id()));
    // when passing intersection, set the flag to false
    // TODO: Another option, check the time on mobilityoperation
    approaching_stop_controlled_interction_ = false;

  }
  
  return true;
}

void SCIStrategicPlugin::caseOneSpeedProfile(double speed_before_decel, double current_speed, double stop_time, 
                                            std::vector<double>* float_metadata_list) const
{
  double desired_acceleration = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  double desired_deceleration = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;  

  // Equations obtained from TSMO UC 1 Algorithm draft doc
  double a_acc = ((1 - desired_acceleration/desired_deceleration)*speed_before_decel - current_speed)/stop_time;
  double a_dec = ((desired_deceleration - desired_acceleration)*speed_before_decel - desired_deceleration * current_speed)/(desired_acceleration * stop_time);
  double t_acc = (speed_before_decel - current_speed)/a_acc;
  double t_dec = -speed_before_decel/a_dec; // a_dec is negative so a - is used to make the t_dec positive. 
  float_metadata_list->push_back(a_acc);
  float_metadata_list->push_back(a_dec);
  float_metadata_list->push_back(t_acc);
  float_metadata_list->push_back(t_dec);
  float_metadata_list->push_back(speed_before_decel);
}

void SCIStrategicPlugin::caseTwoSpeedProfile(double stop_dist, double speed_before_decel, 
                                            double current_speed, double stop_time,  double speed_limit, 
                                            std::vector<double>* float_metadata_list) const
{
  double desired_acceleration = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  double desired_deceleration = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;

  if (speed_before_decel > speed_limit)
  {
    speed_before_decel = speed_limit;
  }
  
  double t_c_nom = 2*stop_dist * ((1 - desired_acceleration/desired_deceleration)*speed_limit - current_speed) - 
                  stop_time * ((1 - desired_acceleration/desired_deceleration)*pow(speed_limit,2) - pow(current_speed, 2));
  double t_c_den = pow(speed_limit - current_speed, 2) - (desired_acceleration/desired_deceleration) * pow(speed_limit, 2);
  double t_cruise = t_c_nom / t_c_den;
  
  // Equations obtained from TSMO UC 1 Algorithm draft doc
  double a_acc = ((1 - desired_acceleration/desired_deceleration)*speed_limit - current_speed)/(stop_time - t_cruise);
  double a_dec = ((desired_deceleration - desired_acceleration)*speed_limit - desired_deceleration * current_speed)/(desired_acceleration*(stop_time - t_cruise));
  double t_acc = (speed_limit - current_speed)/a_acc;
  double t_dec = -speed_limit/a_dec; // a_dec is negative so a - is used to make the t_dec positive. 

  float_metadata_list->push_back(a_acc);
  float_metadata_list->push_back(a_dec);
  float_metadata_list->push_back(t_acc);
  float_metadata_list->push_back(t_dec);
  float_metadata_list->push_back(t_cruise);
  float_metadata_list->push_back(speed_before_decel);

}

void SCIStrategicPlugin::caseThreeSpeedProfile(double stop_dist, double current_speed, double stop_time, 
                                                std::vector<double>* float_metadata_list) const
{
  double a_dec = (2*stop_dist - current_speed*(stop_time + config_.delta_t))/(stop_time * config_.delta_t);
  float_metadata_list->push_back(a_dec);
}

cav_msgs::Maneuver SCIStrategicPlugin::composeLaneFollowingManeuverMessage(int case_num, double start_dist, double end_dist,
                                                                          double start_speed, double target_speed,
                                                                          ros::Time start_time, double time_to_stop,
                                                                          std::vector<lanelet::Id> lane_ids)
{
  cav_msgs::Maneuver maneuver_msg;
  cav_msgs::Maneuver empty_msg;
  maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
  maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = config_.strategic_plugin_name;
  maneuver_msg.lane_following_maneuver.parameters.presence_vector = 
  cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN | cav_msgs::ManeuverParameters::HAS_INT_META_DATA | 
  cav_msgs::ManeuverParameters::HAS_FLOAT_META_DATA | cav_msgs::ManeuverParameters::HAS_STRING_META_DATA;
  maneuver_msg.lane_following_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = config_.lane_following_plugin_name;
  maneuver_msg.lane_following_maneuver.start_dist = start_dist;
  maneuver_msg.lane_following_maneuver.end_dist = end_dist;
  maneuver_msg.lane_following_maneuver.start_speed = start_speed;
  maneuver_msg.lane_following_maneuver.end_speed = target_speed;
  maneuver_msg.lane_following_maneuver.start_time = start_time;
  maneuver_msg.lane_following_maneuver.end_time =  start_time + ros::Duration(time_to_stop);
  maneuver_msg.lane_following_maneuver.lane_ids =
      lanelet::utils::transform(lane_ids, [](auto id) { return std::to_string(id); });

  ROS_INFO_STREAM("Creating lane follow start dist: " << start_dist << " end dist: " << end_dist);

  std::vector<double> float_metadata_list;

  double speed_before_decel = calcSpeedBeforeDecel(time_to_stop, end_dist, start_speed);

  switch(case_num)
  {
    case 1:
      caseOneSpeedProfile(speed_before_decel, start_speed, time_to_stop, &float_metadata_list);
      break;
    case 2:
      caseTwoSpeedProfile(end_dist, speed_before_decel, start_speed, time_to_stop, speed_limit_, &float_metadata_list);
      break;
    case 3:
      caseThreeSpeedProfile(end_dist, start_speed, time_to_stop, &float_metadata_list);
      break;
    default:
      return empty_msg;
  }
  
  
  maneuver_msg.lane_following_maneuver.parameters.int_valued_meta_data.push_back(case_num);
  maneuver_msg.lane_following_maneuver.parameters.string_valued_meta_data.push_back(stop_controlled_intersection_strategy_);
  maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data = float_metadata_list;
  
  return maneuver_msg;
}


double SCIStrategicPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt) const
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

void SCIStrategicPlugin::generateMobilityOperation()
{
    cav_msgs::MobilityOperation mo_;
    mo_.m_header.timestamp = ros::Time::now().toNSec() * 1000000;

    std::string id(bsm_id.begin(), bsm_id.end());
    mo_.m_header.sender_bsm_id = id;

    int flag = (approaching_stop_controlled_interction_ ? 1 : 0);

    double vehicle_acceleration_limit_ = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
    double vehicle_deceleration_limit_ = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;

    mo_.strategy_params = "intersection_box_flag," +  std::to_string(flag) + ",acceleration_limit," + std::to_string(vehicle_acceleration_limit_) + ",deceleration_limit," + std::to_string(vehicle_deceleration_limit_);

    mobility_operation_pub.publish(mo_);
}

cav_msgs::Maneuver SCIStrategicPlugin::composeStopAndWaitManeuverMessage(double current_dist, double end_dist,
                                                                        double start_speed,
                                                                        const lanelet::Id& starting_lane_id,
                                                                        const lanelet::Id& ending_lane_id,
                                                                        ros::Time start_time, ros::Time end_time) const
{
  cav_msgs::Maneuver maneuver_msg;
  maneuver_msg.type = cav_msgs::Maneuver::STOP_AND_WAIT;
  maneuver_msg.stop_and_wait_maneuver.parameters.planning_strategic_plugin = config_.strategic_plugin_name;
  maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector =
      cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN | cav_msgs::ManeuverParameters::HAS_FLOAT_META_DATA;
  maneuver_msg.stop_and_wait_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.stop_and_wait_maneuver.parameters.planning_tactical_plugin = config_.stop_and_wait_plugin_name;
  maneuver_msg.stop_and_wait_maneuver.start_speed = start_speed;
  maneuver_msg.stop_and_wait_maneuver.start_dist = current_dist;
  maneuver_msg.stop_and_wait_maneuver.end_dist = end_dist;
  maneuver_msg.stop_and_wait_maneuver.start_time = start_time;
  maneuver_msg.stop_and_wait_maneuver.end_time = end_time;
  maneuver_msg.stop_and_wait_maneuver.starting_lane_id = std::to_string(starting_lane_id);
  maneuver_msg.stop_and_wait_maneuver.ending_lane_id = std::to_string(ending_lane_id);
  // Set the meta data for the stop location buffer
  maneuver_msg.stop_and_wait_maneuver.parameters.float_valued_meta_data.push_back(config_.stop_line_buffer);
  return maneuver_msg;
}

cav_msgs::Maneuver SCIStrategicPlugin::composeIntersectionTransitMessage(double start_dist, double end_dist,
                                                                        double start_speed, double target_speed,
                                                                        ros::Time start_time, ros::Time end_time,
                                                                        const lanelet::Id& starting_lane_id,
                                                                        const lanelet::Id& ending_lane_id) const
{
  cav_msgs::Maneuver maneuver_msg;
  maneuver_msg.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.planning_strategic_plugin =
      config_.strategic_plugin_name;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.planning_tactical_plugin =
      config_.intersection_transit_plugin_name;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.presence_vector =
      cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.negotiation_type =
      cav_msgs::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.intersection_transit_straight_maneuver.start_dist = start_dist;
  maneuver_msg.intersection_transit_straight_maneuver.end_dist = end_dist;
  maneuver_msg.intersection_transit_straight_maneuver.start_speed = start_speed;
  maneuver_msg.intersection_transit_straight_maneuver.end_speed = target_speed;
  maneuver_msg.intersection_transit_straight_maneuver.start_time = start_time;
  maneuver_msg.intersection_transit_straight_maneuver.end_time = end_time;
  maneuver_msg.intersection_transit_straight_maneuver.starting_lane_id = std::to_string(starting_lane_id);
  maneuver_msg.intersection_transit_straight_maneuver.ending_lane_id = std::to_string(ending_lane_id);

  // Start time and end time for maneuver are assigned in updateTimeProgress

  ROS_INFO_STREAM("Creating IntersectionTransitManeuver start dist: " << start_dist << " end dist: " << end_dist
                                                                      << " From lanelet: " << starting_lane_id
                                                                      << " to lanelet: " << ending_lane_id);

  return maneuver_msg;
}

}  // namespace SCI_strategic_plugin
