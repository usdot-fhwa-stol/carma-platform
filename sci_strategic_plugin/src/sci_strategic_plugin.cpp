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
#include "sci_strategic_plugin.hpp"

#define GET_MANEUVER_PROPERTY(mvr, property)                                                                           \
  (((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ?                                                 \
        (mvr).intersection_transit_left_turn_maneuver.property :                                                       \
        ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ?                                           \
             (mvr).intersection_transit_right_turn_maneuver.property :                                                 \
             ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ?                                        \
                  (mvr).intersection_transit_straight_maneuver.property :                                              \
                  ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_CHANGE    ? (mvr).lane_change_maneuver.property :            \
                  ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :         \
                  ((mvr).type == carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT ? (mvr).stop_and_wait_maneuver.property :           \
                                                                      throw new std::invalid_argument("GET_MANEUVER_"  \
                                                                                                      "PROPERTY "      \
                                                                                                      "(property) "    \
                                                                                                      "called on "     \
                                                                                                      "maneuver with " \
                                                                                                      "invalid type "  \
                                                                                                      "id"))))))))


namespace sci_strategic_plugin
{
namespace std_ph = std::placeholders;

namespace {
  /**
  * \brief Anonymous function to extract maneuver end speed which can not be optained with GET_MANEUVER_PROPERY calls due to it missing in stop and wait plugin
  * \param mvr input maneuver
  * \return end speed
  */ 
  double getManeuverEndSpeed(const carma_planning_msgs::msg::Maneuver& mvr)
  {
    switch(mvr.type) 
    {
      case carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING:
          return mvr.lane_following_maneuver.end_speed;
      case carma_planning_msgs::msg::Maneuver::LANE_CHANGE:
          return mvr.lane_change_maneuver.end_speed;
      case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
          return mvr.intersection_transit_straight_maneuver.end_speed;
      case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
          return mvr.intersection_transit_left_turn_maneuver.end_speed;
      case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
          return mvr.intersection_transit_right_turn_maneuver.end_speed;
      case carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT:
          return 0;
      default:
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Requested end speed from unsupported maneuver type");
          return 0;
    }
  }
} // namespace anonymous

SCIStrategicPlugin::SCIStrategicPlugin(const rclcpp::NodeOptions &options)
  : carma_guidance_plugins::StrategicPlugin(options), config_(SCIStrategicPluginConfig())
{
  // Declare parameters
  config_.vehicle_decel_limit_multiplier = declare_parameter<double>("vehicle_decel_limit_multiplier",   config_.vehicle_decel_limit_multiplier);
  config_.vehicle_accel_limit_multiplier = declare_parameter<double>("vehicle_accel_limit_multiplier",   config_.vehicle_accel_limit_multiplier);
  config_.stop_line_buffer = declare_parameter<double>("stop_line_buffer",   config_.stop_line_buffer);
  config_.delta_t = declare_parameter<double>("delta_t",   config_.delta_t);
  config_.min_gap = declare_parameter<double>("min_gap",   config_.min_gap);
  config_.reaction_time = declare_parameter<double>("reaction_time",   config_.reaction_time);
  config_.intersection_exit_zone_length = declare_parameter<double>("intersection_exit_zone_length",   config_.intersection_exit_zone_length);
  config_.strategic_plugin_name = declare_parameter<std::string>("strategic_plugin_name",            config_.strategic_plugin_name);
  config_.lane_following_plugin_name = declare_parameter<std::string>("lane_following_plugin_name",       config_.lane_following_plugin_name);
  config_.intersection_transit_plugin_name = declare_parameter<std::string>("intersection_transit_plugin_name", config_.intersection_transit_plugin_name);
  config_.vehicle_id = declare_parameter<std::string>("vehicle_id", config_.vehicle_id);
  config_.veh_length = declare_parameter<double>("vehicle_length", config_.veh_length);
  config_.vehicle_decel_limit = declare_parameter<double>("vehicle_deceleration_limit", config_.vehicle_decel_limit);
  config_.vehicle_accel_limit = declare_parameter<double>("vehicle_acceleration_limit", config_.vehicle_accel_limit);

};

carma_ros2_utils::CallbackReturn SCIStrategicPlugin::on_configure_plugin()
{
  // reset config
  config_ = SCIStrategicPluginConfig();
  // Declare parameters
  get_parameter<double>("vehicle_decel_limit_multiplier",   config_.vehicle_decel_limit_multiplier);
  get_parameter<double>("vehicle_accel_limit_multiplier",   config_.vehicle_accel_limit_multiplier);
  get_parameter<double>("stop_line_buffer",   config_.stop_line_buffer);
  get_parameter<double>("delta_t",   config_.delta_t);
  get_parameter<double>("min_gap",   config_.min_gap);
  get_parameter<double>("reaction_time",   config_.reaction_time);
  get_parameter<double>("intersection_exit_zone_length",   config_.intersection_exit_zone_length);
  get_parameter<std::string>("strategic_plugin_name",            config_.strategic_plugin_name);
  get_parameter<std::string>("lane_following_plugin_name",       config_.lane_following_plugin_name);
  get_parameter<std::string>("intersection_transit_plugin_name", config_.intersection_transit_plugin_name);
  get_parameter<std::string>("vehicle_id", config_.vehicle_id);
  get_parameter<double>("vehicle_length", config_.veh_length);
  get_parameter<double>("vehicle_deceleration_limit", config_.vehicle_decel_limit);
  get_parameter<double>("vehicle_acceleration_limit", config_.vehicle_accel_limit);
  
   // Register runtime parameter update callback
  add_on_set_parameters_callback(std::bind(&SCIStrategicPlugin::parameter_update_callback, this, std_ph::_1));

  RCLCPP_INFO_STREAM(rclcpp::get_logger("sci_strategic_plugin"),"Done loading parameters: " << config_);

  // Mobility Operation Subscriber
  mob_operation_sub_ = create_subscription<carma_v2x_msgs::msg::MobilityOperation>("incoming_mobility_operation", 1, 
    std::bind(&SCIStrategicPlugin::mobilityOperationCb,this,std_ph::_1));
  
  // Current Pose Subscriber
  current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 1, 
    std::bind(&SCIStrategicPlugin::currentPoseCb,this,std_ph::_1));

  // BSM subscriber
  bsm_sub_ = create_subscription<carma_v2x_msgs::msg::BSM>("bsm_outbound", 1, 
    std::bind(&SCIStrategicPlugin::BSMCb,this,std_ph::_1));

  // Guidance State subscriber
  guidance_state_sub_ = create_subscription<carma_planning_msgs::msg::GuidanceState>("guidance_state", 5, 
    std::bind(&SCIStrategicPlugin::guidance_state_cb, this, std::placeholders::_1));

  // set world model point form wm listener
  wm_ = get_world_model();
  
  // Setup publishers
  mobility_operation_pub_ = create_publisher<carma_v2x_msgs::msg::MobilityOperation>("outgoing_mobility_operation", 1);

  // Return success if everthing initialized successfully
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult SCIStrategicPlugin::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  auto error_double = update_params<double>({
    {"vehicle_decel_limit_multiplier", config_.vehicle_decel_limit_multiplier},
    {"vehicle_accel_limit_multiplier", config_.vehicle_accel_limit_multiplier},
    {"stop_line_buffer", config_.stop_line_buffer},
    {"delta_t", config_.delta_t},
    {"min_gap", config_.min_gap},
    {"reaction_time", config_.reaction_time},
    {"intersection_exit_zone_length", config_.intersection_exit_zone_length}
  }, parameters); // vehicle_acceleration_limit not updated as it's global param

  rcl_interfaces::msg::SetParametersResult result;

  result.successful = !error_double;

  return result;
}

carma_ros2_utils::CallbackReturn SCIStrategicPlugin::on_activate_plugin()
{
  mob_op_pub_timer_ = create_timer(get_clock(),
    std::chrono::duration<double>(0.1),
    std::bind(&SCIStrategicPlugin::publishMobilityOperation, this));

  return CallbackReturn::SUCCESS;
}

SCIStrategicPlugin::VehicleState SCIStrategicPlugin::extractInitialState(const carma_planning_msgs::srv::PlanManeuvers::Request& req) const
{
  VehicleState state;
  if (!req.prior_plan.maneuvers.empty())
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Provided with initial plan...");
    state.stamp = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_time);
    state.speed = getManeuverEndSpeed(req.prior_plan.maneuvers.back());
    state.downtrack = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_dist);
    state.lane_id = getLaneletsBetweenWithException(state.downtrack, state.downtrack, true).front().id();
  }
  else
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "No initial plan provided...");
    
    state.stamp = req.header.stamp;
    state.downtrack = req.veh_downtrack;
    state.speed = req.veh_logitudinal_velocity;
    state.lane_id = stoi(req.veh_lane_id);
  }
  
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "state.stamp: " << std::to_string(state.stamp.seconds()));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "state.downtrack : " << state.downtrack );
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "state.speed: " << state.speed);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "state.lane_id: " << state.lane_id);

  return state;
}

void SCIStrategicPlugin::mobilityOperationCb(carma_v2x_msgs::msg::MobilityOperation::UniquePtr msg)
{
  if (msg->strategy == stop_controlled_intersection_strategy_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Received Schedule message with id: " << msg->m_header.plan_id);
    approaching_stop_controlled_interction_ = true;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Approaching Stop Controlled Intersection: " << approaching_stop_controlled_interction_);

    if (msg->m_header.recipient_id == config_.vehicle_id)
      {
        street_msg_timestamp_ = msg->m_header.timestamp;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "street_msg_timestamp_: " << street_msg_timestamp_);
        parseStrategyParams(msg->strategy_params);
      }
    previous_strategy_params_ = msg->strategy_params;
  }
  
}

void SCIStrategicPlugin::BSMCb(carma_v2x_msgs::msg::BSM::UniquePtr msg)
{
  std::vector<uint8_t> bsm_id_vec = msg->core_data.id;
  bsm_id_ = BSMHelper::BSMHelper::bsmIDtoString(bsm_id_vec);
  bsm_msg_count_ = msg->core_data.msg_count;
  bsm_sec_mark_ = msg->core_data.sec_mark;
}

void SCIStrategicPlugin::guidance_state_cb(const carma_planning_msgs::msg::GuidanceState::UniquePtr msg)
{
  guidance_engaged_ = (msg->state == carma_planning_msgs::msg::GuidanceState::ENGAGED);
}

void SCIStrategicPlugin::currentPoseCb(geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
  geometry_msgs::msg::PoseStamped pose_msg = geometry_msgs::msg::PoseStamped(*msg.get());
  if (vehicle_engaged_)
  {
    lanelet::BasicPoint2d current_loc(pose_msg.pose.position.x, pose_msg.pose.position.y);
    current_downtrack_ = wm_->routeTrackPos(current_loc).downtrack;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Downtrack from current pose: " << current_downtrack_);
  }
  
  
}

void SCIStrategicPlugin::parseStrategyParams(const std::string& strategy_params)
{
  // sample strategy_params: "st:1634067044,et:1634067059, dt:1634067062.3256602,dp:2,,access: 0"
  std::string params = strategy_params;
  std::vector<std::string> inputsParams;
  boost::algorithm::split(inputsParams, params, boost::is_any_of(","));

  std::vector<std::string> st_parsed;
  boost::algorithm::split(st_parsed, inputsParams[0], boost::is_any_of(":"));
  scheduled_stop_time_ = std::stoull(st_parsed[1]);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "scheduled_stop_time_: " << scheduled_stop_time_);

  std::vector<std::string> et_parsed;
  boost::algorithm::split(et_parsed, inputsParams[1], boost::is_any_of(":"));
  scheduled_enter_time_ = std::stoull(et_parsed[1]);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "scheduled_enter_time_: " << scheduled_enter_time_);

  std::vector<std::string> dt_parsed;
  boost::algorithm::split(dt_parsed, inputsParams[2], boost::is_any_of(":"));
  scheduled_depart_time_ = std::stoull(dt_parsed[1]);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "scheduled_depart_time_: " << scheduled_depart_time_);


  std::vector<std::string> dp_parsed;
  boost::algorithm::split(dp_parsed, inputsParams[3], boost::is_any_of(":"));
  scheduled_departure_position_ = std::stoi(dp_parsed[1]);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "scheduled_departure_position_: " << scheduled_departure_position_);

  std::vector<std::string> access_parsed;
  boost::algorithm::split(access_parsed, inputsParams[4], boost::is_any_of(":"));
  int access = std::stoi(access_parsed[1]);
  is_allowed_int_ = (access == 1);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "is_allowed_int_: " << is_allowed_int_);

}

int SCIStrategicPlugin::determine_speed_profile_case(double stop_dist, double current_speed, double schedule_stop_time, double speed_limit)
{
  int case_num = 0;
  double estimated_stop_time = calcEstimatedStopTime(stop_dist, current_speed);
  
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "estimated_stop_time: " << estimated_stop_time);
  if (estimated_stop_time < schedule_stop_time)
  {
    case_num = 3;
  }
  else
  {
    double speed_before_stop = calc_speed_before_decel(estimated_stop_time, stop_dist, current_speed);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "speed_before_stop: " << speed_before_stop);
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

double SCIStrategicPlugin::calc_speed_before_decel(double stop_time, double stop_dist, double current_speed) const
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



void SCIStrategicPlugin::plan_maneuvers_callback(
  std::shared_ptr<rmw_request_id_t> srv_header, 
  carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
  carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "In Maneuver callback...");
  vehicle_engaged_ = true;

  if (!wm_->getRoute())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Could not plan maneuvers as route was not available");
    return;
  }

  if (!approaching_stop_controlled_interction_)
  {
    resp->new_plan.maneuvers = {};
    RCLCPP_WARN_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Not approaching stop-controlled intersection so no maneuvers");
    return;
  }

  bool is_empty_schedule_msg = (scheduled_depart_time_ == 0 && scheduled_stop_time_ == 0 && scheduled_enter_time_ == 0);
  if (is_empty_schedule_msg)
  {
    resp->new_plan.maneuvers = {};
    RCLCPP_WARN_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Receiving empty schedule message");
    return;
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Planning for intersection...");

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Finding car information");
  // Extract vehicle data from request
  VehicleState current_state = extractInitialState(*req);

  // Get current traffic light information
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "\n\nFinding intersection information");

  auto stop_intersection_list = wm_->getIntersectionsAlongRoute({ req->veh_x, req->veh_y });
  bool no_intersections = stop_intersection_list.empty();
  if (no_intersections)
  {
    resp->new_plan.maneuvers = {};
    RCLCPP_WARN_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "There are no stop controlled intersections in the map");
    return;
  }

  auto nearest_stop_intersection = stop_intersection_list.front();

  bool no_stop_lines = nearest_stop_intersection->stopLines().empty();
  if (no_stop_lines)
  {
    resp->new_plan.maneuvers = {};
    RCLCPP_WARN_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "There are no stop lines on the closest stop-controlled intersections in the map");
    return;
  }
  

  // extract the intersection stop line information
  std::vector<double> stop_lines;
  for (auto l : nearest_stop_intersection->stopLines())
  {
    //TODO: temp use veh_length here until planning stack is updated with front bumper pos
    double stop_bar_dtd = wm_->routeTrackPos(l.front().basicPoint2d()).downtrack - config_.veh_length;
    stop_lines.push_back(stop_bar_dtd);
  }
  std::sort(stop_lines.begin(), stop_lines.end());
  
  double stop_intersection_down_track = stop_lines.front();

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "stop_intersection_down_track  " << stop_intersection_down_track);
  
  
  double distance_to_stopline = stop_intersection_down_track - current_downtrack_ - config_.stop_line_buffer;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "distance_to_stopline  " << distance_to_stopline);

  if (distance_to_stopline < -config_.intersection_exit_zone_length)
  {
    resp->new_plan.maneuvers = {};
    RCLCPP_WARN_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Already passed intersection, sending empty maneuvers");
    return;
  }


  double intersection_end_downtrack = stop_lines.back();
  // Identify the lanelets of the intersection
  std::vector<lanelet::ConstLanelet> intersection_lanelets =
          getLaneletsBetweenWithException(stop_intersection_down_track, intersection_end_downtrack, true, true);

  // find the turn direction at intersection:

  intersection_turn_direction_ = getTurnDirectionAtIntersection(intersection_lanelets);

  uint32_t base_time = street_msg_timestamp_;
  
  bool time_to_approach_int = int((scheduled_stop_time_) - (street_msg_timestamp_))>0;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "time_to_approach_int  " << time_to_approach_int);
  
  carma_planning_msgs::msg::Maneuver maneuver_planned;

  
  if (time_to_approach_int)
  {   
      auto tmp = (scheduled_stop_time_) - (street_msg_timestamp_);
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "tmp  " << tmp);
      double time_to_schedule_stop = (tmp)/1000.0;
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "time_to_schedule_stop  " << time_to_schedule_stop);
      // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
      std::vector<lanelet::ConstLanelet> crossed_lanelets =
          getLaneletsBetweenWithException(current_downtrack_, stop_intersection_down_track, true, true);

      // approaching stop line
      speed_limit_ = findSpeedLimit(crossed_lanelets.front());

      // lane following to intersection
      
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "time_to_schedule_stop  " << time_to_schedule_stop);

      double desired_deceleration = config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;


      double safe_distance = pow(current_state.speed, 2)/(2*desired_deceleration);
       RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "safe_distance:  " << safe_distance);

      if (distance_to_stopline - safe_distance > config_.stop_line_buffer)
      {
        int case_num = determine_speed_profile_case(distance_to_stopline , current_state.speed, time_to_schedule_stop, speed_limit_);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "case_num:  " << case_num);

        if (case_num < 3)
        {
          maneuver_planned = composeLaneFollowingManeuverMessage(
          case_num, current_state.downtrack, stop_intersection_down_track, current_state.speed, 0.0,
          current_state.stamp, time_to_schedule_stop,
          lanelet::utils::transform(crossed_lanelets, [](const auto& ll) { return ll.id(); }));
          resp->new_plan.maneuvers.push_back(maneuver_planned);
        }
        else
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Decelerating to stop at the intersection");

          // at the stop line or decelerating to stop line
          // stop and wait maneuver
        
          std::vector<lanelet::ConstLanelet> crossed_lanelets =
              getLaneletsBetweenWithException(current_downtrack_, stop_intersection_down_track, true, true);
          

          double stopping_accel = caseThreeSpeedProfile(distance_to_stopline, current_state.speed, time_to_schedule_stop);
          stopping_accel = std::max(-stopping_accel, desired_deceleration);
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "used deceleration for case three: " << stopping_accel);

          maneuver_planned = composeStopAndWaitManeuverMessage(
          current_state.downtrack, stop_intersection_down_track, current_state.speed, crossed_lanelets[0].id(),
          crossed_lanelets[0].id(), stopping_accel, current_state.stamp,
          current_state.stamp + rclcpp::Duration(time_to_schedule_stop*1e9));

          resp->new_plan.maneuvers.push_back(maneuver_planned);

        }
      }
      else
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Too close to the intersection, constant deceleration to stop");

        maneuver_planned = composeStopAndWaitManeuverMessage(
          current_state.downtrack, stop_intersection_down_track, current_state.speed, crossed_lanelets[0].id(),
          crossed_lanelets[0].id(), desired_deceleration, current_state.stamp,
          current_state.stamp + rclcpp::Duration(time_to_schedule_stop*1e9));
      }
  }

  bool time_to_reach_stopline = int((street_msg_timestamp_) - (scheduled_stop_time_)) >= 0;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "time_to_reach_stopline: " << time_to_reach_stopline);
  bool not_time_to_intersection_traverse = int((street_msg_timestamp_) - (scheduled_enter_time_)) < 0;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "not_time_to_intersection_traverse: " << not_time_to_intersection_traverse);
  bool time_to_stop_at_stopline = (time_to_reach_stopline && not_time_to_intersection_traverse);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "time_to_stop_at_stopline: " << time_to_stop_at_stopline);

  if (time_to_stop_at_stopline)
  {
    base_time = std::max(scheduled_stop_time_, street_msg_timestamp_);
    double stop_duration = int(scheduled_enter_time_ - base_time)/1000;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "stop_duration:  " << stop_duration);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Planning stop and wait maneuver");
    double stopping_accel = config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
    auto stop_line_lanelet = nearest_stop_intersection->lanelets().front();

    maneuver_planned = composeStopAndWaitManeuverMessage(
          current_state.downtrack, stop_intersection_down_track, current_state.speed, stop_line_lanelet.id(),
          stop_line_lanelet.id(), stopping_accel, current_state.stamp,
          current_state.stamp + rclcpp::Duration(stop_duration*1e9));
  }

  if (!is_allowed_int_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Vehicle at stop line, waiting for access to intersection");
    // vehicle is at the intersection but it is not allowed access, so it must wait
    // arbitrary parameters
    double stop_acc = config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
    double stop_duration = 999;//std::numeric_limits<double>::max();

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "current_downtrack_: " << current_downtrack_);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "current state dtd: " << current_state.downtrack);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "stop_intersection_down_track dtd: " << stop_intersection_down_track);

      maneuver_planned = composeStopAndWaitManeuverMessage(
          current_state.downtrack, stop_intersection_down_track, current_state.speed, current_state.lane_id,
          current_state.lane_id, stop_acc, current_state.stamp,
          current_state.stamp + rclcpp::Duration(stop_duration*1e9));

  }

  bool time_for_crossing = int((street_msg_timestamp_) - (scheduled_enter_time_)) >= 0;
  // time to cross intersection

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "time for crossing? " << time_for_crossing);

  if (time_for_crossing && is_allowed_int_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "street_msg_timestamp_ - scheduled_enter_time_ = " << street_msg_timestamp_ - scheduled_enter_time_);


    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Vehicle is crossing the intersection");
    // Passing the stop line
    
    // Compose intersection transit maneuver
    base_time = std::max(scheduled_enter_time_, street_msg_timestamp_);
    double intersection_transit_time = (scheduled_depart_time_ - base_time)/1000;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "intersection_transit_time: " << intersection_transit_time);
    
    intersection_transit_time = config_.min_maneuver_planning_period;//std::max(intersection_transit_time, config_.min_maneuver_planning_period);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "used intersection_transit_time: " << intersection_transit_time);
    // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
    std::vector<lanelet::ConstLanelet> crossed_lanelets =
          getLaneletsBetweenWithException(current_downtrack_, intersection_end_downtrack, true, true);

    
    // find the turn direction at intersection:

    intersection_turn_direction_ = getTurnDirectionAtIntersection(crossed_lanelets);
    
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "turn direction at the intersection is: " << intersection_turn_direction_);

    double intersection_speed_limit = findSpeedLimit(nearest_stop_intersection->lanelets().front());

    // when passing intersection, set the flag to false
    double end_of_intersection = std::max(config_.intersection_exit_zone_length, intersection_end_downtrack - stop_intersection_down_track);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Actual length of intersection: " << intersection_end_downtrack - stop_intersection_down_track);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Used length of intersection: " << end_of_intersection);

      maneuver_planned = composeIntersectionTransitMessage(
      current_state.downtrack, current_state.downtrack + end_of_intersection, current_state.speed, intersection_speed_limit,
      current_state.stamp, rclcpp::Time(req->header.stamp) + rclcpp::Duration(intersection_transit_time*1e9), intersection_turn_direction_, crossed_lanelets.front().id(), crossed_lanelets.back().id());
    
    
    if (distance_to_stopline < -end_of_intersection)
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Vehicle is out of intersection, stop planning...");
      // once the vehicle crosses the intersection, reset the flag to stop planning and publishing status/intent
      approaching_stop_controlled_interction_ = false;
      // once the intersection is crossed, reset turn direction
      intersection_turn_direction_ = TurnDirection::Straight;
    }

  }
  resp->new_plan.maneuvers.push_back(maneuver_planned);
  
  return;
}

TurnDirection SCIStrategicPlugin::getTurnDirectionAtIntersection(std::vector<lanelet::ConstLanelet> lanelets_list)
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
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "intersection crossed lanelet direction is: " << turn_direction);
    }
    else
    {
      // if there is no attribute, assumption is straight
      turn_direction = TurnDirection::Straight;
    }

  }
  return turn_direction;
}

void SCIStrategicPlugin::caseOneSpeedProfile(double speed_before_decel, double current_speed, double stop_time, 
                                            std::vector<double>* float_metadata_list) const
{
  double desired_acceleration = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  double desired_deceleration = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;  

  // Equations obtained from TSMO UC 1 Algorithm draft doc
  double a_acc = ((1 - desired_acceleration/desired_deceleration)*speed_before_decel - current_speed)/stop_time;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Case one a_acc: " << a_acc);
  a_acc = std::min(a_acc, desired_acceleration);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Used Case one a_acc: " << a_acc);
  
  double a_dec = ((desired_deceleration - desired_acceleration)*speed_before_decel - desired_deceleration * current_speed)/(desired_acceleration * stop_time);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Case one a_dec: " << a_dec);
  a_dec = std::max(a_dec, desired_deceleration);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Used Case one a_dec: " << a_dec);

  double t_acc = (speed_before_decel - current_speed)/a_acc;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Case one t_acc: " << t_acc);
  double t_dec = -speed_before_decel/a_dec; // a_dec is negative so a - is used to make the t_dec positive. 
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Case one t_dec: " << t_dec);
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
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Case two speed_before_decel: " << speed_before_decel);
  }
  
  double t_c_nom = 2*stop_dist * ((1 - desired_acceleration/desired_deceleration)*speed_limit - current_speed) - 
                  stop_time * ((1 - desired_acceleration/desired_deceleration)*pow(speed_limit,2) - pow(current_speed, 2));
  double t_c_den = pow(speed_limit - current_speed, 2) - (desired_acceleration/desired_deceleration) * pow(speed_limit, 2);
  double t_cruise = t_c_nom / t_c_den;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Case two t_cruise: " << t_cruise);

  // Equations obtained from TSMO UC 1 Algorithm draft doc
  double a_acc = ((1 - desired_acceleration/desired_deceleration)*speed_limit - current_speed)/(stop_time - t_cruise);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Case two a_acc: " << a_acc);
  a_acc = std::min(desired_acceleration, std::abs(a_acc));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Used Case two a_acc: " << a_acc);

  double a_dec = ((desired_deceleration - desired_acceleration)*speed_limit - desired_deceleration * current_speed)/(desired_acceleration*(stop_time - t_cruise));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Case two a_dec: " << a_dec);
  a_dec = -1*std::min(desired_deceleration, std::abs(a_dec));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Used Case two a_dec: " << a_dec);

  double t_acc = (speed_limit - current_speed)/a_acc;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Case two t_acc: " << t_acc);
  double t_dec = -speed_limit/a_dec; // a_dec is negative so a - is used to make the t_dec positive. 
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Case two t_dec: " << t_dec);

  float_metadata_list->push_back(a_acc);
  float_metadata_list->push_back(a_dec);
  float_metadata_list->push_back(t_acc);
  float_metadata_list->push_back(t_dec);
  float_metadata_list->push_back(t_cruise);
  float_metadata_list->push_back(speed_before_decel);

}

double SCIStrategicPlugin::caseThreeSpeedProfile(double stop_dist, double current_speed, double stop_time) const
{
  double a_dec = (2*stop_dist - current_speed*(stop_time + config_.delta_t))/(stop_time * config_.delta_t);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Case three a_dec: " << a_dec);
  return a_dec;
}

carma_planning_msgs::msg::Maneuver SCIStrategicPlugin::composeLaneFollowingManeuverMessage(int case_num, double start_dist, double end_dist,
                                                                          double start_speed, double target_speed,
                                                                          rclcpp::Time start_time, double time_to_stop,
                                                                          std::vector<lanelet::Id> lane_ids)
{
  carma_planning_msgs::msg::Maneuver maneuver_msg;
  carma_planning_msgs::msg::Maneuver empty_msg;
  maneuver_msg.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = config_.strategic_plugin_name;
  maneuver_msg.lane_following_maneuver.parameters.presence_vector = 
  carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN | carma_planning_msgs::msg::ManeuverParameters::HAS_INT_META_DATA | 
  carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA | carma_planning_msgs::msg::ManeuverParameters::HAS_STRING_META_DATA;
  maneuver_msg.lane_following_maneuver.parameters.negotiation_type = carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = config_.lane_following_plugin_name;
  maneuver_msg.lane_following_maneuver.start_dist = start_dist;
  maneuver_msg.lane_following_maneuver.end_dist = end_dist;
  maneuver_msg.lane_following_maneuver.start_speed = start_speed;
  maneuver_msg.lane_following_maneuver.end_speed = target_speed;
  maneuver_msg.lane_following_maneuver.start_time = start_time;
  maneuver_msg.lane_following_maneuver.end_time =  start_time + rclcpp::Duration(time_to_stop*1e9);
  maneuver_msg.lane_following_maneuver.lane_ids =
      lanelet::utils::transform(lane_ids, [](auto id) { return std::to_string(id); });

  RCLCPP_INFO_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Creating lane follow start dist: " << start_dist << " end dist: " << end_dist);

  std::vector<double> float_metadata_list;

  double speed_before_decel = calc_speed_before_decel(time_to_stop, end_dist-start_dist, start_speed);

  switch(case_num)
  {
    case 1:
      caseOneSpeedProfile(speed_before_decel, start_speed, time_to_stop, &float_metadata_list);
      break;
    case 2:
      caseTwoSpeedProfile(end_dist-start_dist, speed_before_decel, start_speed, time_to_stop, speed_limit_, &float_metadata_list);
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

carma_v2x_msgs::msg::MobilityOperation SCIStrategicPlugin::generateMobilityOperation()
{
    carma_v2x_msgs::msg::MobilityOperation mo_;
    mo_.m_header.timestamp = now().nanoseconds()/1000000;
    mo_.m_header.sender_id = config_.vehicle_id;
    mo_.m_header.sender_bsm_id = bsm_id_;
    mo_.strategy = stop_controlled_intersection_strategy_;

    double vehicle_acceleration_limit_ = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
    double vehicle_deceleration_limit_ = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;

    std::string intersection_turn_direction = "straight";
    if (intersection_turn_direction_ == TurnDirection::Right) intersection_turn_direction = "right";
    if (intersection_turn_direction_ == TurnDirection::Left) intersection_turn_direction = "left";


    mo_.strategy_params = "access: " +  std::to_string(is_allowed_int_) + ", max_accel: " + std::to_string(vehicle_acceleration_limit_) + 
                        ", max_decel: " + std::to_string(vehicle_deceleration_limit_) + ", react_time: " + std::to_string(config_.reaction_time) +
                        ", min_gap: " + std::to_string(config_.min_gap) + ", depart_pos: " + std::to_string(scheduled_departure_position_) + 
                        ", turn_direction: " + intersection_turn_direction + ", msg_count: " + std::to_string(bsm_msg_count_) + ", sec_mark: " + std::to_string(bsm_sec_mark_);
    

    return mo_;
}

void SCIStrategicPlugin::publishMobilityOperation()
{
  if (approaching_stop_controlled_interction_ && guidance_engaged_)
  {
    carma_v2x_msgs::msg::MobilityOperation status_msg = generateMobilityOperation();
    mobility_operation_pub_->publish(status_msg);
  }
}


carma_planning_msgs::msg::Maneuver SCIStrategicPlugin::composeStopAndWaitManeuverMessage(double current_dist, double end_dist,
                                                                        double start_speed,
                                                                        const lanelet::Id& starting_lane_id,
                                                                        const lanelet::Id& ending_lane_id, double stopping_accel,
                                                                        rclcpp::Time start_time, rclcpp::Time end_time) const
{
  carma_planning_msgs::msg::Maneuver maneuver_msg;
  maneuver_msg.type = carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT;
  maneuver_msg.stop_and_wait_maneuver.parameters.planning_strategic_plugin = config_.strategic_plugin_name;
  maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector =
      carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN | carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA;
  maneuver_msg.stop_and_wait_maneuver.parameters.negotiation_type = carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
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
  maneuver_msg.stop_and_wait_maneuver.parameters.float_valued_meta_data.push_back(stopping_accel);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Creating stop and wait start dist: " << current_dist << " end dist: " << end_dist);
  
  return maneuver_msg;
}

carma_planning_msgs::msg::Maneuver SCIStrategicPlugin::composeIntersectionTransitMessage(double start_dist, double end_dist,
                                                                        double start_speed, double target_speed,
                                                                        rclcpp::Time start_time, rclcpp::Time end_time,
                                                                        TurnDirection turn_direction,
                                                                        const lanelet::Id& starting_lane_id,
                                                                        const lanelet::Id& ending_lane_id) const
{
  carma_planning_msgs::msg::Maneuver maneuver_msg;
  if (turn_direction == TurnDirection::Left)
  {
    maneuver_msg.type = carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN;
    maneuver_msg.intersection_transit_left_turn_maneuver.parameters.planning_strategic_plugin =
        config_.strategic_plugin_name;
    maneuver_msg.intersection_transit_left_turn_maneuver.parameters.planning_tactical_plugin =
        config_.intersection_transit_plugin_name;
    maneuver_msg.intersection_transit_left_turn_maneuver.parameters.presence_vector =
        carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN;
    maneuver_msg.intersection_transit_left_turn_maneuver.parameters.negotiation_type =
        carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
    maneuver_msg.intersection_transit_left_turn_maneuver.start_dist = start_dist;
    maneuver_msg.intersection_transit_left_turn_maneuver.end_dist = end_dist;
    maneuver_msg.intersection_transit_left_turn_maneuver.start_speed = start_speed;
    maneuver_msg.intersection_transit_left_turn_maneuver.end_speed = target_speed;
    maneuver_msg.intersection_transit_left_turn_maneuver.start_time = start_time;
    maneuver_msg.intersection_transit_left_turn_maneuver.end_time = end_time;
    maneuver_msg.intersection_transit_left_turn_maneuver.starting_lane_id = std::to_string(starting_lane_id);
    maneuver_msg.intersection_transit_left_turn_maneuver.ending_lane_id = std::to_string(ending_lane_id);

  }
  else if (turn_direction == TurnDirection::Right)
  {
    maneuver_msg.type = carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN;
    maneuver_msg.intersection_transit_right_turn_maneuver.parameters.planning_strategic_plugin =
        config_.strategic_plugin_name;
    maneuver_msg.intersection_transit_right_turn_maneuver.parameters.planning_tactical_plugin =
        config_.intersection_transit_plugin_name;
    maneuver_msg.intersection_transit_right_turn_maneuver.parameters.presence_vector =
        carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN;
    maneuver_msg.intersection_transit_right_turn_maneuver.parameters.negotiation_type =
        carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
    maneuver_msg.intersection_transit_right_turn_maneuver.start_dist = start_dist;
    maneuver_msg.intersection_transit_right_turn_maneuver.end_dist = end_dist;
    maneuver_msg.intersection_transit_right_turn_maneuver.start_speed = start_speed;
    maneuver_msg.intersection_transit_right_turn_maneuver.end_speed = target_speed;
    maneuver_msg.intersection_transit_right_turn_maneuver.start_time = start_time;
    maneuver_msg.intersection_transit_right_turn_maneuver.end_time = end_time;
    maneuver_msg.intersection_transit_right_turn_maneuver.starting_lane_id = std::to_string(starting_lane_id);
    maneuver_msg.intersection_transit_right_turn_maneuver.ending_lane_id = std::to_string(ending_lane_id);

  }
  else
  {
    maneuver_msg.type = carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT;
    maneuver_msg.intersection_transit_straight_maneuver.parameters.planning_strategic_plugin =
        config_.strategic_plugin_name;
    maneuver_msg.intersection_transit_straight_maneuver.parameters.planning_tactical_plugin =
        config_.intersection_transit_plugin_name;
    maneuver_msg.intersection_transit_straight_maneuver.parameters.presence_vector =
        carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN;
    maneuver_msg.intersection_transit_straight_maneuver.parameters.negotiation_type =
        carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
    maneuver_msg.intersection_transit_straight_maneuver.start_dist = start_dist;
    maneuver_msg.intersection_transit_straight_maneuver.end_dist = end_dist;
    maneuver_msg.intersection_transit_straight_maneuver.start_speed = start_speed;
    maneuver_msg.intersection_transit_straight_maneuver.end_speed = target_speed;
    maneuver_msg.intersection_transit_straight_maneuver.start_time = start_time;
    maneuver_msg.intersection_transit_straight_maneuver.end_time = end_time;
    maneuver_msg.intersection_transit_straight_maneuver.starting_lane_id = std::to_string(starting_lane_id);
    maneuver_msg.intersection_transit_straight_maneuver.ending_lane_id = std::to_string(ending_lane_id);

  }
  
  

  // Start time and end time for maneuver are assigned in updateTimeProgress

  RCLCPP_INFO_STREAM(rclcpp::get_logger("sci_strategic_plugin"), "Creating IntersectionTransitManeuver start dist: " << start_dist << " end dist: " << end_dist
                                                                      << " From lanelet: " << starting_lane_id
                                                                      << " to lanelet: " << ending_lane_id);

  return maneuver_msg;
}

bool SCIStrategicPlugin::get_availability()
{
  return true;
}

std::string SCIStrategicPlugin::get_version_id()
{
  return "v1.0";
}

}  // namespace SCI_strategic_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(sci_strategic_plugin::SCIStrategicPlugin)