/*
 * Copyright (C) 2023 LEIDOS.
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
#include "lci_strategic_plugin/lci_strategic_plugin.hpp"
#include "lci_strategic_plugin/lci_states.hpp"

#define EPSILON 0.01

#define GET_MANEUVER_PROPERTY(mvr, property)                                                                           \
  (((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ?                                                 \
        (mvr).intersection_transit_left_turn_maneuver.property :                                                       \
        ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ?                                           \
             (mvr).intersection_transit_right_turn_maneuver.property :                                                 \
             ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ?                                        \
                  (mvr).intersection_transit_straight_maneuver.property :                                              \
                  ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_CHANGE    ? (mvr).lane_change_maneuver.property :            \
                   (mvr).type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :         \
                                                                      throw new std::invalid_argument("GET_MANEUVER_"  \
                                                                                                      "PROPERTY "      \
                                                                                                      "(property) "    \
                                                                                                      "called on "     \
                                                                                                      "maneuver with " \
                                                                                                      "invalid type "  \
                                                                                                      "id"))))))
   
namespace lci_strategic_plugin
{
  namespace std_ph = std::placeholders;

LCIStrategicPlugin::LCIStrategicPlugin(const rclcpp::NodeOptions &options)
  : carma_guidance_plugins::StrategicPlugin(options), tf2_buffer_(this->get_clock())
{
  config_ = LCIStrategicPluginConfig();

  config_.vehicle_accel_limit = declare_parameter<double>("vehicle_acceleration_limit", config_.vehicle_accel_limit);
  config_.vehicle_decel_limit = declare_parameter<double>("vehicle_deceleration_limit", config_.vehicle_decel_limit);
  config_.vehicle_decel_limit_multiplier = declare_parameter<double>("vehicle_decel_limit_multiplier", config_.vehicle_decel_limit_multiplier);
  config_.vehicle_accel_limit_multiplier = declare_parameter<double>("vehicle_accel_limit_multiplier",   config_.vehicle_accel_limit_multiplier);
  config_.min_approach_distance = declare_parameter<double>("min_approach_distance", config_.min_approach_distance);
  config_.trajectory_smoothing_activation_distance = declare_parameter<double>("trajectory_smoothing_activation_distance", config_.trajectory_smoothing_activation_distance);
  config_.stopping_location_buffer = declare_parameter<double>("stopping_location_buffer", config_.stopping_location_buffer);
  config_.green_light_time_buffer = declare_parameter<double>("green_light_time_buffer", config_.green_light_time_buffer);
  config_.algo_minimum_speed = declare_parameter<double>("algo_minimum_speed", config_.algo_minimum_speed);
  config_.deceleration_fraction = declare_parameter<double>("deceleration_fraction",  config_.deceleration_fraction);
  config_.desired_distance_to_stop_buffer = declare_parameter<double>("desired_distance_to_stop_buffer", config_.desired_distance_to_stop_buffer);
  config_.min_maneuver_planning_period = declare_parameter<double>("min_maneuver_planning_period", config_.min_maneuver_planning_period);
  config_.strategic_plugin_name = declare_parameter<std::string>("strategic_plugin_name",   config_.strategic_plugin_name);
  config_.lane_following_plugin_name = declare_parameter<std::string>("lane_following_plugin_name",  config_.lane_following_plugin_name);
  config_.stop_and_wait_plugin_name = declare_parameter<std::string>("stop_and_wait_plugin_name",  config_.stop_and_wait_plugin_name);
  config_.intersection_transit_plugin_name = declare_parameter<std::string>("intersection_transit_plugin_name", config_.intersection_transit_plugin_name);
  config_.enable_carma_streets_connection = declare_parameter<bool>("enable_carma_streets_connection",config_.enable_carma_streets_connection);
  config_.mobility_rate = declare_parameter<double>("mobility_rate", config_.mobility_rate);
  config_.vehicle_id = declare_parameter<std::string>("vehicle_id", config_.vehicle_id);

  max_comfort_accel_ = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  max_comfort_decel_ = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
  max_comfort_decel_norm_ = config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
  emergency_decel_norm_ = 2 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
};

carma_ros2_utils::CallbackReturn LCIStrategicPlugin::on_configure_plugin()
{
   // reset config
  config_ = LCIStrategicPluginConfig();

  // clang-format off
  get_parameter<double>("vehicle_acceleration_limit", config_.vehicle_accel_limit);
  get_parameter<double>("vehicle_deceleration_limit", config_.vehicle_decel_limit);
  get_parameter<double>("vehicle_decel_limit_multiplier", config_.vehicle_decel_limit_multiplier);
  get_parameter<double>("vehicle_accel_limit_multiplier", config_.vehicle_accel_limit_multiplier);
  get_parameter<double>("min_approach_distance", config_.min_approach_distance);
  get_parameter<double>("trajectory_smoothing_activation_distance", config_.trajectory_smoothing_activation_distance);
  get_parameter<double>("stopping_location_buffer", config_.stopping_location_buffer);
  get_parameter<double>("green_light_time_buffer", config_.green_light_time_buffer);
  get_parameter<double>("algo_minimum_speed", config_.algo_minimum_speed);
  get_parameter<double>("deceleration_fraction", config_.deceleration_fraction);
  get_parameter<double>("desired_distance_to_stop_buffer", config_.desired_distance_to_stop_buffer);
  get_parameter<double>("min_maneuver_planning_period", config_.min_maneuver_planning_period);
  get_parameter<std::string>("strategic_plugin_name", config_.strategic_plugin_name);
  get_parameter<std::string>("lane_following_plugin_name", config_.lane_following_plugin_name);
  get_parameter<std::string>("stop_and_wait_plugin_name", config_.stop_and_wait_plugin_name);
  get_parameter<std::string>("intersection_transit_plugin_name", config_.intersection_transit_plugin_name);
  get_parameter<bool>("enable_carma_streets_connection", config_.enable_carma_streets_connection);
  get_parameter<double>("mobility_rate", config_.mobility_rate);
  get_parameter<std::string>("vehicle_id", config_.vehicle_id);

  max_comfort_accel_ = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  max_comfort_decel_ = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
  max_comfort_decel_norm_ = config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
  emergency_decel_norm_ = 2 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;

  // clang-format on

  // Register runtime parameter update callback
  add_on_set_parameters_callback(std::bind(&LCIStrategicPlugin::parameter_update_callback, this, std_ph::_1));

  RCLCPP_INFO_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Done loading parameters: " << config_);

  lookupFrontBumperTransform();

   // Mobility Operation Subscriber
  mob_operation_sub_ = create_subscription<carma_v2x_msgs::msg::MobilityOperation>("incoming_mobility_operation", 1, 
    std::bind(&LCIStrategicPlugin::mobilityOperationCb,this,std_ph::_1));

  // BSM subscriber
  bsm_sub_ = create_subscription<carma_v2x_msgs::msg::BSM>("bsm_outbound", 1, 
    std::bind(&LCIStrategicPlugin::BSMCb,this,std_ph::_1));

  // set world model point form wm listener
  wm_ = get_world_model();
  
  // Setup publishers
  mobility_operation_pub_ = create_publisher<carma_v2x_msgs::msg::MobilityOperation>("outgoing_mobility_operation", 1);
  case_pub_ = create_publisher<std_msgs::msg::Int8>("lci_strategic_plugin/ts_case_num", 1);
  tf_distance_pub_ = create_publisher<std_msgs::msg::Float64>("lci_strategic_plugin/distance_remaining_to_tf", 1);
  earliest_et_pub_ = create_publisher<std_msgs::msg::Float64>("lci_strategic_plugin/earliest_entry_time", 1);
  et_pub_ = create_publisher<std_msgs::msg::Float64>("lci_strategic_plugin/scheduled_entry_time", 1);

  // Return success if everything initialized successfully
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult LCIStrategicPlugin::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  auto error_double = update_params<double>({
    {"vehicle_acceleration_limit", config_.vehicle_accel_limit},
    {"vehicle_deceleration_limit", config_.vehicle_decel_limit},
    {"vehicle_decel_limit_multiplier", config_.vehicle_decel_limit_multiplier},
    {"vehicle_accel_limit_multiplier", config_.vehicle_accel_limit_multiplier},
    {"min_approach_distance", config_.min_approach_distance},
    {"trajectory_smoothing_activation_distance", config_.trajectory_smoothing_activation_distance},
    {"stopping_location_buffer", config_.stopping_location_buffer},
    {"green_light_time_buffer", config_.green_light_time_buffer},
    {"algo_minimum_speed", config_.algo_minimum_speed},
    {"deceleration_fraction", config_.deceleration_fraction},
    {"desired_distance_to_stop_buffer", config_.desired_distance_to_stop_buffer},
    {"min_maneuver_planning_period", config_.min_maneuver_planning_period},
    {"mobility_rate", config_.mobility_rate},
  }, parameters);

  rcl_interfaces::msg::SetParametersResult result;

  result.successful = !error_double;

  return result;
}

void LCIStrategicPlugin::publishTrajectorySmoothingInfo()
{
  std_msgs::msg::Int8 case_num_msg;
  std_msgs::msg::Float64 tf_distance;
  std_msgs::msg::Float64 earliest_et;
  std_msgs::msg::Float64 scheduled_et;

  case_num_msg.data = static_cast<int>(last_case_num_);
  tf_distance.data = distance_remaining_to_tf_;
  earliest_et.data = earliest_entry_time_;
  scheduled_et.data = scheduled_entry_time_;

  case_pub_->publish(case_num_msg);
  tf_distance_pub_->publish(tf_distance);
  earliest_et_pub_->publish(earliest_et);
  et_pub_->publish(scheduled_et);
}

carma_ros2_utils::CallbackReturn LCIStrategicPlugin::on_activate_plugin()
{
  mob_op_pub_timer_ = create_timer(get_clock(),
    std::chrono::duration<double>(1/config_.mobility_rate),
    std::bind(&LCIStrategicPlugin::publishMobilityOperation, this));

  ts_info_pub_timer_ = create_timer(get_clock(),
    std::chrono::duration<double>(0.5),
    std::bind(&LCIStrategicPlugin::publishTrajectorySmoothingInfo, this));

  return CallbackReturn::SUCCESS;
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
        geometry_msgs::msg::TransformStamped tf2 = tf2_buffer_.lookupTransform("base_link", "vehicle_front", rclcpp::Time(0), rclcpp::Duration(20.0 * 1e9)); //save to local copy of transform 20 sec timeout
        length_to_front_bumper_ = tf2.transform.translation.x;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "length_to_front_bumper_: " << length_to_front_bumper_);
        
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("lci_strategic_plugin"), ex.what());
    }
}

LCIStrategicPlugin::VehicleState LCIStrategicPlugin::extractInitialState(carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req) const
{
  VehicleState state;
  if (!req->prior_plan.maneuvers.empty())
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Provided with initial plan...");
    state.stamp = GET_MANEUVER_PROPERTY(req->prior_plan.maneuvers.back(), end_time);
    state.downtrack = GET_MANEUVER_PROPERTY(req->prior_plan.maneuvers.back(), end_dist);
    state.speed = GET_MANEUVER_PROPERTY(req->prior_plan.maneuvers.back(), end_speed);
    state.lane_id = getLaneletsBetweenWithException(state.downtrack, state.downtrack, true).front().id();
  }
  else
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "No initial plan provided...");
    
    state.stamp = rclcpp::Time(req->header.stamp, RCL_SYSTEM_TIME);
    state.downtrack = req->veh_downtrack;
    state.speed = req->veh_logitudinal_velocity;
    state.lane_id = stoi(req->veh_lane_id);
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "extractInitialState >>>> state.stamp: " << std::to_string(state.stamp.seconds()));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "extractInitialState >>>> state.downtrack : " << state.downtrack );
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "extractInitialState >>>> state.speed: " << state.speed);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "extractInitialState >>>> state.lane_id: " << state.lane_id);

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
                                        const rclcpp::Time& source_time) const
{
  if (!optional_state)
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("lci_strategic_plugin"),"Traffic light data not available for source_time " << std::to_string(source_time.seconds()));
    return false;
  }

  lanelet::CarmaTrafficSignalState light_state = optional_state.get().second;

  if (!supportedLightState(light_state))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "LCIStrategic Plugin asked to handle CarmaTrafficSignalState: " << light_state
                                                                                << " which is not supported.");
    return false;
  }

  return true;
}

boost::optional<bool> LCIStrategicPlugin::canArriveAtGreenWithCertainty(const rclcpp::Time& light_arrival_time_by_algo, const lanelet::CarmaTrafficSignalPtr& traffic_light, bool check_late = true, bool check_early = true) const
{
    rclcpp::Time early_arrival_time_by_algo =
        light_arrival_time_by_algo - rclcpp::Duration(config_.green_light_time_buffer * 1e9);

    rclcpp::Time late_arrival_time_by_algo =
        light_arrival_time_by_algo + rclcpp::Duration(config_.green_light_time_buffer * 1e9);

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "light_arrival_time_by_algo: " << std::to_string(light_arrival_time_by_algo.seconds()));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "early_arrival_time_by_algo: " << std::to_string(early_arrival_time_by_algo.seconds()));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "late_arrival_time_by_algo: " << std::to_string(late_arrival_time_by_algo.seconds()));

    auto early_arrival_state_by_algo_optional = traffic_light->predictState(lanelet::time::timeFromSec(early_arrival_time_by_algo.seconds()));

    if (!validLightState(early_arrival_state_by_algo_optional, early_arrival_time_by_algo))
      return boost::none;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "early_arrival_state_by_algo: " << early_arrival_state_by_algo_optional.get().second);

    auto late_arrival_state_by_algo_optional = traffic_light->predictState(lanelet::time::timeFromSec(late_arrival_time_by_algo.seconds()));

    if (!validLightState(late_arrival_state_by_algo_optional, late_arrival_time_by_algo))
      return boost::none; 

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "late_arrival_state_by_algo: " << late_arrival_state_by_algo_optional.get().second);

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

void LCIStrategicPlugin::handleStopping(carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
  carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp, 
                                        const VehicleState& current_state, 
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light,
                                        const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet,
                                        double traffic_light_down_track,
                                        bool is_emergency)
{
  double distance_remaining_to_traffic_light = traffic_light_down_track - current_state.downtrack;

  // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
        getLaneletsBetweenWithException(current_state.downtrack, traffic_light_down_track, true, true);

  double decel_rate =  max_comfort_decel_norm_; // Kinematic |(v_f - v_i) / t = a|

  if (is_emergency)
  {
    decel_rate = emergency_decel_norm_;
    last_case_num_ = TSCase::EMERGENCY_STOPPING;
  }
  else
  {
    last_case_num_ = TSCase::STOPPING;
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "HANDLE_STOPPING: Planning stop and wait maneuver at decel_rate: " << decel_rate);
  
  resp->new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(
    current_state.downtrack, traffic_light_down_track, current_state.speed, crossed_lanelets.front().id(),
    crossed_lanelets.back().id(), current_state.stamp,
    current_state.stamp + rclcpp::Duration(config_.min_maneuver_planning_period * 1e9), decel_rate));
}


void LCIStrategicPlugin::handleFailureCase(carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
  carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp, 
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

  auto incomplete_traj_params = handleFailureCaseHelper(traffic_light, current_state.stamp.seconds(), current_state_speed, intersection_speed_.get(), speed_limit, distance_remaining_to_traffic_light, traffic_light_down_track);

  if (incomplete_traj_params.is_algorithm_successful == false)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Failed to generate maneuver for edge cases...");
    return;
  } 

  resp->new_plan.maneuvers.push_back(composeTrajectorySmoothingManeuverMessage(current_state.downtrack, traffic_light_down_track, crossed_lanelets,
                                          current_state_speed, incomplete_traj_params.modified_departure_speed, current_state.stamp, current_state.stamp + rclcpp::Duration(incomplete_traj_params.modified_remaining_time * 1e9), incomplete_traj_params));

  double intersection_length = intersection_end_downtrack_.get() - traffic_light_down_track;

  rclcpp::Time intersection_exit_time =
      current_state.stamp + rclcpp::Duration(incomplete_traj_params.modified_remaining_time * 1e9) + rclcpp::Duration(intersection_length / incomplete_traj_params.modified_departure_speed * 1e9);

  resp->new_plan.maneuvers.push_back(composeIntersectionTransitMessage(
      traffic_light_down_track, intersection_end_downtrack_.get(), intersection_speed_.get(),
      incomplete_traj_params.modified_departure_speed, current_state.stamp + rclcpp::Duration(incomplete_traj_params.modified_remaining_time * 1e9), intersection_exit_time, crossed_lanelets.back().id(), crossed_lanelets.back().id()));

  last_case_num_ = TSCase::DEGRADED_TSCASE;
}

void LCIStrategicPlugin::handleCruisingUntilStop(carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
  carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp, 
                                        const VehicleState& current_state, 
                                        double current_state_speed,
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light, 
                                        double traffic_light_down_track, const TrajectoryParams& ts_params)
{
  if (!ts_params.is_algorithm_successful || ts_params.case_num != TSCase::CASE_8)
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("lci_strategic_plugin"),"handleCruisingUntilStop is called but it is not case_8");
    return;
  }

  auto new_ts_params = ts_params; 

  double decel_rate = std::fabs(ts_params.a3_);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CASE_8: Planning cruise and stop with decel_rate: " << decel_rate);
  
  new_ts_params.t3_ = new_ts_params.t2_;
  new_ts_params.x3_ = new_ts_params.x2_;
  new_ts_params.v3_ = new_ts_params.v2_;
  new_ts_params.a3_ = new_ts_params.a2_;

  // Identify the lanelets which will be crossed by approach maneuvers stopping part
  std::vector<lanelet::ConstLanelet> lane_follow_crossed_lanelets =
      getLaneletsBetweenWithException(new_ts_params.x1_, new_ts_params.x2_, true, true);

  resp->new_plan.maneuvers.push_back(composeTrajectorySmoothingManeuverMessage(current_state.downtrack, new_ts_params.x2_, lane_follow_crossed_lanelets, 
                                          current_state_speed, new_ts_params.v2_, current_state.stamp, rclcpp::Time(new_ts_params.t2_ * 1e9), new_ts_params));

  // Identify the lanelets which will be crossed by approach maneuvers stopping part
  std::vector<lanelet::ConstLanelet> case_8_crossed_lanelets =
      getLaneletsBetweenWithException(new_ts_params.x2_, traffic_light_down_track, true, true);

  resp->new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(
    new_ts_params.x2_, traffic_light_down_track, new_ts_params.v2_, case_8_crossed_lanelets.front().id(),
    case_8_crossed_lanelets.back().id(), rclcpp::Time(new_ts_params.t2_ * 1e9),
    rclcpp::Time(new_ts_params.t2_ * 1e9) + rclcpp::Duration(config_.min_maneuver_planning_period * 1e9), decel_rate));

  return;
}

void LCIStrategicPlugin::handleGreenSignalScenario(carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
  carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp, 
                                        const VehicleState& current_state, 
                                        double current_state_speed,
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light,
                                        const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet,
                                        double traffic_light_down_track, const TrajectoryParams& ts_params, bool is_certainty_check_optional)
{
  if (!ts_params.is_algorithm_successful || ts_params.case_num == TSCase::CASE_8) 
  {
    return;
  }

  rclcpp::Time light_arrival_time_by_algo = rclcpp::Time(ts_params.t3_ * 1e9);
  double remaining_time = light_arrival_time_by_algo.seconds() - rclcpp::Time(req->header.stamp, RCL_SYSTEM_TIME).seconds();
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Algo initially successful: New light_arrival_time_by_algo: " << std::to_string(light_arrival_time_by_algo.seconds()) << ", with remaining_time: " << std::to_string(remaining_time));
  auto can_make_green_optional = canArriveAtGreenWithCertainty(light_arrival_time_by_algo, traffic_light);

    // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
        getLaneletsBetweenWithException(current_state.downtrack, traffic_light_down_track, true, true);

  // no change for maneuver if invalid light states
  if (!can_make_green_optional) 
    return;
  
  if (can_make_green_optional.get() || is_certainty_check_optional)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "HANDLE_SUCCESSFULL: Algorithm successful, and able to make it at green with certainty. Planning traj smooth and intersection transit maneuvers");
    
    resp->new_plan.maneuvers.push_back(composeTrajectorySmoothingManeuverMessage(current_state.downtrack, traffic_light_down_track, crossed_lanelets,
                                          current_state_speed, ts_params.v3_, current_state.stamp, light_arrival_time_by_algo, ts_params));

    double intersection_length = intersection_end_downtrack_.get() - traffic_light_down_track;

    rclcpp::Time intersection_exit_time =
        light_arrival_time_by_algo + rclcpp::Duration(intersection_length / intersection_speed_.get() * 1e9);

    resp->new_plan.maneuvers.push_back(composeIntersectionTransitMessage(
        traffic_light_down_track, intersection_end_downtrack_.get(), intersection_speed_.get(),
        intersection_speed_.get(), light_arrival_time_by_algo, intersection_exit_time, crossed_lanelets.back().id(), crossed_lanelets.back().id()));
  }
}


TrajectoryParams LCIStrategicPlugin::handleFailureCaseHelper(const lanelet::CarmaTrafficSignalPtr& traffic_light, double current_time, double starting_speed, double departure_speed,  double speed_limit, double remaining_downtrack, double traffic_light_downtrack)
{
  //Requested maneuver needs to be modified to meet remaining_dist req
  //by trying to get close to the target_speed and remaining_time as much as possible
  TrajectoryParams return_params;
  TrajectoryParams traj_upper;
  TrajectoryParams traj_lower;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "HANDLE_LAST_RESORT_CASE: Starting...");
  double starting_downtrack = traffic_light_downtrack - remaining_downtrack;
  double modified_remaining_time_upper; // upper meaning downtrack vs time trajectory is curved upwards
  double modified_remaining_time_lower; // lower meaning downtrack vs time trajectory is curved lower
  double modified_departure_speed_upper;
  double modified_departure_speed_lower;
  bool calculation_success_upper = true; // identifies places in codes where calculation can be invalid such as negative distance

  // the upper ET
  // accel case
  traj_upper.t0_ = current_time;
  traj_upper.v0_ = starting_speed;
  traj_upper.x0_ = starting_downtrack;
  traj_upper.is_algorithm_successful = true;
  traj_upper.case_num = CASE_1;

  if (departure_speed >= starting_speed)
  {
    if ((pow(departure_speed,2) - pow(starting_speed,2))/(2*max_comfort_accel_) >= remaining_downtrack)
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "HandleFailureCase -> Upper Trajectory -> Current Speed <= Desired Departure Speed, Actual Departure Speed < Desired Departure Speed");
      
      modified_departure_speed_upper = sqrt(pow(starting_speed, 2) + (2 * max_comfort_accel_ * remaining_downtrack));
      modified_remaining_time_upper = (modified_departure_speed_upper - starting_speed) / max_comfort_accel_;

      traj_upper.t1_ = current_time + modified_remaining_time_upper;
      traj_upper.v1_ = modified_departure_speed_upper;
      traj_upper.a1_ = max_comfort_accel_;
      traj_upper.x1_ = traffic_light_downtrack;

      traj_upper.t2_ = traj_upper.t1_;
      traj_upper.v2_ = traj_upper.v1_;
      traj_upper.a2_ = traj_upper.a1_;
      traj_upper.x2_ = traj_upper.x1_;

      traj_upper.modified_departure_speed = modified_departure_speed_upper;
      traj_upper.modified_remaining_time = modified_remaining_time_upper;
    }
    else // NOTE: most likely will not happen as it would have happened at trajectory smoothing part
    { 
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "HandleFailureCase -> Upper Trajectory -> Current Speed < Desired Departure Speed, Actual Departure Speed = Desired Departure Speed");
      
      double cruising_distance = remaining_downtrack - (pow(departure_speed, 2) - pow(starting_speed, 2))/ ( 2 * max_comfort_accel_);
      if (cruising_distance < -EPSILON)
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Detected calculation failure in upper case 2");
        calculation_success_upper = false;
      }
      modified_remaining_time_upper = ((departure_speed - starting_speed) / max_comfort_accel_) + (cruising_distance / departure_speed);

      traj_upper.t1_ = current_time + ((departure_speed - starting_speed) / max_comfort_accel_);
      traj_upper.v1_ = departure_speed;
      traj_upper.a1_ = max_comfort_accel_;
      traj_upper.x1_ = starting_downtrack + (pow(departure_speed, 2) - pow(starting_speed, 2)) / (2 * max_comfort_accel_);

      traj_upper.t2_ = current_time + modified_remaining_time_upper;
      traj_upper.v2_ = departure_speed;
      traj_upper.a2_ = 0;
      traj_upper.x2_ = traffic_light_downtrack;

      traj_upper.modified_departure_speed = departure_speed;
      traj_upper.modified_remaining_time = modified_remaining_time_upper;
    }
  }

  if (departure_speed < starting_speed) // NOTE: most cases will not run due to departure speed being equal to speed limit
  {
    if ((pow(departure_speed,2) - pow(starting_speed,2))/(2*max_comfort_decel_) >= remaining_downtrack)
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "HandleFailureCase -> Upper Trajectory -> Current Speed > Desired Departure Speed, Actual Departure Speed > Desired Departure Speed");
      
      modified_departure_speed_upper = sqrt(pow(starting_speed, 2) + (2 * max_comfort_decel_ * remaining_downtrack));
      modified_remaining_time_upper = (modified_departure_speed_upper - starting_speed) / max_comfort_decel_;

      traj_upper.t1_ = current_time + modified_remaining_time_upper; 
      traj_upper.v1_ = modified_departure_speed_upper;
      traj_upper.a1_ = max_comfort_decel_;
      traj_upper.x1_ = traffic_light_downtrack;

      traj_upper.t2_ = traj_upper.t1_;
      traj_upper.v2_ = traj_upper.v1_;
      traj_upper.a2_ = traj_upper.a1_;
      traj_upper.x2_ = traj_upper.x1_;

      traj_upper.modified_departure_speed = modified_departure_speed_upper;
      traj_upper.modified_remaining_time = modified_remaining_time_upper;
    }
    else  // NOTE: most likely will not happen as it would have happened at trajectory smoothing part
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "HandleFailureCase -> Upper Trajectory -> Current Speed > Desired Departure Speed, Actual Departure Speed = Desired Departure Speed");
      
      double cruising_distance = remaining_downtrack - (pow(departure_speed, 2) - pow(starting_speed, 2))/ ( 2 * max_comfort_decel_);

      if (cruising_distance < -EPSILON)
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Detected calculation failure in upper case 4");
        calculation_success_upper = false;
      }
      
      modified_remaining_time_upper = cruising_distance / starting_speed + (departure_speed - starting_speed) / max_comfort_decel_ ;

      traj_upper.t1_ = current_time + cruising_distance / starting_speed;
      traj_upper.v1_ = starting_speed;
      traj_upper.a1_ = 0.0;
      traj_upper.x1_ = starting_downtrack + cruising_distance;

      traj_upper.t2_ = current_time + modified_remaining_time_upper;
      traj_upper.v2_ = departure_speed;
      traj_upper.a2_ = max_comfort_decel_;
      traj_upper.x2_ = traffic_light_downtrack;

      traj_upper.modified_departure_speed = departure_speed;
      traj_upper.modified_remaining_time = modified_remaining_time_upper;
    }
  }
  
  traj_upper.t3_ = traj_upper.t2_;
  traj_upper.v3_ = traj_upper.v2_;
  traj_upper.a3_ = traj_upper.a2_;
  traj_upper.x3_ = traj_upper.x2_;

  // The lower ET 
  // (note that the distance is definitely not enough for deceleration to zero speed, therefore, modified_departure_speed will be greater than zero for sure!).
  modified_departure_speed_lower = sqrt(pow(starting_speed, 2) + (2 * max_comfort_decel_ * remaining_downtrack));
  modified_remaining_time_lower = (modified_departure_speed_lower - starting_speed) / max_comfort_decel_;

  traj_lower.t0_ = current_time;
  traj_lower.v0_ = starting_speed;
  traj_lower.x0_ = starting_downtrack;
  traj_lower.is_algorithm_successful = true;
  traj_lower.case_num = CASE_1;

  traj_lower.t1_ = current_time + modified_remaining_time_lower;
  traj_lower.v1_ = modified_departure_speed_lower;
  traj_lower.a1_ = max_comfort_decel_;
  traj_lower.x1_ = traffic_light_downtrack;

  traj_lower.t2_ = traj_lower.t1_;
  traj_lower.v2_ = traj_lower.v1_;
  traj_lower.a2_ = traj_lower.a1_;
  traj_lower.x2_ = traj_lower.x1_;
  
  traj_lower.t3_ = traj_lower.t2_;
  traj_lower.v3_ = traj_lower.v2_;
  traj_lower.a3_ = traj_lower.a2_;
  traj_lower.x3_ = traj_lower.x2_;

  traj_lower.modified_departure_speed = modified_departure_speed_lower;
  traj_lower.modified_remaining_time = modified_remaining_time_upper;

  // Pick UPPER or LOWER trajectory based on light
  bool is_return_params_found = false;
  
  if (calculation_success_upper)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Checking this time!: " << current_time + modified_remaining_time_upper);

    auto upper_optional = traffic_light->predictState(lanelet::time::timeFromSec(current_time + modified_remaining_time_upper));

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Checking this time! state: " << upper_optional.get().second);

    if (!validLightState(upper_optional, rclcpp::Time((current_time + modified_remaining_time_upper) * 1e9)))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Unable to resolve give signal for modified_remaining_time_upper: " << std::to_string(current_time + modified_remaining_time_upper));
    }
    else
    {
      if (upper_optional.get().second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED)
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Detected Upper GREEN case");    
        return_params = traj_upper;
        is_return_params_found = true;
      }
    }
  }
 
  if (!is_return_params_found)
  {
    auto lower_optional = traffic_light->predictState(lanelet::time::timeFromSec(current_time + modified_remaining_time_lower));

    if (!validLightState(lower_optional, rclcpp::Time((current_time + modified_remaining_time_lower) * 1e9)))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Unable to resolve give signal for modified_remaining_time_lower:" << std::to_string(current_time + modified_remaining_time_lower));
    }
    else
    {
      if (lower_optional.get().second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED)
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Detected Lower GREEN case");    
        return_params = traj_lower;
        is_return_params_found = true;
      }
    }
  }

  // Handle hard failure case such as nan or invalid states
  if (is_return_params_found && !isnan(return_params.modified_departure_speed) && return_params.modified_departure_speed > epsilon_ &&
      return_params.modified_departure_speed < speed_limit ) //80_mph
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Updated the speed, and using modified_departure_speed: " << return_params.modified_departure_speed);
    print_params(return_params);
    return return_params;
  }
  else
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Unable to handle edge case gracefully");
    return_params = TrajectoryParams(); //reset
    return_params.is_algorithm_successful = false;
    return return_params;
  }
}


void LCIStrategicPlugin::planWhenUNAVAILABLE(carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp, const VehicleState& current_state,
                                            const lanelet::CarmaTrafficSignalPtr& traffic_light, const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet)
{
  // Reset intersection state since in this state we are not yet known to be in or approaching an intersection
  intersection_speed_ = boost::none;
  intersection_end_downtrack_ = boost::none;
  double current_state_speed = std::max(current_state.speed, config_.algo_minimum_speed * 1.001);


  if (!traffic_light)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "No lights found along route. Returning maneuver plan unchanged");
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

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "traffic_light_down_track: " << traffic_light_down_track);

  double distance_remaining_to_traffic_light = traffic_light_down_track - current_state.downtrack;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "distance_remaining_to_traffic_light: " << distance_remaining_to_traffic_light <<
                    ", current_state.downtrack: " << current_state.downtrack);

  distance_remaining_to_tf_ = distance_remaining_to_traffic_light; // performance metric

  auto speed_limit = findSpeedLimit(current_lanelet);

  current_state_speed = std::min(speed_limit, current_state_speed);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "speed_limit (free flow speed): " << speed_limit);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "trajectory_smoothing_activation_distance: " << config_.trajectory_smoothing_activation_distance);

  double stopping_dist = estimate_distance_to_stop(current_state_speed, config_.vehicle_decel_limit_multiplier  *
                                                                            config_.vehicle_decel_limit); //accepts positive decel

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Stopping distance: " << stopping_dist);

  double plugin_activation_distance = std::max(stopping_dist, config_.min_approach_distance);

  plugin_activation_distance = std::max(plugin_activation_distance, config_.trajectory_smoothing_activation_distance);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "plugin_activation_distance: " << plugin_activation_distance);

  if (distance_remaining_to_traffic_light <= plugin_activation_distance)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Within intersection range");
    transition_table_.signal(TransitEvent::IN_STOPPING_RANGE);  // Evaluate Signal and Run Trajectory Smoothing Algorithm
  }
  else
  {
    last_case_num_ = TSCase::UNAVAILABLE;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Not within intersection range");
  }

}

void LCIStrategicPlugin::planWhenAPPROACHING(carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp, const VehicleState& current_state,
                                            const lanelet::CarmaTrafficSignalPtr& traffic_light, const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet)
{
  ///////////// 1. Get various required variables //////////////

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

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "traffic_light_down_track: " << traffic_light_down_track);

  double distance_remaining_to_traffic_light = traffic_light_down_track - current_state.downtrack;

  distance_remaining_to_tf_ = distance_remaining_to_traffic_light;

  if (distance_remaining_to_traffic_light < 0) // there is small discrepancy between signal's routeTrackPos as well as current
  {                                            // downtrack calculated using veh_x,y from state. Therefore, it may have crossed it 
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Crossed the bar distance_remaining_to_traffic_light: " << distance_remaining_to_traffic_light);
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

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "current_state_speed: " << current_state_speed);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "intersection_speed_: " << intersection_speed_.get());
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "distance_remaining_to_traffic_light: " << distance_remaining_to_traffic_light);

  intersection_end_downtrack_ =
      wm_->routeTrackPos(exit_lanelet.centerline2d().front().basicPoint2d()).downtrack;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "intersection_end_downtrack_: " << intersection_end_downtrack_.get());

  // If the vehicle is at a stop trigger the stopped state
  constexpr double HALF_MPH_IN_MPS = 0.22352;
  if (current_state.speed < HALF_MPH_IN_MPS &&
      fabs(distance_remaining_to_traffic_light) < config_.stopping_location_buffer)
  {
    transition_table_.signal(TransitEvent::STOPPED);  // The vehicle has come to a stop at the light
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CARMA has detected that the vehicle stopped at the stop bar. Transitioning to WAITING STATE");

    return;
  }
  
  /////////////  2. Start of TSMO UC2 & UC3 Algorithm : ET determination //////////////

  rclcpp::Time earliest_entry_time = current_state.stamp + get_earliest_entry_time(distance_remaining_to_traffic_light, speed_limit, 
                                                  current_state_speed, intersection_speed_.get(), max_comfort_accel_, max_comfort_decel_);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "earliest_entry_time: " << std::to_string(earliest_entry_time.seconds()) << ", with : " << std::to_string((earliest_entry_time - current_state.stamp).seconds())  << " left at: " << std::to_string(current_state.stamp.seconds()));

  auto [nearest_green_entry_time, is_entry_time_within_green_or_tbd, in_tbd] = get_final_entry_time_and_conditions(current_state, earliest_entry_time, traffic_light);

  if (nearest_green_entry_time == rclcpp::Time(0))
    return;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Final nearest_green_entry_time: " << std::to_string(nearest_green_entry_time.seconds()));

  auto et_state = traffic_light->predictState(lanelet::time::timeFromSec(nearest_green_entry_time.seconds()));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Signal at ET: " << et_state.get().second);
  
  /////////////  3. Start of TSMO UC2 & UC3 Algorithm : Trajectory Smoothing CASE SELECTION //////////////

  double remaining_time = nearest_green_entry_time.seconds() - current_state.stamp.seconds();
  double remaining_time_earliest_entry = earliest_entry_time.seconds() - current_state.stamp.seconds();
  scheduled_entry_time_ = remaining_time; // performance metric
  earliest_entry_time_ = remaining_time_earliest_entry; // performance metric
  
  auto boundary_distances = get_delta_x(current_state_speed, intersection_speed_.get(), speed_limit, config_.algo_minimum_speed, max_comfort_accel_, max_comfort_decel_);
  print_boundary_distances(boundary_distances); //debug

  auto boundary_traj_params = get_boundary_traj_params(rclcpp::Time(req->header.stamp, RCL_SYSTEM_TIME).seconds(), current_state_speed, intersection_speed_.get(), speed_limit, config_.algo_minimum_speed, max_comfort_accel_, max_comfort_decel_, current_state.downtrack, traffic_light_down_track, distance_remaining_to_traffic_light, boundary_distances);

  TrajectoryParams ts_params = get_ts_case(rclcpp::Time(req->header.stamp, RCL_SYSTEM_TIME).seconds(), nearest_green_entry_time.seconds(), current_state_speed, intersection_speed_.get(), speed_limit, config_.algo_minimum_speed, max_comfort_accel_, max_comfort_decel_, current_state.downtrack, traffic_light_down_track, distance_remaining_to_traffic_light, boundary_distances, boundary_traj_params);
  print_params(ts_params);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "SPEED PROFILE CASE:" << ts_params.case_num);

  /////////////  4 . Safety Check against traffic signals and Final Maneuver Generation //////////////
  double emergency_distance_to_stop = pow(current_state.speed, 2)/(2 * emergency_decel_norm_) + config_.stopping_location_buffer / 2; //Idea is to aim the middle part of stopping buffer
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "emergency_distance_to_stop at emergency_decel_norm_ (with stopping_location_buffer/2):  " << emergency_distance_to_stop << ", emergency_decel_norm_: " << emergency_decel_norm_);

  double safe_distance_to_stop = pow(current_state.speed, 2)/(2 * max_comfort_decel_norm_) + config_.stopping_location_buffer / 2; //Idea is to aim the middle part of stopping buffer
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "safe_distance_to_stop at max_comfort_decel (with stopping_location_buffer/2):  " << safe_distance_to_stop << ", max_comfort_decel_norm_: " << max_comfort_decel_norm_);

  double desired_distance_to_stop = pow(current_state.speed, 2)/(2 * max_comfort_decel_norm_ * config_.deceleration_fraction) + config_.desired_distance_to_stop_buffer;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "desired_distance_to_stop at: " << desired_distance_to_stop << ", where effective deceleration rate is: " << max_comfort_decel_norm_ * config_.deceleration_fraction);
  
  emergency_distance_to_stop = std::max(emergency_distance_to_stop, config_.stopping_location_buffer);
  safe_distance_to_stop = std::max(safe_distance_to_stop, config_.stopping_location_buffer);
  desired_distance_to_stop = std::max(desired_distance_to_stop, config_.stopping_location_buffer);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "new emergency_distance_to_stop: " << emergency_distance_to_stop);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "new safe_distance_to_stop: " << safe_distance_to_stop);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "new desired_distance_to_stop: " << desired_distance_to_stop);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "distance_remaining_to_traffic_light:  " << distance_remaining_to_traffic_light << ", current_state.speed: " << current_state.speed);

  // Basic RED signal violation check
  if (distance_remaining_to_traffic_light <= emergency_distance_to_stop || last_case_num_ == TSCase::EMERGENCY_STOPPING)
  {
    if (in_tbd) // Given ET is in TBD, but vehicle is too close to intersection
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "ET is still in TBD despite the vehicle being in desired distance to start stopping. Trying to handle this edge case gracefully...");
    }

    double stopping_time = current_state.speed / 1.5 / max_comfort_decel_norm_; //one half the acceleration (twice the acceleration to stop) to account for emergency case, see emergency_decel_norm_

    rclcpp::Time stopping_arrival_time =
          current_state.stamp + rclcpp::Duration(stopping_time * 1e9);

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "stopping_arrival_time: " << std::to_string(stopping_arrival_time.seconds()));

    auto stopping_arrival_state_optional = traffic_light->predictState(lanelet::time::timeFromSec(stopping_arrival_time.seconds()));

    if (!validLightState(stopping_arrival_state_optional, stopping_arrival_time))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Unable to resolve give signal for stopping_arrival_state_optional: " << std::to_string(stopping_arrival_time.seconds()));
      return;
    }

    if (stopping_arrival_state_optional.get().second == lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN || last_case_num_ == TSCase::EMERGENCY_STOPPING) // if once started emergency stopped, keep doing it to avoid jerkiness
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("lci_strategic_plugin"),"Detected possible RED light violation! Stopping!");
      handleStopping(req,resp, current_state, traffic_light, entry_lanelet, exit_lanelet, current_lanelet, traffic_light_down_track, true); //case_11
      last_case_num_ = TSCase::EMERGENCY_STOPPING;

      return;
    }
  }

  // Check if the vehicle can arrive with certainty (Case 1-7)
  if (ts_params.is_algorithm_successful && ts_params.case_num != TSCase::CASE_8 && 
    (distance_remaining_to_traffic_light >= desired_distance_to_stop || !in_tbd) &&
    is_entry_time_within_green_or_tbd) // ET cannot be explicitly inside RED or YELLOW in available future states, which is ERROR case
  {
    handleGreenSignalScenario(req, resp, current_state, current_state_speed, traffic_light, entry_lanelet, exit_lanelet, traffic_light_down_track, ts_params, in_tbd); //in_tbd means optional to check certainty arrival at green
  
    if (!resp->new_plan.maneuvers.empty()) // able to pass at green
    {
      last_case_num_ = ts_params.case_num;
      return;
    }
    
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Not able to make it with certainty: TSCase: " << ts_params.case_num << ", changing it to 8");
    ts_params = boundary_traj_params[7];
    ts_params.is_algorithm_successful = true; //false correspond to cases when vehicle is beyond safe_distance to stop for case8
    ts_params.case_num = CASE_8;
    print_params(ts_params);
  }
  
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Not able to make it with certainty: NEW TSCase: " << ts_params.case_num);
  // if algorithm is NOT successful or if the vehicle cannot make the green light with certainty

  if (desired_distance_to_stop < distance_remaining_to_traffic_light && last_case_num_ != TSCase::STOPPING) // do not switch from STOPPING to case8 again
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Way too early to stop. Cruising at CASE8");
    handleCruisingUntilStop(req, resp, current_state, current_state_speed, traffic_light, traffic_light_down_track, ts_params);
    
    if (!resp->new_plan.maneuvers.empty())
    {
      last_case_num_ = TSCase::CASE_8;
      return;
    }
  }

  if ((safe_distance_to_stop <= distance_remaining_to_traffic_light && desired_distance_to_stop >= distance_remaining_to_traffic_light) || 
    last_case_num_ == TSCase::STOPPING) // if stopping continue stopping until transition to planwhenWAITING
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Planning stopping now. last case:" << static_cast<int>(last_case_num_));
    handleStopping(req,resp, current_state, traffic_light, entry_lanelet, exit_lanelet, current_lanelet, traffic_light_down_track); //case_9
    return;
  }

  if (safe_distance_to_stop > distance_remaining_to_traffic_light)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "No longer within safe distance to stop! Decide to continue stopping or continue into intersection");
    
    if (last_case_num_ != TSCase::STOPPING && last_case_num_ != TSCase::UNAVAILABLE && last_case_num_ != TSCase::CASE_8) //case 1-7 or emergency stop or handlefailure 
    {
      // 3. if not able to stop nor reach target speed at green, attempt its best to reach the target parameters at the intersection
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "HANDLE_LAST_RESORT: The vehicle is not able to stop at red/yellow light nor is able to reach target speed at green. Attempting its best to pass through at green!");
      
      handleFailureCase(req, resp, current_state, current_state_speed, speed_limit, remaining_time, 
                                    exit_lanelet.id(), traffic_light, traffic_light_down_track, ts_params);
      
      if (resp->new_plan.maneuvers.empty())
      {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("lci_strategic_plugin"),"HANDLE_SAFETY: Planning forced slow-down... last case:" << static_cast<int>(last_case_num_));
        handleStopping(req,resp, current_state, traffic_light, entry_lanelet, exit_lanelet, current_lanelet, traffic_light_down_track, true); //case_11 emergency case with twice the normal deceleration
      }
    }
    else
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "HANDLE_SAFETY: Planning to keep stopping now. last case:" << static_cast<int>(last_case_num_));
      handleStopping(req,resp, current_state, traffic_light, entry_lanelet, exit_lanelet, current_lanelet, traffic_light_down_track); //case_9
    }
  }

 
}

void LCIStrategicPlugin::planWhenWAITING(carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req,
                                        carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp, const VehicleState& current_state,
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light, const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet)
{
  last_case_num_ = TSCase::STOPPING;

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

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "traffic_light_down_track: "<<  traffic_light_down_track);

  double entering_time = current_state.stamp.seconds();
  
  auto current_light_state_optional = traffic_light->predictState(lanelet::time::timeFromSec(entering_time));

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "WAITING STATE: requested time to rclcpp::Time(req->header.stamp, RCL_SYSTEM_TIME) check: " << std::to_string(rclcpp::Time(req->header.stamp, RCL_SYSTEM_TIME).seconds()));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "WAITING STATE: requested time to CURRENT STATE check: " << std::to_string(entering_time));
  
  if (!validLightState(current_light_state_optional, rclcpp::Time(entering_time * 1e9)))
    return;

  auto bool_optional_late_certainty = canArriveAtGreenWithCertainty(rclcpp::Time(entering_time * 1e9), traffic_light, true, false);
  
  if (!bool_optional_late_certainty)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Unable to resolve green light...");
    return;
  }

  bool should_enter = true; //uc2
  
  if (config_.enable_carma_streets_connection && entering_time > current_state.stamp.seconds()) //uc3
    should_enter = false;

  if (current_light_state_optional.get().second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED &&
        bool_optional_late_certainty.get() && should_enter) // if can make with certainty
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

  resp->new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(
      current_state.downtrack - stop_maneuver_buffer, traffic_light_down_track, current_state.speed,
      current_state.lane_id, current_state.lane_id, rclcpp::Time(entering_time * 1e9),
      rclcpp::Time(entering_time * 1e9) + rclcpp::Duration(config_.min_maneuver_planning_period * 1e9), stopping_accel));
}

void LCIStrategicPlugin::planWhenDEPARTING(carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req,
                                          carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp, const VehicleState& current_state,
                                          double intersection_end_downtrack, double intersection_speed_limit)
{
  last_case_num_ = TSCase::UNAVAILABLE;
  
  if (current_state.downtrack > intersection_end_downtrack)
  {
    transition_table_.signal(TransitEvent::INTERSECTION_EXIT);  // If we are past the most recent
    return;
  }

  // Calculate exit time assuming constant acceleration
  rclcpp::Time intersection_exit_time =
      current_state.stamp +
      rclcpp::Duration(2.0 * (intersection_end_downtrack - current_state.downtrack) / (current_state.speed + intersection_speed_limit) * 1e9);

  // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
      getLaneletsBetweenWithException(current_state.downtrack, intersection_end_downtrack, true, false);

  // Compose intersection transit maneuver
  resp->new_plan.maneuvers.push_back(composeIntersectionTransitMessage(
      current_state.downtrack, intersection_end_downtrack, current_state.speed, intersection_speed_limit,
      current_state.stamp, intersection_exit_time, crossed_lanelets.back().id(), crossed_lanelets.back().id()));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LCIStrategicPlugin::mobilityOperationCb(carma_v2x_msgs::msg::MobilityOperation::UniquePtr msg)
{
  if (msg->strategy == light_controlled_intersection_strategy_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Received Schedule message with id: " << msg->m_header.plan_id);
    approaching_light_controlled_intersection_ = true;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Approaching light Controlled Intersection: " << approaching_light_controlled_intersection_);
  
    if (msg->m_header.recipient_id == config_.vehicle_id)
    {
      street_msg_timestamp_ = msg->m_header.timestamp;
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "street_msg_timestamp_: " << street_msg_timestamp_);
      parseStrategyParams(msg->strategy_params);
      previous_strategy_params_ = msg->strategy_params;
    }
  }
  
}

void LCIStrategicPlugin::BSMCb(carma_v2x_msgs::msg::BSM::UniquePtr msg)
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
  auto new_scheduled_enter_time = std::stoull(et_parsed[1]);

  if (scheduled_enter_time_ != new_scheduled_enter_time) //reset green buffer cache so it can be re-evaluated
    nearest_green_entry_time_cached_ = boost::none;

  scheduled_enter_time_ = new_scheduled_enter_time;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "scheduled_enter_time_: " << scheduled_enter_time_);

}

carma_v2x_msgs::msg::MobilityOperation LCIStrategicPlugin::generateMobilityOperation()
{
    carma_v2x_msgs::msg::MobilityOperation mo_;
    mo_.m_header.timestamp = now().nanoseconds()/1000000;
    mo_.m_header.sender_id = config_.vehicle_id;
    mo_.m_header.recipient_id = upcoming_id_;
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

void LCIStrategicPlugin::publishMobilityOperation()
{
  if (approaching_light_controlled_intersection_)
  {
    carma_v2x_msgs::msg::MobilityOperation status_msg = generateMobilityOperation();
    mobility_operation_pub_->publish(status_msg);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LCIStrategicPlugin::plan_maneuvers_callback(
  std::shared_ptr<rmw_request_id_t> srv_header, 
  carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
  carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp)
{
  std::chrono::system_clock::time_point execution_start_time = std::chrono::system_clock::now();  // Start timing the execution time for planning so it can be logged
  
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "<<<<<<<<<<<<<<<<< STARTING LCI_STRATEGIC_PLAN!!!!!!!!! >>>>>>>>>>>>>>>>");

  if (!wm_->getRoute())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Could not plan maneuvers as route was not available");
    return;
  }

  if(config_.enable_carma_streets_connection ==true)
  {
  if (!approaching_light_controlled_intersection_)
  {
    resp->new_plan.maneuvers = {};
    RCLCPP_WARN_STREAM(rclcpp::get_logger("lci_strategic_plugin"),"Not approaching light-controlled intersection so no maneuvers");
    return;
  }

  bool is_empty_schedule_msg = (scheduled_enter_time_ == 0);
  if (is_empty_schedule_msg)
  {
    resp->new_plan.maneuvers = {};
    RCLCPP_WARN_STREAM(rclcpp::get_logger("lci_strategic_plugin"),"Receiving empty schedule message");
    return;
  }
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Finding car information");

  // Extract vehicle data from request
  VehicleState current_state = extractInitialState(req);
  if (transition_table_.getState() != TransitState::UNAVAILABLE && !req->prior_plan.maneuvers.empty())
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("lci_strategic_plugin"),"State is NOT UNAVAILABLE AND Maneuvers in request is NOT empty");
    return;
  }
  // Get current traffic light information
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "\n\nFinding traffic_light information");

  auto inter_list = wm_->getSignalizedIntersectionsAlongRoute({ req->veh_x, req->veh_y });
  auto upcoming_id_= inter_list.front()->id();

  auto traffic_list = wm_->getSignalsAlongRoute({ req->veh_x, req->veh_y });

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Found traffic lights of size: " << traffic_list.size());

  auto current_lanelets = wm_->getLaneletsFromPoint({ req->veh_x, req->veh_y});
  lanelet::ConstLanelet current_lanelet;
  
  if (current_lanelets.empty())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Given vehicle position is not on the road! Returning...");
    return;
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
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Did not find entry.exit along the route");
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
    resp->new_plan = carma_planning_msgs::msg::ManeuverPlan();

    prev_state = transition_table_.getState();  // Cache previous state to check if state has changed after 1 iteration

    RCLCPP_INFO_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Planning in state: " << transition_table_.getState());

    boost::optional<std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>> current_light_state_optional = boost::none;
    if (nearest_traffic_signal)
    {
      current_light_state_optional = nearest_traffic_signal->predictState(lanelet::time::timeFromSec(current_state.stamp.seconds()));
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Real-time current signal: " << current_light_state_optional.get().second << ", for Id: " << nearest_traffic_signal->id());
      if (current_light_state_optional.get().second == lanelet::CarmaTrafficSignalState::UNAVAILABLE || !current_light_state_optional)
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Signal state not available returning..." );
        return;
      }      
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

  std::chrono::system_clock::time_point execution_end_time = std::chrono::system_clock::now();  // Planning complete

  auto execution_duration = execution_end_time - execution_start_time;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "ExecutionTime lci_strategic_plugin: " << std::chrono::duration<double>(execution_duration).count());

  return;
  // We need to evaluate the events so the state transitions can be triggered
}

carma_planning_msgs::msg::Maneuver LCIStrategicPlugin::composeTrajectorySmoothingManeuverMessage(double start_dist, double end_dist, const std::vector<lanelet::ConstLanelet>& crossed_lanelets, double start_speed,
                                                       double target_speed, rclcpp::Time start_time, rclcpp::Time end_time,
                                                       const TrajectoryParams& tsp) const
{
  carma_planning_msgs::msg::Maneuver maneuver_msg;
  maneuver_msg.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  maneuver_msg.lane_following_maneuver.parameters.negotiation_type =
      carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.lane_following_maneuver.parameters.presence_vector =
      carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN | carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA | carma_planning_msgs::msg::ManeuverParameters::HAS_INT_META_DATA;
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

  RCLCPP_INFO_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Creating TrajectorySmoothingManeuver start dist: " << start_dist << " end dist: " << end_dist
                                                                      << " start_time: " << std::to_string(start_time.seconds())
                                                                      << " end_time: " << std::to_string(end_time.seconds()));

  return maneuver_msg;
}

carma_planning_msgs::msg::Maneuver LCIStrategicPlugin::composeStopAndWaitManeuverMessage(double current_dist, double end_dist,
                                                                        double start_speed,
                                                                        const lanelet::Id& starting_lane_id,
                                                                        const lanelet::Id& ending_lane_id,
                                                                        rclcpp::Time start_time, rclcpp::Time end_time, double stopping_accel) const
{
  carma_planning_msgs::msg::Maneuver maneuver_msg;
  maneuver_msg.type = carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT;
  maneuver_msg.stop_and_wait_maneuver.parameters.negotiation_type = carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector =
      carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN | carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA;
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

  RCLCPP_INFO_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Creating stop and wait start dist: " << current_dist << " end dist: " << end_dist
                                                                        << " start_time: " << std::to_string(start_time.seconds())
                                                                        << " end_time: " << std::to_string(end_time.seconds())
                                                                        << " stopping_accel: " << stopping_accel);

  return maneuver_msg;
}

carma_planning_msgs::msg::Maneuver LCIStrategicPlugin::composeIntersectionTransitMessage(double start_dist, double end_dist,
                                                                        double start_speed, double target_speed,
                                                                        rclcpp::Time start_time, rclcpp::Time end_time,
                                                                        const lanelet::Id& starting_lane_id,
                                                                        const lanelet::Id& ending_lane_id) const
{
  carma_planning_msgs::msg::Maneuver maneuver_msg;
  maneuver_msg.type = carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.negotiation_type =
      carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.presence_vector =
      carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN;
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

  RCLCPP_INFO_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Creating IntersectionTransitManeuver start dist: " << start_dist << " end dist: " << end_dist
                                                                      << " From lanelet: " << starting_lane_id
                                                                      << " to lanelet: " << ending_lane_id
                                                                      << " From start_time: " << std::to_string(start_time.seconds())
                                                                      << " to end_time: " << std::to_string(end_time.seconds()));

  return maneuver_msg;
}

bool LCIStrategicPlugin::get_availability()
{
  return true;
}

std::string LCIStrategicPlugin::get_version_id()
{
  return "v1.0";
}

}  // namespace lci_strategic_plugin


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(lci_strategic_plugin::LCIStrategicPlugin)
