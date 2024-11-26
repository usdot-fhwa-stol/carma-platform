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
#include "stop_and_dwell_strategic_plugin.hpp"

#define GET_MANEUVER_PROPERTY(mvr, property)                                                                           \
                  ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_CHANGE    ? (mvr).lane_change_maneuver.property :            \
                  ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :         \
                  ((mvr).type == carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT ? (mvr).stop_and_wait_maneuver.property :           \
                                                                      throw new std::invalid_argument("GET_MANEUVER_"  \
                                                                                                      "PROPERTY "      \
                                                                                                      "(property) "    \
                                                                                                      "called on "     \
                                                                                                      "maneuver with " \
                                                                                                      "invalid type "  \
                                                                                                      "id"))))


namespace stop_and_dwell_strategic_plugin
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
      case carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT:
          return 0;
      default:
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("stop_and_dwell_strategic_plugin"), "Requested end speed from unsupported maneuver type");
          return 0;
    }
  }
} // namespace anonymous

StopAndDwellStrategicPlugin::StopAndDwellStrategicPlugin(const rclcpp::NodeOptions &options)
  : carma_guidance_plugins::StrategicPlugin(options), config_(StopAndDwellStrategicPluginConfig())
{
  // Declare parameters
  config_.vehicle_decel_limit_multiplier = declare_parameter<double>("vehicle_decel_limit_multiplier",   config_.vehicle_decel_limit_multiplier);
  config_.vehicle_accel_limit_multiplier = declare_parameter<double>("vehicle_accel_limit_multiplier",   config_.vehicle_accel_limit_multiplier);
  config_.stop_line_buffer = declare_parameter<double>("stop_line_buffer",   config_.stop_line_buffer);
  config_.bus_line_exit_zone_length = declare_parameter<double>("bus_line_exit_zone_length",   config_.bus_line_exit_zone_length);
  config_.strategic_plugin_name = declare_parameter<std::string>("strategic_plugin_name",            config_.strategic_plugin_name);
  config_.lane_following_plugin_name = declare_parameter<std::string>("lane_following_plugin_name",       config_.lane_following_plugin_name);
  config_.vehicle_id = declare_parameter<std::string>("vehicle_id", config_.vehicle_id);
  config_.veh_length = declare_parameter<double>("vehicle_length", config_.veh_length);
  config_.vehicle_decel_limit = declare_parameter<double>("vehicle_deceleration_limit", config_.vehicle_decel_limit);
  config_.vehicle_accel_limit = declare_parameter<double>("vehicle_acceleration_limit", config_.vehicle_accel_limit);
  config_.activation_distance = declare_parameter<double>("activation_distance", config_.activation_distance);
  config_.dwell_time = declare_parameter<double>("dwell_time", config_.dwell_time);
  config_.deceleration_fraction = declare_parameter<double>("deceleration_fraction", config_.deceleration_fraction);
  config_.desired_distance_to_stop_buffer = declare_parameter<double>("desired_distance_to_stop_buffer", config_.desired_distance_to_stop_buffer);

  max_comfort_accel_ = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  max_comfort_decel_ = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
  max_comfort_decel_norm_ = config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
};

carma_ros2_utils::CallbackReturn StopAndDwellStrategicPlugin::on_configure_plugin()
{
  // reset config
  config_ = StopAndDwellStrategicPluginConfig();
  // Declare parameters
  get_parameter<double>("vehicle_decel_limit_multiplier",   config_.vehicle_decel_limit_multiplier);
  get_parameter<double>("vehicle_accel_limit_multiplier",   config_.vehicle_accel_limit_multiplier);
  get_parameter<double>("stop_line_buffer",   config_.stop_line_buffer);
  get_parameter<double>("bus_line_exit_zone_length",   config_.bus_line_exit_zone_length);
  get_parameter<std::string>("strategic_plugin_name",            config_.strategic_plugin_name);
  get_parameter<std::string>("lane_following_plugin_name",       config_.lane_following_plugin_name);
  get_parameter<std::string>("vehicle_id", config_.vehicle_id);
  get_parameter<double>("vehicle_length", config_.veh_length);
  get_parameter<double>("vehicle_deceleration_limit", config_.vehicle_decel_limit);
  get_parameter<double>("vehicle_acceleration_limit", config_.vehicle_accel_limit);
  get_parameter<double>("activation_distance", config_.activation_distance);
  get_parameter<double>("dwell_time", config_.dwell_time);
  get_parameter<double>("deceleration_fraction", config_.deceleration_fraction);
  get_parameter<double>("desired_distance_to_stop_buffer", config_.desired_distance_to_stop_buffer);

  max_comfort_accel_ = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  max_comfort_decel_ = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
  max_comfort_decel_norm_ = config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;

   // Register runtime parameter update callback
  add_on_set_parameters_callback(std::bind(&StopAndDwellStrategicPlugin::parameter_update_callback, this, std_ph::_1));

  RCLCPP_INFO_STREAM(rclcpp::get_logger("stop_and_dwell_strategic_plugin"),"Done loading parameters: " << config_);

  // Current Pose Subscriber
  current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 1, 
    std::bind(&StopAndDwellStrategicPlugin::currentPoseCb,this,std_ph::_1));

  // Guidance State subscriber
  guidance_state_sub_ = create_subscription<carma_planning_msgs::msg::GuidanceState>("guidance_state", 5, 
    std::bind(&StopAndDwellStrategicPlugin::guidance_state_cb, this, std::placeholders::_1));

  // set world model point form wm listener
  wm_ = get_world_model();

  // Return success if everthing initialized successfully
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult StopAndDwellStrategicPlugin::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  auto error_double = update_params<double>({
    {"vehicle_decel_limit_multiplier", config_.vehicle_decel_limit_multiplier},
    {"vehicle_accel_limit_multiplier", config_.vehicle_accel_limit_multiplier},
    {"stop_line_buffer", config_.stop_line_buffer},
    {"bus_line_exit_zone_length", config_.bus_line_exit_zone_length}
  }, parameters); // vehicle_acceleration_limit not updated as it's global param

  rcl_interfaces::msg::SetParametersResult result;

  result.successful = !error_double;

  return result;
}

carma_ros2_utils::CallbackReturn StopAndDwellStrategicPlugin::on_activate_plugin()
{
  return CallbackReturn::SUCCESS;
}

VehicleState StopAndDwellStrategicPlugin::extractInitialState(const carma_planning_msgs::srv::PlanManeuvers::Request& req) const
{
  VehicleState state;
  if (!req.prior_plan.maneuvers.empty())
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("stop_and_dwell_strategic_plugin"), "Provided with initial plan...");
    state.stamp = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_time);
    state.speed = getManeuverEndSpeed(req.prior_plan.maneuvers.back());
    state.downtrack = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_dist);
    state.lane_id = getLaneletsBetweenWithException(state.downtrack, state.downtrack, true).front().id();
  }
  else
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("stop_and_dwell_strategic_plugin"), "No initial plan provided...");
    
    state.stamp = req.header.stamp;
    state.downtrack = req.veh_downtrack;
    state.speed = req.veh_logitudinal_velocity;
    state.lane_id = stoi(req.veh_lane_id);
  }
  
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("stop_and_dwell_strategic_plugin"), "state.stamp: " << std::to_string(state.stamp.seconds()));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("stop_and_dwell_strategic_plugin"), "state.downtrack : " << state.downtrack );
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("stop_and_dwell_strategic_plugin"), "state.speed: " << state.speed);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("stop_and_dwell_strategic_plugin"), "state.lane_id: " << state.lane_id);

  return state;
}

void StopAndDwellStrategicPlugin::guidance_state_cb(const carma_planning_msgs::msg::GuidanceState::UniquePtr msg)
{
  guidance_engaged_ = (msg->state == carma_planning_msgs::msg::GuidanceState::ENGAGED);
}

void StopAndDwellStrategicPlugin::currentPoseCb(geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
  geometry_msgs::msg::PoseStamped pose_msg = geometry_msgs::msg::PoseStamped(*msg.get());
  if (vehicle_engaged_)
  {
    lanelet::BasicPoint2d current_loc(pose_msg.pose.position.x, pose_msg.pose.position.y);
    current_downtrack_ = wm_->routeTrackPos(current_loc).downtrack;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("stop_and_dwell_strategic_plugin"), "Downtrack from current pose: " << current_downtrack_);
  }
  
}

std::vector<lanelet::ConstLanelet> StopAndDwellStrategicPlugin::getLaneletsBetweenWithException(double start_downtrack,
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

VehicleState StopAndDwellStrategicPlugin::extractInitialState(carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req) const
{
  VehicleState state;
  if (!req->prior_plan.maneuvers.empty())
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "Provided with initial plan...");
    state.stamp = GET_MANEUVER_PROPERTY(req->prior_plan.maneuvers.back(), end_time);
    state.downtrack = GET_MANEUVER_PROPERTY(req->prior_plan.maneuvers.back(), end_dist);
    state.speed = getManeuverEndSpeed(req->prior_plan.maneuvers.back());
    state.lane_id = getLaneletsBetweenWithException(state.downtrack, state.downtrack, true).front().id();
  }
  else
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "No initial plan provided...");
    
    state.stamp = rclcpp::Time(req->header.stamp, RCL_SYSTEM_TIME);
    state.downtrack = req->veh_downtrack;
    state.speed = req->veh_logitudinal_velocity;
    state.lane_id = stoi(req->veh_lane_id);
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "extractInitialState >>>> state.stamp: " << std::to_string(state.stamp.seconds()));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "extractInitialState >>>> state.downtrack : " << state.downtrack );
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "extractInitialState >>>> state.speed: " << state.speed);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "extractInitialState >>>> state.lane_id: " << state.lane_id);

  return state;
}

void StopAndDwellStrategicPlugin::plan_maneuvers_callback(
  std::shared_ptr<rmw_request_id_t> srv_header, 
  carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
  carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp)
{
  std::chrono::system_clock::time_point execution_start_time = std::chrono::system_clock::now();  // Start timing the execution time for planning so it can be logged
  
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "<<<<<<<<<<<<<<<<< STARTING STOP_AND_DWELL_STRATEGIC_PLUIGN!!!!!!!!! >>>>>>>>>>>>>>>>");

  if (!wm_->getRoute())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "Could not plan maneuvers as route was not available");
    return;
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "Finding car information");

  // Extract vehicle data from request
  VehicleState current_state = extractInitialState(req);

  auto bus_stop_list = wm_->getBusStopsAlongRoute({ req->veh_x, req->veh_y });

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "Found bus stops of size: " << bus_stop_list .size());

  if(bus_stop_list.empty())
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "Bus stops list is empty");
    return;
  }

  lanelet::BusStopRulePtr nearest_bus_stop = bus_stop_list.front();

  double bus_stop_downtrack_  = wm_->routeTrackPos(nearest_bus_stop->stopAndWaitLine().front().front().basicPoint2d()).downtrack;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "bus_stop_downtrack_ : " << bus_stop_downtrack_ );
  double distance_remaining_to_bus_stop = bus_stop_downtrack_  - current_state.downtrack;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "distance_remaining_to_bus_stop: " << distance_remaining_to_bus_stop <<
                    ", current_state.downtrack: " << current_state.downtrack);

  if (distance_remaining_to_bus_stop < -config_.bus_line_exit_zone_length)
  {
    resp->new_plan.maneuvers = {};
    RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name_), "Already passed bus stop, sending empty maneuvers");
    return;
  }                  

  constexpr double HALF_MPH_IN_MPS = 0.22352;  

  if (current_state.speed < HALF_MPH_IN_MPS && fabs(bus_stop_downtrack_ - current_state.downtrack ) < config_.stop_line_buffer) 
  {
    if(first_stop_)
    {
      time_to_move_ = now() + rclcpp::Duration(config_.dwell_time  * 1e9); 
      first_stop_ = false;
    }

    if(time_to_move_ <= now())
    {
      std::vector<lanelet::ConstLanelet> crossed_lanelets = getLaneletsBetweenWithException(current_state.downtrack, bus_stop_downtrack_, true, true);
      auto starting_lane_id = crossed_lanelets.front().id();
      auto ending_lane_id = crossed_lanelets.back().id();
      resp->new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(current_state.downtrack ,bus_stop_downtrack_,current_state.speed,starting_lane_id,ending_lane_id,max_comfort_decel_norm_ ,now(),now() + rclcpp::Duration(config_.min_maneuver_planning_period * 1e9) )); 
    }
    else
    {
      double maneuver_end_distance = bus_stop_downtrack_ + config_.bus_line_exit_zone_length;
      std::vector<lanelet::ConstLanelet> crossed_lanelets = getLaneletsBetweenWithException(current_state.downtrack, maneuver_end_distance, true, true);
      std::vector<lanelet::Id> lane_ids = lanelet::utils::transform(crossed_lanelets, [](const auto& ll) { return ll.id(); });
      speed_limit_ = findSpeedLimit(crossed_lanelets.front());
      resp->new_plan.maneuvers.push_back(composeLaneFollowingManeuverMessage(current_state.downtrack ,maneuver_end_distance,current_state.speed,speed_limit_,now(),config_.min_maneuver_planning_period,lane_ids));
    }
  }
 else if ( current_state.downtrack>= ( bus_stop_downtrack_ - config_.activation_distance ))
  {
    double desired_distance_to_stop = pow(current_state.speed, 2)/(2 * max_comfort_decel_norm_ * config_.deceleration_fraction) + config_.desired_distance_to_stop_buffer;
    
    if(current_state.downtrack >= ( bus_stop_downtrack_ - desired_distance_to_stop))
    {
      std::vector<lanelet::ConstLanelet> crossed_lanelets = getLaneletsBetweenWithException(current_state.downtrack, bus_stop_downtrack_, true, true);
      auto starting_lane_id = crossed_lanelets.front().id();
      auto ending_lane_id = crossed_lanelets.back().id();
      rclcpp::Time start_time = now();
      rclcpp::Time end_time = now() + rclcpp::Duration(config_.min_maneuver_planning_period * 1e9) ;
      //Stop at desired distance before bus stop
      resp->new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(current_state.downtrack ,bus_stop_downtrack_,current_state.speed,starting_lane_id,ending_lane_id,max_comfort_decel_norm_ ,start_time,end_time));
    }
    else
    {    
      double time_to_stop = (distance_remaining_to_bus_stop - desired_distance_to_stop)/speed_limit_;
      rclcpp::Time timestamp_to_stop = now() + rclcpp::Duration(time_to_stop * 1e9);
      std::vector<lanelet::ConstLanelet> crossed_lanelets = getLaneletsBetweenWithException(current_state.downtrack, (bus_stop_downtrack_ - desired_distance_to_stop) , true, true);
      std::vector<lanelet::ConstLanelet> crossed_lanelets_stop = getLaneletsBetweenWithException((bus_stop_downtrack_ - desired_distance_to_stop), bus_stop_downtrack_, true, true);
      std::vector<lanelet::Id> lane_ids = lanelet::utils::transform(crossed_lanelets, [](const auto& ll) { return ll.id(); });
      
      auto starting_lane_id = crossed_lanelets_stop.front().id();
      auto ending_lane_id = crossed_lanelets_stop.back().id();

      resp->new_plan.maneuvers.push_back(composeLaneFollowingManeuverMessage(current_state.downtrack ,(bus_stop_downtrack_ - desired_distance_to_stop),current_state.speed,speed_limit_,now(), time_to_stop,lane_ids));
      resp->new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage((bus_stop_downtrack_ - desired_distance_to_stop),bus_stop_downtrack_,speed_limit_,starting_lane_id,ending_lane_id,max_comfort_decel_norm_ ,timestamp_to_stop ,(timestamp_to_stop + rclcpp::Duration(config_.min_maneuver_planning_period * 1e9))));
    }
  }
  std::chrono::system_clock::time_point execution_end_time = std::chrono::system_clock::now();  // Planning complete

  auto execution_duration = execution_end_time - execution_start_time;
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "ExecutionTime stop_and_dwell_strategic_plugin: " << std::chrono::duration<double>(execution_duration).count());
  return;
}

carma_planning_msgs::msg::Maneuver StopAndDwellStrategicPlugin::composeLaneFollowingManeuverMessage(double start_dist, double end_dist,
                                                                          double start_speed, double target_speed,
                                                                          rclcpp::Time start_time, double time_to_stop,
                                                                          std::vector<lanelet::Id> lane_ids)
{
  carma_planning_msgs::msg::Maneuver maneuver_msg;
  carma_planning_msgs::msg::Maneuver empty_msg;
  maneuver_msg.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = config_.strategic_plugin_name;
  maneuver_msg.lane_following_maneuver.parameters.presence_vector = 
  carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN;
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

  RCLCPP_INFO_STREAM(rclcpp::get_logger("stop_and_dwell_strategic_plugin"), "Creating lane follow start dist: " << start_dist << " end dist: " << end_dist);
return maneuver_msg;
}


double StopAndDwellStrategicPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt) const
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

carma_planning_msgs::msg::Maneuver StopAndDwellStrategicPlugin::composeStopAndWaitManeuverMessage(double current_dist, double end_dist,
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

  RCLCPP_INFO_STREAM(rclcpp::get_logger("stop_and_dwell_strategic_plugin"), "Creating stop and wait start dist: " << current_dist << " end dist: " << end_dist);
  
  return maneuver_msg;
}

bool StopAndDwellStrategicPlugin::get_availability()
{
  return true;
}

std::string StopAndDwellStrategicPlugin::get_version_id()
{
  return "v1.0";
}

}  // namespace stop_and_dwell_strategic_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(stop_and_dwell_strategic_plugin::StopAndDwellStrategicPlugin)