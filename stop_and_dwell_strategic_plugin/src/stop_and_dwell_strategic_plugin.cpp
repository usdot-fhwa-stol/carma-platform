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

StopAndDwellStrategicPlugin::VehicleState StopAndDwellStrategicPlugin::extractInitialState(const carma_planning_msgs::srv::PlanManeuvers::Request& req) const
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



void StopAndDwellStrategicPlugin::plan_maneuvers_callback(
  std::shared_ptr<rmw_request_id_t> srv_header, 
  carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
  carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp)
{
   return;
}

carma_planning_msgs::msg::Maneuver StopAndDwellStrategicPlugin::composeLaneFollowingManeuverMessage(int case_num, double start_dist, double end_dist,
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