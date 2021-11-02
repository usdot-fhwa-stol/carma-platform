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

#include <ros/ros.h>
#include <string>
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <sstream>
#include <carma_utils/containers/containers.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <unordered_set>
#include <vector>
#include <cav_msgs/Trajectory.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_utils/containers/containers.h>
#include <carma_wm/Geometry.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <light_controlled_intersection_plugin.h>

using oss = std::ostringstream;

namespace light_controlled_intersection_transit_plugin
{

LightControlledIntersectionTacticalPlugin::LightControlledIntersectionTacticalPlugin(carma_wm::WorldModelConstPtr wm,const LightControlledIntersectionTacticalPluginConfig& config,
                                    const PublishPluginDiscoveryCB& plugin_discovery_publisher)
  : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher)
  {
    plugin_discovery_msg_.name = "LightControlledIntersectionTacticalPlugin";
    plugin_discovery_msg_.versionId = "v1.0";
    plugin_discovery_msg_.available = true;
    plugin_discovery_msg_.activated = false;
    plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
    plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
  }

void LightControlledIntersectionTacticalPlugin::onSpin()
{
    plugin_discovery_publisher_(plugin_discovery_msg_);
}

bool LightControlledIntersectionTacticalPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
{
    ROS_DEBUG_STREAM("Starting light controlled intersection trajectory planning");
    
    if(req.maneuver_index_to_plan >= req.maneuver_plan.maneuvers.size())
    {
    throw std::invalid_argument(
        "Light Control Intersection Plugin asked to plan invalid maneuver index: " + std::to_string(req.maneuver_index_to_plan) + 
        " for plan of size: " + std::to_string(req.maneuver_plan.maneuvers.size()));
    }
    std::vector<cav_msgs::Maneuver> maneuver_plan;
    // expecting only one maneuver for an intersection
    for(size_t i = req.maneuver_index_to_plan; i < req.maneuver_plan.maneuvers.size(); i++){
        
        if(req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT
        || req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN || req.maneuver_plan.maneuvers[i].type ==cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN)
        {
            maneuver_plan.push_back(req.maneuver_plan.maneuvers[i]);
            resp.related_maneuvers.push_back(req.maneuver_plan.maneuvers[i].type);
            break;
        }
        else
        {
            throw std::invalid_argument("Light Control Intersection Plugin asked to plan unsupported maneuver");
        }
    }

    lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
    ROS_DEBUG_STREAM("Planning state x:"<<req.vehicle_state.X_pos_global <<" , y: " << req.vehicle_state.Y_pos_global);

    current_downtrack_ = wm_->routeTrackPos(veh_pos).downtrack;
    ROS_DEBUG_STREAM("Current_downtrack"<< current_downtrack_);

    DetailedTrajConfig wpg_detail_config;
    GeneralTrajConfig wpg_general_config;

    wpg_general_config = basic_autonomy:: waypoint_generation::compose_general_trajectory_config("intersection_transit",
                                                                                config_.default_downsample_ratio,
                                                                                config_.turn_downsample_ratio);

    wpg_detail_config = basic_autonomy:: waypoint_generation::compose_detailed_trajectory_config(config_.trajectory_time_length, 
                                                                              config_.curve_resample_step_size, config_.minimum_speed, 
                                                                              config_.vehicle_accel_limit,
                                                                              config_.lateral_accel_limit, 
                                                                              config_.speed_moving_average_window_size, 
                                                                              config_.curvature_moving_average_window_size, config_.back_distance,
                                                                              config_.buffer_ending_downtrack);

    // Create curve-fitting compatible trajectories (with extra back and front attached points) with raw speed limits from maneuver 
    auto points_and_target_speeds = create_geometry_profile(maneuver_plan, std::max((double)0, current_downtrack_ - config_.back_distance),
                                                                            wm_, ending_state_before_buffer_, req.vehicle_state, wpg_general_config, wpg_detail_config);

    // Change raw speed limit values to target speeds specified by the algorithm
    apply_optimized_target_speed_profile(maneuver_plan.front(), req.vehicle_state.longitudinal_vel, points_and_target_speeds);

    ROS_DEBUG_STREAM("points_and_target_speeds: " << points_and_target_speeds.size());

    ROS_DEBUG_STREAM("PlanTrajectory");

    //Trajectory Plan
    cav_msgs::TrajectoryPlan trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = req.header.stamp;
    trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

    // Compose smooth trajectory/speed by resampling
    trajectory.trajectory_points = basic_autonomy:: waypoint_generation::compose_lanefollow_trajectory_from_path(points_and_target_speeds, 
                                                                                req.vehicle_state, req.header.stamp, wm_, ending_state_before_buffer_, debug_msg_, 
                                                                                wpg_detail_config); // Compute the trajectory
    trajectory.initial_longitudinal_velocity = req.vehicle_state.longitudinal_vel;

    resp.trajectory_plan = trajectory;
    
    resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

    return true;
}

SpeedProfileCase LightControlledIntersectionTacticalPlugin::determineSpeedProfileCase(double entry_dist, double current_speed, double schedule_entry_time, double speed_limit)
{
  SpeedProfileCase case_num;
  double estimated_entry_time = calcEstimatedEntryTimeLeft(entry_dist, current_speed, speed_limit );
  double speed_before_decel = calcSpeedBeforeDecel(estimated_entry_time, entry_dist, current_speed, speed_limit);
  double speed_before_accel = calcSpeedBeforeAccel(estimated_entry_time, entry_dist, current_speed, speed_limit);
  
  ROS_DEBUG_STREAM("estimated_entry_time: " << estimated_entry_time << ", and schedule_entry_time: " << schedule_entry_time);
  if (estimated_entry_time < schedule_entry_time)
  {
    ROS_DEBUG_STREAM("speed_before_accel: " << speed_before_accel << ", and speed_limit: " << speed_limit);

    if (speed_before_accel < config_.minimum_speed)
    {
      case_num = DECEL_CRUISE_ACCEL;
    }
    else
    {
      case_num = DECEL_ACCEL;
    }
  }
  else
  {
    ROS_DEBUG_STREAM("speed_before_decel: " << speed_before_decel << ", and speed_limit: " << speed_limit);

    if (speed_before_decel > speed_limit)
    {
      case_num = ACCEL_CRUISE_DECEL;
    }
    else
    {
      case_num = ACCEL_DECEL;
    }
  }
  
  return case_num;
}

void LightControlledIntersectionTacticalPlugin::apply_optimized_target_speed_profile(const cav_msgs::Maneuver& maneuver, const double starting_speed, std::vector<PointSpeedPair>& points_and_target_speeds)
{
  if(GET_MANEUVER_PROPERTY(maneuver,parameters.float_valued_meta_data).empty()){
    throw std::invalid_argument("Desired entry time into the signalized intersection is not provided in the meta data.");
  }

  double time_to_schedule_entry = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[0]);
  double starting_downtrack = GET_MANEUVER_PROPERTY(maneuver, start_dist);
  SpeedProfileCase case_num = determineSpeedProfileCase(GET_MANEUVER_PROPERTY(maneuver,end_dist) - starting_downtrack, starting_speed, time_to_schedule_entry, speed_limit_);

  // change speed profile depending on algorithm case starting from maneuver start_dist
  // NOTE: when applying the speed profile, each cases should ignore config_.back_distance worth of points' speed in front
  if(case_num == ACCEL_DECEL){
      // acceleration then deceleration to reach desired intersection entry speed/time according to algorithm doc
      apply_case_one_speed_profile(points_and_target_speeds);
  }
  else if(case_num == ACCEL_CRUISE_DECEL){
      // acceleration, cruising, deceleration to reach desired intersection entry speed/time according to algorithm doc
      apply_case_two_speed_profile(points_and_target_speeds);
  }
  else if(case_num == DECEL_ACCEL)
  {
      // deceleration then acceleration to reach desired intersection entry speed/time according to algorithm doc
      apply_case_three_speed_profile(points_and_target_speeds);
  }
  else if(case_num == DECEL_CRUISE_ACCEL)
  {
      // deceleration, cruising, acceleration to reach desired intersection entry speed/time according to algorithm doc
      apply_case_four_speed_profile(points_and_target_speeds);
  }
  else{
      throw std::invalid_argument("The light controlled intersection tactical plugin doesn't handle the case number requested");
  }
}
std::vector<PointSpeedPair> LightControlledIntersectionTacticalPlugin::create_geometry_profile(const std::vector<cav_msgs::Maneuver> &maneuvers, double max_starting_downtrack,const carma_wm::WorldModelConstPtr &wm,
                                                                   cav_msgs::VehicleState &ending_state_before_buffer,const cav_msgs::VehicleState& state,
                                                                   const GeneralTrajConfig &general_config, const DetailedTrajConfig &detailed_config)
{
  std::vector<PointSpeedPair> points_and_target_speeds;
  
  bool first = true;
  std::unordered_set<lanelet::Id> visited_lanelets;

  ROS_DEBUG_STREAM("VehDowntrack:"<<max_starting_downtrack);
  for(const auto &maneuver : maneuvers)
  {
      double starting_downtrack = GET_MANEUVER_PROPERTY(maneuver, start_dist);
      
      starting_downtrack = std::min(starting_downtrack, max_starting_downtrack);

      ROS_DEBUG_STREAM("Used downtrack: " << starting_downtrack);

      // check if required parameter from strategic planner is present
      if(GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data).empty())
      {
        throw std::invalid_argument("No time_to_schedule_entry is provided in float_valued_meta_data");
      }

      //overwrite maneuver type to use lane follow library function
      cav_msgs::Maneuver temp_maneuver = maneuver;
      temp_maneuver.type =cav_msgs::Maneuver::LANE_FOLLOWING;
      ROS_DEBUG_STREAM("Creating Lane Follow Geometry");
      std::vector<PointSpeedPair> lane_follow_points = basic_autonomy::waypoint_generation::create_lanefollow_geometry(maneuver, starting_downtrack, wm, ending_state_before_buffer, general_config, detailed_config, visited_lanelets);
      points_and_target_speeds.insert(points_and_target_speeds.end(), lane_follow_points.begin(), lane_follow_points.end());
      
      break; // expected to receive only one maneuver to plan
  }

  return points_and_target_speeds;
}

double LightControlledIntersectionTacticalPlugin::calcEstimatedEntryTimeLeft(double entry_dist, double current_speed, double speed_limit) const
{
  double t_entry = 0;
  // t = 2 * d / (v_i + v_f)
  // from TSMO USE CASE 2 Algorithm Doc - Figure 4. Equation: Estimation of t*_nt
  t_entry = 2*entry_dist/(current_speed + speed_limit);
  return t_entry;
}

double LightControlledIntersectionTacticalPlugin::calcSpeedBeforeDecel(double entry_time, double entry_dist, double current_speed, double speed_limit) const
{
  double speed_before_decel = 0;

  double desired_acceleration = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  double desired_deceleration = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
  
  // from TSMO USE CASE 2 Algorithm Doc - Figure 7. Equation: Trajectory Smoothing Solution (Case 2)

  // a_r = a_acc / a_dec
  double acc_dec_ratio = desired_acceleration/desired_deceleration;
  // v_r = d / t
  double required_speed = entry_dist / entry_time;
  // sqrt_term  = sqrt((1-a_r)^2*v_r^2 - (1-a_r)(a_r*v_f*(v_f-2*v_r) + v_i*(2*v_r - v_i)))
  double sqr_term = sqrt(pow(1 - (acc_dec_ratio), 2) * pow(required_speed, 2) - (1 -acc_dec_ratio) *
                        (acc_dec_ratio * speed_limit * (speed_limit - 2 * required_speed) + current_speed * (2* required_speed - current_speed)));
  // v_e = v_r + sqrt_term/(1 - a_r)
  speed_before_decel = required_speed + sqr_term/(1 - acc_dec_ratio);

  return speed_before_decel;
}

double LightControlledIntersectionTacticalPlugin::calcSpeedBeforeAccel(double entry_time, double entry_dist, double current_speed, double speed_limit) const
{
  double speed_before_accel = 0;

  double desired_acceleration = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  double desired_deceleration = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;

  // from TSMO USE CASE 2 Algorithm Doc - Figure 11. Equation: Trajectory Smoothing Solution (Case 3)
  
  // a_r = a_acc / a_dec
  double acc_dec_ratio = desired_acceleration/desired_deceleration;
  // v_r = d / t
  double required_speed = entry_dist / entry_time;
  // sqrt_term  = sqrt((a_r - 1)^2*v_r^2 - (a_r-1)(v_f*(v_f-2*v_r) + a_r*v_i*(2*v_r - v_i)))
  double sqr_term = sqrt(pow((acc_dec_ratio - 1), 2) * pow(required_speed, 2) - (acc_dec_ratio - 1) *
                        (speed_limit * (speed_limit - 2 * required_speed) + acc_dec_ratio * current_speed * (2* required_speed - current_speed)));
  // v_e = v_r + sqrt_term / (a_r - 1)
  speed_before_accel = required_speed + sqr_term/(acc_dec_ratio - 1);

  return speed_before_accel;
}

void LightControlledIntersectionTacticalPlugin::apply_case_one_speed_profile(std::vector<PointSpeedPair>& points_and_target_speeds){
    
}
void LightControlledIntersectionTacticalPlugin::apply_case_two_speed_profile(std::vector<PointSpeedPair>& points_and_target_speeds){
    
}
void LightControlledIntersectionTacticalPlugin::apply_case_three_speed_profile(std::vector<PointSpeedPair>& points_and_target_speeds){
    
}
void LightControlledIntersectionTacticalPlugin::apply_case_four_speed_profile(std::vector<PointSpeedPair>& points_and_target_speeds){
    
}


}