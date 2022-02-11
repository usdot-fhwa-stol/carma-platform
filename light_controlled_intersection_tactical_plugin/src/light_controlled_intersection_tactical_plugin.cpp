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
    plugin_discovery_msg_.version_id = "v1.0";
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
        || req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN || req.maneuver_plan.maneuvers[i].type ==cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN
        && GET_MANEUVER_PROPERTY(req.maneuver_plan.maneuvers[i], parameters.string_valued_meta_data.front()) == light_controlled_intersection_strategy_)
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

    lanelet::BasicPoint2d veh_pos(req.vehicle_state.x_pos_global, req.vehicle_state.y_pos_global);
    ROS_DEBUG_STREAM("Planning state x:"<<req.vehicle_state.x_pos_global <<" , y: " << req.vehicle_state.y_pos_global);

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

void LightControlledIntersectionTacticalPlugin::apply_optimized_target_speed_profile(const cav_msgs::Maneuver& maneuver, const double starting_speed, std::vector<PointSpeedPair>& points_and_target_speeds)
{
  if(GET_MANEUVER_PROPERTY(maneuver,parameters.float_valued_meta_data).size() < 7 || 
      GET_MANEUVER_PROPERTY(maneuver,parameters.int_valued_meta_data).size() < 1 ){
    throw std::invalid_argument("There must be 7 float_valued_meta_data and 1 int_valued_meta_data to apply algorithm's parameters.");
  }

  double a_accel = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[0]);
  double a_decel = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[1]);
  double dist_accel = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[2]);
  double dist_cruise = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[3]);
  double dist_decel = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[4]);
  double speed_before_accel = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[5]);
  double speed_before_decel = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[6]);
  SpeedProfileCase case_num = static_cast<SpeedProfileCase>GET_MANEUVER_PROPERTY(maneuver, parameters.int_valued_meta_data[0]);

  double starting_downtrack = GET_MANEUVER_PROPERTY(maneuver, start_dist);
  double ending_downtrack = GET_MANEUVER_PROPERTY(maneuver, end_dist);
  double departure_speed = GET_MANEUVER_PROPERTY(maneuver, end_speed);
  double scheduled_entry_time = GET_MANEUVER_PROPERTY(maneuver, end_time).toSec();
  double entry_dist = ending_downtrack - starting_downtrack;

  // change speed profile depending on algorithm case starting from maneuver start_dist
  if(case_num == ACCEL_CRUISE_DECEL || case_num == ACCEL_DECEL){
    // acceleration (cruising if needed) then deceleration to reach desired intersection entry speed/time according to algorithm doc
    apply_accel_cruise_decel_speed_profile(wm_, points_and_target_speeds, starting_downtrack, entry_dist, starting_speed, speed_before_decel, 
                                            departure_speed, dist_accel, dist_cruise, dist_decel, a_accel, a_decel);
  }
  else if(case_num == DECEL_ACCEL || case_num == DECEL_CRUISE_ACCEL)
  {
    // deceleration (cruising if needed) then acceleration to reach desired intersection entry speed/time according to algorithm doc
    apply_decel_cruise_accel_speed_profile(wm_, points_and_target_speeds, starting_downtrack, entry_dist, starting_speed, speed_before_accel, 
                                            departure_speed, dist_accel, dist_cruise, dist_decel, a_accel, a_decel);
  }
  else
  {
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

void LightControlledIntersectionTacticalPlugin::apply_accel_cruise_decel_speed_profile(const carma_wm::WorldModelConstPtr& wm, std::vector<PointSpeedPair>& points_and_target_speeds, double start_dist, double remaining_dist, 
                                                                              double starting_speed, double speed_before_decel, double departure_speed, double dist_accel, double dist_cruise, double dist_decel, double a_acc, double a_dec)
{
  if (points_and_target_speeds.empty())
  {
    throw std::invalid_argument("Point and target speed list is empty! Unable to apply case one speed profile...");
  }
  
  // Checking route geometry start against start_dist and adjust profile
  double planning_downtrack_start = wm->routeTrackPos(points_and_target_speeds[0].point).downtrack; // this can include buffered points earlier than maneuver start_dist

  //Check calculated total dist against maneuver limits
  double total_distance_needed = dist_accel + dist_cruise + dist_decel;

  ROS_DEBUG_STREAM("total_distance_needed: " << total_distance_needed << "\n" <<
                  "dist_accel: " << dist_accel << "\n" <<
                  "dist_decel: " << dist_decel << "\n" <<
                  "dist_cruise: " << dist_cruise);

  double total_dist_planned = 0; //Starting dist for maneuver treated as 0.0

  if (planning_downtrack_start < start_dist)
  {
    //Account for the buffer distance that is technically not part of this maneuver
    
    total_dist_planned = planning_downtrack_start - start_dist;
    ROS_DEBUG_STREAM("buffered section is present. Adjusted total_dist_planned to: " << total_dist_planned);      
  }
  
  double prev_speed = starting_speed;
  auto prev_point = points_and_target_speeds.front();
  
  for(auto& p : points_and_target_speeds)
  {
    double delta_d = lanelet::geometry::distance2d(prev_point.point, p.point);
    total_dist_planned += delta_d;  

    //Apply the speed from algorithm at dist covered
    //Kinematic: v_f = sqrt(v_o^2 + 2*a*d)
    double speed_i;
    if (total_dist_planned <= epsilon_) 
    {
      //Keep target speed same for buffer distance portion
      speed_i = starting_speed;
    }
    else if(total_dist_planned <= dist_accel + epsilon_){
      //Acceleration part
      speed_i = sqrt(pow(starting_speed, 2) + 2 * a_acc * total_dist_planned);
    }
    else if(dist_cruise > 0 && total_dist_planned > dist_accel && total_dist_planned <= (dist_accel + dist_cruise) + epsilon_){
      //Cruising part
      speed_i = speed_before_decel;
    }
    else if (total_dist_planned <= total_distance_needed + epsilon_)
    {
      //Deceleration part
      speed_i = sqrt(std::max(pow(speed_before_decel, 2) + 2 * a_dec * (total_dist_planned - dist_accel - dist_cruise), 0.0)); //std::max to ensure negative value is not sqrt
    }
    else 
    {
      //buffer points that will be cut
      speed_i = prev_speed;
    }
    
    p.speed = std::min(speed_i,speed_before_decel);
    ROS_DEBUG_STREAM("Applied speed: " << p.speed << ", at dist: " << total_dist_planned);

    prev_point = p;
    prev_speed = speed_i;
  }
}

void LightControlledIntersectionTacticalPlugin::apply_decel_cruise_accel_speed_profile(const carma_wm::WorldModelConstPtr& wm, std::vector<PointSpeedPair>& points_and_target_speeds, double start_dist, double remaining_dist, 
                                                                              double starting_speed, double speed_before_accel, double departure_speed, double dist_accel, double dist_cruise, double dist_decel, double a_acc, double a_dec)
{
  if (points_and_target_speeds.empty())
  {
    throw std::invalid_argument("Point and target speed list is empty! Unable to apply case one speed profile...");
  }

  // Checking route geometry start against start_dist and adjust profile
  double planning_downtrack_start = wm->routeTrackPos(points_and_target_speeds[0].point).downtrack; // this can include buffered points earlier than maneuver start_dist
  
  //Check calculated total dist against maneuver limits
  double total_distance_needed = dist_accel + dist_cruise + dist_decel;

  ROS_DEBUG_STREAM("total_distance_needed: " << total_distance_needed << "\n" <<
                  "dist_accel: " << dist_accel << "\n" <<
                  "dist_decel: " << dist_decel << "\n" <<
                  "dist_cruise: " << dist_cruise);

  double total_dist_planned = 0; //Starting dist for maneuver treated as 0.0

  if (planning_downtrack_start < start_dist)
  {
    //Account for the buffer distance that is technically not part of this maneuver
    
    total_dist_planned = planning_downtrack_start - start_dist;
    ROS_DEBUG_STREAM("buffered section is present. Adjusted total_dist_planned to: " << total_dist_planned);      
  }
  
  double prev_speed = starting_speed;
  auto prev_point = points_and_target_speeds.front();
  
  for(auto& p : points_and_target_speeds)
  {
    double delta_d = lanelet::geometry::distance2d(prev_point.point, p.point);
    total_dist_planned += delta_d;  
    //Apply the speed from algorithm at dist covered
    //Kinematic: v_f = sqrt(v_o^2 + 2*a*d)
    double speed_i;
    if (total_dist_planned <= epsilon_) 
    {
      //Keep target speed same for buffer distance portion
      speed_i = starting_speed;
    }
    else if (total_dist_planned <= dist_decel + epsilon_)
    {
      //Deceleration part
      speed_i = sqrt(pow(starting_speed, 2) + 2 * a_dec * total_dist_planned);
    }
    else if(dist_cruise > 0 && total_dist_planned > dist_decel && total_dist_planned <= (dist_decel + dist_cruise) + epsilon_){
      //Cruising part
      speed_i = speed_before_accel;
    }
    else if(total_dist_planned <= total_distance_needed + epsilon_){
      //Acceleration part
      
      speed_i = sqrt(std::max(pow(speed_before_accel, 2) + 2 * a_acc * (total_dist_planned - dist_decel - dist_cruise), 0.0)); //std::max to ensure negative value is not sqrt
    }
    else 
    {
      //buffer points that will be cut
      
      speed_i = prev_speed;
    }
    
    p.speed = std::max(speed_i,speed_before_accel);
    p.speed = std::min(p.speed,speed_limit_);
    ROS_DEBUG_STREAM("Applied speed: " << p.speed << ", at dist: " << total_dist_planned);

    prev_point = p;
    prev_speed = speed_i;
  }
}



} //namespace light_controlled_intersection_transit_plugin