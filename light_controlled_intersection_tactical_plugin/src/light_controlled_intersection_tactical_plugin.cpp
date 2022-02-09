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

SpeedProfileCase LightControlledIntersectionTacticalPlugin::determineSpeedProfileCase(double estimated_entry_time, double scheduled_entry_time, double speed_before_decel, double speed_before_accel)
{
  SpeedProfileCase case_num;
  
  ROS_DEBUG_STREAM("estimated_entry_time: " << estimated_entry_time << ", and scheduled_entry_time: " << scheduled_entry_time);
  if (estimated_entry_time < scheduled_entry_time)
  {
    ROS_DEBUG_STREAM("speed_before_accel: " << speed_before_accel << ", and config_.minimum_speed: " << config_.minimum_speed);

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
    ROS_DEBUG_STREAM("speed_before_decel: " << speed_before_decel << ", and speed_limit_: " << speed_limit_);

    if (speed_before_decel > speed_limit_)
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
  if(GET_MANEUVER_PROPERTY(maneuver,parameters.float_valued_meta_data).size() < 2){
    throw std::invalid_argument("Manuever timestamp and desired entry time into the signalized intersection are not provided in the meta data.");
  }

  double scheduled_entry_time = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[0]);
  double maneuver_start_time = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[1]);
  double starting_downtrack = GET_MANEUVER_PROPERTY(maneuver, start_dist);
  double ending_downtrack = GET_MANEUVER_PROPERTY(maneuver, end_dist);
  double departure_speed = GET_MANEUVER_PROPERTY(maneuver, end_speed);

  double entry_dist = GET_MANEUVER_PROPERTY(maneuver,end_dist) - starting_downtrack;
  double estimated_entry_time = calcEstimatedEntryTimeLeft(entry_dist, starting_speed, departure_speed );
  double speed_before_decel = calcSpeedBeforeDecel(estimated_entry_time, entry_dist, starting_speed, departure_speed);
  double speed_before_accel = calcSpeedBeforeAccel(estimated_entry_time, entry_dist, starting_speed, departure_speed);
  double remaining_time = scheduled_entry_time - maneuver_start_time;
  SpeedProfileCase case_num = determineSpeedProfileCase(estimated_entry_time, scheduled_entry_time, speed_before_decel, speed_before_accel);

  // change speed profile depending on algorithm case starting from maneuver start_dist
  if(case_num == ACCEL_CRUISE_DECEL || case_num == ACCEL_DECEL){
    // acceleration (cruising if needed) then deceleration to reach desired intersection entry speed/time according to algorithm doc
    apply_accel_cruise_decel_speed_profile(wm_, points_and_target_speeds, starting_downtrack, ending_downtrack, remaining_time, starting_speed, speed_before_decel, departure_speed);
  }
  else if(case_num == DECEL_ACCEL || case_num == DECEL_CRUISE_ACCEL)
  {
    // deceleration (cruising if needed) then acceleration to reach desired intersection entry speed/time according to algorithm doc
    apply_decel_cruise_accel_speed_profile(wm_, points_and_target_speeds, starting_downtrack, ending_downtrack, remaining_time, starting_speed, speed_before_accel, departure_speed);
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

double LightControlledIntersectionTacticalPlugin::calcEstimatedEntryTimeLeft(double entry_dist, double current_speed, double departure_speed) const
{
  double t_entry = 0;
  // t = 2 * d / (v_i + v_f)
  // from TSMO USE CASE 2 Algorithm Doc - Figure 4. Equation: Estimation of t*_nt
  t_entry = 2*entry_dist/(current_speed + departure_speed);
  ROS_DEBUG_STREAM("Estimated entry time: " << t_entry);
  return t_entry;
}

double LightControlledIntersectionTacticalPlugin::calcSpeedBeforeDecel(double entry_time, double entry_dist, double current_speed, double departure_speed) const
{
  double speed_before_decel = 0;

  double max_comfort_accel = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  double max_comfort_decel = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
  
  // from TSMO USE CASE 2 Algorithm Doc - Figure 7. Equation: Trajectory Smoothing Solution (Case 2)

  // a_r = a_acc / a_dec
  double acc_dec_ratio = max_comfort_accel/max_comfort_decel;
  // v_r = d / t
  double required_speed = entry_dist / entry_time;
  // sqrt_term  = sqrt((1-a_r)^2*v_r^2 - (1-a_r)(a_r*v_f*(v_f-2*v_r) + v_i*(2*v_r - v_i)))
  double sqr_term = sqrt(pow(1 - (acc_dec_ratio), 2) * pow(required_speed, 2) - (1 -acc_dec_ratio) *
                        (acc_dec_ratio * departure_speed * (departure_speed - 2 * required_speed) + current_speed * (2* required_speed - current_speed)));
  // v_e = v_r + sqrt_term/(1 - a_r)
  speed_before_decel = required_speed + sqr_term/(1 - acc_dec_ratio);

  return speed_before_decel;
}

double LightControlledIntersectionTacticalPlugin::calcSpeedBeforeAccel(double entry_time, double entry_dist, double current_speed, double departure_speed) const
{
  double speed_before_accel = 0;

  double max_comfort_accel = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  double max_comfort_decel = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;

  // from TSMO USE CASE 2 Algorithm Doc - Figure 11. Equation: Trajectory Smoothing Solution (Case 3)
  
  // a_r = a_acc / a_dec
  double acc_dec_ratio = max_comfort_accel/max_comfort_decel;
  // v_r = d / t
  double required_speed = entry_dist / entry_time;
  // sqrt_term  = sqrt((a_r - 1)^2*v_r^2 - (a_r-1)(v_f*(v_f-2*v_r) + a_r*v_i*(2*v_r - v_i)))
  double sqr_term = sqrt(pow((acc_dec_ratio - 1), 2) * pow(required_speed, 2) - (acc_dec_ratio - 1) *
                        (departure_speed * (departure_speed - 2 * required_speed) + acc_dec_ratio * current_speed * (2* required_speed - current_speed)));
  // v_e = v_r + sqrt_term / (a_r - 1)
  speed_before_accel = required_speed + sqr_term/(acc_dec_ratio - 1);

  return speed_before_accel;
}

void LightControlledIntersectionTacticalPlugin::apply_accel_cruise_decel_speed_profile(const carma_wm::WorldModelConstPtr& wm, std::vector<PointSpeedPair>& points_and_target_speeds, double start_dist, double end_dist, 
                                                                              double remaining_time, double starting_speed, double speed_before_decel, double departure_speed)
{
  if (points_and_target_speeds.empty())
  {
    throw std::invalid_argument("Point and target speed list is empty! Unable to apply case one speed profile...");
  }
  
  double max_comfort_accel = config_.vehicle_accel_limit;
  double max_comfort_decel = -1 * config_.vehicle_decel_limit;
  double remaining_dist = end_dist - start_dist;

  // a_r = a_acc / a_dec
  double acc_dec_ratio = max_comfort_accel/max_comfort_decel;
  
  double t_cruise = 0.0; // Cruising Time Interval for Case 2. TSMO UC 2 Algorithm draft doc Figure 7.
  double t_c_nom = 0.0;
  double t_c_den = epsilon_;

  if (speed_before_decel > speed_limit_)
  {
    ROS_DEBUG_STREAM("Detected that cruising is necessary. Changed speed_before_decel: " << speed_before_decel << ", to : " << speed_limit_);
    speed_before_decel = speed_limit_;

    // Cruising Time Interval Equation (case 1) obtained from TSMO UC 2 Algorithm draft doc Figure 8.
    // Nominator portion
    t_c_nom = 2 * remaining_dist * ((1 - acc_dec_ratio) * speed_before_decel + acc_dec_ratio * departure_speed - starting_speed) - 
                    remaining_time * ((1 - acc_dec_ratio) * pow(speed_before_decel, 2) + acc_dec_ratio * pow(departure_speed, 2) - pow(starting_speed, 2));
    
    // Denominator portion
    t_c_den = pow(speed_before_decel - starting_speed, 2) - acc_dec_ratio * pow(speed_before_decel - departure_speed, 2);
    
    if (t_c_den >= 0 && t_c_den < epsilon_)
    {
      ROS_WARN_STREAM("Denominator of cruising time interval is too close to zero: " 
                        << t_c_den << ", t_c_nom: " << t_c_nom << ", which may indicate there is only cruising portion available. Returning without any change..."); 
      return;
    }
    
    t_cruise = t_c_nom / t_c_den;
  }
  // From TSMO USE CASE 2 Algorithm Doc - Figure 8. Equation: Trajectory Smoothing Solution (Case 1 and 2)

  ROS_DEBUG_STREAM("max_comfort_accel: " << max_comfort_accel << "\n" <<
                   "max_comfort_decel: " << max_comfort_decel << "\n" <<
                   "acc_dec_ratio: " << acc_dec_ratio << "\n" <<
                   "speed_limit_: " << speed_limit_);
  
  // Rest of the equations for acceleration rates and time intervals for when accelerating or decelerating 
  double a_acc = ((1 - acc_dec_ratio) * speed_before_decel + acc_dec_ratio * departure_speed - starting_speed) / (remaining_time - t_cruise);
  double a_dec = ((max_comfort_decel - max_comfort_accel) * speed_before_decel + max_comfort_accel * departure_speed - max_comfort_decel * starting_speed) / (max_comfort_accel * (remaining_time - t_cruise));
  double t_acc = (speed_before_decel - starting_speed) / a_acc;
  double t_dec =  (departure_speed - speed_before_decel) / a_dec;

  ROS_DEBUG_STREAM("speed_before_decel: " << speed_before_decel << "\n" <<
                   "departure_speed: " << departure_speed << "\n" <<
                   "remaining_dist: " << remaining_dist << "\n" <<
                   "t_c_nom: " << t_c_nom << "\n" <<
                   "t_c_den: " << t_c_den << "\n" <<
                   "t_cruise: " << t_cruise << "\n" <<
                   "a_acc: " << a_acc << "\n" <<
                   "a_dec: " << a_dec << "\n" <<
                   "t_acc: " << t_acc << "\n" <<
                   "t_dec: " << t_dec);
  
  if (remaining_time - t_cruise < epsilon_ && remaining_time - t_cruise >= 0.0)
  {
    ROS_WARN_STREAM("Only Cruising is needed... therefore, no speed modification is required. Returning... ");
    return;
  }
  else if (t_cruise < -epsilon_)
  {
    throw std::invalid_argument(std::string("Input parameters are not valid or do not qualify conditions " 
                                "of estimated_time >= scheduled_time (case 1 and 2)"));
  }

  // Checking route geometry start against start_dist and adjust profile
  double planning_downtrack_start = wm->routeTrackPos(points_and_target_speeds[0].point).downtrack; // this can include buffered points earlier than maneuver start_dist
  double dist_accel;        //Distance over which acceleration happens
  double dist_cruise;     //Distance over which cruising happens
  double dist_decel;      //Distance over which deceleration happens

  //Use maneuver parameters to create speed profile
  //Kinematic: d = v_0 * t + 1/2 * a * t^2
  dist_accel = starting_speed * t_acc + 0.5 * a_acc * pow(t_acc, 2);
  dist_cruise = speed_before_decel * t_cruise;
  dist_decel = speed_before_decel * t_dec + 0.5 * a_dec * pow(t_dec, 2);

  //Check calculated total dist against maneuver limits
  double total_distance_needed = dist_accel + dist_cruise + dist_decel;

  ROS_DEBUG_STREAM("total_distance_needed: " << total_distance_needed << "\n" <<
                  "dist_accel: " << dist_accel << "\n" <<
                  "dist_decel: " << dist_decel << "\n" <<
                  "dist_cruise: " << dist_cruise);

  if(dist_accel < - epsilon_ )
  {
    //Requested maneuver needs to be modified to meet start and end dist req
    //Sacrifice on cruising and then acceleration if needed

    //correcting signs. NOTE: Doing so will likely result being over max_comfort_accel
    dist_accel = std::fabs(dist_accel); 
    a_acc = std::fabs(a_acc);
    a_dec = -1 * std::fabs(a_dec);
    //subtract distance from cruising segment to match original distance
    dist_cruise -= 2 * dist_accel;
    if(dist_cruise < 0)
    {
      dist_accel -= std::fabs(dist_cruise);
      dist_cruise = 0;
    }
    ROS_WARN_STREAM("Maneuver needed to be modified (due to negative dist_accel) with new distance and accelerations: \n" << 
                  "total_distance_needed: " << total_distance_needed << "\n" <<
                  "a_acc: " << a_acc << "\n" <<
                  "a_dec: " << a_dec << "\n" <<
                  "dist_accel: " << dist_accel << "\n" <<
                  "dist_decel: " << dist_decel << "\n" <<
                  "dist_cruise: " << dist_cruise);
    // not accounting dist_accel < 0 after this...
  }

  if(dist_decel < - epsilon_ )
  {
    //Requested maneuver needs to be modified to meet start and end dist req
    //Sacrifice on cruising and then acceleration if needed

    //correct signs. NOTE: Doing so will likely result being over max_comfort_accel
    dist_decel = std::fabs(dist_decel);  
    a_acc = std::fabs(a_acc);
    a_dec = -1 * std::fabs(a_dec);
    //subtract distance from cruising segment to match original distance
    dist_cruise -= 2 * dist_decel;
    if(dist_cruise < 0)
    {
      dist_accel -= std::fabs(dist_cruise);
      dist_cruise = 0;
    }
    ROS_WARN_STREAM("Maneuver needed to be modified (due to negative dist_decel) with new distance and accelerations: \n" << 
                  "total_distance_needed: " << total_distance_needed << "\n" <<
                  "a_acc: " << a_acc << "\n" <<
                  "a_dec: " << a_dec << "\n" <<
                  "dist_accel: " << dist_accel << "\n" <<
                  "dist_decel: " << dist_decel << "\n" <<
                  "dist_cruise: " << dist_cruise);
    // not accounting dist_accel < 0 after this...
  }

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

void LightControlledIntersectionTacticalPlugin::apply_decel_cruise_accel_speed_profile(const carma_wm::WorldModelConstPtr& wm, std::vector<PointSpeedPair>& points_and_target_speeds, double start_dist, double end_dist, 
                                                                              double remaining_time, double starting_speed, double speed_before_accel, double departure_speed)
{
  if (points_and_target_speeds.empty())
  {
    throw std::invalid_argument("Point and target speed list is empty! Unable to apply case one speed profile...");
  }
  
  double max_comfort_accel = config_.vehicle_accel_limit;
  double max_comfort_decel = -1 * config_.vehicle_decel_limit;
  double remaining_dist = end_dist - start_dist;

  // a_r = a_acc / a_dec
  double acc_dec_ratio = max_comfort_accel/max_comfort_decel;
  
  double t_cruise = 0.0; // Cruising Time Interval for Case 4. TSMO UC 2 Algorithm draft doc Figure 12.
  double t_c_nom = 0.0;
  double t_c_den = epsilon_;

  if (speed_before_accel < config_.minimum_speed)
  {
    ROS_DEBUG_STREAM("Detected that cruising is necessary. Changed speed_before_accel: " << speed_before_accel << ", to : " << config_.minimum_speed);
    speed_before_accel = config_.minimum_speed;

    // Cruising Time Interval Equation (case 1) obtained from TSMO UC 2 Algorithm draft doc Figure 8.
    // Nominator portion
    t_c_nom = 2 * remaining_dist * ((acc_dec_ratio - 1) * speed_before_accel + departure_speed - acc_dec_ratio * starting_speed) - 
                    remaining_time * ((acc_dec_ratio - 1) * pow(speed_before_accel, 2) + pow(departure_speed, 2) - acc_dec_ratio * pow(starting_speed, 2));
    
    // Denominator portion
    t_c_den = acc_dec_ratio * pow(speed_before_accel - starting_speed, 2) - pow(speed_before_accel - departure_speed, 2);
    
    if (t_c_den >= 0 && t_c_den < epsilon_)
    {
      ROS_WARN_STREAM("Denominator of cruising time interval is too close to zero: " 
                        << t_c_den << ", t_c_nom: " << t_c_nom << ", which may indicate there is only cruising portion available. Returning without any change..."); 
      return;
    }
    
    t_cruise = t_c_nom / t_c_den;
  }
  // From TSMO USE CASE 2 Algorithm Doc - Figure 11 - 13. Equation: Trajectory Smoothing Solution (Case 1 and 2)

  ROS_DEBUG_STREAM("max_comfort_accel: " << max_comfort_accel << "\n" <<
                   "max_comfort_decel: " << max_comfort_decel << "\n" <<
                   "acc_dec_ratio: " << acc_dec_ratio << "\n" <<
                   "config_.minimum_speed: " << config_.minimum_speed);
  
  // Rest of the equations for acceleration rates and time intervals for when accelerating or decelerating 
  double a_acc = ((acc_dec_ratio - 1) * speed_before_accel + departure_speed - acc_dec_ratio * starting_speed) / (remaining_time - t_cruise);
  double a_dec = ((max_comfort_accel - max_comfort_decel) * speed_before_accel + max_comfort_decel * departure_speed - max_comfort_accel * starting_speed) / (max_comfort_accel * (remaining_time - t_cruise));
  double t_acc = (departure_speed - speed_before_accel) / a_acc;
  double t_dec =  (speed_before_accel - starting_speed) / a_dec;

  ROS_DEBUG_STREAM("speed_before_accel: " << speed_before_accel << "\n" <<
                   "departure_speed: " << departure_speed << "\n" <<
                   "remaining_dist: " << remaining_dist << "\n" <<
                   "t_c_nom: " << t_c_nom << "\n" <<
                   "t_c_den: " << t_c_den << "\n" <<
                   "t_cruise: " << t_cruise << "\n" <<
                   "a_acc: " << a_acc << "\n" <<
                   "a_dec: " << a_dec << "\n" <<
                   "t_acc: " << t_acc << "\n" <<
                   "t_dec: " << t_dec);
  
  if (remaining_time - t_cruise < epsilon_ && remaining_time - t_cruise >= 0.0)
  {
    ROS_WARN_STREAM("Only Cruising is needed... therefore, no speed modification is required. Returning... ");
    return;
  }
  else if (t_cruise < -epsilon_)
  {
    throw std::invalid_argument(std::string("Input parameters are not valid or do not qualify conditions " 
                                "of estimated_time < scheduled_time (case 3 and 4)"));
  }

  // Checking route geometry start against start_dist and adjust profile
  double planning_downtrack_start = wm->routeTrackPos(points_and_target_speeds[0].point).downtrack; // this can include buffered points earlier than maneuver start_dist
  double dist_accel;        //Distance over which acceleration happens
  double dist_cruise;     //Distance over which cruising happens
  double dist_decel;      //Distance over which deceleration happens

  //Use maneuver parameters to create speed profile
  //Kinematic: d = v_0 * t + 1/2 * a * t^2
  dist_decel = starting_speed * t_dec + 0.5 * a_dec * pow(t_dec, 2);
  dist_cruise = speed_before_accel * t_cruise;
  dist_accel = speed_before_accel * t_acc + 0.5 * a_acc * pow(t_acc, 2);
  
  //Check calculated total dist against maneuver limits
  double total_distance_needed = dist_accel + dist_cruise + dist_decel;

  ROS_DEBUG_STREAM("total_distance_needed: " << total_distance_needed << "\n" <<
                  "dist_accel: " << dist_accel << "\n" <<
                  "dist_decel: " << dist_decel << "\n" <<
                  "dist_cruise: " << dist_cruise);

  if(dist_decel < - epsilon_ )
  {
    //Requested maneuver needs to be modified to meet start and end dist req
    //Sacrifice on cruising and then deceleration if needed

    //correct signs. NOTE: Doing so will likely result being over max_comfort_decel
    dist_decel = std::fabs(dist_decel);  
    a_acc = std::fabs(a_acc);
    a_dec = -1 * std::fabs(a_dec);
    //subtract distance from cruising segment to match original distance
    dist_cruise -= 2 * dist_decel;
    if(dist_cruise < 0)
    {
      dist_decel -= std::fabs(dist_cruise);
      dist_cruise = 0;
    }
    ROS_WARN_STREAM("Maneuver needed to be modified (due to negative dist_decel) with new distance and accelerations: \n" << 
                  "total_distance_needed: " << total_distance_needed << "\n" <<
                  "a_acc: " << a_acc << "\n" <<
                  "a_dec: " << a_dec << "\n" <<
                  "dist_accel: " << dist_accel << "\n" <<
                  "dist_decel: " << dist_decel << "\n" <<
                  "dist_cruise: " << dist_cruise);
    // not accounting dist_decel < 0 after this...
  }

  if(dist_accel < - epsilon_ )
  {
    //Requested maneuver needs to be modified to meet start and end dist req
    //Sacrifice on cruising and then deceleration if needed

    //correcting signs. NOTE: Doing so will likely result being over max_comfort_decel
    dist_accel = std::fabs(dist_accel); 
    a_acc = std::fabs(a_acc);
    a_dec = -1 * std::fabs(a_dec);
    //subtract distance from cruising segment to match original distance
    dist_cruise -= 2 * dist_accel;
    if(dist_cruise < 0)
    {
      dist_decel -= std::fabs(dist_cruise);
      dist_cruise = 0;
    }
    ROS_WARN_STREAM("Maneuver needed to be modified (due to negative dist_accel) with new distance and accelerations: \n" << 
                  "total_distance_needed: " << total_distance_needed << "\n" <<
                  "a_acc: " << a_acc << "\n" <<
                  "a_dec: " << a_dec << "\n" <<
                  "dist_accel: " << dist_accel << "\n" <<
                  "dist_decel: " << dist_decel << "\n" <<
                  "dist_cruise: " << dist_cruise);
    // not accounting dist_decel < 0 after this...
  }

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