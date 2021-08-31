/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include <inlanecruising_plugin/inlanecruising_plugin.h>




using oss = std::ostringstream;

namespace inlanecruising_plugin
{
InLaneCruisingPlugin::InLaneCruisingPlugin(carma_wm::WorldModelConstPtr wm, InLaneCruisingPluginConfig config,
                                           PublishPluginDiscoveryCB plugin_discovery_publisher, DebugPublisher debug_publisher)
  : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher), debug_publisher_(debug_publisher)
{
  plugin_discovery_msg_.name = "InLaneCruisingPlugin";
  plugin_discovery_msg_.versionId = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = false;
  plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
  plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
}

bool InLaneCruisingPlugin::onSpin()
{
  plugin_discovery_publisher_(plugin_discovery_msg_);
  return true;
}

bool InLaneCruisingPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req,
                                              cav_srvs::PlanTrajectoryResponse& resp)
{
   ros::WallTime start_time = ros::WallTime::now();  // Start timeing the execution time for planning so it can be logged

  lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
  double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

  // Only plan the trajectory for the initial LANE_FOLLOWING maneuver and any immediately sequential maneuvers of the same type
  std::vector<cav_msgs::Maneuver> maneuver_plan;
  for(size_t i = req.maneuver_index_to_plan; i < req.maneuver_plan.maneuvers.size(); i++)
  {
    if(req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::LANE_FOLLOWING)
    {
      speed_limit_ = std::min(speed_limit_, req.maneuver_plan.maneuvers[i].lane_following_maneuver.end_speed);
      maneuver_plan.push_back(req.maneuver_plan.maneuvers[i]);
      resp.related_maneuvers.push_back(i);
    }
    else
    {
      break;
    }
  }

  basic_autonomy:: waypoint_generation::DetailedTrajConfig wpg_detail_config;
  basic_autonomy:: waypoint_generation::GeneralTrajConfig wpg_general_config;

  wpg_general_config = basic_autonomy:: waypoint_generation::compose_general_trajectory_config("inlanecruising",
                                                                              config_.default_downsample_ratio,
                                                                              config_.turn_downsample_ratio);

  wpg_detail_config = basic_autonomy:: waypoint_generation::compose_detailed_trajectory_config(config_.trajectory_time_length, 
                                                                            config_.curve_resample_step_size, config_.minimum_speed, 
                                                                            config_.max_accel * config_.max_accel_multiplier,
                                                                            config_.lateral_accel_limit * config_.lat_accel_multiplier, 
                                                                            config_.speed_moving_average_window_size, 
                                                                            config_.curvature_moving_average_window_size, config_.back_distance,
                                                                            config_.buffer_ending_downtrack);
  
  auto points_and_target_speeds = basic_autonomy::waypoint_generation::create_geometry_profile(maneuver_plan, std::max((double)0, current_downtrack - config_.back_distance),
                                                                         wm_, ending_state_before_buffer_, req.vehicle_state, wpg_general_config, wpg_detail_config);

  ROS_DEBUG_STREAM("points_and_target_speeds: " << points_and_target_speeds.size());

  ROS_DEBUG_STREAM("PlanTrajectory");

  cav_msgs::TrajectoryPlan original_trajectory;
  original_trajectory.header.frame_id = "map";
  original_trajectory.header.stamp = ros::Time::now();
  original_trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

  original_trajectory.trajectory_points = basic_autonomy:: waypoint_generation::compose_lanefollow_trajectory_from_path(points_and_target_speeds, 
                                                                                req.vehicle_state, req.header.stamp, wm_, ending_state_before_buffer_, debug_msg_, 
                                                                                wpg_detail_config); // Compute the trajectory
  original_trajectory.initial_longitudinal_velocity = std::max(req.vehicle_state.longitudinal_vel, config_.minimum_speed);

  
  if (config_.check_for_stop_intersection)
  {
    // Checking for intersections
    auto stop_intersection_list = wm_->getIntersectionsAlongRoute(veh_pos);
    auto nearest_stop_intersection = stop_intersection_list.front();
    double stop_intersection_down_track =
      wm_->routeTrackPos(nearest_stop_intersection->stopLines().front().front().basicPoint2d()).downtrack;
    
    double time_to_schedule_stop = (scheduled_stop_time_ - street_msg_timestamp_)/1000.0;
    int case_num = determine_case(stop_intersection_down_track, req.vehicle_state.longitudinal_vel, time_to_schedule_stop, speed_limit_);

  }


  if (config_.enable_object_avoidance)
  {
    ROS_DEBUG_STREAM("Activate Object Avoidance");
    if (yield_client_ && yield_client_.exists() && yield_client_.isValid())
    {
      ROS_DEBUG_STREAM("Yield Client is valid");
      cav_srvs::PlanTrajectory yield_srv;
      yield_srv.request.initial_trajectory_plan = original_trajectory;
      yield_srv.request.vehicle_state = req.vehicle_state;

      if (yield_client_.call(yield_srv))
      {
        ROS_DEBUG_STREAM("Received Traj from Yield");
        cav_msgs::TrajectoryPlan yield_plan = yield_srv.response.trajectory_plan;
        if (validate_yield_plan(yield_plan))
        {
          ROS_DEBUG_STREAM("Yield trajectory validated");
          resp.trajectory_plan = yield_plan;
        }
        else
        {
          throw std::invalid_argument("Invalid Yield Trajectory");
        }
      }
      else
      {
        throw std::invalid_argument("Unable to Call Yield Plugin");
      }
    }
    else
    {
      throw std::invalid_argument("Yield Client is unavailable");
    }
    
  }
  else
  {
    ROS_DEBUG_STREAM("Ignored Object Avoidance");
    resp.trajectory_plan = original_trajectory;
  }

  if (config_.publish_debug) { // Publish the debug message if in debug logging mode
    debug_msg_.trajectory_plan = resp.trajectory_plan;
    debug_publisher_(debug_msg_); 
  }
  
  resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

  ros::WallTime end_time = ros::WallTime::now();  // Planning complete

  ros::WallDuration duration = end_time - start_time;
  ROS_DEBUG_STREAM("ExecutionTime: " << duration.toSec());

  return true;
}

void InLaneCruisingPlugin::set_yield_client(ros::ServiceClient& client)
{
  yield_client_ = client;
}

bool InLaneCruisingPlugin::validate_yield_plan(const cav_msgs::TrajectoryPlan& yield_plan)
{
  if (yield_plan.trajectory_points.size()>= 2)
  {
    ROS_DEBUG_STREAM("Yield Trajectory Time" << (double)yield_plan.trajectory_points[0].target_time.toSec());
    ROS_DEBUG_STREAM("Now:" << (double)ros::Time::now().toSec());
    if (yield_plan.trajectory_points[0].target_time + ros::Duration(5.0) > ros::Time::now())
    {
      return true;
    }
    else
    {
      ROS_DEBUG_STREAM("Old Yield Trajectory");
    }
  }
  else
  {
    ROS_DEBUG_STREAM("Invalid Yield Trajectory"); 
  }
  return false;
}

void InLaneCruisingPlugin::mob_op_cb(const cav_msgs::MobilityOperationConstPtr& msg)
{
  if ((msg->strategy == stop_controlled_intersection_strategy_) && (msg->strategy_params != previous_strategy_params_))
  {

    parse_strategy_params(msg->strategy_params);
    
  }
}

void InLaneCruisingPlugin::parse_strategy_params(const std::string& strategy_params)
{
  std::istringstream strategy_params_ss(strategy_params);
  boost::property_tree::ptree parser, child;
  boost::property_tree::json_parser::read_json(strategy_params_ss, parser);
  child = parser.get_child("schedule_plan");
  for(auto& p : child)
  {
    if (p.first == "metadata")
    {
      street_msg_timestamp_ = p.second.get<uint32_t>("timestamp");
      std::cout << "timestamp: " << street_msg_timestamp_ << std::endl;
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

int InLaneCruisingPlugin::determine_case(double stop_dist, double current_speed, double schedule_stop_time, double speed_limit)
{
  int case_num = 0;
  double estimated_stop_time = calc_estimated_stop_time(stop_dist, current_speed);
  
  ROS_DEBUG_STREAM("estimated_stop_time: " << estimated_stop_time);
  if (estimated_stop_time < schedule_stop_time)
  {
    case_num = 3;
  }
  else
  {
    double speed_before_stop = calc_stop_speed(estimated_stop_time, stop_dist, current_speed);
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


double InLaneCruisingPlugin::calc_estimated_stop_time(double stop_dist, double current_speed)
{
  
  double t_stop = 0;
  t_stop = 2*stop_dist/current_speed;
  return t_stop;
}

double InLaneCruisingPlugin::calc_stop_speed(double stop_time, double stop_dist, double current_speed)
{
  double stop_speed = 0;

  double desired_acceleration = config_.max_accel;
  double desired_deceleration = -1*config_.max_accel;

  double sqr_term = sqrt(pow(1 - (desired_acceleration/desired_deceleration), 2) * pow(stop_dist/stop_time, 2) 
                        + (1 - (desired_acceleration/desired_deceleration))*(current_speed*current_speed - 2*current_speed*stop_dist/stop_time));

  stop_speed = (stop_dist/stop_time) + (sqr_term)/(1 - (desired_acceleration/desired_deceleration));

  return stop_speed;
}


}  // namespace inlanecruising_plugin