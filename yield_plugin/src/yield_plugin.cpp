/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include <yield_plugin/yield_plugin.h>



using oss = std::ostringstream;

namespace yield_plugin
{
  YieldPlugin::YieldPlugin(carma_wm::WorldModelConstPtr wm, YieldPluginConfig config,
                                            PublishPluginDiscoveryCB plugin_discovery_publisher, 
                                            MobilityResponseCB mobility_response_publisher,
                                            LaneChangeStatusCB lc_status_publisher)
    : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher), 
      mobility_response_publisher_(mobility_response_publisher), lc_status_publisher_(lc_status_publisher)
  {
    plugin_discovery_msg_.name = "YieldPlugin";
    plugin_discovery_msg_.version_id = "v1.0";
    plugin_discovery_msg_.available = true;
    plugin_discovery_msg_.activated = false;
    plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
    plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";   
  }

  bool YieldPlugin::onSpin() 
  {
    plugin_discovery_publisher_(plugin_discovery_msg_);
    return true;
  }

  std::vector<std::pair<int, lanelet::BasicPoint2d>> YieldPlugin::detect_trajectories_intersection(std::vector<lanelet::BasicPoint2d> self_trajectory, std::vector<lanelet::BasicPoint2d> incoming_trajectory) const
  {
    std::vector<std::pair<int, lanelet::BasicPoint2d>> intersection_points;
    boost::geometry::model::linestring<lanelet::BasicPoint2d> self_traj;
    for (auto tpp:self_trajectory)
    {
      boost::geometry::append(self_traj, tpp);
    }
    // distance to consider trajectories colliding (chosen based on lane width and vehicle size)
    for (size_t i=0; i<incoming_trajectory.size(); i++)
    {
      double res = boost::geometry::distance(incoming_trajectory[i], self_traj);
    
      if (fabs(res) <= config_.intervehicle_collision_distance)
      {
         intersection_points.push_back(std::make_pair(i, incoming_trajectory[i]));
      }
    }
    return intersection_points;
  }

  std::vector<lanelet::BasicPoint2d> YieldPlugin::convert_eceftrajectory_to_mappoints(const cav_msgs::Trajectory& ecef_trajectory) const
  {
    cav_msgs::TrajectoryPlan trajectory_plan;
    std::vector<lanelet::BasicPoint2d> map_points;

    lanelet::BasicPoint2d first_point = ecef_to_map_point(ecef_trajectory.location);

    map_points.push_back(first_point);
    auto curr_point = ecef_trajectory.location;

    for (size_t i = 0; i<ecef_trajectory.offsets.size(); i++)
    {
      lanelet::BasicPoint2d offset_point;
      curr_point.ecef_x += ecef_trajectory.offsets[i].offset_x;
      curr_point.ecef_y += ecef_trajectory.offsets[i].offset_y;
      curr_point.ecef_z += ecef_trajectory.offsets[i].offset_z;

      offset_point = ecef_to_map_point(curr_point);
      
      map_points.push_back(offset_point);
    }
    
    return map_points;
  }

  lanelet::BasicPoint2d YieldPlugin::ecef_to_map_point(const cav_msgs::LocationECEF& ecef_point) const
  {

    if (!map_projector_) {
        throw std::invalid_argument("No map projector available for ecef conversion");
    }
      
    lanelet::BasicPoint3d map_point = map_projector_->projectECEF( { (double)ecef_point.ecef_x/100.0, (double)ecef_point.ecef_y/100.0, (double)ecef_point.ecef_z/100.0 } , 1);
    
    return lanelet::traits::to2D(map_point);
  } 
    
  

  cav_msgs::MobilityResponse YieldPlugin::compose_mobility_response(const std::string& resp_recipient_id, const std::string& req_plan_id, bool response) const
  {
    cav_msgs::MobilityResponse out_mobility_response;
    out_mobility_response.m_header.sender_id = config_.vehicle_id;
    out_mobility_response.m_header.recipient_id = resp_recipient_id;
    out_mobility_response.m_header.sender_bsm_id = host_bsm_id_;
    out_mobility_response.m_header.plan_id = req_plan_id;
    out_mobility_response.m_header.timestamp = ros::Time::now().toSec()*1000;


    if (config_.always_accept_mobility_request && response)
    {
      out_mobility_response.is_accepted = true;
    }
    else out_mobility_response.is_accepted = false;
    
    return out_mobility_response;
  }


  void YieldPlugin::mobilityrequest_cb(const cav_msgs::MobilityRequestConstPtr& msg)
  {
    cav_msgs::MobilityRequest incoming_request = *msg;
    cav_msgs::LaneChangeStatus lc_status_msg;
    if (incoming_request.strategy == "carma/cooperative-lane-change")
    {
      if (!map_projector_) {
        ROS_ERROR_STREAM("Cannot process mobility request as map projection is not yet set!");
        return;
      }
      if (incoming_request.plan_type.type == cav_msgs::PlanType::CHANGE_LANE_LEFT || incoming_request.plan_type.type == cav_msgs::PlanType::CHANGE_LANE_RIGHT)
      {
        ROS_DEBUG_STREAM("Cooperative Lane Change Request Received");
        lc_status_msg.status = cav_msgs::LaneChangeStatus::REQUEST_RECEIVED;
        lc_status_msg.description = "Received lane merge request";
        if (incoming_request.m_header.recipient_id == config_.vehicle_id)
        {
          ROS_DEBUG_STREAM("CLC Request correctly received");
        }
        // extract mobility header
        std::string req_sender_id = incoming_request.m_header.sender_id;
        std::string req_plan_id = incoming_request.m_header.plan_id;
        // extract mobility request
        cav_msgs::LocationECEF ecef_location = incoming_request.location;
        cav_msgs::Trajectory incoming_trajectory = incoming_request.trajectory;
        std::string req_strategy_params = incoming_request.strategy_params;
        clc_urgency_ = incoming_request.urgency;
        ROS_DEBUG_STREAM("received urgency: " << clc_urgency_);

        // Parse strategy parameters
        using boost::property_tree::ptree;
        ptree pt;
        std::istringstream strstream(req_strategy_params);
        boost::property_tree::json_parser::read_json(strstream, pt);
        int req_traj_speed_full = pt.get<int>("s");
        int req_traj_fractional = pt.get<int>("f");
        int start_lanelet_id = pt.get<int>("sl");
        int end_lanelet_id = pt.get<int>("el");
        double req_traj_speed = (double)req_traj_speed_full + (double)(req_traj_fractional)/10.0;
        ROS_DEBUG_STREAM("req_traj_speed" << req_traj_speed);
        ROS_DEBUG_STREAM("start_lanelet_id" << start_lanelet_id);
        ROS_DEBUG_STREAM("end_lanelet_id" << end_lanelet_id);

        std::vector<lanelet::BasicPoint2d> req_traj_plan = {};

        req_traj_plan = convert_eceftrajectory_to_mappoints(incoming_trajectory);

        double req_expiration_sec = (double)incoming_request.expiration;
        double current_time_sec = ros::Time::now().toSec();

        bool response_to_clc_req = false;
        // ensure there is enough time for the yield
        double req_plan_time = req_expiration_sec - current_time_sec;
        double req_timestamp = (double)incoming_request.m_header.timestamp / 1000.0 - current_time_sec;
        set_incoming_request_info(req_traj_plan, req_traj_speed, req_plan_time, req_timestamp);

        
        if (req_expiration_sec - current_time_sec >= config_.tpmin && cooperative_request_acceptable_)
        {
          timesteps_since_last_req_ = 0;
          lc_status_msg.status = cav_msgs::LaneChangeStatus::REQUEST_ACCEPTED;
          lc_status_msg.description = "Accepted lane merge request";
          response_to_clc_req = true;  
          ROS_DEBUG_STREAM("CLC accepted"); 
        }
        else
        {
          lc_status_msg.status = cav_msgs::LaneChangeStatus::REQUEST_REJECTED;
          lc_status_msg.description = "Rejected lane merge request";
          response_to_clc_req = false;
          ROS_DEBUG_STREAM("CLC rejected"); 
        }
        cav_msgs::MobilityResponse outgoing_response = compose_mobility_response(req_sender_id, req_plan_id, response_to_clc_req);
        mobility_response_publisher_(outgoing_response);
        lc_status_msg.status = cav_msgs::LaneChangeStatus::RESPONSE_SENT;
        ROS_DEBUG_STREAM("response sent"); 
      }
    }
    lc_status_publisher_(lc_status_msg);
    
  }

  void YieldPlugin::set_incoming_request_info(std::vector <lanelet::BasicPoint2d> req_trajectory, double req_speed, double req_planning_time, double req_timestamp)
  {
    req_trajectory_points_ = req_trajectory;
    req_target_speed_ = req_speed;
    req_target_plan_time_ = req_planning_time;
    ROS_DEBUG_STREAM("req_target_plan_time_" << req_target_plan_time_); 
    req_timestamp_ = req_timestamp;
  }


  void YieldPlugin::bsm_cb(const cav_msgs::BSMConstPtr& msg)
  {
    cav_msgs::BSMCoreData bsm_core_ = msg->core_data;
    host_bsm_id_ = bsmIDtoString(bsm_core_);
  }

  bool YieldPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req,
                                              cav_srvs::PlanTrajectoryResponse& resp)
  {
    if (req.initial_trajectory_plan.trajectory_points.size() < 2){
      throw std::invalid_argument("Empty Trajectory received by Yield");
    }
    cav_msgs::TrajectoryPlan original_trajectory = req.initial_trajectory_plan;
    cav_msgs::TrajectoryPlan yield_trajectory;

    // seperating cooperative yield with regular object detection for better performance.
    if (config_.enable_cooperative_behavior && clc_urgency_ > config_.acceptable_urgency)
    {
      ROS_DEBUG_STREAM("Only consider high urgency clc");
      if (timesteps_since_last_req_ < config_.acceptable_passed_timesteps)
      {
        ROS_DEBUG_STREAM("Yield for CLC. We haven't received an updated negotiation this timestep");
        yield_trajectory = update_traj_for_cooperative_behavior(original_trajectory, req.vehicle_state.longitudinal_vel);
        timesteps_since_last_req_++;
      }
      else
      {
        ROS_DEBUG_STREAM("unreliable CLC communication, switching to object avoidance");
        yield_trajectory = update_traj_for_object(original_trajectory, req.vehicle_state.longitudinal_vel); // Compute the trajectory
      }    
    }
    else
    {
      ROS_DEBUG_STREAM("Yield for object avoidance");
      yield_trajectory = update_traj_for_object(original_trajectory, req.vehicle_state.longitudinal_vel); // Compute the trajectory
    }
    yield_trajectory.header.frame_id = "map";
    yield_trajectory.header.stamp = ros::Time::now();
    yield_trajectory.trajectory_id = original_trajectory.trajectory_id;
    yield_trajectory.initial_longitudinal_velocity = original_trajectory.initial_longitudinal_velocity;//copy the original trajectory's desired speed for now. 

    resp.trajectory_plan = yield_trajectory;
    return true;
  }

  cav_msgs::TrajectoryPlan YieldPlugin::update_traj_for_cooperative_behavior(const cav_msgs::TrajectoryPlan& original_tp, double current_speed)
  {
    cav_msgs::TrajectoryPlan cooperative_trajectory;

    double initial_pos = 0;
    double goal_pos;
    double initial_velocity = current_speed;
    double goal_velocity = req_target_speed_;
    double planning_time = req_target_plan_time_;

    std::vector<lanelet::BasicPoint2d> host_traj_points = {};
    for (size_t i=0; i<original_tp.trajectory_points.size(); i++)
    {
      lanelet::BasicPoint2d traj_point;
      traj_point.x() = original_tp.trajectory_points[i].x;
      traj_point.y() = original_tp.trajectory_points[i].y;
      host_traj_points.push_back(traj_point);
    }

    std::vector<std::pair<int, lanelet::BasicPoint2d>> intersection_points = detect_trajectories_intersection(host_traj_points, req_trajectory_points_);
    if (!intersection_points.empty())
    {
      lanelet::BasicPoint2d intersection_point = intersection_points[0].second;
      double dx = original_tp.trajectory_points[0].x - intersection_point.x();
      double dy = original_tp.trajectory_points[0].y - intersection_point.y();
      // check if a digital_gap is available
      double digital_gap = check_traj_for_digital_min_gap(original_tp);
      ROS_DEBUG_STREAM("digital_gap: " << digital_gap);
      goal_pos = sqrt(dx*dx + dy*dy) - config_.x_gap;
      ROS_DEBUG_STREAM("Goal position (goal_pos): " << goal_pos);
      double collision_time = req_timestamp_ + (intersection_points[0].first * ecef_traj_timestep_) - config_.safety_collision_time_gap;
      ROS_DEBUG_STREAM("req time stamp: " << req_timestamp_);
      ROS_DEBUG_STREAM("Collision time: " << collision_time);
      ROS_DEBUG_STREAM("intersection num: " << intersection_points[0].first);
      ROS_DEBUG_STREAM("Planning time: " << planning_time);
      // calculate distance traveled from beginning of trajectory to collision point
      double dx2 = intersection_point.x() - req_trajectory_points_[0].x();
      double dy2 = intersection_point.y() - req_trajectory_points_[0].y();
      // calculate incoming trajectory speed from time and distance between trajectory points
      double incoming_trajectory_speed = sqrt(dx2*dx2 + dy2*dy2)/(intersection_points[0].first * ecef_traj_timestep_);
      // calculate goal velocity from request trajectory
      goal_velocity = std::min(goal_velocity, incoming_trajectory_speed);
      double min_time = (initial_velocity - goal_velocity)/config_.yield_max_deceleration;

      ROS_DEBUG_STREAM("goal_velocity: " << goal_velocity);
      ROS_DEBUG_STREAM("incoming_trajectory_speed: " << incoming_trajectory_speed);

      if (planning_time > min_time)
      {
        cooperative_request_acceptable_ = true;
        cooperative_trajectory = generate_JMT_trajectory(original_tp, initial_pos, goal_pos, initial_velocity, goal_velocity, planning_time); 
      }
      else
      {
        cooperative_request_acceptable_ = false;
        ROS_DEBUG_STREAM("The incoming requested trajectory is rejected, due to insufficient gap");
        cooperative_trajectory = original_tp;
      }
                                                              
    }
    else
    {
      cooperative_request_acceptable_ = true;
      ROS_DEBUG_STREAM("The incoming requested trajectory does not overlap with host vehicle's trajectory");
      cooperative_trajectory = original_tp;
    }

    return cooperative_trajectory;
  }

  cav_msgs::TrajectoryPlan YieldPlugin::generate_JMT_trajectory(const cav_msgs::TrajectoryPlan& original_tp, double initial_pos, double goal_pos, double initial_velocity, double goal_velocity, double planning_time)                                                         
  {
    cav_msgs::TrajectoryPlan jmt_trajectory;
    std::vector<cav_msgs::TrajectoryPlanPoint> jmt_trajectory_points;
    jmt_trajectory_points.push_back(original_tp.trajectory_points[0]);

    double initial_time = 0;
    double initial_accel = 0.0;
    double goal_accel = 0.0;

    double original_max_speed = max_trajectory_speed(original_tp.trajectory_points);
    ROS_DEBUG_STREAM("original_max_speed" << original_max_speed);
    std::vector<double> values = quintic_coefficient_calculator::quintic_coefficient_calculator(initial_pos, 
                                                                                                goal_pos,
                                                                                                initial_velocity, 
                                                                                                goal_velocity, 
                                                                                                initial_accel, 
                                                                                                goal_accel, 
                                                                                                initial_time, 
                                                                                                planning_time);
 
    std::vector<double> original_traj_downtracks = get_relative_downtracks(original_tp);
    std::vector<double> calculated_speeds = {};
    calculated_speeds.push_back(initial_velocity);
    for(size_t i = 1; i < original_tp.trajectory_points.size(); i++ )
    {
      double traj_target_time = i * planning_time / original_tp.trajectory_points.size();
      double dt_dist = polynomial_calc(values, traj_target_time);
      double dv = polynomial_calc_d(values, traj_target_time);
      // cav_msgs::TrajectoryPlanPoint jmt_tpp;
      if (dv >= original_max_speed)
      {
        dv = original_max_speed;
      }
      calculated_speeds.push_back(dv);
    }
    // moving average filter
    std::vector<double> filtered_speeds = moving_average_filter(calculated_speeds, config_.speed_moving_average_window_size);

    double prev_speed = filtered_speeds[0];
    for(size_t i = 1; i < original_tp.trajectory_points.size(); i++ )
    {
      cav_msgs::TrajectoryPlanPoint jmt_tpp;
      double current_speed = filtered_speeds[i];
      double traj_target_time = i * planning_time / original_tp.trajectory_points.size();
      if (current_speed >= config_.max_stop_speed)
      {
        double dt = (2 * original_traj_downtracks[i]) / (current_speed + prev_speed);
        jmt_tpp = original_tp.trajectory_points[i];
        jmt_tpp.target_time = jmt_trajectory_points[i-1].target_time + ros::Duration(dt);
        jmt_trajectory_points.push_back(jmt_tpp);
      }
      else
      {
        ROS_DEBUG_STREAM("target speed is zero");
        // if speed is zero, the vehicle will stay in previous location.
        jmt_tpp = jmt_trajectory_points[i-1];
        jmt_tpp.target_time = jmt_trajectory_points[0].target_time + ros::Duration(traj_target_time);
        jmt_trajectory_points.push_back(jmt_tpp);
      }
      prev_speed = current_speed;
    }
    
    jmt_trajectory.header = original_tp.header;
    jmt_trajectory.trajectory_id = original_tp.trajectory_id;
    jmt_trajectory.trajectory_points = jmt_trajectory_points;
    return jmt_trajectory;
  }
  
  cav_msgs::TrajectoryPlan YieldPlugin::update_traj_for_object(const cav_msgs::TrajectoryPlan& original_tp, double initial_velocity) 
  {
        
    cav_msgs::TrajectoryPlan update_tpp_vector;
    geometry_msgs::Twist current_velocity;
    current_velocity.linear.x = initial_velocity;

    std::vector<cav_msgs::RoadwayObstacle> rwol = wm_->getRoadwayObjects();
    cav_msgs::RoadwayObstacleList rwol2;
    rwol2.roadway_obstacles = rwol;
    host_vehicle_size.x = config_.vehicle_length;
    host_vehicle_size.y = config_.vehicle_width;
    host_vehicle_size.z = config_.vehicle_height; 


    std::vector<cav_msgs::RoadwayObstacle> rwol_collision;

    // std::vector<cav_msgs::RoadwayObstacle> rwol_collision = carma_wm::collision_detection::WorldCollisionDetection(rwol2, original_tp, host_vehicle_size, current_velocity, config_.collision_horizon);
    
    lanelet::BasicPoint2d point(original_tp.trajectory_points[0].x,original_tp.trajectory_points[0].y);
    double vehicle_downtrack = wm_->routeTrackPos(point).downtrack;
    
    ROS_DEBUG_STREAM("vehicle_downtrack");
    ROS_DEBUG_STREAM(vehicle_downtrack);

    for (auto i : rwol2.roadway_obstacles){

      lanelet::BasicPoint2d point_o(i.object.pose.pose.position.x, i.object.pose.pose.position.y);
      double object_down_track = wm_->routeTrackPos(point_o).downtrack;

      ROS_DEBUG_STREAM("object_down_track");
      ROS_DEBUG_STREAM(object_down_track);
      
      ROS_DEBUG_STREAM("vehicle_downtrack - object_down_track");
      ROS_DEBUG_STREAM(vehicle_downtrack - object_down_track);
      
      ROS_DEBUG_STREAM("i.object.velocity.twist.linear.x");
      ROS_DEBUG_STREAM(i.object.velocity.twist.linear.x);

      if(current_velocity.linear.x > 0.0) {
        
          // std::abs might not be needed cause vehicles in the behind of vehicle to cause problem
        
          ROS_DEBUG_STREAM("std::abs(vehicle_downtrack - object_down_track)/current_velocity.linear.x");

          ROS_DEBUG_STREAM(object_down_track - vehicle_downtrack/current_velocity.linear.x);

          if((object_down_track - vehicle_downtrack)/current_velocity.linear.x < config_.collision_horizon) {
              rwol_collision.push_back(i);
          }
      }

    }

    ROS_DEBUG_STREAM("Roadway Object List (rwol) size: " << rwol.size());

    // correct the input types
    if(!rwol_collision.empty())
    {
      ROS_WARN_STREAM("Collision Detected!");

      // Distance from the original trajectory point to the lead vehicle/object
      double dist_x = rwol_collision[0].object.pose.pose.position.x - original_tp.trajectory_points[0].x;
      double dist_y = rwol_collision[0].object.pose.pose.position.y - original_tp.trajectory_points[0].y;
      double x_lead = sqrt(dist_x*dist_x + dist_y*dist_y);

      // roadway object position
      double gap_time = (x_lead - config_.x_gap)/initial_velocity;

      double collision_time = 0; //\TODO comming from carma_wm collision detection in future (CAR 4288)

      double goal_velocity = rwol_collision[0].object.velocity.twist.linear.x;
      // determine the safety inter-vehicle gap based on speed
      double safety_gap = std::max(goal_velocity * gap_time, config_.x_gap);
      if (config_.enable_adjustable_gap)
      {
        // check if a digital_gap is available
        double digital_gap = check_traj_for_digital_min_gap(original_tp);
        ROS_DEBUG_STREAM("digital_gap: " << digital_gap);
        // if a digital gap is available, it is replaced as safety gap 
        safety_gap = std::max(safety_gap, digital_gap);
      }
      // safety gap is implemented
      double goal_pos = x_lead - safety_gap; 

      if (goal_velocity <= config_.min_obstacle_speed){
        ROS_WARN_STREAM("The obstacle is not moving");
      }


      double initial_time = 0;
      double initial_pos = 0.0; //relative initial position (first trajectory point)

      double initial_accel = 0;
      double goal_accel = 0;

      double delta_v_max = fabs(rwol_collision[0].object.velocity.twist.linear.x - max_trajectory_speed(original_tp.trajectory_points));
      // reference time, is the maximum time available to perform object avoidance (length of a trajectory)
      double t_ref = (original_tp.trajectory_points[original_tp.trajectory_points.size() - 1].target_time.toSec() - original_tp.trajectory_points[0].target_time.toSec());
      // time required for comfortable deceleration
      double t_ph = config_.acceleration_adjustment_factor * delta_v_max / config_.yield_max_deceleration;

      // planning time for object avoidance
      double tp = 0;

      if(t_ph > config_.tpmin && t_ref > t_ph)
      {
        tp = t_ph;
      }
      else if(t_ph < config_.tpmin)
      {
        tp = config_.tpmin;
      }
      else
      {
        tp = t_ref;
      }
      
      ROS_DEBUG_STREAM("Object avoidance planning time: " << tp);

      update_tpp_vector = generate_JMT_trajectory(original_tp, initial_pos, goal_pos, initial_velocity, goal_velocity, tp);

      return update_tpp_vector;
    }
    ROS_DEBUG_STREAM("No collision detection, so trajectory not modified.");
    return original_tp;
  }


  std::vector<double> YieldPlugin::get_relative_downtracks(const cav_msgs::TrajectoryPlan& trajectory_plan) const
  {
    std::vector<double> downtracks;
    downtracks.reserve(trajectory_plan.trajectory_points.size());
    // relative downtrack distance of the fist point is 0.0
    downtracks[0] = 0.0;
    for (size_t i=1; i < trajectory_plan.trajectory_points.size(); i++){
      double dx = trajectory_plan.trajectory_points[i].x - trajectory_plan.trajectory_points[i-1].x;
      double dy = trajectory_plan.trajectory_points[i].y - trajectory_plan.trajectory_points[i-1].y;
      downtracks[i] = sqrt(dx*dx + dy*dy);
    }
    return downtracks;
  }

  double YieldPlugin::polynomial_calc(std::vector<double> coeff, double x) const
  {
    double result = 0;
    for (size_t i = 0; i < coeff.size(); i++)
    {
      double value = coeff[i] * pow(x, (int)(coeff.size() - 1 - i));
      result = result + value;
    }
    return result;
  }

  double YieldPlugin::polynomial_calc_d(std::vector<double> coeff, double x) const
  {
    double result = 0;
    for (size_t i = 0; i < coeff.size()-1; i++) 
    {
      double value = (int)(coeff.size() - 1 - i) * coeff[i] * pow(x, (int)(coeff.size() - 2 - i));
      result = result + value;
    }
    return result;
  }

  double YieldPlugin::max_trajectory_speed(const std::vector<cav_msgs::TrajectoryPlanPoint>& trajectory_points) const
  {
    double max_speed = 0;
    for(size_t i = 0; i < trajectory_points.size() - 2; i++ )
    {
      double dx = trajectory_points[i + 1].x - trajectory_points[i].x;
      double dy = trajectory_points[i + 1].y - trajectory_points[i].y;
      double d = sqrt(dx*dx + dy*dy); 
      double t = (trajectory_points[i + 1].target_time.toSec() - trajectory_points[i].target_time.toSec());
      double v = d/t;
      if(v > max_speed)
      {
        max_speed = v;
      }
    }
    return max_speed;
  }

  double YieldPlugin::check_traj_for_digital_min_gap(const cav_msgs::TrajectoryPlan& original_tp) const
  {
    double desired_gap = 0;

    for (size_t i = 0; i < original_tp.trajectory_points.size(); i++)
    {
      lanelet::BasicPoint2d veh_pos(original_tp.trajectory_points[i].x, original_tp.trajectory_points[i].y);
      auto llts = wm_->getLaneletsFromPoint(veh_pos, 1);
      if (llts.empty())
      {
        ROS_WARN_STREAM("Trajectory point: " << original_tp.trajectory_points[i]);
        throw std::invalid_argument("Trajectory point is not on a valid lanelet.");
      }
      auto digital_min_gap = llts[0].regulatoryElementsAs<lanelet::DigitalMinimumGap>(); //Returns a list of these elements)
      if (!digital_min_gap.empty()) 
      {
        double digital_gap = digital_min_gap[0]->getMinimumGap(); // Provided gap is in meters
        ROS_DEBUG_STREAM("Digital Gap found with value: " << digital_gap);
        desired_gap = std::max(desired_gap, digital_gap);
      }
    }
    return desired_gap;
  }

  void YieldPlugin::georeferenceCallback(const std_msgs::StringConstPtr& msg) 
  {
      map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str());  // Build projector from proj string
  }

}  // namespace yield_plugin
