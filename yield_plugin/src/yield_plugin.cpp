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
                                            PublishPluginDiscoveryCB plugin_discovery_publisher)
    : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher)
  {
    plugin_discovery_msg_.name = "YieldPlugin";
    plugin_discovery_msg_.versionId = "v1.0";
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

  bool YieldPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req,
                                              cav_srvs::PlanTrajectoryResponse& resp)
  {
    if (req.initial_trajectory_plan.trajectory_points.size() < 2){
      throw std::invalid_argument("Empty Trajectory received");
    }
    cav_msgs::TrajectoryPlan original_trajectory = req.initial_trajectory_plan;
    cav_msgs::TrajectoryPlan trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = original_trajectory.header.stamp;// should it be now or the original plan's time? 
    trajectory.trajectory_id = original_trajectory.trajectory_id;

    trajectory = update_traj_for_object(original_trajectory, req.vehicle_state.longitudinal_vel); // Compute the trajectory
    trajectory.initial_longitudinal_velocity = trajectory.initial_longitudinal_velocity;//std::max(req.vehicle_state.longitudinal_vel, config_.minimum_speed);//????  find better value

    resp.trajectory_plan = trajectory;
  }

  
  cav_msgs::TrajectoryPlan YieldPlugin::update_traj_for_object(const cav_msgs::TrajectoryPlan& original_tp, double current_speed_) 
  {
        
    cav_msgs::TrajectoryPlan update_tpp_vector;
    geometry_msgs::Twist current_velocity;
    current_velocity.linear.x = current_speed_;

    std::vector<cav_msgs::RoadwayObstacle> rwol = wm_->getRoadwayObjects();
    cav_msgs::RoadwayObstacleList rwol2;
    rwol2.roadway_obstacles = rwol;
    host_vehicle_size.x = config_.vehicle_length;
    host_vehicle_size.y = config_.vehicle_width;
    host_vehicle_size.z = config_.vehicle_height; 
    // std::cout << "host_vehicle_size" << host_vehicle_size.x << std::endl;
    std::vector<cav_msgs::RoadwayObstacle> rwol_collision = carma_wm::collision_detection::WorldCollisionDetection(rwol2, original_tp, host_vehicle_size, current_velocity, config_.collision_horizon);
    std::cout << "rwol" << rwol.size() << std::endl;

    // correct the input types
    if(rwol_collision.size() > 0)
    {
      ROS_WARN_STREAM("Collision!");

      // Distance from the original trajectory point to the lead vehicle/object
      double dist_x = rwol_collision[0].object.pose.pose.position.x - original_tp.trajectory_points[0].x;
      double dist_y = rwol_collision[0].object.pose.pose.position.y - original_tp.trajectory_points[0].y;
      double x_lead = sqrt(dist_x*dist_x + dist_y*dist_y);

      // roadway object position
      double gap_time = (x_lead - config_.x_gap)/current_speed_;

      double collision_time = 0; //\TODO comming from carma_wm collision detection in future (not used now)

      double goal_velocity = rwol_collision[0].object.velocity.twist.linear.x;
      
      if (goal_velocity <= config_.min_obstacle_speed){
        ROS_WARN_STREAM("The obstacle is not moving");
      }

      double goal_pos = x_lead - goal_velocity * gap_time; 

      double initial_time = 0;
      double initial_pos = 0.0; //relative initial position (first trajectory point)

      double initial_accel = 0;
      double goal_accel = 0;

      double delta_v_max = abs(rwol_collision[0].object.velocity.twist.linear.x - max_trajectory_speed(original_tp.trajectory_points));
      // reference time, is the maximum time available to perform object avoidance (length of a trajectory)
      double t_ref = (original_tp.trajectory_points[original_tp.trajectory_points.size() - 1].target_time.toSec() - original_tp.trajectory_points[0].target_time.toSec());
      // time required for comfortable deceleration
      double t_ph = config_.acceleration_adjustment_factor * delta_v_max / config_.maximum_deceleration_value;

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

      std::vector<double> values = quintic_coefficient_calculator::quintic_coefficient_calculator(initial_pos, 
                                                                                                  goal_pos,
                                                                                                  current_speed_, 
                                                                                                  goal_velocity, 
                                                                                                  initial_accel, 
                                                                                                  goal_accel, 
                                                                                                  initial_time, 
                                                                                                  tp);

      std::vector<cav_msgs::TrajectoryPlanPoint> new_trajectory_points;
      new_trajectory_points.push_back(original_tp.trajectory_points[0]);
      std::vector<double> original_traj_downtracks = get_relative_downtracks(original_tp);
      
      for(size_t i = 1; i < original_tp.trajectory_points.size(); i++ )
      {
        double traj_target_time = i * tp / original_tp.trajectory_points.size();
        double dt_dist = polynomial_calc(values, traj_target_time);
        double dv = polynomial_calc_d(values, traj_target_time);
        cav_msgs::TrajectoryPlanPoint new_tpp;
        // the last moving trajectory point
        if (dv >= 1.0)
        {
          ROS_WARN_STREAM("target speed is positive");
          if (dv >= current_speed_){
            dv = current_speed_;
          }
          // trajectory point is copied to move all the available information, then its target time is updated
          new_tpp = original_tp.trajectory_points[i];
          new_tpp.target_time = new_trajectory_points[i-1].target_time + ros::Duration(original_traj_downtracks[i]/dv);
          new_trajectory_points.push_back(new_tpp);
        }
        else
        {
          ROS_WARN_STREAM("target speed is zero");
          // if speed is zero, the vehicle will stay in previous location.
          new_tpp = new_trajectory_points[i-1];
          new_tpp.target_time = new_trajectory_points[0].target_time + ros::Duration(traj_target_time);
          new_trajectory_points.push_back(new_tpp);
        }
      }
      update_tpp_vector.header = original_tp.header;
      update_tpp_vector.trajectory_id = original_tp.trajectory_id;
      update_tpp_vector.trajectory_points = new_trajectory_points;
      return update_tpp_vector;
    }
    return original_tp;
  }


  std::vector<double> YieldPlugin::get_relative_downtracks(const cav_msgs::TrajectoryPlan& trajectory_plan)
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

  double YieldPlugin::polynomial_calc(std::vector<double> coeff, double x)
  {
    double result = 0;
    for (size_t i = 0; i < coeff.size(); i++)
    {
      double value = coeff[i] * pow(x, (int)(coeff.size() - 1 - i));
      result = result + value;
    }
    return result;
  }

  double YieldPlugin::polynomial_calc_d(std::vector<double> coeff, double x)
  {
    double result = 0;
    for (size_t i = 0; i < coeff.size()-1; i++) 
    {
      double value = (int)(coeff.size() - 1 - i) * coeff[i] * pow(x, (int)(coeff.size() - 2 - i));
      result = result + value;
    }
    return result;
  }

  double YieldPlugin::max_trajectory_speed(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points) 
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




}  // namespace yield_plugin