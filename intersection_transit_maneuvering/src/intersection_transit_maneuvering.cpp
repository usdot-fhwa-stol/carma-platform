#pragma once

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
#include "intersection_transit_maneuvering.h"

using oss = std::ostringstream;

namespace intersection_transit_maneuvering
{
    IntersectionTransitManeuvering::IntersectionTransitManeuvering();


    void IntersectionTransitManeuvering::initialize()
    {
         nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh2_.reset(new ros::CARMANodeHandle("/"));

        trajectory_srv_ = nh_->advertiseService("plan_trajectory",&IntersectionTransitManeuvering::plan_trajectory_cb, this);
        
        plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery",1);
        jerk_pub_ = nh_->advertise<std_msgs::Float64>("jerk",1);
        plugin_discovery_msg_.name = "IntersectionTransitManeuvering";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
        
        pose_sub_ = nh_->subscribe("current_pose",1, &IntersectionTransitManeuvering::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &IntersectionTransitManeuvering::twist_cb, this);

        wml_.reset(new carma_wm::WMListener());
        wm_ = wml_->getWorldModel();
        
        pnh_->param<double>("minimal_trajectory_duration", minimal_trajectory_duration_);
        pnh_->param<double>("max_jerk_limit", max_jerk_limit_);
        pnh_->param<double>("min_timestep",min_timestep_);
        pnh_->param<double>("min_jerk", min_jerk_limit_);

        discovery_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto&) { 
                plugin_discovery_pub_.publish(plugin_discovery_msg_);
                std_msgs::Float64 jerk_msg;
                jerk_msg.data = jerk_;
                jerk_pub_.publish(jerk_msg);
             });
    }



    bool IntersectionTransitManeuvering::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
    {/*Place Code Here*/}

    void IntersectionTransitManeuvering::run()
    {/**/}

    std::vector<PointSpeedPair> IntersectionTransitManeuvering::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers, double max_starting_downtrack, const carma_wm::WorldModelConstPtr& wm)
    {/**/}

    std::vector<cav_msgs::TrajectoryPlanPoint> IntersectionTransitManeuvering::compose_trajectory_from_centerline(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state, const ros::Time& state_time);
    {/**/}

    std::vector<double> IntersectionTransitManeuvering::optimize_speed(const std::vector<double>& downtracks, const std::vector<double>& curve_speeds, double accel_limit);
    {
        if (downtracks.size() != curve_speeds.size())
        {
            throw std::invalid_argument("Downtracks and speeds do not have the same size");
        }

      if (accel_limit <= 0)
      {
       throw std::invalid_argument("Accel limits should be positive");
      }

    bool optimize = true;
    std::unordered_set<size_t> visited_idx;
     visited_idx.reserve(curve_speeds.size());

    std::vector<double> output = curve_speeds;

    while (optimize)
    {
        auto min_pair = min_with_exclusions(curve_speeds, visited_idx);
        size_t min_idx = std::get<1>(min_pair);
        if (min_idx == -1) {
          break;
        }

        visited_idx.insert(min_idx); // Mark this point as visited

        double v_i = std::get<0>(min_pair);
        double x_i = downtracks[min_idx];
        for (int i = min_idx - 1; i > 0; i--) { // NOTE: Do not use size_t for i type here as -- with > 0 will result in overflow
                                            //       First point's speed is left unchanged as it is current speed of the vehicle
          double v_f = curve_speeds[i];
          double dv = v_f - v_i;
      
          double x_f = downtracks[i];
          double dx = x_f - x_i;

          if(dv > 0) {
            v_f = std::min(v_f, sqrt(v_i * v_i - 2 * accel_limit * dx)); // inverting accel as we are only visiting deceleration case
            visited_idx.insert(i);
          } else if (dv < 0) {
            break;
          }
          output[i] = v_f;
          v_i = v_f;
          x_i = x_f;
        }
    }

      log::printDoublesPerLineWithPrefix("only_reverse[i]: ", output);
  
      output = trajectory_utils::apply_accel_limits_by_distance(downtracks, output, accel_limit, accel_limit);
      log::printDoublesPerLineWithPrefix("after_forward[i]: ", output);

      return output;
}









}