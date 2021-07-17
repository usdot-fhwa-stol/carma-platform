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
#include <intersection_transit_maneuvering.h>

using oss = std::ostringstream;

namespace intersection_transit_maneuvering
{
    IntersectionTransitManeuvering::IntersectionTransitManeuvering(carma_wm::WorldModelConstPtr wm)
    {        
        plugin_discovery_msg_.name = "IntersectionTransitManeuvering";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
    }

    bool IntersectionTransitManeuvering::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
    {
        
        ros::WallTime start_time = ros::WallTime::now();  // Start timeing the execution time for planning so it can be logged

        lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
        double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

        std::vector<cav_msgs::Maneuver> maneuver_plan;
        for(size_t i = req.maneuver_index_to_plan; i < req.maneuver_plan.maneuvers.size(); i++)
        {
            if(req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ||
            req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT || 
            req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT )
            {
                maneuver_plan.push_back(req.maneuver_plan.maneuvers[i]);
                resp.related_maneuvers.push_back(i);
            }
            else
                {
                    break;
                }
        }
        cav_srvs::PlanTrajectoryRequest& req2;
        auto new_plan = convert_maneuver_plan(maneuver_plan);

        for(const auto& man : new_plan)
            {
                req2.maneuver_plan.push_back(man);
            }

        /*Once the maneuvers have been successfully converted, call the inlanecruising plugin to calculate the trajectory*/

        if (trajectory_client_ && trajectory_client_.exists() && trajectory_client_.isValid())
        {
            ROS_DEBUG_STREAM("Trajectory Client is valid");
            cav_srvs::PlanTrajectory traj_srv;
            traj_srv.request.initial_trajectory_plan = original_trajectory;
            traj_srv.request.vehicle_state = req.vehicle_state;

            if (trajectory_client_.call(traj_srv))
            {
                ROS_DEBUG_STREAM("Received Traj from InlaneCruisingPlugin");
                cav_msgs::TrajectoryPlan traj_plan = traj_srv.response.trajectory_plan;
                if (validate_trajectory_plan(traj_plan))
                {
                    ROS_DEBUG_STREAM("Inlane Cruising trajectory validated");
                    resp.trajectory_plan = traj_plan;
                }
                else
                {
                    throw std::invalid_argument("Invalid Trajectory");
                }
            }
            else
            {
                throw std::invalid_argument("Unable to Call InlaneCruising Plugin");
            }
        }


        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);
        ros::WallTime end_time = ros::WallTime::now();

        ros::WallDuration duration = end_time - start_time;
        ROS_DEBUG_STREAM("ExecutionTime: " << duration.toSec());
        
        return true;
    }
   
    std::vector<cav_msgs::Maneuver> IntersectionTransitManeuvering::convert_maneuver_plan(const std::vector<cav_msgs::Maneuver>& maneuvers)
    {
        if (maneuvers.size().isEmpty())
        {
            throw std::invalid_argument("No maneuvers to convert");
        }

        std::vector<cav_msgs::Maneuver> new_maneuver_plan;
        cav_msgs:Maneuver new_maneuver;
        new_maneuver.type = cav_msgs::Maneuver::LANE_FOLLOWING; //All of the converted maneuvers will be of type LANE_FOLLOWING
        for(const auto& maneuver : maneuvers)
        {
            /*Throw exception if the manuever type does not match INTERSECTION_TRANSIT*/
            if ( maneuver.type != cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ||
                maneuver.type != cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ||
                maneuver.type != cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ||)
                {
                    throw std::invalid_argument("Intersection transit maneuvering does not support this maneuver type");
                }
            
            /*Convert IT Straight*/
            if (maneuver.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT)
            {
                new_maneuver.lane_following_maneuver.parameters = maneuver.intersection_transit_straight_maneuver.parameters;

                new_maneuver.lane_following_maneuver.start_dist = maneuver.intersection_transit_straight_maneuver.start_dist;
                new_maneuver.lane_following_maneuver.start_speed = maneuver.intersection_transit_straight_maneuver.start_speed;
                new_maneuver.lane_following_maneuver.start_time = maneuver.intersection_transit_straight_maneuver.start_time;

                new_maneuver.lane_following_maneuver.end_dist = maneuver.intersection_transit_straight_maneuver.end_dist;
                new_maneuver.lane_following_maneuver.end_speed = maneuver.intersection_transit_straight_maneuver.end_speed;
                new_maneuver.lane_following_maneuver.end_time = maneuver.intersection_transit_straight_maneuver.end_time;

                new_maneuver.lane_following_maneuver.lane_id = maneuver.intersection_transit_straight_maneuver.lane_id;

                new_maneuver_plan.push_back(new_maneuver);
            }

             /*Convert IT LEFT TURN*/
            if (maneuver.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN)
            {
                new_maneuver.parameters = maneuver.intersection_transit_left_turn_maneuver.parameters;

                new_maneuver.lane_following_maneuver.start_dist = maneuver.intersection_transit_left_turn_maneuver.start_dist;
                new_maneuver.lane_following_maneuver.start_speed = maneuver.intersection_transit_left_turn_maneuver.start_speed;
                new_maneuver.lane_following_maneuver.start_time = maneuver.intersection_transit_left_turn_maneuver.start_time;

                new_maneuver.lane_following_maneuver.end_dist = maneuver.intersection_transit_left_turn_maneuver.end_dist;
                new_maneuver.lane_following_maneuver.end_speed = maneuver.intersection_transit_left_turn_maneuver.end_speed;
                new_maneuver.lane_following_maneuver.end_time = maneuver.intersection_transit_left_turn_maneuver.end_time;

                new_maneuver.lane_following_maneuver.lane_id = maneuver.intersection_transit_left_turn_maneuver.lane_id;

                new_maneuver_plan.push_back(new_maneuver);
            }

             /*Convert IT RIGHT TURN*/
            if (maneuver.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN)
            {
                new_maneuver.lane_following_maneuver.parameters = maneuver.intersection_transit_right_turn_maneuver.parameters;

                new_maneuver.lane_following_maneuver.start_dist = maneuver.intersection_transit_right_turn_maneuver.start_dist;
                new_maneuver.lane_following_maneuver.start_speed = maneuver.intersection_transit_right_turn_maneuver.start_speed;
                new_maneuver.lane_following_maneuver.start_time = maneuver.intersection_transit_right_turn_maneuver.start_time;

                new_maneuver.lane_following_maneuver.end_dist = maneuver.intersection_transit_right_turn_maneuver.end_dist;
                new_maneuver.lane_following_maneuver.end_speed = maneuver.intersection_transit_right_turn_maneuver.end_speed;
                new_maneuver.lane_following_maneuver.end_time = maneuver.intersection_transit_right_turn_maneuver.end_time;

                new_maneuver.lane_following_maneuver.lane_id = maneuver.intersection_transit_right_turn_maneuver.lane_id;

                new_maneuver_plan.push_back(new_maneuver);
            }
  
        }//end for-loop

        return new_maneuver_plan;

    }


    void IntersectionTransitManeuvering::set_trajectory_client(ros::ServiceClient& client)
    {
        trajectory_client_ = client;
    }

    bool IntersectionTransitManeuvering::validate_trajectory_plan(const cav_msgs::TrajectoryPlan& traj_plan)
    {
        if (traj_plan.trajectory_points.size()>= 2)
        {
            ROS_DEBUG_STREAM("Inlane Cruising Trajectory Time" << (double)traj_plan.trajectory_points[0].target_time.toSec());
            ROS_DEBUG_STREAM("Now:" << (double)ros::Time::now().toSec());
            if (traj_plan.trajectory_points[0].target_time + ros::Duration(5.0) > ros::Time::now())
            {
              return true;
            }
            else
            {
                ROS_DEBUG_STREAM("Old InlaneCruising Trajectory");
            }
        }
        else
        {
            ROS_DEBUG_STREAM("Invalid InlaneCruising Trajectory"); 
        }
        return false;
    }








}