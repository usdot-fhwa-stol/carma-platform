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
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <sstream>
#include <carma_utils/containers/containers.h>
#include <intersection_transit_maneuvering.h>


using oss = std::ostringstream;

namespace intersection_transit_maneuvering
{
    IntersectionTransitManeuvering::IntersectionTransitManeuvering(carma_wm::WorldModelConstPtr wm, PublishPluginDiscoveryCB plugin_discovery_publisher,
                                                                     std::shared_ptr<CallInterface> obj)
    {        
        plugin_discovery_msg_.name = "IntersectionTransitManeuvering";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = true;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";

        object_ = obj;
    }

    bool IntersectionTransitManeuvering::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
    {
        ros::WallTime start_time = ros::WallTime::now();  // Start timeing the execution time for planning so it can be logged

        std::vector<cav_msgs::Maneuver> maneuver_plan;
        for(size_t i = req.maneuver_index_to_plan; i < req.maneuver_plan.maneuvers.size(); i++)
        {
            if(req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ||
            req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN || 
            req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN )
            {
                maneuver_plan.push_back(req.maneuver_plan.maneuvers[i]);
                resp.related_maneuvers.push_back(i);
            }
            else
                {
                    break;
                }
        }

        converted_maneuvers_ = convert_maneuver_plan(maneuver_plan);
        cav_srvs::PlanTrajectoryRequest new_req;

        for(auto i : converted_maneuvers_)
        {
            new_req.maneuver_plan.maneuvers.push_back(i);
        }

            new_req.vehicle_state = req.vehicle_state;
            new_req.initial_trajectory_plan = req.initial_trajectory_plan;

        if(object_->call(new_req,resp))//Since we're using an interface for this process, the call() functionality will come from somewhere else
        {
            ROS_DEBUG_STREAM("Call Successful");
        }

        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

        ros::WallTime end_time = ros::WallTime::now();

        ros::WallDuration duration = end_time - start_time;
        ROS_DEBUG_STREAM("ExecutionTime: " << duration.toSec());
        return true;
    }
   
    std::vector<cav_msgs::Maneuver> IntersectionTransitManeuvering::convert_maneuver_plan(const std::vector<cav_msgs::Maneuver>& maneuvers)
    {
        if (maneuvers.size() == 0)
        {
            throw std::invalid_argument("No maneuvers to convert");
        }

        std::vector<cav_msgs::Maneuver> new_maneuver_plan;
        cav_msgs::Maneuver new_maneuver;
        new_maneuver.type = cav_msgs::Maneuver::LANE_FOLLOWING; //All of the converted maneuvers will be of type LANE_FOLLOWING
        ROS_DEBUG_STREAM("Maneuver Type = "<< static_cast<int>(maneuvers.front().type));
        for(const auto& maneuver : maneuvers)
        {
            /*Throw exception if the manuever type does not match INTERSECTION_TRANSIT*/
            if ( maneuver.type != cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT &&
                maneuver.type != cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN &&
                maneuver.type != cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN)
                {
                    throw std::invalid_argument("Intersection transit maneuvering does not support this maneuver type");
                }

            /*Convert IT Straight*/
            if (maneuver.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT)
            {
                new_maneuver.lane_following_maneuver.parameters.maneuver_id = maneuver.intersection_transit_straight_maneuver.parameters.maneuver_id;
                new_maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = maneuver.intersection_transit_straight_maneuver.parameters.planning_strategic_plugin;
                new_maneuver.lane_following_maneuver.parameters.planning_tactical_plugin = maneuver.intersection_transit_straight_maneuver.parameters.planning_tactical_plugin;
                new_maneuver.lane_following_maneuver.parameters.neogition_type = maneuver.intersection_transit_straight_maneuver.parameters.neogition_type;

                new_maneuver.lane_following_maneuver.start_dist = maneuver.intersection_transit_straight_maneuver.start_dist;
                new_maneuver.lane_following_maneuver.start_speed = maneuver.intersection_transit_straight_maneuver.start_speed;
                new_maneuver.lane_following_maneuver.start_time = maneuver.intersection_transit_straight_maneuver.start_time;

                new_maneuver.lane_following_maneuver.end_dist = maneuver.intersection_transit_straight_maneuver.end_dist;
                new_maneuver.lane_following_maneuver.end_speed = maneuver.intersection_transit_straight_maneuver.end_speed;
                new_maneuver.lane_following_maneuver.end_time = maneuver.intersection_transit_straight_maneuver.end_time;

                new_maneuver.lane_following_maneuver.parameters.bool_valued_meta_data.clear();
                new_maneuver.lane_following_maneuver.parameters.float_valued_meta_data.clear();

                new_maneuver.lane_following_maneuver.lane_id = maneuver.intersection_transit_straight_maneuver.starting_lane_id;

                new_maneuver_plan.push_back(new_maneuver);
            }

             /*Convert IT LEFT TURN*/
            if (maneuver.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN)
            {
                new_maneuver.lane_following_maneuver.parameters.maneuver_id = maneuver.intersection_transit_left_turn_maneuver.parameters.maneuver_id;
                new_maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = maneuver.intersection_transit_left_turn_maneuver.parameters.planning_strategic_plugin;
                new_maneuver.lane_following_maneuver.parameters.planning_tactical_plugin = maneuver.intersection_transit_left_turn_maneuver.parameters.planning_tactical_plugin;
                new_maneuver.lane_following_maneuver.parameters.neogition_type = maneuver.intersection_transit_left_turn_maneuver.parameters.neogition_type;

                new_maneuver.lane_following_maneuver.start_dist = maneuver.intersection_transit_left_turn_maneuver.start_dist;
                new_maneuver.lane_following_maneuver.start_speed = maneuver.intersection_transit_left_turn_maneuver.start_speed;
                new_maneuver.lane_following_maneuver.start_time = maneuver.intersection_transit_left_turn_maneuver.start_time;

                new_maneuver.lane_following_maneuver.end_dist = maneuver.intersection_transit_left_turn_maneuver.end_dist;
                new_maneuver.lane_following_maneuver.end_speed = maneuver.intersection_transit_left_turn_maneuver.end_speed;
                new_maneuver.lane_following_maneuver.end_time = maneuver.intersection_transit_left_turn_maneuver.end_time;

                new_maneuver.lane_following_maneuver.lane_id = maneuver.intersection_transit_left_turn_maneuver.starting_lane_id;

                new_maneuver_plan.push_back(new_maneuver);
            }

             /*Convert IT RIGHT TURN*/
            if (maneuver.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN)
            {
                new_maneuver.lane_following_maneuver.parameters.maneuver_id = maneuver.intersection_transit_right_turn_maneuver.parameters.maneuver_id;
                new_maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = maneuver.intersection_transit_right_turn_maneuver.parameters.planning_strategic_plugin;
                new_maneuver.lane_following_maneuver.parameters.planning_tactical_plugin = maneuver.intersection_transit_right_turn_maneuver.parameters.planning_tactical_plugin;
                new_maneuver.lane_following_maneuver.parameters.neogition_type = maneuver.intersection_transit_right_turn_maneuver.parameters.neogition_type;

                new_maneuver.lane_following_maneuver.start_dist = maneuver.intersection_transit_right_turn_maneuver.start_dist;
                new_maneuver.lane_following_maneuver.start_speed = maneuver.intersection_transit_right_turn_maneuver.start_speed;
                new_maneuver.lane_following_maneuver.start_time = maneuver.intersection_transit_right_turn_maneuver.start_time;

                new_maneuver.lane_following_maneuver.end_dist = maneuver.intersection_transit_right_turn_maneuver.end_dist;
                new_maneuver.lane_following_maneuver.end_speed = maneuver.intersection_transit_right_turn_maneuver.end_speed;
                new_maneuver.lane_following_maneuver.end_time = maneuver.intersection_transit_right_turn_maneuver.end_time;

                new_maneuver.lane_following_maneuver.lane_id = maneuver.intersection_transit_right_turn_maneuver.starting_lane_id;

                new_maneuver_plan.push_back(new_maneuver);
            }
  
        }//end for-loop

        return new_maneuver_plan;

    }

    bool IntersectionTransitManeuvering::onSpin()
    {
        plugin_discovery_publisher_(plugin_discovery_msg_);
        return true;
    }

    







}