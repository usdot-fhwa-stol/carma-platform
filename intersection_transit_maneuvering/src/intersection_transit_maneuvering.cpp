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
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <string>
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <trajectory_utils/trajectory_utils.hpp>
#include <trajectory_utils/conversions/conversions.hpp>
#include <sstream>
#include <intersection_transit_maneuvering/intersection_transit_maneuvering_node.hpp>


using oss = std::ostringstream;

namespace intersection_transit_maneuvering
{

namespace std_ph = std::placeholders;

/**
 * \brief Stream operators for carma_planning_msgs::msg::Maneuver and nested messages.
 *        NOTE: Does not print meta data
 */
std::ostream& operator<<(std::ostream& os, carma_planning_msgs::msg::ManeuverParameters m) {
    os << "maneuver_id: " << m.maneuver_id
        << " negotiation_type: " << unsigned(m.negotiation_type)
        << " planning_strategic_plugin: " << m.planning_strategic_plugin
        << " presence_vector: " << unsigned(m.presence_vector)
        << " planning_tactical_plugin: " << m.planning_tactical_plugin;
    return os;
}

std::ostream& operator<<(std::ostream& os, carma_planning_msgs::msg::IntersectionTransitStraightManeuver m) {
    os << "parameters: { " << m.parameters << " }"
        << " start_dist: " << m.start_dist
        << " end_dist: " << m.end_dist
        << " start_speed: " << m.start_speed
        << " end_speed: " << m.end_speed
        << " start_time: " << std::to_string(rclcpp::Time(m.start_time).seconds())
        << " end_time: " << std::to_string(rclcpp::Time(m.end_time).seconds())
        << " starting_lane_id: " << m.starting_lane_id
        << " ending_lane_id: " << m.ending_lane_id;
    return os;
}

std::ostream& operator<<(std::ostream& os, carma_planning_msgs::msg::IntersectionTransitLeftTurnManeuver m) {
    os << "parameters: { " << m.parameters << " }"
        << " start_dist: " << m.start_dist
        << " end_dist: " << m.end_dist
        << " start_speed: " << m.start_speed
        << " end_speed: " << m.end_speed
        << " start_time: " << std::to_string(rclcpp::Time(m.start_time).seconds())
        << " end_time: " << std::to_string(rclcpp::Time(m.end_time).seconds())
        << " starting_lane_id: " << m.starting_lane_id
        << " ending_lane_id: " << m.ending_lane_id;
    return os;
}

std::ostream& operator<<(std::ostream& os, carma_planning_msgs::msg::IntersectionTransitRightTurnManeuver m) {
    os << "parameters: { " << m.parameters << " }"
        << " start_dist: " << m.start_dist
        << " end_dist: " << m.end_dist
        << " start_speed: " << m.start_speed
        << " end_speed: " << m.end_speed
        << " start_time: " << std::to_string(rclcpp::Time(m.start_time).seconds())
        << " end_time: " << std::to_string(rclcpp::Time(m.end_time).seconds())
        << " starting_lane_id: " << m.starting_lane_id
        << " ending_lane_id: " << m.ending_lane_id;
    return os;
}

std::ostream& operator<<(std::ostream& os, carma_planning_msgs::msg::LaneFollowingManeuver m) {
    os << "parameters: { " << m.parameters << " }"
        << " start_dist: " << m.start_dist
        << " end_dist: " << m.end_dist
        << " start_speed: " << m.start_speed
        << " end_speed: " << m.end_speed
        << " start_time: " << std::to_string(rclcpp::Time(m.start_time).seconds())
        << " end_time: " << std::to_string(rclcpp::Time(m.end_time).seconds())
        << " lane_ids: [ ";

        for (const auto& i : m.lane_ids)
            os << i << " ";

        os << "]";
    
    return os;

}

std::ostream& operator<<(std::ostream& os, carma_planning_msgs::msg::Maneuver m) {

    os << "Maneuver Type: " << unsigned(m.type);
    switch(m.type) {
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
            os << m.intersection_transit_straight_maneuver;
            break;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
            os << m.intersection_transit_left_turn_maneuver;
            break;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
            os << m.intersection_transit_right_turn_maneuver;
            break;
        case carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING:
            os << m.lane_following_maneuver;
            break;
        default:
            os << " not yet supported for printing. ";
    }

    return os;
}

IntersectionTransitManeuveringNode::IntersectionTransitManeuveringNode(const rclcpp::NodeOptions &options)
:  carma_guidance_plugins::TacticalPlugin(options) {}

carma_ros2_utils::CallbackReturn IntersectionTransitManeuveringNode::on_configure_plugin()
{
 RCLCPP_INFO_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"), "IntersectionTransitManeuveringNode trying to configure");
                        
 object_ = std::make_shared<intersection_transit_maneuvering::Servicer>();

 auto trajectory_client  = create_client<carma_planning_msgs::srv::PlanTrajectory>("inlanecruising_plugin/plan_trajectory");

 object_->set_client(trajectory_client);

 //Return success if everthing initialized successfully
return CallbackReturn::SUCCESS;

}

  bool IntersectionTransitManeuveringNode::get_availability() {
    return true;
  }

  std::string IntersectionTransitManeuveringNode::get_version_id() {
    return "v4.0"; // Version ID matches the value set in this package's package.xml
  }   


    void  IntersectionTransitManeuveringNode::plan_trajectory_callback(
      std::shared_ptr<rmw_request_id_t>, 
      carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
      carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp) 
      {

        std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();  // Start timing the execution time for planning so it can be logged

        std::vector<carma_planning_msgs::msg::Maneuver> maneuver_plan;

        auto related_maneuvers = resp->related_maneuvers;

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"),"Starting planning for maneuver index: " << req->maneuver_index_to_plan);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"),"req->maneuver_plan.maneuvers.size(): " << req->maneuver_plan.maneuvers.size());

        for(size_t i = req->maneuver_index_to_plan; i < req->maneuver_plan.maneuvers.size(); i++)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"),"Looping");
            if(req->maneuver_plan.maneuvers[i].type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ||
            req->maneuver_plan.maneuvers[i].type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN || 
            req->maneuver_plan.maneuvers[i].type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN )
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"),"Found valid maneuver for planning at index: " << i);
                maneuver_plan.push_back(req->maneuver_plan.maneuvers[i]);
                related_maneuvers.push_back(i);
            }
            else
                {
                    break;
                }
        }

        converted_maneuvers_ = convert_maneuver_plan(maneuver_plan);
        auto new_req = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Request> ();

        for(auto i : converted_maneuvers_)
        {
            new_req->maneuver_plan.maneuvers.push_back(i);
        }

            new_req->header = req->header;
            new_req->vehicle_state = req->vehicle_state;
            new_req->initial_trajectory_plan = req->initial_trajectory_plan;
            object_->call(new_req,resp);

        if(!resp->trajectory_plan.trajectory_points.empty())//Since we're using an interface for this process, the call() functionality will come from somewhere else
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"),"Call Successful");
        } else {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"),"Failed to call service");
            return ;
        }


        resp->related_maneuvers = related_maneuvers; // Set the related maneuvers using the origional maneuver indexs not those sent to inlane-cruising
        for (auto maneuver : related_maneuvers) {
            resp->maneuver_status.push_back(carma_planning_msgs::srv::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);
        }

        std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now();  // Start timing the execution time for planning so it can be logged

        auto duration = end_time - start_time;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"),"ExecutionTime: " <<  std::chrono::duration<double>(duration).count());
        return ; 
    }
   
    std::vector<carma_planning_msgs::msg::Maneuver> IntersectionTransitManeuveringNode::convert_maneuver_plan(const std::vector<carma_planning_msgs::msg::Maneuver>& maneuvers)
    {
       if (maneuvers.size() == 0)
        {
            throw std::invalid_argument("No maneuvers to convert");
        }

        std::vector<carma_planning_msgs::msg::Maneuver> new_maneuver_plan;
        carma_planning_msgs::msg::Maneuver new_maneuver;
        new_maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING; //All of the converted maneuvers will be of type LANE_FOLLOWING
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"),"Input Maneuver Type = "<< static_cast<int>(maneuvers.front().type));
        for(const auto& maneuver : maneuvers)
        {
            //Throw exception if the manuever type does not match INTERSECTION_TRANSIT
            if ( maneuver.type != carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT &&
                maneuver.type != carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN &&
                maneuver.type != carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN)
                {
                    throw std::invalid_argument("Intersection transit maneuvering does not support this maneuver type");
                }

            //Convert IT Straight
            if (maneuver.type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT)
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"),"Converting INTERSECTION_TRANSIT_STRAIGHT");

                new_maneuver.lane_following_maneuver.parameters.maneuver_id = maneuver.intersection_transit_straight_maneuver.parameters.maneuver_id;
                new_maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = maneuver.intersection_transit_straight_maneuver.parameters.planning_strategic_plugin;
                new_maneuver.lane_following_maneuver.parameters.planning_tactical_plugin = maneuver.intersection_transit_straight_maneuver.parameters.planning_tactical_plugin;
                new_maneuver.lane_following_maneuver.parameters.negotiation_type = maneuver.intersection_transit_straight_maneuver.parameters.negotiation_type;
                new_maneuver.lane_following_maneuver.parameters.presence_vector = carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN;

                new_maneuver.lane_following_maneuver.start_dist = maneuver.intersection_transit_straight_maneuver.start_dist;
                new_maneuver.lane_following_maneuver.start_speed = maneuver.intersection_transit_straight_maneuver.start_speed;
                new_maneuver.lane_following_maneuver.start_time = maneuver.intersection_transit_straight_maneuver.start_time;

                new_maneuver.lane_following_maneuver.end_dist = maneuver.intersection_transit_straight_maneuver.end_dist;
                new_maneuver.lane_following_maneuver.end_speed = maneuver.intersection_transit_straight_maneuver.end_speed;
                new_maneuver.lane_following_maneuver.end_time = maneuver.intersection_transit_straight_maneuver.end_time;


                new_maneuver.lane_following_maneuver.lane_ids = { maneuver.intersection_transit_straight_maneuver.starting_lane_id };

                new_maneuver_plan.push_back(new_maneuver);
            }

             //Convert IT LEFT TURN
            if (maneuver.type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN)
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"),"Converting INTERSECTION_TRANSIT_LEFT_TURN");

                new_maneuver.lane_following_maneuver.parameters.maneuver_id = maneuver.intersection_transit_left_turn_maneuver.parameters.maneuver_id;
                new_maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = maneuver.intersection_transit_left_turn_maneuver.parameters.planning_strategic_plugin;
                new_maneuver.lane_following_maneuver.parameters.planning_tactical_plugin = maneuver.intersection_transit_left_turn_maneuver.parameters.planning_tactical_plugin;
                new_maneuver.lane_following_maneuver.parameters.negotiation_type = maneuver.intersection_transit_left_turn_maneuver.parameters.negotiation_type;
                new_maneuver.lane_following_maneuver.parameters.presence_vector = carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN;

                new_maneuver.lane_following_maneuver.start_dist = maneuver.intersection_transit_left_turn_maneuver.start_dist;
                new_maneuver.lane_following_maneuver.start_speed = maneuver.intersection_transit_left_turn_maneuver.start_speed;
                new_maneuver.lane_following_maneuver.start_time = maneuver.intersection_transit_left_turn_maneuver.start_time;

                new_maneuver.lane_following_maneuver.end_dist = maneuver.intersection_transit_left_turn_maneuver.end_dist;
                new_maneuver.lane_following_maneuver.end_speed = maneuver.intersection_transit_left_turn_maneuver.end_speed;
                new_maneuver.lane_following_maneuver.end_time = maneuver.intersection_transit_left_turn_maneuver.end_time;

                new_maneuver.lane_following_maneuver.lane_ids = { maneuver.intersection_transit_left_turn_maneuver.starting_lane_id };

                new_maneuver_plan.push_back(new_maneuver);
            }

             //Convert IT RIGHT TURN
            if (maneuver.type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN)
            {

                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"),"Converting INTERSECTION_TRANSIT_RIGHT_TURN");

                new_maneuver.lane_following_maneuver.parameters.maneuver_id = maneuver.intersection_transit_right_turn_maneuver.parameters.maneuver_id;
                new_maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = maneuver.intersection_transit_right_turn_maneuver.parameters.planning_strategic_plugin;
                new_maneuver.lane_following_maneuver.parameters.planning_tactical_plugin = maneuver.intersection_transit_right_turn_maneuver.parameters.planning_tactical_plugin;
                new_maneuver.lane_following_maneuver.parameters.negotiation_type = maneuver.intersection_transit_right_turn_maneuver.parameters.negotiation_type;
                new_maneuver.lane_following_maneuver.parameters.presence_vector = carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN;

                new_maneuver.lane_following_maneuver.start_dist = maneuver.intersection_transit_right_turn_maneuver.start_dist;
                new_maneuver.lane_following_maneuver.start_speed = maneuver.intersection_transit_right_turn_maneuver.start_speed;
                new_maneuver.lane_following_maneuver.start_time = maneuver.intersection_transit_right_turn_maneuver.start_time;

                new_maneuver.lane_following_maneuver.end_dist = maneuver.intersection_transit_right_turn_maneuver.end_dist;
                new_maneuver.lane_following_maneuver.end_speed = maneuver.intersection_transit_right_turn_maneuver.end_speed;
                new_maneuver.lane_following_maneuver.end_time = maneuver.intersection_transit_right_turn_maneuver.end_time;

                new_maneuver.lane_following_maneuver.lane_ids = { maneuver.intersection_transit_right_turn_maneuver.starting_lane_id };

                new_maneuver_plan.push_back(new_maneuver);
            }

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"),"Original Maneuver : " << maneuver << std::endl <<   "Converted Maneuver: " << new_maneuver);
  
        }//end for-loop 

        return new_maneuver_plan; 

    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(intersection_transit_maneuvering::IntersectionTransitManeuveringNode)