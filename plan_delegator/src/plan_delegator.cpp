/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include <stdexcept>
#include <carma_wm/Geometry.h>
#include "plan_delegator.hpp"

namespace plan_delegator
{

    namespace 
    {
        /**
         * \brief Anonymous function to set the starting_lane_id for all maneuver types except lane following. This
         * maneuver parameter cannot be set with SET_MANEUVER_PROPERTY calls since it is not included in
         * LANE_FOLLOW maneuvers.
         */ 
        void setManeuverStartingLaneletId(cav_msgs::Maneuver& mvr, lanelet::Id start_id) {
            ROS_DEBUG_STREAM("Updating maneuver starting_lane_id to " << start_id);

            switch(mvr.type) {
                case cav_msgs::Maneuver::LANE_CHANGE:
                    mvr.lane_change_maneuver.starting_lane_id = std::to_string(start_id);
                    break;
                case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                    mvr.intersection_transit_straight_maneuver.starting_lane_id = std::to_string(start_id);
                    break;
                case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                    mvr.intersection_transit_left_turn_maneuver.starting_lane_id = std::to_string(start_id);
                    break;
                case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                    mvr.intersection_transit_right_turn_maneuver.starting_lane_id = std::to_string(start_id);
                    break;
                case cav_msgs::Maneuver::STOP_AND_WAIT:
                    mvr.stop_and_wait_maneuver.starting_lane_id = std::to_string(start_id);
                    break;
                default:
                    throw std::invalid_argument("Maneuver type does not have starting and ending lane ids");
            }
        }

        /**
         * \brief Anonymous function to set the ending_lane_id for all maneuver types except lane following. This
         * maneuver parameter cannot be set with SET_MANEUVER_PROPERTY calls since it is not included in
         * LANE_FOLLOW maneuvers.
         */ 
        void setManeuverEndingLaneletId(cav_msgs::Maneuver& mvr, lanelet::Id end_id) {
            ROS_DEBUG_STREAM("Updating maneuver ending_lane_id to " << end_id);

            switch(mvr.type) {
                case cav_msgs::Maneuver::LANE_CHANGE:
                    mvr.lane_change_maneuver.ending_lane_id = std::to_string(end_id);
                    break;
                case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                    mvr.intersection_transit_straight_maneuver.ending_lane_id = std::to_string(end_id);
                    break;
                case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                    mvr.intersection_transit_left_turn_maneuver.ending_lane_id = std::to_string(end_id);
                    break;
                case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                    mvr.intersection_transit_right_turn_maneuver.ending_lane_id = std::to_string(end_id);
                    break;
                case cav_msgs::Maneuver::STOP_AND_WAIT:
                    mvr.stop_and_wait_maneuver.ending_lane_id = std::to_string(end_id);
                    break;
                default:
                    throw std::invalid_argument("Maneuver type does not have starting and ending lane ids");
            }
        }
    
        /**
         * \brief Anonymous function to get the starting lanelet id for all maneuver types except lane following. This
         * maneuver parameters cannot be obtained with GET_MANEUVER_PROPERTY calls since they are not included in
         * LANE_FOLLOW maneuvers.
         */ 
        std::string getManeuverStartingLaneletId(cav_msgs::Maneuver mvr) {
            switch(mvr.type) {
                case cav_msgs::Maneuver::LANE_CHANGE:
                    return mvr.lane_change_maneuver.starting_lane_id;
                case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                    return mvr.intersection_transit_straight_maneuver.starting_lane_id;
                case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                    return mvr.intersection_transit_left_turn_maneuver.starting_lane_id;
                case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                    return mvr.intersection_transit_right_turn_maneuver.starting_lane_id;
                case cav_msgs::Maneuver::STOP_AND_WAIT:
                    return mvr.stop_and_wait_maneuver.starting_lane_id;
                default:
                    throw std::invalid_argument("Maneuver type does not have starting and ending lane ids");
            }
        }

        /**
         * \brief Anonymous function to get the ending lanelet id for all maneuver types except lane following. This
         * maneuver parameters cannot be obtained with GET_MANEUVER_PROPERTY calls since they are not included in
         * LANE_FOLLOW maneuvers.
         */ 
        std::string getManeuverEndingLaneletId(cav_msgs::Maneuver mvr) {
            switch(mvr.type) {
                case cav_msgs::Maneuver::LANE_CHANGE:
                    return mvr.lane_change_maneuver.ending_lane_id;
                case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                    return mvr.intersection_transit_straight_maneuver.ending_lane_id;
                case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                    return mvr.intersection_transit_left_turn_maneuver.ending_lane_id;
                case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                    return mvr.intersection_transit_right_turn_maneuver.ending_lane_id;
                case cav_msgs::Maneuver::STOP_AND_WAIT:
                    return mvr.stop_and_wait_maneuver.ending_lane_id;
                default:
                    throw std::invalid_argument("Maneuver type does not have starting and ending lane ids");
            }
        }
    }
    
    void PlanDelegator::init()
    {
        nh_ = ros::CARMANodeHandle();
        pnh_ = ros::CARMANodeHandle("~");

        // Initialize world model
        wml_.reset(new carma_wm::WMListener());

        

        pnh_.param<std::string>("planning_topic_prefix", planning_topic_prefix_, "/plugins/");        
        pnh_.param<std::string>("planning_topic_suffix", planning_topic_suffix_, "/plan_trajectory");
        pnh_.param<double>("trajectory_planning_rate", trajectory_planning_rate_, 10.0);
        pnh_.param<double>("trajectory_duration_threshold", max_trajectory_duration_, 6.0);
        pnh_.param<double>("min_speed", min_crawl_speed_, min_crawl_speed_);

        traj_pub_ = nh_.advertise<cav_msgs::TrajectoryPlan>("plan_trajectory", 5);
        plan_sub_ = nh_.subscribe("final_maneuver_plan", 5, &PlanDelegator::maneuverPlanCallback, this);
        twist_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("current_velocity", 5,
            [this](const geometry_msgs::TwistStampedConstPtr& twist) {this->latest_twist_ = *twist;});
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("current_pose", 5,
            [this](const geometry_msgs::PoseStampedConstPtr& pose) {this->latest_pose_ = *pose;});
        guidance_state_sub_ = nh_.subscribe<cav_msgs::GuidanceState>("guidance_state", 5, &PlanDelegator::guidanceStateCallback, this);

        lookupFrontBumperTransform();
        wm_ = wml_->getWorldModel();
        
        traj_timer_ = pnh_.createTimer(
            ros::Duration(ros::Rate(trajectory_planning_rate_)),
            &PlanDelegator::onTrajPlanTick, 
            this);
    }
    
    void PlanDelegator::run() 
    {
        ros::CARMANodeHandle::spin();
    }

    void PlanDelegator::guidanceStateCallback(const cav_msgs::GuidanceStateConstPtr& msg)
    {
        guidance_engaged = (msg->state == cav_msgs::GuidanceState::ENGAGED);
    }

    void PlanDelegator::maneuverPlanCallback(const cav_msgs::ManeuverPlanConstPtr& plan)
    {
        ROS_INFO_STREAM("Received request to delegate plan ID " << plan->maneuver_plan_id);
        // do basic check to see if the input is valid
        if (isManeuverPlanValid(plan))
        {
            latest_maneuver_plan_ = *plan;

            ROS_DEBUG_STREAM("Received plan with " << latest_maneuver_plan_.maneuvers.size() << " maneuvers");
            
            // Update the parameters associated with each maneuver
            for (auto& maneuver : latest_maneuver_plan_.maneuvers) {
                updateManeuverParameters(maneuver);
            }
        }
        else {
            ROS_WARN_STREAM("Received empty plan, no maneuvers found in plan ID " << plan->maneuver_plan_id);
        }
    }

    ros::ServiceClient& PlanDelegator::getPlannerClientByName(const std::string& planner_name)
    {
        if(planner_name.size() == 0)
        {
            throw std::invalid_argument("Invalid trajectory planner name because it has zero length!");
        }
        if(trajectory_planners_.find(planner_name) == trajectory_planners_.end())
        {
            ROS_INFO_STREAM("Discovered new trajectory planner: " << planner_name);
            trajectory_planners_.emplace(
                planner_name, nh_.serviceClient<cav_srvs::PlanTrajectory>(planning_topic_prefix_ + planner_name + planning_topic_suffix_));
        }
        return trajectory_planners_[planner_name];
    }

    bool PlanDelegator::isManeuverPlanValid(const cav_msgs::ManeuverPlanConstPtr& maneuver_plan) const noexcept
    {
        // currently it only checks if maneuver list is empty
        return !maneuver_plan->maneuvers.empty();
    }

    bool PlanDelegator::isTrajectoryValid(const cav_msgs::TrajectoryPlan& trajectory_plan) const noexcept
    {
        // currently it only checks if trajectory contains less than 2 points
        return !(trajectory_plan.trajectory_points.size() < 2);
    }

    bool PlanDelegator::isManeuverExpired(const cav_msgs::Maneuver& maneuver, ros::Time current_time) const
    {
        ROS_DEBUG_STREAM("maneuver start time:" << std::to_string(GET_MANEUVER_PROPERTY(maneuver, start_time).toSec()));
        ROS_DEBUG_STREAM("maneuver end time:" << std::to_string(GET_MANEUVER_PROPERTY(maneuver, end_time).toSec()));
        ROS_DEBUG_STREAM("current time:" << std::to_string(ros::Time::now().toSec()));
        bool isexpired = GET_MANEUVER_PROPERTY(maneuver, end_time) <= current_time;
        ROS_DEBUG_STREAM("isexpired:" << isexpired);
        // TODO: temporary disabling expiration check
        return  false;// TODO maneuver expiration should maybe be based off of distance not time? https://github.com/usdot-fhwa-stol/carma-platform/issues/1107
    }

    cav_srvs::PlanTrajectory PlanDelegator::composePlanTrajectoryRequest(const cav_msgs::TrajectoryPlan& latest_trajectory_plan, const uint16_t& current_maneuver_index) const
    {
        auto plan_req = cav_srvs::PlanTrajectory{};
        plan_req.request.maneuver_plan = latest_maneuver_plan_;
        // set current vehicle state if we have NOT planned any previous trajectories
        if(latest_trajectory_plan.trajectory_points.empty())
        {
            plan_req.request.header.stamp = latest_pose_.header.stamp;
            plan_req.request.vehicle_state.longitudinal_vel = latest_twist_.twist.linear.x;
            plan_req.request.vehicle_state.x_pos_global = latest_pose_.pose.position.x;
            plan_req.request.vehicle_state.y_pos_global = latest_pose_.pose.position.y;
            double roll, pitch, yaw;
            carma_wm::geometry::rpyFromQuaternion(latest_pose_.pose.orientation, roll, pitch, yaw);
            plan_req.request.vehicle_state.orientation = yaw;
            plan_req.request.maneuver_index_to_plan = current_maneuver_index;
        }
        // set vehicle state based on last two planned trajectory points
        else
        {
            cav_msgs::TrajectoryPlanPoint last_point = latest_trajectory_plan.trajectory_points.back();
            cav_msgs::TrajectoryPlanPoint second_last_point = *(latest_trajectory_plan.trajectory_points.rbegin() + 1);
            plan_req.request.vehicle_state.x_pos_global = last_point.x;
            plan_req.request.vehicle_state.y_pos_global = last_point.y;
            auto distance_diff = std::sqrt(std::pow(last_point.x - second_last_point.x, 2) + std::pow(last_point.y - second_last_point.y, 2));
            ros::Duration time_diff = last_point.target_time - second_last_point.target_time;
            auto time_diff_sec = time_diff.toSec();
            plan_req.request.maneuver_index_to_plan = current_maneuver_index;
            // this assumes the vehicle does not have significant lateral velocity
            plan_req.request.header.stamp = latest_trajectory_plan.trajectory_points.back().target_time;
            plan_req.request.vehicle_state.longitudinal_vel = distance_diff / time_diff_sec;
            // TODO develop way to set yaw value for future points
        }
        return plan_req;
    }

    bool PlanDelegator::isTrajectoryLongEnough(const cav_msgs::TrajectoryPlan& plan) const noexcept
    {
        ros::Duration time_diff = plan.trajectory_points.back().target_time - plan.trajectory_points.front().target_time;
        return time_diff.toSec() >= max_trajectory_duration_;
    }

    void PlanDelegator::updateManeuverParameters(cav_msgs::Maneuver& maneuver)
    {
        // Update maneuver starting and ending downtrack distances
        double original_start_dist = GET_MANEUVER_PROPERTY(maneuver, start_dist);
        double original_end_dist = GET_MANEUVER_PROPERTY(maneuver, end_dist);
        ROS_DEBUG_STREAM("Changing maneuver distances for planner: " << GET_MANEUVER_PROPERTY(maneuver, parameters.planning_tactical_plugin));
        double adjusted_start_dist = original_start_dist - length_to_front_bumper_;
        ROS_DEBUG_STREAM("original_start_dist:" << original_start_dist);
        ROS_DEBUG_STREAM("adjusted_start_dist:" << adjusted_start_dist);
        double adjusted_end_dist = original_end_dist - length_to_front_bumper_;
        ROS_DEBUG_STREAM("original_end_dist:" << original_end_dist);
        ROS_DEBUG_STREAM("adjusted_end_dist:" << adjusted_end_dist);
        SET_MANEUVER_PROPERTY(maneuver, start_dist, adjusted_start_dist);
        SET_MANEUVER_PROPERTY(maneuver, end_dist, adjusted_end_dist);

        // Get the lanelets crossed by the updated maneuver (considers full route; not just shortest path)
        std::vector<lanelet::ConstLanelet> adjusted_crossed_lanelets = wm_->getLaneletsBetween(adjusted_start_dist, adjusted_end_dist, false, false);
        
        if (adjusted_crossed_lanelets.size() == 0) {
            throw std::invalid_argument("The adjusted maneuver does not cross any lanelets going from: " + std::to_string(adjusted_start_dist) + " to " + std::to_string(adjusted_end_dist));
        }

        // Update maneuver-specific lanelet ID parameters
        // Note: Assumes that the maneuver start and end distances are adjusted by a distance less than the length of a lanelet. 
        if(maneuver.type == cav_msgs::Maneuver::LANE_FOLLOWING && !maneuver.lane_following_maneuver.lane_ids.empty()) 
        {
            // Obtain the original starting lanelet from the maneuver's lane_ids
            lanelet::Id original_starting_lanelet_id = std::stoi(maneuver.lane_following_maneuver.lane_ids.front());
            lanelet::ConstLanelet original_starting_lanelet = wm_->getMap()->laneletLayer.get(original_starting_lanelet_id);

            // Obtain the original ending lanelet from the maneuver's lane_ids
            lanelet::Id original_ending_lanelet_id = std::stoi(maneuver.lane_following_maneuver.lane_ids.back());
            lanelet::ConstLanelet original_ending_lanelet = wm_->getMap()->laneletLayer.get(original_ending_lanelet_id);

            // Check whether the updated maneuver crosses a new starting lanelet and whether it still crosses the original ending lanelet
            bool found_lanelet_before_starting_lanelet = false;
            bool crosses_original_ending_lanelet = false;
            for(auto lanelet : adjusted_crossed_lanelets) {
                auto starting_relation = wm_->getMapRoutingGraph()->routingRelation(lanelet, original_starting_lanelet);

                // Lanelet preceeding the original starting lanelet is crossed by the updated maneuver, so it is added to the beginning of lane_ids
                if (starting_relation == lanelet::routing::RelationType::Successor && !found_lanelet_before_starting_lanelet) {
                    ROS_DEBUG_STREAM("Lanelet " << lanelet.id() << " inserted at the front of maneuver's lane_ids");

                    // lane_ids array is ordered by increasing downtrack, so this new starting lanelet is inserted at the front
                    maneuver.lane_following_maneuver.lane_ids.insert(maneuver.lane_following_maneuver.lane_ids.begin(), std::to_string(lanelet.id()));

                    found_lanelet_before_starting_lanelet = true;
                }
                else if (lanelet == original_ending_lanelet) {
                    crosses_original_ending_lanelet = true;
                }
            }

            // If the updated maneuver does not cross the original ending lanelet, remove that lanelet from the end of the maneuver's lane_ids
            if (!crosses_original_ending_lanelet) {
                ROS_DEBUG_STREAM("Original ending lanelet " << original_ending_lanelet.id() << " removed from lane_ids since the updated maneuver no longer crosses it");
                
                // lane_ids array is ordered by increasing downtrack, so the last element in the array corresponds to the original ending lanelet
                maneuver.lane_following_maneuver.lane_ids.pop_back();
            }
        } 
        else if (maneuver.type != cav_msgs::Maneuver::LANE_FOLLOWING)
        {
            // Obtain the original starting lanelet from the maneuver
            lanelet::Id original_starting_lanelet_id = std::stoi(getManeuverStartingLaneletId(maneuver));
            lanelet::ConstLanelet original_starting_lanelet = wm_->getMap()->laneletLayer.get(original_starting_lanelet_id);

            // Obtain the original ending lanelet from the maneuver
            lanelet::Id original_ending_lanelet_id = std::stoi(getManeuverEndingLaneletId(maneuver));
            lanelet::ConstLanelet original_ending_lanelet = wm_->getMap()->laneletLayer.get(original_ending_lanelet_id);

            // Check whether the updated maneuver crosses a new starting lanelet and whether it still crosses the original ending lanelet
            bool found_lanelet_before_starting_lanelet = false;
            bool found_lanelet_before_ending_lanelet = false;
            bool crosses_original_ending_lanelet = false;
            lanelet::ConstLanelet lanelet_before_original_ending_lanelet;
            for(auto lanelet : adjusted_crossed_lanelets) {
                auto starting_relation = wm_->getMapRoutingGraph()->routingRelation(lanelet, original_starting_lanelet);
                auto ending_relation = wm_->getMapRoutingGraph()->routingRelation(lanelet, original_ending_lanelet);

                // Lanelet preceeding the original starting lanelet is crossed by the updated maneuver, so maneuver's starting_lanelet_id must be updated
                if (starting_relation == lanelet::routing::RelationType::Successor && !found_lanelet_before_starting_lanelet) {
                    setManeuverStartingLaneletId(maneuver, lanelet.id());
                    found_lanelet_before_starting_lanelet = true;
                }
                // Lanelet preceeding the original ending lanelet is found
                else if (ending_relation == lanelet::routing::RelationType::Successor && !found_lanelet_before_ending_lanelet) {
                    lanelet_before_original_ending_lanelet = lanelet;
                    found_lanelet_before_ending_lanelet = true;
                }

                if (lanelet == original_ending_lanelet) {
                    crosses_original_ending_lanelet = true;
                }
            }

            // If the updated maneuver does not cross the original ending lanelet, update the ending lanelet to its preceeding lanelet
            if (!crosses_original_ending_lanelet) {
                if (found_lanelet_before_ending_lanelet){
                    setManeuverEndingLaneletId(maneuver, lanelet_before_original_ending_lanelet.id());
                }
                else {
                    throw std::invalid_argument("Updated maneuver has unknown ending lanelet.");
                }
            }

        }

    }

    cav_msgs::TrajectoryPlan PlanDelegator::planTrajectory()
    {
        cav_msgs::TrajectoryPlan latest_trajectory_plan;
        if(!guidance_engaged)
        {
            ROS_INFO_STREAM("Guidance is not engaged. Plan delegator will not plan trajectory.");
            return latest_trajectory_plan;
        }

        // Flag for the first received trajectory plan service response
        bool first_trajectory_plan = true;
        
        // Track the index of the starting maneuver in the maneuver plan that this trajectory plan service request is for
        uint16_t current_maneuver_index = 0;
        
        // Loop through maneuver list to make service call to applicable Tactical Plugin
        while(current_maneuver_index < latest_maneuver_plan_.maneuvers.size())
        {
            // const auto& maneuver = latest_maneuver_plan_.maneuvers[current_maneuver_index];
            auto& maneuver = latest_maneuver_plan_.maneuvers[current_maneuver_index];

            // // ignore expired maneuvers
            // if(isManeuverExpired(maneuver))
            // {
            //     ROS_INFO_STREAM("Dropping expired maneuver: " << GET_MANEUVER_PROPERTY(maneuver, parameters.maneuver_id));
            //     // Update the maneuver plan index for the next loop
            //     ++current_maneuver_index;
            //     continue;
            // }
            
            lanelet::BasicPoint2d current_loc(latest_pose_.pose.position.x, latest_pose_.pose.position.y);
            double current_downtrack = wm_->routeTrackPos(current_loc).downtrack;
            ROS_DEBUG_STREAM("current_downtrack" << current_downtrack);
            double maneuver_end_dist = GET_MANEUVER_PROPERTY(maneuver, end_dist);
            ROS_DEBUG_STREAM("maneuver_end_dist" << maneuver_end_dist);

            // // ignore maneuver that is passed.
            // if (current_downtrack > maneuver_end_dist)
            // {
            //     ROS_INFO_STREAM("Dropping passed maneuver: " << GET_MANEUVER_PROPERTY(maneuver, parameters.maneuver_id));
            //     // Update the maneuver plan index for the next loop
            //     ++current_maneuver_index;
            //     continue;
            // }
            

            // get corresponding ros service client for plan trajectory
            auto maneuver_planner = GET_MANEUVER_PROPERTY(maneuver, parameters.planning_tactical_plugin);
            auto client = getPlannerClientByName(maneuver_planner);
            
            ROS_DEBUG_STREAM("Current planner: " << maneuver_planner);

            // compose service request
            auto plan_req = composePlanTrajectoryRequest(latest_trajectory_plan, current_maneuver_index);

            if(client.call(plan_req))
            {
                // validate trajectory before add to the plan
                if(!isTrajectoryValid(plan_req.response.trajectory_plan))
                {
                    ROS_WARN_STREAM("Found invalid trajectory with less than 2 trajectory points for " << latest_maneuver_plan_.maneuver_plan_id);
                    break;
                }
                //Remove duplicate point from start of trajectory
                if(latest_trajectory_plan.trajectory_points.size() !=0){
                    
                    if(latest_trajectory_plan.trajectory_points.back().target_time == plan_req.response.trajectory_plan.trajectory_points.front().target_time){
                        ROS_DEBUG_STREAM("Removing duplicate point for planner: " << maneuver_planner);
                        plan_req.response.trajectory_plan.trajectory_points.erase(plan_req.response.trajectory_plan.trajectory_points.begin());
                        ROS_DEBUG_STREAM("plan_req.response.trajectory_plan size: " << plan_req.response.trajectory_plan.trajectory_points.size());

                    }
                }
                latest_trajectory_plan.trajectory_points.insert(latest_trajectory_plan.trajectory_points.end(),
                                                                plan_req.response.trajectory_plan.trajectory_points.begin(),
                                                                plan_req.response.trajectory_plan.trajectory_points.end());
                ROS_DEBUG_STREAM("new latest_trajectory_plan size: " << latest_trajectory_plan.trajectory_points.size());
                
                // Assign the trajectory plan's initial longitudinal velocity based on the first tactical plugin's response
                if(first_trajectory_plan == true)
                {
                    latest_trajectory_plan.initial_longitudinal_velocity = plan_req.response.trajectory_plan.initial_longitudinal_velocity;
                    first_trajectory_plan = false;
                }

                if(isTrajectoryLongEnough(latest_trajectory_plan))
                {
                    ROS_INFO_STREAM("Plan Trajectory completed for " << latest_maneuver_plan_.maneuver_plan_id);
                    break;
                }

                // Update the maneuver plan index based on the last maneuver index converted to a trajectory
                // This is required since inlanecruising_plugin can plan a trajectory over contiguous LANE_FOLLOWING maneuvers
                if(plan_req.response.related_maneuvers.size() > 0)
                {
                    current_maneuver_index = plan_req.response.related_maneuvers.back() + 1;
                } 
            }
            else
            {
                ROS_WARN_STREAM("Unsuccessful service call to trajectory planner:" << maneuver_planner << " for plan ID " << latest_maneuver_plan_.maneuver_plan_id);
                // if one service call fails, it should end plan immediately because it is there is no point to generate plan with empty space
                break;
            }
        }

        return latest_trajectory_plan;
    }

    void PlanDelegator::onTrajPlanTick(const ros::TimerEvent& te)
    {
        cav_msgs::TrajectoryPlan trajectory_plan = planTrajectory();
        
        // Check if planned trajectory is valid before send out
        if(isTrajectoryValid(trajectory_plan))
        {
            trajectory_plan.header.stamp = ros::Time::now();
            traj_pub_.publish(trajectory_plan);
        }
        else
        {
            // ROS_WARN_STREAM("Planned trajectory is empty. It will not be published!");
        }
    }

    void PlanDelegator::lookupFrontBumperTransform() 
    {
        tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));
        tf2_buffer_.setUsingDedicatedThread(true);
        try
        {
            geometry_msgs::TransformStamped tf = tf2_buffer_.lookupTransform("base_link", "vehicle_front", ros::Time(0), ros::Duration(20.0)); //save to local copy of transform 20 sec timeout
            length_to_front_bumper_ = tf.transform.translation.x;
            ROS_DEBUG_STREAM("length_to_front_bumper_: " << length_to_front_bumper_);
            
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
    }

}
