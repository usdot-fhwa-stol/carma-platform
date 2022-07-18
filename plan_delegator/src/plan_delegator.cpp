/*
 * Copyright (C) 2022 LEIDOS.
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
#include <carma_wm_ros2/Geometry.hpp>
#include "plan_delegator.hpp"

namespace plan_delegator
{
    namespace std_ph = std::placeholders;

    namespace 
    {
        /**
         * \brief Anonymous function to set the starting_lane_id for all maneuver types except lane following. This
         * maneuver parameter cannot be set with SET_MANEUVER_PROPERTY calls since it is not included in
         * LANE_FOLLOW maneuvers.
         */ 
        void setManeuverStartingLaneletId(carma_planning_msgs::msg::Maneuver& mvr, lanelet::Id start_id) {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"Updating maneuver starting_lane_id to " << start_id);

            switch(mvr.type) {
                case carma_planning_msgs::msg::Maneuver::LANE_CHANGE:
                    mvr.lane_change_maneuver.starting_lane_id = std::to_string(start_id);
                    break;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                    mvr.intersection_transit_straight_maneuver.starting_lane_id = std::to_string(start_id);
                    break;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                    mvr.intersection_transit_left_turn_maneuver.starting_lane_id = std::to_string(start_id);
                    break;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                    mvr.intersection_transit_right_turn_maneuver.starting_lane_id = std::to_string(start_id);
                    break;
                case carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT:
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
        void setManeuverEndingLaneletId(carma_planning_msgs::msg::Maneuver& mvr, lanelet::Id end_id) {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"Updating maneuver ending_lane_id to " << end_id);

            switch(mvr.type) {
                case carma_planning_msgs::msg::Maneuver::LANE_CHANGE:
                    mvr.lane_change_maneuver.ending_lane_id = std::to_string(end_id);
                    break;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                    mvr.intersection_transit_straight_maneuver.ending_lane_id = std::to_string(end_id);
                    break;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                    mvr.intersection_transit_left_turn_maneuver.ending_lane_id = std::to_string(end_id);
                    break;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                    mvr.intersection_transit_right_turn_maneuver.ending_lane_id = std::to_string(end_id);
                    break;
                case carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT:
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
        std::string getManeuverStartingLaneletId(carma_planning_msgs::msg::Maneuver mvr) {
            switch(mvr.type) {
                case carma_planning_msgs::msg::Maneuver::LANE_CHANGE:
                    return mvr.lane_change_maneuver.starting_lane_id;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                    return mvr.intersection_transit_straight_maneuver.starting_lane_id;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                    return mvr.intersection_transit_left_turn_maneuver.starting_lane_id;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                    return mvr.intersection_transit_right_turn_maneuver.starting_lane_id;
                case carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT:
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
        std::string getManeuverEndingLaneletId(carma_planning_msgs::msg::Maneuver mvr) {
            switch(mvr.type) {
                case carma_planning_msgs::msg::Maneuver::LANE_CHANGE:
                    return mvr.lane_change_maneuver.ending_lane_id;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                    return mvr.intersection_transit_straight_maneuver.ending_lane_id;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                    return mvr.intersection_transit_left_turn_maneuver.ending_lane_id;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                    return mvr.intersection_transit_right_turn_maneuver.ending_lane_id;
                case carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT:
                    return mvr.stop_and_wait_maneuver.ending_lane_id;
                default:
                    throw std::invalid_argument("Maneuver type does not have starting and ending lane ids");
            }
        }
    } //namespace


    PlanDelegator::PlanDelegator(const rclcpp::NodeOptions &options) : carma_ros2_utils::CarmaLifecycleNode(options),
                                                                        tf2_buffer_(this->get_clock()),
                                                                        wml_(this->get_node_base_interface(), this->get_node_logging_interface(),
                                                                            this->get_node_topics_interface(), this->get_node_parameters_interface())
    {
        // Create initial config
        config_ = Config();

        config_.planning_topic_prefix = declare_parameter<std::string>("planning_topic_prefix", config_.planning_topic_prefix);      
        config_.planning_topic_suffix = declare_parameter<std::string>("planning_topic_suffix", config_.planning_topic_suffix);
        config_.trajectory_planning_rate = declare_parameter<double>("trajectory_planning_rate", config_.trajectory_planning_rate); 
        config_.max_trajectory_duration = declare_parameter<double>("trajectory_duration_threshold", config_.max_trajectory_duration);
        config_.min_crawl_speed = declare_parameter<double>("min_speed", config_.min_crawl_speed);
    }

    carma_ros2_utils::CallbackReturn PlanDelegator::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        // Reset config
        config_ = Config();

        get_parameter<std::string>("planning_topic_prefix", config_.planning_topic_prefix);      
        get_parameter<std::string>("planning_topic_suffix", config_.planning_topic_suffix);
        get_parameter<double>("trajectory_planning_rate", config_.trajectory_planning_rate); 
        get_parameter<double>("trajectory_duration_threshold", config_.max_trajectory_duration);
        get_parameter<double>("min_speed", config_.min_crawl_speed);

        RCLCPP_INFO_STREAM(rclcpp::get_logger("plan_delegator"),"Done loading parameters: " << config_);

        traj_pub_ = create_publisher<carma_planning_msgs::msg::TrajectoryPlan>("plan_trajectory", 5);
        plan_sub_ = create_subscription<carma_planning_msgs::msg::ManeuverPlan>("final_maneuver_plan", 5, std::bind(&PlanDelegator::maneuverPlanCallback, this, std_ph::_1));
        twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("current_velocity", 5,
            [this](geometry_msgs::msg::TwistStamped::UniquePtr twist) {this->latest_twist_ = *twist;});
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 5,
            [this](geometry_msgs::msg::PoseStamped::UniquePtr pose) {this->latest_pose_ = *pose;});
        guidance_state_sub_ = create_subscription<carma_planning_msgs::msg::GuidanceState>("guidance_state", 5,  std::bind(&PlanDelegator::guidanceStateCallback, this, std_ph::_1));

        lookupFrontBumperTransform();
        wm_ = wml_.getWorldModel();
        return CallbackReturn::SUCCESS;
    }
      
    carma_ros2_utils::CallbackReturn PlanDelegator::handle_on_activate(const rclcpp_lifecycle::State &)
    {
        traj_timer_ = create_timer(get_clock(), 
            std::chrono::milliseconds((int)(1 / config_.trajectory_planning_rate * 1000)),
            std::bind(&PlanDelegator::onTrajPlanTick, this));
         return CallbackReturn::SUCCESS;
    }
    
    void PlanDelegator::guidanceStateCallback(carma_planning_msgs::msg::GuidanceState::UniquePtr msg)
    {
        guidance_engaged = (msg->state == carma_planning_msgs::msg::GuidanceState::ENGAGED);
    }

    void PlanDelegator::maneuverPlanCallback(carma_planning_msgs::msg::ManeuverPlan::UniquePtr plan)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("plan_delegator"),"Received request to delegate plan ID " << std::string(plan->maneuver_plan_id));
        // do basic check to see if the input is valid
        auto copy_plan = *plan;

        if (isManeuverPlanValid(copy_plan))
        {
            latest_maneuver_plan_ = copy_plan;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"Received plan with " << latest_maneuver_plan_.maneuvers.size() << " maneuvers");
            
            // Update the parameters associated with each maneuver
            for (auto& maneuver : latest_maneuver_plan_.maneuvers) {
                updateManeuverParameters(maneuver);
            }
        }
        else {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("plan_delegator"),"Received empty plan, no maneuvers found in plan ID " << std::string(plan->maneuver_plan_id));
        }
    }

    carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> PlanDelegator::getPlannerClientByName(const std::string& planner_name)
    {
        if(planner_name.size() == 0)
        {
            throw std::invalid_argument("Invalid trajectory planner name because it has zero length!");
        }
        if(trajectory_planners_.find(planner_name) == trajectory_planners_.end())
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("plan_delegator"),"Discovered new trajectory planner: " << planner_name);
            
            trajectory_planners_.emplace(
                planner_name, create_client<carma_planning_msgs::srv::PlanTrajectory>(config_.planning_topic_prefix + planner_name + config_.planning_topic_suffix));
        }
        return trajectory_planners_[planner_name];
    }

    bool PlanDelegator::isManeuverPlanValid(const carma_planning_msgs::msg::ManeuverPlan& maneuver_plan) const noexcept
    {
        // currently it only checks if maneuver list is empty
        return !maneuver_plan.maneuvers.empty();
    }

    bool PlanDelegator::isTrajectoryValid(const carma_planning_msgs::msg::TrajectoryPlan& trajectory_plan) const noexcept
    {
        // currently it only checks if trajectory contains less than 2 points
        return !(trajectory_plan.trajectory_points.size() < 2);
    }

    bool PlanDelegator::isManeuverExpired(const carma_planning_msgs::msg::Maneuver& maneuver, rclcpp::Time current_time) const
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"), "maneuver start time:" << std::to_string(rclcpp::Time(GET_MANEUVER_PROPERTY(maneuver, start_time)).seconds()));
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"), "maneuver end time:" << std::to_string(rclcpp::Time(GET_MANEUVER_PROPERTY(maneuver, end_time)).seconds()));
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"), "current time:" << std::to_string(now().seconds()));
        bool isexpired = rclcpp::Time(GET_MANEUVER_PROPERTY(maneuver, end_time), get_clock()->get_clock_type()) <= current_time; // TODO maneuver expiration should maybe be based off of distance not time? https://github.com/usdot-fhwa-stol/carma-platform/issues/1107
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"), "isexpired:" << isexpired);
        // TODO: temporary disabling expiration check
        return false;
    }
    
    std::shared_ptr<carma_planning_msgs::srv::PlanTrajectory::Request> PlanDelegator::composePlanTrajectoryRequest(const carma_planning_msgs::msg::TrajectoryPlan& latest_trajectory_plan, const uint16_t& current_maneuver_index) const
    {
        auto plan_req = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Request>();
        plan_req->maneuver_plan = latest_maneuver_plan_;

        // set current vehicle state if we have NOT planned any previous trajectories
        if(latest_trajectory_plan.trajectory_points.empty())
        {
            plan_req->header.stamp = latest_pose_.header.stamp;
            plan_req->vehicle_state.longitudinal_vel = latest_twist_.twist.linear.x;
            plan_req->vehicle_state.x_pos_global = latest_pose_.pose.position.x;
            plan_req->vehicle_state.y_pos_global = latest_pose_.pose.position.y;
            double roll, pitch, yaw;
            carma_wm::geometry::rpyFromQuaternion(latest_pose_.pose.orientation, roll, pitch, yaw);
            plan_req->vehicle_state.orientation = yaw;
            plan_req->maneuver_index_to_plan = current_maneuver_index;
        }
        // set vehicle state based on last two planned trajectory points
        else
        {
            carma_planning_msgs::msg::TrajectoryPlanPoint last_point = latest_trajectory_plan.trajectory_points.back();
            carma_planning_msgs::msg::TrajectoryPlanPoint second_last_point = *(latest_trajectory_plan.trajectory_points.rbegin() + 1);
            plan_req->vehicle_state.x_pos_global = last_point.x;
            plan_req->vehicle_state.y_pos_global = last_point.y;
            auto distance_diff = std::sqrt(std::pow(last_point.x - second_last_point.x, 2) + std::pow(last_point.y - second_last_point.y, 2));
            rclcpp::Duration time_diff = rclcpp::Time(last_point.target_time) - rclcpp::Time(second_last_point.target_time);
            auto time_diff_sec = time_diff.seconds();
            plan_req->maneuver_index_to_plan = current_maneuver_index;
            // this assumes the vehicle does not have significant lateral velocity
            plan_req->header.stamp = latest_trajectory_plan.trajectory_points.back().target_time;
            plan_req->vehicle_state.longitudinal_vel = distance_diff / time_diff_sec;
            // TODO develop way to set yaw value for future points
        }
        return plan_req;
    }

    bool PlanDelegator::isTrajectoryLongEnough(const carma_planning_msgs::msg::TrajectoryPlan& plan) const noexcept
    {
        rclcpp::Duration time_diff = rclcpp::Time(plan.trajectory_points.back().target_time) - rclcpp::Time(plan.trajectory_points.front().target_time);
        return time_diff.seconds() >= config_.max_trajectory_duration;
    }

    void PlanDelegator::updateManeuverParameters(carma_planning_msgs::msg::Maneuver& maneuver)
    {
        if (!wm_->getMap())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("plan_delegator"), "Map is not set yet");
            return;
        }
        // Update maneuver starting and ending downtrack distances
        double original_start_dist = GET_MANEUVER_PROPERTY(maneuver, start_dist);
        double original_end_dist = GET_MANEUVER_PROPERTY(maneuver, end_dist);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"Changing maneuver distances for planner: " << GET_MANEUVER_PROPERTY(maneuver, parameters.planning_tactical_plugin));
        double adjusted_start_dist = original_start_dist - length_to_front_bumper_;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"original_start_dist:" << original_start_dist);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"adjusted_start_dist:" << adjusted_start_dist);
        double adjusted_end_dist = original_end_dist - length_to_front_bumper_;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"original_end_dist:" << original_end_dist);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"adjusted_end_dist:" << adjusted_end_dist);
        SET_MANEUVER_PROPERTY(maneuver, start_dist, adjusted_start_dist);
        SET_MANEUVER_PROPERTY(maneuver, end_dist, adjusted_end_dist);

        // Get the lanelets crossed by the updated maneuver (considers full route; not just shortest path)
        std::vector<lanelet::ConstLanelet> adjusted_crossed_lanelets = wm_->getLaneletsBetween(adjusted_start_dist, adjusted_end_dist, false, false);
        
        if (adjusted_crossed_lanelets.size() == 0) {
            throw std::invalid_argument("The adjusted maneuver does not cross any lanelets going from: " + std::to_string(adjusted_start_dist) + " to " + std::to_string(adjusted_end_dist));
        }

        // Update maneuver-specific lanelet ID parameters
        // Note: Assumes that the maneuver start and end distances are adjusted by a distance less than the length of a lanelet. 
        if(maneuver.type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING && !maneuver.lane_following_maneuver.lane_ids.empty()) 
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
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"Lanelet " << lanelet.id() << " inserted at the front of maneuver's lane_ids");

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
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"Original ending lanelet " << original_ending_lanelet.id() << " removed from lane_ids since the updated maneuver no longer crosses it");
                
                // lane_ids array is ordered by increasing downtrack, so the last element in the array corresponds to the original ending lanelet
                maneuver.lane_following_maneuver.lane_ids.pop_back();
            }
        } 
        else if (maneuver.type != carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING)
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

    carma_planning_msgs::msg::TrajectoryPlan PlanDelegator::planTrajectory()
    {
        carma_planning_msgs::msg::TrajectoryPlan latest_trajectory_plan;
        if(!guidance_engaged)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("plan_delegator"),"Guidance is not engaged. Plan delegator will not plan trajectory.");
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

            // ignore expired maneuvers
            if(isManeuverExpired(maneuver, get_clock()->now()))
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("plan_delegator"),"Dropping expired maneuver: " << GET_MANEUVER_PROPERTY(maneuver, parameters.maneuver_id));
                // Update the maneuver plan index for the next loop
                ++current_maneuver_index;
                continue;
            }
            lanelet::BasicPoint2d current_loc(latest_pose_.pose.position.x, latest_pose_.pose.position.y);
            double current_downtrack = wm_->routeTrackPos(current_loc).downtrack;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"current_downtrack" << current_downtrack);
            double maneuver_end_dist = GET_MANEUVER_PROPERTY(maneuver, end_dist);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"maneuver_end_dist" << maneuver_end_dist);

            // ignore maneuver that is passed.
            if (current_downtrack > maneuver_end_dist)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("plan_delegator"),"Dropping passed maneuver: " << GET_MANEUVER_PROPERTY(maneuver, parameters.maneuver_id));
                // Update the maneuver plan index for the next loop
                ++current_maneuver_index;
                continue;
            }
            

            // get corresponding ros service client for plan trajectory
            auto maneuver_planner = GET_MANEUVER_PROPERTY(maneuver, parameters.planning_tactical_plugin);
            
            auto client = getPlannerClientByName(maneuver_planner);
            
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"Current planner: " << maneuver_planner);

            // compose service request
            auto plan_req = composePlanTrajectoryRequest(latest_trajectory_plan, current_maneuver_index);
            
            auto plan_response = client->async_send_request(plan_req);
            
            auto future_status = plan_response.wait_for(std::chrono::milliseconds(100));

            // Wait for the result.
            if (future_status == std::future_status::ready)
            {
                // validate trajectory before add to the plan
                if(!isTrajectoryValid(plan_response.get()->trajectory_plan))
                {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("plan_delegator"),"Found invalid trajectory with less than 2 trajectory points for " << std::string(latest_maneuver_plan_.maneuver_plan_id));
                    break;
                }
                //Remove duplicate point from start of trajectory
                if(latest_trajectory_plan.trajectory_points.size() !=0){
                    
                    if(latest_trajectory_plan.trajectory_points.back().target_time == plan_response.get()->trajectory_plan.trajectory_points.front().target_time){
                        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"Removing duplicate point for planner: " << maneuver_planner);
                        plan_response.get()->trajectory_plan.trajectory_points.erase(plan_response.get()->trajectory_plan.trajectory_points.begin());
                        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"plan_response.get()->trajectory_plan size: " << plan_response.get()->trajectory_plan.trajectory_points.size());

                    }
                }
                latest_trajectory_plan.trajectory_points.insert(latest_trajectory_plan.trajectory_points.end(),
                                                                plan_response.get()->trajectory_plan.trajectory_points.begin(),
                                                                plan_response.get()->trajectory_plan.trajectory_points.end());
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"new latest_trajectory_plan size: " << latest_trajectory_plan.trajectory_points.size());
                
                // Assign the trajectory plan's initial longitudinal velocity based on the first tactical plugin's response
                if(first_trajectory_plan == true)
                {
                    latest_trajectory_plan.initial_longitudinal_velocity = plan_response.get()->trajectory_plan.initial_longitudinal_velocity;
                    first_trajectory_plan = false;
                }

                if(isTrajectoryLongEnough(latest_trajectory_plan))
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("plan_delegator"),"Plan Trajectory completed for " << std::string(latest_maneuver_plan_.maneuver_plan_id));
                    break;
                }

                // Update the maneuver plan index based on the last maneuver index converted to a trajectory
                // This is required since inlanecruising_plugin can plan a trajectory over contiguous LANE_FOLLOWING maneuvers
                if(plan_response.get()->related_maneuvers.size() > 0)
                {
                    current_maneuver_index = plan_response.get()->related_maneuvers.back() + 1;
                } 
            }
            else
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("plan_delegator"),"Unsuccessful service call to trajectory planner:" << maneuver_planner << " for plan ID " << std::string(latest_maneuver_plan_.maneuver_plan_id));
                // if one service call fails, it should end plan immediately because it is there is no point to generate plan with empty space
                break;
            }
        }

        return latest_trajectory_plan;
    }
    
    void PlanDelegator::onTrajPlanTick()
    {
        carma_planning_msgs::msg::TrajectoryPlan trajectory_plan = planTrajectory();
        
        // Check if planned trajectory is valid before send out
        if(isTrajectoryValid(trajectory_plan))
        {
            trajectory_plan.header.stamp = get_clock()->now();
            traj_pub_->publish(trajectory_plan);
        }
        else
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("plan_delegator"),"Planned trajectory is empty. It will not be published!");
        }
    }

    void PlanDelegator::lookupFrontBumperTransform() 
    {
        tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));
        tf2_buffer_.setUsingDedicatedThread(true);
        try
        {
            geometry_msgs::msg::TransformStamped tf = tf2_buffer_.lookupTransform("base_link", "vehicle_front", rclcpp::Time(0), rclcpp::Duration(20.0, 0)); //save to local copy of transform 20 sec timeout
            length_to_front_bumper_ = tf.transform.translation.x;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"length_to_front_bumper_: " << length_to_front_bumper_);
            
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("plan_delegator"), ex.what());
        }
    }

} // namespace plan_delegator


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(plan_delegator::PlanDelegator)
