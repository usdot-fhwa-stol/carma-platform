/*
 * Copyright (C) 2022-2023 LEIDOS.
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
#include <carma_wm/Geometry.hpp>
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
        config_.duration_to_signal_before_lane_change = declare_parameter<double>("duration_to_signal_before_lane_change", config_.duration_to_signal_before_lane_change);
        config_.tactical_plugin_service_call_timeout = declare_parameter<int>("tactical_plugin_service_call_timeout", config_.tactical_plugin_service_call_timeout);
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
        get_parameter<double>("duration_to_signal_before_lane_change", config_.duration_to_signal_before_lane_change);
        get_parameter<int>("tactical_plugin_service_call_timeout", config_.tactical_plugin_service_call_timeout);

        RCLCPP_INFO_STREAM(rclcpp::get_logger("plan_delegator"),"Done loading parameters: " << config_);

        // Setup publishers
        traj_pub_ = create_publisher<carma_planning_msgs::msg::TrajectoryPlan>("plan_trajectory", 5);
        upcoming_lane_change_status_pub_ = create_publisher<carma_planning_msgs::msg::UpcomingLaneChangeStatus>("upcoming_lane_change_status", 1);
        turn_signal_command_pub_ = create_publisher<autoware_msgs::msg::LampCmd>("lamp_cmd", 1);

        // Setup subscribers
        plan_sub_ = create_subscription<carma_planning_msgs::msg::ManeuverPlan>("final_maneuver_plan", 5, std::bind(&PlanDelegator::maneuverPlanCallback, this, std_ph::_1));
        twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("current_velocity", 5,
            [this](geometry_msgs::msg::TwistStamped::UniquePtr twist) {this->latest_twist_ = *twist;});
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 5, std::bind(&PlanDelegator::poseCallback, this, std_ph::_1));
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

        // Update upcoming_lane_change_information_ and current_lane_change_information_ based on the received maneuver plan
        if(!latest_maneuver_plan_.maneuvers.empty()){
            // Get ego vehicle's current downtrack
            lanelet::BasicPoint2d current_loc(latest_pose_.pose.position.x, latest_pose_.pose.position.y);
            double current_downtrack = wm_->routeTrackPos(current_loc).downtrack;

            // Set upcoming_lane_change_information_ based on the first found lane change in the plan that begins after current_downtrack, if one exists
            upcoming_lane_change_information_ = boost::optional<LaneChangeInformation>(); // Reset to empty optional
            for(const auto& maneuver : latest_maneuver_plan_.maneuvers){
                if(maneuver.type == carma_planning_msgs::msg::Maneuver::LANE_CHANGE){
                    if(current_downtrack >= maneuver.lane_change_maneuver.start_dist){
                        // Skip this lane change maneuver since ego vehicle has passed the lane change start point (this is not an 'upcoming' lane change)
                        continue;
                    }
                    else{
                        LaneChangeInformation upcoming_lane_change_information = getLaneChangeInformation(maneuver);
                        upcoming_lane_change_information_ = boost::optional<LaneChangeInformation>(upcoming_lane_change_information);
                        break;
                    }
                }
            }

            // Set current_lane_change_information_ if the first maneuver is a lane change
            current_lane_change_information_ = boost::optional<LaneChangeInformation>(); // Reset to empty optional
            if(latest_maneuver_plan_.maneuvers[0].type == carma_planning_msgs::msg::Maneuver::LANE_CHANGE){
                LaneChangeInformation current_lane_change_information = getLaneChangeInformation(latest_maneuver_plan_.maneuvers[0]);
                current_lane_change_information_ = boost::optional<LaneChangeInformation>(current_lane_change_information);
            }
        }
    }

    void PlanDelegator::poseCallback(geometry_msgs::msg::PoseStamped::UniquePtr pose_msg)
    {
        latest_pose_ = *pose_msg;

        // Publish the upcoming lane change status
        publishUpcomingLaneChangeStatus(upcoming_lane_change_information_);

        // Publish the current turn signal command
        publishTurnSignalCommand(current_lane_change_information_, upcoming_lane_change_status_);
    }

    LaneChangeInformation PlanDelegator::getLaneChangeInformation(const carma_planning_msgs::msg::Maneuver& lane_change_maneuver){
        LaneChangeInformation lane_change_information;

        lane_change_information.starting_downtrack = lane_change_maneuver.lane_change_maneuver.start_dist;

        // Get the starting and ending lanelets for this lane change maneuver
        lanelet::ConstLanelet starting_lanelet = wm_->getMap()->laneletLayer.get(std::stoi(lane_change_maneuver.lane_change_maneuver.starting_lane_id));
        lanelet::ConstLanelet ending_lanelet = wm_->getMap()->laneletLayer.get(std::stoi(lane_change_maneuver.lane_change_maneuver.ending_lane_id));

        // Determine if lane change is a left or right lane change and update lane_change_information accordingly
        bool shared_boundary_found = false;

        lanelet::ConstLanelet current_lanelet = starting_lanelet;

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"), "Searching for shared boundary with starting lanechange lanelet " << std::to_string(current_lanelet.id()) << " and ending lanelet " << std::to_string(ending_lanelet.id()));
        while(!shared_boundary_found){
            // Assumption: Adjacent lanelets share lane boundary

            if(current_lanelet.leftBound() == ending_lanelet.rightBound()){   
                // If current lanelet's left lane boundary matches the ending lanelet's right lane boundary, it is a left lane change
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"), "Lanelet " << std::to_string(current_lanelet.id()) << " shares left boundary with " << std::to_string(ending_lanelet.id()));
                lane_change_information.is_right_lane_change = false;
                shared_boundary_found = true;
            }
            else if(current_lanelet.rightBound() == ending_lanelet.leftBound()){
                // If current lanelet's right lane boundary matches the ending lanelet's left lane boundary, it is a right lane change
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"), "Lanelet " << std::to_string(current_lanelet.id()) << " shares right boundary with " << std::to_string(ending_lanelet.id()));
                lane_change_information.is_right_lane_change = true;
                shared_boundary_found = true;
            }
            else{
                // If there are no following lanelets on route, lanechange should be completing before reaching it
                if(wm_->getMapRoutingGraph()->following(current_lanelet, false).empty())
                {
                    // Maneuver requires we travel further before completing lane change, but there is no routable lanelet directly ahead;
                    // in this case we have reached a lanelet which does not have a routable lanelet ahead and isn't adjacent to the lanelet where lane change ends.
                    // A lane change should have already happened at this point
                    throw(std::invalid_argument("No following lanelets from current lanelet reachable without a lane change, incorrectly chosen end lanelet"));
                }

                current_lanelet = wm_->getMapRoutingGraph()->following(current_lanelet, false).front(); 
                if(current_lanelet.id() == starting_lanelet.id()){
                    //Looped back to starting lanelet
                    throw(std::invalid_argument("No lane change in path"));
                }
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"), "Now checking for shared lane boundary with lanelet " << std::to_string(current_lanelet.id()) << " and ending lanelet " << std::to_string(ending_lanelet.id()));
            }
        }

        return lane_change_information;
    }

    void PlanDelegator::publishUpcomingLaneChangeStatus(const boost::optional<LaneChangeInformation>& upcoming_lane_change_information){
        // Initialize an UpcomingLaneChangeStatus message, which will be populated based on upcoming_lane_change_information
        carma_planning_msgs::msg::UpcomingLaneChangeStatus upcoming_lane_change_status;

        // Update upcoming_lane_change_status
        if(upcoming_lane_change_information){
            // Get the downtrack distance between the ego vehicle and the start of the upcoming lane change maneuver
            lanelet::BasicPoint2d current_loc(latest_pose_.pose.position.x, latest_pose_.pose.position.y);
            double current_downtrack = wm_->routeTrackPos(current_loc).downtrack;       
            upcoming_lane_change_status.downtrack_until_lanechange = std::max(0.0, upcoming_lane_change_information.get().starting_downtrack - current_downtrack);

            // Set upcoming lane change status as a right lane change or left lane change
            if(upcoming_lane_change_information.get().is_right_lane_change){
                upcoming_lane_change_status.lane_change = carma_planning_msgs::msg::UpcomingLaneChangeStatus::RIGHT;
            }
            else{
                upcoming_lane_change_status.lane_change = carma_planning_msgs::msg::UpcomingLaneChangeStatus::LEFT;
            }
        }
        else{
            upcoming_lane_change_status.lane_change = carma_planning_msgs::msg::UpcomingLaneChangeStatus::NONE;
        }

        // Publish upcoming_lane_change_status
        upcoming_lane_change_status_pub_->publish(upcoming_lane_change_status);

        // Store UpcomingLaneChangeStatus in upcoming_lane_change_status_
        upcoming_lane_change_status_ = upcoming_lane_change_status;
    }

    void PlanDelegator::publishTurnSignalCommand(const boost::optional<LaneChangeInformation>& current_lane_change_information, const carma_planning_msgs::msg::UpcomingLaneChangeStatus& upcoming_lane_change_status)
    {
        // Initialize turn signal command message
        // NOTE: A LampCmd message can have its 'r' OR 'l' field set to 1 to indicate an activated right or left turn signal, respectively. Both fields cannot be set to 1 at the same time.
        autoware_msgs::msg::LampCmd turn_signal_command;

        // Publish turn signal command with priority placed on the current lane change, if one exists
        if(current_lane_change_information){
            // Publish turn signal command for the current lane change based on the lane change direction
            if(current_lane_change_information.get().is_right_lane_change){
                turn_signal_command.r = 1;
            }
            else{
                turn_signal_command.l = 1;
            }
            turn_signal_command_pub_->publish(turn_signal_command);
        }
        else if(upcoming_lane_change_status.lane_change != carma_planning_msgs::msg::UpcomingLaneChangeStatus::NONE){
            // Only publish turn signal command for upcoming lane change if it will begin in less than the time defined by config_.duration_to_signal_before_lane_change
            if((upcoming_lane_change_status.downtrack_until_lanechange / latest_twist_.twist.linear.x) <= config_.duration_to_signal_before_lane_change){
                if(upcoming_lane_change_status.lane_change == carma_planning_msgs::msg::UpcomingLaneChangeStatus::RIGHT){
                    turn_signal_command.r = 1;
                }
                else{
                    turn_signal_command.l = 1;
                }
                turn_signal_command_pub_->publish(turn_signal_command);
            }
        }
        else{
            // Publish turn signal command with neither turn signal activated
            turn_signal_command_pub_->publish(turn_signal_command);
        }

        // Store turn signal command in latest_turn_signal_command_
        latest_turn_signal_command_ = turn_signal_command;
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
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"), "latest_pose_.header.stamp: " << std::to_string(rclcpp::Time(latest_pose_.header.stamp).seconds()));
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"), "plan_req->header.stamp: " << std::to_string(rclcpp::Time(plan_req->header.stamp).seconds()));

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
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"), "plan_req->header.stamp: " << std::to_string(rclcpp::Time(plan_req->header.stamp).seconds()));

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

        // Shift maneuver starting and ending lanelets
        // NOTE: Assumes that maneuver start and end downtrack distances have not been shifted by more than one lanelet
        if(maneuver.type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING && !maneuver.lane_following_maneuver.lane_ids.empty()){
            // (1) Add new beginning lanelet to maneuver if necessary and (2) remove ending lanelet from maneuver if necessary

            // Obtain the original starting lanelet from the maneuver
            lanelet::Id original_starting_lanelet_id = std::stoi(maneuver.lane_following_maneuver.lane_ids.front());
            lanelet::ConstLanelet original_starting_lanelet = wm_->getMap()->laneletLayer.get(original_starting_lanelet_id);

            // Get the downtrack of the start of the original starting lanelet
            lanelet::BasicPoint2d original_starting_lanelet_centerline_start_point = lanelet::utils::to2D(original_starting_lanelet.centerline()).front();
            double original_starting_lanelet_centerline_start_point_dt = wm_->routeTrackPos(original_starting_lanelet_centerline_start_point).downtrack;

            if(adjusted_start_dist < original_starting_lanelet_centerline_start_point_dt){
                auto previous_lanelets = wm_->getMapRoutingGraph()->previous(original_starting_lanelet, false);

                if(!previous_lanelets.empty()){
                    // lane_ids array is ordered by increasing downtrack, so this new starting lanelet is inserted at the front
                    maneuver.lane_following_maneuver.lane_ids.insert(maneuver.lane_following_maneuver.lane_ids.begin(), std::to_string(previous_lanelets[0].id()));

                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"), "Inserted lanelet " << std::to_string(previous_lanelets[0].id()) << " to beginning of maneuver.");
                }
                else{
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("plan_delegator"), "No previous lanelet was found for lanelet " << original_starting_lanelet.id());
                }
            }

            // Obtain the maneuver ending lanelet
            lanelet::Id original_ending_lanelet_id = std::stoi(maneuver.lane_following_maneuver.lane_ids.back());
            lanelet::ConstLanelet original_ending_lanelet = wm_->getMap()->laneletLayer.get(original_ending_lanelet_id);

            // Get the downtrack of the start of the maneuver ending lanelet
            lanelet::BasicPoint2d original_ending_lanelet_centerline_start_point = lanelet::utils::to2D(original_ending_lanelet.centerline()).front();
            double original_ending_lanelet_centerline_start_point_dt = wm_->routeTrackPos(original_ending_lanelet_centerline_start_point).downtrack;

            if(adjusted_end_dist < original_ending_lanelet_centerline_start_point_dt){
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("plan_delegator"),"Original ending lanelet " << original_ending_lanelet.id() << " removed from lane_ids since the updated maneuver no longer crosses it");
                
                // lane_ids array is ordered by increasing downtrack, so the last element in the array corresponds to the original ending lanelet
                maneuver.lane_following_maneuver.lane_ids.pop_back();
            }
        }   
        else if (maneuver.type != carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING){
            // (1) Update starting maneuver lanelet if necessary and (2) Update ending maneuver lanelet if necessary

            // Obtain the original starting lanelet from the maneuver
            lanelet::Id original_starting_lanelet_id = std::stoi(getManeuverStartingLaneletId(maneuver));
            lanelet::ConstLanelet original_starting_lanelet = wm_->getMap()->laneletLayer.get(original_starting_lanelet_id);

            // Get the downtrack of the start of the lanelet
            lanelet::BasicPoint2d original_starting_lanelet_centerline_start_point = lanelet::utils::to2D(original_starting_lanelet.centerline()).front();
            double original_starting_lanelet_centerline_start_point_dt = wm_->routeTrackPos(original_starting_lanelet_centerline_start_point).downtrack;

            if(adjusted_start_dist < original_starting_lanelet_centerline_start_point_dt){
                auto previous_lanelets = wm_->getMapRoutingGraph()->previous(original_starting_lanelet, false);

                if(!previous_lanelets.empty()){
                    setManeuverStartingLaneletId(maneuver, previous_lanelets[0].id());
                }
                else{
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("plan_delegator"), "No previous lanelet was found for lanelet " << original_starting_lanelet.id());
                }
            }

            // Obtain the original ending lanelet from the maneuver
            lanelet::Id original_ending_lanelet_id = std::stoi(getManeuverEndingLaneletId(maneuver));
            lanelet::ConstLanelet original_ending_lanelet = wm_->getMap()->laneletLayer.get(original_ending_lanelet_id);

            // Get the downtrack of the start of the ending lanelet
            lanelet::BasicPoint2d original_ending_lanelet_centerline_start_point = lanelet::utils::to2D(original_ending_lanelet.centerline()).front();
            double original_ending_lanelet_centerline_start_point_dt = wm_->routeTrackPos(original_ending_lanelet_centerline_start_point).downtrack;

            if(adjusted_end_dist < original_ending_lanelet_centerline_start_point_dt){
                auto previous_lanelets = wm_->getMapRoutingGraph()->previous(original_ending_lanelet, false);

                if(!previous_lanelets.empty()){
                    setManeuverEndingLaneletId(maneuver, previous_lanelets[0].id());
                }
                else{
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("plan_delegator"), "No previous lanelet was found for lanelet " << original_starting_lanelet.id());
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
            
            auto future_status = plan_response.wait_for(std::chrono::milliseconds(config_.tactical_plugin_service_call_timeout));

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
