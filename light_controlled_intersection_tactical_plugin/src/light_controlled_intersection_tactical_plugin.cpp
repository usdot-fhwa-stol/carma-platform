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
#include "light_controlled_intersection_tactical_plugin/light_controlled_intersection_tactical_plugin_node.hpp"
#include <valarray>

namespace light_controlled_intersection_tactical_plugin
{


    LightControlledIntersectionTacticalPlugin::LightControlledIntersectionTacticalPlugin(carma_wm::WorldModelConstPtr wm, const Config& config, const std::string& plugin_name,
        std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh)
        :wm_(wm), config_(config), plugin_name_(plugin_name), nh_(nh)
    {
    }

    bool LightControlledIntersectionTacticalPlugin::isLastTrajectoryValid(
        const rclcpp::Time& current_time,
        double min_remaining_time_seconds) const
    {
        // Check if we have at least 2 points in the trajectory
        if (last_trajectory_.trajectory_points.size() < 2) {
            return false;
        }

        // Check if the last point's time is sufficiently in the future
        rclcpp::Time last_point_time = rclcpp::Time(last_trajectory_.trajectory_points.back().target_time);
        rclcpp::Duration min_time_remaining = rclcpp::Duration::from_seconds(min_remaining_time_seconds);

        if (last_point_time <= current_time + min_time_remaining) {
            return false;
        }

        // Check if we have case information from previous planning
        if (is_last_case_successful_ == boost::none || last_case_ == boost::none) {
            return false;
        }

        // Ensure we have consistent speed data
        if (last_final_speeds_.size() != last_trajectory_.trajectory_points.size()) {
            return false;
        }

        return true;
    }

    size_t LightControlledIntersectionTacticalPlugin::findClosestPointIndex(
        const lanelet::BasicPoint2d& position,
        const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& trajectory)
    {
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < trajectory.size(); i++)
        {
            auto dist = sqrt(pow(position.x() - trajectory.at(i).x, 2) +
        pow(position.y() - trajectory.at(i).y, 2));

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        return closest_idx;
    }

    bool LightControlledIntersectionTacticalPlugin::shouldUseLastTrajectory(
        TSCase new_case, bool is_new_case_successful, const rclcpp::Time& current_time)
    {
        // Validate trajectory with 1 second minimum remaining time
        if (!isLastTrajectoryValid(current_time, 1.0)) {
            return false;
        }

        // Check if case is the same and both cases are successful
        if (last_case_.get() == new_case && is_new_case_successful == true)
        {
            auto remaining_time = rclcpp::Time(last_trajectory_.trajectory_points.back().target_time) - current_time;
            // Only use last trajectory if we have at least 3 seconds remaining
            return remaining_time.seconds() >= 3.0;
        }

        // Edge case - successful to unsuccessful transition
        if (is_last_case_successful_.get() == true && is_new_case_successful == false)
        {
            if (last_successful_ending_downtrack_ - current_downtrack_ < config_.algorithm_evaluation_distance &&
                last_successful_scheduled_entry_time_ - current_time.seconds() < config_.algorithm_evaluation_period)
            {
                auto remaining_time = rclcpp::Time(last_trajectory_.trajectory_points.back().target_time) - current_time;
                return remaining_time.seconds() >= 3.0;
            }
        }

        return false;
    }

    carma_planning_msgs::msg::TrajectoryPlan LightControlledIntersectionTacticalPlugin::blendTrajectoryPoints(
        const carma_planning_msgs::msg::TrajectoryPlan& old_trajectory,
        const carma_planning_msgs::msg::TrajectoryPlan& new_trajectory,
        size_t transition_point)
    {
        // Create a blended trajectory
        carma_planning_msgs::msg::TrajectoryPlan blended_trajectory;
        blended_trajectory.header = old_trajectory.header;
        blended_trajectory.trajectory_id = old_trajectory.trajectory_id;

        // Validate inputs
        if (old_trajectory.trajectory_points.empty()) {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
           "Old trajectory is empty, using new trajectory");
            return new_trajectory;
        }

        if (new_trajectory.trajectory_points.empty()) {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
           "New trajectory is empty, using old trajectory");
            return old_trajectory;
        }

        // Ensure transition point is valid
        if (transition_point >= old_trajectory.trajectory_points.size()) {
            transition_point = old_trajectory.trajectory_points.size() / 2;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
           "Invalid transition point, using default: " << transition_point);
        }

        // Use trajectory points from old trajectory up to transition
        blended_trajectory.trajectory_points.insert(
            blended_trajectory.trajectory_points.end(),
            old_trajectory.trajectory_points.begin(),
            old_trajectory.trajectory_points.begin() + transition_point
        );

        // Add the new speeds after the transition point
        blended_trajectory.trajectory_points.insert(blended_trajectory.trajectory_points.end(),
    new_trajectory.trajectory_points.begin() + transition_point,
    new_trajectory.trajectory_points.end());

        return blended_trajectory;
    }

    bool LightControlledIntersectionTacticalPlugin::shouldBlendTrajectories(
        TSCase new_case, bool is_new_case_successful, const rclcpp::Time& current_time)
    {
        // Validate trajectory with 1 second minimum remaining time
        if (!isLastTrajectoryValid(current_time, 1.0)) {
            return false;
        }

        // Case 1: Same case, successful, but less than 3 seconds remain
        if (last_case_.get() == new_case && is_new_case_successful == true)
        {
            auto remaining_time = rclcpp::Time(last_trajectory_.trajectory_points.back().target_time) - current_time;
            if (remaining_time.seconds() < 3.0) return true;
        }

        // Case 2: Edge case, less than 3 seconds remain
        if (is_last_case_successful_.get() == true && is_new_case_successful == false &&
            last_successful_ending_downtrack_ - current_downtrack_ < config_.algorithm_evaluation_distance &&
            last_successful_scheduled_entry_time_ - current_time.seconds() < config_.algorithm_evaluation_period)
        {
            auto remaining_time = rclcpp::Time(last_trajectory_.trajectory_points.back().target_time) - current_time;
            if (remaining_time.seconds() < 3.0) return true;
        }

        // Case 3: Same case but different values
        if (last_case_.get() == new_case) return true;

        return false;
    }

    carma_planning_msgs::msg::TrajectoryPlan LightControlledIntersectionTacticalPlugin::reapplyTargetSpeedsToTrajectory(
        const carma_planning_msgs::msg::TrajectoryPlan& old_trajectory,
        const std::vector<carma_planning_msgs::msg::Maneuver>& maneuver_plan,
        const carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr& req,
        const std::vector<double>& final_speeds)
    {
        DetailedTrajConfig wpg_detail_config;

        wpg_detail_config = basic_autonomy::waypoint_generation::compose_detailed_trajectory_config(
            config_.trajectory_time_length,
            config_.curve_resample_step_size, config_.minimum_speed,
            config_.vehicle_accel_limit,
            config_.lateral_accel_limit,
            config_.speed_moving_average_window_size,
            config_.curvature_moving_average_window_size, config_.back_distance,
            config_.buffer_ending_downtrack);

        // Create trajectory with given speeds
        // TODO: implement the function that extracts points from trajectory points and assigns target speeds to return vector
        auto points_and_target_speeds = extractPointsAndAssign(old_trajectory.trajectory_points, final_speeds);

        // Generate points
        carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds debug_msg_unused;
        auto updated_trajectory = old_trajectory;
        updated_trajectory.trajectory_points = basic_autonomy::waypoint_generation::compose_lanefollow_trajectory_from_path(
            points_and_target_speeds,
            req->vehicle_state, req->header.stamp, wm_, ending_state_before_buffer_, debug_msg_unused,
            wpg_detail_config);

        return updated_trajectory;
    }
    std::vector<PointSpeedPair> LightControlledIntersectionTacticalPlugin::extractPointsAndAssign(
        const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& trajectory_points,
        const std::vector<double>& target_speeds)
    {
        // Validate inputs
        if (trajectory_points.empty()) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
                "Empty trajectory points provided to extractPointsAndTargetSpeeds");
            return std::vector<PointSpeedPair>();
        }

        if (target_speeds.empty()) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
                "Empty target speeds provided to extractPointsAndTargetSpeeds");
            return std::vector<PointSpeedPair>();
        }

        // Determine how many pairs we can create (minimum of the two sizes)
        size_t pair_count = std::min(trajectory_points.size(), target_speeds.size());

        if (trajectory_points.size() != target_speeds.size()) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
                "Size mismatch between trajectory points (" << trajectory_points.size() <<
                ") and target speeds (" << target_speeds.size() <<
                "). Using only the first " << pair_count << " elements.");
        }

        // Create vector to hold the points and target speeds
        std::vector<PointSpeedPair> points_and_speeds;
        points_and_speeds.reserve(pair_count);

        // Extract points from trajectory points and pair with target speeds
        for (size_t i = 0; i < pair_count; i++) {
            // Extract 3D position
            lanelet::BasicPoint2d basic_point(
                trajectory_points[i].x,
                trajectory_points[i].y
            );
            PointSpeedPair pair;
            pair.point = basic_point;
            pair.speed = target_speeds[i];
            // Create pair of point and speed
            points_and_speeds.emplace_back(pair);
        }

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
                        "Created " << points_and_speeds.size() << " point-speed pairs");

        return points_and_speeds;
    }

    carma_planning_msgs::msg::TrajectoryPlan LightControlledIntersectionTacticalPlugin::generateNewTrajectory(
        const std::vector<carma_planning_msgs::msg::Maneuver>& maneuver_plan,
        const carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr& req,
        std::vector<double>& final_speeds)
    {
        DetailedTrajConfig wpg_detail_config;
        GeneralTrajConfig wpg_general_config;

        wpg_general_config = basic_autonomy::waypoint_generation::compose_general_trajectory_config(
            "intersection_transit",
            config_.default_downsample_ratio,
            config_.turn_downsample_ratio);

        wpg_detail_config = basic_autonomy::waypoint_generation::compose_detailed_trajectory_config(
            config_.trajectory_time_length,
            config_.curve_resample_step_size, config_.minimum_speed,
            config_.vehicle_accel_limit,
            config_.lateral_accel_limit,
            config_.speed_moving_average_window_size,
            config_.curvature_moving_average_window_size, config_.back_distance,
            config_.buffer_ending_downtrack);

        // Create trajectory with raw speed limits from maneuver
        auto points_and_target_speeds = createGeometryProfile(
            maneuver_plan, std::max((double)0, current_downtrack_ - config_.back_distance),
            wm_, ending_state_before_buffer_, req->vehicle_state, wpg_general_config, wpg_detail_config);

        // Apply optimized speed profile
        applyOptimizedTargetSpeedProfile(maneuver_plan.front(), req->vehicle_state.longitudinal_vel, points_and_target_speeds);

        // Create new trajectory
        carma_planning_msgs::msg::TrajectoryPlan new_trajectory;
        new_trajectory.header.frame_id = "map";
        new_trajectory.header.stamp = req->header.stamp;
        new_trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

        // Generate points
        new_trajectory.trajectory_points = basic_autonomy::waypoint_generation::compose_lanefollow_trajectory_from_path(
            points_and_target_speeds,
            req->vehicle_state, req->header.stamp, wm_, ending_state_before_buffer_, debug_msg_,
            wpg_detail_config);

        // Set the planning plugin field name
        for (auto& p : new_trajectory.trajectory_points) {
            p.planner_plugin_name = plugin_name_;
        }

        // Save final speeds
        final_speeds = debug_msg_.velocity_profile;

        return new_trajectory;
    }

    // Find transition point based on halfway through the time range of the old trajectory
    size_t LightControlledIntersectionTacticalPlugin::findTimeBasedTransitionPoint(
        const carma_planning_msgs::msg::TrajectoryPlan& old_trajectory)
    {
        if (old_trajectory.trajectory_points.size() <= 1) {
            return 0;
        }

        // Get start and end times of the old trajectory
        rclcpp::Time start_time(old_trajectory.trajectory_points.front().target_time);
        rclcpp::Time end_time(old_trajectory.trajectory_points.back().target_time);

        // Calculate midpoint in time
        double total_duration = (end_time - start_time).seconds();
        rclcpp::Time mid_time = start_time + rclcpp::Duration::from_seconds(total_duration / 2.0);

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
    "Trajectory time range: " << start_time.seconds() << " to "
    << end_time.seconds() << ", midpoint: " << mid_time.seconds());

        // Find closest point to mid_time
        size_t closest_idx = 0;
        double min_time_diff = std::numeric_limits<double>::max();

        for (size_t i = 0; i < old_trajectory.trajectory_points.size(); i++) {
            rclcpp::Time point_time(old_trajectory.trajectory_points[i].target_time);
            double time_diff = std::abs((point_time - mid_time).seconds());

            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest_idx = i;
            }
        }

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
    "Selected transition point at index " << closest_idx
    << " (time: " << rclcpp::Time(old_trajectory.trajectory_points[closest_idx].target_time).seconds()
    << ") which is closest to time midpoint");

        return closest_idx;
    }

    // Modified blending function to use the time-based transition point
    std::vector<double> LightControlledIntersectionTacticalPlugin::blendSpeedProfiles(
        const std::vector<double>& old_speeds,
        const std::vector<double>& new_speeds,
        size_t transition_point)
    {
        // Validate inputs
        if (old_speeds.empty() || new_speeds.empty()) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "Empty speed profile provided for blending");
            return old_speeds.empty() ? new_speeds : old_speeds;
        }

        // Ensure transition point is valid
        if (transition_point >= old_speeds.size()) {
            transition_point = old_speeds.size() / 2;  // Default to half if invalid

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "Invalid transition point, using default: " << transition_point);
        }

        // Create the blended speed profile
        std::vector<double> blended_speeds;

        // Take the first portion from old speeds (up to transition point)
        blended_speeds.insert(blended_speeds.end(),
        old_speeds.begin(),
        old_speeds.begin() + transition_point);

        // Add the new speeds after the transition point
        blended_speeds.insert(blended_speeds.end(),
        new_speeds.begin() + transition_point + 1,
        new_speeds.end());

        // Apply smoothing using the moving average filter
        try {
            int window_size = config_.speed_moving_average_window_size;
            if (window_size % 2 == 0) window_size++; // Ensure odd window size

            // Apply smoothing to the entire profile
            std::vector<double> smoothed_speeds = basic_autonomy::smoothing::moving_average_filter(
                blended_speeds,
                window_size,
                false  // Don't ignore first point
            );

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "Successfully smoothed speed profile at transition point " << transition_point);

            return smoothed_speeds;

        } catch (const std::exception& e) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "Error smoothing speed profile: " << e.what() << ". Using basic blended profile.");
            return blended_speeds;
        }
    }

    // Main function refactored to use the helper functions
    void LightControlledIntersectionTacticalPlugin::planTrajectoryCB(
            carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req,
            carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
    {
        std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "Starting light controlled intersection trajectory planning");

        // Validate request
        if(req->maneuver_index_to_plan >= req->maneuver_plan.maneuvers.size())
        {
            throw std::invalid_argument(
                "Light Control Intersection Plugin asked to plan invalid maneuver index: " +
                std::to_string(req->maneuver_index_to_plan) +
                " for plan of size: " + std::to_string(req->maneuver_plan.maneuvers.size()));
        }

        // Extract maneuver plan
        std::vector<carma_planning_msgs::msg::Maneuver> maneuver_plan;
        if(req->maneuver_plan.maneuvers[req->maneuver_index_to_plan].type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING
            && GET_MANEUVER_PROPERTY(req->maneuver_plan.maneuvers[req->maneuver_index_to_plan],
            parameters.string_valued_meta_data.front()) == light_controlled_intersection_strategy_)
        {
            maneuver_plan.push_back(req->maneuver_plan.maneuvers[req->maneuver_index_to_plan]);
            resp->related_maneuvers.push_back(req->maneuver_index_to_plan);
        }
        else
        {
            throw std::invalid_argument("Light Control Intersection Plugin asked to plan unsupported maneuver");
        }

        // Get vehicle position and update tracking variables
        lanelet::BasicPoint2d veh_pos(req->vehicle_state.x_pos_global, req->vehicle_state.y_pos_global);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
    "Planning state x:" << req->vehicle_state.x_pos_global << " , y: " << req->vehicle_state.y_pos_global);

        current_downtrack_ = wm_->routeTrackPos(veh_pos).downtrack;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
    "Current_downtrack: "<< current_downtrack_);

        // Find current lanelet
        auto current_lanelets = wm_->getLaneletsFromPoint({req->vehicle_state.x_pos_global, req->vehicle_state.y_pos_global});
        lanelet::ConstLanelet current_lanelet;

        if (current_lanelets.empty())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "Given vehicle position is not on the road! Returning...");
            return;
        }

        // Get the lanelet that is on the route in case overlapping ones found
        auto llt_on_route_optional = wm_->getFirstLaneletOnShortestPath(current_lanelets);

        if (llt_on_route_optional){
            current_lanelet = llt_on_route_optional.value();
        }
        else{
            RCLCPP_WARN_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "When identifying the corresponding lanelet for requested trajectory plan's state, x: "
        << req->vehicle_state.x_pos_global << ", y: " << req->vehicle_state.y_pos_global
        << ", no possible lanelet was found to be on the shortest path."
        << "Picking arbitrary lanelet: " << current_lanelets[0].id() << ", instead");
            current_lanelet = current_lanelets[0];
        }

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
    "Current_lanelet: " << current_lanelet.id());

        speed_limit_ = findSpeedLimit(current_lanelet, wm_);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
    "speed_limit_: " << speed_limit_);

        // Get new case parameters
        bool is_new_case_successful = GET_MANEUVER_PROPERTY(maneuver_plan.front(), parameters.int_valued_meta_data[1]);
        TSCase new_case = static_cast<TSCase>GET_MANEUVER_PROPERTY(maneuver_plan.front(), parameters.int_valued_meta_data[0]);

        // Log debug info about previous trajectory
        if (is_last_case_successful_ != boost::none && last_case_ != boost::none)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "all variables are set!");
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "is_last_case_successful_.get(): " << (int)is_last_case_successful_.get());
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "evaluation distance: " << last_successful_ending_downtrack_ - current_downtrack_);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "evaluation time: " << std::to_string(last_successful_scheduled_entry_time_ -
        rclcpp::Time(req->header.stamp).seconds()));
        }
        else
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "Not all variables are set...");
        }

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
    "traj points size: " << last_trajectory_.trajectory_points.size() <<
    ", last_final_speeds_ size: " << last_final_speeds_.size());

        // Find closest point in last trajectory to current vehicle position
        size_t idx_to_start_new_traj = findClosestPointIndex(veh_pos, last_trajectory_.trajectory_points);

        // Update last trajectory to start from closest point
        if (!last_trajectory_.trajectory_points.empty()) {
            last_final_speeds_ = std::vector<double>(last_final_speeds_.begin() + idx_to_start_new_traj,
    last_final_speeds_.end());
            last_trajectory_.trajectory_points = std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>
                        (last_trajectory_.trajectory_points.begin() + idx_to_start_new_traj,
    last_trajectory_.trajectory_points.end());
        }

        // Check if we should use the last trajectory completely
        rclcpp::Time current_time = rclcpp::Time(req->header.stamp);
        if (shouldUseLastTrajectory(new_case, is_new_case_successful, current_time))
        {
            resp->trajectory_plan = last_trajectory_;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "USING LAST TRAJ: " << (int)last_case_.get());

            resp->trajectory_plan.initial_longitudinal_velocity = last_final_speeds_.front();
            resp->maneuver_status.push_back(carma_planning_msgs::srv::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

            std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now();
            auto duration = end_time - start_time;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "ExecutionTime Using Existing: " << std::chrono::duration<double>(duration).count());
            return;
        }

        // Generate a new trajectory - needed regardless of whether we blend or not
        std::vector<double> new_final_speeds;
        carma_planning_msgs::msg::TrajectoryPlan new_trajectory =
            generateNewTrajectory(maneuver_plan, req, new_final_speeds);

        // Check if we should blend trajectories
        if (shouldBlendTrajectories(new_case, is_new_case_successful, current_time) &&
            last_trajectory_.trajectory_points.size() >= 2 &&
            new_trajectory.trajectory_points.size() >= 2)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "Blending trajectories");

            // Find transition point based on halfway through time
            size_t transition_point = findTimeBasedTransitionPoint(last_trajectory_);

            // Blend the speed profiles
            std::vector<double> blended_speeds = blendSpeedProfiles(
                last_final_speeds_,
                new_final_speeds,
                transition_point
            );
            // Generate blended trajectory
            carma_planning_msgs::msg::TrajectoryPlan blended_trajectory =
                blendTrajectoryPoints(last_trajectory_, new_trajectory, transition_point);

            // Generate the trajectory based on new blended speeds
            resp->trajectory_plan = reapplyTargetSpeedsToTrajectory(
                blended_trajectory,
                maneuver_plan,
                req,
                blended_speeds);

            resp->trajectory_plan.initial_longitudinal_velocity = blended_speeds.front();

            // Update stored trajectories for next planning cycle
            last_trajectory_ = blended_trajectory;
            last_final_speeds_ = blended_speeds;

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "USING BLENDED TRAJECTORY for case: " << (int)new_case);
        }
        // Use the newly generated trajectory if blending is not possible or not needed
        else if (new_trajectory.trajectory_points.size() >= 2) {
            resp->trajectory_plan = new_trajectory;
            resp->trajectory_plan.initial_longitudinal_velocity = new_final_speeds.front();

            // Update stored trajectories
            last_trajectory_ = new_trajectory;
            last_final_speeds_ = new_final_speeds;

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "USING NEW TRAJECTORY for case: " << (int)new_case);
        }
        // Fall back to last trajectory if new is invalid but last is valid
        else if (last_trajectory_.trajectory_points.size() >= 2 &&
                rclcpp::Time(last_trajectory_.trajectory_points.back().target_time) > current_time) {
            resp->trajectory_plan = last_trajectory_;
            resp->trajectory_plan.initial_longitudinal_velocity = last_final_speeds_.front();

            RCLCPP_WARN_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "Failed to generate new trajectory, so using last valid trajectory!");
        }
        // Last resort - return the invalid new trajectory
        else {
            resp->trajectory_plan = new_trajectory;
            RCLCPP_WARN_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "Failed to generate new trajectory or use old valid trajectory, so returning empty/invalid trajectory!");
        }

        // Update stored case information
        last_case_ = new_case;
        is_last_case_successful_ = is_new_case_successful;

        if (is_new_case_successful) {
            last_successful_ending_downtrack_ = GET_MANEUVER_PROPERTY(maneuver_plan.front(), end_dist);
            last_successful_scheduled_entry_time_ = rclcpp::Time(GET_MANEUVER_PROPERTY(maneuver_plan.front(), end_time)).seconds();
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
        "last_successful_ending_downtrack_:" << last_successful_ending_downtrack_ <<
        ", last_successful_scheduled_entry_time_: " << std::to_string(last_successful_scheduled_entry_time_));
        }

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
    "Debug: new case:" << (int) new_case << ", is_new_case_successful: " << is_new_case_successful);

        // Yield for potential obstacles in the road
        if (config_.enable_object_avoidance && resp->trajectory_plan.trajectory_points.size() >= 2)
        {
            basic_autonomy::waypoint_generation::modify_trajectory_to_yield_to_obstacles(
                nh_, req, resp, yield_client_, config_.tactical_plugin_service_call_timeout);
        }
        else
        {
            RCLCPP_DEBUG(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"), "Ignored Object Avoidance");
        }

        resp->maneuver_status.push_back(carma_planning_msgs::srv::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

        std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now();
        auto duration = end_time - start_time;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"),
    "ExecutionTime: " << std::chrono::duration<double>(duration).count());
    }

    void LightControlledIntersectionTacticalPlugin::applyTrajectorySmoothingAlgorithm(const carma_wm::WorldModelConstPtr& wm, std::vector<PointSpeedPair>& points_and_target_speeds, double start_dist, double remaining_dist,
    double starting_speed, double departure_speed, TrajectoryParams tsp)
    {
        if (points_and_target_speeds.empty())
        {
            throw std::invalid_argument("Point and target speed list is empty! Unable to apply case one speed profile...");
        }

        // Checking route geometry start against start_dist and adjust profile
        double planning_downtrack_start = wm->routeTrackPos(points_and_target_speeds[0].point).downtrack; // this can include buffered points earlier than maneuver start_dist

        //Check calculated total dist against maneuver limits
        double total_distance_needed = remaining_dist;
        double dist1 = tsp.x1_ - start_dist;
        double dist2 = tsp.x2_ - start_dist;
        double dist3 = tsp.x3_ - start_dist;

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"), "total_distance_needed: " << total_distance_needed << "\n" <<
    "dist1: " << dist1 << "\n" <<
    "dist2: " << dist2 << "\n" <<
    "dist3: " << dist3);
        double algo_min_speed = std::min({tsp.v1_,tsp.v2_,tsp.v3_});
        double algo_max_speed = std::max({tsp.v1_,tsp.v2_,tsp.v3_});

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"), "found algo_minimum_speed: " << algo_min_speed << "\n" <<
    "algo_max_speed: " << algo_max_speed);

        double total_dist_planned = 0; //Starting dist for maneuver treated as 0.0

        if (planning_downtrack_start < start_dist)
        {
            //Account for the buffer distance that is technically not part of this maneuver

            total_dist_planned = planning_downtrack_start - start_dist;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"), "buffered section is present. Adjusted total_dist_planned to: " << total_dist_planned);
        }

        double prev_speed = starting_speed;
        auto prev_point = points_and_target_speeds.front();

        for(auto& p : points_and_target_speeds)
        {
            double delta_d = lanelet::geometry::distance2d(prev_point.point, p.point);
            total_dist_planned += delta_d;

            //Apply the speed from algorithm at dist covered
            //Kinematic: v_f = sqrt(v_o^2 + 2*a*d)
            double speed_i;
            if (total_dist_planned <= epsilon_)
            {
                //Keep target speed same for buffer distance portion
                speed_i = starting_speed;
            }
            else if(total_dist_planned <= dist1 + epsilon_){
                //First segment
                speed_i = sqrt(pow(starting_speed, 2) + 2 * tsp.a1_ * total_dist_planned);
            }
            else if(total_dist_planned > dist1 && total_dist_planned <= dist2 + epsilon_){
                //Second segment
                speed_i = sqrt(std::max(pow(tsp.v1_, 2) + 2 * tsp.a2_ * (total_dist_planned - dist1), 0.0)); //std::max to ensure negative value is not sqrt
            }
            else if (total_dist_planned > dist2 && total_dist_planned <= dist3 + epsilon_)
            {
                //Third segment
                speed_i = sqrt(std::max(pow(tsp.v2_, 2) + 2 * tsp.a3_ * (total_dist_planned - dist2), 0.0)); //std::max to ensure negative value is not sqrt
            }
            else
            {
                //buffer points that will be cut
                speed_i = prev_speed;
            }

            if (isnan(speed_i))
            {
                speed_i = std::max(config_.minimum_speed, algo_min_speed);
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"), "Detected nan number from equations. Set to " << speed_i);
            }

            p.speed = std::max({speed_i, config_.minimum_speed, algo_min_speed});
            p.speed = std::min({p.speed, speed_limit_, algo_max_speed});
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"), "Applied speed: " << p.speed << ", at dist: " << total_dist_planned);

            prev_point = p;
            prev_speed = p.speed;
        }
    }

    void LightControlledIntersectionTacticalPlugin::applyOptimizedTargetSpeedProfile(const carma_planning_msgs::msg::Maneuver& maneuver, const double starting_speed, std::vector<PointSpeedPair>& points_and_target_speeds)
    {
        if(GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data).size() < 9 ||
            GET_MANEUVER_PROPERTY(maneuver, parameters.int_valued_meta_data).size() < 2 ){
            throw std::invalid_argument("There must be 9 float_valued_meta_data and 2 int_valued_meta_data to apply algorithm's parameters.");
        }

        TrajectoryParams tsp;

        tsp.a1_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[0]);
        tsp.v1_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[1]);
        tsp.x1_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[2]);

        tsp.a2_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[3]);
        tsp.v2_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[4]);
        tsp.x2_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[5]);

        tsp.a3_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[6]);
        tsp.v3_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[7]);
        tsp.x3_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[8]);

        double starting_downtrack = GET_MANEUVER_PROPERTY(maneuver, start_dist);
        double ending_downtrack = GET_MANEUVER_PROPERTY(maneuver, end_dist);
        double departure_speed = GET_MANEUVER_PROPERTY(maneuver, end_speed);
        double scheduled_entry_time = rclcpp::Time(GET_MANEUVER_PROPERTY(maneuver, end_time)).seconds();
        double entry_dist = ending_downtrack - starting_downtrack;

        // change speed profile depending on algorithm case starting from maneuver start_dist
        applyTrajectorySmoothingAlgorithm(wm_, points_and_target_speeds, starting_downtrack, entry_dist, starting_speed,
    departure_speed, tsp);
    }

    double LightControlledIntersectionTacticalPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt, const carma_wm::WorldModelConstPtr &wm) const
    {
        lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm->getTrafficRules();
        if (traffic_rules)
        {
            return (*traffic_rules)->speedLimit(llt).speedLimit.value();
        }
        else
        {
            throw std::invalid_argument("Valid traffic rules object could not be built");
        }
    }

    std::vector<PointSpeedPair> LightControlledIntersectionTacticalPlugin::createGeometryProfile(const std::vector<carma_planning_msgs::msg::Maneuver> &maneuvers, double max_starting_downtrack,const carma_wm::WorldModelConstPtr &wm,
    carma_planning_msgs::msg::VehicleState &ending_state_before_buffer,const carma_planning_msgs::msg::VehicleState& state,
    const GeneralTrajConfig &general_config, const DetailedTrajConfig &detailed_config)
    {
        std::vector<PointSpeedPair> points_and_target_speeds;

        bool first = true;
        std::unordered_set<lanelet::Id> visited_lanelets;
        std::vector<carma_planning_msgs::msg::Maneuver> processed_maneuvers;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"), "VehDowntrack: "<<max_starting_downtrack);

        // Only one maneuver is expected in the received maneuver plan
        if(maneuvers.size() == 1)
        {
            auto maneuver = maneuvers.front();

            double starting_downtrack = GET_MANEUVER_PROPERTY(maneuver, start_dist);

            starting_downtrack = std::min(starting_downtrack, max_starting_downtrack);

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"), "Used downtrack: " << starting_downtrack);

            // check if required parameter from strategic planner is present
            if(GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data).empty())
            {
                throw std::invalid_argument("No time_to_schedule_entry is provided in float_valued_meta_data");
            }

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("light_controlled_intersection_tactical_plugin"), "Creating Lane Follow Geometry");
            std::vector<PointSpeedPair> lane_follow_points = basic_autonomy::waypoint_generation::create_lanefollow_geometry(maneuver, starting_downtrack, wm, general_config, detailed_config, visited_lanelets);
            points_and_target_speeds.insert(points_and_target_speeds.end(), lane_follow_points.begin(), lane_follow_points.end());
            processed_maneuvers.push_back(maneuver);
        }
        else
        {
            throw std::invalid_argument("Light Control Intersection Plugin can only create a geometry profile for one maneuver");
        }

        //Add buffer ending to lane follow points at the end of maneuver(s) end dist
        if(!processed_maneuvers.empty() && processed_maneuvers.back().type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING){
            points_and_target_speeds = add_lanefollow_buffer(wm, points_and_target_speeds, processed_maneuvers, ending_state_before_buffer, detailed_config);
        }

        return points_and_target_speeds;
    }

    void LightControlledIntersectionTacticalPlugin::setConfig(const Config& config)
    {
        config_ = config;
    }

    void LightControlledIntersectionTacticalPlugin::set_yield_client(carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> client)
    {
        yield_client_ = client;
    }


} // light_controlled_intersection_tactical_plugin
