#pragma once

/*
 * Copyright (C) 2021-2022 LEIDOS.
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

#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_planning_msgs/msg/plugin.h>
#include <boost/shared_ptr.hpp>
#include <carma_ros2_utils/containers/containers.hpp>
#include <trajectory_utils/trajectory_utils.hpp>
#include <trajectory_utils/conversions/conversions.hpp>
#include <boost/geometry.hpp>
#include <carma_wm_ros2/Geometry.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <functional>
#include <unordered_set>
#include <autoware_msgs/msg/lane.h>
#include <lanelet2_core/geometry/Point.h>
#include <basic_autonomy_ros2/smoothing/SplineI.hpp>
#include <basic_autonomy_ros2/smoothing/BSpline.hpp>
#include <basic_autonomy_ros2/smoothing/filters.hpp>
#include <carma_debug_ros2_msgs/msg/trajectory_curvature_speeds.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>

/**
 * \brief Macro definition to enable easier access to fields shared across the maneuver types
 * \param mvr The maneuver object to invoke the accessors on
 * \param property The name of the field to access on the specific maneuver types. Must be shared by all extant maneuver types
 * \return Expands to an expression (in the form of chained ternary operators) that evalutes to the desired field
 */
#define GET_MANEUVER_PROPERTY(mvr, property)\
        (((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ? (mvr).intersection_transit_left_turn_maneuver.property :\
            ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ? (mvr).intersection_transit_right_turn_maneuver.property :\
                ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ? (mvr).intersection_transit_straight_maneuver.property :\
                    ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_CHANGE ? (mvr).lane_change_maneuver.property :\
                        (mvr).type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :\
                        (mvr).type == carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT ? (mvr).lane_following_maneuver.property :\
                        throw new std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id"))))))

namespace basic_autonomy
{
    namespace waypoint_generation
    {
        static const std::string BASIC_AUTONOMY_LOGGER = "basic_autonomy";

        struct PointSpeedPair
        {
            lanelet::BasicPoint2d point;
            double speed = 0;
        };

        struct GeneralTrajConfig
        {
            std::string trajectory_type = "inlanecruising";
            int default_downsample_ratio = 36.0; // Amount to downsample input lanelet centerline data.
            int turn_downsample_ratio = 20.0;    // Amount to downsample input lanelet centerline data if the lanelet is marked as a turn
                                                 // Corresponds to saving each nth point
        };

        struct DetailedTrajConfig
        {
            double trajectory_time_length = 6.0;          // Trajectory length in seconds
            double curve_resample_step_size = 1.0;        // Curve re-sampling step size in m
            double minimum_speed = 2.2352;                // Minimum allowable speed in m/s
            double max_accel = 3;                         // Maximum allowable longitudinal acceleration in m/s^2
            double lateral_accel_limit = 2.5;             // Maximum allowable lateral acceleration m/s^2
            int speed_moving_average_window_size = 5;     // Size of the window used in the moving average filter to smooth both the speed profile
            int curvature_moving_average_window_size = 9; // Size of the window used in the moving average filter to smooth the curvature profile
                                                          // computed curvature and output speeds
            double back_distance = 20;                    // Number of meters behind the first maneuver that need to be included in points for curvature calculation
            double buffer_ending_downtrack = 20.0;        //The additional downtrack beyond requested end dist used to fit points along spline
            std::string desired_controller_plugin = "default"; //The desired controller plugin for the generated trajectory
        };
     

       /**
        * \brief Applies the provided speed limits to the provided speeds such that each element is capped at its corresponding speed limit if needed
        * 
        * \param speeds The speeds to limit
        * \param speed_limits The speed limits to apply. Must have the same size as speeds
        * 
        * \return The capped speed limits. Has the same size as speeds
        */
        std::vector<double> apply_speed_limits(const std::vector<double> speeds, const std::vector<double> speed_limits);

       /**
        * \brief Returns a 2D coordinate frame which is located at p1 and oriented so p2 lies on the +X axis
        * 
        * \param p1 The origin point for the frame in the parent frame
        * \param p2 A point in the parent frame that will define the +X axis relative to p1
        * 
        * \return A 2D coordinate frame transform
        */
        Eigen::Isometry2d compute_heading_frame(const lanelet::BasicPoint2d &p1, const lanelet::BasicPoint2d &p2);

       /**
        * \brief Reduces the input points to only those points that fit within the provided time boundary
        * 
        * \param points The input point speed pairs to reduce
        * \param time_span The time span in seconds which the output points will fit within
        * 
        * \return The subset of points that fit within time_span
        */
        std::vector<PointSpeedPair> constrain_to_time_boundary(const std::vector<PointSpeedPair> &points, double time_span);

       /**
        * \brief Returns the min, and its idx, from the vector of values, excluding given set of values
        * \param values vector of values
        * \param excluded set of excluded values
        * 
        * \return minimum value and its idx
        */
        std::pair<double, size_t> min_with_exclusions(const std::vector<double> &values, const std::unordered_set<size_t> &excluded);

       /**
        * \brief Applies the longitudinal acceleration limit to each point's speed
        * 
        * \param downtracks downtrack distances corresponding to each speed
        * \param curv_speeds vehicle velocity in m/s.
        * \param accel_limit vehicle longitudinal acceleration in m/s^2.
        * 
        * \return optimized speeds for each dowtrack points that satisfies longitudinal acceleration
        */
        std::vector<double> optimize_speed(const std::vector<double> &downtracks, const std::vector<double> &curv_speeds, double accel_limit);

       /**
        * \brief Method combines input points, times, orientations, and an absolute start time to form a valid carma platform trajectory
        * 
        * NOTE: All input vectors must be the same size. The output vector will take this size.
        * 
        * \param points The points in the map frame that the trajectory will follow. Units m
        * \param times The times which at the vehicle should arrive at the specified points. First point should have a value of 0. Units s
        * \param yaws The orientation the vehicle should achieve at each point. Units radians
        * \param startTime The absolute start time which will be used to update the input relative times. Units s
        * \param desired_controller_plugin The name of the controller plugin for the generated trajectory.
        * 
        * \return A list of trajectory points built from the provided inputs.
        */
        std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory_from_points_times_orientations(
            const std::vector<lanelet::BasicPoint2d> &points, const std::vector<double> &times,
            const std::vector<double> &yaws, rclcpp::Time startTime, const std::string &desired_controller_plugin);

       /**
        * \brief Attaches back_distance length of points behind the future points
        * 
        * \param points_set all point speed pairs
        * \param future_points future points before which to attach the points
        * \param nearest_pt_index idx of the first future_point in points_set 
        * \param back_distance  the back distance to be added, in meters
        * 
        * \return point speed pairs with back distance length of points in front of future points
        * NOTE- used to add past points to future trajectory for smooth spline calculation
        */
        std::vector<PointSpeedPair> attach_past_points(const std::vector<PointSpeedPair> &points_set, std::vector<PointSpeedPair> future_points,
                                                       const int nearest_pt_index, double back_distance);

       /**
        * \brief Computes a spline based on the provided points
        * \param basic_points The points to use for fitting the spline
        * 
        * \return A spline which has been fit to the provided points
        */
        std::unique_ptr<basic_autonomy::smoothing::SplineI> compute_fit(const std::vector<lanelet::BasicPoint2d> &basic_points);

       /**
        * \brief Given the curvature fit, computes the curvature at the given step along the curve
        * \param step_along_the_curve Value in double from 0.0 (curvature start) to 1.0 (curvature end) representing where to calculate the curvature
        * \param fit_curve curvature fit
        * 
        * \return Curvature (k = 1/r, 1/meter)
        */
        double compute_curvature_at(const basic_autonomy::smoothing::SplineI &fit_curve, double step_along_the_curve);

       /**
        * \brief Creates geometry profile to return a point speed pair struct for LANE FOLLOW and LANE CHANGE maneuver types
        * \param maneuvers The list of maneuvers to convert to geometry points and calculate associated speed
        *  \param max_starting_downtrack The maximum downtrack that is allowed for the first maneuver. This should be set to the vehicle position or earlier.
        *                               If the first maneuver exceeds this then it's downtrack will be shifted to this value. 
        * \param wm Pointer to intialized world model for semantic map access
        * \param ending_state_before_buffer reference to Vehicle state, which is state before applying extra points for curvature calculation that are removed later
        * \param state The vehicle state at the time the function is called
        * \param general_config Basic autonomy struct defined to load general config parameters from tactical plugins
        * \param detailed_config Basic autonomy struct defined to load detailed config parameters from tactical plugins
        * \return A vector of point speed pair struct which contains geometry points as basicpoint::lanelet2d and speed as a double for the maneuver
        */
        std::vector<PointSpeedPair> create_geometry_profile(const std::vector<carma_planning_msgs::msg::Maneuver> &maneuvers, double max_starting_downtrack, const carma_wm::WorldModelConstPtr &wm,
                                                                   carma_planning_msgs::msg::VehicleState &ending_state_before_buffer,
                                                                   const carma_planning_msgs::msg::VehicleState& state,const GeneralTrajConfig &general_config,
                                                                   const DetailedTrajConfig &detailed_config);
       /**
        * \brief Converts a set of requested LANE_FOLLOWING maneuvers to point speed limit pairs. 
        * \param maneuvers The list of maneuvers to convert geometry points and calculate associated speed
        * \param max_starting_downtrack The maximum downtrack that is allowed for the first maneuver. This should be set to the vehicle position or earlier.
        *                               If the first maneuver exceeds this then it's downtrack will be shifted to this value.
        * \param wm Pointer to intialized world model for semantic map access
        * \param general_config Basic autonomy struct defined to load general config parameters from tactical plugins
        * \param detailed_config Basic autonomy struct defined to load detailed config parameters from tactical plugins
        * 
        * \return List of centerline points paired with speed limits
        */
        std::vector<PointSpeedPair> create_lanefollow_geometry(const carma_planning_msgs::msg::Maneuver &maneuver, double max_starting_downtrack,
                                                                   const carma_wm::WorldModelConstPtr &wm, const GeneralTrajConfig &general_config, 
                                                                   const DetailedTrajConfig &detailed_config, std::unordered_set<lanelet::Id>& visited_lanelets);
 
       /**
        * \brief Adds extra centerline points beyond required message length to lane follow maneuver points so that there's always enough points to calculate trajectory
        * (BUFFER POINTS SHOULD BE REMOVED BEFORE RETURNING FINAL TRAJECTORY)
        * \param wm Pointer to intialized world model for semantic map access
        * \param points_and_target_speeds set of lane follow maneuver points to which buffer is added
        * \param maneuvers The list of lane follow maneuvers which were converted to geometry points and associated speed
        * \param ending_state_before_buffer reference to Vehicle state, which is state before applying extra points for curvature calculation that are removed later
        * \param detailed_config Basic autonomy struct defined to load detailed config parameters from tactical plugins
        * 
        * \return List of centerline points paired with speed limits returned with added buffer 
        */ 
        std::vector<PointSpeedPair> add_lanefollow_buffer(const carma_wm::WorldModelConstPtr &wm, std::vector<PointSpeedPair>& points_and_target_speeds, const std::vector<carma_planning_msgs::msg::Maneuver> &maneuvers,
                carma_planning_msgs::msg::VehicleState &ending_state_before_buffer, const DetailedTrajConfig &detailed_config);

       /**
        * \brief Method converts a list of lanelet centerline points and current vehicle state into a usable list of trajectory points for trajectory planning for a Lane following maneuver.
        * \param points The set of points that define the current lane the vehicle is in and are defined based on the request planning maneuvers. 
        *               These points must be in the same lane as the vehicle and must extend in front of it though it is fine if they also extend behind it. 
        * \param state The current state of the vehicle
        * \param state_time The abosolute time which the provided vehicle state corresponds to
        * 
        * \return A list of trajectory points to send to the carma planning stack
        */
        std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>
        compose_lanefollow_trajectory_from_path(const std::vector<PointSpeedPair> &points, const carma_planning_msgs::msg::VehicleState &state,
                                                      const rclcpp::Time &state_time, const carma_wm::WorldModelConstPtr &wm, 
                                                      const carma_planning_msgs::msg::VehicleState &ending_state_before_buffer, carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds& debug_msg,
                                                      const DetailedTrajConfig &detailed_config);

       //Functions specific to lane change
       /**
        * \brief Converts a set of requested LANE_CHANGE maneuvers to point speed limit pairs. 
        * 
        * \param maneuvers The list of maneuvers to convert
        * \param max_starting_downtrack The maximum downtrack that is allowed for the first maneuver. This should be set to the vehicle position or earlier.
        *                               If the first maneuver exceeds this then it's downtrack will be shifted to this value.
        * 
        * \param wm Pointer to intialized world model for semantic map access
        * \param ending_state_before_buffer reference to Vehicle state, which is state before applying extra points for curvature calculation that are removed later
        * \param state The vehicle state at the time the function is called
        * \param general_config Basic autonomy struct defined to load general config parameters from tactical plugins
        * \param detailed_config Basic autonomy struct defined to load detailed config parameters from tactical plugins
        * 
        * \return A vector of point speed pair struct which contains geometry points as basicpoint::lanelet2d and speed as a double for the maneuver
        */
        std::vector<PointSpeedPair> get_lanechange_points_from_maneuver(const carma_planning_msgs::msg::Maneuver &maneuver, double max_starting_downtrack,
                                                                   const carma_wm::WorldModelConstPtr &wm, carma_planning_msgs::msg::VehicleState &ending_state_before_buffer,
                                                                   const carma_planning_msgs::msg::VehicleState &state, const GeneralTrajConfig &general_config,const DetailedTrajConfig &detailed_config);
     
        /**
          * \brief Creates a vector of lane change points using parameters defined. 
          * 
          * \param starting_lane_id lanelet id for where lane change plan should start
          * \param ending_lane_id lanelet id for where lane change plan should end
          * \param starting_downtrack The downtrack distance from which the lane change maneuver starts
          * \param ending_downtrack The downtrack distance at which the lane change maneuver end
          * \param wm Pointer to intialized world model for semantic map access
          * \param downsample_ratio TODO: add description
          * \param buffer_ending_downtrack The additional downtrack beyond requested end dist used to fit points along spline
          * 
          * \return A vector of geometry points as lanelet::basicpoint2d
          */
        std::vector<lanelet::BasicPoint2d> create_lanechange_geometry(lanelet::Id starting_lane_id, lanelet::Id ending_lane_id, double starting_downtrack, double ending_downtrack,
                                                            const carma_wm::WorldModelConstPtr &wm, int downsample_ratio, double buffer_ending_downtrack);
   
     
        /**
          * \brief Resamples a pair of basicpoint2d lines to get lines of same number of points. 
          * 
          * \param line_1 a vector of points to be resampled
          * \param line_2 a vector of points to be resampled
          * 
          * \return A 2d vector with input lines resampled at same rate. The first iteration is the resampled line_1 and the resampled line_2 is the second iteration
          * Assumption here is for lane change to happen between two adjacent lanelets, they must share a lane boundary (linestring)
          */
        std::vector<std::vector<lanelet::BasicPoint2d>> resample_linestring_pair_to_same_size(std::vector<lanelet::BasicPoint2d>& line_1, std::vector<lanelet::BasicPoint2d>& line_2);

        /**
        * \brief Method converts a list of lanelet centerline points and current vehicle state into a usable list of trajectory points for trajectory planning for a Lane following maneuver.
        * 
        * \param points The set of points that define the current lane the vehicle is in and are defined based on the request planning maneuvers. 
        *               These points must be in the same lane as the vehicle and must extend in front of it though it is fine if they also extend behind it. 
        * \param state The current state of the vehicle
        * \param state_time The abosolute time which the provided vehicle state corresponds to
        * \param wm The carma world model object which the vehicle is operating in.
        * \param ending_state_before_buffer The vehicle state before a buffer was added to the points. Used to revert the trajectory to required distance before returning.
        * 
        * \return A list of trajectory points to send to the carma planning stack
        */
        std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> compose_lanechange_trajectory_from_path(
               const std::vector<PointSpeedPair> &points, const carma_planning_msgs::msg::VehicleState &state, const rclcpp::Time &state_time,
               const carma_wm::WorldModelConstPtr &wm, const carma_planning_msgs::msg::VehicleState &ending_state_before_buffer,
               const DetailedTrajConfig &detailed_config);

        /**
        * \brief Creates a Lanelet2 Linestring from a vector or points along the geometry 
        * \param starting_downtrack downtrack along route where maneuver starts
        * \param ending_downtrack downtrack along route where maneuver starts
        * \param wm Pointer to intialized world model for semantic map access
        * 
        * \return Points in a path from starting downtrack to ending downtrack
        */
        std::vector<lanelet::BasicPoint2d> create_route_geom(double starting_downtrack, int starting_lane_id,
                                                             double ending_downtrack, const carma_wm::WorldModelConstPtr &wm);

        /**
        * \brief Given a start and end point, create a vector of points fit through a spline between the points (using a Spline library)
        * \param start_lanelet The lanelet from which lane change starts
        * \param end_lanelet The lanelet in which lane change ends
        * 
        * \return A linestring path from start to end fit through Spline Library
        */
        lanelet::BasicLineString2d create_lanechange_path(const lanelet::ConstLanelet &start_lanelet, const lanelet::ConstLanelet &end_lanelet);

        DetailedTrajConfig compose_detailed_trajectory_config(double trajectory_time_length,
                                                                double curve_resample_step_size,
                                                                double minimum_speed,
                                                                double max_accel,
                                                                double lateral_accel_limit,
                                                                int speed_moving_average_window_size,
                                                                int curvature_moving_average_window_size,
                                                                double back_distance,
                                                                double buffer_ending_downtrack,
                                                                std::string desired_controller_plugin = "default");

        GeneralTrajConfig compose_general_trajectory_config(const std::string& trajectory_type,
                                                            int default_downsample_ratio,
                                                            int turn_downsample_ratio);
        
        /**
        * \brief Given a carma type of trajectory_plan, generate autoware type of trajectory accounting for speed_lag and stopping case
        *        Generated trajectory is meant to be used in autoware.auto's pure_pursuit library using set_trajectory() function
        * \param tp trajectory plan from tactical plugins
        * 
        * \return trajectory plan of autoware_auto_msgs type
        */
        autoware_auto_msgs::msg::Trajectory process_trajectory_plan(const carma_planning_msgs::msg::TrajectoryPlan& tp, double vehicle_response_lag);

        /**
         * \brief Applies a specified response lag in seconds to the trajectory shifting the whole thing by the specified lag time
         * \param speeds Velocity profile to shift. The first point should be the current vehicle speed
         * \param downtrack Distance points for each velocity point. Should have the same size as speeds and start from 0
         * \param response_lag The lag in seconds before which the vehicle will not meaningfully accelerate
         * 
         * \return A Shifted trajectory
         */ 
        std::vector<double> apply_response_lag(const std::vector<double>& speeds, const std::vector<double> downtracks, double response_lag);
    }

} // basic_autonomy_ros2