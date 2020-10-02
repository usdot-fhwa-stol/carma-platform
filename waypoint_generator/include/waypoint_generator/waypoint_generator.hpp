/*
 * Copyright (C) 2020 LEIDOS.
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

#pragma once

#include <ros/ros.h>
#include <cav_msgs/Route.h>
#include <autoware_msgs/LaneArray.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include "waypoint_generator_config.hpp"

namespace waypoint_generator 
{
    /**
     * The primary entrypoint class for the Waypoint Generator node. Handles 
     * subscribing to the route and CARMA World Model and processing it into a
     * form suitable form for converting to Autoware waypoints.
     * 
     * All indices of note (speed, orientations, centerline points) are assumed 
     * to be according to the same index system, such that speed[i] and orientation[i]
     * correspond to the overall (across all lanelets in the input route) i-th
     * lanelet centerline point.
     */
    class WaypointGenerator 
    {
        public:

            using PublishWaypointsCallback = std::function<void(const autoware_msgs::LaneArray&)>;

            WaypointGenerator(carma_wm::WorldModelConstPtr wm, WaypointGeneratorConfig config, PublishWaypointsCallback waypoint_publisher);
            
            /**!
             * \brief Analyze the list of curvatures to detect any regions of 
             * constant (within epsilon) curvature or linearly increasing curvature
             * (outside of epsilon for a constant number of points)
             * 
             * \param curvatures The list of curvatures
             * \param epsilon The max allowable difference in curvature within a
             * region of constant curvature
             * \param linearity_constraint If this many points violate epsilon then
             * it is considered a region of linearly increasing curvature
             * 
             * \return A vector containing the indexes at which each region ends
             *  (inclusive). I.e. a region with end 7 includes point with index 7
             */
            std::vector<int> compute_constant_curvature_regions(
                std::vector<double> curvatures, 
                double epsilon,
                int linearity_constraint) const;

            /**!
             * \brief Get a list of speed limits at each point of the centerline
             * for each lanelet
             * \param lanelets The lanelets to get speed limits from
             * \return A vector where the i-th element is the speed limit for
             * the i-th centerline point amongst all input lanelets
             */
            std::vector<double> get_speed_limits(std::vector<lanelet::ConstLanelet> lanelets) const;

            /**!
             * \brief Normalize the curvature within curvature regions to an
             * average value.
             * 
             * \param curvatures The list of curvatures to be analyzed
             * \param regions The regions of constant curvature
             * \return The normalized list of curvatures
             */
            std::vector<double> normalize_curvature_regions(
                std::vector<double> curvatures, 
                std::vector<int> regions) const;

            /**!
             * \brief Compute the ideal speed for a curvature within a lateral
             * acceleration constraint
             * 
             * Speed is calculated using assumptions of centripetal motion where
             * 
             * a = v^2 / k,
             * 
             * where k is curvature.
             * 
             * \param curvature The target curvature
             * \param lateral_accel_limit The accel limit to respect
             * 
             * \return A speed which obeys the lateral acceleration around the
             * the specified curve
             */
            double compute_speed_for_curvature(double curvature, 
                double lateral_accel_limit) const;

            
            /**!
             * \brief apply the compute_speed_for_curvature method to a list of
             * curvatures
             * \param curvatures The list of curvatures
             * \param lateral_accel_limit The accel limit to obey
             * \return A vector containing the generated speeds
             */
            std::vector<double> compute_ideal_speeds(
                std::vector<double> curvatures, 
                double lateral_accel_limit) const;

            /**!
             * \brief Apply an acceleration limit to the computed set of speeds
             * based on geometry.
             * 
             * For any changes in speed through the planned path it computes an
             * appropriate distance to perform that speed change and then finds
             * and modifies any points necessary to ensure that the speed target
             * is acheieved within accel/decel limits.
             * 
             * \param speeds The vector of speeds
             * \param regions The regions of continous curvature, it is assumed
             * that speed only changes at the bounds of these regions.
             * \param centerline The list of centerline points indexed the same
             * as the speeds.
             * \param accel_limit Acceleration limit in m/s to be respected when
             * increasing speed
             * \param decel_limit Acceleration limit in m/s to be respected when
             * decreasing speed
             * 
             * \return A copy of the speeds list with accel limits applied
             */
            std::vector<double> apply_accel_limits(std::vector<double> speeds, 
                std::vector<int> regions,
                lanelet::BasicLineString2d centerline,
                double accel_limit,
                double decel_limit) const;

            /**!
             * \brief Compute an approximate orientation for the vehicle at each
             * point along the lanelets.
             * 
             * Uses the tangent vectors along the route to estimate vehicle pose
             * in the yaw dimension. Roll and pitch are not considered.
             * 
             * \param lanelets The list of lanelets to compute over
             * 
             * \returns A vector of quaternions representing the vehicle's 
             * orientation at each point of the lanelet's centerline.
             */
            std::vector<geometry_msgs::Quaternion> compute_orientations(
                std::vector<lanelet::ConstLanelet> lanelets) const;

            /**!
             * \brief Compose the autoware_msgs::LaneArray object from it's
             * constituent pieces.
             * 
             * \param speeds The list of speeds at each point along the lanelet 
             * centerlines
             * \param orientations The list of orientations at each point along
             * the lanelet centerlines
             * \param lanelets The lanelets themselves
             * 
             * \return The fully composed autoware_msgs::LaneArray object ready
             * for publication
             */
            autoware_msgs::LaneArray generate_lane_array_message(
                std::vector<double> speeds, 
                std::vector<geometry_msgs::Quaternion> orientations, 
                std::vector<lanelet::ConstLanelet> lanelets) const;

            /**!
             * \brief Apply a basic speed limiter to all speeds in the input list
             * 
             * \param speeds The list of speeds at each point
             * \param max_speed The maximum allowable speed at each point
             * 
             * \return A copy of the input list with any speeds larger than the
             * limit capped at the limit
             */
            std::vector<double> apply_speed_limits(
                const std::vector<double> speeds, 
                const std::vector<double> speed_limits) const;

            /**!
             * \brief Lists the successig lanelets from the shortest path
             * by removing adjacent lanelets.
             * 
             */
            std::vector<lanelet::ConstLanelet> findSuccessingLanelets();

            void new_route_callback();
        private:
            carma_wm::WorldModelConstPtr _wm;
            WaypointGeneratorConfig _config;
            PublishWaypointsCallback _waypoint_publisher;
    };
};