/*
 * Copyright (C) 2019 LEIDOS.
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
#include <carma_wm/WorldModel.h>
#include <vector>
#include <tf/transform_datatypes.h>

namespace waypoint_generator 
{
    /**
     * The primary entrypoint class for the Waypoint Generator node. Handles 
     * subscribing to the route and CARMA World Model and processing it into a
     * form suitable form for converting to Autoware waypoints.
     */
    class WaypointGenerator 
    {
        public:
            /**
             * Initialize the node and it's subscribers
             */
            void initialize();

            /**
             * Process the route after receipt to compute curvature and target speeds
             */
            void process_route(cav_msgs::Route route_msg);

            /**
             * Publish the processed waypoints list
             */
            void publish_waypoints(autoware_msgs::LaneArray waypoints);

            /**
             * Run the node
             */
            void run();

            std::vector<int> compute_constant_curvature_regions(
                std::vector<double> curvatures, 
                double epsilon,
                int linearity_constraint);
            std::vector<double> normalize_curvature_regions(
                std::vector<double> curvatures, 
                std::vector<int> regions);
            double compute_speed_for_curvature(double curvature, 
                double lateral_accel_limit);
            std::vector<double> compute_ideal_speeds(
                std::vector<double> curvatures, 
                double lateral_accel_limit);
            std::vector<double> apply_accel_limits(std::vector<double> speeds, 
                std::vector<int> regions,
                double accel_limit,
                double decel_limit);
            std::vector<geometry_msgs::Quaternion> compute_orientations(
                std::vector<lanelet::ConstLanelet> lanelets);
            autoware_msgs::LaneArray generate_lane_array_message(
                std::vector<double> speeds, 
                std::vector<geometry_msgs::Quaternion> orientations, 
                std::vector<lanelet::ConstLanelet> lanelets);
        protected:
        private:
            autoware_msgs::LaneArray _waypoints;
            carma_wm::WorldModelConstPtr _wm;
            //ros::NodeHandle _pnh, _nh;
    };
};