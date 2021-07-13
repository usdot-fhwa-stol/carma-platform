#pragma once

/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include <vector>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <boost/geometry.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <functional>
#include <inlanecruising_plugin/smoothing/SplineI.h>
#include "inlanecruising_config.h"
#include <unordered_set>
#include <autoware_msgs/Lane.h>
#include <ros/ros.h>
#include <carma_debug_msgs/TrajectoryCurvatureSpeeds.h>
#include <basic_autonomy/helper_functions.h>

namespace intersection_transit_maneuvering
{


    class IntersectionTransitManeuvering
    {
        public:
        /**
        * \brief Constructor
        * 
       
        */ 
        IntersectionTransitManeuvering();
                       
        /**
         * \brief Service callback for trajectory planning
         * 
         * \param req The service request
         * \param resp The service response
         * 
         * \return True if success. False otherwise
         */ 
        bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp);

        /**
         * \brief General entry point to begin the operation of this class
         */
        void run();


        /**
        * \brief Converts a set of requested LANE_FOLLOWING maneuvers to point speed limit pairs. 
        * 
        * \param maneuvers The list of maneuvers to convert
        * \param max_starting_downtrack The maximum downtrack that is allowed for the first maneuver. This should be set to the vehicle position or earlier.
        *                               If the first maneuver exceeds this then it's downtrack will be shifted to this value.
        * 
        * \param wm Pointer to intialized world model for semantic map access
        * 
        * \return List of centerline points paired with speed limits
        */ 
        std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                  double max_starting_downtrack,
                                                  const carma_wm::WorldModelConstPtr& wm);
        
        /**
        * \brief Method converts a list of lanelet centerline points and current vehicle state into a usable list of trajectory points for trajectory planning
        * 
        * \param points The set of points that define the current lane the vehicle is in and are defined based on the request planning maneuvers. 
        *               These points must be in the same lane as the vehicle and must extend in front of it though it is fine if they also extend behind it. 
        * \param state The current state of the vehicle
        * \param state_time The abosolute time which the provided vehicle state corresponds to
        * 
        * \return A list of trajectory points to send to the carma planning stack
        */ 
        std::vector<cav_msgs::TrajectoryPlanPoint>
        compose_trajectory_from_centerline(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state, const ros::Time& state_time);

        /**
        * \brief Applies the longitudinal acceleration limit to each point's speed
        * 
        * \param downtracks downtrack distances corresponding to each speed
        * \param curve_speeds vehicle velocity in m/s.
        * \param accel_limit vehicle longitudinal acceleration in m/s^2.
        * 
        * \return optimized speeds for each dowtrack points that satisfies longitudinal acceleration
        */ 
        std::vector<double> optimize_speed(const std::vector<double>& downtracks, const std::vector<double>& curve_speeds, double accel_limit);


        private:
            //CARMA ROS node handles
            std::shared_ptr<ros::CARMANodeHandle> nh_,pnh_;

            // ROS service servers
            ros::ServiceServer trajectory_srv_;

            //ROS publishers and subscribers
            ros::Publisher plugin_discovery_pub_;
            ros::Subscriber pose_sub_;
            ros::Subscriber twist_sub_;
            ros::Publisher jerk_pub_;
            ros::Timer discovery_pub_timer_;

            // Current vehicle pose in map
            geometry_msgs::PoseStamped pose_msg_;

            //Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;

            //Calculated jerk for maneuver in m/s3
            double jerk_ =0.0;
            //Total time required to complete the maneuver
            double maneuver_time_;

            //Parameters loaded from config file initialized for unit tests
            //The crawl speed for the maneuver before reaching within acceptable distance from the end
            double min_crawl_speed_ = 1.0;
            //The minimum duration of a trajectory length in seconds
            double minimal_trajectory_duration_ = 6.0;
            //The maximum acceptable jerk 
            double max_jerk_limit_ = 3.0;
            //The minimum acceptable jerk, after which constant speed is assumed
            double min_jerk_limit_ = 0.001;
            //Minimum timestep used for planning trajectory
            double min_timestep_ =0.1;
            //Amount to downsample input lanelet centerline data
            int downsample_ratio_ =8;
        
            //A small static value for comparing doubles
            static constexpr double epsilon_ = 0.001;


            /**
            * \brief Initialize ROS publishers, subscribers, service servers and service clients
            */
            void initialize();

            /**
            * \brief Callback for the pose subscriber, which will store latest pose locally
            * \param msg Latest pose message
            */
            void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

            /**
            * \brief Callback for the twist subscriber, which will store latest twist locally
            * \param msg Latest twist message
            */
            void twist_cb(const geometry_msgs::TwistStampedConstPtr& msg);


            /**
             * \brief Returns the min, and its idx, from the vector of values, excluding given set of values
            * 
            * \param values vector of values
            * 
            * \param excluded set of excluded values
            * 
            * \return minimum value and its idx
            */ 
            std::pair<double, size_t> min_with_exclusions(const std::vector<double>& values, const std::unordered_set<size_t>& excluded) const;
  
            carma_wm::WorldModelConstPtr wm_;
        











    }
















}