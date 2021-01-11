# pragma once
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
#include <carma_wm/CARMAWorldModel.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


namespace stop_and_wait_plugin
{
    /**
     * \brief Convenience class for pairing 2d points with speeds
     */ 
    struct PointSpeedPair
    {
        lanelet::BasicPoint2d point;
        double speed=0;
    };

    class StopandWait
    {
    public:
        /** 
         * \brief Constructor
         */
        StopandWait(){};

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
         * \brief Converts a set of requested STOP_AND_WAIT maneuvers to point speed limit pairs. 
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
                                                                      double starting_downtrack,
                                                                      const carma_wm::WorldModelConstPtr& wm, const cav_msgs::VehicleState& state);
          /**
         * \brief Method converts a list of lanelet centerline points and current vehicle state into a usable list of trajectory points for trajectory planning
         * 
         * \param points The set of points that define the current lane the vehicle is in and are defined based on the request planning maneuvers. 
         *               These points must be in the same lane as the vehicle and must extend in front of it though it is fine if they also extend behind it. 
         * \param state The current state of the vehicle
         * 
         * \return A list of trajectory points to send to the carma planning stack
         */ 
        std::vector<cav_msgs::TrajectoryPlanPoint> compose_trajectory_from_centerline(
        const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state);

        /**
         * \brief Returns the nearest point to the provided vehicle pose in the provided list
         * 
         * \param points The points to evaluate
         * \param state The current vehicle state
         * 
         * \return index of nearest point in points
         */
        int getNearestPointIndex(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state);
        
        /**
         * \brief Returns the nearest point on the route to the provided vehicle pose in the provided list
         * 
         * \param points Route points to evaluate
         * \param state The current vehicle state
         * 
         * \return index of nearest point in points
         */
        int getNearestRouteIndex(lanelet::BasicLineString2d& points, const cav_msgs::VehicleState& state);
        /**
         * \brief Helper method to split a list of PointSpeedPair into separate point and speed lists 
         */ 
        void splitPointSpeedPairs(const std::vector<PointSpeedPair>& points, std::vector<lanelet::BasicPoint2d>* basic_points,
                            std::vector<double>* speeds);

        /**
         * \brief Method converts speed values associated with given points along path to time. Calculated for constant jerk. 
         * For jerk lesser than 0.001m/s3 speed is assumed to be constant
         * \param downtrack downtrack distance corresponding to the distance travelled on the route
         * \param speeds a vector of speeds associated with given downtrack distances
         * \param time a vector of time values associated with downtrack distances, filled into in the function
         * \param jerk constant jerk along maneuver used for calculating time
         */
        void speed_to_time(const std::vector<double>& downtrack, const std::vector<double>& speeds,std::vector<double>& times, double jerk);

       //current vehicle forward speed
        double current_speed_;

        //wm listener pointer and pointer to the actual wm object
        std::shared_ptr<carma_wm::WMListener> wml_;
        carma_wm::WorldModelConstPtr wm_;

    private:
        //CARMA ROS node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_,pnh_;

        std::shared_ptr<ros::CARMANodeHandle> pnh2_;
        // ROS service servers
        ros::ServiceServer trajectory_srv_;

        //ROS publishers and subscribers
        ros::Publisher plugin_discovery_pub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber twist_sub_;

        // Current vehicle pose in map
        geometry_msgs::PoseStamped pose_msg_;
       
        //Plugin discovery message
        cav_msgs::Plugin plugin_discovery_msg_;

        //Calculated jerk for maneuver in m/s3
        double jerk_;
        //Total time required to complete the maneuver
        double maneuver_time_;

        //Parameters loaded from config file initialized for unit tests
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

        double destination_downtrack_range = 0.0;
        
    };
}