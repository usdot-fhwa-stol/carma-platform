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
#include <cav_msgs/Plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <cav_srvs/PlanManeuvers.h>

/**
 * \brief Macro definition to enable easier access to fields shared across the maneuver types
 * \param mvr The maneuver object to invoke the accessors on
 * \param property The name of the field to access on the specific maneuver types. Must be shared by all extant maneuver types
 * \return Expands to an expression (in the form of chained ternary operators) that evalutes to the desired field
 */
#define GET_MANEUVER_PROPERTY(mvr, property)\
        (((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ? (mvr).intersection_transit_left_turn_maneuver.property :\
            ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ? (mvr).intersection_transit_right_turn_maneuver.property :\
                ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ? (mvr).intersection_transit_straight_maneuver.property :\
                    ((mvr).type == cav_msgs::Maneuver::LANE_CHANGE ? (mvr).lane_change_maneuver.property :\
                        (mvr).type == cav_msgs::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :\
                        throw new std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id"))))))

                        
namespace route_following_plugin
{

    class RouteFollowingPlugin
    {
        public:

        /**
         * \brief Default constructor for RouteFollowingPlugin class
         */
        RouteFollowingPlugin();


        /**
         * \brief General entry point to begin the operation of this class
         */
        void run();

        /**
         * \brief Given a LaneletPath object, find index of the lanelet which has target_id as its lanelet ID
         * \param target_id The laenlet ID this function is looking for
         * \param path A list of lanelet with different lanelet IDs
         * \return Index of the target lanelet in the list
         */
        int findLaneletIndexFromPath(int target_id,const lanelet::routing::LaneletPath& path) const;

        /**
         * \brief Compose a lane keeping maneuver message based on input params
         * \param start_dist Start downtrack distance of the current maneuver
         * \param end_dist End downtrack distance of the current maneuver
         * \param start_speed Start speed of the current maneuver
         * \param target_speed Target speed pf the current maneuver, usually it is the lanelet speed limit
         * \param lane_id Lanelet ID of the current maneuver
         * \param start_time Start time of the current maneuver passed as reference
         * \return A lane keeping maneuver message which is ready to be published
         */
        cav_msgs::Maneuver composeLaneFollowingManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id lane_id, ros::Time start_time) const;

        /**
         * \brief Compose a lane change maneuver message based on input params
         * \param start_dist Start downtrack distance of the current maneuver
         * \param end_dist End downtrack distance of the current maneuver
         * \param start_speed Start speed of the current maneuver
         * \param start_lane_id Starting Lanelet ID of the current maneuver
         * \param ending_lane_id Ending Lanelet ID of the current maneuver
         * \param target_speed Target speed of the current maneuver
         * \param start_time Start time of the current maneuver passed as reference
         * \return A lane keeping maneuver message which is ready to be published
         */
        cav_msgs::Maneuver composeLaneChangeManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id starting_lane_id,lanelet::Id ending_lane_id, ros::Time start_time) const;
        
        /**
         * \brief Given a LaneletRelations and ID of the next lanelet in the shortest path
         * \param relations LaneletRelations relative to the previous lanelet
         * \param target_id ID of the next lanelet in the shortest path
         * \return Whether we need a lanechange to reach to the next lanelet in the shortest path.
         */
        bool isLaneChangeNeeded(lanelet::routing::LaneletRelations relations, lanelet::Id target_id) const;
        
        /**
         * \brief Set the start distance of a maneuver based on the progress along the route
         * \param maneuver A maneuver (non-specific to type) to be performed
         * \param start_dist the starting distance that the maneuver need to be updated to
         */
        void setManeuverStartDist(cav_msgs::Maneuver& maneuver, double start_dist) const;

        /**
         * \brief Given an array of maneuvers update the starting time for each
         * \param maneuvers An array of maneuvers (non-specific to type) 
         * \param start_time The starting time for the first maneuver in the sequence, each consequent maneuver is pushed ahead by same amount
         */
        void updateTimeProgress(std::vector<cav_msgs::Maneuver>& maneuvers, ros::Time start_time) const;
        /**
         * \brief Given an maneuver update the starting speed
         * \param maneuver maneuver to update the starting speed for
         * \param start_time The starting speed for the maneuver passed as argument
         */
        void updateStartingSpeed(cav_msgs::Maneuver& maneuver, double start_speed) const;
        /**
         * \brief Service callback for arbitrator maneuver planning
         * \param req Plan maneuver request
         * \param resp Plan maneuver response with a list of maneuver plan
         * \return If service call successed
         */
        bool plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp);

        /**
         * \brief Given a Lanelet, find it's associated Speed Limit
         * \param llt Constant Lanelet object
         * \return value of speed limit in mps
         */
        double findSpeedLimit(const lanelet::ConstLanelet& llt);
        /**
         * \brief Calculate maneuver plan for remaining route. This callback is triggered when a new route has been received and processed by the world model
         * \param route_shortest_path A list of lanelets along the shortest path of the route using which the maneuver plan is calculated.
         */
        std::vector<cav_msgs::Maneuver> route_cb(const lanelet::routing::LaneletPath& route_shortest_path);

        //Internal Variables used in unit tests
        // Current vehicle forward speed
        double current_speed_;

        // Current vehicle pose in map
        geometry_msgs::PoseStampedConstPtr pose_msg_;
        lanelet::BasicPoint2d current_loc_;

        // wm listener pointer and pointer to the actual wm object
        std::shared_ptr<carma_wm::WMListener> wml_;
        carma_wm::WorldModelConstPtr wm_;

        //Queue of maneuver plans
        std::vector<cav_msgs::Maneuver> latest_maneuver_plan_;

        //Tactical plugin being used for planning lane change
        std::string lane_change_plugin_ = "CooperativeLaneChangePlugin";

        private:

        // CARMA ROS node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        std::shared_ptr<ros::CARMANodeHandle> pnh2_; //Global Scope

        // ROS publishers and subscribers
        ros::Publisher plugin_discovery_pub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber twist_sub_;
        ros::Timer discovery_pub_timer_;

        // ROS service servers
        ros::ServiceServer plan_maneuver_srv_;  


        // Minimal duration of maneuver, loaded from config file
        double min_plan_duration_;


        // Plugin discovery message
        cav_msgs::Plugin plugin_discovery_msg_;

        std::string planning_strategic_plugin_ = "RouteFollowingPlugin";
        std::string lanefollow_planning_tactical_plugin_ = "InLaneCruisingPlugin"; 
        //lane change tactical plugin declared as a public member


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

    };
}