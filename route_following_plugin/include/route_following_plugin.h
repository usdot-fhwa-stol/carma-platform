#pragma once

/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include <cav_msgs/UpcomingLaneChangeStatus.h>
#include <gtest/gtest_prod.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unordered_set>

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
                        ((mvr).type == cav_msgs::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :\
                            ((mvr).type == cav_msgs::Maneuver::STOP_AND_WAIT ? (mvr).stop_and_wait_maneuver.property :\
                                throw std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id " + std::to_string((mvr).type)))))))))

#define SET_MANEUVER_PROPERTY(mvr, property, value)\
        (((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ? (mvr).intersection_transit_left_turn_maneuver.property = (value) :\
            ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ? (mvr).intersection_transit_right_turn_maneuver.property = (value) :\
                ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ? (mvr).intersection_transit_straight_maneuver.property = (value) :\
                    ((mvr).type == cav_msgs::Maneuver::LANE_CHANGE ? (mvr).lane_change_maneuver.property = (value) :\
                        ((mvr).type == cav_msgs::Maneuver::STOP_AND_WAIT ? (mvr).stop_and_wait_maneuver.property = (value) :\
                            ((mvr).type == cav_msgs::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property = (value) :\
                                throw std::invalid_argument("SET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id " + std::to_string((mvr).type)))))))))

                        
namespace route_following_plugin
{

    class RouteFollowingPlugin
    {
        public:

        /**
         * \brief Default constructor for RouteFollowingPlugin class
         */
        RouteFollowingPlugin() = default;


        /**
         * \brief General entry point to begin the operation of this class
         */
        void run();

        /**
         * \brief Initialize ROS publishers, subscribers, service servers and service clients
         */
        void initialize();

        // wm listener pointer and pointer to the actual wm object
        std::shared_ptr<carma_wm::WMListener> wml_;
        carma_wm::WorldModelConstPtr wm_;

        /**
         * \brief Compose UpcomingLaneChangeStatus msg from given starting and ending lanelets
         * \param start_lanelet lanelet the lanechange is starting from
         * \param ending_lanelet lanelet the lanechange is starting from
         * \return UpcomingLaneChangeStatus Note: this method will only work correctly if the 
         * two provided lanelets are forming a lane change
         */
        cav_msgs::UpcomingLaneChangeStatus ComposeLaneChangeStatus(lanelet::ConstLanelet starting_lanelet,lanelet::ConstLanelet ending_lanelet);
        
        private:

        /**
         * \brief Compose a lane keeping maneuver message based on input params
         * \param start_dist Start downtrack distance of the current maneuver
         * \param end_dist End downtrack distance of the current maneuver
         * \param start_speed Start speed of the current maneuver
         * \param target_speed Target speed pf the current maneuver, usually it is the lanelet speed limit
         * \param lane_ids List of lanelet IDs that the current maneuver traverses. Message expects these to be contiguous and end to end 
         * \return A lane keeping maneuver message which is ready to be published
         */
        cav_msgs::Maneuver composeLaneFollowingManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, const std::vector<lanelet::Id>& lane_ids) const;

        /**
         * \brief Compose a lane change maneuver message based on input params 
         *        NOTE: The start and stop time are not set. This is because this is recomputed based on requests from the arbitrator
         * \param start_dist Start downtrack distance of the current maneuver
         * \param end_dist End downtrack distance of the current maneuver
         * \param start_speed Start speed of the current maneuver
         * \param start_lane_id Starting Lanelet ID of the current maneuver
         * \param ending_lane_id Ending Lanelet ID of the current maneuver
         * \param target_speed Target speed of the current maneuver
         * \return A lane keeping maneuver message which is ready to be published
         */
        cav_msgs::Maneuver composeLaneChangeManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id starting_lane_id,lanelet::Id ending_lane_id) const;
        
        /**
         * \brief Compose a stop and wait maneuver message based on input params. 
         *        NOTE: The start and stop time are not set. This is because this is recomputed based on requests from the arbitrator
         * \param start_dist Start downtrack distance of the current maneuver
         * \param end_dist End downtrack distance of the current maneuver
         * \param start_speed Start speed of the current maneuver
         * \param start_lane_id Starting Lanelet ID of the current maneuver
         * \param ending_lane_id Ending Lanelet ID of the current maneuver
         * \param stopping_accel Acceleration used for calculating the stopping distance
         * \return A lane keeping maneuver message which is ready to be published
         */
        cav_msgs::Maneuver composeStopAndWaitManeuverMessage(double start_dist, double end_dist, double start_speed, lanelet::Id starting_lane_id, lanelet::Id ending_lane_id, double stopping_accel) const;

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
        bool planManeuverCb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp);

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
        std::vector<cav_msgs::Maneuver> routeCb(const lanelet::routing::LaneletPath& route_shortest_path);

        /**
         * \brief Adds a StopAndWait maneuver to the end of a maneuver set stopping at the provided downtrack value
         *        NOTE: The priority of this method is to plan the stopping maneuver therefore earlier maneuvers will be modified or removed if required to allow the stopping behavior to be executed
         * 
         * \param input_maneuvers The set of maneuvers to modify to support the StopAndWait maneuver.
         * \param route_end_downtrack The target stopping point (normally the end of the route) which the vehicle should stop before. Units meters
         * \param stopping_entry_speed The expected entry speed for stopping. This is used to compute the stopping distance. Units m/s
         * \param stopping_logitudinal_accel The target deceleration (unsigned) for the stopping operation. Units m/s/s
         * \param lateral_accel_limit The lateral acceleration limit allowed for lane changes. Units m/s/s
         * \param min_maneuver_length The absolute minimum allowable maneuver length for any existing maneuvers in meters
         * 
         * NOTE: Only min_maneuver_length can be a zero-valued input. All other parameters must be positive values greater than zero.
         * 
         * \throw std::invalid_argument If existing maneuvers cannot be modified to allow stopping maneuver creation, or if the generated maneuvers do not overlap any lanelets in the map.
         * 
         * \return A list of maneuvers which mirrors the input list but with the modifications required to include a stopping maneuver at the end 
         * 
         * ASSUMPTION: At the moment the stopping entry speed is not updated because the assumption is
         * that any previous maneuvers which were slower need not be accounted for as planning for a higher speed will always be capable of handling that case 
         * and any which were faster would already have their speed reduced by the maneuver which this speed was derived from. 
         */ 
        std::vector<cav_msgs::Maneuver> addStopAndWaitAtRouteEnd (
                const std::vector<cav_msgs::Maneuver>& input_maneuvers, 
                double route_end_downtrack, double stopping_entry_speed, double stopping_logitudinal_accel,
                double lateral_accel_limit, double min_maneuver_length
            ) const;

        /**
         * \brief Identifies if a maneuver starts after the provided downtrack with compensation for a dynamic buffer size based on the maneuver type
         * 
         * \param maneuver The maneuver to compare
         * \param downtrack The downtrack value to evaluate in meters
         * \param lateral_accel The max lateral acceleration allowed for lane changes in m/s/s
         * \param min_maneuver_length The absolute minimum allowable for any maneuver in meters
         * 
         * \return true if the provided maneuver plus the computed dynamic buffer starts after the provided downtrack value
         */ 
        bool maneuverWithBufferStartsAfterDowntrack(const cav_msgs::Maneuver& maneuver, double downtrack, double lateral_accel, double min_maneuver_length) const;

        /**
         * \brief This method returns a new UUID as a string for assignment to a Maneuver message
         * 
         * \return A new UUID as a string
         */ 
        std::string getNewManeuverId() const;

        /**
         * \brief This method re-routes the vehicle back to the shortest path, if the vehicle has left the shortest path, but is still on the route
         * Re-routing is performed by generating a new shortest path via the closest lanelet on the original shortest path
         * 
         * \param current_lanelet curretn lanelet where the vehicle is at
         */ 
        void returnToShortestPath(const lanelet::ConstLanelet &current_lanelet);

        // CARMA ROS node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        // ROS publishers and subscribers
        ros::Publisher plugin_discovery_pub_;
        ros::Publisher upcoming_lane_change_status_pub_;
        ros::Subscriber twist_sub_;
        ros::Subscriber current_maneuver_plan_sub_;
        ros::Timer discovery_pub_timer_;

        // ROS service servers
        ros::ServiceServer plan_maneuver_srv_;  

        // unordered set of all the lanelet ids in shortest path
        std::unordered_set<lanelet::Id> shortest_path_set_;

        // Minimal duration of maneuver, loaded from config file
        double min_plan_duration_ = 16.0;

        double route_end_point_buffer_ = 10.0;

        double stopping_accel_limit_multiplier_ = 0.5;

        double accel_limit_ = 2.0;

        double lateral_accel_limit_ = 2.0;

        double min_maneuver_length_ = 10.0; // Minimum length to allow for a maneuver when updating it for stop and wait
        
        static constexpr double MAX_LANE_WIDTH = 3.70; // Maximum lane width of a US highway

        // Plugin discovery message
        cav_msgs::Plugin plugin_discovery_msg_;

        //Upcoming Lane Change downtrack and its lanechange status message map
        std::queue<std::pair<double, cav_msgs::UpcomingLaneChangeStatus>> upcoming_lane_change_status_msg_map_;
        
        // Current vehicle forward speed
        double current_speed_ = 0.0;

        //Small constant to compare doubles against
        double epsilon_ = 0.0001;

        // Current vehicle pose in map
        geometry_msgs::PoseStamped pose_msg_;
        lanelet::BasicPoint2d current_loc_;
        
        // Currently executing maneuver plan from Arbitrator
        cav_msgs::ManeuverPlanConstPtr current_maneuver_plan_;

        //Queue of maneuver plans
        std::vector<cav_msgs::Maneuver> latest_maneuver_plan_;

        //Tactical plugin being used for planning lane change
        std::string lane_change_plugin_ = "CooperativeLaneChangePlugin";
        std::string stop_and_wait_plugin_ = "StopAndWaitPlugin";

        std::string planning_strategic_plugin_ = "RouteFollowingPlugin";
        std::string lanefollow_planning_tactical_plugin_ = "InLaneCruisingPlugin"; 

        /**
         * \brief Callback for the front bumper pose transform
         */
        void bumper_pose_cb();

        /**
         * \brief Callback for the twist subscriber, which will store latest twist locally
         * \param msg Latest twist message
         */
        void twist_cb(const geometry_msgs::TwistStampedConstPtr& msg);

        /**
         * \brief Callback for the ManeuverPlan subscriber, will store the current maneuver plan received locally.
         * Used as part of the detection system for differentiating leaving the shortest path via another plugin
         * vs. control drift taking the vehicle's reference point outside of the intended lane.
         * \param msg Latest ManeuverPlan message
         */
        void current_maneuver_plan_cb(const cav_msgs::ManeuverPlanConstPtr& msg);

        /**
         * \brief returns duration as ros::Duration required to complete maneuver given its start dist, end dist, start speed and end speed
         * \param maneuver The maneuver message to calculate duration for
         * \param epsilon The acceptable min start_speed + target_speed in the maneuver message, under which the maneuver is treated as faulty.
         * Throws exception if sum of start and target speed of maneuver is below limit defined by parameter epsilon
         */
        ros::Duration getManeuverDuration(cav_msgs::Maneuver &maneuver, double epsilon) const;

        /**
         * \brief Initialize transform lookup from front bumper to map
         */
        void initializeBumperTransformLookup();

        geometry_msgs::TransformStamped tf_;
        
        // front bumper transform
        tf2::Stamped<tf2::Transform> frontbumper_transform_;
        
        // TF listenser
        tf2_ros::Buffer tf2_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

        //Unit Tests
        FRIEND_TEST(RouteFollowingPluginTest, testComposeManeuverMessage);
        FRIEND_TEST(RouteFollowingPluginTest, testIdentifyLaneChange);
        FRIEND_TEST(RouteFollowingPlugin, TestAssociateSpeedLimit);
        FRIEND_TEST(RouteFollowingPlugin, TestAssociateSpeedLimitusingosm);
        FRIEND_TEST(RouteFollowingPlugin, TestHelperfunctions);
        FRIEND_TEST(RouteFollowingPlugin, TestReturnToShortestPath);
        FRIEND_TEST(StopAndWaitTestFixture, CaseOne);
        FRIEND_TEST(StopAndWaitTestFixture, CaseTwo);
        FRIEND_TEST(StopAndWaitTestFixture, CaseThree);
        FRIEND_TEST(StopAndWaitTestFixture, CaseFour);
        FRIEND_TEST(StopAndWaitTestFixture, CaseFive);
        FRIEND_TEST(StopAndWaitTestFixture, CaseSix);
        FRIEND_TEST(StopAndWaitTestFixture, CaseSeven);
        FRIEND_TEST(StopAndWaitTestFixture, CaseEight);
        FRIEND_TEST(StopAndWaitTestFixture, CaseNine);
        FRIEND_TEST(StopAndWaitTestFixture, CaseTen);

    };
}