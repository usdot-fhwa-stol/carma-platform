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
#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
#include <carma_planning_msgs/srv/plan_maneuvers.hpp>
#include <carma_planning_msgs/msg/upcoming_lane_change_status.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <gtest/gtest_prod.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unordered_set>
#include "carma_guidance_plugins/strategic_plugin.hpp"
#include "route_following_plugin_config.hpp"

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
                        ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :\
                            ((mvr).type == carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT ? (mvr).stop_and_wait_maneuver.property :\
                                throw std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id " + std::to_string((mvr).type)))))))))

#define SET_MANEUVER_PROPERTY(mvr, property, value)\
        (((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ? (mvr).intersection_transit_left_turn_maneuver.property = (value) :\
            ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ? (mvr).intersection_transit_right_turn_maneuver.property = (value) :\
                ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ? (mvr).intersection_transit_straight_maneuver.property = (value) :\
                    ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_CHANGE ? (mvr).lane_change_maneuver.property = (value) :\
                        ((mvr).type == carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT ? (mvr).stop_and_wait_maneuver.property = (value) :\
                            ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property = (value) :\
                                throw std::invalid_argument("SET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id " + std::to_string((mvr).type)))))))))
                        
namespace route_following_plugin
{
    class RouteFollowingPlugin : public carma_guidance_plugins::StrategicPlugin
    {
        public:

        /**
         * \brief Default constructor for RouteFollowingPlugin class
         */
        explicit RouteFollowingPlugin(const rclcpp::NodeOptions &);
        
        carma_ros2_utils::CallbackReturn on_configure_plugin();

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
        carma_planning_msgs::msg::UpcomingLaneChangeStatus ComposeLaneChangeStatus(lanelet::ConstLanelet starting_lanelet,lanelet::ConstLanelet ending_lanelet);

        bool get_availability();
        std::string get_version_id();
        
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
        carma_planning_msgs::msg::Maneuver composeLaneFollowingManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, const std::vector<lanelet::Id>& lane_ids) const;

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
        carma_planning_msgs::msg::Maneuver composeLaneChangeManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id starting_lane_id,lanelet::Id ending_lane_id) const;
        
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
        carma_planning_msgs::msg::Maneuver composeStopAndWaitManeuverMessage(double start_dist, double end_dist, double start_speed, lanelet::Id starting_lane_id, lanelet::Id ending_lane_id, double stopping_accel) const;

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
        void setManeuverStartDist(carma_planning_msgs::msg::Maneuver& maneuver, double start_dist) const;

        /**
         * \brief Given an array of maneuvers update the starting time for each
         * \param maneuvers An array of maneuvers (non-specific to type) 
         * \param start_time The starting time for the first maneuver in the sequence, each consequent maneuver is pushed ahead by same amount
         */
        void updateTimeProgress(std::vector<carma_planning_msgs::msg::Maneuver>& maneuvers, rclcpp::Time start_time) const;
        /**
         * \brief Given an maneuver update the starting speed
         * \param maneuver maneuver to update the starting speed for
         * \param start_time The starting speed for the maneuver passed as argument
         */
        void updateStartingSpeed(carma_planning_msgs::msg::Maneuver& maneuver, double start_speed) const;

        /**
         * \brief Service callback for arbitrator maneuver planning
         * \param req Plan maneuver request
         * \param resp Plan maneuver response with a list of maneuver plan
         * \return If service call successed
         */
      
        void plan_maneuvers_callback(
         std::shared_ptr<rmw_request_id_t> srv_header, 
         carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
         carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp);

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
        std::vector<carma_planning_msgs::msg::Maneuver> routeCb(const lanelet::routing::LaneletPath& route_shortest_path);

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
        std::vector<carma_planning_msgs::msg::Maneuver> addStopAndWaitAtRouteEnd (
                const std::vector<carma_planning_msgs::msg::Maneuver>& input_maneuvers, 
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
        bool maneuverWithBufferStartsAfterDowntrack(const carma_planning_msgs::msg::Maneuver& maneuver, double downtrack, double lateral_accel, double min_maneuver_length) const;

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

        //Subscribers
        carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> twist_sub_;
        carma_ros2_utils::SubPtr<carma_planning_msgs::msg::ManeuverPlan> current_maneuver_plan_sub_;
    
        // Publishers
        carma_ros2_utils::PubPtr<carma_planning_msgs::msg::UpcomingLaneChangeStatus> upcoming_lane_change_status_pub_;

        // unordered set of all the lanelet ids in shortest path
        std::unordered_set<lanelet::Id> shortest_path_set_;
       
        static constexpr double MAX_LANE_WIDTH = 3.70; // Maximum lane width of a US highway

        // Node configuration
        Config config_;

        //Upcoming Lane Change downtrack and its lanechange status message map
        std::queue<std::pair<double, carma_planning_msgs::msg::UpcomingLaneChangeStatus>> upcoming_lane_change_status_msg_map_;
        
        // Current vehicle forward speed
        double current_speed_ = 0.0;

        //Small constant to compare doubles against
        double epsilon_ = 0.0001;

        // Current vehicle pose in map
        geometry_msgs::msg::PoseStamped pose_msg_;
        lanelet::BasicPoint2d current_loc_;
        
        // Currently executing maneuver plan from Arbitrator
        carma_planning_msgs::msg::ManeuverPlan::UniquePtr current_maneuver_plan_;

        //Queue of maneuver plans
        std::vector<carma_planning_msgs::msg::Maneuver> latest_maneuver_plan_;

        //Tactical plugin being used for planning lane change
        std::string lane_change_plugin_ = "cooperative_lanechange";
        std::string stop_and_wait_plugin_ = "stop_and_wait_plugin";

        std::string planning_strategic_plugin_ = "route_following_plugin";
        std::string lanefollow_planning_tactical_plugin_ = "inlanecruising_plugin"; 
   
        /**
         * \brief Callback for the front bumper pose transform
         */
        void bumper_pose_cb();

        /**
         * \brief Callback for the twist subscriber, which will store latest twist locally
         * \param msg Latest twist message
         */
        void twist_cb(geometry_msgs::msg::TwistStamped::UniquePtr msg);

        /**
         * \brief Callback for the ManeuverPlan subscriber, will store the current maneuver plan received locally.
         * Used as part of the detection system for differentiating leaving the shortest path via another plugin
         * vs. control drift taking the vehicle's reference point outside of the intended lane.
         * \param msg Latest ManeuverPlan message
         */
        void current_maneuver_plan_cb(carma_planning_msgs::msg::ManeuverPlan::UniquePtr msg);

        /**
         * \brief returns duration as ros::Duration required to complete maneuver given its start dist, end dist, start speed and end speed
         * \param maneuver The maneuver message to calculate duration for
         * \param epsilon The acceptable min start_speed + target_speed in the maneuver message, under which the maneuver is treated as faulty.
         * Throws exception if sum of start and target speed of maneuver is below limit defined by parameter epsilon
         */
        rclcpp::Duration getManeuverDuration(carma_planning_msgs::msg::Maneuver &maneuver, double epsilon) const;

        /**
         * \brief Initialize transform lookup from front bumper to map
         */
        void initializeBumperTransformLookup();

        geometry_msgs::msg::TransformStamped tf_;
        
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