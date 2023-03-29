#pragma once
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

#include <unordered_map>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest_prod.h>
#include <carma_planning_msgs/msg/maneuver_plan.hpp>
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <carma_planning_msgs/msg/upcoming_lane_change_status.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <autoware_msgs/msg/lamp_cmd.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <carma_wm/WMListener.hpp>
#include <carma_wm/WorldModel.hpp>
#include <carma_wm/Geometry.hpp>
#include <string>

// TODO Replace this Macro if possible
/**
 * \brief Macro definition to enable easier access to fields shared across the maneuver typees
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
                            throw new std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id"))))))))


#define SET_MANEUVER_PROPERTY(mvr, property, value)\
        (((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ? (mvr).intersection_transit_left_turn_maneuver.property = (value) :\
            ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ? (mvr).intersection_transit_right_turn_maneuver.property = (value) :\
                ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ? (mvr).intersection_transit_straight_maneuver.property = (value) :\
                    ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_CHANGE ? (mvr).lane_change_maneuver.property = (value) :\
                        ((mvr).type == carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT ? (mvr).stop_and_wait_maneuver.property = (value) :\
                            ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property = (value) :\
                                throw std::invalid_argument("ADJUST_MANEUVER_PROPERTY (property) called on maneuver with invalid type id " + std::to_string((mvr).type)))))))))


namespace plan_delegator
{
    /**
     * \brief Config struct
     */
    struct Config
    {
        // ROS params
        std::string planning_topic_prefix = "/plugins/";
        std::string planning_topic_suffix = "/plan_trajectory";
        double trajectory_planning_rate = 10.0;
        double max_trajectory_duration = 6.0;
        double min_crawl_speed = 2.2352; // Min crawl speed in m/s
        double duration_to_signal_before_lane_change = 2.5; // (Seconds) If an upcoming lane change will begin in under this time threshold, a turn signal activation command will be published.
        int tactical_plugin_service_call_timeout = 100; // (Milliseconds) The maximum duration that Plan Delegator will wait after calling a tactical plugin's trajectory planning service; if trajectory 
                                                        // generation takes longer than this, then planning will immediately end for the current trajectory planning iteration.
        
        // Stream operator for this config
        friend std::ostream &operator<<(std::ostream &output, const Config &c)
        {
        output << "PlanDelegator::Config { " << std::endl
            << "planning_topic_prefix: " << c.planning_topic_prefix << std::endl
            << "planning_topic_suffix: " << c.planning_topic_suffix << std::endl
            << "trajectory_planning_rate: " << c.trajectory_planning_rate << std::endl
            << "max_trajectory_duration: " << c.max_trajectory_duration << std::endl
            << "min_crawl_speed: " << c.min_crawl_speed << std::endl
            << "duration_to_signal_before_lane_change: " << c.duration_to_signal_before_lane_change << std::endl
            << "}" << std::endl;
        return output;
        }
    };

    /**
     * \brief Convenience struct for storing information regarding a lane change maneuver.
     */
    struct LaneChangeInformation
    {
        double starting_downtrack;  // The starting downtrack of the lane change
        bool is_right_lane_change;  // Flag to indicate whether lane change is a right lane change; false if it is a left lane change
    };
    
    class PlanDelegator : public carma_ros2_utils::CarmaLifecycleNode
    {
        public:

            // constants definition
            static const constexpr double MILLISECOND_TO_SECOND = 0.001;

            /**
             * \brief PlanDelegator constructor 
             */
            explicit PlanDelegator(const rclcpp::NodeOptions &);

            /**
             * \brief Callback function of maneuver plan subscriber
             */
            void maneuverPlanCallback(carma_planning_msgs::msg::ManeuverPlan::UniquePtr plan);

            /**
             * \brief Callback function of guidance state subscriber
             */
            void guidanceStateCallback(carma_planning_msgs::msg::GuidanceState::UniquePtr plan);

            /**
             * \brief Callback function for vehicle pose subscriber. Updates latest_pose_ and makes calls to
             * publishUpcomingLaneChangeStatus() and publishTurnSignalCommand().
             * \param pose_msg The received pose message.
             */
            void poseCallback(geometry_msgs::msg::PoseStamped::UniquePtr pose_msg);

            /**
             * \brief Get PlanTrajectory service client by plugin name and
             * create new PlanTrajectory service client if specified name does not exist
             * \return a ServiceClient object which corresponse to the target planner
             */
            carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> getPlannerClientByName(const std::string& planner_name);

            /**
             * \brief Example if a maneuver end time has passed current system time
             * \return if input maneuver is expires
             * NOTE: current_time is assumed to be same clock type as this node
             */
            bool isManeuverExpired(const carma_planning_msgs::msg::Maneuver& maneuver, rclcpp::Time current_time) const;

            /**
             * \brief Generate new PlanTrajecory service request based on current planning progress
             * \return a PlanTrajectoryRequest which is ready to be used in the following service call
             */
            std::shared_ptr<carma_planning_msgs::srv::PlanTrajectory::Request> composePlanTrajectoryRequest(const carma_planning_msgs::msg::TrajectoryPlan& latest_trajectory_plan, const uint16_t& current_maneuver_index) const;

            /**
             * \brief Lookup transfrom from front bumper to base link
             */
            void lookupFrontBumperTransform();

            /**
             * \brief Update the starting downtrack, ending downtrack, and maneuver-specific Lanelet ID parameters associated 
             * with a given maneuver. These updates are required since the starting and ending downtrack values of each maneuver 
             * are shifted based on the distance between the base_link frame and the vehicle_front frame.
             * \param maneuver The maneuver to be updated.
             */
            void updateManeuverParameters(carma_planning_msgs::msg::Maneuver& maneuver);

            ////
            // Overrides
            ////
            carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
            carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);

        protected:
            // Node configuration
            Config config_;
  
            // map to store service clients
            std::unordered_map<std::string, carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory>> trajectory_planners_;
            // local storage of incoming messages
            carma_planning_msgs::msg::ManeuverPlan latest_maneuver_plan_;
            geometry_msgs::msg::PoseStamped latest_pose_;
            geometry_msgs::msg::TwistStamped latest_twist_;

            // wm listener pointer and pointer to the actual wm object
            carma_wm::WMListener wml_;
            carma_wm::WorldModelConstPtr wm_;

        private:
            // ROS Publishers
            carma_ros2_utils::PubPtr<carma_planning_msgs::msg::TrajectoryPlan> traj_pub_;
            carma_ros2_utils::PubPtr<carma_planning_msgs::msg::UpcomingLaneChangeStatus> upcoming_lane_change_status_pub_;
            carma_ros2_utils::PubPtr<autoware_msgs::msg::LampCmd> turn_signal_command_pub_;

            // ROS Subscribers
            carma_ros2_utils::SubPtr<carma_planning_msgs::msg::ManeuverPlan> plan_sub_;
            carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> pose_sub_;
            carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> twist_sub_;
            carma_ros2_utils::SubPtr<carma_planning_msgs::msg::GuidanceState> guidance_state_sub_;

            rclcpp::TimerBase::SharedPtr traj_timer_;

            bool guidance_engaged = false;

            double length_to_front_bumper_ = 3.0;

            // TF listenser
            tf2_ros::Buffer tf2_buffer_;
            std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

            // Object to store information regarding the next upcoming lane change in latest_maneuver_plan_; empty if no upcoming lane change exists in latest_maneuver_plan_
            boost::optional<LaneChangeInformation> upcoming_lane_change_information_;

            // Object to store information regarding the current active lane change in latest_maneuver_plan_; empty if first maneuver in latest_maneuver_plan_ is not a lane change
            boost::optional<LaneChangeInformation> current_lane_change_information_;

            // The latest UpcomingLaneChangeStatus that was published to upcoming_lane_change_status_pub_.
            carma_planning_msgs::msg::UpcomingLaneChangeStatus upcoming_lane_change_status_;

            // The latest turn signal command published to turn_signal_command_pub_.
            autoware_msgs::msg::LampCmd latest_turn_signal_command_;

            /**
             * \brief Callback function for triggering trajectory planning
             */
            void onTrajPlanTick();

            /**
             * \brief Example if a maneuver plan contains at least one maneuver
             * \return if input maneuver plan is valid
             */
            bool isManeuverPlanValid(const carma_planning_msgs::msg::ManeuverPlan& maneuver_plan) const noexcept;

            /**
             * \brief Example if a trajectory plan contains at least two trajectory points
             * \return if input trajectory plan is valid
             */
            bool isTrajectoryValid(const carma_planning_msgs::msg::TrajectoryPlan& trajectory_plan) const noexcept;

            /**
             * \brief Example if a trajectory plan is longer than configured time thresheld
             * \return if input trajectory plan is long enough
             */
            bool isTrajectoryLongEnough(const carma_planning_msgs::msg::TrajectoryPlan& plan) const noexcept;

            /**
             * \brief Plan trajectory based on latest maneuver plan via ROS service call to plugins
             * \return a TrajectoryPlan object which contains PlanTrajectory response from plugins
             */
            carma_planning_msgs::msg::TrajectoryPlan planTrajectory();

            /**
             * \brief Function for generating a LaneChangeInformation object from a provided lane change maneuver.
             * \param lane_change_maneuver The lane change maneuver that a LaneChangeInformation object shall be generated from.
             * \return A LaneChangeInformation object containing information on the provided lane change maneuver.
             */
            LaneChangeInformation getLaneChangeInformation(const carma_planning_msgs::msg::Maneuver& lane_change_maneuver);

            /**
             * \brief Function for processing an optional LaneChangeInformation object pertaining to an upcoming lane change. If not empty, 
             * an UpcomingLaneChangeStatus message is created and published based on the contents of the LaneChangeInformation. The published 
             * UpcomingLaneChangeStatus message is stored in upcoming_lane_change_status_.
             * \param upcoming_lane_change_information An optional LaneChangeInformation object. Empty if no upcoming lane change exists.
             */
            void publishUpcomingLaneChangeStatus(const boost::optional<LaneChangeInformation>& upcoming_lane_change_information);

            /**
             * \brief Function for processing an optional LaneChangeInformation object pertaining to the currently-occurring lane change
             * and an UpcomingLaneChangeStatus message. If the optional object pertaining to the currently-occurring lane change is not empty,
             * then a turn signal command is published based on the current lane change direction. Otherwise, a turn signal command in the direction 
             * of the UpcomingLaneChangeStatus message is published if the vehicle is estimated to begin that lane change in under the time
             * threshold defined by config_.duration_to_signal_before_lane_change. The published TurnSignalComand message is stored in 
             * latest_turn_signal_command_.
             * \param current_lane_change_information An optional LaneChangeInformation object pertaining to the current lane 
             * change. Empty if vehicle is not currently changing lanes.
             * \param upcoming_lane_change_status An UpcomingLaneChangeStatus message containing the lane change direction of an upcoming lane change, along 
             * with the downtrack distance to that lane change.
             */
            void publishTurnSignalCommand(const boost::optional<LaneChangeInformation>& current_lane_change_information, const carma_planning_msgs::msg::UpcomingLaneChangeStatus& upcoming_lane_change_status);

            // Unit Test Accessors
            FRIEND_TEST(TestPlanDelegator, UnitTestPlanDelegator);
            FRIEND_TEST(TestPlanDelegator, TestPlanDelegator);
            FRIEND_TEST(TestPlanDelegator, TestLaneChangeInformation);
            FRIEND_TEST(TestPlanDelegator, TestUpcomingLaneChangeAndTurnSignals);
    };
}