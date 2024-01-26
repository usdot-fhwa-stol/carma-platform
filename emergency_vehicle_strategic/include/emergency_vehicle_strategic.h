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

/*
 * Developed by the JFL Solutions LLC.
 * Author: Fang Zhou
 */

#pragma once

#include <vector>
#include <ros/ros.h>
#include <math.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/format.hpp>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cav_srvs/PlanManeuvers.h>
#include <geometry_msgs/PoseStamped.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>

#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectList.h>
#include <cav_msgs/RoadwayObstacle.h>
#include <cav_msgs/RoadwayObstacleList.h>
#include <cav_msgs/Route.h>
#include "emergency_vehicle_strategic_config.h"

#include <cav_msgs/PlanType.h>
#include <cav_msgs/BSM.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <std_msgs/String.h>
#include <carma_wm/TrafficControl.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <cav_msgs/ManeuverPlan.h>
#include <automotive_platform_msgs/TurnSignalCommand.h>
#include <std_msgs/Int16.h>
#include <autoware_msgs/LampCmd.h>

namespace emergency_vehicle_strategic
{
    using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
    using StopVehicleCB = std::function<void(const std_msgs::Bool&)>;
    using TurningSignalCB = std::function<void(const autoware_msgs::LampCmd&)>;

    class EmergencyVehicleStrategicPlugin
    {
        public: 
            /**
            * \brief Default constructor for EmergencyVehicleStrategicPlugin class
            */ 
            EmergencyVehicleStrategicPlugin();

            /**
            * \brief Constructor
            * 
            * \param wm Pointer to initalized instance of the carma world model for accessing semantic map data
            * \param config The configuration to be used for this object
            */ 
            EmergencyVehicleStrategicPlugin(carma_wm::WorldModelConstPtr wm, 
                                             EmergencyVehicleStrategicPluginConfig config,
                                             PublishPluginDiscoveryCB plugin_discovery_publisher,
                                             StopVehicleCB stop_vehicle_publisher,
                                             TurningSignalCB turn_signal_publisher);

            /**
            * \brief Callback for the georeference
            * 
            * \param msg Latest georeference
            */
            void georeference_cb(const std_msgs::StringConstPtr& msg);

            /**
            * \brief Function to convert pose from map frame to ecef location
            *
            * \param pose_msg pose message
            *
            * \return mobility operation msg
            */
            cav_msgs::LocationECEF pose_to_ecef(geometry_msgs::PoseStamped pose_msg);

            /**
             * \brief Update the private variable pose_ecef_point_
             */
            void setHostECEF(cav_msgs::LocationECEF pose_ecef_point);

            /**
            * \brief Callback function for current pose
            * 
            * \param msg PoseStamped msg
            */
            void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

            /**
            * \brief Callback for the control command
            * 
            * \param msg Latest twist cmd message
            */
            void cmd_cb(const geometry_msgs::TwistStampedConstPtr& msg);

            /**
            * \brief Callback for the twist subscriber, which will store latest twist locally
            * 
            * \param msg Latest twist message
            */
            void twist_cb(const geometry_msgs::TwistStampedConstPtr& msg);
            
            /**
            * \brief Callback for the the object detection result list, category: external
            * 
            * \param msg Latest ObjectList msg
            */
            void route_cb(const cav_msgs::RouteConstPtr& route_msg);
            
            /**
            * \brief Callback for the the object detection result list, category: external
            * 
            * \param msg Latest ObjectList msg
            */
            void external_objects_cb(const cav_msgs::ExternalObjectListConstPtr& msg);

            /**
             * subscribe to lane change finish flag
            */
            void lane_change_finish_manual_cb(const std_msgs::BoolConstPtr& msg );

            /**
            * \brief Callback for the the object detection result list, category: roadway
            * 
            * \param msg Latest ObjectList msg
            */
            // void roadway_obstacles_cb(const cav_msgs::RoadwayObstacleListConstPtr& msg);
    
            /**
            * \brief Callback function to the maneuver request
            * 
            * \param req Maneuver service request
            * \param resp Maneuver service response
            *
            * \return Mobility response message
            */
            bool plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp);

            /**
            * \brief Spin callback function
            */
            bool onSpin();

            /**
             * Helper function to find lanelet index from path
             */
            int findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path);
            
            /**
            * \brief Find lanelet index from path
            * 
            * \param current_dist: current downtrack distance (m)
            * \param end_dist: ending downtrack distance (m)
            * \param current_speed: current speed (m/s)
            * \param target_speed: target speed (m/s)
            * \param lane_id: lanelet id 
            * \param current_time: current time (s)
            *
            * \return Maneuver message
            */
            cav_msgs::Maneuver composeManeuverMessage(double current_dist, 
                                                                      double end_dist, 
                                                                      double current_speed, 
                                                                      double target_speed, 
                                                                      int lane_id, 
                                                                      ros::Time& current_time);

            /**
            * \brief Find start(current) and target(end) lanelet index from path to generate lane change maneuver message.
            * 
            * \param current_dist: current downtrack distance
            * \param end_dist: ending downtrack distance
            * \param current_speed: current speed
            * \param target_speed: target speed
            * \param starting_lane_id: current lanelet id which serves as the starting lanlet id
            * \param ending_lane_id: target lanelet id which is also the ending lanelet id that is in another lane
            * \param current_time: current time in seconds
            *
            * \return Maneuver message
            */
            cav_msgs::Maneuver composeLaneChangeManeuverMessage(double current_dist, 
                                                                                double end_dist, 
                                                                                double current_speed, 
                                                                                double target_speed, 
                                                                                int starting_lane_id, 
                                                                                int ending_lane_id, 
                                                                                ros::Time& current_time);
            
            /**
            * \brief Update maneuver status based on prior plan
            * 
            * \param maneuver maneuver
            * \param speed speed
            * \param current_progress current progress
            * \param lane_id lanelet ud
            */
            void updateCurrentStatus(cav_msgs::Maneuver maneuver, double& speed, double& current_progress, int& lane_id);


            /**
                check if lane change possible and return corresponding lanelet
            */
            void check_lane_change_option(int lane_change_intention,
                                            lanelet::ConstLanelet starting_lanelet, 
                                                                  int& lane_change_option, 
                                                                  lanelet::ConstLanelet& left_target_lanelet,
                                                                  lanelet::ConstLanelet& right_target_lanelet,
                                                                  int& target_lanelet_id);

            void prepass_decision_cb(const std_msgs::BoolConstPtr& msg);

            void right_turn_signal();
            void left_turn_signal();
            void no_turn_signal();

            void controller_setting_cb(std_msgs::Float32 msg);
            
            // public global variable

            // ECEF position of the host vehicle
            cav_msgs::LocationECEF pose_ecef_point_;

            // ECEF position of the detected emergency vehicle
            cav_msgs::LocationECEF emergency_ecef_location_;

            // All lanelets within the current route
            cav_msgs::Route route_msg_;

            // speed below which will be considered as stop, non-zero value allows for sensor noise
            const double STOPPED_SPEED = 0.5; // m/s

            cav_msgs::ManeuverPlan lane_change_maneuverplan_;

            double right_lane_change_finished_downtrack_=0;
            double left_lane_change_finished_downtrack_=0;

        private: 

            // publish plugin discovery call back
            PublishPluginDiscoveryCB plugin_discovery_publisher_;

            // publish stop vehicle call back
            StopVehicleCB stop_vehicle_publisher_;

            TurningSignalCB turn_signal_publisher_;
            
            // pointer to the actual wm object
            carma_wm::WorldModelConstPtr wm_;

            // local copy of configuration file
            EmergencyVehicleStrategicPluginConfig config_;

            // Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;

            // Pointer for map projector
            std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

            // Current vehicle pose in map
            geometry_msgs::PoseStamped pose_msg_;

            // Current vehicle downtrack distance in route, m
            double current_downtrack_ = 0.0;

            bool lane_change_finished_manual = false;

            // Current vehicle crosstrack distance in route, m
            double current_crosstrack_ = 0.0;

            // Current vehicle command speed, m/s
            double cmd_speed_ = 0.0;

            // Current vehicle measured speed, m/s
            double current_speed_ = 0.0;

            // mark the lane change starting point ecef
            cav_msgs::LocationECEF lane_change_start_ecef_;
            
            // cross-track distance to the beginning point of the lane change
            double ctd_to_lane_change_start_point_ = 0.0;

            // down-track distance to the beginning point of the lane change
            double dtd_to_lane_change_start_point_ = 0.0;

            // boolean indicator of whether a emergency vehicle is detected (initiated as false)
            bool emergency_vehicle_detected_ = false;
            bool front_emergency_vehicle_detected_ = false;
            bool rear_emergency_vehicle_detected_ = false;

            // current pose msg of the detected emergency vehicle
            geometry_msgs::PoseStamped emergency_vehicle_pose_;  

            // the ID of the detected emergency vehicle
            std::string emergency_vehicle_id_ = "default_emergency_vehicle_ID";

            // boolean indicator of whether the host has changed lane to the right 
            bool is_pull_over_initiated = false; 

            // boolean indicator of lane change status
            bool right_lane_change_finished_ = false;

            bool left_lane_change_finished_=false;

            bool right_lane_change_sent_ = false;

            bool left_lane_change_sent_ = false;

            // lane change finish point downtrack distance
            double right_lane_change_finish_dtd_ = 0.0;

            double left_lane_change_finish_dtd_ = 0.0;

            bool pull_in_flag_=false;

            double controler_look_ahead_distance_=20;


            /** lane change option
            * 0:lane change impossible, 
            * 1 left lane change possible, 
            * 2 right lane change possible, 
            * 3 both left and right lane chang ppossible
            * 4 route has not been loaded
            */
            int lane_change_option=0;

            /** turn signal option
            * 0:no signal 
            * 1:left turn signal, 
            * 2:right turn signal, 
            */
            int turn_signal_option=0;        


            /**
            * \brief Function to find speed limit of a lanelet
            *
            * \param llt inout lanelet
            *
            * \return speed limit value (m/s)
            */
            double findSpeedLimit(const lanelet::ConstLanelet& llt);

            /**
            * \brief Function to convert ecef location to a 2d point in map frame
            *
            * \param ecef_point ecef location point
            *
            * \return 2d point in map frame
            */
            lanelet::BasicPoint2d ecef_to_map_point(cav_msgs::LocationECEF ecef_point);
            
            /**
            * \brief Function to determine if host vehicle is in the same-lane with emergency vehicle
            *
            * \param pose_msg current pose msg of the detected emergency vehicle 
            *
            * \return true or false 
            */
            bool isSameLaneWithEmergencyVehicle(geometry_msgs::PoseStamped pose_msg);

            /**
            * \brief Function to check if lanechange is possible
            *
            * \param start_lanelet_id start lanelet id
            * \param target_lanelet_id start lanelet id
            *
            * \return true or false
            */
            bool is_lanechange_possible(lanelet::Id start_lanelet_id, lanelet::Id target_lanelet_id);

    };
}
