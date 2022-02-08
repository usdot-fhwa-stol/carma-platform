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
#include <cav_msgs/PlatooningInfo.h>
#include <cav_msgs/PlanType.h>
#include <cav_msgs/BSM.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include "platoon_config.h"
#include <platoon_manager.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <std_msgs/String.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <gtest/gtest_prod.h>

namespace platoon_strategic
{
    using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
    using MobilityResponseCB = std::function<void(const cav_msgs::MobilityResponse&)>;
    using MobilityRequestCB = std::function<void(const cav_msgs::MobilityRequest&)>;
    using MobilityOperationCB = std::function<void(const cav_msgs::MobilityOperation&)>;
    using PlatooningInfoCB = std::function<void(const cav_msgs::PlatooningInfo&)>;

    class PlatoonStrategicPlugin
    {
        public:
            
            /**
            * \brief Default constructor for PlatoonStrategicPlugin class
            */ 
            PlatoonStrategicPlugin();

            /**
            * \brief Constructor
            * 
            * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
            * \param config The configuration to be used for this object
            * \param plugin_discovery_publisher Callback which will publish the current plugin discovery state
            */ 
            PlatoonStrategicPlugin(carma_wm::WorldModelConstPtr wm, PlatoonPluginConfig config,
                                PublishPluginDiscoveryCB plugin_discovery_publisher, MobilityResponseCB mobility_response_publisher,
                                MobilityRequestCB mobility_request_publisher, MobilityOperationCB mobility_operation_publisher,
                                PlatooningInfoCB platooning_info_publisher);


            /**
            * \brief Callback function for Mobility Operation Message
            * 
            * \param msg Mobility Operation Message
            */
            void mob_op_cb(const cav_msgs::MobilityOperation& msg);

            /**
            * \brief Callback function for Mobility Request Message
            * 
            * \param msg Mobility Request Message
            */
            void mob_req_cb(const cav_msgs::MobilityRequest& msg);

            /**
            * \brief Callback function for Mobility Response Message
            * 
            * \param msg Mobility Response Message
            */
            void mob_resp_cb(const cav_msgs::MobilityResponse& msg);

            /**
            * \brief Function to the process and respond to the mobility request
            * 
            * \param msg Mobility Request Message
            *
            * \return Mobility response message
            */
            MobilityRequestResponse handle_mob_req(const cav_msgs::MobilityRequest& msg);

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
            * \brief Find lanelet index from path
            * 
            * \param path path
            * \param target_id target lanelet id
            *
            * \return lanelet index
            */
            int findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path);

            /**
            * \brief Find lanelet index from path
            * 
            * \param current_dist current downtrack distance
            * \param end_dist ending downtrack distance
            * \param current_speed current speed
            * \param target_speed target speed
            * \param lane_id lanelet id
            * \param current_time current time in seconds
            *
            * \return Maneuver message
            */
            cav_msgs::Maneuver composeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time& current_time);

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
            * \brief Callback function for current pose
            * 
            * \param msg PoseStamped msg
            */
            void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

            /**
            * \brief Compose Platoon information message
            * 
            * \return PlatooningInfo msg
            */
            cav_msgs::PlatooningInfo composePlatoonInfoMsg();

            /**
            * \brief Callback for the twist subscriber, which will store latest twist locally
            * \param msg Latest twist message
            */
            void twist_cb(const geometry_msgs::TwistStampedConstPtr& msg);

            /**
            * \brief Callback for the control command
            * \param msg Latest twist cmd message
            */
            void cmd_cb(const geometry_msgs::TwistStampedConstPtr& msg);

            /**
            * \brief Callback for the BSM
            * \param msg Latest BSM message
            */
            void bsm_cb(const cav_msgs::BSMConstPtr& msg);

            /**
            * \brief Callback for the georeference
            * \param msg Latest georeference
            */
            void georeference_cb(const std_msgs::StringConstPtr& msg);

            /**
            * \brief Spin callback function
            */
            bool onSpin();

            // Platoon Manager Object
            PlatoonManager pm_;
            
            /**
            * \brief Run Leader State
            */
            void run_leader();
            
            /**
            * \brief Run Leader Waiting State
            */
            void run_leader_waiting();

            /**
            * \brief Run Candidate Follower State
            */
            void run_candidate_follower();

            /**
            * \brief Run Follower State
            */
            void run_follower();
            
            /**
            * \brief Checks if the current location is on the rightmost lane.
            * \param current_location 2d point of current location x and y
            */
            void checkForRightMostLane(const lanelet::BasicPoint2d& current_location);

            /**
            * \brief Gets the local lane index of a provided location. The local lane index is 0 for rightmost lane,
            * 1 for second rightmost, etc. The local lane index considers only the current travel direction. 
            * \param current_location 2d point of current location x and y.
            * \return The local lane index of the provided location.
            */
            int getCurrentLaneIndex(const lanelet::BasicPoint2d& current_location);

            /**
            * \brief Gets the quantity of lanes in the local lane group of a provided location. This includes the
            * quantity of all side-by-side lanelets including the current lanelet. It considers only the current travel direction.
            * \param current_location 2d point of current location x and y.
            * \return Quantity of lanes in the local lane group of the provided location.
            */
            int getCurrentLaneGroupSize(const lanelet::BasicPoint2d& current_location);
            
            // ECEF position of the host vehicle
            cav_msgs::LocationECEF pose_ecef_point_;

            // variable to show if the vehicle is on the rightmost lane
            bool in_rightmost_lane_ = true;
            // variable to show if the vehicle is on a single-lane lane
            bool single_lane_road_ = false;

            // Flag to enable/disable platooning plugin
            bool platooning_enabled_ = false;
        
        private:

            
            PublishPluginDiscoveryCB plugin_discovery_publisher_;
            MobilityRequestCB mobility_request_publisher_;
            MobilityResponseCB mobility_response_publisher_;
            MobilityOperationCB mobility_operation_publisher_;
            PlatooningInfoCB platooning_info_publisher_;


            // wm listener pointer and pointer to the actual wm object
            std::shared_ptr<carma_wm::WMListener> wml_;
            carma_wm::WorldModelConstPtr wm_;

            // local copy of pose
            PlatoonPluginConfig config_;

            // local copy of pose
            // Current vehicle pose in map
            geometry_msgs::PoseStamped pose_msg_;
            
            //Internal Variables used in unit tests
            // Current vehicle command speed
            double cmd_speed_ = 0;
            // Current vehicle measured speed
            double current_speed_ = 0;
            // Current vehicle downtrack distance in route
            double current_downtrack_ = 0;
            // Current vehicle crosstrack distance in route
            double current_crosstrack_ = 0;
            // start time of leaderwaiting state
            long waitingStartTime = 0;
            // start time of candidate follower state
            long candidatestateStartTime = 0;
            // potential new platood id 
            std::string potentialNewPlatoonId = "";
            // target platood id 
            std::string targetPlatoonId = "";
            // Host Mobility ID
            std::string HostMobilityId = "hostid";
            // Host BSM ID
            std::string host_bsm_id_ = "";

            // leader_waiting applicant id
            std::string lw_applicantId_ = "";

            // Current local lane index of host vehicle's local lane group (0 is rightmost, 1 is second rightmost, etc.); considers only the current travel direction
            int current_lane_index_ = 0;
            // Current quantity of lanes in the host vehicle's local lane group (side-by-side lanelets including the current lanelet; considers only the current travel direction)
            int current_lane_group_size_ = 0;
            // CandidateFollower required local lane index before it can issue PLATOON_FOLLOWER_JOIN MobilityRequest to target leader
            int cf_target_lane_index_ = 0;
            // Flag to indicate whether CandidateFollower has received JOIN_REQUIREMENTS MobilityOperation message from target leader
            bool has_received_join_requirements_ = false;

            // ROS Publishers
            ros::Publisher platoon_strategic_plugin_discovery_pub_;
            ros::Publisher mob_op_pub_;
            ros::Publisher mob_req_pub_;

            // ROS Subscribers
            ros::Subscriber pose_sub_;
            ros::Subscriber mob_req_sub_;
            ros::Subscriber mob_resp_sub_;
            ros::Subscriber mob_op_sub_;


            /**
            * \brief Function to determin if a vehicle is in the front of hose vehicle
            *
            * \param rearVehicleBsmId rear vehicle bsm id
            * \param downtrack vehicle downtrack
            *
            * \return true or false
            */
            bool isVehicleRightInFront(std::string rearVehicleBsmId, double downtrack);
            
            /**
            * \brief Function to find speed limit of a lanelet
            *
            * \param llt inout lanelet
            *
            * \return speed limit value
            */
            double findSpeedLimit(const lanelet::ConstLanelet& llt);
            

            /**
            * \brief Function to process mobility request in leader state
            *
            * \param msg incoming mobility request
            *
            * \return ACK, NACK, or No response
            */
            MobilityRequestResponse mob_req_cb_leader(const cav_msgs::MobilityRequest& msg);

            /**
            * \brief Function to process mobility request in leader waiting state
            *
            * \param msg incoming mobility request
            *
            * \return ACK, NACK, or No response
            */
            MobilityRequestResponse mob_req_cb_leaderwaiting(const cav_msgs::MobilityRequest& msg);

            /**
            * \brief Function to process mobility request in follower state
            *
            * \param msg incoming mobility request
            *
            * \return ACK, NACK, or No response
            */
            MobilityRequestResponse mob_req_cb_follower(const cav_msgs::MobilityRequest& msg);

            /**
            * \brief Function to process mobility request in candidate follower state
            *
            * \param msg incoming mobility request
            *
            * \return ACK, NACK, or No response
            */
            MobilityRequestResponse mob_req_cb_candidatefollower(const cav_msgs::MobilityRequest& msg);

            /**
            * \brief Function to process mobility request in standby state
            *
            * \param msg incoming mobility request
            *
            * \return ACK, NACK, or No response
            */
            MobilityRequestResponse mob_req_cb_standby(const cav_msgs::MobilityRequest& msg);

            /**
            * \brief Function to process mobility response in leader state
            *
            * \param msg incoming mobility response
            */
            void mob_resp_cb_leader(const cav_msgs::MobilityResponse& msg);

            /**
            * \brief Function to process mobility response in leader waiting state
            *
            * \param msg incoming mobility response
            */
            void mob_resp_cb_leaderwaiting(const cav_msgs::MobilityResponse& msg);
            
            /**
            * \brief Function to process mobility response in follower state
            *
            * \param msg incoming mobility response
            */
            void mob_resp_cb_follower(const cav_msgs::MobilityResponse& msg);

            /**
            * \brief Function to process mobility response in candidate follower state
            *
            * \param msg incoming mobility response
            */
            void mob_resp_cb_candidatefollower(const cav_msgs::MobilityResponse& msg);

            /**
            * \brief Function to process mobility response in standby state
            *
            * \param msg incoming mobility response
            */
            void mob_resp_cb_standby(const cav_msgs::MobilityResponse& msg);
            
            /**
            * \brief Function to process mobility operation in leader state
            *
            * \param msg incoming mobility operation
            */
            void mob_op_cb_leader(const cav_msgs::MobilityOperation& msg);
            
            /**
            * \brief Function to process mobility operation in leader waiting state
            *
            * \param msg incoming mobility operation
            */
            void mob_op_cb_leaderwaiting(const cav_msgs::MobilityOperation& msg);
            
            /**
            * \brief Function to process mobility operation in follower state
            *
            * \param msg incoming mobility operation
            */
            void mob_op_cb_follower(const cav_msgs::MobilityOperation& msg);
            
            /**
            * \brief Function to process mobility operation in candidate follower state
            *
            * \param msg incoming mobility operation
            */
            void mob_op_cb_candidatefollower(const cav_msgs::MobilityOperation& msg);
            
            /**
            * \brief Function to process mobility operation in standby state
            *
            * \param msg incoming mobility operation
            */
            void mob_op_cb_standby(const cav_msgs::MobilityOperation& msg);
            
            /**
            * \brief Function to compose mobility operation in leader state
            *
            * \param type type of mobility operation (info or status)
            *
            * \return mobility operation msg
            */
            cav_msgs::MobilityOperation composeMobilityOperationLeader(const std::string& type);

            /**
            * \brief Function to compose mobility operation in follower state
            *
            * \return mobility operation msg
            */
            cav_msgs::MobilityOperation composeMobilityOperationFollower();

            /**
            * \brief Function to compose mobility operation in leader waiting state
            *
            * \param type type of mobility operation (status or join_requirements)
            * 
            * \return mobility operation msg
            */
            cav_msgs::MobilityOperation composeMobilityOperationLeaderWaiting(const std::string& type);

            /**
            * \brief Function to compose mobility operation in candidate follower state
            *
            * \return mobility operation msg
            */
            cav_msgs::MobilityOperation composeMobilityOperationCandidateFollower();
            
            /**
            * \brief Function to convert pose from map frame to ecef location
            *
            * \param pose_msg pose message
            *
            * \return mobility operation msg
            */
            cav_msgs::LocationECEF pose_to_ecef(geometry_msgs::PoseStamped pose_msg);

            /**
            * \brief Function to convert ecef location to a 2d point in map frame
            *
            * \param ecef_point ecef location point
            *
            * \return 2d point in map frame
            */
            lanelet::BasicPoint2d ecef_to_map_point(cav_msgs::LocationECEF ecef_point);


            
            // flag to check if map is loaded
            bool map_loaded_ = false;
            
            // Pointer for map projector
            std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

            // Flag to indicate that Leader must perform a left lane change into a suitable platooning lane before responding ACK to a 'JOIN_PLATOON_AT_REAR' request
            // Note: Leader must be on a single-lane road or in a non-rightmost-lane (of a given travel direction) before forming an initial platoon
            bool leader_lane_change_required_ = false;

            // Flag to indicate that CandidateFollower must change lanes into another lane (provided by the target Leader) to join target Leader's platoon
            bool cf_lane_change_required_ = false;

            // ros service servers
            ros::ServiceServer maneuver_srv_;

            // Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;

            double maxAllowedJoinTimeGap_ = 15.0;
            double maxAllowedJoinGap_ = 90;
            int maxPlatoonSize_ = 10;
            double vehicleLength_ = 5.0;
            int infoMessageInterval_ = 200; //ms
            long lastHeartBeatTime = 0.0;
            int statusMessageInterval_ = 100; //ms
            int NEGOTIATION_TIMEOUT = 5000;  // ms
            int noLeaderUpdatesCounter = 0;
            int LEADER_TIMEOUT_COUNTER_LIMIT = 5;
            double waitingStateTimeout = 25.0; // s
            double desiredJoinGap = 30.0; // m
            double desiredJoinTimeGap = 4.0; // s


            // Strategy types
            const std::string MOBILITY_STRATEGY = "Carma/Platooning";
            const std::string OPERATION_INFO_TYPE = "INFO";
            const std::string OPERATION_STATUS_TYPE = "STATUS";
            const std::string OPERATION_JOIN_REQUIREMENTS_TYPE = "JOIN_REQUIREMENTS";
            const std::string OPERATION_INFO_PARAMS   = "INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d,DTD:%.2f,ECEFX:%.2f,ECEFY:%.2f,ECEFZ:%.2f";
            const std::string OPERATION_STATUS_PARAMS = "STATUS|CMDSPEED:%1%,DTD:%2%,SPEED:%3%,ECEFX:%4%,ECEFY:%5%,ECEFZ:%6%";
            const std::string OPERATION_JOIN_REQUIREMENTS_PARAMS = "JOIN_REQUIREMENTS|LANE_INDEX:%1%,LANE_GROUP_SIZE:%2%"; // 'Join' requirements sent to platoon applicant when in LeaderWaiting state
            const std::string JOIN_AT_REAR_PARAMS = "SIZE:%1%,SPEED:%2%,DTD:%3%,ECEFX:%4%,ECEFY:%5%,ECEFZ:%6%";
            

            std::string bsmIDtoString(cav_msgs::BSMCoreData bsm_core)
            {
                std::string res = "";
                for (size_t i=0; i<bsm_core.id.size(); i++)
                {
                res+=std::to_string(bsm_core.id[i]);
                }
                return res;
            }
            
            // Unit Tests
            FRIEND_TEST(PlatoonStrategicPlugin, test_platoon_formation_lane_conditions);


    };
}
