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
 * Developed by the UCLA Mobility Lab, 10/20/2021. 
 *
 * Creator: Xu Han
 * Author: Xu Han, Xin Xia, Jiaqi Ma
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
#include "platoon_config_ihp.h"
#include <platoon_manager_ihp.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <std_msgs/String.h>
#include <carma_wm/TrafficControl.h>

namespace platoon_strategic_ihp
{
    using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
    using MobilityResponseCB = std::function<void(const cav_msgs::MobilityResponse&)>;
    using MobilityRequestCB = std::function<void(const cav_msgs::MobilityRequest&)>;
    using MobilityOperationCB = std::function<void(const cav_msgs::MobilityOperation&)>;
    using PlatooningInfoCB = std::function<void(const cav_msgs::PlatooningInfo&)>;

    class PlatoonStrategicIHPPlugin
    {
        public:
            
            /**
            * \brief Default constructor for PlatoonStrategicIHPPlugin class
            */ 
            PlatoonStrategicIHPPlugin();

            /**
            * \brief Constructor
            * 
            * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
            * \param config The configuration to be used for this object
            * \param plugin_discovery_publisher Callback which will publish the current plugin discovery state
            */ 
            PlatoonStrategicIHPPlugin(carma_wm::WorldModelConstPtr wm, PlatoonPluginConfig config,
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
            * \brief Find lanelet width from local position
            * 
            * \return lanelet index
            */
            double findLaneWidth();

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
            cav_msgs::Maneuver composeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time& current_time);

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
            cav_msgs::Maneuver composeLaneChangeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int starting_lane_id, int ending_lane_id, ros::Time& current_time);

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

            * Note: There is a difference between the "platoon info status" versus the the "platoon strategic plugin states".
            *       
            *       [platooning info status]
            *       The "platooning info status" reflect the overall operating status. The status can 
            *       include vehicles in different "platoon strategic plugin states" as long as the current state is relavent 
            *       to the genral status.
            *       
            *       [platoon strategic plugin states]
            *       The "platoon strategic plugin states" manage the negotiation strategies and 
            *       vehicle communication in a more refined manner that can achieve the "platooning info status'" objective. Hence, 
            *       vehicles in different states will behave differently based on the corresponding roles and predefined rules. 
            *       However, they can belong to the same "platooning info statu" for the same operation purpose. 
            *       
            *       [Example]  
            *       Multiple strategic states, such as "leader aborting" and "candidate follower", can both be mapped to 
            *       "connecting to new leader" platooning info status, as both states are serving the same purpose of connecting 
            *       to the leader. 
            * 
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
            * \brief Callback for the georeference
            * \param msg Latest georeference
            */
            void georeference_cb(const std_msgs::StringConstPtr& msg);

            /**
            * \brief Spin callback function
            */
            bool onSpin();

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

            // ECEF position of the host vehicle
            cav_msgs::LocationECEF pose_ecef_point_;

            // -------------- UCLA: add two states for frontal join ------------------
            /**
            * \brief Run Leader Aborting State
            */
            void run_leader_aborting();

            /**
            * \brief Run Candidate Leader State
            */
            void run_candidate_leader();

            // -------------- UCLA: add two states for cut-in join ------------------
            /**
            * \brief UCLA Run lead with operation State
            */
            void run_lead_with_operation();
            
            /**
            * \brief UCLA Run prepare to join State
            */
            void run_prepare_to_join();

            /**
             * \brief UCLA Update the private variable pose_ecef_point_
             */
            void setHostECEF(cav_msgs::LocationECEF pose_ecef_point);

            /**
             * \brief UCLA Getter: for PlatoonManager class
             */
            PlatoonManager getHostPM();

            /**
             * \brief UCLA Setter: function to set pm_.platoon_state
             */
            void setPMState(PlatoonState desiredState);

            /**
             * \brief UCLA Setter: Update platoon list (Unit Test).
             */
            void updatePlatoonList(std::vector<PlatoonMember> platoon_list);
           
            /**
             * \brief UCLA Setter: Set the host to follower state (Unit Test).
             */
            void setToFollower();

        private:
            
            PublishPluginDiscoveryCB plugin_discovery_publisher_;
            MobilityRequestCB mobility_request_publisher_;
            MobilityResponseCB mobility_response_publisher_;
            MobilityOperationCB mobility_operation_publisher_;
            PlatooningInfoCB platooning_info_publisher_;

            // Platoon Manager Object
            PlatoonManager pm_;


            // wm listener pointer and pointer to the actual wm object
            std::shared_ptr<carma_wm::WMListener> wml_;
            carma_wm::WorldModelConstPtr wm_;

            // local copy of configuration file
            PlatoonPluginConfig config_;

            // local copy of pose
            // Current vehicle pose in map
            geometry_msgs::PoseStamped pose_msg_;
            
            //Internal Variables used in unit tests
            // Current vehicle command speed, m/s
            double cmd_speed_ = 0;
            // Current vehicle measured speed, m/s
            double current_speed_ = 0;
            // Current vehicle downtrack distance in route, m
            double current_downtrack_ = 0;
            // Current vehicle crosstrack distance in route, m
            double current_crosstrack_ = 0;
            // start time of leaderwaiting state, s
            long waitingStartTime = 0;
            // start time of candidate follower state, s
            long candidatestateStartTime = 0;

            // Host Mobility ID
            std::string HostMobilityId = "hostid";

            // ROS Publishers
            ros::Publisher platoon_strategic_ihp_plugin_discovery_pub_;
            ros::Publisher mob_op_pub_;
            ros::Publisher mob_req_pub_;

            // ROS Subscribers
            ros::Subscriber pose_sub_;
            ros::Subscriber mob_req_sub_;
            ros::Subscriber mob_resp_sub_;
            ros::Subscriber mob_op_sub_;


            /**
            * \brief Function to determine if a target vehicle is in the front of host vehicle
            *        note: This is only applicable for same lane application, so it will check cross track distance.
            *
            * \param downtrack target vehicle downtrack distance relative to host's route (m)
            *        crosstrack: target vehicle crosstrack (m)
            *
            * \return true or false
            */
            bool isVehicleRightInFront(double downtrack, double crosstrack);
            
            /**
            * \brief Function to find speed limit of a lanelet
            *
            * \param llt inout lanelet
            *
            * \return speed limit value (m/s)
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
            * \return mobility operation msg
            */
            cav_msgs::MobilityOperation composeMobilityOperationLeaderWaiting();

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

            // -------------- UCLA implemented functions -----------------------
            
            // ----- 0. Extract data -------
            
            /**
            * \brief Function to determine if the given downtrack distance (m) is behind the host vehicle.
            * 
            * note: This is only applicable for same lane application, so it will check cross track distance.
            *
            * \param downtrack: The downtrack distance (m) of the target vehicle to compare with the host.
            *        crosstrack: The crosstrack distance (m) of the target vehicle to compart with the host.
            *
            * \return (boolean): if target vehicle is behind the host vehicle.
            */
            bool isVehicleRightBehind(double downtrack, double crosstrack);

            /**
            * \brief The method for platoon leader to determine if the joining vehicle is closeby. (Note: Used when host vehicle is the platoon leader)
            *  
            *
            * \param joining_downtrack: The downtrack distance of the joining vehicle.
            *        joining_crosstrack:  The crosstrack distance of the joining vehicle.
            *        
            *
            * \return (boolean): if the host vehicle is close to the target platoon.
            */
            bool isJoiningVehicleNearPlatoon(double joining_downtrack, double joining_crosstrack);
            
            /**
            * \brief Function to determine if the host vehicle is close to the target platoon (used for cut-in join scenarios). 
            *  
            *
            * \param rearVehicleDtd: The downtrack of the neighbor platoon rear vehicle.
            * \param frontVehicleDtd: The downtrack of the neighbor platoon leader.
            * \param frontVehicleCtd: The crosstrack of the neighbor platoon leader.
            *
            * \return (boolean): if the host vehicle is close to the target platoon.
            */
            bool isVehicleNearTargetPlatoon(double rearVehicleDtd, double frontVehicleDtd, double frontVehicleCtd);

            /**
            * \brief Function to find the starting and ending lanelet ID for lane change in a two-lane scenario (used for cut-in join scenarios).
            * 
            * Note: This is a temporary function for internal test only. The scenario is not genrealized. Can only find adjacebt lanletID based on predefiend direction (left, right).
            *       
            * \TODO: This function should be replaced by the complete arbitrary lane change module. 
            *
            * \param start_downtrack: The downtrack distance (m) of the starting point.
            *        end_downtrack: The downtrack distance (m) of the target (end) point.
            *
            * \return (int): The adjacent laneletID based on the provided dontrack distance range. 
            */
            int find_target_lanelet_id(double start_downtrack, double end_downtrack);
            
            // ----- 1. compose message ---------
            
            /**
            * \brief Function to compose mobility operation message with STATUS params.
            *
            * \return mobility operation msg.
            */
            cav_msgs::MobilityOperation composeMobilityOperationSTATUS();
            
            /**
            * \brief Function to compose mobility operation message with INFO params.
            *
            * \return mobility operation msg.
            */
            cav_msgs::MobilityOperation composeMobilityOperationINFO();
            
            /**
            * \brief Function to compose mobility operation in LeaderAborting state.
            *
            * \return mobility operation msg.
            */
            cav_msgs::MobilityOperation composeMobilityOperationLeaderAborting();
            
            /**
            * \brief Function to compose mobility operation in CandidateLeader.
            *
            * \return mobility operation msg.
            */
            cav_msgs::MobilityOperation composeMobilityOperationCandidateLeader();
            
            /**
            * \brief Function to compose mobility operation in LeadWithOperation (cut-in join)
            *
            * \param type type of mobility operation (info or status)
            *
            * \return msg: mobility operation msg 
            *         Return null ("") if the incoming message is trashed/unrecognized. 
            */
            cav_msgs::MobilityOperation composeMobilityOperationLeadWithOperation(const std::string& type);
            
            /**
            * \brief Function to compose mobility operation in PrepareToJoin (cut-in join)
            *
            * \param type type of mobility operation (info or status)
            *
            * \return msg: mobility operation msg
            *         Return null ("") if the incoming message is trashed/unrecognized. 
            */
            cav_msgs::MobilityOperation composeMobilityOperationPrepareToJoin();

            // --------- 2. Mobility operation callback -----------
            
            /**
            * \brief Function to process mobility operation message with STATUS params, 
            *        read ecef location and update platoon member info.
            *
            * \param msg incoming mobility operation message.
            */
            void mob_op_cb_STATUS(const cav_msgs::MobilityOperation& msg);

            /**
            * \brief Function to process mobility operation INFO params to find platoon length in m.
            *
            * \param strategyParams The parsed strategy params, used to find ecef locaton.
            *   
            * \return The length of the platoon in m.
            */
            double mob_op_find_platoon_length_from_INFO_params(std::string strategyParams);
            
            /**
            * \brief Function to process mobility operation INFO params to find platoon leader's ecef location.
            *
            * \param strategyParams The parsed strategy params, used to find ecef locaton.
            *   
            * \return ecef location of the sender.
            */
            cav_msgs::LocationECEF mob_op_find_ecef_from_INFO_params(std::string strategyParams);
            
            /**
            * \brief Function to process mobility operation for STATUS params.
            *
            * \param strategyParams The parsed strategy params, used to find ecef locaton.
            *   
            * \return ecef location of the sender.
            */
            cav_msgs::LocationECEF mob_op_find_ecef_from_STATUS_params(std::string strategyParams);

            /**
            * \brief Function to process mobility operation in leaderaborting state.
            *
            * \param msg incoming mobility operation message.
            */
            void mob_op_cb_leaderaborting(const cav_msgs::MobilityOperation& msg);
            
            /**
            * \brief Function to process mobility operation in candidateleader state.
            *
            * \param msg incoming mobility operation message.
            */
            void mob_op_cb_candidateleader(const cav_msgs::MobilityOperation& msg);
            
            /**
            * \brief Function to process mobility operation in LeadWithOperation state (cut-in join)
            *
            * \param msg incoming mobility operation
            */
            void mob_op_cb_leadwithoperation(const cav_msgs::MobilityOperation& msg);
            
            /**
            * \brief Function to process mobility operation in PrepareToJoin state (cut-in join)
            *
            * \param msg incoming mobility operation
            */
            void mob_op_cb_preparetojoin(const cav_msgs::MobilityOperation& msg);

            //------- 3. Mobility request callback -----------
            
            /**
            * \brief Function to process mobility request in leaderaborting state.
            *
            * \param msg incoming mobility request.
            *
            * \return ACK, NACK, or No response.
            */
            MobilityRequestResponse mob_req_cb_leaderaborting(const cav_msgs::MobilityRequest& msg);
            
            /**
            * \brief Function to process mobility request in candidateleader state.
            *
            * \param msg incoming mobility request.
            *
            * \return ACK, NACK, or No response.
            */
            MobilityRequestResponse mob_req_cb_candidateleader(const cav_msgs::MobilityRequest& msg);
            
            /**
            * \brief Function to process mobility request in leadwithoperation state (cut-in join)
            *
            * \param msg incoming mobility request
            *
            * \return ACK, NACK, or No response
            */
            MobilityRequestResponse mob_req_cb_leadwithoperation(const cav_msgs::MobilityRequest& msg);
            
            /**
            * \brief Function to process mobility request in preparetojoin state (cut-in join)
            *
            * \param msg incoming mobility request
            *
            * \return ACK, NACK, or No response
            */
            MobilityRequestResponse mob_req_cb_preparetojoin(const cav_msgs::MobilityRequest& msg);

            // ------ 4. Mobility response callback ------
            /**
            * \brief Function to process mobility response in leaderaborting state.
            *
            * \param msg incoming mobility response.
            */
            void mob_resp_cb_leaderaborting(const cav_msgs::MobilityResponse& msg);
            
            /**
            * \brief Function to process mobility response in candidateleader state.
            *
            * \param msg incoming mobility response.
            */
            void mob_resp_cb_candidateleader(const cav_msgs::MobilityResponse& msg);
            
            /**
            * \brief Function to process mobility response in leadwithoperation state (cut-in join)
            *
            * \param msg incoming mobility response
            */
            void mob_resp_cb_leadwithoperation(const cav_msgs::MobilityResponse& msg);
            
            /**
            * \brief Function to process mobility response in preparetojoin state 
            *
            * \param msg incoming mobility response
            */
            void mob_resp_cb_preparetojoin(const cav_msgs::MobilityResponse& msg);


            // Pointer for map projector
            std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

            // flag to check if map is loaded
            bool map_loaded_ = false;

            // ros service servers
            ros::ServiceServer maneuver_srv_;

            // Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;

            // Number of calls to the run_leader_aborting() method
            int numLeaderAbortingCalls_ = 0;

            // Number of calls to run_candidate_follower() method after MobReq message sent
            int candidate_follower_delay_count_ = 0;

            double maxAllowedJoinTimeGap_ = 15.0;
            double maxAllowedJoinGap_ = 90;
            int maxPlatoonSize_ = 10;
            double vehicleLength_ = 5.0;
            unsigned long infoMessageInterval_ = 200; // ms
            unsigned long prevHeartBeatTime_ = 0.0;
            int statusMessageInterval_ = 100; // ms
            unsigned long NEGOTIATION_TIMEOUT = 15000;  // ms
            unsigned long LANE_CHANGE_TIMEOUT = 300000; // ms (5 min)
            int noLeaderUpdatesCounter = 0;
            int LEADER_TIMEOUT_COUNTER_LIMIT = 5;
            double waitingStateTimeout = 25.0; // s
            double desiredJoinGap = 30.0; // m
            double desiredJoinTimeGap = 4.0; // s

            lanelet::BasicPoint2d target_cutin_pose_;

            // Speed below which platooning will not be attempted; non-zero value allows for sensor noise
            const double STOPPED_SPEED = 0.5; // m/s

            // UCLA: add member variable for state prepare to join (default -2, front join -1, mid/rear join other integer)
            int target_join_index_ = -2;
            
            // Is there a sufficient gap open in the platoon for a cut-in join?
            bool safeToLaneChange_ = false;

            // Strategy types
            const std::string PLATOONING_STRATEGY = "Carma/Platooning";
            const std::string OPERATION_INFO_TYPE = "INFO";
            const std::string OPERATION_STATUS_TYPE = "STATUS";
            
            // UCLA: edit INFO params to store platoon front info (front join needs 10 info params)
            /**
             * index = 0, LENGTH, physical length of the platoon, in m.
             * index = 1, SPEED, in m/s.
             * index = 2, SIZE, number of members.
             * index = 3, ECEFX, in cm.
             * index = 4, ECEFY, in cm.
             * index = 5, ECEFZ, in cm.
             */
            const std::string OPERATION_INFO_PARAMS   = "INFO|LENGTH:%.2f,SPEED:%.2f,SIZE:%d,ECEFX:%.2f,ECEFY:%.2f,ECEFZ:%.2f";
            
            /**
             * index = 0, CMDSPEED, in m/s.
             * index = 1, SPEED, in m/s.
             * index = 2, ECEFX, in cm.
             * index = 3, ECEFY, in cm.
             * index = 4, ECEFZ, in cm.
             */
            const std::string OPERATION_STATUS_PARAMS = "STATUS|CMDSPEED:%1%,SPEED:%2%,ECEFX:%3%,ECEFY:%4%,ECEFZ:%5%";
            
            // JOIN Strategy Params
            /**
             * index = 0, SIZE, number of members.
             * index = 1, SPEED, in m/s.
             * index = 2, ECEFX, in cm.
             * index = 3, ECEFY, in cm.
             * index = 4, ECEFZ, in cm.
             * index = 5, JOINIDX, index of the position in platoon the incoming vehicle tries to join
             */
            /** note: The cut-in index is zero-based and points to the gap-leading vehicle's index. 
             *  eg:  for rear join, cut-in index == platoon.size()-1; 
             *       for join from front, index == -1;
             *       for cut-in in middle, index indicate the gap leading vehicle's index.
             */    
            const std::string JOIN_PARAMS = "SIZE:%1%,SPEED:%2%,ECEFX:%3%,ECEFY:%4%,ECEFZ:%5%,JOINIDX:%6%";
    };
}
