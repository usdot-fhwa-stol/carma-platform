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


#include <ros/ros.h>
#include <string>
#include "platoon_strategic_ihp.h"
#include <array>
#include <stdlib.h> 


namespace platoon_strategic_ihp
{
    
    PlatoonStrategicIHPPlugin::PlatoonStrategicIHPPlugin()
    {
    }

    // -------------- constructor --------------// 
    PlatoonStrategicIHPPlugin::PlatoonStrategicIHPPlugin(carma_wm::WorldModelConstPtr wm, PlatoonPluginConfig config,
                                PublishPluginDiscoveryCB plugin_discovery_publisher, MobilityResponseCB mobility_response_publisher,
                                MobilityRequestCB mobility_request_publisher, MobilityOperationCB mobility_operation_publisher,
                                PlatooningInfoCB platooning_info_publisher)
    : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher), 
      mobility_response_publisher_(mobility_response_publisher), mobility_request_publisher_(mobility_request_publisher),
      mobility_operation_publisher_(mobility_operation_publisher), platooning_info_publisher_(platooning_info_publisher)
    {
        pm_ = PlatoonManager();
        
        //add host vehicle to platoon list by default, prevent the function "isVehicleRightInFront" from returning zero.
        std::string hostStaticId = pm_.getHostStaticID();
        double hostcmdSpeed = pm_.getCommandSpeed();
        double hostDtD = pm_.getCurrentDowntrackDistance();
        double hostCtD = pm_.getCurrentCrosstrackDistance();
        double hostCurSpeed = pm_.getCurrentSpeed();
        long cur_t = ros::Time::now().toNSec()/1000000; // time in millisecond

        // construct platoon member for host vehicle
        PlatoonMember hostVehicleMember = PlatoonMember(hostStaticId, hostcmdSpeed, hostCurSpeed, hostDtD, hostCtD, cur_t); 
        pm_.platoon.push_back(hostVehicleMember);

        plugin_discovery_msg_.name = "PlatooningStrategicIHPPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";  
    }


    //-------------------------------- Extract Data --------------------------------------//

    // Find ecef point based on pose message
    cav_msgs::LocationECEF PlatoonStrategicIHPPlugin::pose_to_ecef(geometry_msgs::PoseStamped pose_msg)
    {

        if (!map_projector_) 
        {
            throw std::invalid_argument("No map projector available for ecef conversion");
        }
        
        cav_msgs::LocationECEF location;

        lanelet::BasicPoint3d ecef_point = map_projector_->projectECEF({pose_msg.pose.position.x, pose_msg.pose.position.y, 0.0}, 1);
        location.ecef_x = ecef_point.x() * 100.0;
        location.ecef_y = ecef_point.y() * 100.0;
        location.ecef_z = ecef_point.z() * 100.0;    
        

        ROS_DEBUG_STREAM("location.ecef_x: " << location.ecef_x);
        ROS_DEBUG_STREAM("location.ecef_y: " << location.ecef_y);
        ROS_DEBUG_STREAM("location.ecef_z: " << location.ecef_z);

        return location;
    }

    // Function to assign host pose_ecef_point_
    void PlatoonStrategicIHPPlugin::setHostECEF(cav_msgs::LocationECEF pose_ecef_point)
    {
        pose_ecef_point_ = pose_ecef_point;
    }

    // Function to get pm_ object 
    PlatoonManager PlatoonStrategicIHPPlugin::getHostPM()
    {
        return pm_;
    }

    // Function to set platoon manager state
    void PlatoonStrategicIHPPlugin::setPMState(PlatoonState desiredState)
    {
        pm_.current_platoon_state = desiredState;
    }

    // Set validation info for current plan in PM
    void PlatoonStrategicIHPPlugin::setPMValid(bool isPlanValid)
    {
        pm_.current_plan.valid = isPlanValid;
    }
    
    // Update platoon list (Unit Test function)
    void PlatoonStrategicIHPPlugin::updatePlatoonList(std::vector<PlatoonMember> platoon_list)
    {
        pm_.platoon = platoon_list;
    }

    // Set PM to follower state (Unit Test function)
    void PlatoonStrategicIHPPlugin::setToFollower()
    {
        pm_.isFollower = true;
    }


    // Callback to calculate downtrack based on pose message.
    void PlatoonStrategicIHPPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());

        if (pm_.current_platoon_state != PlatoonState::STANDBY)
        {
            lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
            carma_wm::TrackPos tc = wm_->routeTrackPos(current_loc);

            // update DtD for plugin
            current_downtrack_ = tc.downtrack;
            // update DtD for platoon manager 
            pm_.current_downtrack_distance_ = tc.downtrack;
            ROS_DEBUG_STREAM("current_downtrack_ = " << current_downtrack_);
            
            // update CtD for plugin
            current_crosstrack_ = tc.crosstrack;
            // update CtD for platoon manager 
            pm_. current_crosstrack_distance_ = tc.crosstrack;
            ROS_DEBUG_STREAM("current_crosstrack_ = " << current_crosstrack_);

            // pose_ecef_point_ = pose_to_ecef(pose_msg_);
            cav_msgs::LocationECEF pose_ecef_point = pose_to_ecef(pose_msg_);
            setHostECEF(pose_ecef_point);
        } 
    }
   
    // callback kto update the command speed on x direction, in m/s.
    void PlatoonStrategicIHPPlugin::cmd_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        cmd_speed_ = msg->twist.linear.x;
    }
   
    // twist command, linear speed on x direction, in m/s.
    void PlatoonStrategicIHPPlugin::twist_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
        if (current_speed_ < 0.01)
        {
            current_speed_ = 0.0;
        }
    }
    
    // Return the lanelet id.
    int PlatoonStrategicIHPPlugin::findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path)
    {
        for(size_t i = 0; i < path.size(); ++i)
        {
            if(path[i].id() == target_id)
            {
                return i;
            }
        }
        return -1;
    }

    // Find the speed limit for the current road (also used as the desired speed).
    double PlatoonStrategicIHPPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt)
    {
        double target_speed = 0.0;

        lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
        if (traffic_rules)
        {
            target_speed =(*traffic_rules)->speedLimit(llt).speedLimit.value();
        }
        else
        {
            throw std::invalid_argument("Valid traffic rules object could not be built");
        }

        ROS_DEBUG_STREAM("target speed (limit) " << target_speed);
        
        return target_speed;
    }
    
    // Check if target platoon is in front of the host vehicle, and within the same lane (downtrack is the DTD of the platoon rearveh).
    bool PlatoonStrategicIHPPlugin::isVehicleRightInFront(double downtrack, double crosstrack) 
    {
        double currentDtd = current_downtrack_;
        double currentCtd = current_crosstrack_;
        bool samelane = abs(currentCtd-crosstrack) <= config_.laneWidth;

        if (downtrack > currentDtd && samelane)
        {
            ROS_DEBUG_STREAM("Found a platoon in front. We are able to join");
            return true;
        }
        else 
        {
            ROS_DEBUG_STREAM("Ignoring platoon from our back.");
            ROS_DEBUG_STREAM("The front platoon dtd is " << downtrack << " and we are current at " << currentDtd);
            return false;
        }
    }
    
    // UCLA: check if target platoon at back, and within the same lane. Used for same-lane frontal join.
    bool PlatoonStrategicIHPPlugin::isVehicleRightBehind(double downtrack, double crosstrack)
    {   
        double currentDtd = current_downtrack_;
        double currentCtd = current_crosstrack_;
        bool samelane = abs(currentCtd-crosstrack) <= config_.laneWidth;

        if (downtrack < currentDtd && samelane) 
        {
            ROS_DEBUG_STREAM("Found a platoon at behind. We are able to join");
            return true;
        }
        else 
        {
            ROS_DEBUG_STREAM("Ignoring platoon from front.");
            ROS_DEBUG_STREAM("The front platoon dtd is " << downtrack << " and we are current at " << currentDtd);
            return false;
        }
    } 

    // Return the ecef point projected to local map point.
    lanelet::BasicPoint2d PlatoonStrategicIHPPlugin::ecef_to_map_point(cav_msgs::LocationECEF ecef_point)
    {
        if (!map_projector_) 
        {
            throw std::invalid_argument("No map projector available for ecef conversion");
        }

        lanelet::BasicPoint3d map_point = map_projector_->projectECEF( { (double)ecef_point.ecef_x/100.0, (double)ecef_point.ecef_y/100.0, (double)ecef_point.ecef_z/100.0 } , -1);

        lanelet::BasicPoint2d output {map_point.x(), map_point.y()};
        
        ROS_DEBUG_STREAM("map_point.x(): " << map_point.x());
        ROS_DEBUG_STREAM("map_point.y(): " << map_point.y());
        return output;
    } 
    
    // Build map projector from proj string (georefernce).
    void PlatoonStrategicIHPPlugin::georeference_cb(const std_msgs::StringConstPtr& msg) 
    {
        map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str()); 
    }


    //-------------------------------- Mobility Communication --------------------------------------//

    // ------ 1. compose Mobility Operation messages and platoon info ------ //

    // UCLA: Return a Mobility operation message with STATUS params. 
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationSTATUS()
    {    
        /**
         * Note: STATUS params format:
         *       STATUS | --> "CMDSPEED:%1%,SPEED:%2%,ECEFX:%3%,ECEFY:%4%,ECEFZ:%5%"
         *              |----------0----------1---------2---------3---------4------|
         */  

        // Extract data
        cav_msgs::MobilityOperation msg;
        msg.header.plan_id = pm_.currentPlatoonID;
        msg.header.recipient_id = "";
        std::string hostStaticId = config_.vehicleID;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toNSec() / 1000000;
        msg.strategy = MOBILITY_STRATEGY;

        // form message 
        double cmdSpeed = cmd_speed_;
        boost::format fmter(OPERATION_STATUS_PARAMS);
        fmter% cmdSpeed;                // index = 0, in m/s.
        fmter% current_speed_;          // index = 1, in m/s.
        fmter% pose_ecef_point_.ecef_x; // index = 2
        fmter% pose_ecef_point_.ecef_y; // index = 3
        fmter% pose_ecef_point_.ecef_z; // index = 4

        // compose message
        std::string statusParams = fmter.str();
        msg.strategy_params = statusParams;
        ROS_DEBUG_STREAM("Composed a mobility operation message with params " << msg.strategy_params);
        return msg;
    }

// UCLA: Return a Mobility operation message with INFO params.
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationINFO()
    {
        /** 
         * Note: INFO param format:
         *      "INFO| --> LENGTH:%.2f,SPEED:%.2f,SIZE:%d,ECEFX:%.2f,ECEFY:%.2f,ECEFZ:%.2f"
         *           |-------0-----------1---------2--------3----------4----------5-------|
         */ 
        cav_msgs::MobilityOperation msg;
        msg.header.plan_id = pm_.currentPlatoonID; // msg.header.plan_id is the platoon ID of the request sender (rear join and frontal join). 
        msg.header.recipient_id = "";
        // Note: msg.header need to be updated to remove bsm_id.
        std::string hostStaticId = config_.vehicleID;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toNSec() / 1000000;;
        msg.strategy = MOBILITY_STRATEGY;

        int CurrentPlatoonLength = pm_.getCurrentPlatoonLength();
        double current_speed = current_speed_;
        int PlatoonSize = pm_.getTotalPlatooningSize();
        double PlatoonRearDowntrackDistance = pm_.getPlatoonRearDowntrackDistance();
        // UCLA: leader dtd == platoon front dtd --> use it to compare frontal join position 
        double PlatoonFrontDowntrackDistance = current_downtrack_;

        boost::format fmter(OPERATION_INFO_PARAMS);
        //  Note: need to update "OPERATION_INFO_PARAMS" in header file --> strategic_platoon_ihp.h  
        fmter% CurrentPlatoonLength;            // index = 0, in m/s.
        fmter% current_speed;                   // index = 1, in m/s.
        fmter% PlatoonSize;                     // index = 2 
        fmter% pose_ecef_point_.ecef_x;         // index = 3
        fmter% pose_ecef_point_.ecef_y;         // index = 4
        fmter% pose_ecef_point_.ecef_z;         // index = 5

        std::string infoParams = fmter.str();
        msg.strategy_params = infoParams;
        ROS_DEBUG_STREAM("Composed a mobility operation message with params " << msg.strategy_params);
        return msg;
    }

    // ----------------------------- UCLA: helper functions for cut-in from front -------------------------------//

    // Note: The function "find_target_lanelet_id" was used to test the IHP platooning logic and is only a pre-written scenario. 
    // The IHP platooning should provide necessary data in a maneuver plan for the arbitrary lane change module.  

    // Check if the platoon close to the platoon for cut-in join.
    bool PlatoonStrategicIHPPlugin::isVehicleNearPlatoon(double currentDtd, double currentCtd)
    {
        /**
         * @brief Function to determine if the host vehicle is close to the target platoon
         * 
         * @param 
         *   (double) currentDtd: the downtrack of the host vehicle.
         *   (double) currentCtd: the crosstrack of the host vehicle.
         * @return
         *   (bool): if the host vehicle is close to the target platoon
         */

        // platoon leader and rear positions
        int lastVehicleIndex = pm_.platoon.size()-1;
        double rearVehicleDtd = pm_.platoon[lastVehicleIndex].vehiclePosition;
        double frontVehicleDtd = pm_.platoon[0].vehiclePosition;
        double frontVehicleCtd = pm_.platoon[0].vehiclePosition;

        // lateral error for two lanes 
        double two_lane_cross_error = 2*config_.maxCrosstrackError + 3.5; // todo: 3.5m is lane width, consider read it from map
        // todo: use maxgap (35m) as longitudinal threshold, consider change 
        bool longitudinalCheck = (currentDtd >= rearVehicleDtd - config_.maxAllowedJoinGap) || (currentDtd <= frontVehicleDtd + config_.maxGap);
        bool lateralCheck = (currentCtd >= frontVehicleCtd - two_lane_cross_error) || (currentCtd <= frontVehicleCtd + two_lane_cross_error);
        if (longitudinalCheck && lateralCheck) 
        {
            // host vehicle is close to target platoon logitudinaly (within 10m) and laterally (within 5m)
            ROS_DEBUG_STREAM("Found a platoon nearby. We are able to join.");
            return true;
        }
        else 
        {
            ROS_DEBUG_STREAM("Ignoring platoon.");
            ROS_DEBUG_STREAM("The platoon leader dtd is " << frontVehicleDtd << " and we are current at " << currentDtd);
            ROS_DEBUG_STREAM("The platoon leader ctd is " << frontVehicleCtd << " and we are current at " << currentCtd);
            return false;
        }

    }

    // Compose platoon info msg for all states.
    cav_msgs::PlatooningInfo PlatoonStrategicIHPPlugin::composePlatoonInfoMsg()
    {
        /**
         * Note: There is a difference between the "platoon info status" versus the the "platoon strategic plugin states".
         *       The "platooning info status" reflect the overall operating status. 
         *       The "platoon strategic plugin states" manage the negotiation strategies and vehicle communication in a more refined manner. 
         * A more detailed note can be found in the corresponding function declaration in "platoon_strategic_ihp.h" file.
         */

        cav_msgs::PlatooningInfo status_msg;

        if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            status_msg.state = cav_msgs::PlatooningInfo::SEARCHING;
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADER)
        {
            status_msg.state = pm_.getTotalPlatooningSize() == 1 ? cav_msgs::PlatooningInfo::SEARCHING : cav_msgs::PlatooningInfo::LEADING;
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADERWAITING)
        {
            status_msg.state = cav_msgs::PlatooningInfo::CONNECTING_TO_NEW_FOLLOWER;
        }
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATEFOLLOWER)
        {
            status_msg.state = cav_msgs::PlatooningInfo::CONNECTING_TO_NEW_LEADER;
        }
        else if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
        {
            status_msg.state = cav_msgs::PlatooningInfo::FOLLOWING;
        }
        // UCLA: add leader aborting for frontal join (inherited from candidate follower).
        else if (pm_.current_platoon_state == PlatoonState::LEADERABORTING)
        {
            status_msg.state = cav_msgs::PlatooningInfo::CONNECTING_TO_NEW_LEADER;
        }
        // UCLA: add candidate leader for frontal join (inherited from leader waiting).
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATELEADER)
        {
            status_msg.state = cav_msgs::PlatooningInfo::CONNECTING_TO_NEW_FOLLOWER;
        }
        // UCLA: add "lead with operation" for frontal join (inherited from leader waiting).
        else if (pm_.current_platoon_state == PlatoonState::LEADWITHOPERATION)
        {
            status_msg.state = cav_msgs::PlatooningInfo::LEADING;
        }
        // UCLA: add "prepare to join" for frontal join (inherited from leader waiting).
        else if (pm_.current_platoon_state == PlatoonState::PREPARETOJOIN)
        {
            status_msg.state = cav_msgs::PlatooningInfo::CONNECTING_TO_NEW_LEADER;
        }
        
        // compose msgs for standby
        if (pm_.current_platoon_state != PlatoonState::STANDBY)
        {
            status_msg.platoon_id = pm_.currentPlatoonID;
            status_msg.size = std::max(1, pm_.getTotalPlatooningSize());
            status_msg.size_limit = config_.maxPlatoonSize;

            if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
            {
                ROS_DEBUG_STREAM("isFollower: " << pm_.isFollower);
                ROS_DEBUG_STREAM("pm platoonsize: " << pm_.getTotalPlatooningSize());

                pm_.isFollower = true;

                PlatoonMember platoon_leader = pm_.getDynamicLeader();
                ROS_DEBUG_STREAM("platoon_leader " << platoon_leader.staticId);
                status_msg.leader_id = platoon_leader.staticId;
                status_msg.leader_downtrack_distance = platoon_leader.vehiclePosition;
                ROS_DEBUG_STREAM("platoon_leader position: " << platoon_leader.vehiclePosition);
                status_msg.leader_cmd_speed = platoon_leader.commandSpeed;
                status_msg.host_platoon_position = pm_.getNumberOfVehicleInFront();

                status_msg.desired_gap = std::max(config_.standStillHeadway, config_.timeHeadway * current_speed_);
                status_msg.actual_gap = platoon_leader.vehiclePosition - current_downtrack_;
            }
            else
            {
                status_msg.leader_id = config_.vehicleID;
                status_msg.leader_downtrack_distance = current_downtrack_;
                status_msg.leader_cmd_speed = cmd_speed_;
                status_msg.host_platoon_position = 0;

            }

            // This info is updated at platoon control plugin
            status_msg.host_cmd_speed = cmd_speed_;

        }
        return status_msg;
    }

    // Compose the Mobility Operation message for leader state. Message parameter types: STATUS and INFO.
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationLeader(const std::string& type)
    {
        
        cav_msgs::MobilityOperation msg;

        // info params
        if (type == OPERATION_INFO_TYPE) 
        {
            msg = composeMobilityOperationINFO();
        }

        // status params
        else if (type == OPERATION_STATUS_TYPE) 
        {
            msg = composeMobilityOperationSTATUS();
        }
        // Unknown strategy param.
        else 
        {
            ROS_ERROR("UNKNOW strategy param string!!!");
            msg.strategy_params = "";
        }
        ROS_DEBUG_STREAM("Composed Mobility Operation message in Leader state.");
        return msg;
    }

    // Compose the Mobility Operation message for Follower state. Message parameter types: STATUS.
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationFollower()
    {
        cav_msgs::MobilityOperation msg;
        msg = composeMobilityOperationSTATUS();
        ROS_DEBUG_STREAM("Composed Mobility Operation message in Follower state.");
        return msg;
    }

    // Compose the Mobility Operation message for LeaderWaiting state. Message parameter types: STATUS.
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationLeaderWaiting()
    {
        cav_msgs::MobilityOperation msg;
        msg = composeMobilityOperationSTATUS();
        ROS_DEBUG_STREAM("Composed Mobility Operation message in LeaderWaiting state.");
        return msg;
    }

    // Compose the Mobility Operation message for CandidateFollower state. Message parameter types: STATUS.
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationCandidateFollower()
    {
        cav_msgs::MobilityOperation msg;
        msg = composeMobilityOperationSTATUS();
        ROS_DEBUG_STREAM("Composed Mobility Operation message in CandidateFollower state.");
        return msg;
    }

    // UCLA: add compose msgs for LeaderAborting (inherited from candidate follower). Message parameter types: STATUS.
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationLeaderAborting()
    {   
        /*
            UCLA Implementation note: 
            Sending STATUS info for member updates and platoon trajectory regulation.
        */
        cav_msgs::MobilityOperation msg;
        msg = composeMobilityOperationSTATUS();
        ROS_DEBUG_STREAM("Composed Mobility Operation message in LeaderAborting state.");
        return msg;
    }
    
    // UCLA: add compose msgs for CandidateLeader (inherited from leader waiting). Message parameter types: STATUS.
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationCandidateLeader()
    {   
        /*
            UCLA Implementation note: 
            This is the joiner which will later become the new leader,
            host vehicle publish status msgs and waiting to lead the rear platoon
        */
        cav_msgs::MobilityOperation msg;
        msg = composeMobilityOperationSTATUS();
        ROS_DEBUG_STREAM("Composed Mobility Operation message in CandidateLeader state.");
        return msg;
    }

    // UCLA: compose mobility message for prepare to join (cut-in join state, inherited from follower state's compose mob_op) 
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationPrepareToJoin()
    {
        /*
            UCLA Implementation note: 
            This is the joiner that is preapring for cut-in join.
            host vehicle publish status msgs and waiting to lead the rear platoon.
        */
        cav_msgs::MobilityOperation msg;
        msg = composeMobilityOperationSTATUS();
        ROS_DEBUG_STREAM("Composed Mobility Operation message in PrepareToJoin state.");
        return msg;
    }
    // UCLA: compose mobility message for leading with operation (cut-in join state, inherited from leader state's compose mob_op) 
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationLeadWithOperation(const std::string& type)
    {
        cav_msgs::MobilityOperation msg;
        
        // info params
        if (type == OPERATION_INFO_TYPE) 
        {
            msg = composeMobilityOperationINFO();
        }

        // status params
        else if (type == OPERATION_STATUS_TYPE) 
        {
            msg = composeMobilityOperationSTATUS();
        }
        // Unknown strategy param.
        else 
        {
            ROS_ERROR("UNKNOW strategy param string!!!");
            msg.strategy_params = "";
        }
        ROS_DEBUG_STREAM("Composed Mobility Operation message in Lead With Operation state.");
        return msg;
    }


    // ------ 2. Mobility operation callback ------ //
    
    // read ecef pose from STATUS
    cav_msgs::LocationECEF PlatoonStrategicIHPPlugin::mob_op_find_ecef_from_STATUS_params(std::string strategyParams)
    {
        /*
         * Helper function that extract ecef location from STATUS msg.
         * Note: STATUS params format:
         *       STATUS | --> "CMDSPEED:%1%,SPEED:%2%,ECEFX:%3%,ECEFY:%4%,ECEFZ:%5%"
         *              |----------0----------1---------2---------3---------4------|
         */

        std::vector<std::string> inputsParams;
        boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));

        std::vector<std::string> ecef_x_parsed;
        boost::algorithm::split(ecef_x_parsed, inputsParams[2], boost::is_any_of(":"));
        double ecef_x = std::stod(ecef_x_parsed[1]);
        ROS_DEBUG_STREAM("ecef_x_parsed: " << ecef_x);

        std::vector<std::string> ecef_y_parsed;
        boost::algorithm::split(ecef_y_parsed, inputsParams[3], boost::is_any_of(":"));
        double ecef_y = std::stod(ecef_y_parsed[1]);
        ROS_DEBUG_STREAM("ecef_y_parsed: " << ecef_y);

        std::vector<std::string> ecef_z_parsed;
        boost::algorithm::split(ecef_z_parsed, inputsParams[4], boost::is_any_of(":"));
        double ecef_z = std::stod(ecef_z_parsed[1]);
        ROS_DEBUG_STREAM("ecef_z_parsed: " << ecef_z);
        
        cav_msgs::LocationECEF ecef_loc;
        ecef_loc.ecef_x = ecef_x;
        ecef_loc.ecef_y = ecef_y;
        ecef_loc.ecef_z = ecef_z;

        return ecef_loc;
    }


    // UCLA: Handle STATUS operation messages
    void PlatoonStrategicIHPPlugin::mob_op_cb_STATUS(const cav_msgs::MobilityOperation& msg)
    {
        /**
         * Note: STATUS params format:
         *       STATUS | --> "CMDSPEED:%1%,SPEED:%2%,ECEFX:%3%,ECEFY:%4%,ECEFZ:%5%"
         *              |----------0----------1---------2---------3---------4------|
         */

        std::string strategyParams = msg.strategy_params;
        std::string vehicleID = msg.header.sender_id;
        std::string platoonId = msg.header.plan_id;
        std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size() + 1);

        // read ecef from STATUS
        cav_msgs::LocationECEF ecef_loc = mob_op_find_ecef_from_STATUS_params(strategyParams);

        // read Downtrack 
        lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(ecef_loc);
        double dtd = wm_->routeTrackPos(incoming_pose).downtrack;
        ROS_DEBUG_STREAM("DTD calculated from ecef is: " << dtd);
        // read Crosstrack
        double ctd = wm_->routeTrackPos(incoming_pose).crosstrack;
        ROS_DEBUG_STREAM("CTD calculated from ecef is: " << ctd);

        pm_.memberUpdates(vehicleID, platoonId, statusParams, dtd, ctd);
        ROS_DEBUG_STREAM("Received platoon status message from vehicle: " << msg.header.sender_id);

        return;
    }    

    // UCLA: Sparse ecef location from INFO params
    cav_msgs::LocationECEF PlatoonStrategicIHPPlugin::mob_op_find_ecef_from_INFO_params(std::string strategyParams)
    {
        /** 
         * Note: INFO param format:
         *      "INFO| --> LENGTH:%.2f,SPEED:%.2f,SIZE:%d,ECEFX:%.2f,ECEFY:%.2f,ECEFZ:%.2f"
         *           |-------0-----------1---------2--------3----------4----------5-------|
         */
        // For INFO params, the string format is INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d,DTD:%.2f
        std::vector<std::string> inputsParams;
        boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));

        std::vector<std::string> ecef_x_parsed;
        boost::algorithm::split(ecef_x_parsed, inputsParams[3], boost::is_any_of(":"));
        double ecef_x = std::stod(ecef_x_parsed[1]);
        ROS_DEBUG_STREAM("ecef_x_parsed: " << ecef_x);
        std::cerr << "ecef_x_parsed: " << ecef_x << std::endl;

        std::vector<std::string> ecef_y_parsed;
        boost::algorithm::split(ecef_y_parsed, inputsParams[4], boost::is_any_of(":"));
        double ecef_y = std::stod(ecef_y_parsed[1]);
        ROS_DEBUG_STREAM("ecef_y_parsed: " << ecef_y);
        std::cerr << "ecef_y_parsed: " << ecef_y << std::endl;

        std::vector<std::string> ecef_z_parsed;
        boost::algorithm::split(ecef_z_parsed, inputsParams[5], boost::is_any_of(":"));
        double ecef_z = std::stod(ecef_z_parsed[1]);
        ROS_DEBUG_STREAM("ecef_z_parsed: " << ecef_z);
        std::cerr << "ecef_z_parsed: " << ecef_z << std::endl;
        
        cav_msgs::LocationECEF ecef_loc;
        ecef_loc.ecef_x = ecef_x;
        ecef_loc.ecef_y = ecef_y;
        ecef_loc.ecef_z = ecef_z;

        return ecef_loc;
    }

    // handle message for each states.
    void PlatoonStrategicIHPPlugin::mob_op_cb(const cav_msgs::MobilityOperation& msg)
    {
        if (pm_.current_platoon_state == PlatoonState::LEADER)
        {
            mob_op_cb_leader(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
        {
            mob_op_cb_follower(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATEFOLLOWER)
        {
            mob_op_cb_candidatefollower(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADERWAITING)
        {
            mob_op_cb_leaderwaiting(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            mob_op_cb_standby(msg);
        }
        // UCLA: add leader aborting 
        else if (pm_.current_platoon_state == PlatoonState::LEADERABORTING)
        {
            mob_op_cb_leaderaborting(msg);
        }
        // UCLA: add candidate leader 
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATELEADER)
        {   
            mob_op_cb_candidateleader(msg);
        }    
        // UCLA: add lead with operation for cut-in join
        else if (pm_.current_platoon_state == PlatoonState::LEADWITHOPERATION)
        {
            mob_op_cb_leadwithoperation(msg);
        }
        // UCLA: add prepare to join for cut-in join
        else if (pm_.current_platoon_state == PlatoonState::PREPARETOJOIN)
        {
            mob_op_cb_preparetojoin(msg);
        }
        // TODO: If needed (with large size platoons), add a queue for status messages
        // INFO messages always processed, STATUS messages if saved in que
    }
    
    void PlatoonStrategicIHPPlugin::mob_op_cb_standby(const cav_msgs::MobilityOperation& msg)
    {
        // In standby state, it will ignore operation message since it is not actively operating
    }

    // Handle STATUS operation message 
    void PlatoonStrategicIHPPlugin::mob_op_cb_candidatefollower(const cav_msgs::MobilityOperation& msg)
    {
        // We still need to handle STATUS operAtion message from our platoon
        std::string strategyParams = msg.strategy_params;
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        if(isPlatoonStatusMsg) 
        {
            mob_op_cb_STATUS(msg);
            ROS_DEBUG_STREAM("Platoon STATUS operation message processed in candidatefollower state.");

        }
        else 
        {
            ROS_DEBUG_STREAM("Received a mobility operation message with params " << msg.strategy_params << " but ignored.");
        }
    }

    // Handle STATUS operation message 
    void PlatoonStrategicIHPPlugin::mob_op_cb_follower(const cav_msgs::MobilityOperation& msg)
    {
        std::string strategyParams = msg.strategy_params;
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        if(isPlatoonStatusMsg) 
        {
            mob_op_cb_STATUS(msg);
            ROS_DEBUG_STREAM("Platoon STATUS operation message processed in follower state.");
        }
        else 
        {
            ROS_DEBUG_STREAM("Received a mobility operation message with params " << msg.strategy_params << " but ignored.");
        }
    }

    // Handle STATUS operation message 
    void PlatoonStrategicIHPPlugin::mob_op_cb_leaderwaiting(const cav_msgs::MobilityOperation& msg)
    {
        std::string strategyParams = msg.strategy_params;
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        if(isPlatoonStatusMsg) 
        {
            mob_op_cb_STATUS(msg);
            ROS_DEBUG_STREAM("Platoon STATUS operation message processed in leaderwaiting state.");
        } 
        else 
        {
            ROS_DEBUG_STREAM("Received a mobility operation message with params " << msg.strategy_params << " but ignored.");
        }
    }

    // UCLA: Handle both STATUS and INFO operation message. Front join and rear join are all handled if incoming operation message have INFO param. 
    void PlatoonStrategicIHPPlugin::mob_op_cb_leader(const cav_msgs::MobilityOperation& msg)
    {   
        /** 
         * Note: This is the function to handle the mobility operation message. Vehicle in leader state is either a single ADS vehicle or a platoon leader.
         * 
         * Single ADS will send out INFO messages. Platoon leaders will send out both INFO and STATUS messages.
         * 
         * If the host is single vehicle, it shoud have "isPlatoonInfoMsg = true" and "isInNegotiation = false". In such condition, single leader will start 
         * joining and send request to the platoon leader. 
         * 
         * If the host vechile is platoon leader, then it should have "isPlatoonInfoMsg = true" and "isInNegotiation = true". But existing platoon leader do not 
         * need to send out joining request.
         * 
         * Both the platoon leader and single vehcile need to subscribe to the STATUS message to populate the platoon manager with existing platoon members, 
         * so the PM can calculate dtd and ctd corresponds to host vechile's origin position, hence to be used for later calculation.          
         *
         */

        // Read incoming message info
        std::string strategyParams = msg.strategy_params;
        std::string senderId = msg.header.sender_id; 
        std::string platoonId = msg.header.plan_id; 

        // In the current state, host vehicle care about the INFO heart-beat operation message if we are not currently in
        // a negotiation, and host also need to care about operation from members in our current platoon.

        bool isPlatoonInfoMsg = (strategyParams.rfind(OPERATION_INFO_TYPE, 0) == 0);        // INFO message boradcast by leader and single CAV. 
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);    // STATUS message broadcast by platoon leader only.
        bool isInNegotiation = (pm_.current_plan.valid == true);                            // In negotiation indicate the vehicle is not available for a new join (i.e., currently in a platoon or trying to join a platoon). 

        // If host vehicle recieves platoon leader's INFO message and not in negotiating with other platoon, host can start to request join.
        if(isPlatoonInfoMsg && !isInNegotiation)
        {
            // read ecef location from strategy params.
            cav_msgs::LocationECEF ecef_loc;
            ecef_loc = mob_op_find_ecef_from_INFO_params(strategyParams);
            
            // use ecef_loc to calculate front Dtd in m.
            lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(ecef_loc);
            double frontVehicleDtd = wm_->routeTrackPos(incoming_pose).downtrack;
            // use ecef_loc to calculate front Ctd in m.
            double frontVehicleCtd = wm_->routeTrackPos(incoming_pose).crosstrack;
            // downtrack and crosstrack of the platoon leader --> used for frontal join
            ROS_DEBUG_STREAM("frontVehicleDtd from ecef: " << frontVehicleDtd);
            ROS_DEBUG_STREAM("frontVehicleCtd from ecef: " << frontVehicleCtd);

            // Use pm_ to find platoon end vehicle and its downtrack in m.
            int rearVehicleIndex = pm_.getTotalPlatooningSize()- 1;
            double rearVehicleDtd = pm_.platoon[rearVehicleIndex].vehiclePosition; 
            double rearVehicleCtd = pm_.platoon[rearVehicleIndex].vehicleCrossTrack;
            // downtrack and crosstrack of the platoon rear vehicle --> used for frontal join
            ROS_DEBUG_STREAM("rearVehicleDtd from joining vehicle's platoon manager: " << rearVehicleDtd);
            ROS_DEBUG_STREAM("rearVehicleCtd from joining vehicle's platoon manager: " << rearVehicleCtd);

            // Read necessary info for platoon requests
            cav_msgs::MobilityRequest request;
            request.header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
            request.header.recipient_id = senderId;
            request.header.sender_id = config_.vehicleID;
            request.header.timestamp = ros::Time::now().toNSec()/1000000;
            request.location = pose_to_ecef(pose_msg_);
            request.strategy = MOBILITY_STRATEGY;
            
            int platoon_size = pm_.getTotalPlatooningSize();

            // Request Rear Join 
            if(isVehicleRightInFront(rearVehicleDtd, rearVehicleCtd))
            {   
                /**
                 *  Note: "isVehicleRightInFront" only apploes for same-lane maneuvers, so need to check for CtD.
                 */

                ROS_DEBUG_STREAM("Found a platoon with id = " << platoonId << " in front of us.");
                request.plan_type.type = cav_msgs::PlanType::JOIN_PLATOON_AT_REAR;

                /*
                 * SAME_LANE_JOIN_PARAMS format: 
                 *       SAME_LANE_JOIN_PARAMS| --> "SIZE:%1%,SPEED:%2%,ECEFX:%3%,ECEFY:%4%,ECEFZ:%5%"
                 *                            |-------0------ --1---------2---------3---------4------|
                 */

                boost::format fmter(SAME_LANE_JOIN_PARAMS);
                fmter %platoon_size;                //  index = 0
                fmter %current_speed_;              //  index = 1, in m/s
                fmter %pose_ecef_point_.ecef_x;     //  index = 2
                fmter %pose_ecef_point_.ecef_y;     //  index = 3
                fmter %pose_ecef_point_.ecef_z;     //  index = 4
                
                request.strategy_params = fmter.str();
                request.urgency = 50;

                pm_.current_plan = PlatoonPlan(true, request.header.timestamp, request.header.plan_id, request.header.sender_id);
                mobility_request_publisher_(request);
                ROS_DEBUG_STREAM("Publishing request to leader " << senderId << " with params " << request.strategy_params << " and plan id = " << request.header.plan_id);
                potentialNewPlatoonId = platoonId;
            }

            // // Request frontal join
            else if (isVehicleRightBehind(frontVehicleDtd, frontVehicleCtd))
            {   

                /**
                 *  Note: "isVehicleRightBehind" only apploes for same-lane maneuvers, so need to check for CtD.
                 */

                std::string newLeaderPlatoonId = pm_.currentPlatoonID; // set self's (front join veh) platoon ID as the new platoon ID
                ROS_DEBUG_STREAM("Host vehicle platoon id = " << newLeaderPlatoonId << "; join the rear platoon as the new leader (frontal join).");
                
                // UCLA: assign a new plan type
                request.plan_type.type = cav_msgs::PlanType::JOIN_PLATOON_FROM_FRONT;

                /**
                 * SAME_LANE_JOIN_PARAMS format: 
                 *       SAME_LANE_JOIN_PARAMS| --> "SIZE:%1%,SPEED:%2%,ECEFX:%3%,ECEFY:%4%,ECEFZ:%5%"
                 *                            |-------0------ --1---------2---------3---------4------|
                 */
                boost::format fmter(SAME_LANE_JOIN_PARAMS); // Note: Front and rear join uses same params, hence merge to one param for both condition.
                fmter %platoon_size;                //  index = 0
                fmter %current_speed_;              //  index = 1, in m/s
                fmter %pose_ecef_point_.ecef_x;     //  index = 2
                fmter %pose_ecef_point_.ecef_y;     //  index = 3
                fmter %pose_ecef_point_.ecef_z;     //  index = 4

                request.strategy_params = fmter.str();
                request.urgency = 50;

                // UCLA: generate new platoonplan for front join.
                pm_.current_plan = PlatoonPlan(true, request.header.timestamp, request.header.plan_id, request.header.sender_id);
                mobility_request_publisher_(request);
                ROS_DEBUG_STREAM("Publishing request to the leader " << senderId << " with params " << request.strategy_params << " and plan id = " << request.header.plan_id);
                potentialNewPlatoonId_front_ = newLeaderPlatoonId;
            }

            // UCLA: Condition passed the check for cut-in join
            else if (isVehicleNearPlatoon(current_downtrack_, current_crosstrack_))
            {
                /**
                 * UCLA Implementation note:
                 *  1. isVehicleNearPlatoon --> determine if the platoon is next to host vheicle
                 *  2. sender_id == requesting veh ID --> front joiner ID
                 */
                std::string newLeaderPlatoonId = pm_.currentPlatoonID; // set self's (front join veh) platoon ID as the new platoon ID
                ROS_DEBUG_STREAM("Host vehicle platoon id = " << newLeaderPlatoonId << "; join the rear platoon as the new leader (frontal join).");

                // UCLA: Desired joining index for cut-in join, indicate the index of gap-leading vehicle. -1 indicate cut-in from front.
                request.join_index = -1; // Note: not used by same-lane functions.
                // UCLA: this is a newly added plan type 
                request.plan_type.type = cav_msgs::PlanType::CUT_IN_FROM_SIDE; 

                /**
                 * JOIN_CUT_IN_PARAMS format: 
                 *       JOIN_CUT_IN_PARAMS| --> "SIZE:%1%,SPEED:%2%,ECEFX:%3%,ECEFY:%4%,ECEFZ:%5%"
                 *                            |-------0------ --1---------2---------3---------4------|
                 */
                boost::format fmter(JOIN_CUT_IN_PARAMS); // Note: Front and rear join uses same params, hence merge to one param for both condition.
                fmter %platoon_size;                //  index = 0
                fmter %current_speed_;              //  index = 1, in m/s
                fmter %pose_ecef_point_.ecef_x;     //  index = 2
                fmter %pose_ecef_point_.ecef_y;     //  index = 3
                fmter %pose_ecef_point_.ecef_z;     //  index = 4

                request.strategy_params = fmter.str();
                request.urgency = 50;

                // UCLA: generate new platoonplan for front join.
                pm_.current_plan = PlatoonPlan(true, request.header.timestamp, request.header.plan_id, request.header.sender_id);
                mobility_request_publisher_(request);
                ROS_DEBUG_STREAM("Publishing request to the leader " << senderId << " with params " << request.strategy_params << " and plan id = " << request.header.plan_id);
                potentialNewPlatoonId_front_ = newLeaderPlatoonId;
            }

            // Return none if no platoon nearby
            else 
            {
                ROS_DEBUG_STREAM("Ignore platoon with platoon id: " << platoonId << " because it is not right in front of us");
            }
        }
        
        // If host vehicle receives STATUS params, just update the condition.
        else if(isPlatoonStatusMsg) 
        {
            mob_op_cb_STATUS(msg);
            ROS_DEBUG_STREAM("Platoon STATUS operation message processed in leader state.");

        }
        
        // error/time out
        else
        {
            ROS_DEBUG_STREAM("Receive operation message but ignore it because isPlatoonInfoMsg = " << isPlatoonInfoMsg << 
            ", isInNegotiation = " << isInNegotiation << " and isPlatoonStatusMsg = " << isPlatoonStatusMsg);
        }
    }

    // UCLA: mob_op_cb for the new leader aborting state (inherited from candidate follower), handle STATUS message.
    void PlatoonStrategicIHPPlugin::mob_op_cb_leaderaborting(const cav_msgs::MobilityOperation& msg)
    {   
        std::string strategyParams = msg.strategy_params;
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        if(isPlatoonStatusMsg) 
        {
            mob_op_cb_STATUS(msg);
            ROS_DEBUG_STREAM("Platoon STATUS operation message processed in leaderaborting state.");
        }
        else 
        {
            ROS_DEBUG_STREAM("Received a mobility operation message with params " << msg.strategy_params << " but ignored.");
        }
    }
    
    // UCLA: mob_op_candidateleader for the new candidate leader state (inherited from leader waiting), handle STATUS message.
    void PlatoonStrategicIHPPlugin::mob_op_cb_candidateleader(const cav_msgs::MobilityOperation& msg)
    {   
        std::string strategyParams = msg.strategy_params;
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        if(isPlatoonStatusMsg) 
        {
            mob_op_cb_STATUS(msg);
            ROS_DEBUG_STREAM("Platoon STATUS operation message processed in candidateleader state.");
        }
        else {
            ROS_DEBUG_STREAM("Received a mobility operation message with params " << msg.strategy_params << " but ignored.");
        }
    }

    // UCLA: Mobility operation callback for lead_with_operation state (cut-in join).
    void PlatoonStrategicIHPPlugin::mob_op_cb_leadwithoperation(const cav_msgs::MobilityOperation& msg)
    {
        std::string strategyParams = msg.strategy_params;
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        std::string senderId = msg.header.sender_id; 

        if(isPlatoonStatusMsg) 
        {
            mob_op_cb_STATUS(msg);
            ROS_DEBUG_STREAM("Platoon STATUS operation message processed in Lead with Operation state.");
        } 
        else 
        {
            ROS_DEBUG_STREAM("Received a mobility operation message with params " << msg.strategy_params << " but ignored.");
        }
    }

    // UCLA: Mobility operation callback for prepare to join state (cut-in join).
    void PlatoonStrategicIHPPlugin::mob_op_cb_preparetojoin(const cav_msgs::MobilityOperation& msg)
    {
        /*
         * If same lane with leader, then send request to do same lane join. 
         * Otherwise, just send status params.
         * Note: leader in state 'leading with operation' also send out INFO msg
         */

        // read parameters
        std::string strategyParams = msg.strategy_params;
        std::string senderId = msg.header.sender_id; 
        
        // locate INFO type
        bool isPlatoonInfoMsg = (strategyParams.rfind(OPERATION_INFO_TYPE, 0) == 0);
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        bool isNotInNegotiation = (pm_.current_plan.valid == false);

        if (isPlatoonInfoMsg)
        {
            // read ecef location from strategy params.
            cav_msgs::LocationECEF ecef_loc;
            ecef_loc = mob_op_find_ecef_from_INFO_params(strategyParams);
            
            // use ecef_loc to calculate front Dtd in m.
            lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(ecef_loc);
            double frontVehicleDtd = wm_->routeTrackPos(incoming_pose).downtrack;

            // use ecef_loc to calculate front Ctd in m.
            double frontVehicleCtd = wm_->routeTrackPos(incoming_pose).crosstrack;

            // Use pm_ to find platoon end vehicle and its downtrack in m.
            int rearVehicleIndex = pm_.platoon.size() - 1;
            double rearVehicleDtd = pm_.platoon[rearVehicleIndex].vehiclePosition; 
            
            // downtrack of the platoon leader --> used for frontal join
            ROS_DEBUG_STREAM("rearVehicleDtd from ecef: " << rearVehicleDtd);

            // determine if the lane change is finished
            bool isSameLaneWithPlatoon = (frontVehicleCtd - current_crosstrack_ > -1 || frontVehicleCtd - current_crosstrack_ < 1);
            //  determine the cut-in joining type. 
            //TODO:more joining types will be added later.
            bool isCutInFront = (frontVehicleDtd < current_downtrack_);

            if (isSameLaneWithPlatoon)
            {
                // request 1. reset the safeToChangLane indicators if lane change is finished
                pm_.safeToLaneChange = false;
                
                // request 2. change to same lane operation states (determine based on DTD differences)
                cav_msgs::MobilityRequest request;
                request.header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
                request.header.recipient_id = senderId;
                request.header.sender_id = config_.vehicleID;
                request.header.timestamp = ros::Time::now().toNSec()/1000000;
                request.location = pose_to_ecef(pose_msg_);

                // UCLA: send request based on cut-in type
                if (isCutInFront) 
                {
                    request.plan_type.type = cav_msgs::PlanType::CUT_IN_FRONT_DONE;
                }
                else
                {
                    request.plan_type.type = cav_msgs::PlanType::CUT_IN_MID_OR_REAR_DONE;
                }
                request.strategy = MOBILITY_STRATEGY;
                request.strategy_params = "";
                request.urgency = 50;
                /* 
                 * UCLA: Add joining index for cut-in join. Necessary to update the cav_msgs/MobilityRequests.
                 * Note: For rear join, cut-in index == platoon.size()-1; for join from front, index == -1.
                 *       For cut-in in middle, index indicate the gap leading vehicle's index.
                 */       
                request.join_index = target_join_index_;
                mobility_request_publisher_(request); 
                ROS_DEBUG_STREAM("Published Mobility request to revert to same-lane operation"); 
            }
            else
            {
                ROS_DEBUG_STREAM("Lane Change not completed");
            }
        }
        
        // STATUS params
        else if(isPlatoonStatusMsg) 
        {
            mob_op_cb_STATUS(msg);
            ROS_DEBUG_STREAM("Platoon STATUS operation message processed in Prepare to Join state.");
        }
        
        // error/time out
        else
        {
            ROS_DEBUG_STREAM("Receive operation message but ignore it because isPlatoonInfoMsg = " << isPlatoonInfoMsg << 
            ", isNotInNegotiation = " << isNotInNegotiation << " and isPlatoonStatusMsg = " << isPlatoonStatusMsg);
        }

    }
    
    //------- 3. Mobility Request Callback -------
    MobilityRequestResponse PlatoonStrategicIHPPlugin::handle_mob_req(const cav_msgs::MobilityRequest& msg)
    {
        MobilityRequestResponse mobility_response;


        if (pm_.current_platoon_state == PlatoonState::LEADER)
        {
            mobility_response = mob_req_cb_leader(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
        {
            mobility_response = mob_req_cb_follower(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATEFOLLOWER)
        {
            mobility_response = mob_req_cb_candidatefollower(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADERWAITING)
        {
            mobility_response = mob_req_cb_leaderwaiting(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            mobility_response = mob_req_cb_standby(msg);
        }
        // UCLA: leader aborting
        else if (pm_.current_platoon_state == PlatoonState::LEADERABORTING)
        {
            mobility_response = mob_req_cb_leaderaborting(msg);
        }
        // UCLA: candidate leader 
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATELEADER)
        {
            mobility_response = mob_req_cb_candidateleader(msg);
        }

        // UCLA: lead with operation (for cut-in join)
        else if (pm_.current_platoon_state == PlatoonState::LEADWITHOPERATION)
        {
            mobility_response = mob_req_cb_leadwithoperation(msg);
        }
        // UCLA: prepare to join (for cut-in join)
        else if (pm_.current_platoon_state == PlatoonState::PREPARETOJOIN)
        {
            mobility_response = mob_req_cb_preparetojoin(msg);
        }

        return mobility_response;
    }

    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_standby(const cav_msgs::MobilityRequest& msg)
    {
        // In standby state, the plugin is not responsible for replying to any request messages
        return MobilityRequestResponse::NO_RESPONSE;
    }

    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_candidatefollower(const cav_msgs::MobilityRequest& msg)
    {
        // This state does not handle any mobility request for now
        // TODO Maybe it should handle some ABORT request from a waiting leader
        ROS_DEBUG_STREAM("Recived mobility request with type " << msg.plan_type.type << " but ignored.");
        return MobilityRequestResponse::NO_RESPONSE;
    }

    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_follower(const cav_msgs::MobilityRequest& msg)
    {
        return MobilityRequestResponse::NO_RESPONSE;
    }
    
    // Middle state that decided whether to accept joiner  
    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_leaderwaiting(const cav_msgs::MobilityRequest& msg)
    {
        bool isTargetVehicle = (msg.header.sender_id == lw_applicantId_);
        bool isCandidateJoin = msg.plan_type.type == cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN;

        lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(msg.location);
        double obj_cross_track = wm_->routeTrackPos(incoming_pose).crosstrack;
        bool inTheSameLane = (abs(obj_cross_track - current_crosstrack_) < config_.maxCrosstrackError);
        ROS_DEBUG_STREAM("current_cross_track error = " << abs(obj_cross_track - current_crosstrack_));
        ROS_DEBUG_STREAM("inTheSameLane = " << inTheSameLane);
        if (isTargetVehicle && isCandidateJoin && inTheSameLane)
        {
            ROS_DEBUG_STREAM("Target vehicle " << lw_applicantId_ << " is actually joining.");
            ROS_DEBUG_STREAM("Changing to PlatoonLeaderState and send ACK to target vehicle");
            pm_.current_platoon_state = PlatoonState::LEADER;
            return MobilityRequestResponse::ACK;
        }
        else
        {
            ROS_DEBUG_STREAM("Received platoon request with vehicle id = " << msg.header.sender_id);
            ROS_DEBUG_STREAM("The request type is " << msg.plan_type.type << " and we choose to ignore");
            return MobilityRequestResponse::NO_RESPONSE;
        }
    }
    
    // UCLA: add condition to handle frontal join request
    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_leader(const cav_msgs::MobilityRequest& msg)
    {   
        /**
         *   UCLA implementation note: 
         *   1. Here the mobility requests of joining platoon (front and rear) get processed and responded.
         *    2. The host is the leader of the existing platoon or single vehicle in default leader state.
         *    3. Request sender is the joiner.
         *    4. when two single vehicle meet, only allow backjoin.
         *
         *   Note: For front and rear jon, the SAME_LANE_JOIN_PARAMS format: 
         *        SAME_LANE_JOIN_PARAMS| --> "SIZE:%1%,SPEED:%2%,ECEFX:%3%,ECEFY:%4%,ECEFZ:%5%"
         *                             |-------0------ --1---------2---------3---------4------|  
         */

        cav_msgs::PlanType plan_type = msg.plan_type;

        // Check joining plan type.
        /**
         *  Note:
         *      JOIN_FROM_FRONT indicate a same-lane front join.
         *      JOIN_PLATOON_AT_REAR indicate a same-lane rear join.
         *      CUT_IN_FROM_SIDE indicate a cut-in join, which include three cut-in methods: cut-in front, cut-in middle, and cut-in rear.
         * 
         */
        bool isFrontJoin = (plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_FROM_FRONT);
        bool isRearJoin = (plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_AT_REAR);
        bool isCutinJoin = (plan_type.type == cav_msgs::PlanType::CUT_IN_FROM_SIDE);

        // Determine intra-platoon conditions
        // We are currently checking two basic JOIN conditions:
        //     1. The size limitation on current platoon based on the plugin's parameters.
        //     2. Calculate how long that vehicle can be in a reasonable distance to actually join us.
        // TODO: We ignore the lane information for now and assume the applicant is in the same lane with us.
        cav_msgs::MobilityHeader msgHeader = msg.header;
        std::string params = msg.strategy_params;
        std::string applicantId = msgHeader.sender_id;
        ROS_DEBUG_STREAM("Receive mobility JOIN request from " << applicantId << " and PlanId = " << msgHeader.plan_id);
        ROS_DEBUG_STREAM("The strategy parameters are " << params);
        if (params == "")
        {
            ROS_DEBUG_STREAM("The strategy parameters are empty, return no response");
            return MobilityRequestResponse::NO_RESPONSE;
        }

        // The inmcoming message is "mobility Request", which has a location category.
        std::vector<std::string> inputsParams;
        boost::algorithm::split(inputsParams, params, boost::is_any_of(","));

        // Parse applicantSize
        std::vector<std::string> applicantSize_parsed;
        boost::algorithm::split(applicantSize_parsed, inputsParams[0], boost::is_any_of(":"));
        int applicantSize = std::stoi(applicantSize_parsed[1]);
        ROS_DEBUG_STREAM("applicantSize: " << applicantSize);

        // Parse applicant Current Speed in m/s
        std::vector<std::string> applicantCurrentSpeed_parsed;
        boost::algorithm::split(applicantCurrentSpeed_parsed, inputsParams[1], boost::is_any_of(":"));
        double applicantCurrentSpeed = std::stod(applicantCurrentSpeed_parsed[1]);
        ROS_DEBUG_STREAM("applicantCurrentSpeed: " << applicantCurrentSpeed);

        // Calculate downtrack (m) bsaed on ecef. 
        lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(msg.location);
        double applicantCurrentDtd = wm_->routeTrackPos(incoming_pose).downtrack;
        ROS_DEBUG_STREAM("applicantCurrentmemberUpdates from ecef pose: " << applicantCurrentDtd);

        // Calculate crosstrack (m) bsaed on incoming pose. 
        double applicant_crosstrack = wm_->routeTrackPos(incoming_pose).crosstrack;
        ROS_DEBUG_STREAM("applicant_crosstrack from ecef pose: " << applicant_crosstrack);
        bool isInLane = (abs(applicant_crosstrack - current_crosstrack_) < config_.maxCrosstrackError);
        ROS_DEBUG_STREAM("isInLane = " << isInLane);
        
        // Check if we have enough room for that applicant
        int currentPlatoonSize = pm_.getTotalPlatooningSize();
        bool hasEnoughRoomInPlatoon = applicantSize + currentPlatoonSize <= config_.maxPlatoonSize;

        // check if the current request is from the first joining vehicle 
        bool isFirstRequest = (applicantId==joiningID_ or joiningID_ =="" );

        // rear join; platoon leader --> leader waiting
        if (isRearJoin)
        {
            // -- core condition to decided accept joiner or not
            if (hasEnoughRoomInPlatoon && isInLane) 
            {
                ROS_DEBUG_STREAM("The current platoon has enough room for the applicant with size " << applicantSize);
                double currentRearDtd = pm_.getPlatoonRearDowntrackDistance();
                ROS_DEBUG_STREAM("The current platoon rear dtd is " << currentRearDtd);
                double currentGap = currentRearDtd - applicantCurrentDtd - config_.vehicleLength;
                double currentTimeGap = currentGap / applicantCurrentSpeed;
                ROS_DEBUG_STREAM("The gap between current platoon rear and applicant is " << currentGap << "m or " << currentTimeGap << "s");
                if (currentGap < 0) 
                {
                    ROS_WARN("We should not receive any request from the vehicle in front of us. NACK it.");
                    return MobilityRequestResponse::NACK;
                }
                // Check if the applicant can join based on max timeGap/gap
                bool isDistanceCloseEnough = (currentGap <= config_.maxAllowedJoinGap) || (currentTimeGap <= config_.maxAllowedJoinTimeGap);
                if (isDistanceCloseEnough) 
                {
                    ROS_DEBUG_STREAM("The applicant is close enough and we will allow it to try to join");
                    ROS_DEBUG_STREAM("Change to LeaderWaitingState and waiting for " << msg.header.sender_id << " to join");
                    // change state to leaderwaiting !
                    pm_.current_platoon_state = PlatoonState::LEADERWAITING;
                    waitingStartTime = ros::Time::now().toNSec() / 1000000;
                    lw_applicantId_ = msg.header.sender_id;  // this is ID of the backjoin applicant (plan sender is always the car trying to find a leader)
                    // plugin.setState(new LeaderWaitingState(plugin, log, pluginServiceLocator, applicantId));
                    return MobilityRequestResponse::ACK;
                }
                else 
                {
                    ROS_DEBUG_STREAM("The applicant is too far away from us. NACK.");
                    return MobilityRequestResponse::NACK;
                }
            }
            else
            {
                ROS_DEBUG_STREAM("The current platoon does not have enough room for applicant of size " << applicantSize << ". NACK");
                return MobilityRequestResponse::NACK;
            }
        }
        
        // front join; platoon leader --> leader aborting
        else if (isFrontJoin)
        {
            // -- core condition to decided accept joiner or not
            if (hasEnoughRoomInPlatoon && isInLane) 
            {
                ROS_DEBUG_STREAM("The current platoon has enough room for the applicant with size " << applicantSize);
                
                // UCLA: change to read platoon front info
                double currentFrontDtd = pm_.getPlatoonFrontDowntrackDistance();
                ROS_DEBUG_STREAM("The current platoon rear dtd is " << currentFrontDtd);
                // UCLA: adjust for calculating gap between new leader and old leader
                double currentGap =  applicantCurrentDtd - currentFrontDtd - config_.vehicleLength;
                double currentTimeGap = currentGap / applicantCurrentSpeed;
                ROS_DEBUG_STREAM("The gap between current platoon rear and applicant is " << currentGap << "m or " << currentTimeGap << "s");
                
                if (currentGap < 0) 
                {
                    ROS_WARN("The current time gap is not applicatble for frontal join. NACK it.");
                    return MobilityRequestResponse::NACK;
                }
                // Check if the applicant can join based on max timeGap/gap
                bool isDistanceCloseEnough = (currentGap <= config_.maxAllowedJoinGap) || (currentTimeGap <= config_.maxAllowedJoinTimeGap);

                // UCLA: add condition: only allow front join when platoon length >= 2 (make sure when two single vehicle join, only use back join)
                bool isPlatoonNotSingle = (pm_.getTotalPlatooningSize() >= 2) ;

                if (isDistanceCloseEnough && isPlatoonNotSingle) 
                {
                    ROS_DEBUG_STREAM("The applicant is close enough for frontal join, send acceptance response");
                    ROS_DEBUG_STREAM("Change to LeaderAborting state and waiting for " << msg.header.sender_id << " to join as the new platoon leader");

                    // ----------------- give up leader position and look for new leader --------------------------
                    ROS_DEBUG_STREAM("Received positive response for plan id = " << pm_.current_plan.planId);
                    ROS_DEBUG_STREAM("Change to CandidateFollower state and notify trajectory failure in order to replan");

                    // adjust for frontal join. Platoon info is related to the platoon at back of the candidate leader vehicle.
                    pm_.current_platoon_state = PlatoonState::LEADERABORTING;
                    candidatestateStartTime = ros::Time::now().toNSec() / 1000000;

                    // change platoon ID to candidate leader --> read it from current platoonplan, plan is send out by frontal joiner
                    targetPlatoonId = potentialNewPlatoonId_front_;
                    ROS_DEBUG_STREAM("targetPlatoonId = " << targetPlatoonId);
                    // change leader to candidate leader --> read it from current platoonplan --> peerID is the frontal joiner vehID 
                    pm_.targetLeaderId = pm_.current_plan.peerId;
                    ROS_DEBUG_STREAM("pm_.targetLeaderId = " << pm_.targetLeaderId);
                    // -----------------------------------------------------------------------------------------
                    
                    waitingStartTime = ros::Time::now().toNSec() / 1000000;

                    // add front join new joiner ID (fj_new_joiner_Id_)
                    fj_new_joiner_Id_ = msg.header.sender_id; // old leader that is applying to join new leader at front

                    // plugin.setState(new LeaderWaitingState(plugin, log, pluginServiceLocator, applicantId));
                    return MobilityRequestResponse::ACK;

                }
                else 
                {
                    ROS_DEBUG_STREAM("The applicant is too far away from us. NACK.");
                    return MobilityRequestResponse::NACK;
                }
            }
            else
            {
                ROS_DEBUG_STREAM("The current platoon does not have enough room for applicant of size " << applicantSize << ". NACK");
                return MobilityRequestResponse::NACK;
            }
        }
        
        // UCLA: conditions for cut-in join; platoon leader --> leading with operation
        // Use joningID_ to make sure only the leader only interact with the first requesting joining vehicle
        else if (isCutinJoin && isFirstRequest)
        {
            
            // -- core condition to decided accept joiner or not. It is necessary leader only process the first cut-in joining reqeust.
            if (hasEnoughRoomInPlatoon && isVehicleNearPlatoon(current_downtrack_, current_crosstrack_) )
            {
                ROS_DEBUG_STREAM("The current platoon has enough room for the applicant with size " << applicantSize);
                ROS_DEBUG_STREAM("The applicant is close enough for cut-in join, send acceptance response");
                ROS_DEBUG_STREAM("The leader will keep track of the joining vehicle's ID untill the end of the current cut-in maneuver.");
                
                // update the joiningID_ if the current joining request is the first request
                joiningID_ = (joiningID_=="") ? applicantId : joiningID_;
                
                ROS_DEBUG_STREAM("Change to Leading with operation state and waiting for " << msg.header.sender_id << " to change lane");
                // change state to lead with operation
                pm_.current_platoon_state = PlatoonState::LEADWITHOPERATION;
                waitingStartTime = ros::Time::now().toNSec() / 1000000;
                // update leader info
                pm_.targetLeaderId = pm_.current_plan.peerId;
                ROS_DEBUG_STREAM("pm_.targetLeaderId = " << pm_.targetLeaderId);

                return MobilityRequestResponse::ACK;
            }
            
            else
            {   
                ROS_DEBUG_STREAM("The current platoon does not have enough room or the applicant is too far away from us. NACK the request.");
                ROS_DEBUG_STREAM("The current applicant size: " << applicantSize << ".");
                ROS_DEBUG_STREAM("The applicant downtrack is: " << current_downtrack_ << ".");
                ROS_DEBUG_STREAM("The applicant crosstrack is: " << current_crosstrack_ << ".");
                return MobilityRequestResponse::NACK;
            }

        }
        
        // no response 
        else 
        {
            ROS_DEBUG_STREAM("Received mobility request with type " << msg.plan_type.type << " and ignored.");
            return MobilityRequestResponse::NO_RESPONSE;
        }
    }
    
    // UCLA: mobility request leader aborting (inherited from candidate follower)
    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_leaderaborting(const cav_msgs::MobilityRequest& msg)
    {
        // This state does not handle any mobility request for now
        // TODO Maybe it should handle some ABORT request from a waiting leader
        ROS_DEBUG_STREAM("Recived mobility request with type " << msg.plan_type.type << " but ignored.");
        return MobilityRequestResponse::NO_RESPONSE;
    }

    // UCLA: mobility request candidate leader (inherited from leader waiting)
    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_candidateleader(const cav_msgs::MobilityRequest& msg)
    {   
        
        bool isTargetVehicle = (msg.header.sender_id == fj_new_joiner_Id_); // need to check: senderID (old leader) == front join applicant ID
        bool isCandidateJoin = msg.plan_type.type == cav_msgs::PlanType::PLATOON_FRONT_JOIN;

        lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(msg.location);
        double obj_cross_track = wm_->routeTrackPos(incoming_pose).crosstrack;
        bool inTheSameLane = (abs(obj_cross_track - current_crosstrack_) < config_.maxCrosstrackError);
        ROS_DEBUG_STREAM("current_cross_track error = " << abs(obj_cross_track - current_crosstrack_));
        ROS_DEBUG_STREAM("inTheSameLane = " << inTheSameLane);
        if (isTargetVehicle && isCandidateJoin && inTheSameLane)
        {
            ROS_DEBUG_STREAM("New joiner vehicle " << fj_new_joiner_Id_ << " (previous leader) is joining .");
            ROS_DEBUG_STREAM("Changing to PlatoonLeaderState and send ACK to the new joiner vehicle");
            pm_.current_platoon_state = PlatoonState::LEADER;
            return MobilityRequestResponse::ACK;
        }
        else
        {
            ROS_DEBUG_STREAM("Received platoon request with vehicle id = " << msg.header.sender_id);
            ROS_DEBUG_STREAM("The request type is " << msg.plan_type.type << " and we choose to ignore");
            return MobilityRequestResponse::NO_RESPONSE;
        }
    }
    
    // UCLA: add request call-back function for lead with operation state (for cut-in join)
    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_leadwithoperation(const cav_msgs::MobilityRequest& msg)
    {
        /*
        *   Current leader leading while opening the gap for cut-in join.
        */

        // Check request plan type  
        cav_msgs::PlanType plan_type = msg.plan_type;

        // Calculate downtrack (m) bsaed on ecef. 
        lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(msg.location);
        // read downtrack
        double applicantCurrentDtd = wm_->routeTrackPos(incoming_pose).downtrack;
        ROS_DEBUG_STREAM("Applicant downtrack from ecef pose: " << applicantCurrentDtd);

        if (plan_type.type == cav_msgs::PlanType::PLATOON_CUTIN_JOIN) 
        {
            // task 1. slow down to create gap 
            int join_index = msg.join_index;
            // determine if joining vehicle in position
            double cutinDtdDifference = applicantCurrentDtd - current_downtrack_ - config_.vehicleLength;
            bool isFrontJoinerInPosition = (cutinDtdDifference <= 1.5*config_.vehicleLength) || (cutinDtdDifference >= -1.5*config_.vehicleLength);
            // todo: reserved for cut-in from middle/rear
            // bool isMidJoinerInPosition = ((pm_.platoon[join_index].vehiclePosition - current_downtrack_ <= 3*config_.vehicleLength)); 
            // bool isRearJoinerInPosition = (pm_.platoon[pm_.platoon.size()-1].vehiclePosition - current_downtrack_ <= 3*config_.vehicleLength);

            if (join_index == -1 && isFrontJoinerInPosition )
            {
                if (cutinDtdDifference < config_.vehicleLength)
                {
                    // slow down leader to allow joiner cut-in
                    pm_.isCreateGap = true;
                }
                
                ROS_DEBUG_STREAM("The joining vehicle is cutting in from front. Notify platoon leader to slow down");
                ROS_DEBUG_STREAM("Slow down notified, joining vehicle can prepare to join");
                return MobilityRequestResponse::ACK;
            }
            /*
            *************** Reserve for cut-in from middle and cut-in from rear. *****************
            // cut-in rear
            else if ((join_index == pm_.platoon.size()-1) && isRearJoinerInPosition)
            {
                // todo: send request to last member to slow down (i.e., set isCreateGap to true) 
                request. 
                request.
                ROS_DEBUG_STREAM("");
                return MobilityRequestResponse::ACK;
            }
            // cut-in and join in middle
            else if (isMidJoinerInPosition)
            {
                // todo: send request to index member to slow down (i.e., set isCreateGap to true) 
                request. 
                request.
                ROS_DEBUG_STREAM("");
                return MobilityRequestResponse::ACK;
            }
            */
        }

        // task 2: For cut-in from front, the leader need to stop creating gap when gap is big enough
        if ((plan_type.type == cav_msgs::PlanType::STOP_CREATE_GAP) && (pm_.isCreateGap) ) 
        {
            // reset creat gap indicator
            pm_.isCreateGap = false;
            // no need to response, simple reset the indicator
            return MobilityRequestResponse::NO_RESPONSE;

        }

        // task 3.1 cut-in front: After creating gap, revert back to same-lane operation 
        if (plan_type.type == cav_msgs::PlanType::CUT_IN_FRONT_DONE)
        {
            ROS_DEBUG_STREAM("Cut-in from front lane change finished, leader revert to same-lane maneuver, leader reset member variable joiningID_.");
            joiningID_ = "";
            pm_.current_platoon_state = PlatoonState::LEADERABORTING;
            return MobilityRequestResponse::ACK;
        }

        // todo: task 3 cut-in from middle/rear

    }

    // UCLA: add request call-back function for prepare to join (for cut-in join)
    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_preparetojoin(const cav_msgs::MobilityRequest& msg)
    {
        // This state does not handle any mobility request for now
        // TODO: if joining vehicle need to adjust speed, the leader should request it and the request should be handled here. 
        ROS_DEBUG_STREAM("Received mobility request with type " << msg.plan_type.type << " but ignored.");
        return MobilityRequestResponse::NO_RESPONSE;
    }

    // ------ 4. Mobility response callback ------ //
    
    // Mobility response callback for all states.
    void PlatoonStrategicIHPPlugin::mob_resp_cb(const cav_msgs::MobilityResponse& msg)
    {    
        // Firstly, check eligibility of the received message. 
        bool isCurrPlanValid = pm_.current_plan.valid;                          // Check if current plan is still valid (i.e., not timed out).
        bool isForCurrentPlan = msg.header.plan_id == pm_.current_plan.planId;  // Check if plan Id matches.
        // bool isFromTargetVehicle = msg.header.sender_id == pm_.targetLeaderId;  // Check of target leader ID and sender ID matches.
        
        if (!(isCurrPlanValid && isForCurrentPlan)) 
        {
            /**
             * If any of the three condition (i.e., isCurrPlanValid, isForCurrentPlan and isFromTargetVehicle) 
             * was not satisfied, return ignore as this message was not intended for the host. 
             */  
            ROS_DEBUG_STREAM(" Ignore the received response message as it was not intended for the host vehicle.");
            return;
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADER)
        {
            mob_resp_cb_leader(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
        {
            mob_resp_cb_follower(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATEFOLLOWER)
        {
            mob_resp_cb_candidatefollower(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADERWAITING)
        {
            mob_resp_cb_leaderwaiting(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            mob_resp_cb_standby(msg);
        }
        // UCLA: add leader aboorting 
        else if (pm_.current_platoon_state == PlatoonState::LEADERABORTING)
        {
            mob_resp_cb_leaderaborting(msg);
        }
        //UCLA: add candidate leader 
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATELEADER)
        {
            mob_resp_cb_candidateleader(msg);
        }
        // UCLA: add lead with operation for cut-in join
        else if (pm_.current_platoon_state == PlatoonState::LEADWITHOPERATION)
        {
            mob_resp_cb_leadwithoperation(msg);
        }
        // UCLA: add prepare to join for cut-in join
        else if (pm_.current_platoon_state == PlatoonState::PREPARETOJOIN)
        {
            mob_resp_cb_preparetojoin(msg);
        }
    }

    void PlatoonStrategicIHPPlugin::mob_resp_cb_standby(const cav_msgs::MobilityResponse& msg)
    {
        // In standby state, it will not send out any requests so it will also ignore all responses
    }

    void PlatoonStrategicIHPPlugin::mob_resp_cb_candidatefollower(const cav_msgs::MobilityResponse& msg)
    {
        ROS_DEBUG_STREAM("Callback for candidate follower ");
        
        // Check if current plan is still valid (i.e., not timed out)
        if (pm_.current_plan.valid)
        {
            bool isForCurrentPlan = msg.header.plan_id == pm_.current_plan.planId;
            bool isFromTargetVehicle = msg.header.sender_id == pm_.targetLeaderId;
            ROS_DEBUG_STREAM("isForCurrentPlan " << isForCurrentPlan);

            ROS_DEBUG_STREAM("isFromTargetVehicle " << isFromTargetVehicle);

            // Check the response is received correctly (i.e., host vehicle is the desired receiver).
            if (isForCurrentPlan && isFromTargetVehicle)
            {
                if (msg.is_accepted)
                {
                    // We change to follower state and start to actually follow that leader
                    // The platoon manager also need to change the platoon Id to the one that the target leader is using 
                    ROS_DEBUG_STREAM("The leader " << msg.header.sender_id << " agreed on our join. Change to follower state.");
                    pm_.current_platoon_state = PlatoonState::FOLLOWER;
                    targetPlatoonId = msg.header.plan_id;
                    // change platoon manager to follower state 
                    pm_.changeFromLeaderToFollower(targetPlatoonId);
                    ROS_WARN("changed to follower");

                }
                else
                {
                    // We change back to normal leader state and try to join other platoons
                    ROS_DEBUG_STREAM("The leader " << msg.header.sender_id << " does not agree on our join. Change back to leader state.");
                    pm_.current_platoon_state = PlatoonState::LEADER;
                }
            }
            else
            {
                ROS_DEBUG_STREAM("Ignore received response message because it is not for the current plan.");
            }
        }
        else
        {
            ROS_DEBUG_STREAM("Ignore received response message because we are not in any negotiation process.");
        }
    }

    void PlatoonStrategicIHPPlugin::mob_resp_cb_leaderwaiting(const cav_msgs::MobilityResponse& msg)
    {
        /**
         * Leader waiting is the state to check joining vehicle is in proper position 
         * and to prevent platoon leader from receiving messages from other CAVs in leader state. 
         * There was no response involved in this state, hence no action needed in this section.  
         */ 
    }

    
    void PlatoonStrategicIHPPlugin::mob_resp_cb_follower(const cav_msgs::MobilityResponse& msg)
    {   
        /**
         * UCLA Note: 
         * 
         * This method was implemented with the purpose of updating the platoon related refernces 
         * (i.e., platoon leader, platoon Id) to the new leader that joined from front. Changing 
         * the existing follower to candidate follower state will initiate a member update and 
         * therefore point all refernce to the newe leader.
         * 
         * For rear join, since the existing follower already following the platoon leader. There is 
         * no need to establish communication hence no response will be handled for rear-join.
         * 
         */ 

        // UCLA: read plan type 
        cav_msgs::PlanType plan_type = msg.plan_type;
        
        // UCLA: determine joining type 
        bool isFrontJoin = (plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_FROM_FRONT);

        // UCLA: add response so follower can change to candidate follower, then change leader
        if (isFrontJoin && msg.is_accepted)
        {   
            // if frontal join is accepted, change followers to candidate follower to update leader
            ROS_DEBUG_STREAM("Received positive response for front-join plan id = " << pm_.current_plan.planId);
            ROS_DEBUG_STREAM("Change to CandidateFollower state and prepare to update platoon infomration");
            // Change to candidate follower state and request a new plan to catch up with the front platoon
            pm_.current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
            candidatestateStartTime = ros::Time::now().toNSec() / 1000000;
            targetPlatoonId = potentialNewPlatoonId_front_;
            ROS_DEBUG_STREAM("targetPlatoonId = " << targetPlatoonId);
            pm_.targetLeaderId = pm_.current_plan.peerId;
            ROS_DEBUG_STREAM("pm_.targetLeaderId = " << pm_.targetLeaderId);
        }
        
    }

    // UCLA: add conditions to account for frontal join states (candidate follower) 
    void PlatoonStrategicIHPPlugin::mob_resp_cb_leader(const cav_msgs::MobilityResponse& msg)
    {   
        /**  
         *  UCLA implmentation note:
         *  This is where the Mobility response gets processed for leader state. 
         *  
         *  If the host is a single vehicle in the leader state, then the host vehicle is the 
         *  joiner vehicle (frontal join: candidate leader; back join: candidate follower), and 
         *  the response sender is the existing platoon leader (front join: aborting leader, back join: waiting leader). 
         * 
         *  If the host is the current platoon leader, all three case will be false and no further action is needed.
         * 
         *  Disclaimer: Currently, if the host vehicle is platoon leader, there is no further action needed
         *  when receiving the mobility response. However, future developement may add functions in thie mehtod.
         */

        // UCLA: read plan type 
        cav_msgs::PlanType plan_type = msg.plan_type;
        
        // UCLA: determine joining type 
        bool isCutInJoin = (plan_type.type == cav_msgs::PlanType::CUT_IN_FROM_SIDE);
        bool isRearJoin = (plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_AT_REAR);
        bool isFrontJoin = (plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_FROM_FRONT);
        
        
        // Check if current plan is still valid (i.e., not timed out).
        if (pm_.current_plan.valid)
        {
            // Check the response is received correctly (i.e., host vehicle is the desired receiver).
            if (pm_.current_plan.planId == msg.header.plan_id && pm_.current_plan.peerId == msg.header.sender_id) 
            {   
                // bool indicator is initiated in header files
                // rear join
                if (isRearJoin && msg.is_accepted)
                {
                    ROS_DEBUG_STREAM("Received positive response for plan id = " << pm_.current_plan.planId);
                    ROS_DEBUG_STREAM("Change to CandidateFollower state and notify trajectory failure in order to replan");
                    // Change to candidate follower state and request a new plan to catch up with the front platoon
                    pm_.current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
                    candidatestateStartTime = ros::Time::now().toNSec() / 1000000;
                    targetPlatoonId = potentialNewPlatoonId;
                    ROS_DEBUG_STREAM("targetPlatoonId = " << targetPlatoonId);
                    pm_.targetLeaderId = pm_.current_plan.peerId;
                    ROS_DEBUG_STREAM("pm_.targetLeaderId = " << pm_.targetLeaderId);
                }

                // UCLA: frontal join (candidate leader, inherited from leaderwaiting)
                else if (isFrontJoin && msg.is_accepted)
                {   
                    
                    ROS_DEBUG_STREAM("Received positive response for plan id = " << pm_.current_plan.planId);
                    ROS_DEBUG_STREAM("Change to CandidateLeader state and prepare to become new leader. ");

                    // Change to candidate leader and idle
                    pm_.current_platoon_state = PlatoonState::CANDIDATELEADER;
                    candidatestateStartTime = ros::Time::now().toNSec() / 1000000;
                    // do not need to update platoon leader and platoon ID for candidate leader

                }
                // UCLA: CutIn join 
                else if (isCutInJoin && msg.is_accepted)
                {
                    ROS_DEBUG_STREAM("Received positive response for plan id = " << pm_.current_plan.planId);
                    ROS_DEBUG_STREAM("Change to Prepare to join state and prepare to change lane. ");

                    // Change to candidate leader and idle
                    pm_.current_platoon_state = PlatoonState::PREPARETOJOIN;
                    candidatestateStartTime = ros::Time::now().toNSec() / 1000000;
                    // do not need to update platoon leader and platoon ID for candidate leader
                }
                // Current platoon leader
                else if(pm_.platoonSize >= 2)
                {
                    // Keep the leader idling if the host is leading the plaoon.
                    ROS_DEBUG_STREAM("Host received response for joinging vehicles, remain idling as the host is the current platoon leader.");
                }
                else
                {
                    ROS_DEBUG_STREAM("Received negative response for plan id = " << pm_.current_plan.planId);
                    // Forget about the previous plan totally
                    pm_.current_plan.valid = false;
                }
            }
            else
            {
                ROS_DEBUG_STREAM("Ignore the response message because planID match: " << (pm_.current_plan.planId == msg.header.plan_id));
                ROS_DEBUG_STREAM("My plan id = " << pm_.current_plan.planId << " and response plan Id = " << msg.header.plan_id);
                ROS_DEBUG_STREAM("And peer id match " << (pm_.current_plan.peerId == msg.header.sender_id));
                ROS_DEBUG_STREAM("Expected peer id = " << pm_.current_plan.peerId << " and response sender Id = " << msg.header.sender_id);
            }
        }
    }

    // UCLA: response for leader aborting (inherited from candidate follower)
    void PlatoonStrategicIHPPlugin::mob_resp_cb_leaderaborting(const cav_msgs::MobilityResponse& msg)
    {   
        /**  
         *  UCLA implementation note:
         *  This state is the middle state to handle the leader aborting process of 
         *  the previous platoon leader in a front join scenario. Within this state, the 
         *  previous leader will check for front joining vehicle's position and will not
         *  handle any further mobility requests. 
         * 
         *  Note: As the previous leader will join the new leader (joined from front), the
         *  corresponding join request will be send out by the previos leader.
         *  
         */ 
        
        ROS_DEBUG_STREAM("Callback for leader aborting !");
        
        // Check if current plan is still valid (i.e., not timed out).
        if (pm_.current_plan.valid)
        {
            bool isForCurrentPlan = msg.header.plan_id == pm_.current_plan.planId;
            bool isFromTargetVehicle = msg.header.sender_id == pm_.targetLeaderId;
            ROS_DEBUG_STREAM("isForCurrentPlan " << isForCurrentPlan);

            ROS_DEBUG_STREAM("isFromTargetVehicle " << isFromTargetVehicle);

            // Check the response is received correctly (i.e., host vehicle is the desired receiver).
            if (isForCurrentPlan && isFromTargetVehicle)
            {
                if (msg.is_accepted)
                {
                    // We change to follower state and start to actually follow the new leader
                    // The platoon manager also need to change the platoon Id to the one that the target leader is using                
                    ROS_DEBUG_STREAM("The new leader " << msg.header.sender_id << " agreed on the frontal join. Change to follower state.");
                    pm_.current_platoon_state = PlatoonState::FOLLOWER;
                    targetPlatoonId = msg.header.plan_id;
                    // change platoon manager to follower state 
                    pm_.changeFromLeaderToFollower(targetPlatoonId);
                    ROS_WARN("changed to follower");

                }
                else
                {
                    // We change back to normal leader state and try to join other platoons
                    ROS_DEBUG_STREAM("The new leader " << msg.header.sender_id << " does not agree on the frontal join. Change back to leader state.");
                    pm_.current_platoon_state = PlatoonState::LEADER;
                }
            }
            else
            {
                ROS_DEBUG_STREAM("Ignore received response message because it is not for the current plan.");
            }
        }
        else
        {
            ROS_DEBUG_STREAM("Ignore received response message because we are not in any negotiation process.");
        }
    }

    // UCLA: response for candidate leader (inherited from leader waiting)
    void PlatoonStrategicIHPPlugin::mob_resp_cb_candidateleader(const cav_msgs::MobilityResponse& msg)
    {
    }

    // UCLA: response callback for lead with operation
    void PlatoonStrategicIHPPlugin::mob_resp_cb_leadwithoperation(const cav_msgs::MobilityResponse& msg)
    { 
    }

    // UCLA: response callback for prepare to join (inherited from leader waiting)
    void PlatoonStrategicIHPPlugin::mob_resp_cb_preparetojoin(const cav_msgs::MobilityResponse& msg)
    {
        /*
            If leader notify the member to slow down and ACK the request,
            start to check the gap and change lane when gap is large enough 
        */

        cav_msgs::PlanType plan_type = msg.plan_type;
        bool isCreatingGap = plan_type.type == cav_msgs::PlanType::PLATOON_CUTIN_JOIN;
        bool isFinishLaneChangeFront = plan_type.type == cav_msgs::PlanType::CUT_IN_FRONT_DONE; 
        // todo: add cut-in from middle and rear 
        // bool isFinishLaneChangeMid
        // bool isFinishLaneChangeRear

        // UCLA: Creat Gap
        if (msg.is_accepted && isCreatingGap)
        {
            // task 1: check gap 
            double joinerDtD = current_downtrack_;
            double cut_in_gap = pm_.getCutInGap(target_join_index_, joinerDtD);   
            ROS_DEBUG_STREAM("Start loop to check cut-in gap, start lane change when gap allows");
            while (cut_in_gap < config_.maxGap)  // todo: use max gap as "safe to cut-in" gap, may need to adjust change later
            {
                ROS_DEBUG_STREAM("The target gap is not safe for lane change, wait for a larger gap");
                cut_in_gap = pm_.getCutInGap(target_join_index_, joinerDtD);  

                // Sleep period equals statusMessageInterval_ 
                ros::Duration(statusMessageInterval_/1000).sleep();

                // Use LANE_CHANGE_TIMEOUT to bond the "creat gap"
                bool isCurrentPlanTimeout = ((ros::Time::now().toNSec()/1000000  - pm_.current_plan.planStartTime) > LANE_CHANGE_TIMEOUT);
                // Set the plan as invalid and break the loop once the open-gap exceeds timeout.
                if(isCurrentPlanTimeout) 
                {
                    ROS_DEBUG_STREAM("Give up current on waiting plan with planId: " << pm_.current_plan.planId);
                    pm_.current_plan.valid = false;
                    break;
                }  
            }
            
            // task 2: set indicator if gap is safe
            pm_.safeToLaneChange = true;

            // task 3: notify gap-rear vehicle to stop slowing down
            cav_msgs::MobilityRequest request;
            request.header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
            request.header.recipient_id = pm_.targetLeaderId;
            request.header.sender_id = config_.vehicleID;
            request.header.timestamp = ros::Time::now().toNSec() / 1000000;;
            // UCLA: A new plan type to stop creat gap.
            request.plan_type.type = cav_msgs::PlanType::STOP_CREATE_GAP;
            request.strategy = MOBILITY_STRATEGY;
            request.strategy_params = "";
            request.urgency = 50;
            request.location = pose_to_ecef(pose_msg_);
            // note: for rear join, cut-in index == platoon.size()-1; for join from front, index == -1
            //       for cut-in in middle, index indicate the gap leading vehicle's index
            request.join_index = target_join_index_;
            mobility_request_publisher_(request); 
            ROS_DEBUG_STREAM("Published Mobility Candidate-Join request to the leader");
            ROS_WARN("Published Mobility Candidate-Join request to the leader");

        }

        // UCLA: Revert to same-lane for cut-in front 
        if (msg.is_accepted && isFinishLaneChangeFront)
        {
            ROS_DEBUG_STREAM("Cut-in from front lane change finished, the joining vehicle revert to same-lane maneuver.");
            pm_.current_platoon_state = PlatoonState::CANDIDATELEADER;
        }

        // todo: Revert to same-lane operation for cut-in from middle/rear 
    }


    
    // ------ 5. response types ------- //

    // ACK --> yes,accept host as member; NACK --> no, cannot accept host as member
    void PlatoonStrategicIHPPlugin::mob_req_cb(const cav_msgs::MobilityRequest& msg)
    {
        // UCLA: read current request plan 
        cav_msgs::PlanType req_plan_type = msg.plan_type; 
        
        cav_msgs::MobilityResponse response;
        response.header.sender_id = config_.vehicleID;
        response.header.recipient_id = msg.header.sender_id;
        response.header.plan_id = pm_.currentPlatoonID;
        response.header.timestamp = ros::Time::now().toNSec() / 1000000;
        
        // UCLA: add plantype in response 
        response.plan_type.type = req_plan_type.type;
        
        MobilityRequestResponse req_response = handle_mob_req(msg);
        if (req_response == MobilityRequestResponse::ACK)
        {
            response.is_accepted = true;
            mobility_response_publisher_(response);
        }
        else if (req_response == MobilityRequestResponse::NACK)
        {
            response.is_accepted = false;
            mobility_response_publisher_(response);
        }
        else
        {
            ROS_DEBUG_STREAM(" NO response to mobility request. ");
        }
    }
    
    
   //------------------------------------------ FSM states --------------------------------------------------//

   /*
   *------- Additional IHP FSM here -------
   *1. Frontal Join
        * 1.1. Frontal join states
          |--[candidate leader]: Handle the new leader that joined from front. 
          |--[leader aborting]: Handle the process that old leader transition from leader to follower. 
        * 1.2 Additional Params 
          |-- [SAME_LANE_JOIN_PARAMS]
        * 1.3 Additional Plan Type
          |-- [JOIN_PLATOON_FROM_FRONT]
          |-- [PLATOON_FRONT_JOIN]
   
   * 2. TODO: Cut-in join states
   * 3. TODO: Consider adding superstates
   */
    
    void PlatoonStrategicIHPPlugin::run_leader_waiting()
    {
        ROS_DEBUG_STREAM("Run LeaderWaiting State ");
        long tsStart = ros::Time::now().toNSec() / 1000000;
        // Task 1
        if (tsStart - waitingStartTime > waitingStateTimeout * 1000)
        {
            //TODO if the current state timeouts, we need to have a kind of ABORT message to inform the applicant
            ROS_DEBUG_STREAM("LeaderWaitingState is timeout, changing back to PlatoonLeaderState.");
            pm_.current_platoon_state = PlatoonState::LEADER;
        }
        // Task 2
        cav_msgs::MobilityOperation status;
        status = composeMobilityOperationLeaderWaiting();
        mobility_operation_publisher_(status);
        ROS_DEBUG_STREAM("publish status message");
        long tsEnd = ros::Time::now().toNSec() / 1000000;
        long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
        ros::Duration(sleepDuration / 1000).sleep();
    }

    void PlatoonStrategicIHPPlugin::run_leader()
    {
        long tsStart = ros::Time::now().toNSec() / 1000000;
        // Task 1: heart beat timeout: constantly send INFO mob_op
        bool isTimeForHeartBeat = tsStart - lastHeartBeatTime >= infoMessageInterval_;
        ROS_DEBUG_STREAM("time since last heart beat: " << tsStart - lastHeartBeatTime);
        if (isTimeForHeartBeat) 
        {
            cav_msgs::MobilityOperation infoOperation;
            infoOperation = composeMobilityOperationLeader(OPERATION_INFO_TYPE);
            mobility_operation_publisher_(infoOperation);
            lastHeartBeatTime = ros::Time::now().toNSec() / 1000000;
            ROS_DEBUG_STREAM("Published heart beat platoon INFO mobility operatrion message");
        }
        // Task 2
        // if (isTimeForHeartBeat) 
        // {
        //     updateLightBar();
        // }
        // Task 3: plan time out, check if current plan is still valid (i.e., not timed out).
        if (pm_.current_plan.valid)
        {
            bool isCurrentPlanTimeout = ((ros::Time::now().toNSec() / 1000000 - pm_.current_plan.planStartTime) > NEGOTIATION_TIMEOUT);
            if (isCurrentPlanTimeout)
            {
                ROS_DEBUG_STREAM("Give up current on waiting plan with planId: " << pm_.current_plan.planId);
                pm_.current_plan.valid = false;
            }
        }

        // Task 4: STATUS msgs
        bool hasFollower = (pm_.getTotalPlatooningSize() > 1);
        // if has follower, publish platoon message as STATUS mob_op
        if (hasFollower) 
        {
            cav_msgs::MobilityOperation statusOperation;
            statusOperation = composeMobilityOperationLeader(OPERATION_STATUS_TYPE);
            // mob_op_pub_.publish(statusOperation);
            mobility_operation_publisher_(statusOperation);
            ROS_DEBUG_STREAM("Published platoon STATUS operation message");
        }
        long tsEnd = ros::Time::now().toNSec() / 1000000;
        long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
        ros::Duration(sleepDuration / 1000).sleep();
    }

    void PlatoonStrategicIHPPlugin::run_follower()
    {
        // This is a interrupted-safe loop.
        // This loop has four tasks:
        // 1. Check the state start time, if it exceeds a limit it will give up current plan and change back to leader state
        // 2. Abort current request if we wait for long enough time for response from leader and change back to leader state
        
        long tsStart = ros::Time::now().toNSec() / 1000000;
        // Job 1
        cav_msgs::MobilityOperation status;
        status = composeMobilityOperationFollower();
        mobility_operation_publisher_(status);
        // Job 2
        // Get the number of vehicles in this platoon who is in front of us
        int vehicleInFront = pm_.getNumberOfVehicleInFront();
        if (vehicleInFront == 0) 
        {
            noLeaderUpdatesCounter++;
            if (noLeaderUpdatesCounter >= LEADER_TIMEOUT_COUNTER_LIMIT) 
            {
                ROS_DEBUG_STREAM("noLeaderUpdatesCounter = " << noLeaderUpdatesCounter << " and change to leader state");
                pm_.changeFromFollowerToLeader();
                pm_.current_platoon_state = PlatoonState::LEADER;
            }
        }
        else 
        {
            // reset counter to zero when we get updates again
            noLeaderUpdatesCounter = 0;
        }
        long tsEnd = ros::Time::now().toNSec() / 1000000;
        long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
        ros::Duration(sleepDuration / 1000).sleep();
    }

    void PlatoonStrategicIHPPlugin::run_candidate_follower()
    {
        long tsStart = ros::Time::now().toNSec() / 1000000;
        // Task 1: state timeout
        bool isCurrentStateTimeout = (tsStart - candidatestateStartTime) > waitingStateTimeout * 1000;
        ROS_DEBUG_STREAM("timeout1: " << tsStart - candidatestateStartTime);
        ROS_DEBUG_STREAM("waitingStateTimeout: " << waitingStateTimeout * 1000);
        if (isCurrentStateTimeout) 
        {
            ROS_DEBUG_STREAM("The current candidate follower state is timeout. Change back to leader state.");
            pm_.current_platoon_state = PlatoonState::LEADER;
        }

        // Task 2: plan timeout, check if current plan is still valid (i.e., not timed out).   
        if (pm_.current_plan.valid) 
        {
                
            ROS_DEBUG_STREAM("pm_.current_plan.planStartTime: " << pm_.current_plan.planStartTime);
            ROS_DEBUG_STREAM("timeout2: " << tsStart - pm_.current_plan.planStartTime);
            ROS_DEBUG_STREAM("NEGOTIATION_TIMEOUT: " << NEGOTIATION_TIMEOUT);
            bool isPlanTimeout = (tsStart - pm_.current_plan.planStartTime) > NEGOTIATION_TIMEOUT;
            if (isPlanTimeout) 
            {
                pm_.current_plan.valid = false;
                ROS_DEBUG_STREAM("The current plan did not receive any response. Abort and change to leader state.");
                pm_.current_platoon_state = PlatoonState::LEADER;
                ROS_DEBUG_STREAM("Changed the state back to Leader");
            }
        }

        // Task 3: update plan calculate gap, update plan: PLATOON_FOLLOWER_JOIN with new gap
        double desiredJoinGap2 = config_.desiredJoinTimeGap * current_speed_;
        double maxJoinGap = std::max(config_.desiredJoinGap, desiredJoinGap2);
        double currentGap = pm_.getDistanceToPredVehicle();
        ROS_DEBUG_STREAM("Based on desired join time gap, the desired join distance gap is " << desiredJoinGap2 << " ms");
        ROS_DEBUG_STREAM("Since we have max allowed gap as " << config_.desiredJoinGap << " m then max join gap became " << maxJoinGap << " m");
        ROS_DEBUG_STREAM("The current gap from radar is " << currentGap << " m");
        if (currentGap <= maxJoinGap && pm_.current_plan.valid == false)
        {
            cav_msgs::MobilityRequest request;
            std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
            long currentTime = ros::Time::now().toNSec() / 1000000;
            request.header.plan_id = planId;
            request.header.recipient_id = pm_.targetLeaderId;
            request.header.sender_id = config_.vehicleID;
            request.header.timestamp = currentTime;

            request.plan_type.type = cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN;
            request.strategy = MOBILITY_STRATEGY;
            request.strategy_params = "";
            request.urgency = 50;
            request.location = pose_to_ecef(pose_msg_);

            mobility_request_publisher_(request);
            ROS_DEBUG_STREAM("Published Mobility Candidate-Join request to the leader");
            ROS_WARN("Published Mobility Candidate-Join request to the leader");
            PlatoonPlan* new_plan = new PlatoonPlan(true, currentTime, planId, pm_.targetLeaderId);

            pm_.current_plan = *new_plan;
        }

        //Task 4: publish platoon status message (as single joiner)
        if (pm_.getTotalPlatooningSize() > 1) 
        {
            cav_msgs::MobilityOperation status;
            status = composeMobilityOperationCandidateFollower();
            mobility_operation_publisher_(status);
            ROS_DEBUG_STREAM("Published platoon STATUS operation message");
        }
        long tsEnd = ros::Time::now().toNSec() / 1000000;
        long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
        ros::Duration(sleepDuration / 1000).sleep();
    }

    // UCLA: forntal join state (inherit from candidate follwoer: prepare to give up leading state and accept the new leader)
    void PlatoonStrategicIHPPlugin::run_leader_aborting() 
    {
        /*  
            UCLA implementation note:
            1. this function  send step plan type: "PLATOON_FRONT_JOIN"
            2. the sender of the plan is the previous leader (old leader), it wil prepare to follow front joiner (new leader)
        */
        long tsStart = ros::Time::now().toNSec() / 1000000;
        // Task 1: state timeout
        bool isCurrentStateTimeout = (tsStart - candidatestateStartTime) > waitingStateTimeout * 1000;
        ROS_DEBUG_STREAM("timeout1: " << tsStart - candidatestateStartTime);
        ROS_DEBUG_STREAM("waitingStateTimeout: " << waitingStateTimeout * 1000);
        if (isCurrentStateTimeout) 
        {
            ROS_DEBUG_STREAM("The current leader aborting state is timeout. Change back to leader state.");
            pm_.current_platoon_state = PlatoonState::LEADER;
        }

        // Task 2: plan timeout, check if current plan is still valid (i.e., not timed out).
        if (pm_.current_plan.valid) 
        {
            ROS_DEBUG_STREAM("pm_.current_plan.planStartTime: " << pm_.current_plan.planStartTime);
            ROS_DEBUG_STREAM("timeout2: " << tsStart - pm_.current_plan.planStartTime);
            ROS_DEBUG_STREAM("NEGOTIATION_TIMEOUT: " << NEGOTIATION_TIMEOUT);
            bool isPlanTimeout = (tsStart - pm_.current_plan.planStartTime) > NEGOTIATION_TIMEOUT;
            if (isPlanTimeout) 
            {
                pm_.current_plan.valid = false;
                ROS_DEBUG_STREAM("The current plan did not receive any response. Abort and change to leader state.");
                pm_.current_platoon_state = PlatoonState::LEADER;
                ROS_DEBUG_STREAM("Changed the state back to Leader");

                // End the method if time out.
                return;
            } 
        }

        // Task 3: update plan: PLATOON_FRONT_JOIN with new gap
        double desiredJoinGap2 = config_.desiredJoinTimeGap * current_speed_;
        double maxJoinGap = std::max(config_.desiredJoinGap, desiredJoinGap2);
        // check if compatible for front join --> return front gap, no veh type check, is compatible
        double currentGap = pm_.getDistanceToPredVehicle();
        ROS_DEBUG_STREAM("Based on desired join time gap, the desired join distance gap is " << desiredJoinGap2 << " ms");
        ROS_DEBUG_STREAM("Since we have max allowed gap as " << config_.desiredJoinGap << " m then max join gap became " << maxJoinGap << " m");
        ROS_DEBUG_STREAM("The current gap from radar is " << currentGap << " m");
        
        // Check if gap is big enough and if current plan is timeout.
        if (currentGap <= maxJoinGap && pm_.current_plan.valid == false)
        {
            // compose frontal joining plan, senderID is the old leader 
            cav_msgs::MobilityRequest request;
            std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
            long currentTime = ros::Time::now().toNSec() / 1000000;
            request.header.plan_id = planId;
            request.header.recipient_id = pm_.targetLeaderId;
            request.header.sender_id = config_.vehicleID;
            request.header.timestamp = currentTime;
            // UCLA added for cut-in, not used for other cases
            request.join_index = -1; // Note: not used by same-lane functions.

            // assign a new plan type 
            request.plan_type.type = cav_msgs::PlanType::PLATOON_FRONT_JOIN;
            request.strategy = MOBILITY_STRATEGY;
            request.strategy_params = "";
            request.urgency = 50;
            request.location = pose_to_ecef(pose_msg_);
            mobility_request_publisher_(request);
            ROS_DEBUG_STREAM("Published Mobility Candidate-Join request to the leader");
            ROS_WARN("Published Mobility Candidate-Join request to the leader");
            PlatoonPlan* new_plan = new PlatoonPlan(true, currentTime, planId, pm_.targetLeaderId);

            pm_.current_plan = *new_plan;
        }

        //Task 4: publish platoon status message (as single joiner)
        if (pm_.getTotalPlatooningSize() > 1) 
        {
            cav_msgs::MobilityOperation status;
            // call compose mshs function
            status = composeMobilityOperationLeaderAborting();
            mobility_operation_publisher_(status);
            ROS_DEBUG_STREAM("Published platoon STATUS operation message");
        }
        long tsEnd = ros::Time::now().toNSec() / 1000000;
        long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
        ros::Duration(sleepDuration / 1000).sleep();
    }

    // UCLA: frontal join state (inherited from leader waiting: prepare to join as th new leader)
    void PlatoonStrategicIHPPlugin::run_candidate_leader()
    {
        ROS_DEBUG_STREAM("Run Candidate Leader State ");
        long tsStart = ros::Time::now().toNSec() / 1000000;
        // Task 1: State time out
        if (tsStart - waitingStartTime > waitingStateTimeout * 1000)
        {
            //TODO if the current state timeouts, we need to have a kind of ABORT message to inform the applicant
            ROS_DEBUG_STREAM("CandidateLeader state is timeout, changing back to PlatoonLeaderState.");
            pm_.current_platoon_state = PlatoonState::LEADER;
        }
        // Task 2: publish status message
        cav_msgs::MobilityOperation status;
        status = composeMobilityOperationCandidateLeader();
        mobility_operation_publisher_(status);
        ROS_DEBUG_STREAM("publish status message");
        long tsEnd = ros::Time::now().toNSec() / 1000000;
        long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
        ros::Duration(sleepDuration / 1000).sleep();
    }

    // UCLA: add leading with operation state for cut-in join platoon leader
    void PlatoonStrategicIHPPlugin::run_lead_with_operation()
    {
        long tsStart = ros::Time::now().toNSec() / 1000000;
        // Task 1: heart beat timeout: constantly send INFO mob_op
        bool isTimeForHeartBeat = tsStart - lastHeartBeatTime >= infoMessageInterval_;
        ROS_DEBUG_STREAM("time since last heart beat: " << tsStart - lastHeartBeatTime);
        if (isTimeForHeartBeat) 
        {
            cav_msgs::MobilityOperation infoOperation;
            infoOperation = composeMobilityOperationLeadWithOperation(OPERATION_INFO_TYPE);
            mobility_operation_publisher_(infoOperation);
            lastHeartBeatTime = ros::Time::now().toNSec() / 1000000;
            ROS_DEBUG_STREAM("Published heart beat platoon INFO mobility operatrion message");
        }
        // Task 2
        // if (isTimeForHeartBeat) 
        // {
        //     updateLightBar();
        // }
        // Task 3: plan time out
        if (pm_.current_plan.valid)
        {
            bool isCurrentPlanTimeout = ((ros::Time::now().toNSec() / 1000000 - pm_.current_plan.planStartTime) > NEGOTIATION_TIMEOUT);
            if (isCurrentPlanTimeout)
            {
                ROS_DEBUG_STREAM("Give up current on waiting plan with planId: " << pm_.current_plan.planId);
                pm_.current_plan.valid = false;
            }
        }

        // Task 4: STATUS msgs
        bool hasFollower = (pm_.getTotalPlatooningSize() > 1);
        // if has follower, publish platoon message as STATUS mob_op
        if (hasFollower) 
        {
            cav_msgs::MobilityOperation statusOperation;
            statusOperation = composeMobilityOperationLeadWithOperation(OPERATION_STATUS_TYPE);
            // mob_op_pub_.publish(statusOperation);
            mobility_operation_publisher_(statusOperation);
            ROS_DEBUG_STREAM("Published platoon STATUS operation message");
        }
        long tsEnd = ros::Time::now().toNSec() / 1000000;
        long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
        ros::Duration(sleepDuration / 1000).sleep();
    }

    // UCLA: add prepare to join state for cut-in joining vehicle 
    void PlatoonStrategicIHPPlugin::run_prepare_to_join()
    {   
        /*
        * The prepare join state should have the following tasks: 
        *   1. Compose mobility operation param: status.
        *   2. Time out check.
        *   3. Calculate proper cut_in index 
        *   4. Send out lane change intend to leader.
        *   Note: 1. pm_.safetolanechange monitors the gap condition.
        *         2. Once it is safe to lane change, the updated plan will send-out in "plan_maneuver_cb"
        */

        // Task 1: compose mobility operation (status)
        cav_msgs::MobilityOperation status;
        status = composeMobilityOperationPrepareToJoin();

        long tsStart = ros::Time::now().toNSec() / 1000000;
        // Task 2.1: state timeout
        bool isCurrentStateTimeout = (tsStart - candidatestateStartTime) > waitingStateTimeout * 1000;
        ROS_DEBUG_STREAM("timeout1: " << tsStart - candidatestateStartTime);
        ROS_DEBUG_STREAM("waitingStateTimeout: " << waitingStateTimeout * 1000);
        if (isCurrentStateTimeout) 
        {
            ROS_DEBUG_STREAM("The current prepare to join state is timeout. Change back to leader state. The member variable joiningID_ is reset.");
            joiningID_ = "";
            pm_.current_platoon_state = PlatoonState::LEADER;
        }

        // Task 2.2: plan timeout
        if (pm_.current_plan.valid) 
        {
            ROS_DEBUG_STREAM("pm_.current_plan.planStartTime: " << pm_.current_plan.planStartTime);
            ROS_DEBUG_STREAM("timeout2: " << tsStart - pm_.current_plan.planStartTime);
            ROS_DEBUG_STREAM("NEGOTIATION_TIMEOUT: " << NEGOTIATION_TIMEOUT);
            bool isPlanTimeout = (tsStart - pm_.current_plan.planStartTime) > NEGOTIATION_TIMEOUT;
            if (isPlanTimeout) 
            {
                pm_.current_plan.valid = false;
                joiningID_ = "";
                ROS_DEBUG_STREAM("The current plan did not receive any response. Abort and change to leader state. The member variable joiningID_ is reset");
                pm_.current_platoon_state = PlatoonState::LEADER;
                ROS_DEBUG_STREAM("Changed the state back to Leader");
            }
        }

        // Task 3: Calculate proper cut_in index 
        // Note: The cut-in index is zero-based and points to the gap-leading vehicle's index. For cut-in from front, the join index = -1.
        double joinerDtD = current_downtrack_;
        target_join_index_ = pm_.getClosestIndex(joinerDtD);

        // Task 4: Send out request to leader about cut-in position
        cav_msgs::MobilityRequest request;
        std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
        long currentTime = ros::Time::now().toNSec() / 1000000;
        request.header.plan_id = planId;
        request.header.recipient_id = pm_.targetLeaderId;
        request.header.sender_id = config_.vehicleID;
        request.header.timestamp = currentTime;
        // UCLA: assign a new plan type
        request.plan_type.type = cav_msgs::PlanType::PLATOON_CUTIN_JOIN;
        request.strategy = MOBILITY_STRATEGY;
        request.strategy_params = "";
        request.urgency = 50;
        request.location = pose_to_ecef(pose_msg_);
        
        /** note: The cut-in index is zero-based and points to the gap-leading vehicle's index. 
         *  eg:  for rear join, cut-in index == platoon.size()-1; 
         *       for join from front, index == -1;
         *       for cut-in in middle, index indicate the gap leading vehicle's index.
         */      
        request.join_index = target_join_index_;
        
        mobility_request_publisher_(request); 
        ROS_DEBUG_STREAM("Published Mobility cut-in join request to the leader");
        ROS_WARN("Published Mobility cut-in join request to the leader");

    }

    //------------------------------------------- main functions for platoon plugin --------------------------------------------//
    
    // Platoon on spin
    bool PlatoonStrategicIHPPlugin::onSpin() 
    {
        plugin_discovery_publisher_(plugin_discovery_msg_);
        
        if (pm_.current_platoon_state == PlatoonState::LEADER)
        {
            run_leader();
        }
        else if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
        {
            run_follower();
        }
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATEFOLLOWER)
        {
            run_candidate_follower();
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADERWAITING)
        {
            run_leader_waiting();
        }
        // UCLA: added for frontal join
        else if (pm_.current_platoon_state == PlatoonState::LEADERABORTING)
        {
            run_leader_aborting();
        }
        // UCLA: added for frontal join
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATELEADER)
        {
            run_candidate_leader();
        }
        // UCLA: added lead with operationfor CUT-IN join
        else if (pm_.current_platoon_state == PlatoonState::LEADWITHOPERATION)
        {
            run_lead_with_operation();
        }
        // UCLA: added prepare to join for CUT-IN join
        else if (pm_.current_platoon_state == PlatoonState::PREPARETOJOIN)
        {
            run_prepare_to_join();
        }

        cav_msgs::PlatooningInfo platoon_status = composePlatoonInfoMsg();
        platooning_info_publisher_(platoon_status);

        return true;
    }

    // ------- Generate manuver plan (Service Callback) ------- //
    
    // compose maneuver message 
    cav_msgs::Maneuver PlatoonStrategicIHPPlugin::composeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time& current_time)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::PLATOONING;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "PlatooningTacticalPlugin";
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "PlatooningStrategicIHPPlugin";
        maneuver_msg.lane_following_maneuver.start_dist = current_dist;
        maneuver_msg.lane_following_maneuver.start_speed = current_speed;
        maneuver_msg.lane_following_maneuver.start_time = current_time;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        double cur_plus_target = current_speed + target_speed;
        if (cur_plus_target < 0.00001) 
        {
            maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration(config_.time_step);
        } 
        else 
        {
            maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration((end_dist - current_dist) / (0.5 * cur_plus_target));
        }
        maneuver_msg.lane_following_maneuver.lane_ids = { std::to_string(lane_id) };
        current_time = maneuver_msg.lane_following_maneuver.end_time;
        ROS_DEBUG_STREAM("Creating lane follow start dist:"<<current_dist<<" end dist:"<<end_dist);
        ROS_DEBUG_STREAM("Duration:"<< maneuver_msg.lane_following_maneuver.end_time.toSec() - maneuver_msg.lane_following_maneuver.start_time.toSec());
        return maneuver_msg;
    }
    
    // UCLA: compose maneuver message for lane change 
    cav_msgs::Maneuver PlatoonStrategicIHPPlugin::composeLaneChangeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int starting_lane_id, int ending_lane_id, ros::Time& current_time)
    {
        cav_msgs::Maneuver maneuver_msg;
        // UCLA: change to lane change maneuvers
        maneuver_msg.type = cav_msgs::Maneuver::LANE_CHANGE;
        maneuver_msg.lane_change_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::PLATOONING;
        maneuver_msg.lane_change_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_change_maneuver.parameters.planning_tactical_plugin = "PlatooningTacticalPlugin";
        maneuver_msg.lane_change_maneuver.parameters.planning_strategic_plugin = "PlatooningStrategicIHPPlugin";
        maneuver_msg.lane_change_maneuver.start_dist = current_dist;
        maneuver_msg.lane_change_maneuver.start_speed = current_speed;
        maneuver_msg.lane_change_maneuver.start_time = current_time;
        maneuver_msg.lane_change_maneuver.end_dist = end_dist;
        maneuver_msg.lane_change_maneuver.end_speed = target_speed;        
        // Generate a new maneuver ID for the lane change maneuver (not needed for lane following maneuver).
        maneuver_msg.lane_following_maneuver.parameters.maneuver_id = boost::uuids::to_string(boost::uuids::random_generator()());
        
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        double cur_plus_target = current_speed + target_speed;
        if (cur_plus_target < 0.00001) 
        {
            maneuver_msg.lane_change_maneuver.end_time = current_time + ros::Duration(config_.time_step);
        } 
        else 
        {
            maneuver_msg.lane_change_maneuver.end_time = current_time + ros::Duration((end_dist - current_dist) / (0.5 * cur_plus_target));
        }

        // UCLA: need both start laneID and end laneID  for lane change
        maneuver_msg.lane_change_maneuver.starting_lane_id = { std::to_string(starting_lane_id) };
        maneuver_msg.lane_change_maneuver.ending_lane_id = { std::to_string(ending_lane_id) };

        current_time = maneuver_msg.lane_change_maneuver.end_time;
        ROS_DEBUG_STREAM("Creating lane follow start dist:"<<current_dist<<" end dist:"<<end_dist);
        ROS_DEBUG_STREAM("Duration:"<< maneuver_msg.lane_change_maneuver.end_time.toSec() - maneuver_msg.lane_change_maneuver.start_time.toSec());
        return maneuver_msg;
    }

    // update current status based on maneuver 
    void PlatoonStrategicIHPPlugin::updateCurrentStatus(cav_msgs::Maneuver maneuver, double& speed, double& current_progress, int& lane_id)
    {
        if(maneuver.type == cav_msgs::Maneuver::LANE_FOLLOWING){
            speed =  maneuver.lane_following_maneuver.end_speed;
            current_progress =  maneuver.lane_following_maneuver.end_dist;
            if (maneuver.lane_following_maneuver.lane_ids.empty()) 
            {
                ROS_WARN_STREAM("Lane id of lane following maneuver not set. Using 0");
                lane_id = 0;
            } 
            else 
            {
                lane_id =  stoi(maneuver.lane_following_maneuver.lane_ids[0]);
            }
        }
    }
    
    // manuver plan callback (provide cav_srvs for arbitrator) 
    bool PlatoonStrategicIHPPlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp)
    {

        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);

        // *** get the actually closest lanelets that relate to current location (n=10) ***//
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 10);  
        if(current_lanelets.size() == 0)
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return true;
        }
        
        auto shortest_path = wm_->getRoute()->shortestPath(); // find path amoung route

        lanelet::ConstLanelet current_lanelet;
        int last_lanelet_index = -1;
        for (auto llt : current_lanelets)
        {
            if (boost::geometry::within(current_loc, llt.second.polygon2d())) 
            {
                int potential_index = findLaneletIndexFromPath(llt.second.id(), shortest_path); // usage: findLaneletIndexFromPath(target_id, lanelet2_path)
                if (potential_index != -1)
                {
                    last_lanelet_index = potential_index;
                    current_lanelet = shortest_path[last_lanelet_index]; // find lanelet2 from map that corresponse to the path
                    break;
                }
            }
        }
        if(last_lanelet_index == -1)
        {
            ROS_ERROR_STREAM("Current position is not on the shortest path! Returning an empty maneuver");
            return true;
        }
        double current_progress = wm_->routeTrackPos(current_loc).downtrack;
        double speed_progress = current_speed_;
        ros::Time time_progress = ros::Time::now();

        // ---------------- use IHP platoon trajectory regulation here --------------------
        double total_maneuver_length;
        double target_speed;
        // 1. determine if follower 
        if (pm_.isFollower)
        {
            // 2. Use IHP platoon trajectory regulation for followers.
            double dt = config_.time_step;
            total_maneuver_length = pm_.getIHPDesPosFollower(dt);
            target_speed = (total_maneuver_length - current_progress) / dt;
        }
        else
        {
            // 3. Use CARMA planning method (control_length = step_time * speed limit).
            target_speed = findSpeedLimit(current_lanelet);   //get Speed Limit TOTO update
            total_maneuver_length = current_progress + config_.time_step * target_speed;
        }
        // ----------------------------------------------------------------

        // UCLA: add create gap indicator, if true, use smaller target speed to create gap
        if (pm_.isCreateGap)
        {
            target_speed = target_speed * 0.75;  // todo --> use 0.75 for demo, include this in config later
        }
        
        // pick smaller length, accomendate when host is close to the route end
        double route_length =  wm_->getRouteEndTrackPos().downtrack; 
        total_maneuver_length = std::min(total_maneuver_length, route_length);

        // Update current status based on prior plan
        if(req.prior_plan.maneuvers.size()!=0)
        {
            time_progress = req.prior_plan.planning_completion_time;
            int end_lanelet =0;
            updateCurrentStatus(req.prior_plan.maneuvers.back(),speed_progress,current_progress,end_lanelet);
            last_lanelet_index = findLaneletIndexFromPath(end_lanelet,shortest_path);
        }
        bool approaching_route_end = false;
        double time_req_to_stop,stopping_dist;

        ROS_DEBUG_STREAM("Starting Loop");
        ROS_DEBUG_STREAM("total_maneuver_length: " << total_maneuver_length << " route_length: " << route_length);
        
        // lane change maneuver 
        if (pm_.safeToLaneChange)
        {   
            // for testing purpose only, check lane change status
            double start_crosstrack = 0.5*config_.laneWidth; // Assume vehicle start at left lane when testing.
            double crosstrackDiff = current_crosstrack_ - start_crosstrack; 
            bool isLaneChangeFinished = crosstrackDiff >= config_.laneWidth*0.85; // Use 85% of lane width to account for noise.
             
            /**  
             * Note: The function "find_target_lanelet_id" was used to test the IHP platooning logic and is only a pre-written scenario. 
             *       We re-use the existing lane change maneuver for route-following, and followed the data flow just to compile the code. 
             * 
             * TODO: The IHP2 platooning should provide necessary data in a maneuver plan for the arbitrary lane change module.
             *       This means this entire block need to be updated accordingly. 
             */ 
            
            // lane change not finished, use lane change plan
            if(!isLaneChangeFinished)  
            {
                // send out lane change plan
                while(current_progress < total_maneuver_length)
                {   
                    ROS_DEBUG_STREAM("Lane Change Maneuver for Cut-in join ! ");
                    ROS_DEBUG_STREAM("Lanlet: " << shortest_path[last_lanelet_index].id());
                    ROS_DEBUG_STREAM("current_progress: "<< current_progress);
                    ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
                    ROS_DEBUG_STREAM("target_speed: " << target_speed);
                    ROS_DEBUG_STREAM("time_progress: " << time_progress.toSec());

                    // ----------------------- UCLA: consider change according to maneuver plan requirement --------------------
                    auto p = shortest_path[last_lanelet_index].centerline2d().back(); // change to lane change path
                    // set to next lane destination, consider sending ecef instead of dtd 
                    double end_dist = wm_->routeTrackPos(shortest_path[last_lanelet_index].centerline2d().back()).downtrack;
                    end_dist = std::min(end_dist, total_maneuver_length);
                    ROS_DEBUG_STREAM("end_dist: " << end_dist);
                    // consider calculate dtd_diff and ctd_diff
                    double dist_diff = end_dist - current_progress;
                    ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
                    // UCLA: start and end lane IDs
                    int current_lanelet_id = shortest_path[last_lanelet_index].id();

                    // note: This is just mock info to compile the code. 
                    // TODO: This should be updated according to arbitrary lane change requirements. 
                    int target_lanelet_id = current_lanelet_id;

                    // ----------------------------------------------------------------------------------------------------------
                    if(end_dist < current_progress)
                    {
                        break;
                    }
                    // Update lane change status to stop the while loop when langchange finshed.
                    crosstrackDiff = current_crosstrack_ - start_crosstrack;  // Assume vehicle start at left lane when testing.
                    if (crosstrackDiff >= config_.laneWidth*0.85) // Use 85% of lane width to account for noise.
                    {
                        break; 
                    }
                    // "composeManeuverMessage" --> maneuver = lanefollowng, need to add a "lane changing" --> maneuver = lane change
                    resp.new_plan.maneuvers.push_back(composeLaneChangeManeuverMessage(current_progress, end_dist,  
                                            speed_progress, target_speed, current_lanelet_id, target_lanelet_id, time_progress));

                    current_progress += dist_diff;
                    // read lane change maneuver end time as time progress
                    time_progress = resp.new_plan.maneuvers.back().lane_change_maneuver.end_time;
                    speed_progress = target_speed;
                    if(current_progress >= total_maneuver_length || last_lanelet_index == shortest_path.size() - 1)
                    {
                        break;
                    }
                    ++last_lanelet_index;
                }

            }

            // lane change finished, use lane following plan
            else
            {
                // send out lane following plan
                while(current_progress < total_maneuver_length)
                {   
                    ROS_DEBUG_STREAM("Same Lane Maneuver for platoon join ! ");
                    ROS_DEBUG_STREAM("Lanlet: " << shortest_path[last_lanelet_index].id());
                    ROS_DEBUG_STREAM("current_progress: "<< current_progress);
                    ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
                    ROS_DEBUG_STREAM("target_speed: " << target_speed);
                    ROS_DEBUG_STREAM("time_progress: " << time_progress.toSec());
                    auto p = shortest_path[last_lanelet_index].centerline2d().back();
                    double end_dist = wm_->routeTrackPos(shortest_path[last_lanelet_index].centerline2d().back()).downtrack;
                    end_dist = std::min(end_dist, total_maneuver_length);
                    ROS_DEBUG_STREAM("end_dist: " << end_dist);
                    double dist_diff = end_dist - current_progress;
                    ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
                    if(end_dist < current_progress)
                    {
                        break;
                    }
                    // Note: The previous plan was generated at the beginning of the trip. It is necessary to update 
                    //       it as the lane ID and lanelet Index are different.
                    resp.new_plan.maneuvers.push_back(composeManeuverMessage(current_progress, end_dist,  
                                            speed_progress, target_speed,shortest_path[last_lanelet_index].id(), time_progress));

                    
                    current_progress += dist_diff;
                    time_progress = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
                    speed_progress = target_speed;
                    if(current_progress >= total_maneuver_length || last_lanelet_index == shortest_path.size() - 1)
                    {
                        break;
                    }
                    ++last_lanelet_index;
                }
            }
        }
        
        // same-lane maneuver  
        else 
        {
            while(current_progress < total_maneuver_length)
            {   
                ROS_DEBUG_STREAM("Same Lane Maneuver for platoon join ! ");
                ROS_DEBUG_STREAM("Lanlet: " << shortest_path[last_lanelet_index].id());
                ROS_DEBUG_STREAM("current_progress: "<< current_progress);
                ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
                ROS_DEBUG_STREAM("target_speed: " << target_speed);
                ROS_DEBUG_STREAM("time_progress: " << time_progress.toSec());
                auto p = shortest_path[last_lanelet_index].centerline2d().back();
                double end_dist = wm_->routeTrackPos(shortest_path[last_lanelet_index].centerline2d().back()).downtrack;
                end_dist = std::min(end_dist, total_maneuver_length);
                ROS_DEBUG_STREAM("end_dist: " << end_dist);
                double dist_diff = end_dist - current_progress;
                ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
                if(end_dist < current_progress)
                {
                    break;
                }

                resp.new_plan.maneuvers.push_back(composeManeuverMessage(current_progress, end_dist,  
                                        speed_progress, target_speed,shortest_path[last_lanelet_index].id(), time_progress));

                
                current_progress += dist_diff;
                time_progress = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
                speed_progress = target_speed;
                if(current_progress >= total_maneuver_length || last_lanelet_index == shortest_path.size() - 1)
                {
                    break;
                }
                ++last_lanelet_index;
            }
        }

        if(resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
        }  

        if (pm_.getTotalPlatooningSize() < 2)
        {
            resp.new_plan.maneuvers = {};
            ROS_WARN_STREAM("Platoon size 1 so Empty maneuver sent");
        }

        if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            pm_.current_platoon_state = PlatoonState::LEADER;
            pm_.currentPlatoonID = boost::uuids::to_string(boost::uuids::random_generator()());
            ROS_DEBUG_STREAM("change the state from standby to leader at start-up");
        }

        pm_.current_downtrack_distance_ = current_downtrack_;
        pm_.HostMobilityId = config_.vehicleID;
        ROS_DEBUG_STREAM("current_downtrack: " << current_downtrack_);
        
        return true;
    }
    //--------------------------------------------------------------------------//

}
