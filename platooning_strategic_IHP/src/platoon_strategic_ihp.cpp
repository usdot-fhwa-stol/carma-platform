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
   : plugin_discovery_publisher_(plugin_discovery_publisher), mobility_request_publisher_(mobility_request_publisher), 
      mobility_response_publisher_(mobility_response_publisher), mobility_operation_publisher_(mobility_operation_publisher), 
      platooning_info_publisher_(platooning_info_publisher), wm_(wm), config_(config)
    {
        ROS_DEBUG_STREAM("Top of PlatoonStrategicIHP ctor.");
        std::string hostStaticId = config_.vehicleID; //static ID for this vehicle
        pm_.HostMobilityId = hostStaticId;
        
        // construct platoon member for host vehicle as the first element in the vector, since it starts life as a solo vehicle
        long cur_t = ros::Time::now().toNSec()/1000000; // time in millisecond
        PlatoonMember hostVehicleMember = PlatoonMember(hostStaticId, 0.0, 0.0, 0.0, 0.0, cur_t); 
        pm_.host_platoon_.push_back(hostVehicleMember);

        plugin_discovery_msg_.name = "PlatooningStrategicIHPPlugin";
        plugin_discovery_msg_.version_id = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";
        ROS_DEBUG_STREAM("ctor complete. hostStaticId = " << hostStaticId);
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

        // note: ecef point read from map projector is in m.
        lanelet::BasicPoint3d ecef_point = map_projector_->projectECEF({pose_msg.pose.position.x, pose_msg.pose.position.y, 0.0}, 1);
        location.ecef_x = ecef_point.x() * 100.0;
        location.ecef_y = ecef_point.y() * 100.0;
        location.ecef_z = ecef_point.z() * 100.0;    
        

        // ROS_DEBUG_STREAM("location.ecef_x: " << location.ecef_x);
        // ROS_DEBUG_STREAM("location.ecef_y: " << location.ecef_y);
        // ROS_DEBUG_STREAM("location.ecef_z: " << location.ecef_z);

        // note: the returned ecef is in cm.
        return location;
    }

    // Function to assign host pose_ecef_point_
    void PlatoonStrategicIHPPlugin::setHostECEF(cav_msgs::LocationECEF pose_ecef_point)
    {
        // Note, the ecef here is in cm. 
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

    // Update platoon list (Unit Test function)
    void PlatoonStrategicIHPPlugin::updatePlatoonList(std::vector<PlatoonMember> platoon_list)
    {
        pm_.host_platoon_ = platoon_list;
    }

    // Callback to calculate downtrack based on pose message.
    void PlatoonStrategicIHPPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());

        if (pm_.current_platoon_state != PlatoonState::STANDBY)
        {
            lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
            carma_wm::TrackPos tc = wm_->routeTrackPos(current_loc);

            // update host's DtD and CtD
            current_downtrack_ = tc.downtrack;
            current_crosstrack_ = tc.crosstrack;
            // ROS_DEBUG_STREAM("current_downtrack_ = " << current_downtrack_ << ", current_crosstrack_ = " << current_crosstrack_);
            pm_.updateHostPose(current_downtrack_, current_crosstrack_);

            // note: the ecef read from "pose_ecef_point" is in cm.
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
        if (current_speed_ < STOPPED_SPEED)
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

    // Find the lane width based on current location 
    double PlatoonStrategicIHPPlugin::findLaneWidth()
    {
        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);

        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 10);
        lanelet::ConstLanelet current_lanelet = current_lanelets[0].second;

        // find left and right bound 
        lanelet::ConstLineString3d left_bound = current_lanelet.leftBound();
        lanelet::ConstLineString3d right_bound = current_lanelet.leftBound();

        // find lane width 
        double dx = abs (left_bound[0].x() - right_bound[0].x());
        double dy = abs (left_bound[0].y() - right_bound[0].y());
        double laneWidth = sqrt(dx*dx + dy*dy);
        ROS_DEBUG_STREAM("calculated lane width: " << laneWidth);

        // TODO temporary disable this function and return constant value
        laneWidth = 3.5;
        

        return laneWidth;
    }
    
    // Check if target platoon is in front of the host vehicle, and within the same lane (downtrack is the DTD of the target vehicle).
    bool PlatoonStrategicIHPPlugin::isVehicleRightInFront(double downtrack, double crosstrack) 
    {
        // Position info of the host vehcile
        double currentDtd = current_downtrack_;
        double currentCtd = current_crosstrack_;
        bool samelane = abs(currentCtd-crosstrack) <= config_.maxCrosstrackError;

        if (downtrack > currentDtd && samelane)
        {
            ROS_DEBUG_STREAM("Found a platoon in front. We are able to join");
            return true;
        }
        else 
        {
            ROS_DEBUG_STREAM("Ignoring platoon that is either behind host or in another lane.");
            ROS_DEBUG_STREAM("The front platoon dtd is " << downtrack << " and we are current at " << currentDtd);
            return false;
        }
    }
    
    // UCLA: check if target platoon at back, and within the same lane. Used for same-lane frontal join.
    bool PlatoonStrategicIHPPlugin::isVehicleRightBehind(double downtrack, double crosstrack)
    {   
        double currentDtd = current_downtrack_;
        double currentCtd = current_crosstrack_;
        bool samelane = abs(currentCtd-crosstrack) <= config_.maxCrosstrackError;

        if (downtrack < currentDtd && samelane) 
        {
            ROS_DEBUG_STREAM("Found a platoon at behind. We are able to join");
            return true;
        }
        else 
        {
            ROS_DEBUG_STREAM("Ignoring platoon that is either ahead of us or in another lane.");
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

        //TODO future: consider setting recipient_id to the platoon ID to make it obvious that anyone in that group is intended reader.
        //             This requires an architectural agreement on use of group messaging protocol.

        // Extract data
        cav_msgs::MobilityOperation msg;
        msg.m_header.plan_id = pm_.currentPlatoonID;
        msg.m_header.recipient_id = "";
        std::string hostStaticId = config_.vehicleID;
        msg.m_header.sender_id = hostStaticId;
        msg.m_header.timestamp = ros::Time::now().toNSec() / 1000000;
        msg.strategy = PLATOONING_STRATEGY;

        // form message 
        double cmdSpeed = cmd_speed_;
        boost::format fmter(OPERATION_STATUS_PARAMS);
        fmter% cmdSpeed;                // index = 0, in m/s.
        fmter% current_speed_;          // index = 1, in m/s.
        fmter% pose_ecef_point_.ecef_x; // index = 2, in cm.
        fmter% pose_ecef_point_.ecef_y; // index = 3, in cm.
        fmter% pose_ecef_point_.ecef_z; // index = 4, in cm.

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
        msg.m_header.plan_id = pm_.currentPlatoonID; // msg.m_header.plan_id is the platoon ID of the request sender (rear join and frontal join). 
        msg.m_header.recipient_id = "";
        std::string hostStaticId = config_.vehicleID;
        msg.m_header.sender_id = hostStaticId;
        msg.m_header.timestamp = ros::Time::now().toNSec() / 1000000;;
        msg.strategy = PLATOONING_STRATEGY;

        double CurrentPlatoonLength = pm_.getCurrentPlatoonLength();
        int PlatoonSize = pm_.getHostPlatoonSize();

        boost::format fmter(OPERATION_INFO_PARAMS);
        //  Note: need to update "OPERATION_INFO_PARAMS" in m_header file --> strategic_platoon_ihp.h  
        fmter% CurrentPlatoonLength;            // index = 0, physical length of the platoon, in m.
        fmter% current_speed_;                  // index = 1, in m/s.
        fmter% PlatoonSize;                     // index = 2, number of members 
        fmter% pose_ecef_point_.ecef_x;         // index = 3, in cm.
        fmter% pose_ecef_point_.ecef_y;         // index = 4, in cm.
        fmter% pose_ecef_point_.ecef_z;         // index = 5, in cm.

        std::string infoParams = fmter.str();
        msg.strategy_params = infoParams;
        ROS_DEBUG_STREAM("Composed a mobility operation message with params " << msg.strategy_params);
        return msg;
    }

    // ----------------------------- UCLA: helper functions for cut-in from front -------------------------------//

    // Note: The function "find_target_lanelet_id" was used to test the IHP platooning logic and is only a pre-written scenario. 
    // The IHP platooning should provide necessary data in a maneuver plan for the arbitrary lane change module.  

    // This is the platoon leader's function to determine if the joining vehicle is closeby
    bool PlatoonStrategicIHPPlugin::isJoiningVehicleNearPlatoon(double joining_downtrack, double joining_crosstrack)
    {
        // Position info of the host vehicle
        double frontVehicleDtd = current_downtrack_;
        double frontVehicleCtd = current_crosstrack_;
        // platoon rear positions
        int lastVehicleIndex = pm_.host_platoon_.size()-1;
        double rearVehicleDtd = pm_.host_platoon_[lastVehicleIndex].vehiclePosition;
        
        // lateral error for two lanes (lane width was calculated based on current lanelet)
        double two_lane_cross_error = 2*config_.maxCrosstrackError + findLaneWidth(); 
        // add longitudinal check threshold in config file 
        bool longitudinalCheck = joining_downtrack >= rearVehicleDtd - config_.longitudinalCheckThresold  || 
                                 joining_downtrack <= frontVehicleDtd + config_.maxCutinGap;
        bool lateralCheck = joining_crosstrack >= frontVehicleCtd - two_lane_cross_error  || 
                            joining_crosstrack <= frontVehicleCtd + two_lane_cross_error;
        // logs for longitudinal and lateral check 
        ROS_DEBUG_STREAM("The longitudinalCheck result is: " << longitudinalCheck );
        ROS_DEBUG_STREAM("The lateralCheck result is: " << lateralCheck );

        if (longitudinalCheck && lateralCheck) 
        {
            ROS_DEBUG_STREAM("Joining vehicle is nearby. It is able to join.");
            return true;
        }
        else 
        {
            ROS_DEBUG_STREAM("The joining vehicle is not close by, the join request will not be approved.");
            ROS_DEBUG_STREAM("The joining vehicle downtrack is " << joining_downtrack << " and the host (platoon leader) downtrack is " << frontVehicleDtd);
            ROS_DEBUG_STREAM("The joining vehicle crosstrack is " << joining_crosstrack << " and the host (platoon leader) crosstrack is " << frontVehicleCtd);
            return false;
        }
    }

    // Check if the host vehicle is close to the platoon for cut-in join.
    bool PlatoonStrategicIHPPlugin::isVehicleNearTargetPlatoon(double rearVehicleDtd, double frontVehicleDtd, double frontVehicleCtd)
    {

        // lateral error for two lanes (lane width was calculated based on current lanelet)
        double two_lane_cross_error = 2*config_.maxCrosstrackError + findLaneWidth(); 
        // add longitudinal check threshold in config file 
        bool longitudinalCheck = current_downtrack_ >= rearVehicleDtd - config_.longitudinalCheckThresold || 
                                 current_downtrack_ <= frontVehicleDtd + config_.longitudinalCheckThresold;
        bool lateralCheck = abs(current_crosstrack_ - frontVehicleCtd) <= two_lane_cross_error;
        // current_crosstrack_ >= frontVehicleCtd - two_lane_cross_error || 
        //                     current_crosstrack_ <= frontVehicleCtd + two_lane_cross_error;
        // logs for longitudinal and lateral check 
        ROS_DEBUG_STREAM("The longitudinalCheck result is: " << longitudinalCheck );
        ROS_DEBUG_STREAM("The lateralCheck result is: " << lateralCheck );

        if (longitudinalCheck && lateralCheck) 
        {
            // host vehicle is close to target platoon longitudinally (within 10m) and laterally (within 5m)
            ROS_DEBUG_STREAM("Found a platoon nearby. We are able to join.");
            return true;
        }
        else 
        {
            ROS_DEBUG_STREAM("Ignoring platoon.");
            ROS_DEBUG_STREAM("The platoon leader dtd is " << frontVehicleDtd << " and we are current at " << current_downtrack_);
            ROS_DEBUG_STREAM("The platoon leader ctd is " << frontVehicleCtd << " and we are current at " << current_crosstrack_);
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
            status_msg.state = cav_msgs::PlatooningInfo::DISABLED;
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADER)
        {
            status_msg.state = pm_.getHostPlatoonSize() == 1 ? cav_msgs::PlatooningInfo::SEARCHING : cav_msgs::PlatooningInfo::LEADING;
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
            status_msg.state = cav_msgs::PlatooningInfo::CONNECTING_TO_NEW_FOLLOWER;
        }
        // UCLA: add "prepare to join" for frontal join (inherited from leader waiting).
        else if (pm_.current_platoon_state == PlatoonState::PREPARETOJOIN)
        {
            status_msg.state = cav_msgs::PlatooningInfo::CONNECTING_TO_NEW_LEADER;
        }
        //TODO: Place holder for departure (PREPARE TO DEPART)
        
        // compose msgs for anything other than standby
        if (pm_.current_platoon_state != PlatoonState::STANDBY)
        {
            status_msg.platoon_id = pm_.currentPlatoonID;
            status_msg.size = pm_.getHostPlatoonSize();
            status_msg.size_limit = config_.maxPlatoonSize;

            if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
            {
                PlatoonMember platoon_leader = pm_.getDynamicLeader();
                status_msg.leader_id = platoon_leader.staticId;
                status_msg.leader_downtrack_distance = platoon_leader.vehiclePosition;
                status_msg.leader_cmd_speed = platoon_leader.commandSpeed;
                status_msg.host_platoon_position = pm_.getNumberOfVehicleInFront();
                ROS_DEBUG_STREAM("pm platoonsize: " << pm_.getHostPlatoonSize() << ", platoon_leader " << platoon_leader.staticId);

                int numOfVehiclesGaps = pm_.getNumberOfVehicleInFront() - pm_.dynamic_leader_index_;
                ROS_DEBUG_STREAM("The host vehicle have " << numOfVehiclesGaps << " vehicles between itself and its leader (includes the leader)");
                
                // use current position to find lanelet ID
                lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);

                auto llts = wm_->getLaneletsFromPoint(current_loc, 1);

                double lanelet_digitalgap = config_.standStillHeadway;

                if (!llts.empty())
                {
                    auto geofence_gaps = llts[0].regulatoryElementsAs<lanelet::DigitalMinimumGap>();
                    
                    if (!geofence_gaps.empty())
                    {
                        lanelet_digitalgap = geofence_gaps[0]->getMinimumGap();
                    }
                }

                else
                {
                    ROS_DEBUG_STREAM("No lanelets in this location!!!: ");
                }
                
                ROS_DEBUG_STREAM("lanelet_digitalgap: " << lanelet_digitalgap);
                double desired_headway = std::max(current_speed_ * config_.timeHeadway, lanelet_digitalgap);
                ROS_DEBUG_STREAM("speed based gap: " << current_speed_ * config_.timeHeadway);
                ROS_DEBUG_STREAM("max desired_headway " << desired_headway);
                // TODO: currently the average length of the vehicle is obtained from a config parameter. In future, plugin needs to be updated to receive each vehicle's actual length through status or BSM messages for more accuracy.
                status_msg.desired_gap = std::max(config_.standStillHeadway * numOfVehiclesGaps, desired_headway * numOfVehiclesGaps) + (numOfVehiclesGaps - 1) * 5.0;//config_.averageVehicleLength;
                ROS_DEBUG_STREAM("The desired gap with the leader is " << status_msg.desired_gap);


                // TODO: To uncomment the following lines, platooninfo msg must be updated
                // UCLA: Add the value of the summation of "veh_len/veh_speed" for all predecessors
                status_msg.current_predecessor_time_headway_sum = pm_.getPredecessorTimeHeadwaySum();
                // UCLA: preceding vehicle info 
                status_msg.predecessor_speed = pm_.getPredecessorSpeed();
                status_msg.predecessor_position = pm_.getPredecessorPosition();

                // Note: use isCreateGap to adjust the desired gap send to control plugin 
                double regular_gap = status_msg.desired_gap;
                ROS_DEBUG_STREAM("regular_gap: " << regular_gap);
                ROS_DEBUG_STREAM("current_speed_: " << current_speed_);
                ROS_DEBUG_STREAM("speed based gap: " << desired_headway);
                if (pm_.isCreateGap){
                    // enlarged desired gap for gap creation
                    status_msg.desired_gap = regular_gap*(1 + config_.createGapAdjuster);
                }
                status_msg.actual_gap = platoon_leader.vehiclePosition - current_downtrack_;
                ROS_DEBUG_STREAM("status_msg.actual_gap: " << status_msg.actual_gap);
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
            ROS_ERROR("UNKNOWN strategy param string!!!");
            msg.strategy_params = "";
        }
        return msg;
    }

    // Compose the Mobility Operation message for Follower state. Message parameter types: STATUS.
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationFollower()
    {
        cav_msgs::MobilityOperation msg;
        msg = composeMobilityOperationSTATUS();
        return msg;
    }

    // Compose the Mobility Operation message for LeaderWaiting state. Message parameter types: STATUS.
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationLeaderWaiting()
    {
        //TODO: shouldn't a leaderwaiting also be sending INFO messages since it is still leading?

        cav_msgs::MobilityOperation msg;
        msg = composeMobilityOperationSTATUS();
        return msg;
    }

    // Compose the Mobility Operation message for CandidateFollower state. Message parameter types: STATUS.
    cav_msgs::MobilityOperation PlatoonStrategicIHPPlugin::composeMobilityOperationCandidateFollower()
    {
        cav_msgs::MobilityOperation msg;
        msg = composeMobilityOperationSTATUS();
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
            ROS_ERROR("UNKNOWN strategy param string!!!");
            msg.strategy_params = "";
        }
        return msg;
    }

    // TODO: Place holder for compose mobility operation msg for prepare to depart.

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

        std::vector<std::string> ecef_y_parsed;
        boost::algorithm::split(ecef_y_parsed, inputsParams[3], boost::is_any_of(":"));
        double ecef_y = std::stod(ecef_y_parsed[1]);

        std::vector<std::string> ecef_z_parsed;
        boost::algorithm::split(ecef_z_parsed, inputsParams[4], boost::is_any_of(":"));
        double ecef_z = std::stod(ecef_z_parsed[1]);
        
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

        ROS_DEBUG_STREAM("Entered mob_op_cb_STATUS");
        std::string strategyParams = msg.strategy_params;
        std::string vehicleID = msg.m_header.sender_id;
        std::string platoonId = msg.m_header.plan_id;
        ROS_DEBUG_STREAM("strategyParams = " << strategyParams);
        ROS_DEBUG_STREAM("platoonId = " << platoonId << ", sender ID = " << vehicleID);
        std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size() + 1);
        ROS_DEBUG_STREAM("pm_.currentPlatoonID = " << pm_.currentPlatoonID << ", targetPlatoonID = " << pm_.targetPlatoonID);

        // read Downtrack 
        cav_msgs::LocationECEF ecef_loc = mob_op_find_ecef_from_STATUS_params(strategyParams);
        lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(ecef_loc);
        double dtd = wm_->routeTrackPos(incoming_pose).downtrack;
        ROS_DEBUG_STREAM("DTD calculated from ecef is: " << dtd);
        // read Crosstrack
        double ctd = wm_->routeTrackPos(incoming_pose).crosstrack;
        ROS_DEBUG_STREAM("CTD calculated from ecef is: " << ctd);

        // If it comes from a member of an identified neighbor platoon, then
        if (platoonId.compare(pm_.neighborPlatoonID) == 0 && platoonId.compare(pm_.dummyID) != 0)
        {
            ROS_DEBUG_STREAM("Incoming platoonID matches target platoon id");
            // // Update this member's status (or add if it's unknown to us)
            pm_.neighborMemberUpdates(vehicleID, platoonId, statusParams, dtd, ctd);
        }

        // else if this message is for our platoon then store its info
        else if (platoonId.compare(pm_.currentPlatoonID) == 0 && platoonId.compare(pm_.dummyID) != 0)
        {
            pm_.hostMemberUpdates(vehicleID, platoonId, statusParams, dtd, ctd);
        }

        // else it represents an uninteresting platoon
        else
        {
            ROS_DEBUG_STREAM("Received mob op for platoon " << platoonId << " that doesn't match our platoon: " << pm_.currentPlatoonID
                             << " or known neighbor platoon: " << pm_.targetPlatoonID);
        }
    }    

    //
    double PlatoonStrategicIHPPlugin::mob_op_find_platoon_length_from_INFO_params(std::string strategyParams)
    {
        /** 
         * Note: INFO param format:
         *      "INFO| --> LENGTH:%.2f,SPEED:%.2f,SIZE:%d,ECEFX:%.2f,ECEFY:%.2f,ECEFZ:%.2f"
         *           |-------0-----------1---------2--------3----------4----------5-------|
         */
        // For INFO params, the string format is INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d,DTD:%.2f
        std::vector<std::string> inputsParams;
        boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));

        // Use the strategy params' length value and leader location to determine DTD of its rear
        std::vector<std::string> target_platoon_length;
        boost::algorithm::split(target_platoon_length, inputsParams[0], boost::is_any_of(":"));
        double platoon_length = std::stod(target_platoon_length[1]);

        return platoon_length;
    }

    // UCLA: Parse ecef location from INFO params
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

        std::vector<std::string> ecef_y_parsed;
        boost::algorithm::split(ecef_y_parsed, inputsParams[4], boost::is_any_of(":"));
        double ecef_y = std::stod(ecef_y_parsed[1]);

        std::vector<std::string> ecef_z_parsed;
        boost::algorithm::split(ecef_z_parsed, inputsParams[5], boost::is_any_of(":"));
        double ecef_z = std::stod(ecef_z_parsed[1]);
        
        cav_msgs::LocationECEF ecef_loc;
        ecef_loc.ecef_x = ecef_x;
        ecef_loc.ecef_y = ecef_y;
        ecef_loc.ecef_z = ecef_z;

        return ecef_loc;
    }

    // handle message for each states.
    void PlatoonStrategicIHPPlugin::mob_op_cb(const cav_msgs::MobilityOperation& msg)
    {
        if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            return;
        }

        ROS_DEBUG_STREAM("mob_op_cb received msg with sender ID " << msg.m_header.sender_id
                        << ", plan ID " << msg.m_header.plan_id);
        ROS_DEBUG_STREAM("...strategy " << msg.strategy << ", strategy params " << msg.strategy_params);

        // Check that this is a message about platooning (could be from some other Carma activity nearby)
        std::string strategy = msg.strategy;
        if (strategy.rfind(PLATOONING_STRATEGY, 0) != 0)
        {
            ROS_DEBUG_STREAM("Ignoring mobility operation message for " << strategy << " strategy.");
            return;
        }

        // Ignore messages as long as host vehicle is stopped
        if (current_speed_ < STOPPED_SPEED)
        {
            ROS_DEBUG_STREAM("Ignoring message since host is stopped.");
            return;
        }

        // Perform common operations that apply to all states
        std::string strategyParams = msg.strategy_params;
        bool isPlatoonStatusMsg = strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0;
        bool isPlatoonInfoMsg = strategyParams.rfind(OPERATION_INFO_TYPE, 0) == 0;
        ROS_DEBUG_STREAM("strategyParams: " << strategyParams << "isPlatoonStatusMsg: " << isPlatoonStatusMsg << "isPlatoonInfoMsg: " << isPlatoonInfoMsg);
        if (isPlatoonStatusMsg) 
        {
            mob_op_cb_STATUS(msg);
        }
        else if (isPlatoonInfoMsg)
        {
            // Note: this is where confusion will reign if multiple neighbor platoons are discovered; need more sophisticated
            //       logic to handle that situation

            // If it is a legitimate platoon (2 or more members) other than our own then
            std::vector<std::string> inputsParams;
            boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));
            std::vector<std::string> p_size;
            boost::algorithm::split(p_size, inputsParams[2], boost::is_any_of(":"));
            int platoon_size = std::stoi(p_size[1]);
            ROS_DEBUG_STREAM("neighbor platoon_size from INFO: " << platoon_size);
            if (platoon_size > 1  &&  msg.m_header.plan_id.compare(pm_.currentPlatoonID) != 0)
            {
                // If platoon ID doesn't match our known target platoon then clear any old neighbor platoon info and record
                // the platoon ID and the sender as the leader (only leaders send INFO)
                if (msg.m_header.plan_id.compare(pm_.neighborPlatoonID) != 0)
                {
                    pm_.resetNeighborPlatoon();
                    pm_.neighborPlatoonID = msg.m_header.plan_id;
                }
                pm_.neighbor_platoon_leader_id_ = msg.m_header.sender_id;
                ROS_DEBUG_STREAM("pm_.neighbor_platoon_leader_id_: " << pm_.neighbor_platoon_leader_id_);
                pm_.neighbor_platoon_info_size_ = platoon_size;
                ROS_DEBUG_STREAM("pm_.neighbor_platoon_info_size_: " << pm_.neighbor_platoon_info_size_);
            }
        }

        else
        {
            ROS_DEBUG_STREAM("Invalid Mob Op received");
        }

        // Perform state-specific additional actions
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
        // TODO: Place holder for prepare to depart

        // TODO: If needed (with large size platoons), add a queue for status messages
        //       INFO messages always processed, STATUS messages if saved in que
    }
    
    void PlatoonStrategicIHPPlugin::mob_op_cb_standby(const cav_msgs::MobilityOperation& msg)
    {
        // In standby state, it will ignore operation message since it is not actively operating
        ROS_DEBUG_STREAM("STANDBY state no further action on message from " << msg.m_header.sender_id);
    }

    // Handle STATUS operation message 
    void PlatoonStrategicIHPPlugin::mob_op_cb_candidatefollower(const cav_msgs::MobilityOperation& msg)
    {
        ROS_DEBUG_STREAM("CANDIDATEFOLLOWER state no further action on message from " << msg.m_header.sender_id);
    }

    // Handle STATUS operation message 
    void PlatoonStrategicIHPPlugin::mob_op_cb_follower(const cav_msgs::MobilityOperation& msg)
    {
        //std::string strategyParams = msg.strategy_params;
        //bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);

        // TODO: Place holder for prepare to depart

        ROS_DEBUG_STREAM("FOLLOWER state no further action on message from " << msg.m_header.sender_id);
    }

    // Handle STATUS operation message 
    void PlatoonStrategicIHPPlugin::mob_op_cb_leaderwaiting(const cav_msgs::MobilityOperation& msg)
    {
        ROS_DEBUG_STREAM("LEADERWAITING state no further action on message from " << msg.m_header.sender_id);
    }

    // UCLA: Handle both STATUS and INFO operation message. Front join and rear join are all handled if incoming operation message have INFO param. 
    void PlatoonStrategicIHPPlugin::mob_op_cb_leader(const cav_msgs::MobilityOperation& msg)
    {   
        /** 
         * Note: This is the function to handle the mobility operation message. Vehicle in leader state is either a single ADS vehicle or a platoon leader.
         * 
         * Single ADS will send out INFO messages. Platoon leaders will send out both INFO and STATUS messages.
         * 
         * If the host is single vehicle, it should have "isPlatoonInfoMsg = true" and "isInNegotiation = false". In such condition, single leader will start 
         * joining and send request to the platoon leader. mob_op_cb_leader(
         * 
         * If the host vehicle is platoon leader, then it should have "isPlatoonInfoMsg = true" and "isInNegotiation = true". But existing platoon leader do not 
         * need to send out joining request.
         * 
         * Both the platoon leader and single vehicle need to subscribe to the STATUS message to populate the platoon manager with existing platoon members, 
         * so the PM can calculate dtd and ctd corresponds to host vehicle's origin position, hence to be used for later calculation.          
         */

        // Read incoming message info
        std::string strategyParams = msg.strategy_params;
        std::string senderId = msg.m_header.sender_id; 
        std::string platoonId = msg.m_header.plan_id; 

        // In the current state, host vehicle care about the INFO heart-beat operation message if we are not currently in
        // a negotiation, and host also need to care about operation from members in our current platoon.
        bool isPlatoonInfoMsg = strategyParams.rfind(OPERATION_INFO_TYPE, 0) == 0;            // INFO message only broadcast by leader and single CAV.
        bool isInNegotiation = pm_.current_plan.valid  ||  pm_.currentPlatoonID.compare(pm_.dummyID) != 0; // In negotiation indicates host is not available to become a joiner
                                                                                              // (i.e., not currently in a platoon or trying to join a platoon).
        ROS_DEBUG_STREAM("Top of mob_op_cb_leader, isInNegotiation = " << isInNegotiation);

        // Condition 1. Host vehicle is the single CAV joining the platoon.
        if (isPlatoonInfoMsg && !isInNegotiation)
        {
            //TODO future enhancement: add logic here for if/how to join two existing platoons
            //     (e.g. the shorter platoon should join the longer one, or rear joins front if same length)

            // step 1. read INFO message from the target platoon leader

            // read ecef location from strategy params.
            cav_msgs::LocationECEF ecef_loc;
            ecef_loc = mob_op_find_ecef_from_INFO_params(strategyParams);
            
            // use ecef_loc to calculate front Dtd in m.
            lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(ecef_loc);
            double frontVehicleDtd = wm_->routeTrackPos(incoming_pose).downtrack;

            // use ecef_loc to calculate front Ctd in m.
            double frontVehicleCtd = wm_->routeTrackPos(incoming_pose).crosstrack;
            // downtrack and crosstrack of the platoon leader --> used for frontal join
            ROS_DEBUG_STREAM("Neighbor platoon frontVehicleDtd from ecef: " << frontVehicleDtd << ", frontVehicleCtd from ecef: " << frontVehicleCtd);

            // use INFO param to find platoon rear vehicle DTD and CTD.
            double platoon_length = mob_op_find_platoon_length_from_INFO_params(strategyParams); // length of the entire platoon in meters.
            /**
             *  note: 
             *       [**veh3*] ---------- [*veh2*] ----------------- [*veh1*] ---------- [*veh0**]
             *           |<------------------ front-rear DTD difference -------------------->|
             *       |<----------------------------- platoon length ---------------------------->|
             *       
             *       front-rear DTD difference = platoon_length + one_vehicle_length
             *       Vehicle length is already accounted for in the message's LENGTH value
             */

            double rearVehicleDtd = frontVehicleDtd - platoon_length; 
            ROS_DEBUG_STREAM("rear veh dtd from platoon length: " << rearVehicleDtd);
            if (!pm_.neighbor_platoon_.empty())
            {
                rearVehicleDtd = pm_.neighbor_platoon_.back().vehiclePosition;
                ROS_DEBUG_STREAM("rear veh dtd from neighbor platoon: " << rearVehicleDtd);
            }
            
            // Note: For one platoon, we assume all members are in the same lane.
            double rearVehicleCtd = frontVehicleCtd;
            ROS_DEBUG_STREAM("Neighbor platoon rearVehicleDtd: " << rearVehicleDtd << ", rearVehicleCtd: " << rearVehicleCtd);

            // Parse the strategy params
            std::vector<std::string> inputsParams;
            boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));

            // Get the target platoon's size (number of members) from strategy params
            std::vector<std::string> targetPlatoonSize_parsed;
            boost::algorithm::split(targetPlatoonSize_parsed, inputsParams[2], boost::is_any_of(":"));
            int targetPlatoonSize = std::stoi(targetPlatoonSize_parsed[1]);
            ROS_DEBUG_STREAM("target Platoon Size: " << targetPlatoonSize);
            ROS_DEBUG_STREAM("Found a vehicle/platoon with id = " << platoonId << " within range.");

            //TODO future: add logic here to assess closeness of the neighbor platoon, as well as its speed, destination
            //             & other attributes to decide if we want to join before assembling a join request

            // step 2. Generate default info for join request
            cav_msgs::MobilityRequest request;
            request.m_header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
            request.m_header.recipient_id = senderId;
            request.m_header.sender_id = config_.vehicleID;
            request.m_header.timestamp = ros::Time::now().toNSec()/1000000;
            request.location = pose_to_ecef(pose_msg_);
            request.strategy = PLATOONING_STRATEGY;
            request.urgency = 50;

            int platoon_size = pm_.getHostPlatoonSize();
            
            // step 3. Request Rear Join 
            if(isVehicleRightInFront(rearVehicleDtd, rearVehicleCtd)  &&  !config_.test_front_join)
            {   
                /**
                 *  Note: "isVehicleRightInFront" tests for same lane
                 */
                ROS_DEBUG_STREAM("Neighbor platoon is right in front of us");

                request.plan_type.type = cav_msgs::PlanType::JOIN_PLATOON_AT_REAR;

                /*
                 * JOIN_PARAMS format: 
                 *        JOIN_PARAMS| --> "SIZE:%1%,SPEED:%2%,ECEFX:%3%,ECEFY:%4%,ECEFZ:%5%,JOINIDX:%6%"
                 *                   |-------0------ --1---------2---------3---------4----------5-------|  
                 */

                boost::format fmter(JOIN_PARAMS);
                int dummy_join_index = -2; //not used for this message, but message spec requires it
                fmter %platoon_size;                //  index = 0
                fmter %current_speed_;              //  index = 1, in m/s
                fmter %pose_ecef_point_.ecef_x;     //  index = 2, in cm.
                fmter %pose_ecef_point_.ecef_y;     //  index = 3, in cm.
                fmter %pose_ecef_point_.ecef_z;     //  index = 4, in cm.
                fmter %dummy_join_index;            //  index = 5
                request.strategy_params = fmter.str();
                mobility_request_publisher_(request);
                ROS_DEBUG_STREAM("Publishing request to leader " << senderId << " with params " << request.strategy_params << " and plan id = " << request.m_header.plan_id);

                // Create a new join plan
                pm_.current_plan = ActionPlan(true, request.m_header.timestamp, request.m_header.plan_id, senderId);

                // If we are asking to join an actual platoon (not a solo vehicle), then save its ID for later use
                if (platoonId.compare(pm_.dummyID) != 0)
                {
                    pm_.targetPlatoonID = platoonId;
                    ROS_DEBUG_STREAM("Detected neighbor as a real platoon & storing its ID: " << platoonId);
                }
            }

            // step 4. Request frontal join, if the neighbor is a real platoon
            else if ((targetPlatoonSize > 1  ||  config_.test_front_join)  &&  isVehicleRightBehind(frontVehicleDtd, frontVehicleCtd))
            {   
                /**
                 *  Note: "isVehicleRightBehind" tests for same lane
                 */
                ROS_DEBUG_STREAM("Neighbor platoon leader is right behind us");
                
                // UCLA: assign a new plan type
                request.plan_type.type = cav_msgs::PlanType::JOIN_PLATOON_FROM_FRONT;

                /**
                 * JOIN_PARAMS format: 
                 *        JOIN_PARAMS| --> "SIZE:%1%,SPEED:%2%,ECEFX:%3%,ECEFY:%4%,ECEFZ:%5%,JOINIDX:%6%"
                 *                   |-------0------ --1---------2---------3---------4----------5-------|  
                 */
                boost::format fmter(JOIN_PARAMS); // Note: Front and rear join uses same params, hence merge to one param for both condition.
                int dummy_join_index = -2; //not used for this message, but message spec requires it
                fmter %platoon_size;                //  index = 0
                fmter %current_speed_;              //  index = 1, in m/s
                fmter %pose_ecef_point_.ecef_x;     //  index = 2, in cm.
                fmter %pose_ecef_point_.ecef_y;     //  index = 3, in cm.
                fmter %pose_ecef_point_.ecef_z;     //  index = 4, in cm.
                fmter %dummy_join_index;            //  index = 5
                request.strategy_params = fmter.str();
                mobility_request_publisher_(request);
                ROS_DEBUG_STREAM("Publishing front join request to the leader " << senderId << " with params " << request.strategy_params << " and plan id = " << request.m_header.plan_id);

                // Create a new join plan
                pm_.current_plan = ActionPlan(true, request.m_header.timestamp, request.m_header.plan_id, senderId);

                // If testing with a solo vehicle to represent the target platoon, then use the current join plan ID for that platoon;
                // otherwise, it will already have and ID so store it for future use
                if (config_.test_front_join)
                {
                    pm_.targetPlatoonID = request.m_header.plan_id;
                    ROS_DEBUG_STREAM("Since neighbor is a fake platoon, storing " << pm_.targetPlatoonID << " as its platoon ID");
                }
                else
                {
                    pm_.targetPlatoonID = platoonId;
                    ROS_DEBUG_STREAM("Storing real neighbor platoon's ID as target: " << pm_.targetPlatoonID);
                }
            }

            // step 5. Request cut-in join (front, middle or rear, from adjacent lane)
            else if ((targetPlatoonSize > 1  ||  config_.test_cutin_join)  &&  !config_.test_front_join  
                        &&  isVehicleNearTargetPlatoon(rearVehicleDtd, frontVehicleDtd, frontVehicleCtd))
            {

                ROS_DEBUG_STREAM("starting cut-in join process");
                ROS_DEBUG_STREAM("rearVehicleDtd " << rearVehicleDtd);
                ROS_DEBUG_STREAM("rearVehicleCtd " << rearVehicleCtd);

                // If we are asking to join an actual platoon (not a solo vehicle), then save its ID for later use
                if (platoonId.compare(pm_.dummyID) != 0)
                {
                    pm_.targetPlatoonID = platoonId;
                    ROS_DEBUG_STREAM("Detected neighbor as a real platoon & storing its ID: " << platoonId);
                }

                
                carma_wm::TrackPos target_trackpose(rearVehicleDtd, rearVehicleCtd);
                auto target_rear_pose = wm_->pointFromRouteTrackPos(target_trackpose);
                if (target_rear_pose)
                {
                    target_cutin_pose_ = incoming_pose;

                    auto target_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, target_rear_pose.get(), 1);  
                    if (!target_lanelets.empty())
                    {
                        long target_rear_pose_lanelet_id = target_lanelets[0].second.id();
                        ROS_DEBUG_STREAM("target_rear_pose_lanelet_id: " << target_rear_pose_lanelet_id);
                    }
                    else
                    {
                        ROS_DEBUG_STREAM("target_rear_pose_lanelet not found!!");
                    }
                }
                    
                else
                {
                    ROS_DEBUG_STREAM("No target pose is found, so we cannot prodeed with a cutin join request.");
                    return;
                }

                /**
                 * UCLA Implementation note:
                 *  1. isVehicleNearTargetPlatoon --> determine if the platoon is next to host vehicle
                 *  2. sender_id == requesting veh ID --> joiner ID
                 */

                //TODO: verify purpose of this logic then its correctness accordingly:
                //  -is step 5 (this block) really for all 3 types of joining?
                //  -ensure join_index gets an appropriate value for the type of join
                //  -ensure plan type corresponds to join_index

                // complete the request
                // UCLA: this is a newly added plan type 
                // Note: Request conposed outside of if conditions
                // UCLA: Desired joining index for cut-in join, indicate the index of gap-leading vehicle. -1 indicate cut-in from front.
                // Note: remove join_index to info param.
                request.plan_type.type = cav_msgs::PlanType::PLATOON_CUT_IN_JOIN; 

                // At this step all cut-in types start with this request, so the join_index at this point is set to default, -2.
                int join_index = -2;
                boost::format fmter(JOIN_PARAMS); // Note: Front and rear join uses same params, hence merge to one param for both condition.
                fmter %platoon_size;                //  index = 0
                fmter %current_speed_;              //  index = 1, in m/s
                fmter %pose_ecef_point_.ecef_x;     //  index = 2
                fmter %pose_ecef_point_.ecef_y;     //  index = 3
                fmter %pose_ecef_point_.ecef_z;     //  index = 4
                fmter %join_index;                  //  index = 5
                request.strategy_params = fmter.str();
                mobility_request_publisher_(request);
                ROS_DEBUG_STREAM("Publishing request to the leader " << senderId << " with params " << request.strategy_params << " and plan id = " << request.m_header.plan_id);

                // Create a new join plan
                pm_.current_plan = ActionPlan(true, request.m_header.timestamp, request.m_header.plan_id, senderId);
            }

            // step 6. Return none if no platoon nearby
            else 
            {
                ROS_DEBUG_STREAM("Ignore platoon with platoon id: " << platoonId << " because it is too far away to join.");
            }
        }
        
        // TODO: Place holder for prepare to depart (else if isdepart)

    }

    // UCLA: mob_op_cb for the new leader aborting state (inherited from candidate follower), handle STATUS message.
    void PlatoonStrategicIHPPlugin::mob_op_cb_leaderaborting(const cav_msgs::MobilityOperation& msg)
    {   
        ROS_DEBUG_STREAM("LEADERABORTING state no further action on message from " << msg.m_header.sender_id);
    }
    
    // UCLA: mob_op_candidateleader for the new candidate leader state (inherited from leader waiting), handle STATUS message.
    void PlatoonStrategicIHPPlugin::mob_op_cb_candidateleader(const cav_msgs::MobilityOperation& msg)
    {   
        ROS_DEBUG_STREAM("CANDIDATELEADER state no further action on message from " << msg.m_header.sender_id);
    }

    // UCLA: Mobility operation callback for lead_with_operation state (cut-in join).
    void PlatoonStrategicIHPPlugin::mob_op_cb_leadwithoperation(const cav_msgs::MobilityOperation& msg)
    {
        ROS_DEBUG_STREAM("LEADWITHOPERATION state no further action on message from " << msg.m_header.sender_id);
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
        std::string senderId = msg.m_header.sender_id; 
        
        // locate INFO type
        bool isPlatoonInfoMsg = strategyParams.rfind(OPERATION_INFO_TYPE, 0) == 0;

        // If this is an INFO message and our record of the neighbor platoon is complete then
        // pm_.is_neighbor_record_complete_ = true; //TODO temporary
        ROS_DEBUG_STREAM("pm_.is_neighbor_record_complete_" << pm_.is_neighbor_record_complete_);

        if (isPlatoonInfoMsg  &&  pm_.is_neighbor_record_complete_)
        {

            //TODO: would be good to have a timeout here; if a neighbor platoon has been identified, and no INFO messages
            //      from it are received in a while, then its record should be erased, and any in-work joining should be
            //      aborted.

            // read ecef location from strategy params.
            cav_msgs::LocationECEF ecef_loc;
            ecef_loc = mob_op_find_ecef_from_INFO_params(strategyParams);
            // use ecef_loc to calculate front Dtd in m.
            lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(ecef_loc);

            double frontVehicleDtd = wm_->routeTrackPos(incoming_pose).downtrack;

            // use ecef_loc to calculate front Ctd in m.
            double frontVehicleCtd = wm_->routeTrackPos(incoming_pose).crosstrack;

            // // Find neighbor platoon end vehicle and its downtrack in m
            int rearVehicleIndex = pm_.neighbor_platoon_.size() - 1;
            ROS_DEBUG_STREAM("rearVehicleIndex: " << rearVehicleIndex);
            double rearVehicleDtd = pm_.neighbor_platoon_[rearVehicleIndex].vehiclePosition; 
            ROS_DEBUG_STREAM("Neighbor rearVehicleDtd from ecef: " << rearVehicleDtd);

            // If lane change has not yet been authorized, stop here (this method will be running before the negotiations
            // with the platoon leader are complete)
            if (!safeToLaneChange_)
            {
                return;
            }

            // determine if the lane change is finished
            bool isSameLaneWithPlatoon = abs(frontVehicleCtd - current_crosstrack_) <= config_.maxCrosstrackError;
            ROS_DEBUG_STREAM("Lane change has been authorized. isSameLaneWithPlatoon = " << isSameLaneWithPlatoon);
            ROS_DEBUG_STREAM("crosstrack diff" << abs(frontVehicleCtd - current_crosstrack_));
            if (isSameLaneWithPlatoon)
            {
                // request 1. reset the safeToChangLane indicators if lane change is finished
                safeToLaneChange_ = false;
                
                // request 2. change to same lane operation states (determine based on DTD differences)
                cav_msgs::MobilityRequest request;
                request.m_header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
                request.m_header.recipient_id = senderId;
                request.m_header.sender_id = config_.vehicleID;
                request.m_header.timestamp = ros::Time::now().toNSec()/1000000;
                request.location = pose_to_ecef(pose_msg_);

                // UCLA: send request based on cut-in type
                if (frontVehicleDtd < current_downtrack_) 
                {
                    request.plan_type.type = cav_msgs::PlanType::CUT_IN_FRONT_DONE;
                }
                else
                {
                    request.plan_type.type = cav_msgs::PlanType::CUT_IN_MID_OR_REAR_DONE;
                }
                request.strategy = PLATOONING_STRATEGY;
                double host_platoon_size = pm_.getHostPlatoonSize();

                boost::format fmter(JOIN_PARAMS);
                fmter %host_platoon_size;           //  index = 0
                fmter %current_speed_;              //  index = 1, in m/s
                fmter %pose_ecef_point_.ecef_x;     //  index = 2
                fmter %pose_ecef_point_.ecef_y;     //  index = 3
                fmter %pose_ecef_point_.ecef_z;     //  index = 4     
                fmter %target_join_index_;          //  index = 5
                request.strategy_params = fmter.str();
                request.urgency = 50;

                mobility_request_publisher_(request); 
                if (pm_.currentPlatoonID.compare(pm_.dummyID) == 0)
                {
                    pm_.currentPlatoonID = request.m_header.plan_id;
                }
                
                ROS_DEBUG_STREAM("new platoon id: " << pm_.currentPlatoonID);
                pm_.current_plan = ActionPlan(true, request.m_header.timestamp, request.m_header.plan_id, senderId);
                
                ROS_DEBUG_STREAM("Published Mobility request to revert to same-lane operation"); 
            }
            else
            {
                ROS_DEBUG_STREAM("Lane Change not completed");
            }
        }
    }
    
    // TODO: Place holder for prepare to depart (mob_op_cb_depart)

    //------- 3. Mobility Request Callback -------
    MobilityRequestResponse PlatoonStrategicIHPPlugin::handle_mob_req(const cav_msgs::MobilityRequest& msg)
    {
        MobilityRequestResponse mobility_response = MobilityRequestResponse::NO_RESPONSE;

        // Check that this is a message about platooning (could be from some other Carma activity nearby)
        std::string strategy = msg.strategy;
        if (strategy.rfind(PLATOONING_STRATEGY, 0) != 0)
        {
            ROS_DEBUG_STREAM("Ignoring mobility operation message for " << strategy << " strategy.");
            return MobilityRequestResponse::NO_RESPONSE;
        }

        // Handle the message based on host's current state
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
        // TODO: Place holder for prepare to depart

        return mobility_response;
    }

    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_standby(const cav_msgs::MobilityRequest& msg)
    {
        // In standby state, the plugin is not responsible for replying to any request messages
        ROS_DEBUG_STREAM("STANDBY state does nothing with msg from " << msg.m_header.sender_id);
        return MobilityRequestResponse::NO_RESPONSE;
    }

    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_candidatefollower(const cav_msgs::MobilityRequest& msg)
    {
        // This state does not handle any mobility request for now
        // TODO Maybe it should handle some ABORT request from a waiting leader
        ROS_DEBUG_STREAM("Received mobility request with type " << msg.plan_type.type << " but ignored.");
        return MobilityRequestResponse::NO_RESPONSE;
    }

    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_follower(const cav_msgs::MobilityRequest& msg)
    {
        /**
         * For cut-in join, the gap rear vehicle need to slow down once they received the request from platoon leader. 
         * Note:1. (cut-in) When joiner is in position, the leader will send the request to relevent platoon member. So no need to 
         *          check for "in-position" one more time. 
         *      2. (cut-in) If a new member is added in middle, "mob_op_cb_STATUS" will update and resort all vehicle information.
         *          So the ordering of the platoon can be updated. 
         *      3. (depart) If the current leader is departing, the closest follower (i.e., 2nd in platoon) will become the new leader, 
         *          and switch to candidate follower state. While the previous leader depart and operating in single leader state.
         */
        
        cav_msgs::PlanType plan_type = msg.plan_type;
        std::string reccipientID = msg.m_header.recipient_id;
        std::string reqSenderID = msg.m_header.sender_id;

        // Check joining plan type.
        bool isCutInJoin = plan_type.type == cav_msgs::PlanType::PLATOON_CUT_IN_JOIN;
        bool isGapCreated = plan_type.type == cav_msgs::PlanType::STOP_CREATE_GAP;
        // TODO: Place holder for departure

        // Check if host is intended recipient 
        bool isHostRecipent = pm_.getHostStaticID() == reccipientID;

        if (isCutInJoin && isHostRecipent)
        {
            // Read requesting vehicle's joining index
            std::string strategyParams = msg.strategy_params;
            std::vector<std::string> inputsParams;
            boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));
            std::vector<std::string> join_index_parsed;
            boost::algorithm::split(join_index_parsed, inputsParams[5], boost::is_any_of(":"));
            int req_sender_join_index = std::stoi(join_index_parsed[1]);
            ROS_DEBUG_STREAM("Requesting join_index parsed: " << req_sender_join_index);
        
            // Control vehicle speed based on cut-in type
            // 1. cut-in from rear
            if (static_cast<size_t>(req_sender_join_index) == pm_.host_platoon_.size()-1)
            {
                // Accept plan and idle (becasue rear join, gap leading vehicle do not need to slow down).
                ROS_WARN("Requested cut-in from rear, start approve cut-in and wait for lane change.");
                ROS_WARN("Due to the rear join nature, there is no need to slow down or create gap.");
                return MobilityRequestResponse::ACK;

            }
            // 2. cut-in from middle 
            else if (req_sender_join_index >= 0  &&  static_cast<size_t>(req_sender_join_index) < pm_.host_platoon_.size()-1)
            {   
                // Accept plan and slow down to create gap.
                pm_.isCreateGap = true;
                ROS_DEBUG_STREAM("Requested cut-in index is: " << req_sender_join_index << ", approve cut-in and start create gap.");
                return MobilityRequestResponse::ACK;
            }
            // 3. Abnormal join index
            else
            {
                // Note: Leader will abort plan if reponse is not ACK for plantype "PLATOON_CUT_IN_JOIN".
                ROS_DEBUG_STREAM("Abnormal cut-in index, abort operation.");
                return MobilityRequestResponse::NACK;
            }
        }
        

        // 4. Reset to normal speed once the gap is created.
        else if (isGapCreated)
        {
            ROS_DEBUG_STREAM("Gap is created, revert to normal operating speed.");
            // Only reset create gap indicator, no need to send response. 
            pm_.isCreateGap = false;
            return MobilityRequestResponse::NO_RESPONSE;
        }

        // 5. Place holder for departure (host is follower)

        // 6. Same-lane join, no need to respond.
        else 
        {
            return MobilityRequestResponse::NO_RESPONSE;
        }
    }
    
    // Middle state that decided whether to accept joiner  
    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_leaderwaiting(const cav_msgs::MobilityRequest& msg)
    {
        bool isTargetVehicle = msg.m_header.sender_id == pm_.current_plan.peerId;
        bool isCandidateJoin = msg.plan_type.type == cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN;

        lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(msg.location);
        double obj_cross_track = wm_->routeTrackPos(incoming_pose).crosstrack;
        bool inTheSameLane = abs(obj_cross_track - current_crosstrack_) < config_.maxCrosstrackError;
        ROS_DEBUG_STREAM("current_cross_track error = " << abs(obj_cross_track - current_crosstrack_));
        ROS_DEBUG_STREAM("inTheSameLane = " << inTheSameLane);

        // If everything is agreeable then approve the request; if it is from an unexpected vehicle or
        // is not a candidate join request, then we can just ignore it with no action
        MobilityRequestResponse response = MobilityRequestResponse::NO_RESPONSE;
        if (isTargetVehicle && isCandidateJoin)
        {
            if (inTheSameLane)
            {
                ROS_DEBUG_STREAM("Target vehicle " << pm_.current_plan.peerId << " is actually joining.");
                ROS_DEBUG_STREAM("Changing to PlatoonLeaderState and send ACK to target vehicle");

                // Change state to LEADER
                pm_.current_platoon_state = PlatoonState::LEADER;

                // If we are not already in a platoon, then use this activity plan ID for the newly formed platoon
                if (pm_.currentPlatoonID.compare(pm_.dummyID) == 0)
                {
                    pm_.currentPlatoonID = msg.m_header.plan_id;
                }

                // Add the joiner to our platoon record (ASSUMES that we have not yet received a mob_op STATUS msg from joiner!)
                PlatoonMember newMember = PlatoonMember();
                newMember.staticId = msg.m_header.sender_id;
                newMember.vehiclePosition = wm_->routeTrackPos(incoming_pose).downtrack;
                ROS_DEBUG_STREAM("New member being added to platoon vector whose size is currently " << pm_.host_platoon_.size());
                pm_.host_platoon_.push_back(newMember);
                ROS_DEBUG_STREAM("pm_ now thinks platoon size is " << pm_.getHostPlatoonSize());

                // Send approval of the request
                response = MobilityRequestResponse::ACK;
                // Indicate the current join activity is complete
                pm_.clearActionPlan();
            }
            else //correct vehicle and intent, but it's in the wrong lane
            {
                ROS_DEBUG_STREAM("Received platoon request with vehicle id = " << msg.m_header.sender_id << " but in wrong lane. NACK");
                response = MobilityRequestResponse::NACK;

                // // Remove the candidate joiner from the platoon structure
                // if (!pm_.removeMemberById(msg.m_header.sender_id))
                // {
                //     ROS_DEBUG_STREAM("Failed to remove candidate joiner from platoon record: " << msg.m_header.sender_id);
                // }
            }
        }

        
        return response;
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
         *   Note: For front and rear jon, the JOIN_PARAMS format: 
         *        JOIN_PARAMS| --> "SIZE:%1%,SPEED:%2%,ECEFX:%3%,ECEFY:%4%,ECEFZ:%5%,JOINIDX:%6%"
         *                   |-------0------ --1---------2---------3---------4----------5-------|  
         */
        
        // Check joining plan type.
        cav_msgs::PlanType plan_type = msg.plan_type;
        /**
         *  Note:
         *      JOIN_FROM_FRONT indicate a same-lane front join.
         *      JOIN_PLATOON_AT_REAR indicate a same-lane rear join.
         *      PLATOON_CUT_IN_JOIN indicate a cut-in join, which include three cut-in methods: cut-in front, cut-in middle, and cut-in rear.
         */
        bool isFrontJoin = plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_FROM_FRONT;
        bool isRearJoin = plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_AT_REAR;
        bool isCutInJoin = plan_type.type == cav_msgs::PlanType::PLATOON_CUT_IN_JOIN;
        bool isDepart = (plan_type.type == cav_msgs::PlanType::PLATOON_DEPARTURE);

        // Ignore the request if we are already working with a join/departure process or if no join type was requested (prevents multiple applicants)
        if (isFrontJoin  ||  isRearJoin  ||  isCutInJoin  ||  isDepart)
        {
            if (pm_.current_plan.valid){
                ROS_DEBUG_STREAM("Ignoring incoming request since we are already negotiating a join.");
                return MobilityRequestResponse::NO_RESPONSE; //TODO: replace with NACK that indicates to ask me later
            }
        }
        else
        {
            ROS_WARN_STREAM("Received request with bogus message type " << plan_type.type << "; ignoring");
            return MobilityRequestResponse::NO_RESPONSE;
        }

        // TODO: Generalize - We currently ignore the lane information for now and assume the applicant is in the same lane with us.
        // Determine intra-platoon conditions
        // We are currently checking two basic JOIN conditions:
        //     1. The size limitation on current platoon based on the plugin's parameters.
        //     2. Calculate how long that vehicle can be in a reasonable distance to actually join us.
        cav_msgs::MobilityHeader msgHeader = msg.m_header;
        std::string params = msg.strategy_params;
        std::string applicantId = msgHeader.sender_id;
        ROS_DEBUG_STREAM("Received mobility JOIN request from " << applicantId << " and PlanId = " << msgHeader.plan_id);
        ROS_DEBUG_STREAM("The strategy parameters are " << params);
        if (params.length() == 0)
        {
            ROS_DEBUG_STREAM("The strategy parameters are empty, return no response");
            return MobilityRequestResponse::NO_RESPONSE;
        }

        // The incoming message is "mobility Request", which has a location category.
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

        // Calculate downtrack (m) based on incoming pose. 
        lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(msg.location);
        double applicantCurrentDtd = wm_->routeTrackPos(incoming_pose).downtrack;
        ROS_DEBUG_STREAM("applicantCurrentmemberUpdates from ecef pose: " << applicantCurrentDtd);

        // Calculate crosstrack (m) based on incoming pose. 
        double applicantCurrentCtd = wm_->routeTrackPos(incoming_pose).crosstrack;
        ROS_DEBUG_STREAM("applicantCurrentCtd from ecef pose: " << applicantCurrentCtd);
        bool isInLane = abs(applicantCurrentCtd - current_crosstrack_) < config_.maxCrosstrackError;
        ROS_DEBUG_STREAM("isInLane = " << isInLane);
        
        // Check if we have enough room for that applicant
        int currentPlatoonSize = pm_.getHostPlatoonSize();
        bool hasEnoughRoomInPlatoon = applicantSize + currentPlatoonSize <= config_.maxPlatoonSize;

        // rear join; platoon leader --> leader waiting
        if (isRearJoin)
        {
            // Log the request  type
            ROS_DEBUG_STREAM("The received mobility JOIN request from " << applicantId << " and PlanId = " << msgHeader.plan_id << " is a same-lane REAR-JOIN request !");

            // -- core condition to decided accept joiner or not
            if (hasEnoughRoomInPlatoon && isInLane) 
            {
                ROS_DEBUG_STREAM("The current platoon has enough room for the applicant with size " << applicantSize);
                double currentRearDtd = pm_.getPlatoonRearDowntrackDistance();
                ROS_DEBUG_STREAM("The current platoon rear dtd is " << currentRearDtd);
                double currentGap = currentRearDtd - applicantCurrentDtd - config_.vehicleLength;
                double currentTimeGap = currentGap / applicantCurrentSpeed;
                ROS_DEBUG_STREAM("The gap between current platoon rear and applicant is " << currentGap << "m or " << currentTimeGap << "s");
                if (currentGap < config_.minAllowedJoinGap) 
                {
                    ROS_WARN("We should not receive any request from the vehicle in front of us. NACK it.");
                    return MobilityRequestResponse::NACK;
                }
                
                // Check if the applicant can join based on max timeGap/gap
                bool isDistanceCloseEnough = currentGap <= config_.maxAllowedJoinGap  ||  currentTimeGap <= config_.maxAllowedJoinTimeGap;
                if (isDistanceCloseEnough) 
                {
                    ROS_DEBUG_STREAM("The applicant is close enough and we will allow it to try to join");
                    ROS_DEBUG_STREAM("Change to LeaderWaitingState and waiting for " << msg.m_header.sender_id << " to join");

                    // change state to leaderwaiting !
                    pm_.current_platoon_state = PlatoonState::LEADERWAITING;
                    waitingStartTime = ros::Time::now().toNSec() / 1000000;
                    pm_.current_plan = ActionPlan(true, waitingStartTime, msgHeader.plan_id, applicantId);
                    pm_.platoonLeaderID = pm_.HostMobilityId;
                    return MobilityRequestResponse::ACK;
                }
                else 
                {
                    ROS_DEBUG_STREAM("The applicant is too far away from us. NACK.");
                    return MobilityRequestResponse::NACK; //TODO: add reason & request to try again
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
            // Log the request  type
            ROS_DEBUG_STREAM("The received mobility JOIN request from " << applicantId << " and PlanId = " << msgHeader.plan_id << " is a same-lane FRONT-JOIN request !");

            // -- core condition to decided accept joiner or not
            if (hasEnoughRoomInPlatoon && isInLane) 
            {
                ROS_DEBUG_STREAM("The current platoon has enough room for the applicant with size " << applicantSize);
                
                // UCLA: change to read platoon front info
                double currentFrontDtd = pm_.getPlatoonFrontDowntrackDistance();
                ROS_DEBUG_STREAM("The current platoon front dtd is " << currentFrontDtd);
                // UCLA: adjust for calculating gap between new leader and old leader
                double currentGap =  applicantCurrentDtd - currentFrontDtd - config_.vehicleLength;
                double currentTimeGap = currentGap / applicantCurrentSpeed;
                ROS_DEBUG_STREAM("The gap between current platoon front and applicant is " << currentGap << "m or " << currentTimeGap << "s");
                
                if (currentGap < config_.minAllowedJoinGap) 
                {
                    ROS_WARN("The current time gap is not suitable for frontal join. NACK it.");
                    return MobilityRequestResponse::NACK;
                }

                // Check if the applicant can join based on max timeGap/gap
                bool isDistanceCloseEnough = currentGap <= config_.maxAllowedJoinGap  ||  currentTimeGap <= config_.maxAllowedJoinTimeGap;

                // UCLA: add condition: only allow front join when host platoon size >= 2 (make sure when two single vehicle join, only use back join)
                bool isPlatoonNotSingle = pm_.getHostPlatoonSize() >= 2 || config_.test_front_join;

                if (isDistanceCloseEnough && isPlatoonNotSingle) 
                {
                    ROS_DEBUG_STREAM("The applicant is close enough for frontal join, send acceptance response");
                    ROS_DEBUG_STREAM("Change to LeaderAborting state and waiting for " << msg.m_header.sender_id << " to join as the new platoon leader");

                    // ----------------- give up leader position and look for new leader --------------------------

                    // adjust for frontal join. Platoon info is related to the platoon at back of the candidate leader vehicle.
                    // Don't want an action plan here
                    pm_.current_platoon_state = PlatoonState::LEADERABORTING;
                    candidatestateStartTime = ros::Time::now().toNSec() / 1000000;

                    // If we are testing with a single vehicle representing this platoon, then we don't yet have a platoon ID,
                    // so use the ID for the proposed joining action plan
                    if (config_.test_front_join)
                    {
                        pm_.currentPlatoonID = msgHeader.plan_id;
                    }

                    // Store the leader ID as that of the joiner to allow run_leader_aborting to work correctly
                    pm_.platoonLeaderID = applicantId;

                    waitingStartTime = ros::Time::now().toNSec() / 1000000;
                    pm_.current_plan.valid = false;
                    return MobilityRequestResponse::ACK;
                }
                else 
                {
                    ROS_DEBUG_STREAM("The joining gap (" << currentGap << " m) is too far away from us or the target platoon size (" << pm_.getHostPlatoonSize() << ") is one. NACK.");
                    return MobilityRequestResponse::NACK;  //TODO: add reason & request to try again
                }
            }
            else
            {
                ROS_DEBUG_STREAM("The current platoon does not have enough room for applicant of size " << applicantSize << ". NACK");
                return MobilityRequestResponse::NACK;
            }
        }
        
        // UCLA: conditions for cut-in join; platoon leader --> leading with operation
        else if (isCutInJoin)
        {
            // Log the request  type
            ROS_DEBUG_STREAM("The received mobility JOIN request from " << applicantId << " and PlanId = " << msgHeader.plan_id << " is a CUT-IN-JOIN request !");

            // -- core condition to decided accept joiner or not. It is necessary leader only process the first cut-in joining request.
            // Note: The host is the platoon leader, need to use a different method to determine if joining vehicle is nearby.
            if (hasEnoughRoomInPlatoon && isJoiningVehicleNearPlatoon(applicantCurrentDtd, applicantCurrentCtd))
            {
                ROS_DEBUG_STREAM("The current platoon has enough room for the applicant with size " << applicantSize);
                ROS_DEBUG_STREAM("The applicant is close enough for cut-in join, send acceptance response");
                ROS_DEBUG_STREAM("Change to Leading with operation state and waiting for " << msg.m_header.sender_id << " to change lane");
                // change state to lead with operation
                pm_.current_platoon_state = PlatoonState::LEADWITHOPERATION;
                waitingStartTime = ros::Time::now().toNSec() / 1000000;
                pm_.current_plan = ActionPlan(true, waitingStartTime, msgHeader.plan_id, applicantId);
                pm_.platoonLeaderID = pm_.HostMobilityId;
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
        
        // TODO: Place holder for deaprture.

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
        // TODO Maybe it should handle some ABORT request from a candidate leader
        ROS_DEBUG_STREAM("Received mobility request with type " << msg.plan_type.type << " but ignored.");
        return MobilityRequestResponse::NO_RESPONSE;
    }

    // UCLA: mobility request candidate leader (inherited from leader waiting)
    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_candidateleader(const cav_msgs::MobilityRequest& msg)
    {   
        bool isTargetVehicle = msg.m_header.sender_id == pm_.current_plan.peerId; // need to check: senderID (old leader)
        bool isCandidateJoin = msg.plan_type.type == cav_msgs::PlanType::PLATOON_FRONT_JOIN;

        lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(msg.location);
        double obj_cross_track = wm_->routeTrackPos(incoming_pose).crosstrack;
        bool inTheSameLane = abs(obj_cross_track - current_crosstrack_) < config_.maxCrosstrackError;
        ROS_DEBUG_STREAM("current_cross_track error = " << abs(obj_cross_track - current_crosstrack_));
        ROS_DEBUG_STREAM("obj_cross_track = " << obj_cross_track);
        ROS_DEBUG_STREAM("current_crosstrack_ = " << current_crosstrack_);
        ROS_DEBUG_STREAM("inTheSameLane = " << inTheSameLane);
        ROS_DEBUG_STREAM("isTargetVehicle = " << isTargetVehicle);
        ROS_DEBUG_STREAM("isCandidateJoin = " << isCandidateJoin);
        if (isCandidateJoin && inTheSameLane  &&  isTargetVehicle)
        {
            ROS_DEBUG_STREAM("Old platoon leader " << pm_.current_plan.peerId << " has agreed to joining.");
            ROS_DEBUG_STREAM("Changing to PlatoonLeaderState and send ACK to the previous leader vehicle");
            pm_.current_platoon_state = PlatoonState::LEADER;
            
            // Clean up planning info
            pm_.clearActionPlan();
            pm_.platoonLeaderID = config_.vehicleID;

            // Clean up neighbor platoon info since we just joined it
            pm_.resetNeighborPlatoon();

            return MobilityRequestResponse::ACK;
        }
        else
        {
            ROS_DEBUG_STREAM("Received platoon request with vehicle id = " << msg.m_header.sender_id);
            ROS_DEBUG_STREAM("The request type is " << msg.plan_type.type << " and we choose to ignore");
            pm_.clearActionPlan();
            pm_.resetHostPlatoon(); //ASSUMES host is a solo joiner

            // return to leader state as a solo vehicle
            pm_.current_platoon_state = PlatoonState::LEADER;
            return MobilityRequestResponse::NACK;
        }
    }
    
    // UCLA: add request call-back function for lead with operation state (for cut-in join)
    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_leadwithoperation(const cav_msgs::MobilityRequest& msg)
    {
        /*
        *   Current leader change state to lead with opertaion once the cut-in join request is accepeted. 
            For cut-in front, the leading vehicle will open gap for the joining vehicle.
            For cut-in middle, the leading vechile will request gap following vehcile to create gap.
            The host leading vheicle will also send lane cut-in approval response to the joining vehicle.
            The host leading vehicle will change to same-lane state once the lanechange completion plan was recieved. 
            
        */

        // Check request plan type  
        cav_msgs::PlanType plan_type = msg.plan_type;
        std::string strategyParams = msg.strategy_params;
        std::string reqSenderID = msg.m_header.sender_id;

        // Calculate downtrack (m) based on ecef. 
        lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(msg.location);
        // read downtrack
        double applicantCurrentDtd = wm_->routeTrackPos(incoming_pose).downtrack;
        ROS_DEBUG_STREAM("Applicant downtrack from ecef pose: " << applicantCurrentDtd);

        // Read requesting join index
        std::vector<std::string> inputsParams;
        boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));

        std::vector<std::string> join_index_parsed;
        boost::algorithm::split(join_index_parsed, inputsParams[5], boost::is_any_of(":"));
        int req_sender_join_index = std::stoi(join_index_parsed[1]);
        ROS_DEBUG_STREAM("Requesting join_index parsed: " << req_sender_join_index);

        if (plan_type.type == cav_msgs::PlanType::PLATOON_CUT_IN_JOIN) 
        {
            // Send response 
            // ----- CUT-IN front -----
            if (req_sender_join_index == -1)
            {
                // determine if joining vehicle in position for cut-in front - this value must be > 0
                double cutinDtdDifference = applicantCurrentDtd - current_downtrack_ - config_.vehicleLength;
                // check dtd between current leader (host) and joining vehicle is far enough away to avoid a collision
                bool isFrontJoinerInPosition = cutinDtdDifference >= 1.5*config_.vehicleLength;
            
                if (isFrontJoinerInPosition)
                {
                    ROS_DEBUG_STREAM("The joining vehicle is cutting in from front.  Gap is already sufficient.");
                    return MobilityRequestResponse::ACK;
                }
                else if (cutinDtdDifference > 0.0  &&  cutinDtdDifference < 1.5*config_.vehicleLength)
                {
                    // slow down leader to allow joiner cut-in
                    pm_.isCreateGap = true;
                    ROS_DEBUG_STREAM("The joining vehicle is cutting in from front.");
                    ROS_DEBUG_STREAM("Host (leader) slow down notified, joining vehicle can prepare to join");
                    return MobilityRequestResponse::ACK;
                }
                    
                else
                {
                    ROS_DEBUG_STREAM("Front join geometry violation. NACK.  cutinDtdDifference = " << cutinDtdDifference);
                    pm_.current_platoon_state = PlatoonState::LEADER;
                    return MobilityRequestResponse::NACK;
                }
            }

            // ----- CUT-IN rear -----
            else if (static_cast<size_t>(req_sender_join_index) == pm_.host_platoon_.size()-1)
            {
                // determine if joining vehicle in position for cut-in rear
                // To pass, the joining vehicle need to be behind the last member, within three vehicle length.
                //TODO: should it also test CTE to ensure vehicle is in same lane?
                double platoonEndVehicleDtd = pm_.host_platoon_[pm_.host_platoon_.size()-1].vehiclePosition - config_.vehicleLength;
                double rearGap = platoonEndVehicleDtd - applicantCurrentDtd;
                bool isRearJoinerInPosition = rearGap >= 0  &&  rearGap <= config_.maxCutinGap;//3*config_.vehicleLength; TODO: temporary increase
                        
                if (isRearJoinerInPosition)
                {
                    ROS_DEBUG_STREAM("Published Mobility cut-in-rear-Join request to relavent platoon member, host is leader.");
                    ROS_WARN("Published Mobility cut-in-rear-Join request to relavent platoon members to signal gap creation.");
                    pm_.isCreateGap = true;
                    return MobilityRequestResponse::ACK;
                }
                else
                {
                    ROS_DEBUG_STREAM("Rear join geometry violation. NACK. rearGap = " << rearGap);
                    pm_.current_platoon_state = PlatoonState::LEADER;
                    return MobilityRequestResponse::NACK;
                }
            }
            // ----- CUT-IN middle -----
            else //any other join_index value
            {
                // determine if joining vehicle in position for cut-in mid
                // To pass, the joining vehicle should be in front of the gap following vehicle, within three vehicle length.  
                double gapFollowingVehicleDtd = pm_.host_platoon_[req_sender_join_index].vehiclePosition;
                double gapFollowerDiff = applicantCurrentDtd - gapFollowingVehicleDtd;
                bool isMidJoinerInPosition = gapFollowerDiff >=0  &&  gapFollowerDiff <= 3*config_.vehicleLength; 
            
                // Task 4: send request to index member to slow down (i.e., set isCreateGap to true) 
                // Note: Request recieving vehicle set pm_.isCreateGap 
                if (isMidJoinerInPosition)
                {
                    // compose request (Note: The recipient should be the gap following vehicle.)
                    cav_msgs::MobilityRequest request;
                    request.m_header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
                    // Note: For cut-in mid, notify gap rear member to create/increase gap.
                    std::string recipient_ID = pm_.host_platoon_[req_sender_join_index+1].staticId;
                    request.m_header.sender_id = config_.vehicleID;
                    request.m_header.timestamp = ros::Time::now().toNSec() / 1000000;;
                    // UCLA: add plan type, add this in cav_mwgs/plan_type
                    request.plan_type.type = cav_msgs::PlanType::PLATOON_CUT_IN_JOIN;
                    request.strategy = PLATOONING_STRATEGY;

                    double platoon_size = pm_.getHostPlatoonSize();

                    boost::format fmter(JOIN_PARAMS);   // Note: Front and rear join uses same params, hence merge to one param for both condition.
                    fmter %platoon_size;                //  index = 0
                    fmter %current_speed_;              //  index = 1, in m/s
                    fmter %pose_ecef_point_.ecef_x;     //  index = 2
                    fmter %pose_ecef_point_.ecef_y;     //  index = 3
                    fmter %pose_ecef_point_.ecef_z;     //  index = 4     
                    fmter %req_sender_join_index;       //  index = 5

                    request.strategy_params = fmter.str();
                    request.urgency = 50;
                    request.location = pose_to_ecef(pose_msg_);
                    // note: for rear join, cut-in index == host_platoon_.size()-1; for join from front, index == -1
                    //       for cut-in in middle, index indicate the gap leading vehicle's index
                    mobility_request_publisher_(request); 
                    ROS_DEBUG_STREAM("Published Mobility cut-in-mid-Join request to relavent platoon members to signal gap creation, host is leader.");
                    ROS_WARN("Published Mobility cut-in-mid-Join request to relavent platoon members to signal gap creation.");
                    ROS_DEBUG_STREAM("The joining vehicle is cutting in at index: "<< req_sender_join_index <<". Notify gap rear vehicle with ID: " << recipient_ID << " to slow down");
                    return MobilityRequestResponse::ACK;
                }
                else
                {
                    ROS_DEBUG_STREAM("Mid join geometry violation. NACK. gapFollwerDiff = " << gapFollowerDiff);
                    pm_.current_platoon_state = PlatoonState::LEADER;
                    return MobilityRequestResponse::NACK;
                }
            }
        }

        // task 2: For cut-in from front, the leader need to stop creating gap
        else if (plan_type.type == cav_msgs::PlanType::STOP_CREATE_GAP) 
        {
            // reset create gap indicator
            pm_.isCreateGap = false;
            // no need to response, simple reset the indicator
            return MobilityRequestResponse::NO_RESPONSE;
        }

        // task 3 cut-in front: After creating gap, revert back to same-lane operation 
        else if (plan_type.type == cav_msgs::PlanType::CUT_IN_FRONT_DONE)
        {
            ROS_DEBUG_STREAM("Cut-in from front lane change finished, leader revert to same-lane maneuver.");
            pm_.current_platoon_state = PlatoonState::LEADERABORTING;
            candidatestateStartTime = ros::Time::now().toNSec() / 1000000;
            // if testing with two vehicles, use plan id as platoon id
            if (pm_.currentPlatoonID.compare(pm_.dummyID) == 0)
            // if (config_.allowCutinJoin)
            {
                pm_.currentPlatoonID = msg.m_header.plan_id;
            }
            // Store the leader ID as that of the joiner to allow run_leader_aborting to work correctly
            pm_.platoonLeaderID = msg.m_header.sender_id;
            pm_.current_plan.valid = false;
            return MobilityRequestResponse::ACK;
        }

        // task 4 cut-in from middle/rear
        else if (plan_type.type == cav_msgs::PlanType::CUT_IN_MID_OR_REAR_DONE)
        {
            ROS_DEBUG_STREAM("Cut-in from mid/rear lane change finished, leader revert to same-lane maneuver.");
            pm_.current_platoon_state = PlatoonState::LEADERWAITING;
            waitingStartTime = ros::Time::now().toNSec() / 1000000;
            return MobilityRequestResponse::ACK;
        }


        // task 5: if other joining vehicle send joning request, NACK it since there is already a cut-in join going on.
        else
        {
            ROS_DEBUG_STREAM("CUT-IN join maneuver is already in operation, NACK incoming join requests from other candidates.");
            ROS_DEBUG_STREAM("Plan Type: " << plan_type.type );
            return MobilityRequestResponse::NACK;
        }

        // this statement should never be reached, but will ensure reasonable behavior in case of coding error above
        ROS_WARN_STREAM("End of method reached! Apparent logic fault above.");
        ROS_DEBUG_STREAM("End of method reached! Apparent logic fault above."); //since WARN doesn't always print
        return MobilityRequestResponse::NO_RESPONSE;
    }

    // UCLA: add request call-back function for prepare to join (for cut-in join)
    MobilityRequestResponse PlatoonStrategicIHPPlugin::mob_req_cb_preparetojoin(const cav_msgs::MobilityRequest& msg)
    {
        // This state does not handle any mobility request for now
        // TODO: if joining vehicle need to adjust speed, the leader should request it and the request should be handled here. 
        ROS_DEBUG_STREAM("Received mobility request with type " << msg.plan_type.type << " but ignored.");
        return MobilityRequestResponse::NO_RESPONSE;
    }

    // TODO: Place holder for departure

    // ------ 4. Mobility response callback ------ //
    
    // Mobility response callback for all states.
    void PlatoonStrategicIHPPlugin::mob_resp_cb(const cav_msgs::MobilityResponse& msg)
    {
        // Firstly, check eligibility of the received message. 
        bool isCurrPlanValid = pm_.current_plan.valid;                          // Check if current plan is still valid (i.e., not timed out).
        bool isForCurrentPlan = msg.m_header.plan_id == pm_.current_plan.planId;  // Check if plan Id matches.
        bool isFromTargetVehicle = msg.m_header.sender_id == pm_.current_plan.peerId;  // Check if expected peer ID and sender ID matches.
        ROS_DEBUG_STREAM("mob_resp_cb: isCurrPlanValid = " << isCurrPlanValid << ", isForCurrentPlan = " << 
                        isForCurrentPlan << ", isFromTargetVehicle = " << isFromTargetVehicle);
        ROS_DEBUG_STREAM("sender ID = " << msg.m_header.sender_id << ", current peer ID = " << pm_.current_plan.peerId);
        ROS_DEBUG_STREAM("incoming plan ID = " << msg.m_header.plan_id << "current plan ID = " << pm_.current_plan.planId);

        if (!(isCurrPlanValid && isForCurrentPlan && isFromTargetVehicle)) 
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
        // TODO: Place holder for departure.
    }

    void PlatoonStrategicIHPPlugin::mob_resp_cb_standby(const cav_msgs::MobilityResponse& msg)
    {
        // In standby state, it will not send out any requests so it will also ignore all responses
        ROS_DEBUG_STREAM("STANDBY state does nothing with msg from " << msg.m_header.sender_id);
    }

    void PlatoonStrategicIHPPlugin::mob_resp_cb_candidatefollower(const cav_msgs::MobilityResponse& msg)
    {
        ROS_DEBUG_STREAM("Callback for candidate follower ");
        
        // Check if current plan is still valid (i.e., not timed out)
        if (pm_.current_plan.valid)
        {
            bool isForCurrentPlan = msg.m_header.plan_id == pm_.current_plan.planId;
            ROS_DEBUG_STREAM("isForCurrentPlan " << isForCurrentPlan);

            // Check the response is received correctly (i.e., host vehicle is the desired receiver).
            if (isForCurrentPlan)
            {
                if (msg.is_accepted)
                {
                    // We change to follower state and start to actually follow that leader
                    // The platoon manager also need to change the platoon Id to the one that the target leader is using 
                    pm_.current_platoon_state = PlatoonState::FOLLOWER;
                    ROS_DEBUG_STREAM("pm_.currentPlatoonID: " << pm_.currentPlatoonID);
                    ROS_DEBUG_STREAM("pm_.targetPlatoonID: " << pm_.currentPlatoonID);

                    if (pm_.targetPlatoonID.compare(pm_.dummyID) != 0)
                    {
                        pm_.currentPlatoonID = pm_.targetPlatoonID;
                        ROS_DEBUG_STREAM("pm_.currentPlatoonID now: " << pm_.currentPlatoonID);
                        pm_.resetNeighborPlatoon();
                    }
                    else
                    {
                        pm_.currentPlatoonID = msg.m_header.plan_id;
                    }

                    pm_.changeFromLeaderToFollower(pm_.currentPlatoonID, msg.m_header.sender_id);
                    ROS_DEBUG_STREAM("The leader " << msg.m_header.sender_id << " agreed on our join. Change to follower state.");
                    ROS_WARN("changed to follower");
                    pm_.clearActionPlan();
                }
                else
                {
                    // We change back to normal leader state and try to join other platoons
                    ROS_DEBUG_STREAM("The leader " << msg.m_header.sender_id << " does not agree on our join. Change back to leader state.");
                    ROS_DEBUG_STREAM("Trying again..");
                    // join plan failed, but we still need the peerid
                    pm_.current_plan.valid = false;

                    // Clear out any platooning plan we don't need
                    if (pm_.getHostPlatoonSize() == 1)
                    {
                        pm_.resetHostPlatoon();
                    }
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
        ROS_DEBUG_STREAM("LEADERWAITING state does nothing with msg from " << msg.m_header.sender_id);
    }

    void PlatoonStrategicIHPPlugin::mob_resp_cb_follower(const cav_msgs::MobilityResponse& msg)
    {   
        /**
         * UCLA Note: 
         * 
         * This method was implemented with the purpose of updating the platoon related references 
         * (i.e., platoon leader, platoon Id) to the new leader that joined from front. Changing 
         * the existing follower to candidate follower state will initiate a member update and 
         * therefore point all refernce to the new leader.
         * 
         * For rear join, since the existing follower already following the platoon leader. There is 
         * no need to establish communication hence no response will be handled for rear-join.
         */ 

        // UCLA: read plan type 
        cav_msgs::PlanType plan_type = msg.plan_type;
        
        // UCLA: determine joining type 
        bool isFrontJoin = (plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_FROM_FRONT);

        //TODO: when would this code block ever be used? A normal follower would have to talk to a front joiner.
        // UCLA: add response so follower can change to candidate follower, then change leader
        if (isFrontJoin && msg.is_accepted)
        {   
            // if frontal join is accepted, change followers to candidate follower to update leader
            ROS_DEBUG_STREAM("Received positive response for front-join plan id = " << pm_.current_plan.planId);
            ROS_DEBUG_STREAM("Change to CandidateFollower state and prepare to update platoon information");
            // Change to candidate follower state and request a new plan to catch up with the front platoon
            pm_.current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
            candidatestateStartTime = ros::Time::now().toNSec() / 1000000;
        }
        
        // TODO: Place holder for follower departure.
    }

    // UCLA: add conditions to account for frontal join states (candidate follower) 
    void PlatoonStrategicIHPPlugin::mob_resp_cb_leader(const cav_msgs::MobilityResponse& msg)
    {   
        /**  
         *  UCLA implementation note:
         *  This is where the Mobility response gets processed for leader state. 
         *  
         *  If the host is a single vehicle in the leader state, then the host vehicle is the 
         *  joiner vehicle (frontal join: candidate leader; back join: candidate follower), and 
         *  the response sender is the existing platoon leader (front join: aborting leader, back join: waiting leader). 
         * 
         *  If the host is the current platoon leader, all three case will be false and no further action is needed.
         * 
         *  Disclaimer: Currently, if the host vehicle is platoon leader, there is no further action needed
         *  when receiving the mobility response. However, future development may add functions in this mehtod.
         */

        // UCLA: read plan type 
        cav_msgs::PlanType plan_type = msg.plan_type;
        ROS_DEBUG_STREAM("plan_type = " << plan_type);
        ROS_DEBUG_STREAM("plan_type.type = " << plan_type.type);
        
        // UCLA: determine joining type 
        bool isCutInJoin = plan_type.type == cav_msgs::PlanType::PLATOON_CUT_IN_JOIN      &&  !config_.test_front_join;
        bool isRearJoin = plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_AT_REAR      &&  !config_.test_front_join;
        bool isFrontJoin = plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_FROM_FRONT  ||  config_.test_front_join;
        ROS_DEBUG_STREAM("Joining type: isRearJoin = " << isRearJoin);
        ROS_DEBUG_STREAM("Joining type: isFrontJoin = " << isFrontJoin);
        ROS_DEBUG_STREAM("Joining type: isCutInJoin = " << isCutInJoin);
        
        // Check if current plan is still valid (i.e., not timed out).
        if (pm_.current_plan.valid)
        {
            ROS_DEBUG_STREAM("My plan id = " << pm_.current_plan.planId << " and response plan Id = " << msg.m_header.plan_id);
            ROS_DEBUG_STREAM("Expected peer id = " << pm_.current_plan.peerId << " and response sender Id = " << msg.m_header.sender_id);

            // Check the response is received correctly (i.e., host vehicle is the desired receiver).
            if (pm_.current_plan.planId == msg.m_header.plan_id && pm_.current_plan.peerId == msg.m_header.sender_id) 
            {   
                // rear join
                if (isRearJoin && msg.is_accepted)
                {
                    ROS_DEBUG_STREAM("Received positive response for plan id = " << pm_.current_plan.planId);
                    ROS_DEBUG_STREAM("Change to CandidateFollower state and notify trajectory failure in order to replan");

                    // Change to candidate follower state and wait to catch up with the front platoon
                    pm_.current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
                    candidatestateStartTime = ros::Time::now().toNSec() / 1000000;
                    pm_.current_plan.valid = false; //but leave peerId intact for use in second request
                }

                // UCLA: frontal join (candidate leader, inherited from leaderwaiting)
                else if (isFrontJoin && msg.is_accepted)
                {   
                    ROS_DEBUG_STREAM("Received positive response for plan id = " << pm_.current_plan.planId);
                    ROS_DEBUG_STREAM("Change to CandidateLeader state and prepare to become new leader. ");

                    // Change to candidate leader and idle
                    pm_.current_platoon_state = PlatoonState::CANDIDATELEADER;
                    candidatestateStartTime = ros::Time::now().toNSec() / 1000000;
                    pm_.current_plan.valid = false; //but leave peerId intact for use in second request

                    // Set the platoon ID to that of the target platoon even though we haven't yet joined;
                    // for front join this is necessary for the aborting leader to recognize us as an incoming
                    // member (via our published op STATUS messages)
                    pm_.currentPlatoonID = pm_.targetPlatoonID;
                }

                // UCLA: CutIn join 
                else if (isCutInJoin && msg.is_accepted)
                {
                    ROS_DEBUG_STREAM("Received positive response for plan id = " << pm_.current_plan.planId);
                    ROS_DEBUG_STREAM("Change to Prepare to join state and prepare to change lane. ");

                    // Change to candidate leader and idle
                    pm_.current_platoon_state = PlatoonState::PREPARETOJOIN;
                    pm_.neighbor_platoon_leader_id_ = msg.m_header.sender_id;
                    candidatestateStartTime = ros::Time::now().toNSec() / 1000000;
                    pm_.current_plan.valid = false; //but leave peerId intact for use in second request
                }

                // Current leader of an actual platoon (to be in this method host is in leader state)
                else if(pm_.getHostPlatoonSize() >= 2)
                {
                    //TODO future: add logic here to allow two platoons to join together

                    // Keep the leader idling, since this must be a bogus response
                    ROS_WARN_STREAM("Host received response for joining vehicles, remain idling as the host is a current platoon leader.");
                }
                else
                {
                    ROS_DEBUG_STREAM("Received negative response for plan id = " << pm_.current_plan.planId << ". Resetting plan & platoon info.");
                    // Forget about the previous plan totally
                    pm_.clearActionPlan();
                    pm_.resetHostPlatoon();
                }
            }
            else
            {
                ROS_DEBUG_STREAM("Ignore the response message because planID match: " << (pm_.current_plan.planId == msg.m_header.plan_id));
                ROS_DEBUG_STREAM("My plan id = " << pm_.current_plan.planId << " and response plan Id = " << msg.m_header.plan_id);
                ROS_DEBUG_STREAM("And peer id match " << (pm_.current_plan.peerId == msg.m_header.sender_id));
                ROS_DEBUG_STREAM("Expected peer id = " << pm_.current_plan.peerId << " and response sender Id = " << msg.m_header.sender_id);
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
         */   
        
        ROS_DEBUG_STREAM("Callback for leader aborting !");

        // Check if current plan is still valid (i.e., not timed out).
        if (pm_.current_plan.valid)
        {
            bool isForCurrentPlan = msg.m_header.plan_id == pm_.current_plan.planId;
            bool isForFrontJoin = msg.plan_type.type == cav_msgs::PlanType::PLATOON_FRONT_JOIN;

            if (msg.plan_type.type == cav_msgs::PlanType::UNKNOWN){
                ROS_DEBUG_STREAM("*** plan type UNKNOWN");
            }else if (msg.plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_FROM_FRONT){
                ROS_DEBUG_STREAM("*** plan type JOIN_PLATOON_FROM_FRONT");
            }else if (msg.plan_type.type == cav_msgs::PlanType::PLATOON_CUT_IN_JOIN){
                ROS_DEBUG_STREAM("*** plan type PLATOON_CUT_IN_JOIN");
            }else {
                ROS_DEBUG_STREAM("*** plan type not captured.");
            }

            bool isFromTargetVehicle = msg.m_header.sender_id == pm_.current_plan.peerId;
            ROS_DEBUG_STREAM("msg.header.sender_id " << msg.m_header.sender_id);
            ROS_DEBUG_STREAM("Plan Type " << msg.plan_type.type);
            ROS_DEBUG_STREAM("isForFrontJoin " << isForFrontJoin);
            ROS_DEBUG_STREAM("isForCurrentPlan " << isForCurrentPlan);
            ROS_DEBUG_STREAM("isFromTargetVehicle " << isFromTargetVehicle);

            // Check the response is received correctly (i.e., host vehicle is the desired receiver).
            if (isForCurrentPlan && isFromTargetVehicle && isForFrontJoin)
            {
                if (msg.is_accepted)
                {
                    // We change to follower state and start to actually follow the new leader
                    // The platoon manager also need to change the platoon Id to the one that the target leader is using                
                    pm_.current_platoon_state = PlatoonState::FOLLOWER;
                    pm_.changeFromLeaderToFollower(pm_.currentPlatoonID, msg.m_header.sender_id);
                    ROS_DEBUG_STREAM("The new leader " << msg.m_header.sender_id << " agreed on the frontal join. Change to follower state.");
                    ROS_WARN("changed to follower");

                    // reset leader aborting request marker
                    numLeaderAbortingCalls_ = 0;
                }
                else
                {
                    // We change back to normal leader state
                    ROS_DEBUG_STREAM("The new leader " << msg.m_header.sender_id << " does not agree on the frontal join. Change back to leader state.");
                    pm_.current_platoon_state = PlatoonState::LEADER;
                    // We were already leading a platoon, so don't erase any of that info. But we need to remove the erstwhile candidate
                    // leader from our platoon roster; we know it is in position 0, so just remove that element
                    if (!pm_.removeMember(0))
                    {
                        ROS_DEBUG_STREAM("Failed to remove candidate leader from the platoon!");
                    }
                }

                // Clean up the joining plan
                pm_.clearActionPlan();
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
        ROS_DEBUG_STREAM("CANDIDATELEADER state does nothing with msg from " << msg.m_header.sender_id);
    }

    // UCLA: response callback for lead with operation
    void PlatoonStrategicIHPPlugin::mob_resp_cb_leadwithoperation(const cav_msgs::MobilityResponse& msg)
    { 
        ROS_DEBUG_STREAM("LEADWITHOPERATION state does nothing with msg from " << msg.m_header.sender_id);
    }

    // UCLA: response callback for prepare to join (inherited from leader waiting)
    void PlatoonStrategicIHPPlugin::mob_resp_cb_preparetojoin(const cav_msgs::MobilityResponse& msg)
    {
        /*
            If leader notify the member to slow down and ACK the request,
            start to check the gap and change lane when gap is large enough 
        */

        cav_msgs::PlanType plan_type = msg.plan_type;
        bool isCreatingGap = plan_type.type == cav_msgs::PlanType::PLATOON_CUT_IN_JOIN;
        bool isFinishLaneChangeFront = plan_type.type == cav_msgs::PlanType::CUT_IN_FRONT_DONE; 
        bool isFinishLaneChangeMidorRear = plan_type.type == cav_msgs::PlanType::CUT_IN_MID_OR_REAR_DONE;
        ROS_DEBUG_STREAM("isCreatingGap = " << isCreatingGap << ", is_neighbor_record_complete = " << pm_.is_neighbor_record_complete_);

        if (!msg.is_accepted)
        {
            ROS_DEBUG_STREAM("Request " << msg.m_header.plan_id << " was rejected by leader.");
            ROS_DEBUG_STREAM("Action Plan reset.");
            ROS_DEBUG_STREAM("Trying again....");
            pm_.current_plan.valid = false;
            pm_.current_platoon_state = PlatoonState::LEADER;
            return;
        }
        // UCLA: Create Gap or perform a rear join (no gap creation necessary)
        ROS_DEBUG_STREAM("pm_.is_neighbor_record_complete_ " << pm_.is_neighbor_record_complete_);
        if (isCreatingGap  &&  pm_.is_neighbor_record_complete_)
        {
            // task 1: check gap 
            double cut_in_gap = pm_.getCutInGap(target_join_index_, current_downtrack_);   
            // cut-in gap not needed for front and rear join, so ignored
                   
            // task 2: set indicator if gap is safe
            safeToLaneChange_ = true;
            ROS_DEBUG_STREAM("Gap is now sufficiently large.");
            ROS_DEBUG_STREAM("in mob_resp_cb safeToLaneChange_: " << safeToLaneChange_);

            // task 3: notify gap-rear vehicle to stop slowing down
            cav_msgs::MobilityRequest request;
            request.m_header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
            request.m_header.recipient_id = pm_.current_plan.peerId;
            request.m_header.sender_id = config_.vehicleID;
            request.m_header.timestamp = ros::Time::now().toNSec() / 1000000;;
            // UCLA: A new plan type to stop creat gap.
            request.plan_type.type = cav_msgs::PlanType::STOP_CREATE_GAP;
            request.strategy = PLATOONING_STRATEGY;
            request.urgency = 50;
            request.location = pose_to_ecef(pose_msg_);
            double platoon_size = pm_.getHostPlatoonSize(); 

            boost::format fmter(JOIN_PARAMS);
            fmter %platoon_size;                    //  index = 0
            fmter %current_speed_;                  //  index = 1, in m/s
            fmter %pose_ecef_point_.ecef_x;         //  index = 2
            fmter %pose_ecef_point_.ecef_y;         //  index = 3
            fmter %pose_ecef_point_.ecef_z;         //  index = 4   
            fmter %target_join_index_;              //  index = 5

            request.strategy_params = fmter.str();
            mobility_request_publisher_(request); 
            ROS_DEBUG_STREAM("Published Mobility Candidate-Join request to the leader to stop creating gap");
        }

        // UCLA: Revert to same-lane for cut-in front 
        else if (isFinishLaneChangeFront)
        {
            ROS_DEBUG_STREAM("Cut-in from front lane change finished, the joining vehicle revert to same-lane maneuver.");
            pm_.current_platoon_state = PlatoonState::CANDIDATELEADER;
            candidatestateStartTime = ros::Time::now().toNSec() / 1000000;
            ROS_DEBUG_STREAM("pm_.currentPlatoonID: " << pm_.currentPlatoonID);
            ROS_DEBUG_STREAM("pm_.targetPlatoonID: " << pm_.targetPlatoonID);
            if (pm_.targetPlatoonID.compare(pm_.dummyID) != 0)
            {
                pm_.currentPlatoonID = pm_.targetPlatoonID;
                ROS_DEBUG_STREAM("pm_.currentPlatoonID now: " << pm_.currentPlatoonID);
            }
            
            pm_.current_plan.valid = false; //but leave peerId intact for use in second request
        }

        // UCLA: Revert to same-lane operation for cut-in from middle/rear 
        else if (isFinishLaneChangeMidorRear)
        {
            ROS_DEBUG_STREAM("Cut-in from mid or rear, the lane change finished, the joining vehicle revert to same-lane maneuver.");
            pm_.current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
            candidatestateStartTime = ros::Time::now().toNSec() / 1000000;
            pm_.current_plan.valid = false; //but leave peerId intact for use in second request

        } 

        else
        {
            ROS_DEBUG_STREAM("End of mob_resp_cb_preparetojoin");
        }
    }

    // TODO: Place holder for departure.
    
    // ------ 5. response types ------- //

    // ACK --> yes,accept host as member; NACK --> no, cannot accept host as member
    void PlatoonStrategicIHPPlugin::mob_req_cb(const cav_msgs::MobilityRequest& msg)
    {
        // Ignore messages as long as host vehicle is stopped
        if (current_speed_ < config_.minPlatooningSpeed)
        {
            ROS_DEBUG_STREAM("Ignoring message since host speed is below platooning speed.");
            return;
        }
        
        // Check that this is a message about platooning (could be from some other Carma activity nearby)
        std::string strategy = msg.strategy;
        if (strategy.rfind(PLATOONING_STRATEGY, 0) != 0)
        {
            ROS_DEBUG_STREAM("Ignoring mobility operation message for " << strategy << " strategy.");
            return;
        }

        cav_msgs::MobilityResponse response;
        response.m_header.sender_id = config_.vehicleID;
        response.m_header.recipient_id = msg.m_header.sender_id;
        response.m_header.plan_id = msg.m_header.plan_id;
        response.m_header.timestamp = ros::Time::now().toNSec() / 1000000;

       // UCLA: add plantype in response 
        response.plan_type.type = msg.plan_type.type;
        
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
            pm_.clearActionPlan();
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
        unsigned long tsStart = ros::Time::now().toNSec() / 1000000;

        // If vehicle is not rolling then return
        if (current_speed_ <= STOPPED_SPEED)
        {
            return;
        }

        // Task 1: heart beat timeout: send INFO mob_op
        bool isTimeForHeartBeat = tsStart - prevHeartBeatTime_ >= infoMessageInterval_;
        ROS_DEBUG_STREAM("time since last heart beat: " << tsStart - prevHeartBeatTime_);
        if (isTimeForHeartBeat) 
        {
            cav_msgs::MobilityOperation infoOperation;
            infoOperation = composeMobilityOperationLeader(OPERATION_INFO_TYPE);
            mobility_operation_publisher_(infoOperation);
            prevHeartBeatTime_ = ros::Time::now().toNSec() / 1000000;
            ROS_DEBUG_STREAM("Published heart beat platoon INFO mobility operation message");
        }

        // Task 3: plan time out, check if any current join plan is still valid (i.e., not timed out).
        if (pm_.current_plan.valid)
        {
            bool isCurrentPlanTimeout = tsStart - pm_.current_plan.planStartTime > NEGOTIATION_TIMEOUT;
            if (isCurrentPlanTimeout)
            {
                ROS_DEBUG_STREAM("Give up current on waiting plan with planId: " << pm_.current_plan.planId);
                pm_.clearActionPlan();
            }
        }

        // Task 4: STATUS msgs
        bool hasFollower = pm_.getHostPlatoonSize() > 1  ||  config_.test_cutin_join;
        ROS_DEBUG_STREAM("hasFollower" << hasFollower);
        // if has follower, publish platoon message as STATUS mob_op
        if (hasFollower)
        {
            cav_msgs::MobilityOperation statusOperation;
            statusOperation = composeMobilityOperationLeader(OPERATION_STATUS_TYPE);
            // mob_op_pub_.publish(statusOperation);
            mobility_operation_publisher_(statusOperation);
            ROS_DEBUG_STREAM("Published platoon STATUS operation message as a Leader with Follower");
        }

        long tsEnd = ros::Time::now().toNSec() / 1000000;
        long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
        ros::Duration(sleepDuration / 1000).sleep();

        // Job 5: Dissoleve request. 
        // TODO: Place holder for departure. Need to change to departing state and tracking departng ID.

    }

    void PlatoonStrategicIHPPlugin::run_follower()
    {
        ROS_DEBUG_STREAM("run follower");
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
                noLeaderUpdatesCounter = 0;
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

        // Job 3: Dissoleve request. 
        //TODO: set departure indicator 

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
            pm_.clearActionPlan();
        }

        // Task 2: plan timeout, check if current plan is still valid (i.e., not timed out).   
        if (pm_.current_plan.valid) 
        {
            ROS_DEBUG_STREAM("pm_.current_plan.planStartTime: " << pm_.current_plan.planStartTime);
            ROS_DEBUG_STREAM("timeout2: " << tsStart - pm_.current_plan.planStartTime);
            ROS_DEBUG_STREAM("NEGOTIATION_TIMEOUT: " << NEGOTIATION_TIMEOUT);
            bool isPlanTimeout = tsStart - pm_.current_plan.planStartTime > NEGOTIATION_TIMEOUT;
            if (isPlanTimeout) 
            {
                pm_.current_platoon_state = PlatoonState::LEADER;
                pm_.clearActionPlan();
                ROS_DEBUG_STREAM("The current plan did not receive any response. Abort and change to leader state.");
                ROS_DEBUG_STREAM("Changed the state back to Leader");
            }
        }

        // Task 3: update plan calculate gap, update plan: send PLATOON_FOLLOWER_JOIN request with new gap
        double desiredJoinGap2 = config_.desiredJoinTimeGap * current_speed_;
        double maxJoinGap = std::max(config_.desiredJoinGap, desiredJoinGap2);
        double currentGap = 0.0;
        if (!pm_.neighbor_platoon_.empty())
        {
            currentGap = pm_.neighbor_platoon_.back().vehiclePosition - current_downtrack_;
            ROS_DEBUG_STREAM("curent gap calculated from back of neighbor platoon: " << currentGap);
            ROS_DEBUG_STREAM("pm_.neighbor_platoon_.back().vehiclePosition " << pm_.neighbor_platoon_.back().vehiclePosition);
       
        }
        else
        {
            currentGap = pm_.getDistanceToPredVehicle();
            ROS_DEBUG_STREAM("curent gap when there is no neighbor platoon: " << currentGap);
        }

        ROS_DEBUG_STREAM("Based on desired join time gap, the desired join distance gap is " << desiredJoinGap2 << " ms");
        ROS_DEBUG_STREAM("Since we have max allowed gap as " << config_.desiredJoinGap << " m then max join gap became " << maxJoinGap << " m");
        ROS_DEBUG_STREAM("The current gap from radar is " << currentGap << " m");
        if (currentGap <= maxJoinGap  &&  !pm_.current_plan.valid)
        {
            cav_msgs::MobilityRequest request;
            std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
            long currentTime = ros::Time::now().toNSec() / 1000000;
            request.m_header.plan_id = planId;
            request.m_header.recipient_id = pm_.current_plan.peerId;
            request.m_header.sender_id = config_.vehicleID;
            request.m_header.timestamp = currentTime;

            request.plan_type.type = cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN;
            request.strategy = PLATOONING_STRATEGY;
            request.strategy_params = ""; //params will not be read by receiver since this is 2nd msg in sequence
            request.urgency = 50;
            request.location = pose_to_ecef(pose_msg_);
            mobility_request_publisher_(request);
            ROS_DEBUG_STREAM("Published Mobility Candidate-Join request to the leader");
            ROS_DEBUG_STREAM("current plan peer id: " << pm_.current_plan.peerId);

            // Update the local record of the new activity plan and now establish that we have a platoon plan as well,
            // which allows us to start sending necessary op STATUS messages
            pm_.current_plan = ActionPlan(true, currentTime, planId, pm_.current_plan.peerId);
            pm_.platoonLeaderID = pm_.current_plan.peerId;

            // Initialize counter to delay transmission of first operation STATUS message
            candidate_follower_delay_count_ = 0;
        }

        //Task 4: publish platoon status message (as single joiner)
        if (pm_.current_plan.valid) 
        {
            // Don't want to do this until after the above MobReq message is delivered, otherwise recipient will double-count us in their platoon
            if (++candidate_follower_delay_count_ > 2)
            {
                cav_msgs::MobilityOperation status;
                status = composeMobilityOperationCandidateFollower();
                mobility_operation_publisher_(status);
                ROS_DEBUG_STREAM("Published platoon STATUS operation message as Candidate Follower");
            }
        }

        long tsEnd = ros::Time::now().toNSec() / 1000000;
        long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
        ros::Duration(sleepDuration / 1000).sleep();
    }

    // UCLA: frontal join state (inherit from candidate follower: prepare to give up leading state and accept the new leader)
    void PlatoonStrategicIHPPlugin::run_leader_aborting() 
    {
        /*  
            UCLA implementation note:
            1. this function  send step plan type: "PLATOON_FRONT_JOIN"
            2. the sender of the plan (host vehicle) is the previous leader, it wil prepare to follow front joiner (new leader)
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

            //clear plan validity & end; leave platoon info alone, as we may still be leading a valid platoon
            pm_.clearActionPlan();
            return;
        }

        // Task 3: update plan: PLATOON_FRONT_JOIN with new gap
        double desiredJoinGap2 = config_.desiredJoinTimeGap * current_speed_;
        double maxJoinGap = std::max(config_.desiredJoinGap, desiredJoinGap2);

        // check if compatible for front join --> return front gap, no veh type check, is compatible
        // Note that pm_ only represents host's platoon members. As the aborting leader, the only vehicle
        // preceding host is the candidate joiner. For this code to work, it depends on the candidate to publish
        // mobility operation STATUS messages so that host can include it in the pm_ platoon membership.
        double currentGap = pm_.getDistanceToPredVehicle(); //returns 0 if we haven't received op STATUS from joiner yet
        ROS_DEBUG_STREAM("Based on desired join time gap, the desired join distance gap is " << desiredJoinGap2 << " m");
        ROS_DEBUG_STREAM("Since we have max allowed gap as " << config_.desiredJoinGap << " m then max join gap became " << maxJoinGap << " m");
        ROS_DEBUG_STREAM("The current gap to joiner is " << currentGap << " m");

        // NOTE: The front join depends upon the joiner to publish op STATUS messages with this platoon's ID, then host receives at least one
        // and thereby adds the joiner to the platoon record. This process requires host's mob_req_cb_leader() to ACK the join request, then 
        // the remote vehicle to handle that ACK and broadcast its first op STATUS, then host to receive and process it. All that has to happen
        // before the below code block runs, even though this method starts to spin immediately after we send out the mentioned ACK. To avoid
        // a race condition, we must wait a few cycles to ensure the 2-way messaging has completed. The race is that above calculation for
        // current gap will be returned as 0 until we have received said STATUS message from the joiner, which would prematurely trigger the
        // process to move forward.

        // Check if gap is big enough and if there is no currently active plan and this method has been called several times
        // Add a condition to prevent sending repeated requests (Note: This is a same-lane maneuver, so no need to consider lower bound of joining gap.)
        ++numLeaderAbortingCalls_;
        ROS_DEBUG_STREAM("numLeaderAbortingCalls = " << numLeaderAbortingCalls_ << ", max = " << config_.maxLeaderAbortingCalls);
        if (currentGap <= maxJoinGap  &&  !pm_.current_plan.valid  &&  numLeaderAbortingCalls_ > config_.maxLeaderAbortingCalls) 
        {
            // compose frontal joining plan, senderID is the old leader 
            cav_msgs::MobilityRequest request;
            std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
            long currentTime = ros::Time::now().toNSec() / 1000000;
            request.m_header.plan_id = planId;
            request.m_header.recipient_id = pm_.platoonLeaderID; //the new joiner
            request.m_header.sender_id = config_.vehicleID;
            request.m_header.timestamp = currentTime;

            int platoon_size = pm_.getHostPlatoonSize(); //depends on joiner to send op STATUS messages while joining
            
            boost::format fmter(JOIN_PARAMS); // Note: Front and rear join uses same params, hence merge to one param for both condition.
            int dummy_join_index = -2; //leader aborting doesn't need join_index so use default value
            fmter %platoon_size;                //  index = 0
            fmter %current_speed_;              //  index = 1, in m/s
            fmter %pose_ecef_point_.ecef_x;     //  index = 2
            fmter %pose_ecef_point_.ecef_y;     //  index = 3
            fmter %pose_ecef_point_.ecef_z;     //  index = 4   
            fmter %dummy_join_index;            //  index = 5
            request.strategy_params = fmter.str();

            // assign a new plan type 
            request.plan_type.type = cav_msgs::PlanType::PLATOON_FRONT_JOIN;
            request.strategy = PLATOONING_STRATEGY;
            request.urgency = 50;
            request.location = pose_to_ecef(pose_msg_);
            mobility_request_publisher_(request);
            ROS_WARN("Published Mobility Candidate-Join request to the new leader");

            // Create a new join action plan
            pm_.current_plan = ActionPlan(true, currentTime, planId, pm_.platoonLeaderID);
        }

        //Task 4: publish platoon status message
        cav_msgs::MobilityOperation status;
        status = composeMobilityOperationLeaderAborting();
        mobility_operation_publisher_(status);
        ROS_DEBUG_STREAM("Published platoon STATUS operation message");

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
        if (tsStart - candidatestateStartTime > waitingStateTimeout * 1000)
        {
            //TODO if the current state timeouts, we need to have a kind of ABORT message to inform the applicant
            ROS_DEBUG_STREAM("CandidateLeader state is timeout, changing back to PlatoonLeaderState.");
            pm_.current_platoon_state = PlatoonState::LEADER;
            pm_.clearActionPlan();
            pm_.resetHostPlatoon();
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
        bool isTimeForHeartBeat = tsStart - prevHeartBeatTime_ >= infoMessageInterval_;
        ROS_DEBUG_STREAM("time since last heart beat: " << tsStart - prevHeartBeatTime_);
        if (isTimeForHeartBeat) 
        {
            cav_msgs::MobilityOperation infoOperation;
            infoOperation = composeMobilityOperationLeadWithOperation(OPERATION_INFO_TYPE);
            mobility_operation_publisher_(infoOperation);
            prevHeartBeatTime_ = ros::Time::now().toNSec() / 1000000;
        }

        // // Task 3: plan time out
        // if (pm_.current_plan.valid)
        // {
        //     bool isCurrentPlanTimeout = ((ros::Time::now().toNSec() / 1000000 - pm_.current_plan.planStartTime) > NEGOTIATION_TIMEOUT);
        //     if (isCurrentPlanTimeout)
        //     {
        //         ROS_DEBUG_STREAM("Give up waiting on plan with planId: " << pm_.current_plan.planId << "; stay in LEADWITHOPERATION");
        //         pm_.current_plan.valid = false;
        //     }
        // }

        // Task 4: STATUS msgs
        bool hasFollower = pm_.getHostPlatoonSize() > 1  ||  config_.test_cutin_join;
        // if has follower, publish platoon message as STATUS mob_op
        if (hasFollower) 
        {
            cav_msgs::MobilityOperation statusOperation;
            statusOperation = composeMobilityOperationLeadWithOperation(OPERATION_STATUS_TYPE);
            mobility_operation_publisher_(statusOperation);
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
        *   Note: 1. safeToLaneChange_ monitors the gap condition.
        *         2. Once it is safe to lane change, the updated plan will send-out in "plan_maneuver_cb"
        */

        // Task 2.1: state timeout
        long tsStart = ros::Time::now().toNSec() / 1000000;
        bool isCurrentStateTimeout = (tsStart - candidatestateStartTime) > waitingStateTimeout * 1000;
        ROS_DEBUG_STREAM("timeout1: " << tsStart - candidatestateStartTime);
        ROS_DEBUG_STREAM("waitingStateTimeout: " << waitingStateTimeout * 1000);
        if (isCurrentStateTimeout) 
        {
            ROS_DEBUG_STREAM("The current prepare to join state is timeout. Change back to leader state and abort lane change.");
            pm_.current_platoon_state = PlatoonState::LEADER;
            safeToLaneChange_ = false;
            pm_.clearActionPlan();
            pm_.resetHostPlatoon();
            // Leave neighbor platoon info in place, as we may retry the join later
        }

        // TODO: Plan timeout is not needed for this state

        // If we aren't already waiting on a response to one of these plans, create one once neighbor info is available
        ROS_DEBUG_STREAM("current_plan.valid = " << pm_.current_plan.valid << ", is_neighbor_record_complete = " << pm_.is_neighbor_record_complete_);
        
        if (!pm_.current_plan.valid  &&  pm_.is_neighbor_record_complete_)
        {
            // Task 1: compose mobility operation (status)
            cav_msgs::MobilityOperation status;
            status = composeMobilityOperationPrepareToJoin(); //TODO: I bet we could consolidate a lot of these compose methods
            mobility_operation_publisher_(status);
            ROS_DEBUG_STREAM("Published platoon STATUS operation message");

            // Task 3: Calculate proper cut_in index 
            // Note: The cut-in index is zero-based and points to the gap-leading vehicle's index. For cut-in from front, the join index = -1.
            double joinerDtD = current_downtrack_;
            target_join_index_ = pm_.getClosestIndex(joinerDtD);
            ROS_DEBUG_STREAM("calculated join index: " << target_join_index_);

            // Task 4: Send out request to leader about cut-in position
            cav_msgs::MobilityRequest request;
            std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
            long currentTime = ros::Time::now().toNSec() / 1000000;
            request.m_header.plan_id = planId;
            request.m_header.recipient_id = pm_.neighbor_platoon_leader_id_;
            request.m_header.sender_id = config_.vehicleID;
            request.m_header.timestamp = currentTime;
            request.plan_type.type = cav_msgs::PlanType::PLATOON_CUT_IN_JOIN;
            request.strategy = PLATOONING_STRATEGY;
            request.urgency = 50;
            request.location = pose_to_ecef(pose_msg_);
            double platoon_size = pm_.getHostPlatoonSize(); 

            boost::format fmter(JOIN_PARAMS);
            fmter %platoon_size;                //  index = 0
            fmter %current_speed_;              //  index = 1, in m/s
            fmter %pose_ecef_point_.ecef_x;     //  index = 2
            fmter %pose_ecef_point_.ecef_y;     //  index = 3
            fmter %pose_ecef_point_.ecef_z;     //  index = 4
            fmter %target_join_index_;          //  index = 5
            request.strategy_params = fmter.str();
            mobility_request_publisher_(request); 
            ROS_DEBUG_STREAM("Published Mobility cut-in join request to leader " << request.m_header.recipient_id << " with planId = " << planId);

            // Create a new join action plan
            pm_.current_plan = ActionPlan(true, currentTime, planId, pm_.neighbor_platoon_leader_id_);
        }
    }

    // UCLA: run prepare to depart state for depart from platoon
    // TODO: Place holder for departure


    //------------------------------------------- main functions for platoon plugin --------------------------------------------//
    
    // Platoon on spin
    bool PlatoonStrategicIHPPlugin::onSpin() 
    {
        plugin_discovery_publisher_(plugin_discovery_msg_);
        
        // Update the platoon manager for host's current location & speeds
        pm_.updateHostPose(current_downtrack_, current_crosstrack_);
        pm_.updateHostSpeeds(current_speed_, cmd_speed_);

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
        else if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            ROS_DEBUG_STREAM("standby state, nothing to do");
        }
        // coding oversight
        else
        {
            ROS_ERROR_STREAM("///// unhandled state " << pm_.current_platoon_state);
        }
        // TODO: Place holder for departure

        cav_msgs::PlatooningInfo platoon_status = composePlatoonInfoMsg();
        platooning_info_publisher_(platoon_status);

        return true;
    }

    // ------- Generate maneuver plan (Service Callback) ------- //
    
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
        maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration(config_.time_step);
        maneuver_msg.lane_following_maneuver.lane_ids = { std::to_string(lane_id) };

        ROS_DEBUG_STREAM("in compose maneuver lane id:"<< lane_id);

        lanelet::ConstLanelet current_lanelet = wm_->getMap()->laneletLayer.get(lane_id);
        if(!wm_->getMapRoutingGraph()->following(current_lanelet, false).empty())
        {

            auto next_lanelet_id = wm_->getMapRoutingGraph()->following(current_lanelet, false).front().id();
            ROS_DEBUG_STREAM("next_lanelet_id:"<< next_lanelet_id);
            maneuver_msg.lane_following_maneuver.lane_ids.push_back(std::to_string(next_lanelet_id));
        }
        else
        {
            ROS_DEBUG_STREAM("No following lanelets");
        }

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
        maneuver_msg.lane_change_maneuver.parameters.planning_tactical_plugin = "CooperativeLaneChangePlugin";
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
            // maneuver_msg.lane_change_maneuver.end_time = current_time + ros::Duration((end_dist - current_dist) / (0.5 * cur_plus_target));
            maneuver_msg.lane_change_maneuver.end_time = current_time + ros::Duration(20.0);

        }

        // UCLA: need both start laneID and end laneID  for lane change
        maneuver_msg.lane_change_maneuver.starting_lane_id = { std::to_string(starting_lane_id) };
        maneuver_msg.lane_change_maneuver.ending_lane_id = { std::to_string(ending_lane_id) };

        current_time = maneuver_msg.lane_change_maneuver.end_time;
        ROS_DEBUG_STREAM("Creating lane change start dist:"<<current_dist<<" end dist:"<<end_dist);
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

    // maneuver plan callback (provide cav_srvs for arbitrator) 
    bool PlatoonStrategicIHPPlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp)
    {
        // use current position to find lanelet ID
        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);

        // *** get the actually closest lanelets that relate to current location (n=10) ***//
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 10);  

        // raise warn if no path was found
        if(current_lanelets.size() == 0)
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return true;
        }

        // locate lanelet on shortest path
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


        // read status data
        double current_progress = wm_->routeTrackPos(current_loc).downtrack;
        double speed_progress = current_speed_;
        ros::Time time_progress = ros::Time::now();

        // ---------------- use IHP platoon trajectory regulation here --------------------
        // Note: The desired gap will be adjusted and send to control plugin (via platoon_info_msg) where gap creation will be handled.
        double target_speed;   
        
        // Note: gap regulation has moved to control plug-in, no need to adjust speed here.
        target_speed = findSpeedLimit(current_lanelet);   //get Speed Limit
        double total_maneuver_length = current_progress + config_.time_step * target_speed;
        // ----------------------------------------------------------------

        // pick smaller length, accomendate when host is close to the route end
        double route_length =  wm_->getRouteEndTrackPos().downtrack; 
        total_maneuver_length = std::min(total_maneuver_length, route_length);

        // Update current status based on prior plan
        if(req.prior_plan.maneuvers.size()!= 0)
        {
            ROS_DEBUG_STREAM("Provided with initial plan...");
            time_progress = req.prior_plan.planning_completion_time;
            int end_lanelet = 0;
            updateCurrentStatus(req.prior_plan.maneuvers.back(), speed_progress, current_progress, end_lanelet);
            last_lanelet_index = findLaneletIndexFromPath(end_lanelet, shortest_path);
        }
        
        ROS_DEBUG_STREAM("Starting Loop");
        ROS_DEBUG_STREAM("total_maneuver_length: " << total_maneuver_length << " route_length: " << route_length);
        

        ROS_DEBUG_STREAM("in mvr  callback safeToLaneChange: " << safeToLaneChange_);

        // Note: Use current_lanlet list (which was determined based on vehicle pose) to find current lanelet ID. 
        long current_lanelet_id = current_lanelets[0].second.id();
        ROS_DEBUG_STREAM("current_lanelet_id: " << current_lanelet_id);
        // lane change maneuver 
        if (safeToLaneChange_)
        {   
            // for testing purpose only, check lane change status
            double target_crosstrack = wm_->routeTrackPos(target_cutin_pose_).crosstrack;
            ROS_DEBUG_STREAM("target_crosstrack: " << target_crosstrack);
            double crosstrackDiff = current_crosstrack_ - target_crosstrack; 
            bool isLaneChangeFinished = abs(crosstrackDiff) <= config_.maxCrosstrackError; 
            ROS_DEBUG_STREAM("crosstrackDiff: " << crosstrackDiff);
            ROS_DEBUG_STREAM("isLaneChangeFinished: " << isLaneChangeFinished);

            // lane change not finished, use lane change plan
            if(!isLaneChangeFinished)  
            {
                // send out lane change plan
                while (current_progress < total_maneuver_length)
                {   
                    ROS_DEBUG_STREAM("Lane Change Maneuver for Cut-in join ! ");
                    ROS_DEBUG_STREAM("current_progress: "<< current_progress);
                    ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
                    ROS_DEBUG_STREAM("target_speed: " << target_speed);
                    ROS_DEBUG_STREAM("time_progress: " << time_progress.toSec());

                    // set to next lane destination, consider sending ecef instead of dtd 
                    double end_dist = total_maneuver_length;
                    ROS_DEBUG_STREAM("end_dist: " << end_dist);
                    // consider calculate dtd_diff and ctd_diff
                    double dist_diff = end_dist - current_progress;
                    ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
    
                    
                    //TODO: target_cutin_pose_ represents the platoon leader. It seems this may be the wrong answer for mid- or rear-cutins?
                    //SAINA: currently, the functions do not provide the correct point of rear vehicle of the platoon
                    double lc_end_dist = wm_->routeTrackPos(target_cutin_pose_).downtrack;
                    ROS_DEBUG_STREAM("lc_end_dist before buffer: " << lc_end_dist);
                    lc_end_dist = std::max(lc_end_dist, current_progress + config_.maxCutinGap);
                    ROS_DEBUG_STREAM("lc_end_dist after buffer: " << lc_end_dist);
                    
                    //TODO: target_cutin_pose_ represents the platoon leader. Is this the best pose to use here?
                    // get the actually closest lanelets, 
                    auto target_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, target_cutin_pose_, 1); 
                    if (target_lanelets.empty())
                    {
                        ROS_DEBUG_STREAM("The target cutin pose is not on a valid lanelet. So no lane change!");
                        break;
                    } 
                    int target_lanelet_id = target_lanelets[0].second.id();
                    ROS_DEBUG_STREAM("target_lanelet_id: " << target_lanelet_id);

                    lanelet::ConstLanelet starting_lanelet = wm_->getMap()->laneletLayer.get(current_lanelet_id);
                    lanelet::ConstLanelet ending_lanelet = wm_->getMap()->laneletLayer.get(target_lanelet_id);

                    auto relation = wm_->getMapRoutingGraph()->routingRelation(starting_lanelet, ending_lanelet);
                    bool lanechangePossible = false;
                    // TODO: Assuming a lane change is only needed from an adjacent left/right lanelet. Only valid for IHP platooning. 
                    // Need to generalize in future. Refer to issue #1864
                    if (relation == lanelet::routing::RelationType::Left || relation == lanelet::routing::RelationType::Right)
                    {
                        lanechangePossible = true;
                    }
                    else
                    {
                        lanechangePossible = false;
                    }

                    if (lanechangePossible)
                    {
                        ROS_DEBUG_STREAM("Lane change possible, planning it.. " );
                        resp.new_plan.maneuvers.push_back(composeLaneChangeManeuverMessage(current_downtrack_, lc_end_dist,  
                                            speed_progress, target_speed, current_lanelet_id, target_lanelet_id , time_progress));
                        
                    }
                    else
                    {
                        ROS_DEBUG_STREAM("Lane change impossible, planning lanefollow instead ... " );
                        resp.new_plan.maneuvers.push_back(composeManeuverMessage(current_downtrack_, end_dist,  
                                            speed_progress, target_speed, current_lanelet_id, time_progress));
                    }

                    

                    current_progress += dist_diff;
                    // read lane change maneuver end time as time progress
                    time_progress = resp.new_plan.maneuvers.back().lane_change_maneuver.end_time;
                    speed_progress = target_speed;
                    if(current_progress >= total_maneuver_length || last_lanelet_index == static_cast<int>(shortest_path.size()) - 1)
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
                while (current_progress < total_maneuver_length)
                {   
                    ROS_DEBUG_STREAM("Same Lane Maneuver for platoon join ! ");
                    ROS_DEBUG_STREAM("current_progress: "<< current_progress);
                    ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
                    ROS_DEBUG_STREAM("target_speed: " << target_speed);
                    ROS_DEBUG_STREAM("time_progress: " << time_progress.toSec());
                    double end_dist = total_maneuver_length;
                    ROS_DEBUG_STREAM("end_dist: " << end_dist);
                    double dist_diff = end_dist - current_progress;
                    ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
                    if(end_dist < current_progress)
                    {
                        break;
                    }
                    // Note: The previous plan was generated at the beginning of the trip. It is necessary to update 
                    //       it as the lane ID and lanelet Index are different.
                    resp.new_plan.maneuvers.push_back(composeManeuverMessage(current_downtrack_, end_dist,  
                                            speed_progress, target_speed, current_lanelet_id, time_progress));
                    
                    current_progress += dist_diff;
                    time_progress = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
                    speed_progress = target_speed;
                    if(current_progress >= total_maneuver_length || last_lanelet_index == static_cast<int>(shortest_path.size()) - 1)
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
            ROS_DEBUG_STREAM("Planning Same Lane Maneuver! ");
            while (current_progress < total_maneuver_length)
            {   
                ROS_DEBUG_STREAM("Same Lane Maneuver for platoon join ! ");
                ROS_DEBUG_STREAM("current_progress: "<< current_progress);
                ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
                ROS_DEBUG_STREAM("target_speed: " << target_speed);
                ROS_DEBUG_STREAM("time_progress: " << time_progress.toSec());
                double end_dist = total_maneuver_length;
                ROS_DEBUG_STREAM("end_dist: " << end_dist);
                double dist_diff = end_dist - current_progress;
                ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
                if (end_dist < current_progress)
                {
                    break;
                }

                resp.new_plan.maneuvers.push_back(composeManeuverMessage(current_progress, end_dist,  
                                        speed_progress, target_speed,current_lanelet_id, time_progress));
                                    

                current_progress += dist_diff;
                time_progress = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
                speed_progress = target_speed;
                if(current_progress >= total_maneuver_length || last_lanelet_index == static_cast<int>(shortest_path.size()) - 1)
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


        if (pm_.getHostPlatoonSize() < 2 && !safeToLaneChange_)
        {
            resp.new_plan.maneuvers = {};
            ROS_WARN_STREAM("Platoon size 1 so Empty maneuver sent");
        }
        else
        {
            ROS_DEBUG_STREAM("Planning maneuvers: ");
            ROS_DEBUG_STREAM("safeToLaneChange_: " << safeToLaneChange_);
            ROS_DEBUG_STREAM("pm_.getHostPlatoonSize(): " << pm_.getHostPlatoonSize());
        }

        if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            pm_.current_platoon_state = PlatoonState::LEADER;
            ROS_DEBUG_STREAM("change the state from standby to leader at start-up");
        }

        ROS_DEBUG_STREAM("current_downtrack: " << current_downtrack_);
        
        return true;
    }
    //--------------------------------------------------------------------------//
}
