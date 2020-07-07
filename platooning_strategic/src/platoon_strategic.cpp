
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
#include <ros/ros.h>
#include <string>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "platoon_strategic.hpp"

namespace platoon_strategic
{
    PlatoonStrategicPlugin::PlatoonStrategicPlugin(){}
                                    

    void PlatoonStrategicPlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));

        
        
        trajectory_srv_ = nh_->advertiseService("plugins/PlatoonStrategicPlugin/plan_trajectory", &PlatoonStrategicPlugin::plan_trajectory_cb, this);

        mob_op_pub_ = nh_->advertise<cav_msgs::MobilityOperation>("mobility_operation_message", 5);
                
        platoon_strategic_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "PlatoonStrategicPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
        
        pose_sub_ = nh_->subscribe("current_pose", 1, &PlatoonStrategicPlugin::pose_cb, this);

        mob_req_sub_ = nh_->subscribe("incoming_mobility_request_message", 1, &PlatoonStrategicPlugin::mob_req_cb, this);
        mob_resp_sub_ = nh_->subscribe("incoming_mobility_response_message", 1, &PlatoonStrategicPlugin::mob_resp_cb, this);
        mob_op_sub_ = nh_->subscribe("incoming_mobility_operation_message", 1, &PlatoonStrategicPlugin::mob_op_cb, this);

        pnh_->param<double>("trajectory_time_length", trajectory_time_length_, 6.0);
        pnh_->param<std::string>("control_plugin_name", control_plugin_name_, "NULL");

        ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            platoon_strategic_plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });


    }

    void PlatoonStrategicPlugin::run_states(){
        switch (psm_->current_platoon_state)
        {
        case PlatoonState::STANDBY:
            run_standby();
            break;
        case PlatoonState::LEADERWAITING:
            waitingStartTime = ros::Time::now().toSec()*1000;
            run_leader_waiting();
            break;

        case PlatoonState::LEADER:
            run_leader();
            break;
        
        case PlatoonState::CANDIDATEFOLLOWER:
            candidatestateStartTime = ros::Time::now().toSec()*1000;
            run_candidate_follower();
            break;
        
        case PlatoonState::FOLLOWER:
            run_follower();
            break;
        
        default:
            throw std::invalid_argument("Invalid State");
            break;
        }


    }

    void PlatoonStrategicPlugin::run()
    {
    	initialize();
        run_states();
        ros::CARMANodeHandle::setSpinRate(10.0);
        ros::CARMANodeHandle::spin();

    }

    void PlatoonStrategicPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = msg;
    }

    bool PlatoonStrategicPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp){
        

        return true;
    }


    void PlatoonStrategicPlugin::run_standby(){

    }

    void PlatoonStrategicPlugin::run_leader_waiting(){
        long tsStart = ros::Time::now().toSec()*1000;
            // Task 1
                if(tsStart - waitingStartTime > waitingStateTimeout * 1000) {
                    //TODO if the current state timeouts, we need to have a kind of ABORT message to inform the applicant
                    ROS_DEBUG("LeaderWaitingState is timeout, changing back to PlatoonLeaderState.");
                    psm_->current_platoon_state = PlatoonState::LEADER;
                }
                // Task 2
                cav_msgs::MobilityOperation status;
                composeMobilityOperationLeaderWaiting(status);
                mob_op_pub_.publish(status);
                long tsEnd = ros::Time::now().toSec()*1000;
                int sleepDuration = std::max(int(statusMessageInterval - (tsEnd - tsStart)), 0);
                ros::Duration(sleepDuration/1000).sleep();
    }

    void PlatoonStrategicPlugin::run_leader(){

        long tsStart = ros::Time::now().toSec()*1000;
            // Task 1
            bool isTimeForHeartBeat = tsStart - lastHeartBeatTime >= infoMessageInterval;
            if(isTimeForHeartBeat) {
                    cav_msgs::MobilityOperation infoOperation;
                    composeMobilityOperationLeader(infoOperation, "INFO");
                    mob_op_pub_.publish(infoOperation);
                    lastHeartBeatTime = ros::Time::now().toSec()*1000.0;
                    ROS_DEBUG("Published heart beat platoon INFO mobility operatrion message");
                }
            // Task 2
            if (isTimeForHeartBeat) {
                    // updateLightBar();
            }
            // Task 3
            {
                std::lock_guard<std::mutex> lock(plan_mutex_);
                if(psm_->current_plan.valid) {
                    bool isCurrentPlanTimeout = ((ros::Time::now().toSec()*1000 - psm_->current_plan.planStartTime) > NEGOTIATION_TIMEOUT);
                    if(isCurrentPlanTimeout) {
                        ROS_DEBUG("Give up current on waiting plan with planId: " , psm_->current_plan.planId);
                        psm_->current_plan.valid = false;
                    }    
                }
            }

            // Task 4
            bool hasFollower = (pm_->getTotalPlatooningSize() > 1);// ????
            if(hasFollower) {
                cav_msgs::MobilityOperation statusOperation;
                composeMobilityOperationLeader(statusOperation, "STATUS");
                mob_op_pub_.publish(statusOperation);
                ROS_DEBUG("Published platoon STATUS operation message");
            }
            long tsEnd =  ros::Time::now().toSec()*1000;
            long sleepDuration = std::max(int(statusMessageInterval - (tsEnd - tsStart)), 0);
            ros::Duration(sleepDuration/1000).sleep();
        
    }

    void PlatoonStrategicPlugin::run_follower(){
        long tsStart = ros::Time::now().toSec()*1000;
            // Job 1
            cav_msgs::MobilityOperation status;
            composeMobilityOperationFollower(status);
            mob_op_pub_.publish(status);
            // Job 2
            // Get the number of vehicles in this platoon who is in front of us
            int vehicleInFront;// = plugin.platoonManager.getNumberOfVehicleInFront(); ????
                if(vehicleInFront == 0) {
                    noLeaderUpdatesCounter++;
                    if(noLeaderUpdatesCounter >= LEADER_TIMEOUT_COUNTER_LIMIT) {
                        ROS_DEBUG("noLeaderUpdatesCounter = " , noLeaderUpdatesCounter , " and change to leader state");
                        // plugin.platoonManager.changeFromFollowerToLeader();
                        psm_->current_platoon_state = PlatoonState::LEADER;
                        
                    }
                } else {
                    // reset counter to zero when we get updates again
                    noLeaderUpdatesCounter = 0;
                }
                long tsEnd = ros::Time::now().toSec()*1000.0;
                long sleepDuration = std::max(int(statusMessageInterval - (tsEnd - tsStart)), 0);
                ros::Duration(sleepDuration/1000).sleep();
        
    }

    void PlatoonStrategicPlugin::run_candidate_follower(){
        long tsStart = ros::Time::now().toSec()*1000.0; 
        // Task 1
        bool isCurrentStateTimeout = (tsStart - candidatestateStartTime) > waitingStateTimeout * 1000;
        if(isCurrentStateTimeout) {
            ROS_DEBUG("The current candidate follower state is timeout. Change back to leader state.");
            psm_->current_platoon_state = PlatoonState::LEADER;
        }
        // Task 2

        if(psm_->current_plan.valid) {
            {
                std::lock_guard<std::mutex> lock(plan_mutex_);
                if(psm_->current_plan.valid) {
                    bool isPlanTimeout = (tsStart - psm_->current_plan.planStartTime) > NEGOTIATION_TIMEOUT;
                    if(isPlanTimeout) {
                        psm_->current_plan.valid = false;
                        ROS_DEBUG("The current plan did not receive any response. Abort and change to leader state.");
                        psm_->current_platoon_state = PlatoonState::LEADER;
                    }    
                }
            }
        }

        // Task 3
                double desiredJoinGap2 = desiredJoinTimeGap;// * plugin.getManeuverInputs().getCurrentSpeed();?????
                double maxJoinGap = std::max(desiredJoinGap, desiredJoinGap2);
                double currentGap;// = plugin.getManeuverInputs().getDistanceToFrontVehicle();????????
                ROS_DEBUG("Based on desired join time gap, the desired join distance gap is " , desiredJoinGap2 , " ms");
                ROS_DEBUG("Since we have max allowed gap as " , desiredJoinGap , " m then max join gap became " , maxJoinGap , " m");
                ROS_DEBUG("The current gap from radar is " , currentGap , " m");
                if(currentGap <= maxJoinGap && psm_->current_plan.valid == false) {
                    cav_msgs::MobilityRequest request;
                    std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
                    long currentTime = ros::Time::now().toSec()*1000.0; 
                    request.header.plan_id = planId;
                    request.header.recipient_id = "";//????? targetLeaderId;
                    request.header.sender_bsm_id = "";//????????
                    request.header.sender_id = "";//?????????
                    request.header.timestamp = currentTime;
                    // request.getHeader().setRecipientId(targetLeaderId);
                    // request.getHeader().setSenderBsmId(pluginServiceLocator.getTrackingService().getCurrentBSMId());
                    // request.getHeader().setSenderId(pluginServiceLocator.getMobilityRouter().getHostMobilityId());
                    // request.getHeader().setTimestamp(currentTime);
                    cav_msgs::LocationECEF loc;
                    request.location = loc;
                    
                    // RoutePointStamped currentLocation = new RoutePointStamped(plugin.getManeuverInputs().getDistanceFromRouteStart(),
                    // plugin.getManeuverInputs().getCrosstrackDistance(), currentTime / 1000.0);
                    // cav_msgs.Trajectory currentLocationMsg = pluginServiceLocator.getTrajectoryConverter().pathToMessage(Arrays.asList(currentLocation));
                    // request.setLocation(currentLocationMsg.getLocation());
                    request.plan_type.type = cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN;
                    std::string MOBILITY_STRATEGY;
                    request.strategy = MOBILITY_STRATEGY;
                    request.strategy_params = "";
                    request.urgency = 50;
                    // plugin.mobilityRequestPublisher.publish(request);
                    // request.getPlanType().setType(PlanType.PLATOON_FOLLOWER_JOIN);
                    // request.setStrategy(PlatooningPlugin.MOBILITY_STRATEGY);
                    // request.setStrategyParams("");
                    // // TODO Maybe need to add some params (vehicle IDs) into strategy string
                    // // TODO Maybe need to populate the urgency later
                    // request.setUrgency((short) 50);
                    // plugin.mobilityRequestPublisher.publish(request);
                    // ROS_DEBUG("Published Mobility Candidate-Join request to the leader");
                    // // this.currentPlan = new PlatoonPlan(System.currentTimeMillis(), planId, targetLeaderId);????????
                }
        
         //Task 4
                if(pm_->getTotalPlatooningSize() > 1) {
                    cav_msgs::MobilityOperation status;
                    composeMobilityOperationCandidateFollower(status);
                    mob_op_pub_.publish(status);
                    ROS_DEBUG("Published platoon STATUS operation message");
                }
                long tsEnd =  ros::Time::now().toSec()*1000;
                long sleepDuration = std::max(int(statusMessageInterval - (tsEnd - tsStart)), 0);
                ros::Duration(sleepDuration/1000).sleep();
        
    }



    void PlatoonStrategicPlugin::mob_req_cb(const cav_msgs::MobilityRequest& msg)
    {
        mobility_req_msg_ = msg;
    }

    void PlatoonStrategicPlugin::mob_resp_cb(const cav_msgs::MobilityResponse& msg)
    {
        mobility_resp_msg_ = msg;
    }

    void PlatoonStrategicPlugin::mob_op_cb(const cav_msgs::MobilityOperation& msg)
    {
        mobility_op_msg_ = msg;
    }


    void PlatoonStrategicPlugin::composeMobilityOperationLeader(cav_msgs::MobilityOperation &msg, std::string type){
        msg.header.plan_id = "";//??????
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = "";//????
        std::string hostStaticId;// = ?????
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toSec()*1000.0;
        std::string MOBILITY_STRATEGY, OPERATION_INFO_TYPE, OPERATION_STATUS_TYPE;
        msg.strategy = MOBILITY_STRATEGY;
        if (type == OPERATION_INFO_TYPE){
            std::string infoParams;//????????????????????
            msg.strategy_params = infoParams;
        }
        else if (type == OPERATION_STATUS_TYPE){
            double cmdSpeed;
            std::string statusParams;
            msg.strategy_params = statusParams;
        } else{
            ROS_ERROR("UNKNOW strategy param string!!!");
            msg.strategy_params = "";
        }
        ROS_DEBUG("Composed a mobility operation message with params " , msg.strategy_params);
    
    }

    void PlatoonStrategicPlugin::composeMobilityOperationFollower(cav_msgs::MobilityOperation &msg){
        msg.header.plan_id = pm_->currentPlatoonID;
        // All platoon mobility operation message is just for broadcast
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = "";
        std::string hostStaticId; //= pluginServiceLocator.getMobilityRouter().getHostMobilityId();
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toSec()*1000.0;
        std::string MOBILITY_STRATEGY;
        msg.strategy = MOBILITY_STRATEGY;
        double cmdSpeed; // = plugin.getLastSpeedCmd();

        std::string OPERATION_STATUS_PARAMS = "STATUS|CMDSPEED:%1%,DTD:%2%,SPEED:%3%";
        double current_speed, current_downtrack;

        boost::format fmter(OPERATION_STATUS_PARAMS);
        fmter %cmdSpeed;
        fmter %current_downtrack;
        fmter %current_speed;
                    
        std::string statusParams = fmter.str();// = String.format(PlatooningPlugin.OPERATION_STATUS_PARAMS,
                                            // cmdSpeed, pluginServiceLocator.getRouteService().getCurrentDowntrackDistance(),
                                            // pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed());
        msg.strategy_params = statusParams;
        ROS_DEBUG("Composed a mobility operation message with params " , msg.strategy_params);
    }


    void PlatoonStrategicPlugin::composeMobilityOperationLeaderWaiting(cav_msgs::MobilityOperation &msg)
    {
        msg.header.plan_id = "";//????????
        // This message is for broadcast
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = "";//???????
        std::string hostStaticId; // = ???????????????
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toSec()*1000;
        std::string MOBILITY_STRATEGY;
        msg.strategy = MOBILITY_STRATEGY;
        // For STATUS params, the string format is "STATUS|CMDSPEED:5.0,DOWNTRACK:100.0,SPEED:5.0"
        double cmdSpeed;// = ???????
        double downtrackDistance; //=??????? 
        double currentSpeed; // = ????????
        std::string params; //= ????????????
        msg.strategy_params = params;
    }


    void PlatoonStrategicPlugin::composeMobilityOperationCandidateFollower(cav_msgs::MobilityOperation &msg)
    {
        msg.header.plan_id = "";//?????????
        // All platoon mobility operation message is just for broadcast
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = "";//????????
        std::string hostStaticId; // = ????????
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toSec()*1000.0; 
        std::string MOBILITY_STRATEGY;
        msg.strategy = MOBILITY_STRATEGY;
        double cmdSpeed;// = plugin.getLastSpeedCmd();????????
        // // For STATUS params, the string format is "STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx"
        std::string statusParams;//????????????????
        msg.strategy_params = statusParams;
        ROS_DEBUG("Composed a mobility operation message with params " , msg.strategy_params);
    }


}
