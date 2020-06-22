#include "leader_state.hpp"
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>



namespace platoon_strategic
{
    

    MobilityRequestResponse LeaderState::onMobilityRequestMessage(cav_msgs::MobilityRequest &msg){
        // If we received a JOIN request, we decide whether we allow that CAV to join.
        // If we agree on its join, we need to change to LeaderWaiting state.
        if (msg.plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_AT_REAR){
            // We are currently checking two basic JOIN conditions:
            //     1. The size limitation on current platoon based on the plugin's parameters.
            //     2. Calculate how long that vehicle can be in a reasonable distance to actually join us.
            std::string params = msg.strategy_params;
            std::string applicantId = msg.header.sender_id;
            ROS_DEBUG("Receive mobility JOIN request from " , applicantId, " and PlanId = " , msg.header.plan_id);
            ROS_DEBUG("The strategy parameters are " , params);
            // For JOIN_PLATOON_AT_REAR message, the strategy params is defined as "SIZE:xx,SPEED:xx,DTD:xx"
            // TODO In future, we should remove down track distance from this string and use location field in request message
            
            std::vector<std::string> parsed_params;
            boost::algorithm::split(parsed_params, params, boost::is_any_of(","));

            std::vector<std::string> parsed_size;
            boost::algorithm::split(parsed_size, parsed_params[0], boost::is_any_of(":"));
            int applicantSize = std::stoi(parsed_size[1]);

            std::vector<std::string> parsed_speed;
            boost::algorithm::split(parsed_speed, parsed_params[1], boost::is_any_of(":"));
            double applicantCurrentSpeed = std::stod(parsed_size[1]);

            std::vector<std::string> parsed_dtd;
            boost::algorithm::split(parsed_dtd, parsed_params[1], boost::is_any_of(":"));
            double applicantCurrentDtd = std::stod(parsed_size[1]);

            // Check if we have enough room for that applicant
            int currentPlatoonSize;// = plugin.platoonManager.getTotalPlatooningSize();
            bool hasEnoughRoomInPlatoon = applicantSize + currentPlatoonSize <= maxPlatoonSize;
            if(hasEnoughRoomInPlatoon) {
                ROS_DEBUG("The current platoon has enough room for the applicant with size " , applicantSize);
                double currentRearDtd;// = plugin.platoonManager.getPlatoonRearDowntrackDistance();
                ROS_DEBUG("The current platoon rear dtd is " , currentRearDtd);
                double currentGap = currentRearDtd - applicantCurrentDtd - vehicleLength;
                double currentTimeGap = currentGap / applicantCurrentSpeed;
                ROS_DEBUG("The gap between current platoon rear and applicant is " , currentGap , "m or " , currentTimeGap , "s");
                if(currentGap < 0) {
                    ROS_WARN("We should not receive any request from the vehicle in front of us. NACK it.");
                    return MobilityRequestResponse::NACK;
                }
                // Check if the applicant can join based on max timeGap/gap
                bool isDistanceCloseEnough = (currentGap <= maxAllowedJoinGap) || (currentTimeGap <= maxAllowedJoinTimeGap);
                if(isDistanceCloseEnough) {
                    ROS_DEBUG("The applicant is close enough and we will allow it to try to join");
                    ROS_DEBUG("Change to LeaderWaitingState and waiting for " , msg.header.sender_id , " to join");
                    current_platoon_state = PlatoonState::LEADERWAITING;
                    // applicantID = applicantId;
                    // plugin.setState(new LeaderWaitingState(plugin, log, pluginServiceLocator, applicantId));
                    return MobilityRequestResponse::ACK;
                }
                else {
                    ROS_DEBUG("The applicant is too far away from us. NACK.");
                    return MobilityRequestResponse::NACK;
                }
            }
            else {
                ROS_DEBUG("The current platoon does not have enough room for applicant of size " , applicantSize , ". NACK");
                return MobilityRequestResponse::NACK;
            }
        }
        else {
            ROS_DEBUG("Received mobility request with type " , msg.plan_type.type , " and ignored." );
        }
        return MobilityRequestResponse::NO_RESPONSE;
    }
    void LeaderState::onMobilityOperationMessage(cav_msgs::MobilityOperation &msg)
    {
        std::string strategyParams = msg.strategy_params;
        std::string senderId = msg.header.sender_id;
        std::string platoonId = msg.header.plan_id;
        // In the current state, we care about the INFO heart-beat operation message if we are not currently in
        // a negotiation, and also we need to care about operation from members in our current platoon
        std::string OPERATION_INFO_TYPE;
        std::string OPERATION_STATUS_TYPE;
        bool isPlatoonInfoMsg = (strategyParams.find(OPERATION_INFO_TYPE) == 0);// = strategyParams.startsWith(PlatooningPlugin.OPERATION_INFO_TYPE);
        bool isPlatoonStatusMsg = (strategyParams.find(OPERATION_STATUS_TYPE) == 0);;// = strategyParams.startsWith(PlatooningPlugin.OPERATION_STATUS_TYPE);
        {
            std::lock_guard<std::mutex> lock(plan_mutex_);

            bool isNotInNegotiation = (!current_plan.valid);
            if(isPlatoonInfoMsg && isNotInNegotiation) {
                // For INFO params, the string format is INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d
                // TODO In future, we should remove downtrack distance from this string and send XYZ location in ECEF
                std::vector<std::string> parsed_strategy_params;
                boost::algorithm::split(parsed_strategy_params, strategyParams, boost::is_any_of(","));

                std::vector<std::string> parsed_BsmId;
                boost::algorithm::split(parsed_BsmId, parsed_strategy_params[0], boost::is_any_of(":"));
                std::string rearVehicleBsmId = parsed_BsmId[1];

                std::vector<std::string> parsed_dtd;
                boost::algorithm::split(parsed_dtd, parsed_strategy_params[4], boost::is_any_of(":"));
                double rearVehicleDtd = std::stod(parsed_dtd[1]);
                // We are trying to validate is the platoon rear is right in front of the host vehicle
                if(isVehicleRightInFront(rearVehicleBsmId, rearVehicleDtd)) {
                    ROS_DEBUG("Found a platoon with id = " , platoonId , " in front of us.");
                    cav_msgs::MobilityRequest request;// = plugin.mobilityRequestPublisher.newMessage();
                    std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
                    long currentTime = ros::Time::now().toSec();
                    request.header.plan_id = planId;
                    request.header.recipient_id = senderId;
                    cav_msgs::LocationECEF location;
                    // location.x; ??????????????
                    // location.y;
                    // location.z;
                    // location.timestamp;
                    request.location = location;
                    request.plan_type.type = cav_msgs::PlanType::JOIN_PLATOON_AT_REAR;
                    // request.strategy; = MOBILITY_STRATEGY?????????????
                    std::string strategyParamsString; // = String.format(PlatooningPlugin.JOIN_AT_REAR_PARAMS,
                    //                                       plugin.platoonManager.getTotalPlatooningSize(),
                    //                                       plugin.getManeuverInputs().getCurrentSpeed(), pluginServiceLocator.getRouteService().getCurrentDowntrackDistance());
                    request.strategy_params = strategyParamsString;
                    request.urgency = 50;
                    // this.currentPlan = new PlatoonPlan(System.currentTimeMillis(), request.getHeader().getPlanId(), senderId);
                    // plugin.mobilityRequestPublisher.publish(request);
                    ROS_DEBUG("Publishing request to leader " , senderId , " with params " , request.strategy_params , " and plan id = " , request.header.plan_id);
                    // this.potentialNewPlatoonId = platoonId;
                } else{
                        ROS_DEBUG("Ignore platoon with platoon id: " , platoonId , " because it is not right in front of us");
                }
            }
            else if(isPlatoonStatusMsg) {
                // If it is platoon status message, the params string is in format: STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx
                std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size()+1);// = strategyParams.substring(PlatooningPlugin.OPERATION_STATUS_TYPE.length() + 1);
                ROS_DEBUG("Receive operation status message from vehicle: " , senderId , " with params: " , statusParams);
                // plugin.platoonManager.memberUpdates(senderId, platoonId, msg.getHeader().getSenderBsmId(), statusParams);
            }
            else {
                ROS_DEBUG("Receive operation message but ignore it because isPlatoonInfoMsg = " , isPlatoonInfoMsg 
                        , ", isNotInNegotiation = " , isNotInNegotiation , " and isPlatoonStatusMsg = " , isPlatoonStatusMsg);
            }

        }

    }
    void LeaderState::onMobilityResponseMessage(cav_msgs::MobilityResponse &msg)
    {
        // Only care the response message for the plan for which we are waiting
        {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            if (current_plan.valid){
                if ((current_plan.planId == msg.header.plan_id) && (current_plan.peerId == msg.header.sender_id)){
                    if (msg.is_accepted){
                        ROS_DEBUG("Received positive response for plan id = " , current_plan.planId);
                        ROS_DEBUG("Change to CandidateFollower state and notify trajectory failure in order to replan");
        //                 // Change to candidate follower state and request a new plan to catch up with the front platoon
                        current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
        //                 plugin.setState(new CandidateFollowerState(plugin, log, pluginServiceLocator, currentPlan.peerId, potentialNewPlatoonId, this.trajectoryEndLocation));
        //                 pluginServiceLocator.getArbitratorService().requestNewPlan(this.trajectoryEndLocation);
                    } else{
                        ROS_DEBUG("Received negative response for plan id = " , current_plan.planId);
                        // Forget about the previous plan totally
        //                 this.currentPlan = null;
                            current_plan.valid = false;
                    }
                } else{
                    ROS_DEBUG("Ignore the response message because planID match: " , current_plan.planId == msg.header.plan_id);
                    ROS_DEBUG("My plan id = " , current_plan.planId , " and response plan Id = " , msg.header.plan_id);
                    ROS_DEBUG("And peer id match: " , current_plan.peerId == msg.header.sender_id);
                    ROS_DEBUG("Expected peer id = " , current_plan.peerId , " and response sender Id = " , msg.header.sender_id);
                }
            }
        }
    }

    void LeaderState::composeMobilityOperationStatus(cav_msgs::MobilityOperation &msg, std::string type){
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

    void LeaderState::run(){
        // // This is a loop which is safe to interrupt
        // // This loop does four tasks:
        // // 1. Send out heart beat mobility operation INFO message every ~3 seconds if the platoon is not full
        // // 2. Updates the light bar status every ~3 seconds 
        // // 3. Remove current plan if we wait for a long enough time
        // // 4. Publish operation status every 100 milliseconds if we have follower
        // while (!ros::isShuttingDown()){
        //     long tsStart = ros::Time::now().toSec()*1000;
        //     // Task 1
        //     bool isTimeForHeartBeat = tsStart - lastHeartBeatTime >= infoMessageInterval;
        //     if(isTimeForHeartBeat) {
        //             cav_msgs::MobilityOperation infoOperation;
        //             composeMobilityOperation(infoOperation, "INFO");
        //             // plugin.mobilityOperationPublisher.publish(infoOperation);
        //             lastHeartBeatTime = ros::Time::now().toSec()*1000.0;
        //             ROS_DEBUG("Published heart beat platoon INFO mobility operatrion message");
        //         }
        //     // Task 2
        //     if (isTimeForHeartBeat) {
        //             // updateLightBar();
        //     }
        //     // Task 3
        //     {
        //         std::lock_guard<std::mutex> lock(plan_mutex_);
        //         if(current_plan.valid) {
        //             boo; isCurrentPlanTimeout;// = ((System.currentTimeMillis() - this.currentPlan.planStartTime) > PlatooningPlugin.NEGOTIATION_TIMEOUT);
        //             if(isCurrentPlanTimeout) {
        //                 ROS_DEBUG("Give up current on waiting plan with planId: " + this.currentPlan.planId);
        //                 current_plan.valid = false;
        //             }    
        //         }
        //     }

        //     // Task 4
        //     boolean hasFollower;// = plugin.platoonManager.getTotalPlatooningSize() > 1; 
        //     if(hasFollower) {
        //         cav_msgs::MobilityOperation statusOperation;
        //         composeMobilityOperation(statusOperation, "STATUS");
        //         // plugin.mobilityOperationPublisher.publish(statusOperation);
        //         ROS_DEBUG("Published platoon STATUS operation message");
        //     }
        //     long tsEnd =  ros::Time::now().toSec()*1000;
        //     long sleepDuration = Math.max(plugin.statusMessageInterval - (tsEnd - tsStart), 0);
        //     // Thread.sleep(sleepDuration);
        //     ros::Duration(sleepDuration/1000).sleep();

        // }
    }


    bool LeaderState::isVehicleRightInFront(std::string rearVehicleBsmId, double downtrack) {
        double currentDtd;// = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
        if(downtrack > currentDtd) {
            std::clog << "Found a platoon in front. We are able to join" << std::endl;
            return true;
        } else {
            std::clog <<"Ignoring platoon from our back." << std::endl;
            std::clog << "The front platoon dtd is " << downtrack << " and we are current at " << currentDtd << std::endl;
            return false;
        }
    }

    cav_msgs::Maneuver LeaderState::planManeuver(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time){
        cav_msgs::Maneuver maneuver_msg;
        return maneuver_msg;
    }

}