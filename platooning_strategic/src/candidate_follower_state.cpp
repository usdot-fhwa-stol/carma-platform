#include <ros/ros.h>
#include "candidate_follower_state.hpp"


namespace platoon_strategic
{
    

    MobilityRequestResponse CandidateFollowerState::onMobilityRequestMessage(cav_msgs::MobilityRequest &msg)
    {
        // No need to response as a follower
        return MobilityRequestResponse::NO_RESPONSE;
    }
    void CandidateFollowerState::onMobilityOperationMessage(cav_msgs::MobilityOperation &msg)
    {
        // We still need to handle STATUS operAtion message from our platoon
        std::string strategyParams = msg.strategy_params;
        std::string OPERATION_STATUS_TYPE;
        bool isPlatoonStatusMsg = (strategyParams.find(OPERATION_STATUS_TYPE) == 0);// = strategyParams.startsWith(PlatooningPlugin.OPERATION_STATUS_TYPE);
        if (isPlatoonStatusMsg){
            std::string vehicleId = msg.header.sender_id;
            std::string platoonId = msg.header.plan_id;
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size()+1);// = strategyParams.substring(PlatooningPlugin.OPERATION_STATUS_TYPE.length() + 1);
            //     plugin.platoonManager.memberUpdates(vehicleID, platoonId, msg.getHeader().getSenderBsmId(), statusParams);
            ROS_DEBUG("Received platoon status message from " , vehicleId);
        }
        else {
            ROS_DEBUG("Received a mobility operation message with params " , msg.strategy_params , " but ignored.");
        }

    }
    void CandidateFollowerState::onMobilityResponseMessage(cav_msgs::MobilityResponse &msg)
    {
        // Waiting on response for the current Candidate-Join plan
        if (current_plan.valid){
            {
                std::lock_guard<std::mutex> lock(plan_mutex_);
                if (current_plan.valid){
                    bool isForCurrentPlan = (msg.header.plan_id == current_plan.planId);
                    bool isFromTargetVehicle = (msg.header.sender_id == targetLeaderId);
                    if(isForCurrentPlan && isFromTargetVehicle) {
                        if(msg.is_accepted) {
                        // We change to follower state and start to actually follow that leader
                        // The platoon manager also need to change the platoon Id to the one that the target leader is using 
                        ROS_DEBUG("The leader " , msg.header.sender_id , " agreed on our join. Change to follower state.");
                        // plugin.platoonManager.changeFromLeaderToFollower(targetPlatoonId);
                        current_platoon_state = PlatoonState::FOLLOWER;
                        // plugin.setState(new FollowerState(plugin, log, pluginServiceLocator));
                        // pluginServiceLocator.getArbitratorService().requestNewPlan(this.trajectoryEndLocation);
                        }
                        else{
                            // We change back to normal leader state and try to join other platoons
                            ROS_DEBUG("The leader " , msg.header.sender_id , " does not agree on our join. Change back to leader state.");
                            current_platoon_state = PlatoonState::LEADER;
                            // plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
                        }
                    }
                    else{
                        ROS_DEBUG("Ignore received response message because it is not for the current plan.");
                    }
                }
            }
        }
        else{
            ROS_DEBUG("Ignore received response message because we are not in any negotiation process.");
        }
    }

    cav_msgs::Maneuver CandidateFollowerState::planManeuver(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time){
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::GENERAL_NEGOTIATION;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "PlatooningTacticalPlugin";
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "PlatooningStrategicPlugin";
        maneuver_msg.lane_following_maneuver.start_dist = current_dist;
        maneuver_msg.lane_following_maneuver.start_speed = current_speed;
        maneuver_msg.lane_following_maneuver.start_time = current_time;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration((end_dist - current_dist) / (0.5 * (current_speed + target_speed)));
        maneuver_msg.lane_following_maneuver.lane_id = std::to_string(lane_id);
        return maneuver_msg;
    }


    void CandidateFollowerState::composeMobilityOperationStatus(cav_msgs::MobilityOperation &msg){
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

    void CandidateFollowerState::run(){
        // This is a interrupted-safe loop.
        // This loop has four tasks:
        // 1. Check the state start time, if it exceeds a limit it will give up current plan and change back to leader state
        // 2. Abort current request if we wait for long enough time for response from leader and change back to leader state
        // 3. Check the current distance with the target platoon rear and send out CANDIDATE-JOIN request when we get close
        // 4. Publish operation status every 100 milliseconds if we still have followers

        // while (!ros::isShuttingDown()){
        //     long tsStart = ros::Time::now().toSec()*1000.0; 
        //         // Task 1
        //         bool isCurrentStateTimeout;// = (tsStart - this.stateStartTime) > plugin.waitingStateTimeout * 1000;
        //         if(isCurrentStateTimeout) {
        //             ROS_DEBUG("The current candidate follower state is timeout. Change back to leader state.");
        //             current_platoon_state = PlatoonState::LEADER;
        //         }
        //         // Task 2

        //         if(current_plan.valid) {
        //             {
        //                 std::lock_guard<std::mutex> lock(plan_mutex_);
        //                 if(current_plan.valid) {
        //                     bool isPlanTimeout;// = (tsStart - this.currentPlan.planStartTime) > PlatooningPlugin.NEGOTIATION_TIMEOUT;
        //                     if(isPlanTimeout) {
        //                         current_plan.valid = false;
        //                         ROS_DEBUG("The current plan did not receive any response. Abort and change to leader state.");
        //                         current_platoon_state = PlatoonState::LEADER;
        //                     }    
        //                 }
        //             }
        //         }

        //         // Task 3
        //         double desiredJoinGap;// = plugin.desiredJoinTimeGap * plugin.getManeuverInputs().getCurrentSpeed();
        //         double maxJoinGap;// = Math.max(plugin.desiredJoinGap, desiredJoinGap);
        //         double currentGap;// = plugin.getManeuverInputs().getDistanceToFrontVehicle();
        //         ROS_DEBUG("Based on desired join time gap, the desired join distance gap is " , desiredJoinGap , " ms");
        //         ROS_DEBUG("Since we have max allowed gap as " , plugin.desiredJoinGap , " m then max join gap became " , maxJoinGap , " m");
        //         ROS_DEBUG("The current gap from radar is " , currentGap , " m");
        //         if(currentGap <= maxJoinGap && current_plan.valid == null) {
        //             cav_msgs::MobilityRequest request;
        //             std::string planId = std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
        //             long currentTime = ros::Time::now().toSec()*1000.0; 
        //             request.header.plan_id = planId;
        //             request.header.recipient_id = targetLeaderId;
        //             request.header.sender_bsm_id = "";
        //             request.header.sender_id = "";
        //             request.header.timestamp = currentTime;
        //             // request.getHeader().setRecipientId(targetLeaderId);
        //             // request.getHeader().setSenderBsmId(pluginServiceLocator.getTrackingService().getCurrentBSMId());
        //             // request.getHeader().setSenderId(pluginServiceLocator.getMobilityRouter().getHostMobilityId());
        //             // request.getHeader().setTimestamp(currentTime);
        //             cav_msgs::LocationECEF loc;
        //             request.location = loc;
                    
        //             // RoutePointStamped currentLocation = new RoutePointStamped(plugin.getManeuverInputs().getDistanceFromRouteStart(),
        //             // plugin.getManeuverInputs().getCrosstrackDistance(), currentTime / 1000.0);
        //             // cav_msgs.Trajectory currentLocationMsg = pluginServiceLocator.getTrajectoryConverter().pathToMessage(Arrays.asList(currentLocation));
        //             // request.setLocation(currentLocationMsg.getLocation());
        //             request.plan_type = cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN;
        //             std::string MOBILITY_STRATEGY;
        //             request.strategy = MOBILITY_STRATEGY;
        //             request.strategy_params = "";
        //             request.urgency = 50;
        //             plugin.mobilityRequestPublisher.publish(request);


        //             request.getPlanType().setType(PlanType.PLATOON_FOLLOWER_JOIN);
        //             request.setStrategy(PlatooningPlugin.MOBILITY_STRATEGY);
        //             request.setStrategyParams("");
        //             // TODO Maybe need to add some params (vehicle IDs) into strategy string
        //             // TODO Maybe need to populate the urgency later
        //             request.setUrgency((short) 50);
        //             plugin.mobilityRequestPublisher.publish(request);
        //             ROS_DEBUG("Published Mobility Candidate-Join request to the leader");
        //             // this.currentPlan = new PlatoonPlan(System.currentTimeMillis(), planId, targetLeaderId);
        //         }

        //         // Task 4
        //         if(plugin.platoonManager.getTotalPlatooningSize() > 1) {
        //             MobilityOperation status = plugin.mobilityOperationPublisher.newMessage();
        //             composeMobilityOperationStatus(status);
        //             plugin.mobilityOperationPublisher.publish(status);
        //         }
        //         long tsEnd =  ros::Time::now().toSec()*1000;
        // //      long sleepDuration = Math.max(plugin.statusMessageInterval - (tsEnd - tsStart), 0);
        //         // Thread.sleep(sleepDuration);
        //         ros::Duration(sleepDuration/1000).sleep();
        // }
    }

}