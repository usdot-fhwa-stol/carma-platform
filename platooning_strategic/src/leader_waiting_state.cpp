#include "leader_waiting_state.hpp"

namespace platoon_strategic
{
    

    MobilityRequestResponse LeaderWaitingState::onMobilityRequestMessage(cav_msgs::MobilityRequest &msg)
    {
        bool isTargetVehicle = (msg.header.sender_id == applicantID);
        bool isCandidateJoin = (msg.plan_type.type == cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN);
        if(isTargetVehicle && isCandidateJoin) {
            ROS_DEBUG("Target vehicle " , applicantID , " is actually joining.");
            ROS_DEBUG("Changing to PlatoonLeaderState and send ACK to target vehicle");
            current_platoon_state = PlatoonState::LEADER;
            // plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
            return MobilityRequestResponse::ACK;
        }
        else{
            ROS_DEBUG("Received platoon request with vehicle id = " , msg.header.sender_id);
            ROS_DEBUG("The request type is " , msg.plan_type.type,  " and we choose to ignore");
            return MobilityRequestResponse::NO_RESPONSE;
        }


    }
    void LeaderWaitingState::onMobilityOperationMessage(cav_msgs::MobilityOperation &msg)
    {
        // We still need to handle STATUS operation message from our platoon
        std::string strategyParams = msg.strategy_params;
        std::string OPERATION_STATUS_TYPE;
        bool isPlatoonStatusMsg = (strategyParams.find(OPERATION_STATUS_TYPE) == 0); //= strategyParams.startsWith(PlatooningPlugin.OPERATION_STATUS_TYPE);
        if (isPlatoonStatusMsg){
            std::string vehicleID = msg.header.sender_id;
            std::string platoonID = msg.header.plan_id;
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size()+1); //statusParams = strategyParams.substring(PlatooningPlugin.OPERATION_STATUS_TYPE.length() + 1);
        //     plugin.platoonManager.memberUpdates(vehicleID, platoonId, msg.getHeader().getSenderBsmId(), statusParams);
            ROS_DEBUG("Received platoon status message from " , msg.header.sender_id);
        }
        else
        {
            ROS_DEBUG("Received a mobility operation message with params " , msg.strategy_params + " but ignored.");
        }
    }
    void LeaderWaitingState::onMobilityResponseMessage(cav_msgs::MobilityResponse &msg)
    {
        
    }

    cav_msgs::Maneuver LeaderWaitingState::planManeuver(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time){
        cav_msgs::Maneuver maneuver_msg;
        return maneuver_msg;
    }

    void LeaderWaitingState::composeMobilityOperationStatus(cav_msgs::MobilityOperation &msg){
        
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

    void LeaderWaitingState::run(){
        // // The job for this loop is:
        //     // 1. Check the waiting time for LeaderWaiting state, if timeouts we transit to normal leader state
        //     // 2. Send out operation STATUS messages
        // while (!ros::isShuttingDown()){
        //     long tsStart = ros::Time::now().toSec()*1000;
        //     // Task 1
        //         if(tsStart - this.waitingStartTime > plugin.waitingStateTimeout * 1000) {
        //             //TODO if the current state timeouts, we need to have a kind of ABORT message to inform the applicant
        //             ROS_DEBUG("LeaderWaitingState is timeout, changing back to PlatoonLeaderState.");
        //             // plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
        //             current_platoon_state = PlatoonState::LEADER;
        //         }
        //         // Task 2
        //         cav_msgs::MobilityOperation status;
        //         composeMobilityOperationStatus(status);
        //         plugin.mobilityOperationPublisher.publish(status);
        //         long tsEnd = ros::Time::now().toSec()*1000;
        //         long sleepDuration = Math.max(plugin.statusMessageInterval - (tsEnd - tsStart), 0);
        //         // Thread.sleep(sleepDuration);
        //         ros::Duration(sleepDuration/1000).sleep();
        // }
    }

}