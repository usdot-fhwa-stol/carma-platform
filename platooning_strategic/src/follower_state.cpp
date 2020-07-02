#include "follower_state.hpp"
#include <boost/format.hpp>
#include <ros/ros.h>

namespace platoon_strategic
{
    

    MobilityRequestResponse FollowerState::onMobilityRequestMessage(cav_msgs::MobilityRequest &msg)
    {
        ROS_DEBUG("Initialized all node handles");
        return MobilityRequestResponse::NO_RESPONSE;
    }
    
    void FollowerState::onMobilityResponseMessage(cav_msgs::MobilityResponse &msg)
    {

    }

    void FollowerState::onMobilityOperationMessage(cav_msgs::MobilityOperation &msg)
    {
        std::string strategyParams = msg.strategy_params;
        // In the current state, we care about the STATUS message
        std::string OPERATION_STATUS_TYPE, OPERATION_INFO_TYPE;
        bool isPlatoonStatusMsg = (strategyParams.find(OPERATION_STATUS_TYPE) == 0);// = strategyParams.startsWith(PlatooningPlugin.OPERATION_STATUS_TYPE);
        bool isPlatoonInfoMsg = (strategyParams.find(OPERATION_INFO_TYPE) == 0);// = strategyParams.startsWith(PlatooningPlugin.OPERATION_INFO_TYPE);
        // If it is platoon status message, the params string is in format:
        // STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx
        if(isPlatoonStatusMsg) {
            std::string vehicleID = msg.header.sender_id;
            std::string platoonID = msg.header.plan_id;
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size()+1);// = strategyParams.substring(PlatooningPlugin.OPERATION_STATUS_TYPE.length() + 1);
            ROS_DEBUG("Receive operation message from vehicle: " , vehicleID);
        //     plugin.platoonManager.memberUpdates(vehicleID, platoonID, msg.getHeader().getSenderBsmId(), statusParams);
        } else if(isPlatoonInfoMsg) {
        //         if(msg.getHeader().getSenderId().equals(plugin.platoonManager.leaderID)) {
        //         String infoParams = strategyParams.substring(PlatooningPlugin.OPERATION_INFO_TYPE.length() + 1);
        //         plugin.platoonManager.platoonSize = Integer.parseInt(infoParams.split(",")[3].split(":")[1]);
        //         ROS_DEBUG("Update from the lead: the current platoon size is " , plugin.platoonManager.getTotalPlatooningSize());
        }
        else{
            ROS_DEBUG("Ignore other operation messages with params: " , strategyParams);

        }


    }


    cav_msgs::Maneuver FollowerState::planManeuver(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time){
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::PLATOONING;
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

    void FollowerState::composeMobilityOperationStatus(cav_msgs::MobilityOperation &msg){

        msg.header.plan_id = ""; //plugin.platoonManager.currentPlatoonID
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

    void FollowerState::run(){
        // // This is an interrupted-safe loop 
        // // This loop has two tasks:
        // // 1. Publish operation status every 100 milliseconds
        // // 2. Change to leader state if there is no active leader

        // while (!ros::isShuttingDown()){
            // long tsStart = ros::Time::now().toSec()*1000;
        //     // Job 1
        //     cav_msgs::MobilityOperation status;
        //     composeMobilityOperationStatus(status);
        //     // plugin.mobilityOperationPublisher.publish(status);
        //     // Job 2
        //     // Get the number of vehicles in this platoon who is in front of us
        //     int vehicleInFront;// = plugin.platoonManager.getNumberOfVehicleInFront(); 
        //         if(vehicleInFront == 0) {
        //             noLeaderUpdatesCounter++;
        //             if(noLeaderUpdatesCounter >= LEADER_TIMEOUT_COUNTER_LIMIT) {
        //                 ROS_DEBUG("noLeaderUpdatesCounter = " , noLeaderUpdatesCounter , " and change to leader state");
        //                 plugin.platoonManager.changeFromFollowerToLeader();
        //                 plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
                        
        //             }
        //         } else {
        //             // reset counter to zero when we get updates again
        //             noLeaderUpdatesCounter = 0;
        //         }
        //         long tsEnd = ros::Time::now().toSec()*1000.0;
        //         long sleepDuration = Math.max(plugin.statusMessageInterval - (tsEnd - tsStart), 0);
        //         // Thread.sleep(sleepDuration);
        //         ros::Duration(sleepDuration/1000).sleep();
        // }
    }



}   