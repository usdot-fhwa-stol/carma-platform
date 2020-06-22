#include "platoon_manager.hpp"
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>


namespace platoon_strategic
{
    void PlatoonManager::memberUpdates(std::string senderId, std::string platoonId, std::string senderBsmId, std::string params){

        std::vector<std::string> inputsParams;
        boost::algorithm::split(inputsParams, params, boost::is_any_of(","));

        std::vector<std::string> cmd_parsed;
        boost::algorithm::split(cmd_parsed, inputsParams[0], boost::is_any_of(":"));
        double cmdSpeed = std::stod(cmd_parsed[1]);

        std::vector<std::string> dtd_parsed;
        boost::algorithm::split(dtd_parsed, inputsParams[1], boost::is_any_of(":"));
        double dtDistance = std::stod(dtd_parsed[1]);

        std::vector<std::string> cur_parsed;
        boost::algorithm::split(cur_parsed, inputsParams[2], boost::is_any_of(":"));
        double curSpeed = std::stod(cur_parsed[1]);

        // If we are currently in a follower state:
        // 1. We will update platoon ID based on leader's STATUS
        // 2. We will update platoon members info based on platoon ID if it is in front of us 
        if(isFollower) {
            bool isFromLeader = (leaderID == senderId);
            bool needPlatoonIdChange = isFromLeader && (currentPlatoonID == platoonId);
            bool isVehicleInFrontOf;// = dtDistance >= psl.getRouteService().getCurrentDowntrackDistance();????
            if(needPlatoonIdChange) {
                ROS_DEBUG("It seems that the current leader is joining another platoon.");
                ROS_DEBUG("So the platoon ID is changed from " , currentPlatoonID , " to " , platoonId);
                currentPlatoonID = platoonId;
                // updatesOrAddMemberInfo(senderId, senderBsmId, cmdSpeed, dtDistance, curSpeed);
            } else if((currentPlatoonID == platoonId) && isVehicleInFrontOf) {
                ROS_DEBUG("This STATUS messages is from our platoon in front of us. Updating the info...");
        //         updatesOrAddMemberInfo(senderId, senderBsmId, cmdSpeed, dtDistance, curSpeed);
        //         this.leaderID = platoon.isEmpty() ? psl.getMobilityRouter().getHostMobilityId() : platoon.get(0).staticId;
                ROS_DEBUG("The first vehicle in our list is now " , leaderID);
            } else{
                ROS_DEBUG("This STATUS message is not from our platoon. We ignore this message with id: " , senderId);
            }
        }else {
            // If we are currently in any leader state, we only updates platoon member based on platoon ID
            if(currentPlatoonID == platoonId) {
                ROS_DEBUG("This STATUS messages is from our platoon. Updating the info...");
        //         updatesOrAddMemberInfo(senderId, senderBsmId, cmdSpeed, dtDistance, curSpeed);
            }
        }
    }

    

    void PlatoonManager::updatesOrAddMemberInfo(std::string senderId, std::string senderBsmId, double cmdSpeed, double dtDistance, double curSpeed) {

        bool isExisted = false;
        // update/add this info into the list
        for (PlatoonMember pm : platoon){
            if(pm.staticId == senderId) {
                pm.bsmId = senderBsmId;
                pm.commandSpeed = cmdSpeed;
                pm.vehiclePosition = dtDistance;
                pm.vehicleSpeed = curSpeed;
                pm.timestamp = ros::Time::now().toSec()*1000;
                ROS_DEBUG("Receive and update platooning info on vehicel " , pm.staticId);
                ROS_DEBUG("    BSM ID = "                                  , pm.bsmId);
                ROS_DEBUG("    Speed = "                                   , pm.vehicleSpeed);
                ROS_DEBUG("    Location = "                                , pm.vehiclePosition);
                ROS_DEBUG("    CommandSpeed = "                            , pm.commandSpeed);
                isExisted = true;
                break;
            }
        }

        if(!isExisted) {
            long cur_t = ros::Time::now().toSec()*1000;
            PlatoonMember* newMember = new PlatoonMember(senderId, senderBsmId, cmdSpeed, curSpeed, dtDistance, cur_t);
            platoon.push_back(*newMember);
            // Collections.sort(platoon, (a, b) -> (Double.compare(b.vehiclePosition, a.vehiclePosition)));?????????????
            ROS_DEBUG("Add a new vehicle into our platoon list " , newMember->staticId);
        }

    }



    int PlatoonManager::getTotalPlatooningSize(){
        if(isFollower) {
            return 2;//platoonSize;
        }
        return 1;//platoon.size() + 1;
    }
        
    double PlatoonManager::getPlatoonRearDowntrackDistance(){
        // if(this.platoon.size() == 0) {
        //     return psl.getRouteService().getCurrentDowntrackDistance();
        // }
        // return this.platoon.get(this.platoon.size() - 1).vehiclePosition;
        return 0.0;
    }

    PlatoonMember PlatoonManager::getLeader(){
        PlatoonMember leader = NULL;
        if(isFollower && platoon.size() != 0) {
            // return the first vehicle in the platoon as default if no valid algorithm applied
            leader = platoon[0];
        }

    }

}