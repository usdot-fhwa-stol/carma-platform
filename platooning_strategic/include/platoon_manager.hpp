#pragma once

#include <ros/ros.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/PlanType.h>



///////////////EVERY FUNC SYNCRONIZED ??????

namespace platoon_strategic
{
        struct PlatoonMember{
            // Static ID is permanent ID for each vehicle
            std::string staticId;
            // Current BSM Id for each CAV
            std::string bsmId;
            // Vehicle real time command speed in m/s
            double commandSpeed;
            // Actual vehicle speed in m/s
            double vehicleSpeed;
            // Vehicle current down track distance on the current route in m
            double vehiclePosition;
            // The local time stamp when the host vehicle update any informations of this member
            long   timestamp;
            PlatoonMember(std::string staticId, std::string bsmId, double commandSpeed, double vehicleSpeed, double vehiclePosition, long timestamp): staticId(staticId),
            bsmId(bsmId), commandSpeed(commandSpeed), vehicleSpeed(vehicleSpeed), timestamp(timestamp) {}
        };


    class PlatoonManager
    {
    public:

        std::vector<PlatoonMember> platoon;


        void memberUpdates(std::string senderId, std::string platoonId, std::string senderBsmId, std::string params);

        /**
         * Given any valid platooning mobility STATUS operation parameters and sender staticId,
         * in leader state this method will add/updates the information of platoon member if it is using
         * the same platoon ID, in follower state this method will updates the vehicle information who
         * is in front of the subject vehicle or update platoon id if the leader is join another platoon
         * @param senderId sender ID for the current info
         * @param platoonId sender platoon id
         * @param senderBsmId sender BSM ID
         * @param params strategy params from STATUS message in the format of "CMDSPEED:xx,DOWNTRACK:xx,SPEED:xx"
         **/
        void updatesOrAddMemberInfo(std::string senderId, std::string senderBsmId, double cmdSpeed, double dtDistance, double curSpeed);

        int getTotalPlatooningSize();
        
        double getPlatoonRearDowntrackDistance();

        PlatoonMember getLeader();


    private:
    int platoonSize;
    std::string leaderID;
    std::string currentPlatoonID;
    bool isFollower;

    

    };
}