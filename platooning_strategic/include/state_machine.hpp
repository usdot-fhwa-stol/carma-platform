#pragma once

#include <ros/ros.h>
#include <iostream>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_srvs/PlanManeuvers.h>
#include <cav_msgs/PlanType.h>
#include <mutex>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <platoon_manager.hpp>

namespace platoon_strategic
{
    struct PlatoonPlan {
        bool valid;
        long planStartTime;
        std::string planId;
        std::string peerId;
    };

    /**
        * A response to an MobilityRequest message.
        * ACK - indicates that the plugin accepts the MobilityRequest and will handle making any adjustments needed to avoid a conflict
        * NACK - indicates that the plugin rejects the MobilityRequest and would suggest the other vehicle replan
        * NO_RESPONSE - indicates that the plugin is indifferent but sees no conflict
    */

    enum MobilityRequestResponse {
            ACK,
            NACK,
            NO_RESPONSE
    };

    enum PlatoonState{
        STANDBY,
        LEADERWAITING,
        LEADER,
        CANDIDATEFOLLOWER,
        FOLLOWER
    };

    class PlatooningStateMachine
    {
    public:
        MobilityRequestResponse onMobilityRequestMessage(cav_msgs::MobilityRequest &msg);
        void onMobilityResponseMessage(cav_msgs::MobilityResponse &msg);
        void onMobilityOperationMessage(cav_msgs::MobilityOperation &msg);
        cav_msgs::Maneuver planManeuver(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time);
        
        PlatoonState current_platoon_state;
        std::string applicantID;
        PlatoonPlan current_plan;

        PlatoonManager *pm_;


    private:
    
        MobilityRequestResponse onMobilityRequestMessageFollower(cav_msgs::MobilityRequest &msg);
        void onMobilityResponseMessageFollower(cav_msgs::MobilityResponse &msg);
        void onMobilityOperationMessageFollower(cav_msgs::MobilityOperation &msg);

        MobilityRequestResponse onMobilityRequestMessageLeader(cav_msgs::MobilityRequest &msg);
        void onMobilityResponseMessageLeader(cav_msgs::MobilityResponse &msg);
        void onMobilityOperationMessageLeader(cav_msgs::MobilityOperation &msg);

        MobilityRequestResponse onMobilityRequestMessageLeaderWaiting(cav_msgs::MobilityRequest &msg);
        void onMobilityResponseMessageLeaderWaiting(cav_msgs::MobilityResponse &msg);
        void onMobilityOperationMessageLeaderWaiting(cav_msgs::MobilityOperation &msg);

        MobilityRequestResponse onMobilityRequestMessageCandidateFollower(cav_msgs::MobilityRequest &msg);
        void onMobilityResponseMessageCandidateFollower(cav_msgs::MobilityResponse &msg);
        void onMobilityOperationMessageCandidateFollower(cav_msgs::MobilityOperation &msg);


        MobilityRequestResponse onMobilityRequestMessageStandby(cav_msgs::MobilityRequest &msg);
        void onMobilityResponseMessageStandby(cav_msgs::MobilityResponse &msg);
        void onMobilityOperationMessageStandby(cav_msgs::MobilityOperation &msg);



        
        bool isVehicleRightInFront(std::string rearVehicleBsmId, double downtrack);
        double maxAllowedJoinTimeGap = 15.0;
        double maxAllowedJoinGap = 90;
        int maxPlatoonSize = 10;
        double vehicleLength = 5.0;
        std::mutex plan_mutex_;
        int infoMessageInterval;
        std::string targetLeaderId;
        std::string targetPlatoonId;
        std::string OPERATION_INFO_TYPE = "INFO";
        std::string OPERATION_STATUS_TYPE = "STATUS";
    };
}

