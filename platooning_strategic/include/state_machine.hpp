#pragma once

#include <ros/ros.h>
#include <iostream>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_srvs/PlanManeuvers.h>
#include <cav_msgs/PlanType.h>
#include <mutex>

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
        virtual MobilityRequestResponse onMobilityRequestMessage(cav_msgs::MobilityRequest &msg) = 0;
        virtual void onMobilityResponseMessage(cav_msgs::MobilityResponse &msg) = 0;
        virtual void onMobilityOperationMessage(cav_msgs::MobilityOperation &msg) = 0;
        virtual cav_msgs::Maneuver planManeuver(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time) = 0;
        PlatoonState current_platoon_state;

    // protected:
    //     std::string applicantID;
    };
}

