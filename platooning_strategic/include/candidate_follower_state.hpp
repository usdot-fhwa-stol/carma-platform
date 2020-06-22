#pragma once

#include "state_machine.hpp"
#include <mutex>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace platoon_strategic
{
    class CandidateFollowerState: public PlatooningStateMachine
    {
    public:
        CandidateFollowerState() {};

        MobilityRequestResponse onMobilityRequestMessage(cav_msgs::MobilityRequest &msg);
        void onMobilityResponseMessage(cav_msgs::MobilityResponse &msg);
        void onMobilityOperationMessage(cav_msgs::MobilityOperation &msg);
        cav_msgs::Maneuver planManeuver(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time);
        void composeMobilityOperationStatus(cav_msgs::MobilityOperation &msg);
        void run();

    protected:
        PlatoonPlan current_plan;

    private:
        std::mutex plan_mutex_;
        std::string targetLeaderId;

    };
}