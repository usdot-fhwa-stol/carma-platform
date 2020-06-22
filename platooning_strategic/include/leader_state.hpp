#pragma once

#include "state_machine.hpp"

namespace platoon_strategic
{
    class LeaderState : public PlatooningStateMachine
    {
    public:
        LeaderState() {};

        MobilityRequestResponse onMobilityRequestMessage(cav_msgs::MobilityRequest &msg);
        void onMobilityResponseMessage(cav_msgs::MobilityResponse &msg);
        void onMobilityOperationMessage(cav_msgs::MobilityOperation &msg);
        cav_msgs::Maneuver planManeuver(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time);
        void composeMobilityOperationStatus(cav_msgs::MobilityOperation &msg, std::string type);
        void run();

    protected:
        PlatoonPlan current_plan;

    private:

        bool isVehicleRightInFront(std::string rearVehicleBsmId, double downtrack);

        double maxAllowedJoinTimeGap = 15.0;
        double maxAllowedJoinGap = 90;
        int maxPlatoonSize = 10;
        double vehicleLength = 5.0;
        std::mutex plan_mutex_;
        int infoMessageInterval;
    };

}
