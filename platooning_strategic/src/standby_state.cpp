#include "standby_state.hpp"

namespace platoon_strategic
{
    

    MobilityRequestResponse StandbyState::onMobilityRequestMessage(cav_msgs::MobilityRequest &msg)
    {
        // In standby state, the plugin is not responsible for replying to any request messages
        return MobilityRequestResponse::NO_RESPONSE;
    }
    void StandbyState::onMobilityResponseMessage(cav_msgs::MobilityResponse &msg)
    {

    }
    void StandbyState::onMobilityOperationMessage(cav_msgs::MobilityOperation &msg)
    {
        
    }

    cav_msgs::Maneuver StandbyState::planManeuver(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time){
        cav_msgs::Maneuver maneuver_msg;
        return maneuver_msg;
    }

}