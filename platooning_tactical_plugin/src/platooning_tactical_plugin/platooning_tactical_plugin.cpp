#include "platooning_tactical_plugin/platooning_tactical_plugin.h"

namespace platooning_tactical_plugin {

    PlatooningTacticalPlugin::PlatooningTacticalPlugin(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle) {

        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters.");
        }

        waypoint_subscriber_ = nodeHandle_.subscribe("final_waypoint", 1, &PlatooningTacticalPlugin::waypoint_subscriber_callback, this);

        ROS_INFO("Successfully launched node.");
    }

    PlatooningTacticalPlugin::~PlatooningTacticalPlugin()
    {
    }

    bool PlatooningTacticalPlugin::readParameters()
    {
        // if (!nodeHandle_.getParam("trajectory_time_length", trajectory_time_length)) {
        //     return false;
        // }

        return true;
    }

    void PlatooningTacticalPlugin::waypoint_subscriber_callback(const autoware_msgs::Lane::ConstPtr& msg)
    {
        ROS_DEBUG("I heard SpeedAccel");
    };


    bool PlatooningTacticalPlugin::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
    {
        response.success = true;
        response.message = "ok";
        return true;
    };

}