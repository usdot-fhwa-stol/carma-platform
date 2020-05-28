#include "collision_checking/collision_checking.h"

namespace collision_checking {

    CollisionChecking::CollisionChecking(ros::NodeHandle &nodeHandle)
        : nodeHandle_(nodeHandle)
    {
        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters.");
        }

        ROS_INFO("Successfully launched node.");
    }

    CollisionChecking::~CollisionChecking()
    {
    }

    bool CollisionChecking::readParameters()
    {
        // nodeHandle_.param<std::string>("SpeedAccelPublisher_topic", SpeedAccelPublisherTopic_, "/republish/cmd_speed");
        // nodeHandle_.param<std::string>("WrenchEffortPublisher_topic", WrenchEffortPublisherTopic_, "/republish/cmd_longitudinal_effort");
        // nodeHandle_.param<std::string>("LateralControlPublisher_topic", LateralControlPublisherTopic_, "/republish/cmd_lateral");
        // nodeHandle_.param("publish_rate", rate, 10);
        // nodeHandle_.param("timeout_thresh", timeout, 0.5);

        return true;
    }

}