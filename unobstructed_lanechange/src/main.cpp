
#include <ros/ros.h>
#include "unobstructed_lanechange.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "unobstructed_lanechange");
    unobstructed_lanechange::UnobstructedLaneChangePlugin node;
    node.run();
    return 0;
};