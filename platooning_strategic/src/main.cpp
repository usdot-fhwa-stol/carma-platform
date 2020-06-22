
#include <ros/ros.h>
#include "platoon_strategic.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "platoon_strategic");
    platoon_strategic::PlatoonStrategicPlugin node;
    node.run();
    return 0;
};