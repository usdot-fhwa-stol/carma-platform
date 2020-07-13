
#include <ros/ros.h>
#include "platoon_control.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "platoon_control");
    ros::CARMANodeHandle nh("");
    platoon_control::PlatoonControlPlugin node;
    node.run();
    return 0;
};