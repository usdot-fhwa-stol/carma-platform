#include <sstream>
#include <cav_msgs/SpeedAccel.h>

#include <ros/ros.h>
#include "new_guidance_commands/NewGuidanceCommands.hpp"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "new_guidance_commands");
  ros::NodeHandle nodeHandle("~");

  new_guidance_commands::NewGuidanceCommands NewGuidanceCommands(nodeHandle);

  ros::spin();
  return 0;
}

