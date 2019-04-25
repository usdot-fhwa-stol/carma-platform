#include "new_guidance_commands/NewGuidanceCommands.hpp"

namespace new_guidance_commands {

NewGuidanceCommands::NewGuidanceCommands(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &NewGuidanceCommands::topicCallback, this);
  // serviceServer_ = nodeHandle_.advertiseService("placeholder",
  //                                               &NewGuidanceCommands::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}

NewGuidanceCommands::~NewGuidanceCommands()
{
}

bool NewGuidanceCommands::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void NewGuidanceCommands::topicCallback(const cav_msgs::SpeedAccel& msg)
{
  ROS_INFO("I heard");
}

} // namespace new_guidance_commands