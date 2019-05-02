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
  speedAccelsubscriber_ = nodeHandle_.subscribe(speedAccel_subscriberTopic_, 1,
                                      &NewGuidanceCommands::speedAccel_SubscriberCallback, this);
  wrenchEffortsubscriber_ = nodeHandle_.subscribe(wrenchEffort_subscriberTopic_, 1,
                                      &NewGuidanceCommands::wrenchEffort_SubscriberCallback, this);
  lateralControlsubscriber_ = nodeHandle_.subscribe(lateralControl_subscriberTopic_, 1,
                                      &NewGuidanceCommands::lateralControl_SubscriberCallback, this);

  speedAccel_publisher_ = nodeHandle_.advertise<cav_msgs::SpeedAccel>(speedAccel_publisherTopic_, 1000);
  wrenchEffort_publisher_ = nodeHandle_.advertise<std_msgs::Float32>(wrenchEffort_publisherTopic_, 1000);
  lateralControl_publisher_ = nodeHandle_.advertise<cav_msgs::LateralControl>(lateralControl_publisherTopic_, 1000);

  ROS_INFO("Successfully launched node.");
}

NewGuidanceCommands::~NewGuidanceCommands()
{
}

void NewGuidanceCommands::publisher(){
  ROS_INFO("publisher");
  speedAccel_Publisher();
  wrenchEffort_Publisher();
  lateralControl_Publisher();
} 

bool NewGuidanceCommands::readParameters()
{
  if (!nodeHandle_.getParam("speedAccel_subscriber_topic", speedAccel_subscriberTopic_) ||
      !nodeHandle_.getParam("wrenchEffort_subscriber_topic", wrenchEffort_subscriberTopic_) ||
      !nodeHandle_.getParam("lateralControl_subscriber_topic", lateralControl_subscriberTopic_) ||
      !nodeHandle_.getParam("speedAccel_publisher_topic", speedAccel_publisherTopic_) ||
      !nodeHandle_.getParam("wrenchEffort_publisher_topic", wrenchEffort_publisherTopic_) ||
      !nodeHandle_.getParam("lateralControl_publisher_topic", lateralControl_publisherTopic_) ||
      !nodeHandle_.getParam("rate", rate)){
            return false;
      }

  return true;
}


void NewGuidanceCommands::speedAccel_SubscriberCallback(const cav_msgs::SpeedAccel::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(SpeedAccel_msg_mutex);
    SpeedAccel_msg = msg;
    ROS_INFO("I heard SpeedAccel");
};

void NewGuidanceCommands::wrenchEffort_SubscriberCallback(const std_msgs::Float32::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(WrenchEffort_msg_mutex);
    WrenchEffort_msg = msg;
    ROS_INFO("I heard wrenchEffort");
};

void NewGuidanceCommands::lateralControl_SubscriberCallback(const cav_msgs::LateralControl::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(LateralControl_msg_mutex);
    LateralControl_msg = msg;
    ROS_INFO("I heard lateralControl");
};

void NewGuidanceCommands::speedAccel_Publisher(){
    std::lock_guard<std::mutex> lock(SpeedAccel_msg_mutex);
    if(SpeedAccel_msg != NULL) {
      speedAccel_publisher_.publish(SpeedAccel_msg);
      ROS_INFO("I publish SpeedAccel");
    }
};

void NewGuidanceCommands::wrenchEffort_Publisher(){
    std::lock_guard<std::mutex> lock(WrenchEffort_msg_mutex);
    if(WrenchEffort_msg != NULL) {
      wrenchEffort_publisher_.publish(WrenchEffort_msg);
      ROS_INFO("I publish wrenchEffort");
    }
};

void NewGuidanceCommands::lateralControl_Publisher(){
    std::lock_guard<std::mutex> lock(LateralControl_msg_mutex);
    if(LateralControl_msg != NULL) {
      lateralControl_publisher_.publish(LateralControl_msg);
      ROS_INFO("I publish lateralControl");
    }
};

} // namespace new_guidance_commands