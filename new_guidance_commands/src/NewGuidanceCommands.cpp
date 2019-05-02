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
  enable_roboticsubscriber_ = nodeHandle_.subscribe(enable_robotic_subscriberTopic_, 1,
                                      &NewGuidanceCommands::enable_robotic_SubscriberCallback, this);



  wrenchEffort_publisher_ = nodeHandle_.advertise<std_msgs::Float32>(wrenchEffort_publisherTopic_, 1000);

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
  enable_robotic_Publisher();
} 

bool NewGuidanceCommands::readParameters()
{
  if (!nodeHandle_.getParam("speedAccel_subscriber_topic", speedAccel_subscriberTopic_) ||
      !nodeHandle_.getParam("wrenchEffort_subscriber_topic", wrenchEffort_subscriberTopic_) ||
      !nodeHandle_.getParam("lateralControl_subscriber_topic", lateralControl_subscriberTopic_) ||
      !nodeHandle_.getParam("enable_robotic_subscriber_topic", enable_robotic_subscriberTopic_) ||
      !nodeHandle_.getParam("speedAccel_publisher_topic", speedAccel_publisherTopic_) ||
      !nodeHandle_.getParam("wrenchEffort_publisher_topic", wrenchEffort_publisherTopic_) ||
      !nodeHandle_.getParam("lateralControl_publisher_topic", lateralControl_publisherTopic_)  ||
      !nodeHandle_.getParam("enable_robotic_publisher_topic", enable_robotic_publisherTopic_)){
            return false;
      }

  return true;
}


void NewGuidanceCommands::speedAccel_SubscriberCallback(const cav_msgs::SpeedAccel& msg){
    std::lock_guard<std::mutex> lock(SpeedAccel_msg_mutex);
    SpeedAccel_msg = msg;
    ROS_INFO("I heard SpeedAccel");
};

void NewGuidanceCommands::wrenchEffort_SubscriberCallback(const std_msgs::Float32::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(WrenchEffort_msg_mutex);
    WrenchEffort_msg = msg;
    ROS_INFO("I heard wrenchEffort");
};

void NewGuidanceCommands::lateralControl_SubscriberCallback(const cav_msgs::LateralControl& msg){
    std::lock_guard<std::mutex> lock(LateralControl_msg_mutex);
    LateralControl_msg = msg;
    ROS_INFO("I heard lateralControl");
};

void NewGuidanceCommands::enable_robotic_SubscriberCallback(const cav_msgs::RobotEnabled& msg){
    std::lock_guard<std::mutex> lock(RobotEnabled_msg_mutex);
    RobotEnabled_msg = msg;
    ROS_INFO("I heard enable_robotic");
};

void NewGuidanceCommands::speedAccel_Publisher(){
    std::lock_guard<std::mutex> lock(SpeedAccel_msg_mutex);
    if(SpeedAccel_msg.speed != NULL){
      speedAccel_publisher_.publish(SpeedAccel_msg);
      ROS_INFO("I publish SpeedAccel");
    }
};

void NewGuidanceCommands::wrenchEffort_Publisher(){
    std::lock_guard<std::mutex> lock(WrenchEffort_msg_mutex);
    if(WrenchEffort_msg != NULL){
      wrenchEffort_publisher_.publish(WrenchEffort_msg);
      ROS_INFO("I publish wrenchEffort");
    }
};

void NewGuidanceCommands::lateralControl_Publisher(){
    std::lock_guard<std::mutex> lock(LateralControl_msg_mutex);
    if(LateralControl_msg.max_accel != NULL){
      lateralControl_publisher_.publish(LateralControl_msg);
      ROS_INFO("I publish lateralControl");
    }
};

void  NewGuidanceCommands::enable_robotic_Publisher(){
    std::lock_guard<std::mutex> lock(RobotEnabled_msg_mutex);
    if(RobotEnabled_msg.robot_active != NULL){
      enable_robotic_publisher_.publish(RobotEnabled_msg);
      ROS_INFO("I publish enable_robotic");
    }
};

} // namespace new_guidance_commands