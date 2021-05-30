#include "ros/ros.h"
#include <carma_utils/CARMAUtils.h>

#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectList.h>
#include <geometry_msgs/PoseArray.h>
#include <cav_msgs/RoadwayObstacleList.h>

ros::Publisher pose_array_pub;

void external_object_callback(const cav_msgs::ExternalObjectListPtr msg) {

  geometry_msgs::PoseArray posearray;
  posearray.header.stamp = ros::Time::now(); 
  posearray.header.frame_id = "map";

  for (auto& obj : msg->objects)
  {
    ROS_INFO("I heard:");

    for (auto& p : obj.predictions) {
        posearray.poses.push_back(p.predicted_position);
    }

    pose_array_pub.publish(posearray);
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "motion_computation_visualizer");

  ros::CARMANodeHandle nh_;

  ros::Subscriber sub = nh_.subscribe("external_objects", 1000, external_object_callback);
  pose_array_pub = nh_.advertise<geometry_msgs::PoseArray>("motion_computation_visualize", 1);

  nh_.spin();

  return 0;
}

