#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectList.h>
#include "motion_computation_visualizer.h"
#include <geometry_msgs/PoseArray.h>
#include <cav_msgs/RoadwayObstacleList.h>

ros::Publisher pose_array_pub;

void chatterCallback(const cav_msgs::ExternalObjectListPtr msg) {

  visualization_msgs::MarkerArray line_list;

  geometry_msgs::PoseArray posearray;
  posearray.header.stamp = ros::Time::now(); 
  posearray.header.frame_id = "map";

  for (auto& obj : msg->objects)
  {
    ROS_INFO("I heard:");

    for (auto& p : obj.predictions) {
        posearray.poses.push_back(p.predicted_position);
        // line_list.markers.push_back(p.predicted_position);
    }

    pose_array_pub.publish(posearray);
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "motion_computation_visualizer");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/environment/external_objects", 1000, chatterCallback);
  pose_array_pub = n.advertise<geometry_msgs::PoseArray>("motion_computation_visualize", 1);

  ros::spin();

  return 0;
}

