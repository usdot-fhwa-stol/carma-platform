#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectList.h>
#include "motion_computation_visualizer.h"
#include <geometry_msgs/PoseArray.h>

ros::Publisher marker_pub;

void chatterCallback(const cav_msgs::ExternalObjectListPtr msg) {

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "/map";
  line_list.header.stamp = ros::Time::now();

  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  for (auto& obj : msg->objects)
  {
    ROS_INFO("I heard:");

    for (auto& p : obj.predictions) {
        line_list.points.push_back(p.predicted_position.position);
    }
    // p.predicted_velocity.linear
    // p.predicted_velocity.angular

    marker_pub.publish(line_list);
  }

  // ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/enviroment/roadway_object", 1000, chatterCallback);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::spin();

  return 0;
}

