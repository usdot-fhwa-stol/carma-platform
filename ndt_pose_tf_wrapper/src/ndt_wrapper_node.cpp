#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

void ndtPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = msg->header.stamp;
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "base_link";
	transformStamped.transform.translation.x = msg->pose.position.x;
	transformStamped.transform.translation.y = msg->pose.position.y;
	transformStamped.transform.translation.z = msg->pose.position.z;
	transformStamped.transform.rotation.x = msg->pose.orientation.x;
	transformStamped.transform.rotation.y = msg->pose.orientation.y;
	transformStamped.transform.rotation.z = msg->pose.orientation.z;
	transformStamped.transform.rotation.w = msg->pose.orientation.w;
	br.sendTransform(transformStamped);
}

int main(int argc, char** argv){

  ros::init(argc, argv, "ndt_tf_wrapper");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("ndt_pose", 5, &ndtPoseCallback);
  ros::spin();
  return 0;

};
