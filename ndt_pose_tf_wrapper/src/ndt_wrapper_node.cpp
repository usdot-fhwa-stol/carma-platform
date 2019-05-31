/*
 * Copyright (C) 2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

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
