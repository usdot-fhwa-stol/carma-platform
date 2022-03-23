/*
 * Copyright (C) 2020 LEIDOS.
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <pose_to_tf/PoseToTF2.h>
#include <pose_to_tf/PoseToTF2Config.h>

using namespace pose_to_tf;

TEST(PoseToTF2, test_methods)
{
  PoseToTF2Config config;
  config.child_frame = "base_link";
  config.default_parent_frame = "map";
  bool msg_set = false;
  geometry_msgs::TransformStamped msg;

  PoseToTF2 manager(config, [&msg, &msg_set](auto in) { msg_set = true; msg = in; });

  ros::Time now(1.5);
  ros::Time::setNow(now);

  geometry_msgs::Pose poseA;
  poseA.position.x = 1.2;
  poseA.position.y = 2.3;
  poseA.position.z = 5.0;
  poseA.orientation.x = 3.0;
  poseA.orientation.y = 3.4;
  poseA.orientation.z = 9.9;
  poseA.orientation.w = 1.0;

  geometry_msgs::PoseStamped poseB;

  poseB.pose = poseA;
  poseB.header.stamp = ros::Time(0.5);
  poseB.header.frame_id = "odom";

  geometry_msgs::PoseWithCovariance poseC;

  poseC.pose = poseA;

  geometry_msgs::PoseWithCovarianceStamped poseD;

  poseD.pose.pose = poseA;
  poseD.header.stamp = ros::Time(0.5);
  poseD.header.frame_id = "odom";


  ASSERT_FALSE(msg_set);
  geometry_msgs::PoseConstPtr poseA_ptr(new geometry_msgs::Pose(poseA));
  manager.poseCallback(poseA_ptr);

  ASSERT_TRUE(msg_set);
  ASSERT_EQ(poseA.position.x, msg.transform.translation.x);
  ASSERT_EQ(poseA.position.y, msg.transform.translation.y);
  ASSERT_EQ(poseA.position.z, msg.transform.translation.z);
  ASSERT_EQ(poseA.orientation.x, msg.transform.rotation.x);
  ASSERT_EQ(poseA.orientation.y, msg.transform.rotation.y);
  ASSERT_EQ(poseA.orientation.z, msg.transform.rotation.z);
  ASSERT_EQ(poseA.orientation.w, msg.transform.rotation.w);

  ASSERT_EQ(0, msg.header.frame_id.compare(config.default_parent_frame));
  ASSERT_EQ(0, msg.child_frame_id.compare(config.child_frame));
  ASSERT_NEAR(1.5, msg.header.stamp.toSec(), 0.000000001);
  

  msg_set = false;
  geometry_msgs::PoseStampedConstPtr poseB_ptr(new geometry_msgs::PoseStamped(poseB));
  manager.poseStampedCallback(poseB_ptr);

  ASSERT_TRUE(msg_set);
  ASSERT_EQ(poseB.pose.position.x, msg.transform.translation.x);
  ASSERT_EQ(poseB.pose.position.y, msg.transform.translation.y);
  ASSERT_EQ(poseB.pose.position.z, msg.transform.translation.z);
  ASSERT_EQ(poseB.pose.orientation.x, msg.transform.rotation.x);
  ASSERT_EQ(poseB.pose.orientation.y, msg.transform.rotation.y);
  ASSERT_EQ(poseB.pose.orientation.z, msg.transform.rotation.z);
  ASSERT_EQ(poseB.pose.orientation.w, msg.transform.rotation.w);
  
  ASSERT_EQ(0, msg.header.frame_id.compare(poseB.header.frame_id));
  ASSERT_EQ(0, msg.child_frame_id.compare(config.child_frame));
  ASSERT_NEAR(poseB.header.stamp.toSec(), msg.header.stamp.toSec(), 0.000000001);

  msg_set = false;
  geometry_msgs::PoseWithCovarianceConstPtr poseC_ptr(new geometry_msgs::PoseWithCovariance(poseC));
  manager.poseWithCovarianceCallback(poseC_ptr);

  ASSERT_TRUE(msg_set);
  ASSERT_EQ(poseC.pose.position.x, msg.transform.translation.x);
  ASSERT_EQ(poseC.pose.position.y, msg.transform.translation.y);
  ASSERT_EQ(poseC.pose.position.z, msg.transform.translation.z);
  ASSERT_EQ(poseC.pose.orientation.x, msg.transform.rotation.x);
  ASSERT_EQ(poseC.pose.orientation.y, msg.transform.rotation.y);
  ASSERT_EQ(poseC.pose.orientation.z, msg.transform.rotation.z);
  ASSERT_EQ(poseC.pose.orientation.w, msg.transform.rotation.w);
  
  ASSERT_EQ(0, msg.header.frame_id.compare(config.default_parent_frame));
  ASSERT_EQ(0, msg.child_frame_id.compare(config.child_frame));
  ASSERT_NEAR(1.5, msg.header.stamp.toSec(), 0.000000001);

  msg_set = false;
  geometry_msgs::PoseWithCovarianceStampedConstPtr poseD_ptr(new geometry_msgs::PoseWithCovarianceStamped(poseD));
  manager.poseWithCovarianceStampedCallback(poseD_ptr);

  ASSERT_TRUE(msg_set);
  ASSERT_EQ(poseD.pose.pose.position.x, msg.transform.translation.x);
  ASSERT_EQ(poseD.pose.pose.position.y, msg.transform.translation.y);
  ASSERT_EQ(poseD.pose.pose.position.z, msg.transform.translation.z);
  ASSERT_EQ(poseD.pose.pose.orientation.x, msg.transform.rotation.x);
  ASSERT_EQ(poseD.pose.pose.orientation.y, msg.transform.rotation.y);
  ASSERT_EQ(poseD.pose.pose.orientation.z, msg.transform.rotation.z);
  ASSERT_EQ(poseD.pose.pose.orientation.w, msg.transform.rotation.w);
  
  ASSERT_EQ(0, msg.header.frame_id.compare(poseD.header.frame_id));
  ASSERT_EQ(0, msg.child_frame_id.compare(config.child_frame));
  ASSERT_NEAR(poseD.header.stamp.toSec(), msg.header.stamp.toSec(), 0.000000001);
}
