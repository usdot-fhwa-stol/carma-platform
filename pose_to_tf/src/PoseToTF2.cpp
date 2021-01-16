/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <functional>
#include <pose_to_tf/PoseToTF2.h>

namespace tf2
{
namespace
{
void convert(const geometry_msgs::Pose& pose, geometry_msgs::Transform& trans)
{
  trans.rotation = pose.orientation;
  trans.translation.x = pose.position.x;
  trans.translation.y = pose.position.y;
  trans.translation.z = pose.position.z;
}

void convert(const geometry_msgs::PoseWithCovariance& pose, geometry_msgs::Transform& trans)
{
  convert(pose.pose, trans);
}

void convert(const geometry_msgs::PoseStamped& pose, geometry_msgs::TransformStamped& trans)
{
  convert(pose.pose, trans.transform);
  trans.header = pose.header;
}

void convert(const geometry_msgs::PoseWithCovarianceStamped& pose, geometry_msgs::TransformStamped& trans)
{
  convert(pose.pose, trans.transform);
  trans.header = pose.header;
}
}  // namespace
}  // namespace tf2

namespace pose_to_tf
{
PoseToTF2::PoseToTF2(PoseToTF2Config config, TransformPublisher transform_pub)
  : config_(config), transform_pub_(transform_pub)
{
}

void PoseToTF2::poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::TransformStamped out_tf;
  tf2::convert(*msg, out_tf);
  out_tf.child_frame_id = config_.child_frame;
  transform_pub_(out_tf);
}

void PoseToTF2::poseWithCovarianceStampedCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  geometry_msgs::TransformStamped out_tf;
  tf2::convert(*msg, out_tf);
  out_tf.child_frame_id = config_.child_frame;
  transform_pub_(out_tf);
}

void PoseToTF2::poseCallback(const geometry_msgs::PoseConstPtr& msg)
{
  geometry_msgs::TransformStamped out_tf;
  tf2::convert(*msg, out_tf.transform);
  out_tf.header.stamp = ros::Time::now();
  out_tf.header.frame_id = config_.default_parent_frame;
  out_tf.child_frame_id = config_.child_frame;
  transform_pub_(out_tf);
}

void PoseToTF2::poseWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceConstPtr& msg)
{
  geometry_msgs::TransformStamped out_tf;
  tf2::convert(*msg, out_tf.transform);
  out_tf.header.stamp = ros::Time::now();
  out_tf.header.frame_id = config_.default_parent_frame;
  out_tf.child_frame_id = config_.child_frame;
  transform_pub_(out_tf);
}
}  // namespace pose_to_tf
