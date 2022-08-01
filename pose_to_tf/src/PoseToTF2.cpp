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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <functional>
#include "pose_to_tf/PoseToTF2.hpp"

namespace tf2
{
namespace
{
void convert(const geometry_msgs::msg::Pose& pose, geometry_msgs::msg::Transform& trans)
{
  trans.rotation = pose.orientation;
  trans.translation.x = pose.position.x;
  trans.translation.y = pose.position.y;
  trans.translation.z = pose.position.z;
}

void convert(const geometry_msgs::msg::PoseWithCovariance& pose, geometry_msgs::msg::Transform& trans)
{
  convert(pose.pose, trans);
}

void convert(const geometry_msgs::msg::PoseStamped& pose, geometry_msgs::msg::TransformStamped& trans)
{
  convert(pose.pose, trans.transform);
  trans.header = pose.header;
}

void convert(const geometry_msgs::msg::PoseWithCovarianceStamped& pose, geometry_msgs::msg::TransformStamped& trans)
{
  convert(pose.pose, trans.transform);
  trans.header = pose.header;
}
}  // namespace
}  // namespace tf2

namespace pose_to_tf
{

PoseToTF2::PoseToTF2(PoseToTF2Config config, TransformPublisher transform_pub, std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> node)
  : config_(config), transform_pub_(transform_pub), node_(node)  
{
}

void PoseToTF2::poseStampedCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
  geometry_msgs::msg::TransformStamped out_tf;
  tf2::convert(*msg, out_tf);
  out_tf.child_frame_id = config_.child_frame;
  transform_pub_(out_tf);
}

void PoseToTF2::poseWithCovarianceStampedCallback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg)
{
  geometry_msgs::msg::TransformStamped out_tf;
  tf2::convert(*msg, out_tf);
  out_tf.child_frame_id = config_.child_frame;
  transform_pub_(out_tf);
}

void PoseToTF2::poseCallback(geometry_msgs::msg::Pose::UniquePtr msg)
{
  geometry_msgs::msg::TransformStamped out_tf;
  tf2::convert(*msg, out_tf.transform);
  out_tf.header.stamp = node_->now();
  out_tf.header.frame_id = config_.default_parent_frame;
  out_tf.child_frame_id = config_.child_frame;
  transform_pub_(out_tf);
}

void PoseToTF2::poseWithCovarianceCallback(geometry_msgs::msg::PoseWithCovariance::UniquePtr msg)
{
  geometry_msgs::msg::TransformStamped out_tf;
  tf2::convert(*msg, out_tf.transform);
  out_tf.header.stamp = node_->now();
  out_tf.header.frame_id = config_.default_parent_frame;
  out_tf.child_frame_id = config_.child_frame;
  transform_pub_(out_tf);
}
}//namespace pose_to_tf
