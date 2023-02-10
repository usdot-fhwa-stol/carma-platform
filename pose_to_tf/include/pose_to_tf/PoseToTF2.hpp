#pragma once
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
#include <pose_to_tf/PoseToTF2Config.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace pose_to_tf
{
/**
 * \brief Primary logic class for the PoseToTF2 node
 */
class PoseToTF2
{
public:
  using TransformPublisher = std::function<void(const geometry_msgs::msg::TransformStamped&)>;

  /**
   * \brief Constructor
   *
   * \param transform_pub A callback to trigger transform broadcast
   */
  PoseToTF2(PoseToTF2Config config, TransformPublisher transform_pub,std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> node);

  /**
   * \brief Callback for new pose stamped messages
   *
   * \param msg The pose message to forward
   */
  void poseStampedCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg);

  /**
   * \brief Callback for new pose stamped messages
   *
   * \param msg The pose message to forward
   */
  void poseWithCovarianceStampedCallback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg);

  /**
   * \brief Callback for new pose stamped messages
   *
   * \param msg The pose message to forward
   */
  void poseCallback(geometry_msgs::msg::Pose::UniquePtr msg);

  /**
   * \brief Callback for new pose with covariance messages
   *
   * \param msg The pose message to forward
   */
  void poseWithCovarianceCallback(geometry_msgs::msg::PoseWithCovariance::UniquePtr msg);

private:

  std::shared_ptr <carma_ros2_utils::CarmaLifecycleNode> node_;
  
  PoseToTF2Config config_;
  TransformPublisher transform_pub_;
};
}//namespace pose_to_tf
