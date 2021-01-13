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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <functional>
#include <pose_to_tf/PoseToTF2Config.h>

namespace pose_to_tf
{
/**
 * \brief Primary logic class for the PoseToTF2 node
 */
class PoseToTF2
{
public:
  using TransformPublisher = std::function<void(const geometry_msgs::TransformStamped&)>;

  /**
   * \brief Constructor
   *
   * \param transform_pub A callback to trigger transform broadcast
   */
  PoseToTF2(PoseToTF2Config config, TransformPublisher transform_pub);

  /**
   * \brief Callback for new pose stamped messages
   *
   * \param msg The pose message to forward
   */
  void poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  /**
   * \brief Callback for new pose stamped messages
   *
   * \param msg The pose message to forward
   */
  void poseWithCovarianceStampedCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  /**
   * \brief Callback for new pose stamped messages
   *
   * \param msg The pose message to forward
   */
  void poseCallback(const geometry_msgs::PoseConstPtr& msg);

  /**
   * \brief Callback for new pose with covariance messages
   *
   * \param msg The pose message to forward
   */
  void poseWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceConstPtr& msg);

private:

  PoseToTF2Config config_;
  TransformPublisher transform_pub_;
};

}  // namespace pose_to_tf
