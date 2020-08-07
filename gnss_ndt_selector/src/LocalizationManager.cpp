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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "LocalizationManager.h"

namespace localizer
{
LocalizationManager::LocalizationManager(PosePublisher pose_pub, TransformPublisher transform_pub,
                                         LocalizationManagerConfig config)
  : pose_pub_(pose_pub)
  , transform_pub_(transform_pub)
  , config_(config)
  , counter_(config.score_upper_limit, config.unreliable_message_upper_limit)
{
}

void LocalizationManager::publishPoseStamped(const geometry_msgs::PoseStamped& pose)
{
  geometry_msgs::TransformStamped tf_msg;

  tf_msg.header.stamp = pose.header.stamp;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = pose.header.frame_id;
  tf_msg.transform.translation.x = pose.pose.position.x;
  tf_msg.transform.translation.y = pose.pose.position.y;
  tf_msg.transform.translation.z = pose.pose.position.z;
  tf_msg.transform.rotation.x = pose.pose.orientation.x;
  tf_msg.transform.rotation.y = pose.pose.orientation.y;
  tf_msg.transform.rotation.z = pose.pose.orientation.z;
  tf_msg.transform.rotation.w = pose.pose.orientation.w;

  pose_pub_(pose);
  transform_pub_(tf_msg);
}

// callbacks
void LocalizationManager::ndtPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (config_.localization_mode == LocalizerMode::NDT)
  {
    publishPoseStamped(*msg);
  }
  else if (config_.localization_mode == LocalizerMode::AUTO &&
           counter_.getNDTReliabilityCounter() <= config_.unreliable_message_upper_limit)
  {
    publishPoseStamped(*msg);
  }
}

void LocalizationManager::gnssPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (config_.localization_mode == LocalizerMode::GNSS)
  {
    publishPoseStamped(*msg);
  }
  else if (config_.localization_mode == LocalizerMode::AUTO &&
           counter_.getNDTReliabilityCounter() > config_.unreliable_message_upper_limit)
  {
    publishPoseStamped(*msg);
  }
}

void LocalizationManager::ndtScoreCallback(const autoware_msgs::NDTStatConstPtr& msg)
{
  counter_.onNDTScore(msg->score);
}
}  // namespace localizer
