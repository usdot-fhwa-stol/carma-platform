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
#include <algorithm>
#include "LocalizationManager.h"

namespace localizer
{
// TODO timeout logic
LocalizationManager::LocalizationManager(PosePublisher pose_pub, TransformPublisher transform_pub,
                                         StatePublisher state_pub, LocalizationManagerConfig config)
  : pose_pub_(pose_pub)
  , transform_pub_(transform_pub)
  , state_pub_(state_pub)
  , config_(config)
  , transition_table_(config_.localization_mode)
{
  transition_table_.setTransitionCallback(
      std::bind(&LocalizationManager::stateTransitionCallback, this, std::placeholders::_1))
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

double LocalizationManager::computeNDTFreq(const ros::Time& new_stamp)
{
  if (prev_ndt_stamp_ == ros::Time(0))
  {  // Check if this is the first data point
    prev_ndt_stamp_ = new_stamp;
    // When no historic data is available force the frequency into the operational range
    return config_.ndt_frequency_degraded_threshold * 2;
  }

  if (new_stamp < prev_ndt_stamp_)
  {
    ROS_ERROR_STREAM("LocalizationManager received NDT data out of order. Prev stamp was "
                     << prev_ndt_stamp_ << " new stamp is " << new_stamp);
    // When invalid data is received from NDT force the frequency into the fault range
    return config_.ndt_frequency_fault_threshold / 2;
  }

  return 1.0 / (new_stamp - prev_ndt_stamp_).toSec();  // Convert delta to frequency (Hz = 1/s)
}

// callbacks
void LocalizationManager::poseAndStatsCallback(const geometry_msgs::PoseStampedConstPtr& pose,
                                               const autoware_msgs::NDTStatConstPtr& stats)
{
  double ndt_freq = computeNDTFreq(pose.stamp);  // TODO create function
  if (stats.score >= config_.fitness_score_fault_threshold || ndt_freq <= config_.ndt_frequency_fault_threshold)
  {
    transition_table_.signal(LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE);
  }
  else if (stats.score >= config_.fitness_score_degraded_threshold ||
           ndt_freq <= config_.ndt_frequency_degraded_threshold)
  {
    transition_table_.signal(LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE);
  }
  else
  {
    transition_table_.signal(LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE);
  }
}

void LocalizationManager::gnssPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (transition_table_.getState() == LocalizationState::DEGRADED_NO_LIDAR_FIX)
  {
    publishPoseStamped(*msg);
  }
}

void LocalizationManager::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  transition_table_.signal(LocalizationSignal::INITIAL_POSE);
}

void LocalizationManager::systemAlertCallback(const cav_msgs::SystemAlertConstPtr& alert)
{
  if (LIDAR_FAILURE_STRINGS.find(alert.description) != LIDAR_FAILURE_STRINGS.end())
  {
    transition_table_.signal(LocalizationSignal::LIDAR_SENSOR_FAILURE);
  }
}

cav_msgs::LocalizationStatusReport LocalizationManager::stateToMsg(LocalizationState state)
{
  cav_msgs::LocalizationStatusReport msg;
  switch (state)
  {
    case UNINITIALIZED:
      msg.status = cav_msgs::LocalizationStatusReport::UNINITIALIZED;
      break;
    case INITIALIZING:
      msg.status = cav_msgs::LocalizationStatusReport::INITIALIZING;
      break;
    case OPERATIONAL:
      msg.status = cav_msgs::LocalizationStatusReport::OPERATIONAL;
      break;
    case DEGRADED:
      msg.status = cav_msgs::LocalizationStatusReport::DEGRADED;
      break;
    case DEGRADED_NO_LIDAR_FIX:
      msg.status = cav_msgs::LocalizationStatusReport::DEGRADED_NO_LIDAR_FIX;
      break;
    case AWAIT_MANUAL_INITIALIZATION:
      msg.status = cav_msgs::LocalizationStatusReport::AWAIT_MANUAL_INITIALIZATION;
      break;
    default:
      throw std::invalid_argument("LocalizationManager states do not match cav_msgs::LocalizationStatusReport "
                                  "states") break;
  }
  msg.header.stamp = ros::Time::now();
}

void LocalizationManager::stateTransitionCallback(LocalizationState prev_state, LocalizationState new_state,
                                                  LocalizationSignal signal)
{
  switch (new_state)
  {
    case LocalizationState::INITIALIZING:
      // TODO create timer. Do we need to check the mode?
      break;
    case LocalizationState::DEGRADED_NO_LIDAR_FIX:
      // TODO create timer. Do we need to check the mode?
      break;
    default:
      break;
  }
}

bool LocalizationManager::onSpin()
{
  // Evaluate Timeouts
  if (transition_table_.getState() == INITIALIZING)
  {
  }

  // Create and publish status report message
  cav_msgs::LocalizationStatusReport msg = stateToMsg(transition_table_.getState());
  state_pub_(msg);
}

}  // namespace localizer
