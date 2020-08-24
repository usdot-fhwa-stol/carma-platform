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
#include "localization_manager/LocalizationManager.h"

namespace localizer
{
// Initialize static values
const std::unordered_set<std::string> LocalizationManager::LIDAR_FAILURE_STRINGS({ "One LIDAR Failed", "Both LIDARS "
                                                                                                       "Failed" });

LocalizationManager::LocalizationManager(PosePublisher pose_pub, TransformPublisher transform_pub,
                                         StatePublisher state_pub, LocalizationManagerConfig config,
                                         std::unique_ptr<carma_utils::timers::TimerFactory> timer_factory)
  : pose_pub_(pose_pub)
  , transform_pub_(transform_pub)
  , state_pub_(state_pub)
  , config_(config)
  , timer_factory_(std::move(timer_factory))
  , transition_table_(config_.localization_mode)
{
  transition_table_.setTransitionCallback(std::bind(&LocalizationManager::stateTransitionCallback, this,
                                                    std::placeholders::_1, std::placeholders::_2,
                                                    std::placeholders::_3));
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

double LocalizationManager::computeFreq(const ros::Time& old_stamp, const ros::Time& new_stamp) const {
  return 1.0 / (new_stamp - old_stamp).toSec();  // Convert delta to frequency (Hz = 1/s)
}

double LocalizationManager::computeNDTFreq(const ros::Time& new_stamp)
{
  if (!prev_ndt_stamp_)
  {  // Check if this is the first data point
    prev_ndt_stamp_ = new_stamp;
    // When no historic data is available force the frequency into the operational range
    return config_.ndt_frequency_degraded_threshold * 2;
  }

  if (new_stamp <= prev_ndt_stamp_)
  {
    ROS_ERROR_STREAM("LocalizationManager received NDT data out of order. Prev stamp was "
                     << prev_ndt_stamp_.get() << " new stamp is " << new_stamp);
    // When invalid data is received from NDT force the frequency into the fault range
    return config_.ndt_frequency_fault_threshold / 2;
  }

  return computeFreq(prev_ndt_stamp_.get(), new_stamp);  // Convert delta to frequency (Hz = 1/s)
}

void LocalizationManager::poseAndStatsCallback(const geometry_msgs::PoseStampedConstPtr& pose,
                                               const autoware_msgs::NDTStatConstPtr& stats)
{
  double ndt_freq = computeNDTFreq(pose->header.stamp);
  ROS_INFO_STREAM("Received pose resulting in frequency value of " << ndt_freq << " with score of " << stats->score);

  if (stats->score >= config_.fitness_score_fault_threshold || ndt_freq <= config_.ndt_frequency_fault_threshold)
  {
    transition_table_.signal(LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE);
  }
  else if (stats->score >= config_.fitness_score_degraded_threshold ||
           ndt_freq <= config_.ndt_frequency_degraded_threshold)
  {
    transition_table_.signal(LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE);
  }
  else
  {
    transition_table_.signal(LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE);
  }

  const LocalizationState state =  transition_table_.getState();
  if (state == LocalizationState::INITIALIZING
      || state == LocalizationState::OPERATIONAL
      || state == LocalizationState::DEGRADED) {
     publishPoseStamped(*pose);
  }

  prev_ndt_stamp_ = pose->header.stamp;
}

void LocalizationManager::gnssPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (transition_table_.getState() == LocalizationState::DEGRADED_NO_LIDAR_FIX)
  {
    publishPoseStamped(*msg);
  }
}

void LocalizationManager::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  transition_table_.signal(LocalizationSignal::INITIAL_POSE);
}

void LocalizationManager::systemAlertCallback(const cav_msgs::SystemAlertConstPtr& alert)
{
  if (LIDAR_FAILURE_STRINGS.find(alert->description) != LIDAR_FAILURE_STRINGS.end())
  {
    transition_table_.signal(LocalizationSignal::LIDAR_SENSOR_FAILURE);
  }
}

void LocalizationManager::timerCallback(const ros::TimerEvent& event, const LocalizationState origin_state)
{
  // If there is already a timer callback in progress or the expected state has changed then return
  if (origin_state != transition_table_.getState())
    return;

  transition_table_.signal(LocalizationSignal::TIMEOUT);
}

void LocalizationManager::stateTransitionCallback(LocalizationState prev_state, LocalizationState new_state,
                                                  LocalizationSignal signal)
{

  // Mark the existing timer as expired if any
  if (current_timer_)
  {
    // NOTE unit testing of current algorithm depends on all timers being one-shot timers. 
    expired_timers_.push_back(std::move(current_timer_));
  }

  switch (new_state)
  {
    case LocalizationState::INITIALIZING:

      current_timer_ = timer_factory_->buildTimer(
          1, ros::Duration((double)config_.auto_initialization_timeout / 1000.0),
          std::bind(&LocalizationManager::timerCallback, this, std::placeholders::_1, new_state), true);

      break;
    case LocalizationState::DEGRADED_NO_LIDAR_FIX:

      current_timer_ = timer_factory_->buildTimer(
          1, ros::Duration((double)config_.gnss_only_operation_timeout / 1000.0),
          std::bind(&LocalizationManager::timerCallback, this, std::placeholders::_1, new_state), true);

      break;
    default:
      break;
  }

}

bool LocalizationManager::onSpin()
{
  // Clear any expired timers
  expired_timers_.clear();

  // Evaluate NDT Frequency if we have started receiving messages
  // This check provides protection against excessively long NDT computation times that do not trigger the callback
  if (prev_ndt_stamp_) {
    double freq = computeFreq(prev_ndt_stamp_.get(), ros::Time::now());
    if (freq <= config_.ndt_frequency_fault_threshold)
    {
      transition_table_.signal(LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE);
    }
    else if (freq <= config_.ndt_frequency_degraded_threshold)
    {
      transition_table_.signal(LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE);
    }
  }

  // Create and publish status report message
  cav_msgs::LocalizationStatusReport msg = stateToMsg(transition_table_.getState(), ros::Time::now());
  state_pub_(msg);

  return true;
}

LocalizationState LocalizationManager::getState() {
  return transition_table_.getState();
}

}  // namespace localizer
