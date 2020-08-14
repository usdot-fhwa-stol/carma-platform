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
#include <boost/shared_ptr.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <autoware_msgs/NDTStat.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/LocalizationStatusReport.h>
#include <functional>
#include <unordered_set>
#include <carma_utils/timers/Timer.h>
#include <carma_utils/timers/TimerFactory.h>
#include "LocalizationTypes.h"
#include "LocalizationManagerConfig.h"
#include "LocalizationTransitionTable.h"

namespace localizer
{
class LocalizationManager
{
public:
  using PosePublisher = std::function<void(const geometry_msgs::PoseStamped&)>;
  using TransformPublisher = std::function<void(const geometry_msgs::TransformStamped&)>;
  using StatePublisher = std::function<void(const cav_msgs::LocalizationStatusReport&)>;

  LocalizationManager(PosePublisher pose_pub, TransformPublisher transform_pub, StatePublisher state_pub,
                      LocalizationManagerConfig config,
                      std::unique_ptr<carma_utils::timers::TimerFactory> timer_factory);

  void gnssPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void poseAndStatsCallback(const geometry_msgs::PoseStampedConstPtr& pose,
                            const autoware_msgs::NDTStatConstPtr& stats);

  void systemAlertCallback(const cav_msgs::SystemAlertConstPtr& alert);

  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  bool onSpin();

  void stateTransitionCallback(LocalizationState prev_state, LocalizationState new_state, LocalizationSignal signal);

  void timerCallback(const ros::TimerEvent& event, const LocalizationState origin_state);

private:
  static const std::unordered_set<std::string> LIDAR_FAILURE_STRINGS; // Static const container defined in cpp file 

  PosePublisher pose_pub_;
  TransformPublisher transform_pub_;
  StatePublisher state_pub_;

  LocalizationManagerConfig config_;

  std::unique_ptr<carma_utils::timers::TimerFactory> timer_factory_;

  LocalizationTransitionTable transition_table_;

  ros::Time prev_ndt_stamp_ = ros::Time(0);

  boost::optional<std::unique_ptr<carma_utils::timers::Timer>> current_timer_;

  cav_msgs::LocalizationStatusReport stateToMsg(LocalizationState state);

  void publishPoseStamped(const geometry_msgs::PoseStamped& pose);

  double computeNDTFreq(const ros::Time& new_stamp);
};

}  // namespace localizer

// TODO how to handle GPS failure