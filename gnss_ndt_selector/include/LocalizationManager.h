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
#include <autoware_msgs/NDTStat.h>
#include "ndt_reliability_counter.h"
#include <functional>
#include "LocalizerMode.h"
#include "LocalizationManagerConfig.h"

namespace localizer
{
class LocalizationManager
{
public:
  using PosePublisher = std::function<void(const geometry_msgs::PoseStamped&)>;
  using TransformPublisher = std::function<void(const geometry_msgs::TransformStamped&)>;

  LocalizationManager(PosePublisher pose_pub, TransformPublisher transform_pub, LocalizationManagerConfig config);

  void ndtPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void gnssPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void ndtScoreCallback(const autoware_msgs::NDTStatConstPtr& msg);

private:
  PosePublisher pose_pub_;
  TransformPublisher transform_pub_;

  LocalizationManagerConfig config_;

  // reliability counter
  NDTReliabilityCounter counter_;

  void publishPoseStamped(const geometry_msgs::PoseStamped& pose);
};

}  // namespace localizer
