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
#include <carma_utils/CARMAUtils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/NDTStat.h>
#include "ndt_reliability_counter.h"
#include "LocalizerMode.h"
#include "LocalizationManagerConfig.h"
#include "LocalizationManager.h"

namespace localizer
{
class Localizer
{
public:
  Localizer();

  // general starting point of this node
  void run();

  // Publication callbacks
  void publishPoseStamped(const geometry_msgs::PoseStamped& msg);
  void publishTransform(const geometry_msgs::TransformStamped& msg);

private:
  // node handles
  ros::CARMANodeHandle nh_, pnh_;

  // transform broadcaster
  tf2_ros::TransformBroadcaster br_;

  // subscribers
  ros::Subscriber ndt_pose_sub_;
  ros::Subscriber ndt_score_sub_;
  ros::Subscriber gnss_pose_sub_;

  // publisher
  ros::Publisher pose_pub_;

  // member variables
  double spin_rate_{ 10 };
};

}  // namespace localizer
