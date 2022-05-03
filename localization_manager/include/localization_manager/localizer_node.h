#pragma once
/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <carma_localization_msgs/msg/localization_status_report.h>
#include "LocalizationTypes.h"
#include "LocalizationManagerConfig.h"
#include "LocalizationManager.h"

namespace localization_manager
{
/**
 * \brief Node class for this package
 */
class Localizer
{
public:
  /**
   * \brief Default constructor
   */
  Localizer()=default;

  /**
   * \brief Primary entry function for node execution. Will not exit until node shutdown.
   */
  void run();

  /**
   * \brief Callback to publish the selected pose
   * \param msg The pose to publish
   */
  void publishPoseStamped(const geometry_msgs::msg::PoseStamped& msg) const;

  /**
   * \brief Callback to publish the provided localization status report
   * \param msg The report to publish
   */
  void publishStatus(const carma_localization_msgs::msg::LocalizationStatusReport& msg) const;

  /**
   * \brief Callback to publish the initial pose deemed suitable to intialize NDT
   * \param msg The msg to publish
   */
  void publishManagedInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) const;


  /**
   * \brief Synchronized callback for pose and stats data for usage with message_filters.
   *        Provides exception handling. 
   * \param pose The received pose message
   * \param stats The received stats message
   */ 
  void poseAndStatsCallback(const geometry_msgs::msg::PoseStampedConstPtr& pose,
                            const autoware_msgs::msg::NDTStatConstPtr& stats) const;

private:
  // node handles
  ros::CARMANodeHandle nh_, pnh_;

  // subscribers
  ros::Subscriber ndt_pose_sub_;
  ros::Subscriber ndt_score_sub_;
  ros::Subscriber gnss_pose_sub_;
  ros::Subscriber initialpose_sub_;

  // publisher
  ros::Publisher pose_pub_;
  ros::Publisher state_pub_;
  ros::Publisher managed_initial_pose_pub_;
  ros::Timer pose_timer_;
  double pose_pub_rate_ = 10.0;

  std::unique_ptr<LocalizationManager> manager_; // Worker object

  // Message filters policies
  typedef message_filters::sync_policies::ExactTime<geometry_msgs::msg::PoseStamped, autoware_msgs::msg::NDTStat>
      PoseStatsSyncPolicy;
  typedef message_filters::Synchronizer<PoseStatsSyncPolicy> PoseStatsSynchronizer;
};

}  // namespace localization_manager
