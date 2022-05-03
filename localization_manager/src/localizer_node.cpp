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

#include "localization_manager/localizer_node.h"
#include <boost/bind/placeholders.hpp>

namespace localization_manager
{
namespace std_ph = std::placeholders;  // Make alias for std placeholders to differentiate from boost.

void Localizer::publishPoseStamped(const geometry_msgs::msg::PoseStamped& msg) const
{
  pose_pub_.publish(msg);
}

void Localizer::publishStatus(const carma_localization_msgs::msg::LocalizationStatusReport& msg) const
{
  state_pub_.publish(msg);
}


void Localizer::publishManagedInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) const
{
  managed_initial_pose_pub_.publish(msg);
}

void Localizer::poseAndStatsCallback(const geometry_msgs::msg::PoseStampedConstPtr& pose,
                                     const autoware_msgs::msg::NDTStatConstPtr& stats) const
{
  try
  {
    manager_->poseAndStatsCallback(pose, stats);
  }
  catch (const std::exception& e)
  {
    ros::CARMANodeHandle::handleException(e);
  }
}

void Localizer::run()
{
  // initialize node handles
  nh_ = ros::CARMANodeHandle();
  pnh_ = ros::CARMANodeHandle("~");
  // get params
  LocalizationManagerConfig config;

  pnh_.param<double>("pose_pub_rate", pose_pub_rate_, 10.0);
  pnh_.param<double>("fitness_score_degraded_threshold", config.fitness_score_degraded_threshold,
                     config.fitness_score_degraded_threshold);
  pnh_.param<double>("fitness_score_fault_threshold", config.fitness_score_fault_threshold,
                     config.fitness_score_fault_threshold);
  pnh_.param<double>("ndt_frequency_degraded_threshold", config.ndt_frequency_degraded_threshold,
                     config.ndt_frequency_degraded_threshold);
  pnh_.param<double>("ndt_frequency_fault_threshold", config.ndt_frequency_fault_threshold,
                     config.ndt_frequency_fault_threshold);
  pnh_.param<int>("auto_initialization_timeout", config.auto_initialization_timeout,
                  config.auto_initialization_timeout);
  pnh_.param<int>("gnss_only_operation_timeout", config.gnss_only_operation_timeout,
                  config.gnss_only_operation_timeout);
  pnh_.param<int>("sequential_timesteps_until_gps_operation", config.sequential_timesteps_until_gps_operation,
                  config.sequential_timesteps_until_gps_operation);
  pnh_.param<int>("gnss_data_timeout", config.gnss_data_timeout,
                  config.gnss_data_timeout);

  int localization_mode;
  pnh_.param<int>("localization_mode", localization_mode, 0);
  config.localization_mode = static_cast<LocalizerMode>(localization_mode);

  // Initialize worker object
  manager_.reset(new LocalizationManager(std::bind(&Localizer::publishPoseStamped, this, std_ph::_1),
                                         std::bind(&Localizer::publishStatus, this, std_ph::_1),
                                         std::bind(&Localizer::publishManagedInitialPose, this, std_ph::_1),
                                         config,
                                         std::make_unique<carma_utils::timers::ROSTimerFactory>()));

  // initialize subscribers
  gnss_pose_sub_ = nh_.subscribe("gnss_pose", 5, &LocalizationManager::gnssPoseCallback, manager_.get());
  initialpose_sub_ = nh_.subscribe("initialpose", 1, &LocalizationManager::initialPoseCallback, manager_.get());

  // Setup synchronized message_filters subscribers
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub(nh_, "ndt_pose", 5);
  message_filters::Subscriber<autoware_msgs::msg::NDTStat> stats_sub(nh_, "ndt_stat", 5);

  PoseStatsSynchronizer pose_stats_synchronizer(PoseStatsSyncPolicy(5), pose_sub, stats_sub);

  pose_stats_synchronizer.registerCallback(boost::bind(&Localizer::poseAndStatsCallback, this, _1, _2));

  ros::CARMANodeHandle::setSystemAlertCallback(std::bind(&LocalizationManager::systemAlertCallback, manager_.get(), std_ph::_1));

  // initialize publishers
  managed_initial_pose_pub_ = nh_.advertise<geometry_msgs::msg::PoseWithCovarianceStamped>("managed_initialpose", 1, true);
  pose_pub_ = nh_.advertise<geometry_msgs::msg::PoseStamped>("selected_pose", 5);
  state_pub_ = nh_.advertise<carma_localization_msgs::msg::LocalizationStatusReport>("localization_status", 1);

  // spin
  pose_timer_ = nh_.createTimer(
      ros::Duration(ros::Rate(pose_pub_rate_)),
      &LocalizationManager::posePubTick, 
      manager_.get());
  ros::CARMANodeHandle::spin();
}
}  // namespace localization_manager
