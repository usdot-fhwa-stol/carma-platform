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

#include "localizer.h"
#include <boost/bind/placeholders.hpp>

namespace localizer
{
namespace std_ph = std::placeholders;  // Make alias for std placeholders to differentiate from boost.
Localizer::Localizer()
{
}

void Localizer::publishTransform(const geometry_msgs::TransformStamped& msg)
{
  br_.sendTransform(msg);
}

void Localizer::publishPoseStamped(const geometry_msgs::PoseStamped& msg)
{
  pose_pub_.publish(msg);
}

void Localizer::poseAndStatsCallback(const geometry_msgs::PoseStampedConstPtr& pose,
                                     const autoware_msgs::NDTStatConstPtr& stats)
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

  pnh_.param<double>("spin_rate", spin_rate_, 10.0);
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
  pnh_.param<int>("gnss_initialization_timeout", config.gnss_initialization_timeout,
                  config.gnss_initialization_timeout);

  int localization_mode;
  pnh_.param<int>("localization_mode", localization_mode, 0);
  config.localization_mode = static_cast<LocalizerMode>(localization_mode);

  manager_.reset(new LocalizationManager(std::bind(&Localizer::publishPoseStamped, this, std_ph::_1),
                                         std::bind(&Localizer::publishTransform, this, std_ph::_1),
                                         std::bind(&Localizer::publishStatus, this, std_ph::_1), config));

  // initialize subscribers
  gnss_pose_sub_ = nh_.subscribe("gnss_pose", 5, &LocalizationManager::gnssPoseCallback, manager_.get());
  initialpose_sub_ = nh.subscribe("initialpose", 1, &LocalizationManager::initialPoseCallback, manager_.get());

  // TODO fix comments
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh_, "ndt_pose", 5);
  message_filters::Subscriber<autoware_msgs::NDTStat> stats_sub(nh_, "ndt_stats", 5);

  PoseStatsSynchronizer pose_stats_synchronizer(PoseStatsSyncPolicy(5), pose_sub, stats_sub);

  pose_stats_synchronizer.registerCallback(boost::bind(&Localizer::poseAndStatsCallback, this, _1, _2));

  nh_.setSystemAlertCallback(std::bind(&LocalizationManager::systemAlertCallback, manager_.get(), std_ph::_1));
  nh_.setSpinCallback(std::bind(&LocalizationManager::onSpin, manager_.get(), std_ph::_1));

  // initialize publishers
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("selected_pose", 5);
  state_pub_ = nh_.advertise<cav_msgs::LocalizationStatusReport>("localization_status", 1);

  // spin
  nh_.setSpinRate(spin_rate_);
  nh_.spin();
}
}  // namespace localizer
