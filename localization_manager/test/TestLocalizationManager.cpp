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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/optional.hpp>
#include <carma_utils/timers/testing/TestTimer.h>
#include <carma_utils/timers/testing/TestTimerFactory.h>
#include <carma_utils/testing/TestHelpers.h>
#include "localization_manager/LocalizationManager.h"

using namespace localizer;

/*

  using PosePublisher = std::function<void(const geometry_msgs::PoseStamped&)>;
  using TransformPublisher = std::function<void(const geometry_msgs::TransformStamped&)>;
  using StatePublisher = std::function<void(const cav_msgs::LocalizationStatusReport&)>;
  */

TEST(LocalizationManager, testConstructor)
{
  LocalizationManagerConfig config;
  LocalizationManager manager([](auto pose) {}, [](auto tf) {}, [](auto status) {}, config,
                              std::make_unique<carma_utils::timers::testing::TestTimerFactory>());
}

TEST(LocalizationManager, testSpin)
{
  LocalizationManagerConfig config;

  boost::optional<cav_msgs::LocalizationStatusReport> status_msg;
  LocalizationManager manager([](auto pose) {}, [](auto tf) {}, [&](auto status) { status_msg = status; }, config,
                              std::make_unique<carma_utils::timers::testing::TestTimerFactory>());

  ASSERT_FALSE(!!status_msg);

  ros::Time::setNow(ros::Time(1.0));

  ASSERT_TRUE(manager.onSpin());
  ASSERT_TRUE(!!status_msg);

  ASSERT_EQ(cav_msgs::LocalizationStatusReport::UNINITIALIZED, status_msg.get().status);
  ASSERT_EQ(ros::Time::now().toSec(), status_msg.get().header.stamp.toSec());
}

TEST(LocalizationManager, testSignals)
{
  LocalizationManagerConfig config;
  config.localization_mode = LocalizerMode::AUTO;
  config.auto_initialization_timeout = 1000;
  config.gnss_only_operation_timeout = 2000;

  boost::optional<geometry_msgs::PoseStamped> published_pose;
  boost::optional<geometry_msgs::TransformStamped> published_transform;

  LocalizationManager manager([&](auto pose) { published_pose = pose; }, [&](auto tf) { published_transform = tf; },
                              [](auto status) {}, config,
                              std::make_unique<carma_utils::timers::testing::TestTimerFactory>());

  ros::Time::setNow(ros::Time(1.0));

  ASSERT_EQ(LocalizationState::UNINITIALIZED, manager.getState());

  geometry_msgs::PoseWithCovarianceStamped msg;
  geometry_msgs::PoseWithCovarianceStampedConstPtr msg_ptr(new geometry_msgs::PoseWithCovarianceStamped(msg));
  manager.initialPoseCallback(msg_ptr);

  ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

  ros::Time::setNow(ros::Time(2.1));

  auto period = std::chrono::milliseconds(2000);
  std::this_thread::sleep_for(period);

  ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, manager.getState());

  ros::Time::setNow(ros::Time(3.1));

  manager.initialPoseCallback(msg_ptr);

  ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  geometry_msgs::PoseStampedConstPtr pose_msg_ptr(new geometry_msgs::PoseStamped(pose_msg));

  autoware_msgs::NDTStat stat_msg;
  stat_msg.score = 0.1;
  autoware_msgs::NDTStatConstPtr stat_msg_ptr(new autoware_msgs::NDTStat(stat_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

  ros::Time::setNow(ros::Time(3.2));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::PoseStampedConstPtr(new geometry_msgs::PoseStamped(pose_msg));
  stat_msg.score = 1.1;
  stat_msg_ptr = autoware_msgs::NDTStatConstPtr(new autoware_msgs::NDTStat(stat_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::DEGRADED, manager.getState());

  ros::Time::setNow(ros::Time(3.3));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::PoseStampedConstPtr(new geometry_msgs::PoseStamped(pose_msg));
  stat_msg.score = 0.1;
  stat_msg_ptr = autoware_msgs::NDTStatConstPtr(new autoware_msgs::NDTStat(stat_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

  ros::Time::setNow(ros::Time(3.415));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::PoseStampedConstPtr(new geometry_msgs::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::DEGRADED, manager.getState());

  ros::Time::setNow(ros::Time(3.5));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::PoseStampedConstPtr(new geometry_msgs::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

  ros::Time::setNow(ros::Time(3.7));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::PoseStampedConstPtr(new geometry_msgs::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, manager.getState());


  ros::Time::setNow(ros::Time(3.7));

  manager.initialPoseCallback(msg_ptr);

  ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

  ros::Time::setNow(ros::Time(3.8));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::PoseStampedConstPtr(new geometry_msgs::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, manager.getState());

  ros::Time::setNow(ros::Time(5.9));

  std::this_thread::sleep_for(period);

  ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, manager.getState());

  ros::Time::setNow(ros::Time(6.0));

  manager.initialPoseCallback(msg_ptr);

  ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::PoseStampedConstPtr(new geometry_msgs::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ros::Time::setNow(ros::Time(6.1));

  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::PoseStampedConstPtr(new geometry_msgs::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

  ros::Time::setNow(ros::Time(6.215));

  manager.onSpin();

  ASSERT_EQ(LocalizationState::DEGRADED, manager.getState());

  manager.initialPoseCallback(msg_ptr);

  ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

  ros::Time::setNow(ros::Time(6.3));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::PoseStampedConstPtr(new geometry_msgs::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ros::Time::setNow(ros::Time(6.4));

  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::PoseStampedConstPtr(new geometry_msgs::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

  ros::Time::setNow(ros::Time(6.6));
  
  manager.onSpin();

  ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, manager.getState());
  
}
