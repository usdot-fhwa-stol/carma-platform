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
#include <carma_localization_msgs/msg/localization_status_report.hpp>
#include <carma_utils/timers/testing/TestTimer.h>
#include <carma_utils/timers/testing/TestTimerFactory.h>
#include <carma_utils/testing/TestHelpers.h>
#include "localization_manager/LocalizationManager.h"


namespace localization_manager 
{

// Helper function returns the config that these tests were origionally meant to assume was the default
LocalizationManagerConfig getDefaultConfigForTest() {
  LocalizationManagerConfig config;
  config.fitness_score_degraded_threshold = 1.0;
  config.fitness_score_fault_threshold = 2.0;
  config.ndt_frequency_degraded_threshold = 9;
  config.ndt_frequency_fault_threshold = 5;
  config.auto_initialization_timeout = 30000;
  config.gnss_only_operation_timeout = 6000;
  config.gnss_data_timeout = 500;
  config.localization_mode = LocalizerMode::NDT;

  return config;
}

TEST(LocalizationManager, testConstructor)
{
  LocalizationManagerConfig config = getDefaultConfigForTest();
  LocalizationManager manager([](auto pose) {}, [](auto status) {}, [](auto pose_with_cov) {}, config,
                              std::make_unique<carma_utils::timers::testing::TestTimerFactory>());
}

TEST(LocalizationManager, testSpin)
{
  LocalizationManagerConfig config = getDefaultConfigForTest();

  boost::optional<carma_localization_msgs::msg::LocalizationStatusReport> status_msg;
  LocalizationManager manager([](auto pose) {}, [&](auto status) { status_msg = status; }, [](auto pose_with_cov) {}, config,
                              std::make_unique<carma_utils::timers::testing::TestTimerFactory>());

  ASSERT_FALSE(!!status_msg);

  ros::Time::setNow(ros::Time(1.0));

  ros::TimerEvent e;
  manager.posePubTick(e);
  ASSERT_TRUE(!!status_msg);

  ASSERT_EQ(carma_localization_msgs::msg::LocalizationStatusReport::UNINITIALIZED, status_msg.get().status);
  ASSERT_EQ(ros::Time::now().toSec(), status_msg.get().header.stamp.toSec());
}

TEST(LocalizationManager, testGNSSTimeout)
{
  LocalizationManagerConfig config = getDefaultConfigForTest();
  config.localization_mode = LocalizerMode::GNSS;

  boost::optional<carma_localization_msgs::msg::LocalizationStatusReport> status_msg;
  LocalizationManager manager([](auto pose) {}, [&](auto status) { status_msg = status; }, [](auto pose_with_cov) {}, config,
                              std::make_unique<carma_utils::timers::testing::TestTimerFactory>());

  ASSERT_FALSE(!!status_msg);

  ros::Time::setNow(ros::Time(1.0));

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.seq = 1;
  msg.header.stamp = ros::Time::now();
  geometry_msgs::msg::PoseWithCovarianceStampedConstPtr msg_ptr(new geometry_msgs::msg::PoseWithCovarianceStamped(msg));
  manager.initialPoseCallback(msg_ptr);

  geometry_msgs::msg::PoseStamped msg_2;
  msg_2.header.seq = 1;
  msg_2.header.stamp = ros::Time::now();
  geometry_msgs::msg::PoseStampedConstPtr msg_ptr_2(new geometry_msgs::msg::PoseStamped(msg_2));
  manager.gnssPoseCallback(msg_ptr_2);


  ros::TimerEvent e;
  manager.posePubTick(e);
  ASSERT_TRUE(!!status_msg);

  ASSERT_EQ(carma_localization_msgs::msg::LocalizationStatusReport::DEGRADED_NO_LIDAR_FIX, status_msg.get().status);
  ASSERT_EQ(ros::Time::now().toSec(), status_msg.get().header.stamp.toSec());

  ros::Time::setNow(ros::Time(1.1)); // Still no timeout
  
  manager.posePubTick(e);
  ASSERT_TRUE(!!status_msg);

  ASSERT_EQ(carma_localization_msgs::msg::LocalizationStatusReport::DEGRADED_NO_LIDAR_FIX, status_msg.get().status);
  ASSERT_EQ(ros::Time::now().toSec(), status_msg.get().header.stamp.toSec());

  ros::Time::setNow(ros::Time(1.6)); // timout

  ASSERT_THROW(manager.posePubTick(e), std::runtime_error);
}

TEST(LocalizationManager, testSignals)
{
  LocalizationManagerConfig config = getDefaultConfigForTest();
  config.localization_mode = LocalizerMode::AUTO_WITH_TIMEOUT;
  config.auto_initialization_timeout = 1000;
  config.gnss_only_operation_timeout = 2000;

  boost::optional<geometry_msgs::msg::PoseStamped> published_pose;
  boost::optional<geometry_msgs::msg::PoseWithCovarianceStamped> published_initial_pose;

  LocalizationManager manager([&](auto pose) { published_pose = pose; },
                              [](auto status) {}, [&](auto initial) { published_initial_pose = initial; }, config,
                              std::make_unique<carma_utils::timers::testing::TestTimerFactory>());

  ros::Time::setNow(ros::Time(1.0));

  ASSERT_EQ(LocalizationState::UNINITIALIZED, manager.getState());

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.seq = 1;
  geometry_msgs::msg::PoseWithCovarianceStampedConstPtr msg_ptr(new geometry_msgs::msg::PoseWithCovarianceStamped(msg));
  manager.initialPoseCallback(msg_ptr);

  ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());
  ASSERT_TRUE(!!published_initial_pose);
  ASSERT_EQ(msg.header.seq, published_initial_pose->header.seq);

  ros::Time::setNow(ros::Time(2.1));

  auto period = std::chrono::milliseconds(2000);
  std::this_thread::sleep_for(period);

  ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, manager.getState());

  ros::Time::setNow(ros::Time(3.1));

  manager.initialPoseCallback(msg_ptr);

  ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  geometry_msgs::msg::PoseStampedConstPtr pose_msg_ptr(new geometry_msgs::msg::PoseStamped(pose_msg));

  autoware_msgs::msg::NDTStat stat_msg;
  stat_msg.score = 0.1;
  autoware_msgs::msg::NDTStatConstPtr stat_msg_ptr(new autoware_msgs::msg::NDTStat(stat_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());
  ros::TimerEvent e;
  manager.posePubTick(e);
  ASSERT_TRUE(!!published_pose);

  ros::Time::setNow(ros::Time(3.2));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::msg::PoseStampedConstPtr(new geometry_msgs::msg::PoseStamped(pose_msg));
  stat_msg.score = 1.1;
  stat_msg_ptr = autoware_msgs::msg::NDTStatConstPtr(new autoware_msgs::msg::NDTStat(stat_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::DEGRADED, manager.getState());

  ros::Time::setNow(ros::Time(3.3));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::msg::PoseStampedConstPtr(new geometry_msgs::msg::PoseStamped(pose_msg));
  stat_msg.score = 0.1;
  stat_msg_ptr = autoware_msgs::msg::NDTStatConstPtr(new autoware_msgs::msg::NDTStat(stat_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

  ros::Time::setNow(ros::Time(3.415));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::msg::PoseStampedConstPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::DEGRADED, manager.getState());

  ros::Time::setNow(ros::Time(3.5));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::msg::PoseStampedConstPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

  ros::Time::setNow(ros::Time(3.7));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::msg::PoseStampedConstPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, manager.getState());

  published_pose = boost::none;
  published_initial_pose = boost::none;

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_FALSE(!!published_pose);

  manager.gnssPoseCallback(pose_msg_ptr);
  manager.posePubTick(e);
  ASSERT_TRUE(!!published_pose);

  ros::Time::setNow(ros::Time(3.7));

  manager.initialPoseCallback(msg_ptr);

  ASSERT_TRUE(!!published_initial_pose);

  ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

  ros::Time::setNow(ros::Time(3.8));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::msg::PoseStampedConstPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, manager.getState());
  manager.gnssPoseCallback(pose_msg_ptr);

  ros::Time::setNow(ros::Time(5.9));

  std::this_thread::sleep_for(period);

  ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, manager.getState());

  ros::Time::setNow(ros::Time(6.0));

  manager.initialPoseCallback(msg_ptr);

  ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::msg::PoseStampedConstPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ros::Time::setNow(ros::Time(6.1));

  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::msg::PoseStampedConstPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

  ros::Time::setNow(ros::Time(6.215));

  manager.posePubTick(e);

  ASSERT_EQ(LocalizationState::DEGRADED, manager.getState());

  manager.initialPoseCallback(msg_ptr);

  ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

  ros::Time::setNow(ros::Time(6.3));
  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::msg::PoseStampedConstPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ros::Time::setNow(ros::Time(6.4));

  pose_msg.header.stamp = ros::Time::now();
  pose_msg_ptr = geometry_msgs::msg::PoseStampedConstPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

  manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

  ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

  ros::Time::setNow(ros::Time(6.6));
  
  manager.gnssPoseCallback(pose_msg_ptr);
  manager.posePubTick(e);

  ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, manager.getState());
  
}

TEST(LocalizationManager, testGNSSCorrection)
{
  LocalizationManagerConfig config = getDefaultConfigForTest();
  config.localization_mode = LocalizerMode::AUTO_WITHOUT_TIMEOUT;
  config.auto_initialization_timeout = 1000;
  config.gnss_only_operation_timeout = 2000;

  boost::optional<geometry_msgs::msg::PoseStamped> published_pose;
  boost::optional<geometry_msgs::msg::PoseWithCovarianceStamped> published_initial_pose;

  LocalizationManager manager([&](auto pose) { published_pose = pose; },
                              [](auto status) {}, [&](auto initial) { published_initial_pose = initial; }, config,
                              std::make_unique<carma_utils::timers::testing::TestTimerFactory>());

  ros::Time::setNow(ros::Time(1.0));

  ASSERT_EQ(LocalizationState::UNINITIALIZED, manager.getState());

  geometry_msgs::msg::PoseStamped msg;
  msg.header.seq = 1;
  geometry_msgs::msg::PoseStampedConstPtr msg_ptr(new geometry_msgs::msg::PoseStamped(msg));
  manager.gnssPoseCallback(msg_ptr);

  ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());
  ASSERT_TRUE(!!published_initial_pose);
  ASSERT_EQ(msg.header.seq, published_initial_pose->header.seq);

}

}; // localizer