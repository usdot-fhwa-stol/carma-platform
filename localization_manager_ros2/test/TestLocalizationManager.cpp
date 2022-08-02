/*
 * Copyright (C) 2022 LEIDOS.
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
#include <rclcpp/rclcpp.hpp>
#include <boost/optional.hpp>
#include <carma_localization_msgs/msg/localization_status_report.hpp>
#include "localization_manager/LocalizationManager.hpp"

#include <carma_ros2_utils/testing/TestHelpers.hpp>
#include <carma_ros2_utils/timers/testing/TestTimer.hpp>
#include <carma_ros2_utils/timers/testing/TestTimerFactory.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using carma_ros2_utils::timers::testing::TestTimer;
using carma_ros2_utils::timers::testing::TestTimerFactory;

namespace localization_manager
{
    TEST(LocalizationManager, DISABLED_testConstructor)
    {
        // Using default values from LocalizationManagerConfig
        LocalizationManagerConfig config;

        auto worker_node = rclcpp_lifecycle::LifecycleNode::make_shared("worker_node");
        worker_node->configure();
        worker_node->activate();

        auto timer = std::make_unique<carma_ros2_utils::timers::testing::TestTimerFactory>();
        timer->buildTimer(0, rclcpp::Duration(0,0), [](){}, true, true);
        boost::optional<carma_localization_msgs::msg::LocalizationStatusReport> status_msg;
        LocalizationManager manager([](auto pose) {}, [&](auto status) {status_msg = status;}, [](auto pose_with_cov) {}, config,
                                        worker_node->get_node_logging_interface(),
                                        std::move(timer));

    }

    TEST(LocalizationManager, DISABLED_testSpin)
    {
        // Using default values from LocalizationManagerConfig
        LocalizationManagerConfig config;

        auto worker_node = rclcpp_lifecycle::LifecycleNode::make_shared("worker_node");
        worker_node->configure();
        worker_node->activate();

        auto timer = std::make_unique<carma_ros2_utils::timers::testing::TestTimerFactory>();
        // Initialize clock in timer 
        timer->buildTimer(0, rclcpp::Duration(0,0), [](){}, true, true);
        timer->setNow(rclcpp::Time(1.0e9));
        rclcpp::Time time_now = timer->now();
        boost::optional<carma_localization_msgs::msg::LocalizationStatusReport> status_msg;
        LocalizationManager manager([](auto pose) {}, [&](auto status) {status_msg = status;}, [](auto pose_with_cov) {}, config,
                                        worker_node->get_node_logging_interface(),
                                        std::move(timer));

        ASSERT_FALSE(!!status_msg);
        
        manager.posePubTick();
        ASSERT_TRUE(!!status_msg);

        ASSERT_EQ(carma_localization_msgs::msg::LocalizationStatusReport::UNINITIALIZED, status_msg.get().status);
        ASSERT_EQ(time_now.seconds(), status_msg.get().header.stamp.sec);
    }

    TEST(LocalizationManager, DISABLED_testGNSSTimeout)
    {
         // Using default values from LocalizationManagerConfig
        LocalizationManagerConfig config;
        config.localization_mode = LocalizerMode::GNSS;

        auto worker_node = rclcpp_lifecycle::LifecycleNode::make_shared("worker_node");
        worker_node->configure();
        worker_node->activate();

        carma_ros2_utils::timers::testing::TestTimerFactory timer;
        // Initialize clock in timer 
        timer.buildTimer(0, rclcpp::Duration(0,0), [](){}, true, true);
        
        boost::optional<carma_localization_msgs::msg::LocalizationStatusReport> status_msg;
        LocalizationManager manager([](auto pose) {}, [&](auto status) {status_msg = status;}, [](auto pose_with_cov) {}, config,
                                        worker_node->get_node_logging_interface(),
                                        std::make_unique<carma_ros2_utils::timers::testing::TestTimerFactory>(timer));

        ASSERT_FALSE(!!status_msg);   
        
        timer.setNow(rclcpp::Time(1e9, worker_node->now().get_clock_type()));

        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = timer.now();
        geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_ptr(new geometry_msgs::msg::PoseWithCovarianceStamped(msg));
        manager.initialPoseCallback(msg_ptr);

        geometry_msgs::msg::PoseStamped msg_2;
        msg_2.header.stamp = timer.now();
        geometry_msgs::msg::PoseStamped::SharedPtr msg_ptr_2(new geometry_msgs::msg::PoseStamped(msg_2));
        manager.gnssPoseCallback(msg_ptr_2);    
        
        manager.posePubTick();
        ASSERT_TRUE(!!status_msg);
        
        ASSERT_EQ(carma_localization_msgs::msg::LocalizationStatusReport::DEGRADED_NO_LIDAR_FIX, status_msg->status);     
        ASSERT_EQ(timer.now().seconds(), rclcpp::Time(status_msg.get().header.stamp).seconds());

        
        timer.setNow(rclcpp::Time(1, 1e8, worker_node->now().get_clock_type()));  // Still no timeout
        manager.posePubTick();
        ASSERT_TRUE(!!status_msg);
        
        ASSERT_EQ(carma_localization_msgs::msg::LocalizationStatusReport::DEGRADED_NO_LIDAR_FIX, status_msg->status);
        ASSERT_EQ(timer.now().seconds(), rclcpp::Time(status_msg.get().header.stamp).seconds());

        timer.setNow(rclcpp::Time(1, 6e8, worker_node->now().get_clock_type()));
        ASSERT_THROW(manager.posePubTick(), std::runtime_error);
                          
    }

    TEST(LocalizationManager, testSignals)
    {
        LocalizationManagerConfig config;
        config.localization_mode = LocalizerMode::AUTO_WITH_TIMEOUT;
        config.auto_initialization_timeout = 1000;
        config.gnss_only_operation_timeout = 2000;  
        config.fitness_score_degraded_threshold = 1.0;

        config.fitness_score_fault_threshold = 2.0;
        config.ndt_frequency_degraded_threshold = 9;
        config.ndt_frequency_fault_threshold = 5;
        config.gnss_data_timeout = 500;

        boost::optional<geometry_msgs::msg::PoseStamped> published_pose;
        boost::optional<geometry_msgs::msg::PoseWithCovarianceStamped> published_initial_pose;

        auto worker_node = rclcpp_lifecycle::LifecycleNode::make_shared("worker_node");
        worker_node->configure();
        worker_node->activate();

        carma_ros2_utils::timers::testing::TestTimerFactory timer;
        // Initialize clock in timer 
        timer.buildTimer(0, rclcpp::Duration(0,0), [](){}, true, true);
        
        LocalizationManager manager([&](auto pose) {published_pose = pose;}, [&](auto status) {}, [&](auto initial) { published_initial_pose = initial; }, config,
                                        worker_node->get_node_logging_interface(),
                                        std::make_unique<carma_ros2_utils::timers::testing::TestTimerFactory>(timer));


        timer.setNow(rclcpp::Time(1e9, worker_node->now().get_clock_type()));
        
        ASSERT_EQ(LocalizationState::UNINITIALIZED, manager.getState());

        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_ptr(new geometry_msgs::msg::PoseWithCovarianceStamped(msg));
        manager.initialPoseCallback(msg_ptr);

        ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());
        ASSERT_TRUE(!!published_initial_pose);
        
        timer.setNow(rclcpp::Time(2, 1e8, RCL_SYSTEM_TIME));

        auto period = std::chrono::milliseconds(2000);
        std::this_thread::sleep_for(period);

        ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, manager.getState());

        timer.setNow(rclcpp::Time(3, 1e8, RCL_SYSTEM_TIME));
        manager.initialPoseCallback(msg_ptr);

        ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = timer.now();
        geometry_msgs::msg::PoseStamped::SharedPtr pose_msg_ptr(new geometry_msgs::msg::PoseStamped(pose_msg));

        autoware_msgs::msg::NDTStat stat_msg;
        stat_msg.score = 0.1;
        autoware_msgs::msg::NDTStat::SharedPtr stat_msg_ptr(new autoware_msgs::msg::NDTStat(stat_msg));

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());
        manager.posePubTick();
        ASSERT_TRUE(!!published_pose);

        timer.setNow(rclcpp::Time(3, 2e8, RCL_SYSTEM_TIME));
        pose_msg.header.stamp = timer.now();
        pose_msg_ptr = geometry_msgs::msg::PoseStamped::SharedPtr (new geometry_msgs::msg::PoseStamped(pose_msg));
        stat_msg.score = 1.1;
        stat_msg_ptr = autoware_msgs::msg::NDTStat::SharedPtr (new autoware_msgs::msg::NDTStat(stat_msg));

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        ASSERT_EQ(LocalizationState::DEGRADED, manager.getState());

        timer.setNow(rclcpp::Time(3,3e8, RCL_SYSTEM_TIME));
        pose_msg.header.stamp = timer.now();
        pose_msg_ptr = geometry_msgs::msg::PoseStamped::SharedPtr(new geometry_msgs::msg::PoseStamped(pose_msg));
        stat_msg.score = 0.1;
        stat_msg_ptr = autoware_msgs::msg::NDTStat::SharedPtr(new autoware_msgs::msg::NDTStat(stat_msg));   

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

        timer.setNow(rclcpp::Time(3,415e6, RCL_SYSTEM_TIME));
        pose_msg.header.stamp = timer.now();
        pose_msg_ptr = geometry_msgs::msg::PoseStamped::SharedPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        ASSERT_EQ(LocalizationState::DEGRADED, manager.getState());

        timer.setNow(rclcpp::Time(3,5e8, RCL_SYSTEM_TIME));
        pose_msg.header.stamp = timer.now();
        pose_msg_ptr = geometry_msgs::msg::PoseStamped::SharedPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

        timer.setNow(rclcpp::Time(3,7e8, RCL_SYSTEM_TIME));
        pose_msg.header.stamp = timer.now();
        pose_msg_ptr = geometry_msgs::msg::PoseStamped::SharedPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, manager.getState());

        published_pose = boost::none;
        published_initial_pose = boost::none;

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        ASSERT_FALSE(!!published_pose);

        manager.gnssPoseCallback(pose_msg_ptr);
        manager.posePubTick();
        ASSERT_TRUE(!!published_pose);

        timer.setNow(rclcpp::Time(3,7e8, RCL_SYSTEM_TIME));

        manager.initialPoseCallback(msg_ptr);

        ASSERT_TRUE(!!published_initial_pose);
        ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

        timer.setNow(rclcpp::Time(3,8e8, RCL_SYSTEM_TIME));
        pose_msg.header.stamp = timer.now();
        pose_msg_ptr = geometry_msgs::msg::PoseStamped::SharedPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, manager.getState());
        manager.gnssPoseCallback(pose_msg_ptr);

        timer.setNow(rclcpp::Time(5,9e8, RCL_SYSTEM_TIME));

        std::this_thread::sleep_for(period);

        ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, manager.getState());

        timer.setNow(rclcpp::Time(6,0, RCL_SYSTEM_TIME));

        manager.initialPoseCallback(msg_ptr);

        ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

        pose_msg.header.stamp = timer.now();
        pose_msg_ptr = geometry_msgs::msg::PoseStamped::SharedPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        timer.setNow(rclcpp::Time(6, 1e8, RCL_SYSTEM_TIME));

        pose_msg.header.stamp = timer.now();
        pose_msg_ptr = geometry_msgs::msg::PoseStamped::SharedPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

        timer.setNow(rclcpp::Time(6,215e6, RCL_SYSTEM_TIME));

        manager.posePubTick();
        ASSERT_EQ(LocalizationState::DEGRADED, manager.getState());

        manager.initialPoseCallback(msg_ptr);
        ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());

        timer.setNow(rclcpp::Time(6,3e8));
        pose_msg.header.stamp = timer.now();
        pose_msg_ptr = geometry_msgs::msg::PoseStamped::SharedPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        timer.setNow(rclcpp::Time(6,4e8, RCL_SYSTEM_TIME));

        pose_msg.header.stamp = timer.now();
        pose_msg_ptr = geometry_msgs::msg::PoseStamped::SharedPtr(new geometry_msgs::msg::PoseStamped(pose_msg));

        manager.poseAndStatsCallback(pose_msg_ptr, stat_msg_ptr);

        ASSERT_EQ(LocalizationState::OPERATIONAL, manager.getState());

        timer.setNow(rclcpp::Time(6,6e8, RCL_SYSTEM_TIME));
  
        manager.gnssPoseCallback(pose_msg_ptr);
        manager.posePubTick();

        ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, manager.getState());

    }

    TEST(LocalizationManager, DISABLED_testGNSSCorrection)
    {
        LocalizationManagerConfig config;
        config.localization_mode = static_cast<int>(LocalizerMode::AUTO_WITHOUT_TIMEOUT);
        config.auto_initialization_timeout = 1000;
        config.gnss_only_operation_timeout = 2000;

        boost::optional<geometry_msgs::msg::PoseStamped> published_pose;
        boost::optional<geometry_msgs::msg::PoseWithCovarianceStamped> published_initial_pose;

        auto worker_node = rclcpp_lifecycle::LifecycleNode::make_shared("worker_node");
            worker_node->configure();
            worker_node->activate();

        carma_ros2_utils::timers::testing::TestTimerFactory timer;
        // Initialize clock in timer 
        timer.buildTimer(0, rclcpp::Duration(0,0), [](){}, true, true);
        LocalizationManager manager([&](auto pose) {published_pose = pose;}, [&](auto status) {}, [&](auto initial) { published_initial_pose = initial; }, config,
                                        worker_node->get_node_logging_interface(),
                                        std::make_unique<carma_ros2_utils::timers::testing::TestTimerFactory>(timer));

        timer.setNow(rclcpp::Time(1.0e9, RCL_SYSTEM_TIME));

        ASSERT_EQ(LocalizationState::UNINITIALIZED, manager.getState());

        geometry_msgs::msg::PoseStamped msg;
        geometry_msgs::msg::PoseStamped::SharedPtr msg_ptr(new geometry_msgs::msg::PoseStamped(msg));
        manager.gnssPoseCallback(msg_ptr);

        ASSERT_EQ(LocalizationState::INITIALIZING, manager.getState());
        ASSERT_TRUE(!!published_initial_pose);

    }
}
