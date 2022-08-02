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
    TEST(LocalizationManager, testConstructor)
    {
        // Using default values from LocalizationManagerConfig
        LocalizationManagerConfig config;

        auto worker_node = rclcpp_lifecycle::LifecycleNode::make_shared("worker_node");
        worker_node->configure();
        worker_node->activate();

        auto timer = std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>();
        boost::optional<carma_localization_msgs::msg::LocalizationStatusReport> status_msg;
        LocalizationManager manager([](auto pose) {}, [&](auto status) {status_msg = status;}, [](auto pose_with_cov) {}, config,
                                        worker_node->get_node_logging_interface(),
                                        timer);

    }

    TEST(LocalizationManager, testSpin)
    {
        // Using default values from LocalizationManagerConfig
        LocalizationManagerConfig config;

        auto worker_node = rclcpp_lifecycle::LifecycleNode::make_shared("worker_node");
        worker_node->configure();
        worker_node->activate();

        auto timer = std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>();
        // Initialize clock in timer 
        timer->buildTimer(0, rclcpp::Duration(0,0), [](){}, true, true);

        boost::optional<carma_localization_msgs::msg::LocalizationStatusReport> status_msg;
        LocalizationManager manager([](auto pose) {}, [&](auto status) {status_msg = status;}, [](auto pose_with_cov) {}, config,
                                        worker_node->get_node_logging_interface(),
                                        timer);

        ASSERT_FALSE(!!status_msg);

        timer->setNow(rclcpp::Time(1.0e9));
        manager.posePubTick();
        ASSERT_TRUE(!!status_msg);

        ASSERT_EQ(carma_localization_msgs::msg::LocalizationStatusReport::UNINITIALIZED, status_msg.get().status);
        ASSERT_EQ(timer->now().seconds(), status_msg.get().header.stamp.sec);
    }

    TEST(LocalizationManager, testSomething)
    {
        LocalizationManagerConfig config;
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<carma_ros2_utils::CarmaLifecycleNode>(options);
        worker_node->configure();
        worker_node->activate();

        auto timer = std::make_shared<carma_ros2_utils::timers::ROSTimerFactory>(std::weak_ptr(worker_node));

        boost::optional<carma_localization_msgs::msg::LocalizationStatusReport> status_msg;
        LocalizationManager manager([](auto pose) {}, [&](auto status) {status_msg = status;}, [](auto pose_with_cov) {}, config,
                                        worker_node->get_node_logging_interface(),
                                        timer);

        ASSERT_FALSE(!!status_msg); 

        geometry_msgs::msg::PoseWithCovarianceStamped msg; 
        msg.header.stamp = timer->now();
        geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_ptr(new geometry_msgs::msg::PoseWithCovarianceStamped(msg));
        manager.initialPoseCallback(msg_ptr);

        geometry_msgs::msg::PoseStamped msg_2;
        msg_2.header.stamp = rclcpp::Time(timer->now());
        geometry_msgs::msg::PoseStamped::SharedPtr msg_ptr_2(new geometry_msgs::msg::PoseStamped(msg_2));
        manager.gnssPoseCallback(msg_ptr_2);
        ASSERT_EQ(carma_localization_msgs::msg::LocalizationStatusReport::DEGRADED_NO_LIDAR_FIX, status_msg->status);  
    }

    // TEST(LocalizationManager, testGNSSTimeout)
    // {
    //      // Using default values from LocalizationManagerConfig
    //     LocalizationManagerConfig config;

    //     auto worker_node = rclcpp_lifecycle::LifecycleNode::make_shared("worker_node");
    //     worker_node->configure();
    //     worker_node->activate();

    //     auto timer = std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>();
    //     // Initialize clock in timer 
    //     timer->buildTimer(0, rclcpp::Duration(0,0), [](){}, true, true);
    //     timer->setNow(rclcpp::Time(1e9, worker_node->now().get_clock_type()));

    //     boost::optional<carma_localization_msgs::msg::LocalizationStatusReport> status_msg;
    //     LocalizationManager manager([](auto pose) {}, [&](auto status) {status_msg = status;}, [](auto pose_with_cov) {}, config,
    //                                     worker_node->get_node_logging_interface(),
    //                                     timer);

    //     ASSERT_FALSE(!!status_msg);   
        
        
        
    //     geometry_msgs::msg::PoseWithCovarianceStamped msg;
    //     // msg.header.seq = 1;
    //     msg.header.stamp = timer->now();
    //     geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_ptr(new geometry_msgs::msg::PoseWithCovarianceStamped(msg));
    //     manager.initialPoseCallback(msg_ptr);

    //     geometry_msgs::msg::PoseStamped msg_2;
    //     // msg_2.header.seq = 1;
    //     msg_2.header.stamp = rclcpp::Time(timer->now(), RCL_SYSTEM_TIME);
    //     geometry_msgs::msg::PoseStamped::SharedPtr msg_ptr_2(new geometry_msgs::msg::PoseStamped(msg_2));
    //     manager.gnssPoseCallback(msg_ptr_2);    
    //     std::cout<<"Msg 2 stamp type: "<<rclcpp::Time(msg_2.header.stamp).get_clock_type()<<std::endl;
    //     manager.posePubTick();
    //     ASSERT_TRUE(!!status_msg);
    //     ASSERT_EQ(carma_localization_msgs::msg::LocalizationStatusReport::DEGRADED_NO_LIDAR_FIX, status_msg->status);     
        // ASSERT_EQ(ros::Time::now().toSec(), status_msg.get().header.stamp.toSec());

        // ros::Time::setNow(ros::Time(1.1)); // Still no timeout

        // manager.posePubTick(e);
        // ASSERT_TRUE(!!status_msg);
        // ASSERT_EQ(cav_msgs::LocalizationStatusReport::DEGRADED_NO_LIDAR_FIX, status_msg.get().status);
        // ASSERT_EQ(ros::Time::now().toSec(), status_msg.get().header.stamp.toSec());

        // ros::Time::setNow(ros::Time(1.6)); // timout

        // ASSERT_THROW(manager.posePubTick(e), std::runtime_error);
                          
    // }
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
} 