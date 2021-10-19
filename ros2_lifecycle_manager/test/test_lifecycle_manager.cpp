/*
 * Copyright (C) 2021 LEIDOS.
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
#include <memory>
#include <chrono>
#include <thread>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_lifecycle_manager/ros2_lifecycle_manager.hpp"
#include "boost/core/ignore_unused.hpp"


using std_msec = std::chrono::milliseconds;

/**
 * This test is meant to exercise the standard flow of the lifecycle manager. 
 * To allow for callbacks to be processed the test makes use of a rather roundabout thread paradigm 
 * Care should be taken to understand this code before trying to duplicate it in other test scenarios.
 * There may be better approaches such as the launch_testing framework.
 */ 
TEST(LifecycleManagerTest, BasicTest)
{
  // Test constructor
  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_test_node"); // Node to connect to ROS network

  auto ret = rcutils_logging_set_logger_level(
        node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  boost::ignore_unused(ret);

  std::promise<bool> promise_test_result; // Promise which will be used to spin the node until the test completes
  std::shared_future<bool> future_test_result(promise_test_result.get_future()); // Future to check for spin

  std::thread test_thread([&promise_test_result, node](){
    
    ros2_lifecycle_manager::Ros2LifecycleManager lifecycle_mgr_( // Construct lifecycle manager
      node->get_node_base_interface(), 
      node->get_node_graph_interface(), 
      node->get_node_logging_interface(), 
      node->get_node_services_interface()
    );

    // Test set_managed_nodes
    lifecycle_mgr_.set_managed_nodes( { "test_lifecycle_node_1", "test_lifecycle_node_2" } );
    
    std::this_thread::sleep_for(std_msec(2000));

    // Test get_managed_nodes
    auto managed_nodes = lifecycle_mgr_.get_managed_nodes();
    
    ASSERT_EQ((uint8_t)2, managed_nodes.size());
    ASSERT_EQ((uint8_t)0, managed_nodes[0].compare("test_lifecycle_node_1"));
    ASSERT_EQ((uint8_t)0, managed_nodes[1].compare("test_lifecycle_node_2"));
    
    // Test get managed node states
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, lifecycle_mgr_.get_managed_node_state("unknown_node"));

    // Test Configure
    ASSERT_TRUE(lifecycle_mgr_.configure(std_msec(2000), std_msec(2000)));
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

    // Test Activate
    ASSERT_TRUE(lifecycle_mgr_.activate(std_msec(2000), std_msec(2000)));
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE , lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE , lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

    // Test Deactivate
    ASSERT_TRUE(lifecycle_mgr_.deactivate(std_msec(2000), std_msec(2000)));
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

    // Test cleanup
    ASSERT_TRUE(lifecycle_mgr_.cleanup(std_msec(2000), std_msec(2000)));
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED , lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED , lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

    // Bring back to active then shutdown
    ASSERT_TRUE(lifecycle_mgr_.configure(std_msec(2000), std_msec(2000)));
    ASSERT_TRUE(lifecycle_mgr_.activate(std_msec(2000), std_msec(2000)));
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE , lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE , lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

    ASSERT_TRUE(lifecycle_mgr_.shutdown(std_msec(2000), std_msec(2000)));
    ASSERT_TRUE(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED  == lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1")
                || lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN  == lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_1"));
    ASSERT_TRUE(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED  == lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2")
                || lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN  == lifecycle_mgr_.get_managed_node_state("test_lifecycle_node_2"));

    promise_test_result.set_value(true);
  });

  RCLCPP_INFO(node->get_logger(), "Spinning");
  rclcpp::spin_until_future_complete(node, future_test_result);

  test_thread.join(); // Ensure thread closes

  
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);


  bool success = RUN_ALL_TESTS();  

  // shutdown ROS
  rclcpp::shutdown();

  return success;
}
