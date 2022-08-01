/*
 * Copyright (C) 2018-2022 LEIDOS.
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

#include "guidance/guidance_worker.hpp"
#include "guidance_test_suite.cpp"

TEST(GuidanceNodeTest, TestGuidanceLaunch){
    // Activate the guidance node
    rclcpp::NodeOptions options;
    auto guidance_worker_node = std::make_shared<guidance::GuidanceWorker>(options);
    guidance_worker_node->configure(); //Call configure state transition
    guidance_worker_node->activate();  //Call activate state transition to get not read for runtime;

    // Activate the GuidanceTestSuite node
    rclcpp::NodeOptions options2;
    auto test_suite_node = std::make_shared<guidance_test_suite::GuidanceTestSuite>(options2);
    test_suite_node->configure();
    test_suite_node->activate();

    // Add these nodes to an executor to spin them and trigger their callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(guidance_worker_node->get_node_base_interface());
    executor.add_node(test_suite_node->get_node_base_interface());

    // Spin executor for 500 milliseconds
    auto end_time = std::chrono::system_clock::now() + std::chrono::milliseconds(500);
    while(std::chrono::system_clock::now() < end_time){
        executor.spin_once();
    }

    // Verify that the GuidnaceTestSuite node has received a DRIVERS_READY GuidanceState message since the Guidance node has been activated
    EXPECT_EQ(carma_planning_msgs::msg::GuidanceState::DRIVERS_READY, test_suite_node->latest_state.state);
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