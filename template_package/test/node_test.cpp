/*
 * Copyright (C) <SUB><year> LEIDOS.
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

#include "<SUB><package_name>/<SUB><package_name>_node.hpp"


// TODO for USER: Implement a real test using GTest
TEST(Test<SUB><package_name>, example_test){

    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<<SUB><package_name>::Node>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    std::unique_ptr<std_msgs::msg::String> msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "my string";

    worker_node->example_callback(move(msg)); // Manually drive topic callbacks

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