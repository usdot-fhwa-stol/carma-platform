// Copyright (c) 2020 Shivang Patel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "carma_ros2_utils/carma_lifecycle_node.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

// This is a test node to support unit tests for the carma_lifecycle_node
class CarmaLifecycleNodeTest : public carma_ros2_utils::CarmaLifecycleNode
{
public:
  CarmaLifecycleNodeTest(const rclcpp::NodeOptions &options)
      : CarmaLifecycleNode(options) {}

  ~CarmaLifecycleNodeTest() {};

  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Configured!");

    auto client = create_client<lifecycle_msgs::srv::GetState>("/my_service"); // Test that service clients can be created

    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Activated!");
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn handle_on_deactivate(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Deactivated!");
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn handle_on_cleanup(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Cleanup!");
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn handle_on_shutdown(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Shutdown!");
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn handle_on_error(const rclcpp_lifecycle::State & /*state*/,const std::string &exception_string) override
  {
    RCLCPP_INFO_STREAM(get_logger(), "CARMA Lifecycle Test node is encountered an error! Error: " << exception_string);
    return CallbackReturn::SUCCESS;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<CarmaLifecycleNodeTest>(options);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}