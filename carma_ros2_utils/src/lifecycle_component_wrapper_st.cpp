// Copyright 2019 Open Source Robotics Foundation, Inc.
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

/**
 * Modifications copyright (C) 2021 Leidos
 * - Converted to use Lifecycle Component Wrapper
 * 
 */ 

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "carma_ros2_utils/lifecycle_component_wrapper.hpp"

int main(int argc, char * argv[])
{
  /// Component container with a single-threaded executor.
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<carma_ros2_utils::LifecycleComponentWrapper>();
  node->initialize(exec);
  exec->add_node(node->get_node_base_interface());
  exec->spin();
}