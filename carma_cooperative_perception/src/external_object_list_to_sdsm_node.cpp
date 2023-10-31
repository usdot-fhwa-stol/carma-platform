// Copyright 2023 Leidos
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

#include <rclcpp/rclcpp.hpp>

#include <carma_cooperative_perception/external_object_list_to_sdsm_component.hpp>

#include <memory>

auto main(int argc, char * argv[]) noexcept -> int
{
  rclcpp::init(argc, argv);

  auto node{std::make_shared<carma_cooperative_perception::ExternalObjectListToSdsmNode>(
    rclcpp::NodeOptions{})};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
