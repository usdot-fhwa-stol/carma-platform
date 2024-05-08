/*
 * Copyright (C) 2024 LEIDOS.
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

#include <rclcpp/rclcpp.hpp>
#include "platoon_control/platoon_control.hpp"

int main(int argc, char **argv)
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("platoon_control"), "Entering main");
  rclcpp::init(argc, argv);
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("platoon_control"), "Reaching before create node");
  auto node = std::make_shared<platoon_control::PlatoonControlPlugin>(rclcpp::NodeOptions());
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("platoon_control"), "Reaching after node create");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
