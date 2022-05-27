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

#include <carma_wm_ctrl_ros2/WMBroadcasterNode.hpp>
#include <rclcpp/rclcpp.hpp>

// Main execution
int main(int argc, char** argv)
{
  // Initialize node
  rclcpp::init(argc, argv);
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_mw_ctrl::main"), "ENTERED MAIN!!!");
  
  auto node = std::make_shared<carma_wm_ctrl::WMBroadcasterNode>(rclcpp::NodeOptions());
  
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_mw_ctrl::main"), "we are done constructing!");

  node->initializeWorker(node);
  
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_mw_ctrl::main"), "we are done initializing! counter:" << node.use_count());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
};