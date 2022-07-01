/*
 * Copyright (C) 2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "arbitrator_node.hpp"
#include <rclcpp/rclcpp.hpp>


int main(int argc, char** argv) 
{
    // Initialize node
    rclcpp::init(argc, argv);

    // Call current CARMA arbitrator node instance called ArbitratorNode with specific planning paradigm.
    auto node = std::make_shared<arbitrator::ArbitratorNode>(rclcpp::NodeOptions());
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
