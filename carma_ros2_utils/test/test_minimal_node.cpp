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

#include "rclcpp/rclcpp.hpp"

namespace carma_ros2_utils_testing {

// No-Op node for testing the lifecycle_component_wrapper
class MinimalNode : public rclcpp::Node
{
    public:
        MinimalNode(const rclcpp::NodeOptions&)
            : Node("minimal_node") {}

};

} // carma_ros2_utils_testing

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(carma_ros2_utils_testing::MinimalNode)
