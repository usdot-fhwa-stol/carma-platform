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

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include "light_controlled_intersection_tactical_plugin/light_controlled_intersection_tactical_plugin_node.hpp"

namespace light_controlled_intersection_transit_plugin
{
    
    TEST(LCITacticalPluginTest, apply_accel_cruise_decel_speed_profile_test)
    {
        rclcpp::NodeOptions options;
        auto lci_node = std::make_shared<light_controlled_intersection_tactical_plugin::LightControlledIntersectionTransitPluginNode>(options);
        lci_node->configure(); // Call configure state transition
        lci_node->activate(); // Call activate state transition
    }

} // namespace light_controlled_intersection_transit_plugin


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