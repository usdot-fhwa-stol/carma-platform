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

#include "sci_strategic_plugin.hpp"
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

namespace sci_strategic_plugin
{
namespace std_ph = std::placeholders;

class DummyNode :  public carma_ros2_utils::CarmaLifecycleNode
{
public:

    explicit DummyNode(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
    {}

    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &){return CallbackReturn::SUCCESS;}
};


TEST(SCIStrategicPlugin, UnitTest1)
{
    auto nh1 = std::make_shared<DummyNode>(rclcpp::NodeOptions());
    auto nh2 = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
    
    nh2->configure();
    nh2->activate();

    auto mob_op_pub = nh1->create_publisher<carma_v2x_msgs::msg::MobilityOperation>("incoming_mobility_operation", 5);
    auto pose_pub = nh1->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 5);
    nh1->configure();
    nh2->activate();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh1->get_node_base_interface());
    executor.add_node(nh2->get_node_base_interface());

    // Spin executor for 2 seconds
    auto end_time = std::chrono::system_clock::now() + std::chrono::seconds(3);
    while(std::chrono::system_clock::now() < end_time){
        executor.spin_once();
    }
    
    EXPECT_EQ(1, mob_op_pub->get_subscription_count());
    EXPECT_EQ(1, pose_pub->get_subscription_count());
}

} // namespace sci_strategic_plugin

