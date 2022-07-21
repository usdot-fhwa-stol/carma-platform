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

#include "carma_guidance_plugins/plugin_base_node.hpp"
#include "TestPlugins.h"

bool has_publisher(std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> node, const std::string& topic_name, const std::string& type)
{
    bool found = false;
    for (const auto& endpoint : node->get_publishers_info_by_topic(topic_name)) {
        
        std::cerr << "name: " << endpoint.node_name() << " type: " << endpoint.topic_type() << std::endl;

        if (endpoint.node_name() == node->get_name() && endpoint.topic_type() == type)
            found = true;
    }

    return found;
}

bool has_subscriber(std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> node, const std::string& topic_name, const std::string& type)
{
    bool found = false;
    for (const auto& endpoint : node->get_subscriptions_info_by_topic(topic_name)) {
        
        std::cerr << "name: " << endpoint.node_name() << " type: " << endpoint.topic_type() << std::endl;

        if (endpoint.node_name() == node->get_name() && endpoint.topic_type() == type)
            found = true;
    }

    return found;
}

bool has_service(std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> node, std::string_view service_name, std::string_view type)
{
    bool found = false;
    for (const auto& [service_k, type_v] : node->get_service_names_and_types_by_node(node->get_name(), "")) {
        
        std::cerr << "service: " << service_k << " type: " << type_v[0] << std::endl;

        if ( service_k == service_name && type_v[0] == type)
            found = true;
    }

    return found;
}

namespace carma_guidance_plugins
{

TEST(carma_guidance_plugins_test, connections_test) {


    std::vector<std::string> str_remaps = {"--ros-args", "-r", "__node:=strategic_plugin_test"};
    std::vector<std::string> tac_remaps = {"--ros-args", "-r", "__node:=tactical_plugin_test"};
    std::vector<std::string> ctrl_remaps = {"--ros-args", "-r", "__node:=control_plugin_test"};

    rclcpp::NodeOptions str_options;
    str_options.arguments(str_remaps);
    str_options.use_intra_process_comms(true);

    rclcpp::NodeOptions tac_options;
    tac_options.arguments(tac_remaps);
    tac_options.use_intra_process_comms(true);

    rclcpp::NodeOptions ctrl_options;
    ctrl_options.arguments(ctrl_remaps);
    ctrl_options.use_intra_process_comms(true);

    auto strategic_plugin = std::make_shared<carma_guidance_plugins::TestStrategicPlugin>(str_options);
    auto tactical_plugin = std::make_shared<carma_guidance_plugins::TestTacticalPlugin>(tac_options);
    auto control_plugin = std::make_shared<carma_guidance_plugins::TestControlPlugin>(ctrl_options);

    ASSERT_TRUE(strategic_plugin->get_availability());
    ASSERT_TRUE(tactical_plugin->get_availability());
    ASSERT_TRUE(control_plugin->get_availability());

    ASSERT_EQ(strategic_plugin->get_plugin_name(), "TestStrategicPlugin");
    ASSERT_EQ(tactical_plugin->get_plugin_name(), "TestTacticalPlugin");
    ASSERT_EQ(control_plugin->get_plugin_name(), "TestControlPlugin");

    ASSERT_EQ(strategic_plugin->get_version_id(), "1.0");
    ASSERT_EQ(tactical_plugin->get_version_id(), "1.1");
    ASSERT_EQ(control_plugin->get_version_id(), "1.2");

    ASSERT_EQ(strategic_plugin->get_capability(), "strategic_plan/plan_maneuvers/test_capability");
    ASSERT_EQ(tactical_plugin->get_capability(), "tactical_plan/plan_trajectory/test_capability");
    ASSERT_EQ(control_plugin->get_capability(), "control/trajectory_control/test_capability");

    // Trigger the discovery callback once to ensure discovery publisher is setup
    strategic_plugin->discovery_timer_callback();
    tactical_plugin->discovery_timer_callback();
    control_plugin->discovery_timer_callback();

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Give a bit for publisher registration to go through

    ASSERT_TRUE(has_publisher(strategic_plugin, "plugin_discovery", "carma_planning_msgs/msg/Plugin")); // Only one true before configure is called
    ASSERT_FALSE(has_service(strategic_plugin, "/TestStrategicPlugin/plan_maneuvers", "carma_planning_msgs/srv/PlanManeuvers"));

    ASSERT_TRUE(has_publisher(tactical_plugin, "plugin_discovery", "carma_planning_msgs/msg/Plugin")); // Only one true before configure is called
    ASSERT_FALSE(has_service(tactical_plugin, "/TestTacticalPlugin/plan_trajectory", "carma_planning_msgs/srv/PlanTrajectory"));

    ASSERT_TRUE(has_publisher(control_plugin, "plugin_discovery", "carma_planning_msgs/msg/Plugin")); // Only one true before configure is called
    ASSERT_FALSE(has_publisher(control_plugin, "ctrl_raw", "autoware_msgs/msg/ControlCommandStamped"));
    ASSERT_FALSE(has_subscriber(control_plugin, "current_pose", "geometry_msgs/msg/PoseStamped"));
    ASSERT_FALSE(has_subscriber(control_plugin, "vehicle/twist", "geometry_msgs/msg/TwistStamped"));
    ASSERT_FALSE(has_subscriber(control_plugin, "TestControlPlugin/plan_trajectory", "carma_planning_msgs/msg/TrajectoryPlan"));

    strategic_plugin->configure();
    tactical_plugin->configure();
    control_plugin->configure();

    ASSERT_TRUE(has_publisher(strategic_plugin, "plugin_discovery", "carma_planning_msgs/msg/Plugin")); // Only one true before configure is called
    ASSERT_TRUE(has_service(strategic_plugin, "/TestStrategicPlugin/plan_maneuvers", "carma_planning_msgs/srv/PlanManeuvers"));

    ASSERT_TRUE(has_publisher(tactical_plugin, "plugin_discovery", "carma_planning_msgs/msg/Plugin")); // Only one true before configure is called
    ASSERT_TRUE(has_service(tactical_plugin, "/TestTacticalPlugin/plan_trajectory", "carma_planning_msgs/srv/PlanTrajectory"));

    ASSERT_TRUE(has_publisher(control_plugin, "plugin_discovery", "carma_planning_msgs/msg/Plugin"));
    ASSERT_TRUE(has_publisher(control_plugin, "ctrl_raw", "autoware_msgs/msg/ControlCommandStamped"));
    ASSERT_TRUE(has_subscriber(control_plugin, "current_pose", "geometry_msgs/msg/PoseStamped"));
    ASSERT_TRUE(has_subscriber(control_plugin, "vehicle/twist", "geometry_msgs/msg/TwistStamped"));
    ASSERT_TRUE(has_subscriber(control_plugin, "TestControlPlugin/plan_trajectory", "carma_planning_msgs/msg/TrajectoryPlan"));

}

} // carma_guidance_plugins

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