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
#include <chrono>

#include <subsystem_controllers/localization_controller/localization_controller.hpp>

TEST(Testsubsystem_controllers, localization_controller)
{

    std::vector<std::string> remaps; // Remaps to keep topics separate from other tests
    remaps.push_back("--ros-args");
    remaps.push_back("-r");
    remaps.push_back("/system_alert:=/localization_controller_test/system_alert");
    remaps.push_back("-r");
    remaps.push_back("__node:=subsystem_controllers_localization_controller_test");

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.arguments(remaps);

    auto worker_node = std::make_shared<subsystem_controllers::LocalizationControllerNode>(options);

    // Set the parameters for this test
    worker_node->set_parameter(rclcpp::Parameter("service_timeout_ms", 200));
    worker_node->set_parameter(rclcpp::Parameter("call_timeout_ms", 1000));
    worker_node->set_parameter(rclcpp::Parameter("subsystem_namespace", "/localization"));
    std::vector<std::string> empty_str_vec;  
    worker_node->set_parameter(rclcpp::Parameter("required_subsystem_nodes", empty_str_vec));
    worker_node->set_parameter(rclcpp::Parameter("unmanaged_required_nodes", empty_str_vec));
    worker_node->set_parameter(rclcpp::Parameter("full_subsystem_required", false));
    std::vector<std::string> sensor_nodes = {
        "/localization_controller/test_sensor_node_1",
        "/localization_controller/test_sensor_node_2"
    };
    worker_node->set_parameter(rclcpp::Parameter("sensor_nodes", sensor_nodes));
    worker_node->set_parameter(rclcpp::Parameter("sensor_fault_map", "{"
        "\"sensor_fault_map\":"
        "["
          "[0,1,3],"
          "[1,0,2],"
          "[0,0,0]"
        "]"
      "}"));


    // There are no managed nodes in this test so we should be able to make it to the activated state
    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    ASSERT_EQ(worker_node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    std::vector<std::string> remaps2; // Remaps to keep topics separate from other tests
    remaps2.push_back("--ros-args");
    remaps2.push_back("-r");
    remaps2.push_back("/system_alert:=/localization_controller_test/system_alert");
    remaps2.push_back("-r");
    remaps2.push_back("__node:=subsystem_controllers_localization_controller_test_sub_node");
    rclcpp::NodeOptions options2;
    options2.use_intra_process_comms(true);
    options2.arguments(remaps);
    auto sub_node = std::make_shared<rclcpp::Node>("sub_node", options2);
    
    carma_msgs::msg::SystemAlert result;
    auto sub = sub_node->create_subscription<carma_msgs::msg::SystemAlert>("/localization_controller_test/system_alert", 1,
        [&](carma_msgs::msg::SystemAlert::UniquePtr msg)
        {
            result = *msg;
        });

    std::unique_ptr<carma_msgs::msg::SystemAlert> alert_msg = std::make_unique<carma_msgs::msg::SystemAlert>();
    alert_msg->type = carma_msgs::msg::SystemAlert::FATAL;
    alert_msg->source_node = "/localization_controller/test_sensor_node_2";

    worker_node->on_system_alert(move(alert_msg)); // Manually drive topic callbacks

    // Provide some time for publication to occur
    std::this_thread::sleep_for(std::chrono::seconds(2));

    rclcpp::spin_some(sub_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger


    ASSERT_EQ(result.type, carma_msgs::msg::SystemAlert::CAUTION);
    ASSERT_EQ(worker_node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    // Cause second node to fail
    std::unique_ptr<carma_msgs::msg::SystemAlert> alert_msg_2 = std::make_unique<carma_msgs::msg::SystemAlert>();
    alert_msg_2->type = carma_msgs::msg::SystemAlert::FATAL;
    alert_msg_2->source_node = "/localization_controller/test_sensor_node_1";

    worker_node->on_system_alert(move(alert_msg_2)); // Manually drive topic callbacks

    // Provide some time for publication to occur
    std::this_thread::sleep_for(std::chrono::seconds(2));

    rclcpp::spin_some(sub_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger


    ASSERT_EQ(result.type, carma_msgs::msg::SystemAlert::FATAL);

    /////////
    // Check the warning case
    /////////
    worker_node = std::make_shared<subsystem_controllers::LocalizationControllerNode>(options);

    // Set the parameters for this test
    worker_node->set_parameter(rclcpp::Parameter("service_timeout_ms", 200));
    worker_node->set_parameter(rclcpp::Parameter("call_timeout_ms", 1000));
    worker_node->set_parameter(rclcpp::Parameter("subsystem_namespace", "/localization"));
    worker_node->set_parameter(rclcpp::Parameter("required_subsystem_nodes", empty_str_vec));
    worker_node->set_parameter(rclcpp::Parameter("unmanaged_required_nodes", empty_str_vec));
    worker_node->set_parameter(rclcpp::Parameter("full_subsystem_required", false));
    worker_node->set_parameter(rclcpp::Parameter("sensor_nodes", sensor_nodes ));
    worker_node->set_parameter(rclcpp::Parameter("sensor_fault_map", "{"
        "\"sensor_fault_map\":" 
        "["
          "[0,1,3],"
          "[1,0,2],"
          "[0,0,0]"
        "]"
      "}"));


    // There are no managed nodes in this test so we should be able to make it to the activated state
    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    ASSERT_EQ(worker_node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    // Cause second node to fail
    std::unique_ptr<carma_msgs::msg::SystemAlert> alert_msg_3 = std::make_unique<carma_msgs::msg::SystemAlert>();
    alert_msg_3->type = carma_msgs::msg::SystemAlert::FATAL;
    alert_msg_3->source_node = "/localization_controller/test_sensor_node_1";

    worker_node->on_system_alert(move(alert_msg_3)); // Manually drive topic callbacks

    // Provide some time for publication to occur
    std::this_thread::sleep_for(std::chrono::seconds(2));

    rclcpp::spin_some(sub_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger
    

    ASSERT_EQ(result.type, carma_msgs::msg::SystemAlert::WARNING);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
}
