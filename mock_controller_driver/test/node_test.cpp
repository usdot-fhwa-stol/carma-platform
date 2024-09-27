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

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include "mock_controller_driver/mock_controller_driver_node.hpp"


// TODO for USER: Implement a real test using GTest
TEST(Testmock_controller_driver, test_callbacks){

    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<mock_controller_driver::MockControllerDriver>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    bool enabled = false;
    bool active = false;

    std::unique_ptr<autoware_msgs::msg::VehicleCmd> cmd = std::make_unique<autoware_msgs::msg::VehicleCmd>();
    cmd->ctrl_cmd.linear_velocity = 1.0;
    cmd->ctrl_cmd.steering_angle = 0.1;

    worker_node->vehicle_cmd_callback(move(cmd)); // Manually drive topic callbacks
    EXPECT_TRUE(worker_node->robot_enabled_);

    carma_driver_msgs::srv::SetEnableRobotic::Request robot_req;
    carma_driver_msgs::srv::SetEnableRobotic::Response robot_resp;
    robot_req.set = carma_driver_msgs::srv::SetEnableRobotic::Request::ENABLE;

    std::shared_ptr<rmw_request_id_t> header;
    auto robot_req_ptr = std::make_shared<carma_driver_msgs::srv::SetEnableRobotic::Request>(robot_req);
    auto robot_resp_ptr = std::make_shared<carma_driver_msgs::srv::SetEnableRobotic::Response>(robot_resp);

    worker_node->enable_robotic_srv(header, robot_req_ptr, robot_resp_ptr);

    EXPECT_TRUE(worker_node->robot_active_);


}

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
