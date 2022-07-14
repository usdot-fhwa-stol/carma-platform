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

#include <inlanecruising_plugin/inlanecruising_plugin_node.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include "helper.hpp"
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>


namespace inlanecruising_plugin
{

TEST(InLaneCruisingPluginTest, rostest1)
{
    auto yield_node = std::make_shared<Node>(rclcpp::NodeOptions());
     
    auto service = yield_node->create_service<carma_planning_msgs::srv::PlanTrajectory>("yield_plugin/plan_trajectory", 
                                            std::bind(&Node::callback, yield_node.get(), std_ph::_1, std_ph::_2, std_ph::_3));

    yield_node->configure();
    yield_node->activate(); 

    auto ilc_node = std::make_shared<inlanecruising_plugin::InLaneCruisingPluginNode>(rclcpp::NodeOptions());

    ilc_node->configure();
    ilc_node->activate(); 

    // Add these nodes to an executor to spin them and trigger callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(yield_node->get_node_base_interface());
    executor.add_node(ilc_node->get_node_base_interface());

    carma_planning_msgs::srv::PlanTrajectory::Request traj_srv;
    traj_srv.initial_trajectory_plan.trajectory_id = "YieldReq";
   
    InLaneCruisingPluginConfig config;
    config.enable_object_avoidance = true;
    config.default_downsample_ratio = 1;

    auto yield_srv = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Request>(traj_srv);
    auto yield_resp = ilc_node->yield_client_->async_send_request(yield_srv);

      // Spin executor for 2 seconds
    auto end_time = std::chrono::system_clock::now() + std::chrono::seconds(2);
    
    while(std::chrono::system_clock::now() < end_time){
        executor.spin_once();
    }

    auto future_status = yield_resp.wait_for(std::chrono::milliseconds(100));

    if (future_status == std::future_status::ready)
    {
        RCLCPP_DEBUG_STREAM(ilc_node->get_logger(), "Received Traj from Yield");
        RCLCPP_INFO_STREAM(rclcpp::get_logger(ILC_LOGGER), "ILC Traj Service called");
    }
    else
    {
        throw std::invalid_argument("Unable to Call Yield Plugin");
    }


    ASSERT_EQ(yield_resp.get()->trajectory_plan.trajectory_id, "YieldResp");
}
}