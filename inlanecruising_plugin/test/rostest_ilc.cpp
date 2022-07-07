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
//
#include <thread>
#include <chrono>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>

TEST(InLaneCruisingPluginTest, rostest1)
{
    rclcpp::CARMANodeHandle nh;
    bool flag_trajectory = false;
    bool flag_yield = false;
    std::string res = "";

    carma_planning_msgs::srv::PlanTrajectory traj_srv;
    traj_srv.request.initial_trajectory_plan.trajectory_id = "ILCReq";
   
    rclcpp::ServiceClient plugin1= nh.serviceClient<carma_planning_msgs::srv::PlanTrajectory>("plugins/InLaneCruisingPlugin/plan_trajectory");

    RCLCPP_INFO_STREAM(rclcpp::get_logger(ILC_LOGGER), "ilc service: " << plugin1.getService());
    if(plugin1.waitForExistence(rclcpp::Duration(5.0)))
    {
        rclcpp::spinOnce();
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(ILC_LOGGER), ("waiting");
        if (plugin1.call(traj_srv))
        {
            res = traj_srv.response.trajectory_plan.trajectory_id;
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(ILC_LOGGER), ("ILC Traj Service called");
            flag_trajectory = true;
            flag_yield = true;
            
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(ILC_LOGGER), ("ILC Trajectory Service not called");
            res = "error";
        }
    }
    EXPECT_TRUE(flag_trajectory);
    ASSERT_TRUE(flag_yield);
    // EXPECT_EQ(res, "ILC2Yield");
}



int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv, "rostest_ilc");
    rclcpp::NodeHandle nh;
    auto res = RUN_ALL_TESTS();
    return res;
}

