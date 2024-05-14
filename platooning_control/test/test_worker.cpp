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

#include "platoon_control/platoon_control_worker.hpp"


TEST(PlatoonControlWorkerTest, test1)
{
    platoon_control::PlatoonControlWorker pcw;
    pcw.ctrl_config_ = std::make_shared<platoon_control::PlatooningControlPluginConfig>();
    carma_planning_msgs::msg::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 2.0;
    pcw.generate_speed(point);
    EXPECT_NEAR(0, pcw.speedCmd_, 0.1);
}

TEST(PlatoonControlWorkerTest, test11)
{
    platoon_control::PlatoonLeaderInfo leader;
    platoon_control::PlatoonControlWorker pcw;
    pcw.ctrl_config_ = std::make_shared<platoon_control::PlatooningControlPluginConfig>();
    leader.staticId = "";
    leader.leaderIndex = 0;
    leader.NumberOfVehicleInFront = 1;
    pcw.set_leader(leader);
    carma_planning_msgs::msg::TrajectoryPlanPoint point;
    point.x = 30.0;
    point.y = 20.0;
    pcw.currentSpeed = 10.0;
    pcw.lastCmdSpeed = 10;
    pcw.generate_speed(point);
    EXPECT_NEAR(10.0, pcw.get_last_speed_command(), 0.5);
}

TEST(PlatoonControlWorkerTest, test2)
{

    platoon_control::PlatoonControlWorker pcw;
    platoon_control::PlatooningControlPluginConfig config;
    pcw.ctrl_config_ = std::make_shared<platoon_control::PlatooningControlPluginConfig>();
    platoon_control::PlatoonLeaderInfo leader;
    leader.commandSpeed = 10;
    leader.vehicleSpeed = 10;
    leader.vehiclePosition = 50;
    leader.staticId = "id";
    leader.leaderIndex = 0;
    leader.NumberOfVehicleInFront = 1;
    pcw.set_leader(leader);

    carma_planning_msgs::msg::TrajectoryPlanPoint point;
    point.x = 30.0;
    point.y = 20.0;
    pcw.currentSpeed = 10.0;
    pcw.lastCmdSpeed = 10;
    pcw.generate_speed(point);
    EXPECT_NEAR(9.75, pcw.get_last_speed_command(), 0.5);


    carma_planning_msgs::msg::TrajectoryPlanPoint point2;
    point2.x = 30.0;
    point2.y = 40.0;
    pcw.generate_speed(point2);
    EXPECT_NEAR(10, pcw.get_last_speed_command(), 0.5);

    carma_planning_msgs::msg::TrajectoryPlanPoint point3;
    point3.x = 50.0;
    point3.y = 60.0;
    pcw.generate_speed(point3);
    EXPECT_NEAR(10.25, pcw.get_last_speed_command(), 0.5);

    config.enable_max_adjustment_filter = false;
    config.vehicle_id = "id";
    pcw.generate_speed(point3);
    EXPECT_NEAR(9.5, pcw.get_last_speed_command(), 0.5);

}

TEST(PlatoonControlWorkerTest, test3)
{

    platoon_control::PlatoonControlWorker pcw;
    platoon_control::PlatooningControlPluginConfig config;
    pcw.ctrl_config_ = std::make_shared<platoon_control::PlatooningControlPluginConfig>(config);
    platoon_control::PlatoonLeaderInfo leader;
    leader.commandSpeed = 10;
    leader.vehicleSpeed = 10;
    leader.vehiclePosition = 50;
    leader.staticId = "id";
    leader.leaderIndex = 0;
    leader.NumberOfVehicleInFront = 2;
    pcw.set_leader(leader);

    carma_planning_msgs::msg::TrajectoryPlanPoint point;
    point.x = 30.0;
    point.y = 15.0;
    pcw.currentSpeed = 10.0;
    pcw.lastCmdSpeed = 10;
    pcw.generate_speed(point);
    EXPECT_NEAR(10.25, pcw.get_last_speed_command(), 0.5);


    carma_planning_msgs::msg::TrajectoryPlanPoint point2;
    point2.x = 50.0;
    point2.y = 60.0;
    pcw.platoon_leader.vehiclePosition = 51;
    pcw.generate_speed(point2);
    EXPECT_NEAR(10.5, pcw.get_last_speed_command(), 0.5);

    carma_planning_msgs::msg::TrajectoryPlanPoint point3;
    point3.x = 50.0;
    point3.y = 60.0;
    pcw.platoon_leader.vehiclePosition = 49;
    pcw.generate_speed(point3);
    EXPECT_NEAR(10.25, pcw.get_last_speed_command(), 0.5);

}
