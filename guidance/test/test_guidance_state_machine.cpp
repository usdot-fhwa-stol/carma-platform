/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include "guidance/guidance_state_machine.hpp"
#include "guidance/guidance_worker.hpp"
#include <gtest/gtest.h>

TEST(GuidanceStateMachineTest, testStates)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    auto logger_interface = node->get_node_logging_interface();
    guidance::GuidanceStateMachine gsm(logger_interface);

    // test initial state
    EXPECT_EQ(1, static_cast<int>(gsm.getCurrentState()));

    // test SetEnableRobotic call flag
    EXPECT_TRUE(!gsm.shouldCallSetEnableRobotic());
    gsm.onSetGuidanceActive(true);

    // test engage on startup state, should not cause state change
    EXPECT_EQ(1, static_cast<int>(gsm.getCurrentState()));

    // test drivers ready state
    gsm.onGuidanceInitialized();
    EXPECT_EQ(2, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(true);

    // test active state
    EXPECT_EQ(3, static_cast<int>(gsm.getCurrentState()));
    EXPECT_TRUE(gsm.shouldCallSetEnableRobotic());
    EXPECT_TRUE(!gsm.shouldCallSetEnableRobotic());
    gsm.onSetGuidanceActive(false);

    // test disengage on active
    EXPECT_EQ(2, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(true);
    EXPECT_EQ(3, static_cast<int>(gsm.getCurrentState()));
    carma_driver_msgs::msg::RobotEnabled engage_status;
    engage_status.robot_enabled = true;
    engage_status.robot_active = true;
    carma_driver_msgs::msg::RobotEnabled disengage_status;
    disengage_status.robot_enabled = true;
    disengage_status.robot_active = false;
    std::unique_ptr<carma_driver_msgs::msg::RobotEnabled> engage_status_pointer = std::make_unique<carma_driver_msgs::msg::RobotEnabled>(engage_status);
    std::unique_ptr<carma_driver_msgs::msg::RobotEnabled> disengage_status_pointer = std::make_unique<carma_driver_msgs::msg::RobotEnabled>(disengage_status);
    gsm.onRoboticStatus(move(disengage_status_pointer));
    EXPECT_EQ(3, static_cast<int>(gsm.getCurrentState()));
    gsm.onRoboticStatus(move(engage_status_pointer));

    // test engaged state
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    std::unique_ptr<carma_driver_msgs::msg::RobotEnabled> engage_status_pointer_2 = std::make_unique<carma_driver_msgs::msg::RobotEnabled>(engage_status);
    gsm.onRoboticStatus(move(engage_status_pointer_2));
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(false);
    std::unique_ptr<carma_driver_msgs::msg::RobotEnabled> disengage_status_pointer_2 = std::make_unique<carma_driver_msgs::msg::RobotEnabled>(disengage_status);
    gsm.onRoboticStatus(move(disengage_status_pointer_2));

    // test disengage and restart state
    EXPECT_EQ(2, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(true);
    std::unique_ptr<carma_driver_msgs::msg::RobotEnabled> engage_status_pointer_3 = std::make_unique<carma_driver_msgs::msg::RobotEnabled>(engage_status);
    gsm.onRoboticStatus(move(engage_status_pointer_3));

    // test re-engage state
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    std::unique_ptr<carma_driver_msgs::msg::RobotEnabled> disengage_status_pointer_3 = std::make_unique<carma_driver_msgs::msg::RobotEnabled>(disengage_status);
    gsm.onRoboticStatus(move(disengage_status_pointer_3));

    // test manual override and inactive state
    EXPECT_EQ(5, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(true);
    std::unique_ptr<carma_driver_msgs::msg::RobotEnabled> engage_status_pointer_4 = std::make_unique<carma_driver_msgs::msg::RobotEnabled>(engage_status);
    gsm.onRoboticStatus(move(engage_status_pointer_4));

    // test re-engage state from inactive
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    carma_planning_msgs::msg::RouteEvent route_event;
    route_event.event = carma_planning_msgs::msg::RouteEvent::ROUTE_COMPLETED;
    std::unique_ptr<carma_planning_msgs::msg::RouteEvent> route_event_pointer = std::make_unique<carma_planning_msgs::msg::RouteEvent>(route_event);
    gsm.onRouteEvent(move(route_event_pointer));

    // test ENTER_PARK state at end of route
    EXPECT_EQ(6, static_cast<int>(gsm.getCurrentState()));
    autoware_msgs::msg::VehicleStatus vehicle_status;
    vehicle_status.current_gear.gear = autoware_msgs::msg::Gear::PARK; // '3' indicates gearshift is set to Park
    std::unique_ptr<autoware_msgs::msg::VehicleStatus> vehicle_status_pointer = std::make_unique<autoware_msgs::msg::VehicleStatus>(vehicle_status);
    gsm.onVehicleStatus(move(vehicle_status_pointer));

    // test DRIVERS_READY state
    EXPECT_EQ(2, static_cast<int>(gsm.getCurrentState()));

    // test shut down state
    gsm.onGuidanceShutdown();
    EXPECT_EQ(0, static_cast<int>(gsm.getCurrentState()));

    // Should not recover from OFF state
    gsm.onGuidanceInitialized();
    EXPECT_EQ(0, static_cast<int>(gsm.getCurrentState()));
}