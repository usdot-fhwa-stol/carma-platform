/*
 * Copyright (C) 2019 LEIDOS.
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
#include <gtest/gtest.h>

TEST(GuidanceStateMachineTest, testStates)
{
    guidance::GuidanceStateMachine gsm = guidance_state_machine_factory.createCadilacInstance();
    // test initial state
    EXPECT_EQ(1, static_cast<int>(gsm.getCurrentState()));
    cav_msgs::SystemAlert alert;
    alert.type = alert.DRIVERS_READY;
    cav_msgs::SystemAlertConstPtr alert_pointer(new cav_msgs::SystemAlert(alert));
    gsm.onSystemAlert(alert_pointer);
    // test drivers ready state
    EXPECT_EQ(2, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(true);
    // test active state
    EXPECT_EQ(3, static_cast<int>(gsm.getCurrentState()));
    cav_msgs::RobotEnabled status;
    status.robot_enabled = true;
    status.robot_active = true;
    cav_msgs::RobotEnabledConstPtr status_pointer(new cav_msgs::RobotEnabled(status));
    gsm.onRoboticStatus(status_pointer);
    // test engaged state
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(false);
    // test disengage and restart state
    EXPECT_EQ(2, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(true);
    gsm.onRoboticStatus(status_pointer);
    // test re-engage state
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    status.robot_active = false;
    cav_msgs::RobotEnabledConstPtr status_pointer_2(new cav_msgs::RobotEnabled(status));
    gsm.onRoboticStatus(status_pointer_2);
    // test manual override and inactive state
    EXPECT_EQ(5, static_cast<int>(gsm.getCurrentState()));
    alert.type = alert.SHUTDOWN;
    cav_msgs::SystemAlertConstPtr alert_pointer_2(new cav_msgs::SystemAlert(alert));
    gsm.onSystemAlert(alert_pointer_2);
    // test shut down state
    EXPECT_EQ(0, static_cast<int>(gsm.getCurrentState()));
}

TEST(GuidanceStateMachineTest2, testStates)
{
    guidance::GuidanceStateMachine gsm = guidance_state_machine_factory.createLexusInstance();
    // test initial state
    EXPECT_EQ(1, static_cast<int>(gsm.getCurrentState()));
    cav_msgs::SystemAlert alert;
    alert.type = alert.DRIVERS_READY;
    cav_msgs::SystemAlertConstPtr alert_pointer(new cav_msgs::SystemAlert(alert));
    gsm.onSystemAlert(alert_pointer);
    // test drivers ready state
    EXPECT_EQ(2, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(true);
    // test active state
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    cav_msgs::RobotEnabled status;
    status.robot_enabled = true;
    status.robot_active = true;
    cav_msgs::RobotEnabledConstPtr status_pointer(new cav_msgs::RobotEnabled(status));
    gsm.onRoboticStatus(status_pointer);
    // test engaged state
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(false);
    // test disengage and restart state
    EXPECT_EQ(2, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(true);
    gsm.onRoboticStatus(status_pointer);
    // test re-engage state
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    status.robot_active = false;
    cav_msgs::RobotEnabledConstPtr status_pointer_2(new cav_msgs::RobotEnabled(status));
    gsm.onRoboticStatus(status_pointer_2);
    // test manual override and inactive state
    EXPECT_EQ(5, static_cast<int>(gsm.getCurrentState()));
    alert.type = alert.SHUTDOWN;
    cav_msgs::SystemAlertConstPtr alert_pointer_2(new cav_msgs::SystemAlert(alert));
    gsm.onSystemAlert(alert_pointer_2);
    // test shut down state
    EXPECT_EQ(0, static_cast<int>(gsm.getCurrentState()));
}

// Run all the tests
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
