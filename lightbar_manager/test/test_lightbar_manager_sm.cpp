/*
 * Copyright (C) 2023 LEIDOS.
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

#include "lightbar_manager/lightbar_manager_sm.hpp"
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace lightbar_manager
{

TEST(LightBarManagerStateMachineTest, testStates)
{
    LightBarManagerStateMachine lbsm, lbsm_reset;
    // test initial state
    EXPECT_EQ(DISENGAGED, lbsm.getCurrentState());
    
    /*
    * test possible/impossible states from lightbar DISENGAGE
    */
    carma_planning_msgs::msg::GuidanceState msg;
    // guidance startup, lightbar should not change state
    msg.state = carma_planning_msgs::msg::GuidanceState::STARTUP;
    lbsm.handleStateChange(msg);
    EXPECT_EQ(DISENGAGED, lbsm.getCurrentState());
    // guidance driver_ready, lightbar should become active
    msg.state = carma_planning_msgs::msg::GuidanceState::DRIVERS_READY;
    lbsm.handleStateChange(msg);
    EXPECT_EQ(ACTIVE, lbsm.getCurrentState());
    // guidance SHUTDOWN, lightbar should disengage
    msg.state = carma_planning_msgs::msg::GuidanceState::SHUTDOWN;
    lbsm.handleStateChange(msg);
    EXPECT_EQ(DISENGAGED, lbsm.getCurrentState());
     // guidance INACTIVE, lightbar should become active
    msg.state = carma_planning_msgs::msg::GuidanceState::INACTIVE;
    lbsm.handleStateChange(msg);
    EXPECT_EQ(ACTIVE, lbsm.getCurrentState());
    // guidance ACTIVE, lightbar should change to ACTIVE
    msg.state = carma_planning_msgs::msg::GuidanceState::ACTIVE;
    lbsm.handleStateChange(msg);
    EXPECT_EQ(ACTIVE, lbsm.getCurrentState());
    lbsm = lbsm_reset;
    // guidance ENGAGED, lightbar should change to ENGAGED
    msg.state = carma_planning_msgs::msg::GuidanceState::ENGAGED;
    lbsm.handleStateChange(msg);
    EXPECT_EQ(ENGAGED, lbsm.getCurrentState());

    /*
    * test possible/impossible states from ENGAGED/ACTIVE
    */
    msg.state = carma_planning_msgs::msg::GuidanceState::ENGAGED;
    // guidance ENGAGED, lightbar should not change
    msg.state = carma_planning_msgs::msg::GuidanceState::ENGAGED;
    lbsm.handleStateChange(msg);
    EXPECT_EQ(ENGAGED, lbsm.getCurrentState());
    // guidance ACTIVE, lightbar should change to ACTIVE
    msg.state = carma_planning_msgs::msg::GuidanceState::ACTIVE;
    lbsm.handleStateChange(msg);
    EXPECT_EQ(ACTIVE, lbsm.getCurrentState());
    // guidance ENGAGED, lightbar should change to ENGAGED
    msg.state = carma_planning_msgs::msg::GuidanceState::ENGAGED;
    lbsm.handleStateChange(msg);
    EXPECT_EQ(ENGAGED, lbsm.getCurrentState());
    // guidance SHUTDOWN, lightbar should change to DISENGAGED
    msg.state = carma_planning_msgs::msg::GuidanceState::SHUTDOWN;
    lbsm.handleStateChange(msg);
    EXPECT_EQ(DISENGAGED, lbsm.getCurrentState());
    msg.state = carma_planning_msgs::msg::GuidanceState::ACTIVE;
    lbsm.handleStateChange(msg);
    // guidance SHUTDOWN, lightbar should change to DISENGAGED
    msg.state = carma_planning_msgs::msg::GuidanceState::SHUTDOWN;
    lbsm.handleStateChange(msg);
    EXPECT_EQ(DISENGAGED, lbsm.getCurrentState());

    // Illogical states check, should not change the states.
    msg.state = 7;
    lbsm.handleStateChange(msg);
    EXPECT_EQ(DISENGAGED, lbsm.getCurrentState());
}

}   // namespace lightbar_manager