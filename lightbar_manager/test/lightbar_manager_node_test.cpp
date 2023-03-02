/*
 * Copyright (C) 2023 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License") { you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either exp`tations under
 * the License.
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <thread>
#include "lightbar_manager/lightbar_manager_node.hpp"

namespace lightbar_manager
{

void setupUnitTest(const std::shared_ptr<lightbar_manager::LightBarManagerWorker>& lbm_)
{
    // Add mock components for unit test
    while (lbm_->control_priorities.size() != 0)
        lbm_->control_priorities.pop_back();
    lbm_->control_priorities.push_back("lightbar_manager");
    lbm_->control_priorities.push_back("tester1");
    lbm_->control_priorities.push_back("tester2");
    lbm_->control_priorities.push_back("tester3");
    return;
}

TEST(LightBarManagerNodeTest, testSetIndicator) 
{   
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    auto lbm = std::make_shared<lightbar_manager::LightBarManager>(options);
    setupUnitTest(lbm->lbm_);
    rclcpp_lifecycle::State dummy;
    lbm->handle_on_configure(dummy);
    lbm->handle_on_activate(dummy);

    int response_code;
    /*
    * Response_code: SUCCESS(0) will never be achieved in unit test
    * However, receiving ERROR(2) can be thought as successful as it reached 
    * lightbar driver service request stage
    */

    // Lightbar_manager should be able to set the green indicators right away  
    response_code = lbm->setIndicator(GREEN_SOLID, ON, "lightbar_manager");
    EXPECT_EQ(2, response_code);
    // Any other component should not be able to change indicators
    response_code = lbm->setIndicator(GREEN_SOLID, ON, "CARMAAtItAgain!");
    EXPECT_EQ(1, response_code);
    // Empty component should not be able to control as well
    // Although they are technically controlling it
    response_code = lbm->setIndicator(YELLOW_ARROW_LEFT, ON, "");
    EXPECT_EQ(1, response_code); 
    // Priority list component cannot change indicator before taking control
    response_code = lbm->setIndicator(YELLOW_ARROW_LEFT, ON, "tester1");
    EXPECT_EQ(1, response_code);

}

TEST(LightBarManagerNodeTest, testTurnOffAll) 
{
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    auto lbm = std::make_shared<lightbar_manager::LightBarManager>(options);
    setupUnitTest(lbm->lbm_);
    rclcpp_lifecycle::State dummy;
    lbm->handle_on_configure(dummy);
    lbm->handle_on_activate(dummy);
    auto worker = lbm->lbm_;

    // As unit test cannot actually turn on indicators, observing change in control is enough
    std::vector<LightBarIndicator> some_indicators = 
        {YELLOW_ARROW_LEFT, YELLOW_ARROW_OUT, YELLOW_DIM};
    worker->requestControl(some_indicators, "tester1");
    std::map<LightBarIndicator, std::string> curr_owners= worker->getIndicatorControllers();
    EXPECT_EQ("tester1", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("tester1", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("tester1", curr_owners[YELLOW_DIM]);
    // In order to turn all off, lightbar_manager has to take control and release once done
    lbm->turnOffAll();
    curr_owners= lbm->getWorker()->getIndicatorControllers();
    EXPECT_EQ("", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("", curr_owners[YELLOW_DIM]);
}

TEST(LightBarManagerNodeTest, testTurnSignalCallback)
{
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    auto lbm = std::make_shared<lightbar_manager::LightBarManager>(options);
    setupUnitTest(lbm->lbm_);
    rclcpp_lifecycle::State dummy;
    lbm->handle_on_configure(dummy);
    lbm->handle_on_activate(dummy);
    auto worker = lbm->lbm_;

    //rclcpp::service::waitForService("/hardware_interface/lightbar/set_lights", rclcpp::Duration(60, 0));
    lbm->getWorker()->control_priorities.push_back("tester_left");
    lbm->getWorker()->control_priorities.push_back("tester_right");
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("lightbar_manager"),"Below 'LightBarManager was not able to set light...' errors are expected");
    automotive_platform_msgs::msg::TurnSignalCommand msg;
    msg.mode = 1;
    // turn right (no prev owner)
    msg.turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::RIGHT;
    lbm->processTurnSignal(msg);
    std::map<lightbar_manager::LightBarIndicator, std::string> owners = lbm->getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("lightbar_manager") == 0);
    msg.turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::NONE;
    lbm->processTurnSignal(msg);
    owners = lbm->getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("") == 0);

    // through other mutually inclusive lights, it was on before the turn
    lbm->getWorker()->requestControl({YELLOW_ARROW_LEFT}, "tester_left");
    lbm->getWorker()->requestControl({YELLOW_ARROW_RIGHT}, "tester_unaffected");
    owners =lbm->getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_LEFT].compare("tester_left") == 0);
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("tester_unaffected") == 0);
    // turn left and finish
    msg.turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::LEFT;
    lbm->processTurnSignal(msg);
    msg.turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::NONE;
    lbm->processTurnSignal(msg);
    // make sure the previous owner is there
    owners = lbm->getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_LEFT].compare("tester_left") == 0);
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("tester_unaffected") == 0);

    // through other mutually inclusive lights, it was on before the turn
    lbm->getWorker()->requestControl({YELLOW_ARROW_RIGHT}, "tester_right");
    owners =lbm->getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("tester_right") == 0);
    // turn right and don't finish 
    msg.turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::RIGHT;
    lbm->processTurnSignal(msg);
    // middle of turn
    owners =lbm->getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("lightbar_manager") == 0);
    lbm->processTurnSignal(msg);
    owners =lbm->getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("lightbar_manager") == 0);
    msg.turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::NONE;
    lbm->processTurnSignal(msg);
    // make sure the previous owner is there
    owners = lbm->getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_LEFT].compare("tester_left") == 0);
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("tester_right") == 0);
    
}

} // namespace lightbar_manager


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
 
  bool success = RUN_ALL_TESTS();

  //shutdown ROS
  rclcpp::shutdown();

  return success;
}