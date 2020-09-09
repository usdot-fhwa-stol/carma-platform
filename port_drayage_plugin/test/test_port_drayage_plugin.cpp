/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include "port_drayage_plugin/port_drayage_worker.h"
#include "port_drayage_plugin/port_drayage_state_machine.h"
#include "port_drayage_plugin/port_drayage_plugin.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <sstream>

TEST(PortDrayageTest, testComposeArrivalMessage)
{
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        "TEST_ID", 
        "TEST_CARGO_ID", 
        "TEST_CARMA_HOST_ID", 
        std::function<void(cav_msgs::MobilityOperation)>(), 
        1.0};

    cav_msgs::MobilityOperation msg = pdw.compose_arrival_message();

    ASSERT_EQ("carma/port_drayage", msg.strategy);
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg.header.sender_id);
    ASSERT_FALSE(msg.strategy_params.empty());

    std::istringstream strstream(msg.strategy_params);
    using boost::property_tree::ptree;
    ptree pt;
    std::stringstream body_stream;
    boost::property_tree::json_parser::read_json(strstream, pt);
    std::string cmv_id = pt.get<std::string>("cmv_id");
    std::string cargo_id = pt.get<std::string>("cargo_id");
    std::string operation = pt.get<std::string>("operation");

    ASSERT_EQ("TEST_ID", cmv_id);
    ASSERT_EQ("TEST_CARGO_ID", cargo_id);
    ASSERT_EQ("ARRIVED_AT_DESTINATION", operation);
}

TEST(PortDrayageTest, testCheckStop1)
{
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        "TEST_ID", 
        "TEST_CARGO_ID", 
        "TEST_CARMA_HOST_ID", 
        std::function<void(cav_msgs::MobilityOperation)>(), 
        1.0};

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 0;
    geometry_msgs::TwistStampedConstPtr twistptr{ new geometry_msgs::TwistStamped{twist}};
    pdw.set_current_speed(twistptr);

    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver mvr;
    mvr.type = cav_msgs::Maneuver::STOP_AND_WAIT;
    mvr.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "Port Drayage Plugin";
    plan.maneuvers.push_back(mvr);
    cav_msgs::ManeuverPlanConstPtr planptr{ new cav_msgs::ManeuverPlan{plan}};
    pdw.set_maneuver_plan(planptr);

    bool stopped = pdw.check_for_stop(planptr, twistptr);

    ASSERT_TRUE(stopped);
}

TEST(PortDrayageTest, testCheckStop2)
{
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        "TEST_ID", 
        "TEST_CARGO_ID", 
        "TEST_CARMA_HOST_ID", 
        std::function<void(cav_msgs::MobilityOperation)>(), 
        1.0};

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 2.0;
    geometry_msgs::TwistStampedConstPtr twistptr{ new geometry_msgs::TwistStamped{twist}};
    pdw.set_current_speed(twistptr);

    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver mvr;
    mvr.type = cav_msgs::Maneuver::STOP_AND_WAIT;
    mvr.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "Port Drayage Plugin";
    plan.maneuvers.push_back(mvr);
    cav_msgs::ManeuverPlanConstPtr planptr{ new cav_msgs::ManeuverPlan{plan}};
    pdw.set_maneuver_plan(planptr);

    bool stopped = pdw.check_for_stop(planptr, twistptr);

    ASSERT_FALSE(stopped);
}

TEST(PortDrayageTest, testCheckStop3)
{
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        "TEST_ID", 
        "TEST_CARGO_ID", 
        "TEST_CARMA_HOST_ID", 
        std::function<void(cav_msgs::MobilityOperation)>(), 
        1.0};

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 0.0;
    geometry_msgs::TwistStampedConstPtr twistptr{ new geometry_msgs::TwistStamped{twist}};
    pdw.set_current_speed(twistptr);

    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver mvr;
    mvr.type = cav_msgs::Maneuver::STOP_AND_WAIT;
    mvr.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "Route Following Plugin";
    plan.maneuvers.push_back(mvr);
    plan.maneuvers.push_back(mvr);
    cav_msgs::ManeuverPlanConstPtr planptr{ new cav_msgs::ManeuverPlan{plan}};
    pdw.set_maneuver_plan(planptr);

    bool stopped = pdw.check_for_stop(planptr, twistptr);

    ASSERT_FALSE(stopped);
}

TEST(PortDrayageTest, testCheckStop4)
{
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        "TEST_ID", 
        "TEST_CARGO_ID", 
        "TEST_CARMA_HOST_ID", 
        std::function<void(cav_msgs::MobilityOperation)>(), 
        1.0};

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 2.0;
    geometry_msgs::TwistStampedConstPtr twistptr{ new geometry_msgs::TwistStamped{twist}};
    pdw.set_current_speed(twistptr);

    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver mvr;
    mvr.type = cav_msgs::Maneuver::STOP_AND_WAIT;
    mvr.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "Route Following Plugin";
    plan.maneuvers.push_back(mvr);
    cav_msgs::ManeuverPlanConstPtr planptr{ new cav_msgs::ManeuverPlan{plan}};
    pdw.set_maneuver_plan(planptr);

    bool stopped = pdw.check_for_stop(planptr, twistptr);

    ASSERT_FALSE(stopped);
}

TEST(PortDrayageTest, testStateMachine)
{
    port_drayage_plugin::PortDrayageStateMachine pdsm;

    // Verify system initial state
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::INACTIVE, pdsm.get_state());

    // Startup CARMA system and begin automation
    pdsm.process_event(port_drayage_plugin::PortDrayageEvent::DRAYAGE_START);

    // Verify that we are en route to our next destination (port)
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE, pdsm.get_state());

    // Notify state machine we've arrived
    pdsm.process_event(port_drayage_plugin::PortDrayageEvent::ARRIVED_AT_DESTINATION);
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::AWAITING_DIRECTION, pdsm.get_state());

    // Notify state machine we've recieved the next destination
    pdsm.process_event(port_drayage_plugin::PortDrayageEvent::RECEIVED_NEW_DESTINATION);
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE, pdsm.get_state());

    // Rest of the state machine to be implemented and validated in future stories
}

TEST(PortDrayageTest, testEstimateDistanceToStop)
{
    ASSERT_EQ(port_drayage_plugin::estimate_distance_to_stop(10.0, 1.0), 50.0);
}

TEST(PortDrayageTest, testEstimateTimeToStop)
{
    ASSERT_EQ(port_drayage_plugin::estimate_time_to_stop(10.0, 1.0), 20.0);
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
