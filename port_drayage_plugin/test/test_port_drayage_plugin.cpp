/*
 * Copyright (C) 2020-2022 LEIDOS.
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

#include "port_drayage_plugin/port_drayage_worker.hpp"
#include "port_drayage_plugin/port_drayage_state_machine.hpp"
#include "port_drayage_plugin/port_drayage_plugin.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

TEST(PortDrayageTest, testComposeArrivalMessage)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::Clock::SharedPtr clock = node->get_clock();

    // Test initial arrival message for pdw that has cargo and has the Staging Area Entrance as its first destination
    port_drayage_plugin::PortDrayageWorker pdw(node->get_node_logging_interface(),
        clock,
        [](carma_v2x_msgs::msg::MobilityOperation){}, 
        [](carma_msgs::msg::UIInstructions){},
        [](std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request>){return true;} 
    );

    pdw.setVehicleID("DOT-11111");
    pdw.setCargoID("321");
    pdw.setEnablePortDrayageFlag(true);
    pdw.setStartingAtStagingAreaFlag(true);

    // Set the pdw's map projector set its current pose
    std::string base_proj = "+proj=tmerc +lat_0=38.95622708 +lon_0=-77.15066142 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    std_msgs::msg::String georeference_msg;
    georeference_msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> georeference_msg_ptr = std::make_unique<std_msgs::msg::String>(georeference_msg);
    pdw.onNewGeoreference(std::move(georeference_msg_ptr));

    // State Machine should transition to EN_ROUTE_TO_INITIAL_DESTINATION after guidance state is first engaged
    carma_planning_msgs::msg::GuidanceState guidance_state;
    guidance_state.state = carma_planning_msgs::msg::GuidanceState::ENGAGED;
    std::unique_ptr<carma_planning_msgs::msg::GuidanceState> guidance_state_ptr = std::make_unique<carma_planning_msgs::msg::GuidanceState>(guidance_state);
    pdw.onGuidanceState(std::move(guidance_state_ptr));

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
    pdw.onNewPose(std::move(pose_msg_ptr)); // Sets the host vehicle's current gps lat/lon position

    carma_v2x_msgs::msg::MobilityOperation msg = pdw.composeArrivalMessage();
    
    ASSERT_EQ("carma/port_drayage", msg.strategy);
    ASSERT_EQ("DOT-11111", msg.m_header.sender_id);
    ASSERT_FALSE(msg.strategy_params.empty());

    std::istringstream strstream(msg.strategy_params);
    using boost::property_tree::ptree;
    ptree pt;
    boost::property_tree::json_parser::read_json(strstream, pt);

    std::string cmv_id = pt.get<std::string>("cmv_id");
    std::string operation = pt.get<std::string>("operation");
    bool has_cargo = pt.get<bool>("cargo");
    double vehicle_longitude = pt.get<double>("location.longitude");
    double vehicle_latitude = pt.get<double>("location.latitude");

    ASSERT_EQ("DOT-11111", cmv_id);
    ASSERT_EQ("ENTER_STAGING_AREA", operation);
    ASSERT_EQ(pt.count("action_id"), 0);
    ASSERT_TRUE(has_cargo);
    ASSERT_NEAR(38.95622708, vehicle_latitude, 0.00000001);
    ASSERT_NEAR(-77.15066142, vehicle_longitude, 0.00000001);

    // Test initial arrival message for pdw that doesn't have cargo and has the Port Entrance as its first destination
    port_drayage_plugin::PortDrayageWorker pdw2(node->get_node_logging_interface(),
        clock,
        [](carma_v2x_msgs::msg::MobilityOperation){}, 
        [](carma_msgs::msg::UIInstructions){},
        [](std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request>){return true;} 
    );
    
    pdw2.setVehicleID("123");
    pdw2.setCargoID("");
    pdw2.setEnablePortDrayageFlag(true);
    pdw2.setStartingAtStagingAreaFlag(false); // Sets the Port Entrance as the first destination

    std::unique_ptr<std_msgs::msg::String> georeference_msg_ptr2 = std::make_unique<std_msgs::msg::String>(georeference_msg);
    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr2 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
    std::unique_ptr<carma_planning_msgs::msg::GuidanceState> guidance_state_ptr2 = std::make_unique<carma_planning_msgs::msg::GuidanceState>(guidance_state);
    pdw2.onNewGeoreference(std::move(georeference_msg_ptr2));
    pdw2.onNewPose(std::move(pose_msg_ptr2));
    pdw2.onGuidanceState(std::move(guidance_state_ptr2));

    carma_v2x_msgs::msg::MobilityOperation msg2 = pdw2.composeArrivalMessage();

    ASSERT_EQ("carma/port_drayage", msg2.strategy);
    ASSERT_EQ("123", msg2.m_header.sender_id);
    ASSERT_FALSE(msg2.strategy_params.empty());

    std::istringstream strstream2(msg2.strategy_params);
    ptree pt2;
    boost::property_tree::json_parser::read_json(strstream2, pt2);

    std::string cmv_id2 = pt2.get<std::string>("cmv_id");
    std::string operation2 = pt2.get<std::string>("operation");
    bool has_cargo2 = pt2.get<bool>("cargo");
    double vehicle_longitude2 = pt2.get<double>("location.longitude");
    double vehicle_latitude2 = pt2.get<double>("location.latitude");

    ASSERT_EQ("123", cmv_id2);
    ASSERT_EQ(0, pt2.count("cargo_id")); // Broadcasted arrival message should not include cargo_id since vehicle is not carrying cargo
    ASSERT_EQ("ENTER_PORT", operation2);
    ASSERT_FALSE(has_cargo2); // False since vehicle is not currently carrying cargo
    ASSERT_NEAR(38.95622708, vehicle_latitude2, 0.00000001);
    ASSERT_NEAR(-77.15066142, vehicle_longitude2, 0.00000001);
}

// Test State Machine flow strictly from PortDrayageEvents
TEST(PortDrayageTest, testStateMachine1)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    port_drayage_plugin::PortDrayageStateMachine pdsm(node->get_node_logging_interface());

    // Verify system initial state
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::INACTIVE, pdsm.getState());

    // Startup CARMA system and begin automation
    pdsm.processEvent(port_drayage_plugin::PortDrayageEvent::DRAYAGE_START);

    // Verify that we are en route to our next destination (port)
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE_TO_INITIAL_DESTINATION, pdsm.getState());

    // Notify state machine we've arrived
    pdsm.processEvent(port_drayage_plugin::PortDrayageEvent::ARRIVED_AT_DESTINATION);
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::AWAITING_DIRECTION, pdsm.getState());

    // Notify state machine we've recieved the next destination
    pdsm.processEvent(port_drayage_plugin::PortDrayageEvent::RECEIVED_NEW_DESTINATION);
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE_TO_RECEIVED_DESTINATION, pdsm.getState());
}

// Test communication between PortDrayageWorker and PortDrayageStateMachine for State Machine flow
TEST(PortDrayageTest, testPortDrayageStateMachine2)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::Clock::SharedPtr clock = node->get_clock();

    // Test initial arrival message for pdw that has cargo and has the Staging Area Entrance as its first destination
    port_drayage_plugin::PortDrayageWorker pdw(node->get_node_logging_interface(),
        clock,
        [](carma_v2x_msgs::msg::MobilityOperation){}, 
        [](carma_msgs::msg::UIInstructions){},
        [](std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request>){return true;} 
    );

    pdw.setVehicleID("123");
    pdw.setCargoID("");
    pdw.setEnablePortDrayageFlag(true);
    pdw.setStartingAtStagingAreaFlag(true);

    // State Machine should begin in INACTIVE state
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::INACTIVE, pdw.getPortDrayageState());

    // State Machine should transition to EN_ROUTE_TO_INITIAL_DESTINATION after guidance state is first engaged
    carma_planning_msgs::msg::GuidanceState guidance_state;
    guidance_state.state = carma_planning_msgs::msg::GuidanceState::ENGAGED;
    std::unique_ptr<carma_planning_msgs::msg::GuidanceState> guidance_state_pointer = std::make_unique<carma_planning_msgs::msg::GuidanceState>(guidance_state);
    pdw.onGuidanceState(std::move(guidance_state_pointer));

    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE_TO_INITIAL_DESTINATION, pdw.getPortDrayageState());

    // State Machine should transition to AWAITING_DIRECTION if a 'ROUTE_LOADED' event occurs immediately after a 'ROUTE_COMPLETED' event
    carma_planning_msgs::msg::RouteEvent route_event_1;
    route_event_1.event = carma_planning_msgs::msg::RouteEvent::ROUTE_COMPLETED; 
    std::unique_ptr<carma_planning_msgs::msg::RouteEvent> route_event_pointer_1 = std::make_unique<carma_planning_msgs::msg::RouteEvent>(route_event_1);
    pdw.onRouteEvent(std::move(route_event_pointer_1)); // PortDrayageWorker receives RouteEvent indicating route has been completed

    carma_planning_msgs::msg::RouteEvent route_event_2;
    route_event_2.event = carma_planning_msgs::msg::RouteEvent::ROUTE_LOADED; 
    std::unique_ptr<carma_planning_msgs::msg::RouteEvent> route_event_pointer_2 = std::make_unique<carma_planning_msgs::msg::RouteEvent>(route_event_2);
    pdw.onRouteEvent(std::move(route_event_pointer_2)); // PortDrayageWorker receives RouteEvent indicating the previously completed route is no longer active
    
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::AWAITING_DIRECTION, pdw.getPortDrayageState());

    // State Machine should transition to 'EN_ROUTE_TO_RECEIVED_DESTINATION' if a new port drayage MobilityOperation message is received

    // Create a MobilityOperation with a cmv_id that is intended for this specific vehicle
    // Note: The strategy_params using the schema for messages of this type that have strategy "carma/port_drayage"
    carma_v2x_msgs::msg::MobilityOperation mobility_operation_msg;
    mobility_operation_msg.strategy = "carma/port_drayage";
    mobility_operation_msg.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"321\", \"location\"\
        : { \"latitude\": \"38.9554377\", \"longitude\": \"-77.1503421\" }, \"destination\": { \"latitude\"\
        : \"38.9550038\", \"longitude\": \"-77.1481983\" }, \"operation\": \"PICKUP\", \"action_id\"\
        : \"32\"}";
    std::unique_ptr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_msg_ptr = std::make_unique<carma_v2x_msgs::msg::MobilityOperation>(mobility_operation_msg);
    pdw.onInboundMobilityOperation(std::move(mobility_operation_msg_ptr)); 

    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE_TO_RECEIVED_DESTINATION, pdw.getPortDrayageState());

    // State Machine should transition to 'AWAITING_DIRECTION' again if a ROUTE_COMPLETED event occurs while the vehicle is stopped
    std::unique_ptr<carma_planning_msgs::msg::RouteEvent> route_event_pointer_3 = std::make_unique<carma_planning_msgs::msg::RouteEvent>(route_event_1);
    std::unique_ptr<carma_planning_msgs::msg::RouteEvent> route_event_pointer_4 = std::make_unique<carma_planning_msgs::msg::RouteEvent>(route_event_2);
    pdw.onRouteEvent(std::move(route_event_pointer_3)); // PortDrayageWorker receives RouteEvent indicating route has been completed
    pdw.onRouteEvent(std::move(route_event_pointer_4)); // PortDrayageWorker receives RouteEvent indicating the previously completed route is no longer active
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::AWAITING_DIRECTION, pdw.getPortDrayageState());
}

TEST(PortDrayageTest, testComposeSetActiveRouteRequest)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::Clock::SharedPtr clock = node->get_clock();

    // Test initial arrival message for pdw that has cargo and has the Staging Area Entrance as its first destination
    port_drayage_plugin::PortDrayageWorker pdw(node->get_node_logging_interface(),
        clock,
        [](carma_v2x_msgs::msg::MobilityOperation){}, 
        [](carma_msgs::msg::UIInstructions){},
        [](std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request>){return true;} 
    );

    pdw.setVehicleID("123");
    pdw.setCargoID("TEST_CARGO_ID");
    pdw.setEnablePortDrayageFlag(true);
    pdw.setStartingAtStagingAreaFlag(true);

    // Create a MobilityOperation with a cmv_id that is intended for this specific vehicle
    // Note: The strategy_params using the schema for messages of this type that have strategy "carma/port_drayage"
    carma_v2x_msgs::msg::MobilityOperation mobility_operation_msg;
    mobility_operation_msg.strategy = "carma/port_drayage";
    mobility_operation_msg.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"321\", \"cargo\": \"false\", \"location\"\
        : { \"latitude\": \"38.9554377\", \"longitude\": \"-77.1503421\" }, \"destination\": { \"latitude\"\
        : \"38.9550038\", \"longitude\": \"-77.1481983\" }, \"operation\": \"MOVING_TO_LOADING_AREA\", \"action_id\"\
        : \"32\", \"next_action\": \"33\" }";
    std::unique_ptr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_msg_ptr = std::make_unique<carma_v2x_msgs::msg::MobilityOperation>(mobility_operation_msg);
    pdw.onInboundMobilityOperation(std::move(mobility_operation_msg_ptr)); 

    // Verify the results of PortDrayageWorker's composeSetActiveRouteRequest() method
    std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request> route_req = pdw.composeSetActiveRouteRequest(*pdw.latest_mobility_operation_msg_.dest_latitude, *pdw.latest_mobility_operation_msg_.dest_longitude);
    ASSERT_EQ(carma_planning_msgs::srv::SetActiveRoute::Request::DESTINATION_POINTS_ARRAY, route_req->choice);
    ASSERT_EQ("MOVING_TO_LOADING_AREA", route_req->route_id);
    ASSERT_EQ(1, route_req->destination_points.size());
    ASSERT_EQ(38.9550038, route_req->destination_points[0].latitude);
    ASSERT_EQ(-77.1481983, route_req->destination_points[0].longitude);
    ASSERT_EQ(false, route_req->destination_points[0].elevation_exists);
}

// Test Case for testing all potential inbound Port Drayage MobilityOperation messages
TEST(PortDrayageTest, testInboundAndComposedMobilityOperation)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::Clock::SharedPtr clock = node->get_clock();

    // Test initial arrival message for pdw that has cargo and has the Staging Area Entrance as its first destination
    port_drayage_plugin::PortDrayageWorker pdw(node->get_node_logging_interface(),
        clock,
        [](carma_v2x_msgs::msg::MobilityOperation){}, 
        [](carma_msgs::msg::UIInstructions){},
        [](std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request>){return true;} 
    );

    pdw.setVehicleID("123");
    pdw.setCargoID("");
    pdw.setEnablePortDrayageFlag(true);
    pdw.setStartingAtStagingAreaFlag(true);

    // State Machine should transition to EN_ROUTE_TO_INITIAL_DESTINATION after guidance state is first engaged
    carma_planning_msgs::msg::GuidanceState guidance_state;
    guidance_state.state = carma_planning_msgs::msg::GuidanceState::ENGAGED;
    std::unique_ptr<carma_planning_msgs::msg::GuidanceState> guidance_state_pointer = std::make_unique<carma_planning_msgs::msg::GuidanceState>(guidance_state);
    pdw.onGuidanceState(std::move(guidance_state_pointer));

    // State Machine should transition to AWAITING_DIRECTION if a 'ROUTE_LOADED' event occurs immediately after a 'ROUTE_COMPLETED' event
    carma_planning_msgs::msg::RouteEvent route_event_1;
    route_event_1.event = carma_planning_msgs::msg::RouteEvent::ROUTE_COMPLETED; 
    std::unique_ptr<carma_planning_msgs::msg::RouteEvent> route_event_pointer_1 = std::make_unique<carma_planning_msgs::msg::RouteEvent>(route_event_1);
    pdw.onRouteEvent(std::move(route_event_pointer_1)); // PortDrayageWorker receives RouteEvent indicating route has been completed

    carma_planning_msgs::msg::RouteEvent route_event_2;
    route_event_2.event = carma_planning_msgs::msg::RouteEvent::ROUTE_LOADED; 
    std::unique_ptr<carma_planning_msgs::msg::RouteEvent> route_event_pointer_2 = std::make_unique<carma_planning_msgs::msg::RouteEvent>(route_event_2);
    pdw.onRouteEvent(std::move(route_event_pointer_2)); // PortDrayageWorker receives RouteEvent indicating the previously completed route is no longer active
    
    // Create a "PICKUP" MobilityOperationConstPtr for pdw
    carma_v2x_msgs::msg::MobilityOperation mobility_operation_msg;
    mobility_operation_msg.strategy = "carma/port_drayage";
    mobility_operation_msg.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"321\", \"location\"\
        : { \"latitude\": \"38.9554377\", \"longitude\": \"-77.1503421\" }, \"destination\": { \"latitude\"\
        : \"38.9550038\", \"longitude\": \"-77.1481983\" }, \"operation\": \"PICKUP\", \"action_id\"\
        : \"32\" }";
    std::unique_ptr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_msg_ptr = std::make_unique<carma_v2x_msgs::msg::MobilityOperation>(mobility_operation_msg);
    pdw.onInboundMobilityOperation(std::move(mobility_operation_msg_ptr));

    // Check that the received message was parsed and stored correctly
    ASSERT_EQ("321", *pdw.latest_mobility_operation_msg_.cargo_id);
    ASSERT_EQ("PICKUP", pdw.latest_mobility_operation_msg_.operation);
    ASSERT_NEAR(-77.1503421, *pdw.latest_mobility_operation_msg_.start_longitude, 0.00000001);
    ASSERT_NEAR(38.9554377, *pdw.latest_mobility_operation_msg_.start_latitude, 0.00000001);
    ASSERT_NEAR(-77.1481983, *pdw.latest_mobility_operation_msg_.dest_longitude, 0.00000001);
    ASSERT_NEAR(38.9550038, *pdw.latest_mobility_operation_msg_.dest_latitude, 0.00000001);
    ASSERT_EQ("32", *pdw.latest_mobility_operation_msg_.current_action_id);

    // Create a MobilityOperationConstPtr with a cmv_id that is not intended for this specific vehicle
    carma_v2x_msgs::msg::MobilityOperation mobility_operation_msg2;
    mobility_operation_msg2.strategy = "carma/port_drayage";
    mobility_operation_msg2.strategy_params = "{ \"cmv_id\": \"444\", \"cargo_id\": \"567\", \"location\"\
        : { \"latitude\": \"48.9554377\", \"longitude\": \"-67.1503421\" }, \"destination\": { \"latitude\"\
        : \"48.9550038\", \"longitude\": \"-57.1481983\" }, \"operation\": \"PICKUP\", \"action_id\"\
        : \"44\" }";
    std::unique_ptr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_msg_ptr2 = std::make_unique<carma_v2x_msgs::msg::MobilityOperation>(mobility_operation_msg2);
    pdw.onInboundMobilityOperation(std::move(mobility_operation_msg_ptr2));

    // Check that the contents of the received message was not parsed and stored since it was not intended for this CMV
    ASSERT_EQ("321", *pdw.latest_mobility_operation_msg_.cargo_id);
    ASSERT_EQ("PICKUP", pdw.latest_mobility_operation_msg_.operation);
    ASSERT_NEAR(-77.1503421, *pdw.latest_mobility_operation_msg_.start_longitude, 0.00000001);
    ASSERT_NEAR(38.9554377, *pdw.latest_mobility_operation_msg_.start_latitude, 0.00000001);
    ASSERT_NEAR(-77.1481983, *pdw.latest_mobility_operation_msg_.dest_longitude, 0.00000001);
    ASSERT_NEAR(38.9550038, *pdw.latest_mobility_operation_msg_.dest_latitude, 0.00000001);
    ASSERT_EQ("32", *pdw.latest_mobility_operation_msg_.current_action_id);

    // Test composeArrivalMessage for when CMV has arrived at the Loading Area
    
    // Set the pdw's map projector and its current pose
    std::string base_proj = "+proj=tmerc +lat_0=38.95622708 +lon_0=-77.15066142 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    std_msgs::msg::String georeference_msg;
    georeference_msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> georeference_msg_ptr = std::make_unique<std_msgs::msg::String>(georeference_msg);
    pdw.onNewGeoreference(std::move(georeference_msg_ptr));

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
    pdw.onNewPose(std::move(pose_msg_ptr)); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Loading Area
    carma_v2x_msgs::msg::MobilityOperation msg = pdw.composeArrivalMessage();
    std::istringstream strstream(msg.strategy_params);
    using boost::property_tree::ptree;
    ptree pt;
    boost::property_tree::json_parser::read_json(strstream, pt);

    std::string cmv_id = pt.get<std::string>("cmv_id");
    std::string cargo_id = pt.get<std::string>("cargo_id");
    bool has_cargo = pt.get<bool>("cargo");
    std::string action_id = pt.get<std::string>("action_id");
    std::string operation = pt.get<std::string>("operation");
    double vehicle_longitude = pt.get<double>("location.longitude");
    double vehicle_latitude = pt.get<double>("location.latitude");

    // Verify the contents of the broadcasted message
    ASSERT_EQ("carma/port_drayage", msg.strategy);
    ASSERT_EQ("123", msg.m_header.sender_id);
    ASSERT_FALSE(msg.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_FALSE(has_cargo);
    ASSERT_EQ("321", cargo_id); 
    ASSERT_EQ("PICKUP", operation);
    ASSERT_EQ("32", action_id);
    ASSERT_NEAR(38.95622708, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.15066142, vehicle_longitude, 0.001);

    // Create an "EXIT_STAGING_AREA" MobilityOperationConstPtr for pdw
    carma_v2x_msgs::msg::MobilityOperation mobility_operation_msg3;
    mobility_operation_msg3.strategy = "carma/port_drayage";
    mobility_operation_msg3.strategy_params = "{ \"cmv_id\": \"123\", \"destination\": { \"latitude\"\
        : \"38.9103493\", \"longitude\": \"-77.1499283\" }, \"operation\": \"EXIT_STAGING_AREA\", \"action_id\"\
        : \"34\" }";
    std::unique_ptr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_msg_ptr3 = std::make_unique<carma_v2x_msgs::msg::MobilityOperation>(mobility_operation_msg3);
    pdw.onInboundMobilityOperation(std::move(mobility_operation_msg_ptr3));

    // Test composeArrivalMessage for when CMV has arrived at the Staging Area Exit

    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.9103493 +lon_0=-77.1499283 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> georeference_msg_ptr2 = std::make_unique<std_msgs::msg::String>(georeference_msg);
    pdw.onNewGeoreference(std::move(georeference_msg_ptr2));

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr2 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
    pdw.onNewPose(std::move(pose_msg_ptr2)); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Staging Area Exit
    carma_v2x_msgs::msg::MobilityOperation msg2 = pdw.composeArrivalMessage();
    std::istringstream strstream2(msg2.strategy_params);
    ptree pt2;
    boost::property_tree::json_parser::read_json(strstream2, pt2);

    cmv_id = pt2.get<std::string>("cmv_id");
    cargo_id = pt2.get<std::string>("cargo_id");
    has_cargo = pt2.get<bool>("cargo");
    action_id = pt2.get<std::string>("action_id");
    operation = pt2.get<std::string>("operation");
    vehicle_longitude = pt2.get<double>("location.longitude");
    vehicle_latitude = pt2.get<double>("location.latitude");

    // Verify the contents of the broadcasted message
    ASSERT_EQ("carma/port_drayage", msg2.strategy);
    ASSERT_EQ("123", msg2.m_header.sender_id);
    ASSERT_FALSE(msg2.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("321", cargo_id);
    ASSERT_EQ("EXIT_STAGING_AREA", operation);
    ASSERT_EQ("34", action_id);
    ASSERT_NEAR(38.9103493, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.1499283, vehicle_longitude, 0.001);

    // Create an "ENTER_PORT" MobilityOperationConstPtr for pdw
    carma_v2x_msgs::msg::MobilityOperation mobility_operation_msg4;
    mobility_operation_msg4.strategy = "carma/port_drayage";
    mobility_operation_msg4.strategy_params = "{ \"cmv_id\": \"123\", \"destination\": { \"latitude\"\
        : \"38.9199993\", \"longitude\": \"-77.1434283\" }, \"operation\": \"ENTER_PORT\", \"action_id\"\
        : \"36\" }";
    std::unique_ptr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_msg_ptr4 = std::make_unique<carma_v2x_msgs::msg::MobilityOperation>(mobility_operation_msg4);
    pdw.onInboundMobilityOperation(std::move(mobility_operation_msg_ptr4));

    // Test composeArrivalMessage for when CMV has arrived at the Port Entrance

    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.9199993 +lon_0=-77.1434283 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> georeference_msg_ptr3 = std::make_unique<std_msgs::msg::String>(georeference_msg);
    pdw.onNewGeoreference(std::move(georeference_msg_ptr3));

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr3 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
    pdw.onNewPose(std::move(pose_msg_ptr3)); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    carma_v2x_msgs::msg::MobilityOperation msg3 = pdw.composeArrivalMessage();
    std::istringstream strstream3(msg3.strategy_params);
    ptree pt3;
    boost::property_tree::json_parser::read_json(strstream3, pt3);

    cmv_id = pt3.get<std::string>("cmv_id");
    has_cargo = pt3.get<bool>("cargo");
    cargo_id = pt3.get<std::string>("cargo_id");
    action_id = pt3.get<std::string>("action_id");
    operation = pt3.get<std::string>("operation");
    vehicle_longitude = pt3.get<double>("location.longitude");
    vehicle_latitude = pt3.get<double>("location.latitude");

    // Verify the contents of the broadcasted message
    ASSERT_EQ("carma/port_drayage", msg3.strategy);
    ASSERT_EQ("123", msg3.m_header.sender_id);
    ASSERT_FALSE(msg3.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("321", cargo_id);
    ASSERT_EQ("ENTER_PORT", operation);
    ASSERT_EQ("36", action_id);
    ASSERT_NEAR(38.9199993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.1434283, vehicle_longitude, 0.001);

    // Create a "DROPOFF" MobilityOperationConstPtr for pdw
    carma_v2x_msgs::msg::MobilityOperation mobility_operation_msg5;
    mobility_operation_msg5.strategy = "carma/port_drayage";
    mobility_operation_msg5.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"321\", \"destination\": { \"latitude\"\
        : \"38.34259993\", \"longitude\": \"-77.1224283\" }, \"operation\": \"DROPOFF\", \"action_id\"\
        : \"37\" }";
    std::unique_ptr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_msg_ptr5 = std::make_unique<carma_v2x_msgs::msg::MobilityOperation>(mobility_operation_msg5);
    pdw.onInboundMobilityOperation(std::move(mobility_operation_msg_ptr5));

    // Test composeArrivalMessage for when CMV has arrived at the Dropoff location

    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.34259993 +lon_0=-77.1224283 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> georeference_msg_ptr4 = std::make_unique<std_msgs::msg::String>(georeference_msg);
    pdw.onNewGeoreference(std::move(georeference_msg_ptr4));

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr4 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
    pdw.onNewPose(std::move(pose_msg_ptr4)); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    carma_v2x_msgs::msg::MobilityOperation msg4 = pdw.composeArrivalMessage();
    std::istringstream strstream4(msg4.strategy_params);
    ptree pt4;
    boost::property_tree::json_parser::read_json(strstream4, pt4);

    cmv_id = pt4.get<std::string>("cmv_id");
    cargo_id = pt4.get<std::string>("cargo_id");
    has_cargo = pt4.get<bool>("cargo");
    action_id = pt4.get<std::string>("action_id");
    operation = pt4.get<std::string>("operation");
    vehicle_longitude = pt4.get<double>("location.longitude");
    vehicle_latitude = pt4.get<double>("location.latitude");

    // Verify the contents of the broadcasted message
    ASSERT_EQ("carma/port_drayage", msg4.strategy);
    ASSERT_EQ("123", msg4.m_header.sender_id);
    ASSERT_FALSE(msg4.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("321", cargo_id);
    ASSERT_EQ("DROPOFF", operation);
    ASSERT_EQ("37", action_id);
    ASSERT_NEAR(38.34259993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.1224283, vehicle_longitude, 0.001);

    // Create a "PICKUP" MobilityOperationConstPtr for pdw
    carma_v2x_msgs::msg::MobilityOperation mobility_operation_msg6;
    mobility_operation_msg6.strategy = "carma/port_drayage";
    mobility_operation_msg6.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"422\", \"destination\": { \"latitude\"\
        : \"38.3119993\", \"longitude\": \"-77.2314283\" }, \"operation\": \"PICKUP\", \"action_id\"\
        : \"38\" }";
    std::unique_ptr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_msg_ptr6 = std::make_unique<carma_v2x_msgs::msg::MobilityOperation>(mobility_operation_msg6);
    pdw.onInboundMobilityOperation(std::move(mobility_operation_msg_ptr6));

    // Test composeArrivalMessage for when CMV has arrived at the Pickup location
    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.3119993 +lon_0=-77.2314283 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> georeference_msg_ptr5 = std::make_unique<std_msgs::msg::String>(georeference_msg);
    pdw.onNewGeoreference(std::move(georeference_msg_ptr5));

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr5 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
    pdw.onNewPose(std::move(pose_msg_ptr5)); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    carma_v2x_msgs::msg::MobilityOperation msg5 = pdw.composeArrivalMessage();
    std::istringstream strstream5(msg5.strategy_params);
    ptree pt5;
    boost::property_tree::json_parser::read_json(strstream5, pt5);

    cmv_id = pt5.get<std::string>("cmv_id");
    cargo_id = pt5.get<std::string>("cargo_id");
    has_cargo = pt5.get<bool>("cargo");
    action_id = pt5.get<std::string>("action_id");
    operation = pt5.get<std::string>("operation");
    vehicle_longitude = pt5.get<double>("location.longitude");
    vehicle_latitude = pt5.get<double>("location.latitude");

    // Verify the contents of the broadcasted message
    ASSERT_EQ("carma/port_drayage", msg5.strategy);
    ASSERT_EQ("123", msg5.m_header.sender_id);
    ASSERT_FALSE(msg5.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_FALSE(has_cargo);
    ASSERT_EQ("422", cargo_id);
    ASSERT_EQ("PICKUP", operation);
    ASSERT_EQ("38", action_id);
    ASSERT_NEAR(38.3119993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.2314283, vehicle_longitude, 0.001);

    // Create an "PORT_CHECKPOINT" MobilityOperationConstPtr for pdw
    carma_v2x_msgs::msg::MobilityOperation mobility_operation_msg7;
    mobility_operation_msg7.strategy = "carma/port_drayage";
    mobility_operation_msg7.strategy_params = "{ \"cmv_id\": \"123\", \"destination\": { \"latitude\"\
        : \"38.3339993\", \"longitude\": \"-77.2594283\" }, \"operation\": \"PORT_CHECKPOINT\", \"action_id\"\
        : \"39\" }";
    std::unique_ptr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_msg_ptr7(new carma_v2x_msgs::msg::MobilityOperation(mobility_operation_msg7));
    pdw.onInboundMobilityOperation(std::move(mobility_operation_msg_ptr7));

    // Test composeArrivalMessage for when CMV has arrived at the Port Checkpoint
    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.3339993 +lon_0=-77.2594283 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> georeference_msg_ptr6 = std::make_unique<std_msgs::msg::String>(georeference_msg);
    pdw.onNewGeoreference(std::move(georeference_msg_ptr6));

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr6 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
    pdw.onNewPose(std::move(pose_msg_ptr6)); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    carma_v2x_msgs::msg::MobilityOperation msg6 = pdw.composeArrivalMessage();
    std::istringstream strstream6(msg6.strategy_params);
    ptree pt6;
    boost::property_tree::json_parser::read_json(strstream6, pt6);

    cmv_id = pt6.get<std::string>("cmv_id");
    cargo_id = pt6.get<std::string>("cargo_id");
    has_cargo = pt6.get<bool>("cargo");
    action_id = pt6.get<std::string>("action_id");
    operation = pt6.get<std::string>("operation");
    vehicle_longitude = pt6.get<double>("location.longitude");
    vehicle_latitude = pt6.get<double>("location.latitude");

    // Verify the contents of the broadcasted message
    ASSERT_EQ("carma/port_drayage", msg6.strategy);
    ASSERT_EQ("123", msg6.m_header.sender_id);
    ASSERT_FALSE(msg6.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("422", cargo_id);
    ASSERT_EQ("PORT_CHECKPOINT", operation);
    ASSERT_EQ("39", action_id);
    ASSERT_NEAR(38.3339993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.2594283, vehicle_longitude, 0.001);

    // Create a "HOLDING_AREA" MobilityOperationConstPtr for pdw
    carma_v2x_msgs::msg::MobilityOperation mobility_operation_msg8;
    mobility_operation_msg8.strategy = "carma/port_drayage";
    mobility_operation_msg8.strategy_params = "{ \"cmv_id\": \"123\", \"destination\": { \"latitude\"\
        : \"38.4139993\", \"longitude\": \"-77.2595583\" }, \"operation\": \"HOLDING_AREA\", \"action_id\"\
        : \"40\" }";
    std::unique_ptr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_msg_ptr8 = std::make_unique<carma_v2x_msgs::msg::MobilityOperation>(mobility_operation_msg8);
    pdw.onInboundMobilityOperation(std::move(mobility_operation_msg_ptr8));

    // Test composeArrivalMessage for when CMV has arrived at the Holding Area
    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.4139993 +lon_0=-77.2595583 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> georeference_msg_ptr7 = std::make_unique<std_msgs::msg::String>(georeference_msg);
    pdw.onNewGeoreference(std::move(georeference_msg_ptr7));

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr7 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
    pdw.onNewPose(std::move(pose_msg_ptr7)); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    carma_v2x_msgs::msg::MobilityOperation msg7 = pdw.composeArrivalMessage();
    std::istringstream strstream7(msg7.strategy_params);
    ptree pt7;
    boost::property_tree::json_parser::read_json(strstream7, pt7);

    cmv_id = pt7.get<std::string>("cmv_id");
    cargo_id = pt7.get<std::string>("cargo_id");
    has_cargo = pt7.get<bool>("cargo");
    action_id = pt7.get<std::string>("action_id");
    operation = pt7.get<std::string>("operation");
    vehicle_longitude = pt7.get<double>("location.longitude");
    vehicle_latitude = pt7.get<double>("location.latitude");

    // Verify the contents of the broadcasted message
    ASSERT_EQ("carma/port_drayage", msg7.strategy);
    ASSERT_EQ("123", msg7.m_header.sender_id);
    ASSERT_FALSE(msg7.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("422", cargo_id);
    ASSERT_EQ("HOLDING_AREA", operation);
    ASSERT_EQ("40", action_id);
    ASSERT_NEAR(38.4139993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.2595583, vehicle_longitude, 0.001);

   // Create an "EXIT_PORT" MobilityOperationConstPtr for pdw
    carma_v2x_msgs::msg::MobilityOperation mobility_operation_msg9;
    mobility_operation_msg9.strategy = "carma/port_drayage";
    mobility_operation_msg9.strategy_params = "{ \"cmv_id\": \"123\", \"destination\": { \"latitude\"\
        : \"38.6639993\", \"longitude\": \"-77.8395583\" }, \"operation\": \"EXIT_PORT\", \"action_id\"\
        : \"41\" }";
    std::unique_ptr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_msg_ptr9 = std::make_unique<carma_v2x_msgs::msg::MobilityOperation>(mobility_operation_msg9);
    pdw.onInboundMobilityOperation(std::move(mobility_operation_msg_ptr9));

    // Test composeArrivalMessage for when CMV has arrived at the Port Exit
    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.6639993 +lon_0=-77.8395583 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> georeference_msg_ptr8 = std::make_unique<std_msgs::msg::String>(georeference_msg);
    pdw.onNewGeoreference(std::move(georeference_msg_ptr8));

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr8 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
    pdw.onNewPose(std::move(pose_msg_ptr8)); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    carma_v2x_msgs::msg::MobilityOperation msg8 = pdw.composeArrivalMessage();
    std::istringstream strstream8(msg8.strategy_params);
    ptree pt8;
    boost::property_tree::json_parser::read_json(strstream8, pt8);

    cmv_id = pt8.get<std::string>("cmv_id");
    cargo_id = pt8.get<std::string>("cargo_id");
    has_cargo = pt8.get<bool>("cargo");
    action_id = pt8.get<std::string>("action_id");
    operation = pt8.get<std::string>("operation");
    vehicle_longitude = pt8.get<double>("location.longitude");
    vehicle_latitude = pt8.get<double>("location.latitude");

    // Verify the contents of the broadcasted message
    ASSERT_EQ("carma/port_drayage", msg8.strategy);
    ASSERT_EQ("123", msg8.m_header.sender_id);
    ASSERT_FALSE(msg8.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("422", cargo_id);
    ASSERT_EQ("EXIT_PORT", operation);
    ASSERT_EQ("41", action_id);
    ASSERT_NEAR(38.6639993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.8395583, vehicle_longitude, 0.001);

   // Create an "ENTER_STAGING_AREA" MobilityOperationConstPtr for pdw
    carma_v2x_msgs::msg::MobilityOperation mobility_operation_msg10;
    mobility_operation_msg10.strategy = "carma/port_drayage";
    mobility_operation_msg10.strategy_params = "{ \"cmv_id\": \"123\", \"destination\": { \"latitude\"\
        : \"38.6889993\", \"longitude\": \"-77.8124583\" }, \"operation\": \"ENTER_STAGING_AREA\", \"action_id\"\
        : \"42\" }";
    std::unique_ptr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_msg_ptr10(new carma_v2x_msgs::msg::MobilityOperation(mobility_operation_msg10));
    pdw.onInboundMobilityOperation(std::move(mobility_operation_msg_ptr10));

    // Test composeArrivalMessage for when CMV has arrived at the Staging Area Entrance
    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.6889993 +lon_0=-77.8124583 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> georeference_msg_ptr9 = std::make_unique<std_msgs::msg::String>(georeference_msg);
    pdw.onNewGeoreference(std::move(georeference_msg_ptr9));

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr9 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
    pdw.onNewPose(std::move(pose_msg_ptr9)); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    carma_v2x_msgs::msg::MobilityOperation msg9 = pdw.composeArrivalMessage();
    std::istringstream strstream9(msg9.strategy_params);
    ptree pt9;
    boost::property_tree::json_parser::read_json(strstream9, pt9);

    cmv_id = pt9.get<std::string>("cmv_id");
    cargo_id = pt9.get<std::string>("cargo_id");
    has_cargo = pt9.get<bool>("cargo");
    action_id = pt9.get<std::string>("action_id");
    operation = pt9.get<std::string>("operation");
    vehicle_longitude = pt9.get<double>("location.longitude");
    vehicle_latitude = pt9.get<double>("location.latitude");

    // Verify the contents of the broadcasted message
    ASSERT_EQ("carma/port_drayage", msg9.strategy);
    ASSERT_EQ("123", msg9.m_header.sender_id);
    ASSERT_FALSE(msg9.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("422", cargo_id);
    ASSERT_EQ("ENTER_STAGING_AREA", operation);
    ASSERT_EQ("42", action_id);
    ASSERT_NEAR(38.6889993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.8124583, vehicle_longitude, 0.001);
}

TEST(PortDrayageTest, testComposeUIInstructions)
{
    // Create PortDrayageWorker object with _cmv_id of "123" and no cargo
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::Clock::SharedPtr clock = node->get_clock();
    port_drayage_plugin::PortDrayageWorker pdw(node->get_node_logging_interface(),
        clock,
        [](carma_v2x_msgs::msg::MobilityOperation){}, 
        [](carma_msgs::msg::UIInstructions){},
        [](std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request>){return true;} 
    );

    pdw.setVehicleID("123");
    pdw.setCargoID("");
    pdw.setEnablePortDrayageFlag(true);
    pdw.setStartingAtStagingAreaFlag(true);

    // First 'operation' is for 'PICKUP'. Verify the created UI Instructions message:
    std::string current_operation = "PICKUP";
    std::string previous_operation = "";

    carma_msgs::msg::UIInstructions ui_instructions_msg = pdw.composeUIInstructions(current_operation, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "A new Port Drayage route with operation type 'PICKUP' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, carma_msgs::msg::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");

    // Second received MobilityOperation message is for a 'DROPOFF' operation. The previous 'PICKUP' operation has been completed.
    current_operation = "DROPOFF";
    previous_operation = "PICKUP";

    ui_instructions_msg = pdw.composeUIInstructions(current_operation, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "The pickup action was completed successfully. A new Port Drayage route with operation type 'DROPOFF' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, carma_msgs::msg::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");

    // Third received MobilityOperation message is for a 'EXIT_STAGING_AREA' operation. The previous 'DROPOFF' operation has been completed.
    current_operation = "EXIT_STAGING_AREA";
    previous_operation = "DROPOFF";

    ui_instructions_msg = pdw.composeUIInstructions(current_operation, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "The dropoff action was completed successfully. A new Port Drayage route with operation type 'EXIT_STAGING_AREA' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, carma_msgs::msg::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");
    
    // Fourth received MobilityOperation message is for a 'ENTER_PORT' operation. The previous 'EXIT_STAGING_AREA' operation has been completed.
    current_operation = "ENTER_PORT";
    previous_operation = "EXIT_STAGING_AREA";

    ui_instructions_msg = pdw.composeUIInstructions(current_operation, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "A new Port Drayage route with operation type 'ENTER_PORT' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, carma_msgs::msg::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");

    // Fifth received MobilityOperation message is for a 'ENTER_PORT' operation. The previous 'EXIT_STAGING_AREA' operation has been completed.
    current_operation = "EXIT_PORT";
    previous_operation = "HOLDING_AREA";

    ui_instructions_msg = pdw.composeUIInstructions(current_operation, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "The inspection was completed successfully. A new Port Drayage route with operation type 'EXIT_PORT' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, carma_msgs::msg::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");
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