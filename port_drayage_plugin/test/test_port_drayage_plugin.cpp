/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_wm/WMTestLibForGuidance.h>
#include <lanelet2_extension/regulatory_elements/StopRule.h>

TEST(PortDrayageTest, testComposeArrivalMessage)
{
    // Test arrival message for pdw initialized with cargo
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        123, // CMV ID 
        "321", // Cargo ID 
        "TEST_CARMA_HOST_ID", // Host ID
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){},
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;} 
    };

    // Set the pdw's map projector set its current pose
    std::string base_proj = "+proj=tmerc +lat_0=38.95622708 +lon_0=-77.15066142 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    std_msgs::String georeference_msg;
    georeference_msg.data = base_proj;
    std_msgs::StringConstPtr georeference_msg_ptr(new std_msgs::String(georeference_msg));
    pdw.on_new_georeference(georeference_msg_ptr);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    geometry_msgs::PoseStampedConstPtr pose_msg_ptr(new geometry_msgs::PoseStamped(pose_msg));
    pdw.on_new_pose(pose_msg_ptr); // Sets the host vehicle's current gps lat/lon position

    cav_msgs::MobilityOperation msg = pdw.compose_arrival_message();

    ASSERT_EQ("carma/port_drayage", msg.strategy);
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg.header.sender_id);
    ASSERT_FALSE(msg.strategy_params.empty());

    std::istringstream strstream(msg.strategy_params);
    using boost::property_tree::ptree;
    ptree pt;
    boost::property_tree::json_parser::read_json(strstream, pt);

    unsigned long cmv_id = pt.get<unsigned long>("cmv_id");
    std::string operation = pt.get<std::string>("operation");
    bool has_cargo = pt.get<bool>("cargo");
    double vehicle_longitude = pt.get<double>("location.longitude");
    double vehicle_latitude = pt.get<double>("location.latitude");

    ASSERT_EQ(123, cmv_id);
    ASSERT_EQ("ENTER_STAGING_AREA", operation);
    ASSERT_EQ(pt.count("action_id"), 0);
    ASSERT_TRUE(has_cargo);
    ASSERT_NEAR(38.95622708, vehicle_latitude, 0.00000001);
    ASSERT_NEAR(-77.15066142, vehicle_longitude, 0.00000001);

    // Test arrival message for pdw initialized without cargo
    port_drayage_plugin::PortDrayageWorker pdw2{
        123, // CMV ID 
        "", // Cargo ID 
        "TEST_CARMA_HOST_ID", 
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){},
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;} 
    };

    pdw2.on_new_georeference(georeference_msg_ptr);
    pdw2.on_new_pose(pose_msg_ptr);

    cav_msgs::MobilityOperation msg2 = pdw2.compose_arrival_message();

    ASSERT_EQ("carma/port_drayage", msg2.strategy);
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg2.header.sender_id);
    ASSERT_FALSE(msg2.strategy_params.empty());

    std::istringstream strstream2(msg2.strategy_params);
    ptree pt2;
    boost::property_tree::json_parser::read_json(strstream2, pt2);

    unsigned long cmv_id2 = pt2.get<unsigned long>("cmv_id");
    std::string operation2 = pt2.get<std::string>("operation");
    bool has_cargo2 = pt2.get<bool>("cargo");
    double vehicle_longitude2 = pt2.get<double>("location.longitude");
    double vehicle_latitude2 = pt2.get<double>("location.latitude");

    ASSERT_EQ(123, cmv_id2);
    ASSERT_EQ(0, pt2.count("cargo_id")); // Broadcasted arrival message should not include cargo_id since vehicle is not carrying cargo
    ASSERT_EQ("ENTER_STAGING_AREA", operation2);
    ASSERT_FALSE(has_cargo2); // False since vehicle is not currently carrying cargo
    ASSERT_NEAR(38.95622708, vehicle_latitude2, 0.00000001);
    ASSERT_NEAR(-77.15066142, vehicle_longitude2, 0.00000001);
}

TEST(PortDrayageTest, testCheckStop1)
{
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        123, // CMV ID
        "321", // Cargo ID
        "TEST_CARMA_HOST_ID", 
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){},
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;}
    };

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 0;
    geometry_msgs::TwistStampedConstPtr twistptr{ new geometry_msgs::TwistStamped{twist}};
    pdw.set_current_speed(twistptr);

    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver mvr;
    mvr.type = cav_msgs::Maneuver::STOP_AND_WAIT;
    mvr.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "port_drayage_plugin";
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
        123, // CMV ID 
        "321", // Cargo ID
        "TEST_CARMA_HOST_ID", 
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){},
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;}
    };

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
        123, // CMV ID
        "321", // Cargo ID 
        "TEST_CARMA_HOST_ID", 
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){}, 
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;}
    };

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
        123, // CMV ID
        "321", // Cargo ID 
        "TEST_CARMA_HOST_ID", 
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){}, 
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;}
    };

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

// Test State Machine flow strictly from PortDrayageEvents
TEST(PortDrayageTest, testStateMachine1)
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

// Test communication between PortDrayageWorker and PortDrayageStateMachine for State Machine flow
TEST(PortDrayageTest, testPortDrayageStateMachine2)
{
    // Create PortDrayageWorker object with _cmv_id of 123 and no cargo
    //std::function<void(cav_msgs::MobilityOperation)> fun = [](cav_msgs::MobilityOperation){return;};
    port_drayage_plugin::PortDrayageWorker pdw{
        123, // CMV ID 
        "", // Cargo ID; empty string indicates CMV begins without no cargo
        "TEST_CARMA_HOST_ID", 
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){}, 
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;}
    };

    // State Machine should begin in INACTIVE state
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::INACTIVE, pdw.get_port_drayage_state());

    // State Machine should transition to EN_ROUTE after guidance state is first engaged
    cav_msgs::GuidanceState guidance_state;
    guidance_state.state = cav_msgs::GuidanceState::ENGAGED;
    cav_msgs::GuidanceStateConstPtr guidance_state_pointer(new cav_msgs::GuidanceState(guidance_state));
    pdw.on_guidance_state(guidance_state_pointer);

    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE, pdw.get_port_drayage_state());

    // State Machine should transition to AWAITING_DIRECTION if a 'ROUTE_LOADED' event occurs immediately after a 'ROUTE_COMPLETED' event
    cav_msgs::RouteEvent route_event_1;
    route_event_1.event = cav_msgs::RouteEvent::ROUTE_COMPLETED; 
    cav_msgs::RouteEventConstPtr route_event_pointer_1(new cav_msgs::RouteEvent(route_event_1));
    pdw.on_route_event(route_event_pointer_1); // PortDrayageWorker receives RouteEvent indicating route has been completed

    cav_msgs::RouteEvent route_event_2;
    route_event_2.event = cav_msgs::RouteEvent::ROUTE_LOADED; 
    cav_msgs::RouteEventConstPtr route_event_pointer_2(new cav_msgs::RouteEvent(route_event_2));
    pdw.on_route_event(route_event_pointer_2); // PortDrayageWorker receives RouteEvent indicating the previously completed route is no longer active
    
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::AWAITING_DIRECTION, pdw.get_port_drayage_state());

    // State Machine should transition to 'EN_ROUTE' if a new port drayage MobilityOperation message is received

    // Create a MobilityOperationConstPtr with a cmv_id that is intended for this specific vehicle
    // Note: The strategy_params using the schema for messages of this type that have strategy "carma/port_drayage"
    cav_msgs::MobilityOperation mobility_operation_msg;
    mobility_operation_msg.strategy = "carma/port_drayage";
    mobility_operation_msg.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"321\", \"location\"\
        : { \"latitude\": \"38.9554377\", \"longitude\": \"-77.1503421\" }, \"destination\": { \"latitude\"\
        : \"38.9550038\", \"longitude\": \"-77.1481983\" }, \"operation\": \"PICKUP\", \"action_id\"\
        : \"32\"}";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr(new cav_msgs::MobilityOperation(mobility_operation_msg));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr); 

    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE, pdw.get_port_drayage_state());

    // State Machine should transition to 'AWAITING_DIRECTION' again if a ROUTE_COMPLETED event occurs while the vehicle is stopped
    pdw.on_route_event(route_event_pointer_1); // PortDrayageWorker receives RouteEvent indicating route has been completed
    pdw.on_route_event(route_event_pointer_2); // PortDrayageWorker receives RouteEvent indicating the previously completed route is no longer active
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::AWAITING_DIRECTION, pdw.get_port_drayage_state());
}

TEST(PortDrayageTest, testEstimateDistanceToStop)
{
    ASSERT_EQ(port_drayage_plugin::estimate_distance_to_stop(10.0, 1.0), 50.0);
}

TEST(PortDrayageTest, testEstimateTimeToStop)
{
    ASSERT_EQ(port_drayage_plugin::estimate_time_to_stop(10.0, 1.0), 20.0);
}

TEST(PortDrayageTest, testPlanManeuverCb)
{
    cav_srvs::PlanManeuversRequest req;
    cav_srvs::PlanManeuversResponse resp;

    port_drayage_plugin::PortDrayagePlugin pdp{nullptr, nullptr};
    pdp.declaration = 1;

    ASSERT_EQ(pdp.plan_maneuver_cb(req,resp),false);

    geometry_msgs::Twist _cur_speed;
    _cur_speed.linear.x = 1;
    _cur_speed.linear.y = 2;

    pdp._cur_speed = _cur_speed;

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;    

    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.pose = pose;

    std::shared_ptr<geometry_msgs::PoseStamped> curr_pose_ = std::make_shared<geometry_msgs::PoseStamped>(pose_stamped);
    pdp.curr_pose_ = curr_pose_;

    std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();

    auto pl1 = carma_wm::test::getPoint(0, 0, 0);
    auto pl2 = carma_wm::test::getPoint(0, 1, 0);
    auto pl3 = carma_wm::test::getPoint(0, 2, 0);
    auto pr1 = carma_wm::test::getPoint(1, 0, 0);
    auto pr2 = carma_wm::test::getPoint(1, 1, 0);
    auto pr3 = carma_wm::test::getPoint(1, 2, 0);

    std::vector<lanelet::Point3d> left_1 = { pl1, pl2 };
    std::vector<lanelet::Point3d> right_1 = { pr1, pr2 };
    auto ll_1 = carma_wm::test::getLanelet(left_1, right_1, lanelet::AttributeValueString::SolidSolid, lanelet::AttributeValueString::Dashed);

    std::vector<lanelet::Point3d> left_2 = { pl2, pl3 };
    std::vector<lanelet::Point3d> right_2 = { pr2, pr3 };
    auto ll_2 = carma_wm::test::getLanelet(left_2, right_2);

    // 1. Confirm all pointers are false (done above)
    // Ensure that none of the returned pointers are valid if the map has not been set
    ASSERT_FALSE((bool)cmw->getMap());
    ASSERT_FALSE((bool)cmw->getRoute());
    ASSERT_FALSE((bool)cmw->getMapRoutingGraph());


    lanelet::Id stop_line_id = lanelet::utils::getId();
    lanelet::LineString3d virtual_stop_line(stop_line_id, {pl2, pr2});
    // Creat passing control line for solid dashed line
    std::shared_ptr<lanelet::StopRule> stop_and_wait(new lanelet::StopRule(lanelet::StopRule::buildData(
    lanelet::utils::getId(), { virtual_stop_line }, { lanelet::Participants::Vehicle })));

    ll_1.addRegulatoryElement(stop_and_wait);


    // 2. Build map but do not assign
    // Create basic map and verify that the map and routing graph can be build, but the route remains false
    lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1, ll_2 }, {});
    
    // // traffic lights are created from a linestring that shows a traffic light and optionally a stop line.
    // lanelet::LineString3d stop_rule = lanelet::LineString3d(lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 1, 0, 0}, lanelet::Point3d{lanelet::utils::getId(), 1, 1, 0}, lanelet::Point3d{lanelet::utils::getId(), 1, 2, 0}});
    // trafficLight.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::TrafficLight;
    // // this creates our traffic light. Regelems are passed around as shared pointers.
    // lanelet::RegulatoryElementPtr trafficLightRegelem = lanelet::TrafficLight::make(lanelet::utils::getId(), {}, {trafficLight});

    // // 3. Build routing graph but do not assign
    // // Build routing graph from map
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::VehicleCar);

    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

    // 4. Generate route
    auto optional_route = map_graph->getRoute(ll_1, ll_2);
    ASSERT_TRUE((bool)optional_route);

    lanelet::routing::Route route = std::move(*optional_route);
    carma_wm::LaneletRoutePtr route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
    // 5. Try to set route without map and ensure it passes
    cmw->setRoute(route_ptr);
    // 6. getRoute is true but other pointers are false
    ASSERT_FALSE((bool)cmw->getMap());
    ASSERT_TRUE((bool)cmw->getRoute());
    ASSERT_FALSE((bool)cmw->getMapRoutingGraph());

    cmw->setMap(map);
    // 8. All pointers exist
    ASSERT_TRUE((bool)cmw->getMap());
    ASSERT_TRUE((bool)cmw->getRoute());
    ASSERT_TRUE((bool)cmw->getMapRoutingGraph());
    // 9. Call setRoute again to confirm no errors
    cmw->setRoute(route_ptr);
    // 10. All pointers exist
    ASSERT_TRUE((bool)cmw->getMap());
    ASSERT_TRUE((bool)cmw->getRoute());
    ASSERT_TRUE((bool)cmw->getMapRoutingGraph());

    pdp.wm_ = cmw;

    ASSERT_EQ(pdp.plan_maneuver_cb(req,resp),true);

    ros::Time time;
    time.sec = 1;
    time.nsec = 0;

    cav_msgs::Maneuver maneuver = pdp.compose_stop_and_wait_maneuver_message(1, 2, 1, 2, 1, time, 10);
    ASSERT_EQ(maneuver.stop_and_wait_maneuver.end_time,time + ros::Duration(15));

    maneuver = pdp.compose_lane_following_maneuver_message(1, 2, 1, 2, 1, time);
    ASSERT_NEAR(maneuver.lane_following_maneuver.end_time.sec, 1, 0.0001);
}

TEST(PortDrayageTest, testComposeSetActiveRouteRequest)
{
    // Create PortDrayageWorker object with _cmv_id of "123"
    port_drayage_plugin::PortDrayageWorker pdw{
        123, 
        "TEST_CARGO_ID", 
        "TEST_CARMA_HOST_ID", 
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){},
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;} 
    };

    // Create a MobilityOperationConstPtr with a cmv_id that is intended for this specific vehicle
    // Note: The strategy_params using the schema for messages of this type that have strategy "carma/port_drayage"
    cav_msgs::MobilityOperation mobility_operation_msg;
    mobility_operation_msg.strategy = "carma/port_drayage";
    mobility_operation_msg.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"321\", \"cargo\": \"false\", \"location\"\
        : { \"latitude\": \"38.9554377\", \"longitude\": \"-77.1503421\" }, \"destination\": { \"latitude\"\
        : \"38.9550038\", \"longitude\": \"-77.1481983\" }, \"operation\": \"MOVING_TO_LOADING_AREA\", \"action_id\"\
        : \"32\", \"next_action\": \"33\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr(new cav_msgs::MobilityOperation(mobility_operation_msg));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr); 

    // Verify the results of PortDrayageWorker's compose_set_active_route_request() method
    cav_srvs::SetActiveRoute route_req = pdw.compose_set_active_route_request(*pdw._latest_mobility_operation_msg.dest_latitude, *pdw._latest_mobility_operation_msg.dest_longitude);
    ASSERT_EQ(cav_srvs::SetActiveRouteRequest::DESTINATION_POINTS_ARRAY, route_req.request.choice);
    ASSERT_EQ(1, route_req.request.destination_points.size());
    ASSERT_EQ(38.9550038, route_req.request.destination_points[0].latitude);
    ASSERT_EQ(-77.1481983, route_req.request.destination_points[0].longitude);
    ASSERT_EQ(false, route_req.request.destination_points[0].elevation_exists);
}

TEST(PortDrayageTest, testInboundMobilityOperation)
{
    // Create PortDrayageWorker object with _cmv_id of 123 that is carrying cargo
    port_drayage_plugin::PortDrayageWorker pdw{
        123, // CMV ID 
        "123", // Cargo ID
        "TEST_CARMA_HOST_ID", 
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){},
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;}
    };

    // Create a "PICKUP" MobilityOperationConstPtr with a cmv_id that is intended for this specific vehicle
    // Note: The strategy_params using the schema for messages of this type that have strategy "carma/port_drayage"
    cav_msgs::MobilityOperation mobility_operation_msg;
    mobility_operation_msg.strategy = "carma/port_drayage";
    mobility_operation_msg.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"541\", \"location\"\
        : { \"latitude\": \"38.9554377\", \"longitude\": \"-77.1503421\" }, \"destination\": { \"latitude\"\
        : \"38.9550038\", \"longitude\": \"-77.1481983\" }, \"operation\": \"PICKUP\", \"action_id\"\
        : \"32\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr(new cav_msgs::MobilityOperation(mobility_operation_msg));
    
    // Exception should be thrown since vehicle was initialized with cargo, and its first received message requires it to pickup new cargo
    ASSERT_THROW(pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr), std::invalid_argument);

    // Create a new PortDrayageWorker object with _cmv_id of 123 and no cargo
    port_drayage_plugin::PortDrayageWorker pdw2{
        123, // CMV ID 
        "", // Cargo ID; empty string indicates it is not carrying cargo
        "TEST_CARMA_HOST_ID", 
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){},
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;}
    };

    // Create a "PICKUP" MobilityOperationConstPtr for the newly created PortDrayageWorker
    cav_msgs::MobilityOperation mobility_operation_msg2;
    mobility_operation_msg2.strategy = "carma/port_drayage";
    mobility_operation_msg2.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"321\", \"location\"\
        : { \"latitude\": \"38.9554377\", \"longitude\": \"-77.1503421\" }, \"destination\": { \"latitude\"\
        : \"38.9550038\", \"longitude\": \"-77.1481983\" }, \"operation\": \"PICKUP\", \"action_id\"\
        : \"32\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr2(new cav_msgs::MobilityOperation(mobility_operation_msg2));
    pdw2.on_inbound_mobility_operation(mobility_operation_msg_ptr2);
    
    // Check that the received message was parsed and stored correctly
    ASSERT_EQ("321", *pdw2._latest_mobility_operation_msg.cargo_id);
    ASSERT_EQ("PICKUP", pdw2._latest_mobility_operation_msg.operation);
    ASSERT_EQ(port_drayage_plugin::PortDrayageEvent::RECEIVED_NEW_DESTINATION, pdw2._latest_mobility_operation_msg.port_drayage_event_type);
    ASSERT_EQ(false, pdw2._latest_mobility_operation_msg.has_cargo);
    ASSERT_NEAR(-77.1503421, *pdw2._latest_mobility_operation_msg.start_longitude, 0.00000001);
    ASSERT_NEAR(38.9554377, *pdw2._latest_mobility_operation_msg.start_latitude, 0.00000001);
    ASSERT_NEAR(-77.1481983, *pdw2._latest_mobility_operation_msg.dest_longitude, 0.00000001);
    ASSERT_NEAR(38.9550038, *pdw2._latest_mobility_operation_msg.dest_latitude, 0.00000001);
    ASSERT_EQ("32", *pdw2._latest_mobility_operation_msg.current_action_id);

    // Create a MobilityOperationConstPtr with a cmv_id that is not intended for this specific vehicle
    cav_msgs::MobilityOperation mobility_operation_msg3;
    mobility_operation_msg3.strategy = "carma/port_drayage";
    mobility_operation_msg3.strategy_params = "{ \"cmv_id\": \"444\", \"cargo_id\": \"567\", \"location\"\
        : { \"latitude\": \"48.9554377\", \"longitude\": \"-67.1503421\" }, \"destination\": { \"latitude\"\
        : \"48.9550038\", \"longitude\": \"-57.1481983\" }, \"operation\": \"PICKUP\", \"action_id\"\
        : \"44\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr3(new cav_msgs::MobilityOperation(mobility_operation_msg3));
    pdw2.on_inbound_mobility_operation(mobility_operation_msg_ptr3);

    // Check that the contents of the received message was not parsed and stored since it was not intended for this CMV
    ASSERT_EQ("321", *pdw2._latest_mobility_operation_msg.cargo_id);
    ASSERT_EQ("PICKUP", pdw2._latest_mobility_operation_msg.operation);
    ASSERT_EQ(port_drayage_plugin::PortDrayageEvent::RECEIVED_NEW_DESTINATION, pdw2._latest_mobility_operation_msg.port_drayage_event_type);
    ASSERT_EQ(false, pdw2._latest_mobility_operation_msg.has_cargo);
    ASSERT_NEAR(-77.1503421, *pdw2._latest_mobility_operation_msg.start_longitude, 0.00000001);
    ASSERT_NEAR(38.9554377, *pdw2._latest_mobility_operation_msg.start_latitude, 0.00000001);
    ASSERT_NEAR(-77.1481983, *pdw2._latest_mobility_operation_msg.dest_longitude, 0.00000001);
    ASSERT_NEAR(38.9550038, *pdw2._latest_mobility_operation_msg.dest_latitude, 0.00000001);
    ASSERT_EQ("32", *pdw2._latest_mobility_operation_msg.current_action_id);

    // Test composeArrivalMessage for when CMV has arrived at the Loading Area
    
    // Set the pdw's map projector and its current pose
    std::string base_proj = "+proj=tmerc +lat_0=38.95622708 +lon_0=-77.15066142 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    std_msgs::String georeference_msg;
    georeference_msg.data = base_proj;
    std_msgs::StringConstPtr georeference_msg_ptr(new std_msgs::String(georeference_msg));
    pdw2.on_new_georeference(georeference_msg_ptr);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    geometry_msgs::PoseStampedConstPtr pose_msg_ptr(new geometry_msgs::PoseStamped(pose_msg));
    pdw2.on_new_pose(pose_msg_ptr); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Loading Area
    cav_msgs::MobilityOperation msg = pdw2.compose_arrival_message();
    std::istringstream strstream(msg.strategy_params);
    using boost::property_tree::ptree;
    ptree pt;
    boost::property_tree::json_parser::read_json(strstream, pt);

    unsigned long cmv_id = pt.get<unsigned long>("cmv_id");
    std::string cargo_id = pt.get<std::string>("cargo_id");
    bool has_cargo = pt.get<bool>("cargo");
    std::string action_id = pt.get<std::string>("action_id");
    std::string operation = pt.get<std::string>("operation");
    double vehicle_longitude = pt.get<double>("location.longitude");
    double vehicle_latitude = pt.get<double>("location.latitude");

    // Verify the contents of the broadcasted message
    ASSERT_EQ("carma/port_drayage", msg.strategy);
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg.header.sender_id);
    ASSERT_FALSE(msg.strategy_params.empty());
    ASSERT_EQ(123, cmv_id);
    ASSERT_EQ("321", cargo_id); 
    ASSERT_FALSE(has_cargo);
    ASSERT_EQ("PICKUP", operation);
    ASSERT_EQ("32", action_id);
    ASSERT_NEAR(38.95622708, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.15066142, vehicle_longitude, 0.001);
}

TEST(PortDrayageTest, testComposeUIInstructions)
{

    // Create PortDrayageWorker object with _cmv_id of "123"
    port_drayage_plugin::PortDrayageWorker pdw{
        123, 
        "", // Empty string indicates CMV is not carrying cargo 
        "TEST_CARMA_HOST_ID", 
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){},
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;} 
    };

    // First received MobilityOperation message is for a 'PICKUP' operation. Verify the created UI Instructions message:
    port_drayage_plugin::PortDrayageMobilityOperationMsg mob_op_msg;
    mob_op_msg.operation = "PICKUP";

    std::string previous_operation = "";

    cav_msgs::UIInstructions ui_instructions_msg = pdw.compose_ui_instructions(mob_op_msg, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "A new Port Drayage route with operation type 'PICKUP' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, cav_msgs::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");

    // Second received MobilityOperation message is for a 'DROPOFF' operation. The previous 'PICKUP' operation has been completed.
    mob_op_msg.operation = "DROPOFF";
    previous_operation = "PICKUP";

    ui_instructions_msg = pdw.compose_ui_instructions(mob_op_msg, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "The pickup action was completed successfully. A new Port Drayage route with operation type 'DROPOFF' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, cav_msgs::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");

    // Third received MobilityOperation message is for a 'PICKUP' operation. The previous 'DROPOFF' operation has been completed.
    mob_op_msg.operation = "PICKUP";
    previous_operation = "DROPOFF";

    ui_instructions_msg = pdw.compose_ui_instructions(mob_op_msg, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "The dropoff action was completed successfully. A new Port Drayage route with operation type 'PICKUP' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, cav_msgs::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");




    // Receive a Dropoff action

    // Receive some other location operation
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
