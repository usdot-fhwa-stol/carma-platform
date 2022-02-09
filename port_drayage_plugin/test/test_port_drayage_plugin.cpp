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
    // Test initial arrival message for pdw initialized with cargo with the Staging Area Entrance as its first destination
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        "DOT-11111", // CMV ID 
        "321", // Cargo ID 
        "TEST_CARMA_HOST_ID", // Host ID
        true, // Flag indicating CMV's first destination; 'true' is Staging Area Entrance, 'false' is Port Entrance
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

    // State Machine should transition to EN_ROUTE_TO_INITIAL_DESTINATION after guidance state is first engaged
    cav_msgs::GuidanceState guidance_state;
    guidance_state.state = cav_msgs::GuidanceState::ENGAGED;
    cav_msgs::GuidanceStateConstPtr guidance_state_pointer(new cav_msgs::GuidanceState(guidance_state));
    pdw.on_guidance_state(guidance_state_pointer);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    geometry_msgs::PoseStampedConstPtr pose_msg_ptr(new geometry_msgs::PoseStamped(pose_msg));
    pdw.on_new_pose(pose_msg_ptr); // Sets the host vehicle's current gps lat/lon position

    cav_msgs::MobilityOperation msg = pdw.compose_arrival_message();

    ASSERT_EQ("carma/port_drayage", msg.strategy);
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg.m_header.sender_id);
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

    // Test initial arrival message for pdw initialized without cargo with the Port Entrance as its first destination
    port_drayage_plugin::PortDrayageWorker pdw2{
        "123", // CMV ID 
        "", // Cargo ID 
        "TEST_CARMA_HOST_ID", 
        false, // Flag indicating CMV's first destination; 'true' is Staging Area Entrance, 'false' is Port Entrance
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){},
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;} 
    };

    pdw2.on_new_georeference(georeference_msg_ptr);
    pdw2.on_new_pose(pose_msg_ptr);
    pdw2.on_guidance_state(guidance_state_pointer);

    cav_msgs::MobilityOperation msg2 = pdw2.compose_arrival_message();

    ASSERT_EQ("carma/port_drayage", msg2.strategy);
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg2.m_header.sender_id);
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

TEST(PortDrayageTest, testCheckStop1)
{
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        "123", // CMV ID
        "321", // Cargo ID
        "TEST_CARMA_HOST_ID", 
        true, // Flag indicating CMV's first destination; 'true' is Staging Area Entrance, 'false' is Port Entrance
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
        "123", // CMV ID 
        "321", // Cargo ID
        "TEST_CARMA_HOST_ID", 
        true, // Flag indicating CMV's first destination; 'true' is Staging Area Entrance, 'false' is Port Entrance
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
        "123", // CMV ID
        "321", // Cargo ID 
        "TEST_CARMA_HOST_ID", 
        true, // Flag indicating CMV's first destination; 'true' is Staging Area Entrance, 'false' is Port Entrance
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
        "123", // CMV ID
        "321", // Cargo ID 
        "TEST_CARMA_HOST_ID", 
        true, // Flag indicating CMV's first destination; 'true' is Staging Area Entrance, 'false' is Port Entrance
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
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE_TO_INITIAL_DESTINATION, pdsm.get_state());

    // Notify state machine we've arrived
    pdsm.process_event(port_drayage_plugin::PortDrayageEvent::ARRIVED_AT_DESTINATION);
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::AWAITING_DIRECTION, pdsm.get_state());

    // Notify state machine we've recieved the next destination
    pdsm.process_event(port_drayage_plugin::PortDrayageEvent::RECEIVED_NEW_DESTINATION);
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE_TO_RECEIVED_DESTINATION, pdsm.get_state());

    // Rest of the state machine to be implemented and validated in future stories
}

// Test communication between PortDrayageWorker and PortDrayageStateMachine for State Machine flow
TEST(PortDrayageTest, testPortDrayageStateMachine2)
{
    // Create PortDrayageWorker object with _cmv_id of "123" and no cargo
    //std::function<void(cav_msgs::MobilityOperation)> fun = [](cav_msgs::MobilityOperation){return;};
    port_drayage_plugin::PortDrayageWorker pdw{
        "123", // CMV ID 
        "", // Cargo ID; empty string indicates CMV begins without no cargo
        "TEST_CARMA_HOST_ID", 
        true, // Flag indicating CMV's first destination; 'true' is Staging Area Entrance, 'false' is Port Entrance
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){}, 
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;}
    };

    // State Machine should begin in INACTIVE state
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::INACTIVE, pdw.get_port_drayage_state());

    // State Machine should transition to EN_ROUTE_TO_INITIAL_DESTINATION after guidance state is first engaged
    cav_msgs::GuidanceState guidance_state;
    guidance_state.state = cav_msgs::GuidanceState::ENGAGED;
    cav_msgs::GuidanceStateConstPtr guidance_state_pointer(new cav_msgs::GuidanceState(guidance_state));
    pdw.on_guidance_state(guidance_state_pointer);

    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE_TO_INITIAL_DESTINATION, pdw.get_port_drayage_state());

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

    // State Machine should transition to 'EN_ROUTE_TO_RECEIVED_DESTINATION' if a new port drayage MobilityOperation message is received

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

    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE_TO_RECEIVED_DESTINATION, pdw.get_port_drayage_state());

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
        "123", 
        "TEST_CARGO_ID", 
        "TEST_CARMA_HOST_ID", 
        true, // Flag indicating CMV's first destination; 'true' is Staging Area Entrance, 'false' is Port Entrance
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
    ASSERT_EQ("MOVING_TO_LOADING_AREA", route_req.request.routeID);
    ASSERT_EQ(1, route_req.request.destination_points.size());
    ASSERT_EQ(38.9550038, route_req.request.destination_points[0].latitude);
    ASSERT_EQ(-77.1481983, route_req.request.destination_points[0].longitude);
    ASSERT_EQ(false, route_req.request.destination_points[0].elevation_exists);
}

// Test Case for testing all potential inbound Port Drayage MobilityOperation messages
TEST(PortDrayageTest, testInboundAndComposedMobilityOperation)
{
    // Create PortDrayageWorker object with cmv_id of "123" that is not carrying cargo
    port_drayage_plugin::PortDrayageWorker pdw{
        "123", // CMV ID 
        "", // Cargo ID; empty string indicates the CMV is not carrying cargo
        "TEST_CARMA_HOST_ID", 
        true, // Flag indicating CMV's first destination; 'true' is Staging Area Entrance, 'false' is Port Entrance
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){},
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;}
    };

    // State Machine should transition to EN_ROUTE_TO_INITIAL_DESTINATION after guidance state is first engaged
    cav_msgs::GuidanceState guidance_state;
    guidance_state.state = cav_msgs::GuidanceState::ENGAGED;
    cav_msgs::GuidanceStateConstPtr guidance_state_pointer(new cav_msgs::GuidanceState(guidance_state));
    pdw.on_guidance_state(guidance_state_pointer);

    // State Machine should transition to AWAITING_DIRECTION if a 'ROUTE_LOADED' event occurs immediately after a 'ROUTE_COMPLETED' event
    cav_msgs::RouteEvent route_event_1;
    route_event_1.event = cav_msgs::RouteEvent::ROUTE_COMPLETED; 
    cav_msgs::RouteEventConstPtr route_event_pointer_1(new cav_msgs::RouteEvent(route_event_1));
    pdw.on_route_event(route_event_pointer_1); // PortDrayageWorker receives RouteEvent indicating route has been completed

    cav_msgs::RouteEvent route_event_2;
    route_event_2.event = cav_msgs::RouteEvent::ROUTE_LOADED; 
    cav_msgs::RouteEventConstPtr route_event_pointer_2(new cav_msgs::RouteEvent(route_event_2));
    pdw.on_route_event(route_event_pointer_2); // PortDrayageWorker receives RouteEvent indicating the previously completed route is no longer active
    
    // Create a "PICKUP" MobilityOperationConstPtr for pdw
    cav_msgs::MobilityOperation mobility_operation_msg;
    mobility_operation_msg.strategy = "carma/port_drayage";
    mobility_operation_msg.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"321\", \"location\"\
        : { \"latitude\": \"38.9554377\", \"longitude\": \"-77.1503421\" }, \"destination\": { \"latitude\"\
        : \"38.9550038\", \"longitude\": \"-77.1481983\" }, \"operation\": \"PICKUP\", \"action_id\"\
        : \"32\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr(new cav_msgs::MobilityOperation(mobility_operation_msg));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr);

    // Check that the received message was parsed and stored correctly
    ASSERT_EQ("321", *pdw._latest_mobility_operation_msg.cargo_id);
    ASSERT_EQ("PICKUP", pdw._latest_mobility_operation_msg.operation);
    ASSERT_NEAR(-77.1503421, *pdw._latest_mobility_operation_msg.start_longitude, 0.00000001);
    ASSERT_NEAR(38.9554377, *pdw._latest_mobility_operation_msg.start_latitude, 0.00000001);
    ASSERT_NEAR(-77.1481983, *pdw._latest_mobility_operation_msg.dest_longitude, 0.00000001);
    ASSERT_NEAR(38.9550038, *pdw._latest_mobility_operation_msg.dest_latitude, 0.00000001);
    ASSERT_EQ("32", *pdw._latest_mobility_operation_msg.current_action_id);

    // Create a MobilityOperationConstPtr with a cmv_id that is not intended for this specific vehicle
    cav_msgs::MobilityOperation mobility_operation_msg2;
    mobility_operation_msg2.strategy = "carma/port_drayage";
    mobility_operation_msg2.strategy_params = "{ \"cmv_id\": \"444\", \"cargo_id\": \"567\", \"location\"\
        : { \"latitude\": \"48.9554377\", \"longitude\": \"-67.1503421\" }, \"destination\": { \"latitude\"\
        : \"48.9550038\", \"longitude\": \"-57.1481983\" }, \"operation\": \"PICKUP\", \"action_id\"\
        : \"44\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr2(new cav_msgs::MobilityOperation(mobility_operation_msg2));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr2);

    // Check that the contents of the received message was not parsed and stored since it was not intended for this CMV
    ASSERT_EQ("321", *pdw._latest_mobility_operation_msg.cargo_id);
    ASSERT_EQ("PICKUP", pdw._latest_mobility_operation_msg.operation);
    ASSERT_NEAR(-77.1503421, *pdw._latest_mobility_operation_msg.start_longitude, 0.00000001);
    ASSERT_NEAR(38.9554377, *pdw._latest_mobility_operation_msg.start_latitude, 0.00000001);
    ASSERT_NEAR(-77.1481983, *pdw._latest_mobility_operation_msg.dest_longitude, 0.00000001);
    ASSERT_NEAR(38.9550038, *pdw._latest_mobility_operation_msg.dest_latitude, 0.00000001);
    ASSERT_EQ("32", *pdw._latest_mobility_operation_msg.current_action_id);

    // Test composeArrivalMessage for when CMV has arrived at the Loading Area
    
    // Set the pdw's map projector and its current pose
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

    // Obtain the contents of the broadcasted message when the CMV arrives at the Loading Area
    cav_msgs::MobilityOperation msg = pdw.compose_arrival_message();
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
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg.m_header.sender_id);
    ASSERT_FALSE(msg.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_FALSE(has_cargo);
    ASSERT_EQ("321", cargo_id); 
    ASSERT_EQ("PICKUP", operation);
    ASSERT_EQ("32", action_id);
    ASSERT_NEAR(38.95622708, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.15066142, vehicle_longitude, 0.001);

    // Create an "EXIT_STAGING_AREA" MobilityOperationConstPtr for pdw
    cav_msgs::MobilityOperation mobility_operation_msg3;
    mobility_operation_msg3.strategy = "carma/port_drayage";
    mobility_operation_msg3.strategy_params = "{ \"cmv_id\": \"123\", \"destination\": { \"latitude\"\
        : \"38.9103493\", \"longitude\": \"-77.1499283\" }, \"operation\": \"EXIT_STAGING_AREA\", \"action_id\"\
        : \"34\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr3(new cav_msgs::MobilityOperation(mobility_operation_msg3));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr3);

    // Test composeArrivalMessage for when CMV has arrived at the Staging Area Exit

    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.9103493 +lon_0=-77.1499283 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std_msgs::StringConstPtr georeference_msg_ptr2(new std_msgs::String(georeference_msg));
    pdw.on_new_georeference(georeference_msg_ptr2);

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    geometry_msgs::PoseStampedConstPtr pose_msg_ptr2(new geometry_msgs::PoseStamped(pose_msg));
    pdw.on_new_pose(pose_msg_ptr2); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Staging Area Exit
    cav_msgs::MobilityOperation msg2 = pdw.compose_arrival_message();
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
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg2.m_header.sender_id);
    ASSERT_FALSE(msg2.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("321", cargo_id);
    ASSERT_EQ("EXIT_STAGING_AREA", operation);
    ASSERT_EQ("34", action_id);
    ASSERT_NEAR(38.9103493, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.1499283, vehicle_longitude, 0.001);

    // Create an "ENTER_PORT" MobilityOperationConstPtr for pdw
    cav_msgs::MobilityOperation mobility_operation_msg4;
    mobility_operation_msg4.strategy = "carma/port_drayage";
    mobility_operation_msg4.strategy_params = "{ \"cmv_id\": \"123\", \"destination\": { \"latitude\"\
        : \"38.9199993\", \"longitude\": \"-77.1434283\" }, \"operation\": \"ENTER_PORT\", \"action_id\"\
        : \"36\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr4(new cav_msgs::MobilityOperation(mobility_operation_msg4));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr4);

    // Test composeArrivalMessage for when CMV has arrived at the Port Entrance

    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.9199993 +lon_0=-77.1434283 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std_msgs::StringConstPtr georeference_msg_ptr3(new std_msgs::String(georeference_msg));
    pdw.on_new_georeference(georeference_msg_ptr3);

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    geometry_msgs::PoseStampedConstPtr pose_msg_ptr3(new geometry_msgs::PoseStamped(pose_msg));
    pdw.on_new_pose(pose_msg_ptr3); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    cav_msgs::MobilityOperation msg3 = pdw.compose_arrival_message();
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
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg3.m_header.sender_id);
    ASSERT_FALSE(msg3.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("321", cargo_id);
    ASSERT_EQ("ENTER_PORT", operation);
    ASSERT_EQ("36", action_id);
    ASSERT_NEAR(38.9199993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.1434283, vehicle_longitude, 0.001);

    // Create a "DROPOFF" MobilityOperationConstPtr for pdw
    cav_msgs::MobilityOperation mobility_operation_msg5;
    mobility_operation_msg5.strategy = "carma/port_drayage";
    mobility_operation_msg5.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"321\", \"destination\": { \"latitude\"\
        : \"38.34259993\", \"longitude\": \"-77.1224283\" }, \"operation\": \"DROPOFF\", \"action_id\"\
        : \"37\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr5(new cav_msgs::MobilityOperation(mobility_operation_msg5));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr5);

    // Test composeArrivalMessage for when CMV has arrived at the Dropoff location

    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.34259993 +lon_0=-77.1224283 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std_msgs::StringConstPtr georeference_msg_ptr4(new std_msgs::String(georeference_msg));
    pdw.on_new_georeference(georeference_msg_ptr4);

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    geometry_msgs::PoseStampedConstPtr pose_msg_ptr4(new geometry_msgs::PoseStamped(pose_msg));
    pdw.on_new_pose(pose_msg_ptr4); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    cav_msgs::MobilityOperation msg4 = pdw.compose_arrival_message();
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
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg4.m_header.sender_id);
    ASSERT_FALSE(msg4.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("321", cargo_id);
    ASSERT_EQ("DROPOFF", operation);
    ASSERT_EQ("37", action_id);
    ASSERT_NEAR(38.34259993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.1224283, vehicle_longitude, 0.001);

    // Create a "PICKUP" MobilityOperationConstPtr for pdw
    cav_msgs::MobilityOperation mobility_operation_msg6;
    mobility_operation_msg6.strategy = "carma/port_drayage";
    mobility_operation_msg6.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"422\", \"destination\": { \"latitude\"\
        : \"38.3119993\", \"longitude\": \"-77.2314283\" }, \"operation\": \"PICKUP\", \"action_id\"\
        : \"38\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr6(new cav_msgs::MobilityOperation(mobility_operation_msg6));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr6);

    // Test composeArrivalMessage for when CMV has arrived at the Pickup location
    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.3119993 +lon_0=-77.2314283 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std_msgs::StringConstPtr georeference_msg_ptr5(new std_msgs::String(georeference_msg));
    pdw.on_new_georeference(georeference_msg_ptr5);

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    geometry_msgs::PoseStampedConstPtr pose_msg_ptr5(new geometry_msgs::PoseStamped(pose_msg));
    pdw.on_new_pose(pose_msg_ptr5); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    cav_msgs::MobilityOperation msg5 = pdw.compose_arrival_message();
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
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg5.m_header.sender_id);
    ASSERT_FALSE(msg5.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_FALSE(has_cargo);
    ASSERT_EQ("422", cargo_id);
    ASSERT_EQ("PICKUP", operation);
    ASSERT_EQ("38", action_id);
    ASSERT_NEAR(38.3119993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.2314283, vehicle_longitude, 0.001);

    // Create an "PORT_CHECKPOINT" MobilityOperationConstPtr for pdw
    cav_msgs::MobilityOperation mobility_operation_msg7;
    mobility_operation_msg7.strategy = "carma/port_drayage";
    mobility_operation_msg7.strategy_params = "{ \"cmv_id\": \"123\", \"destination\": { \"latitude\"\
        : \"38.3339993\", \"longitude\": \"-77.2594283\" }, \"operation\": \"PORT_CHECKPOINT\", \"action_id\"\
        : \"39\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr7(new cav_msgs::MobilityOperation(mobility_operation_msg7));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr7);

    // Test composeArrivalMessage for when CMV has arrived at the Port Checkpoint
    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.3339993 +lon_0=-77.2594283 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std_msgs::StringConstPtr georeference_msg_ptr6(new std_msgs::String(georeference_msg));
    pdw.on_new_georeference(georeference_msg_ptr6);

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    geometry_msgs::PoseStampedConstPtr pose_msg_ptr6(new geometry_msgs::PoseStamped(pose_msg));
    pdw.on_new_pose(pose_msg_ptr6); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    cav_msgs::MobilityOperation msg6 = pdw.compose_arrival_message();
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
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg6.m_header.sender_id);
    ASSERT_FALSE(msg6.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("422", cargo_id);
    ASSERT_EQ("PORT_CHECKPOINT", operation);
    ASSERT_EQ("39", action_id);
    ASSERT_NEAR(38.3339993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.2594283, vehicle_longitude, 0.001);

    // Create a "HOLDING_AREA" MobilityOperationConstPtr for pdw
    cav_msgs::MobilityOperation mobility_operation_msg8;
    mobility_operation_msg8.strategy = "carma/port_drayage";
    mobility_operation_msg8.strategy_params = "{ \"cmv_id\": \"123\", \"destination\": { \"latitude\"\
        : \"38.4139993\", \"longitude\": \"-77.2595583\" }, \"operation\": \"HOLDING_AREA\", \"action_id\"\
        : \"40\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr8(new cav_msgs::MobilityOperation(mobility_operation_msg8));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr8);

    // Test composeArrivalMessage for when CMV has arrived at the Holding Area
    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.4139993 +lon_0=-77.2595583 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std_msgs::StringConstPtr georeference_msg_ptr7(new std_msgs::String(georeference_msg));
    pdw.on_new_georeference(georeference_msg_ptr7);

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    geometry_msgs::PoseStampedConstPtr pose_msg_ptr7(new geometry_msgs::PoseStamped(pose_msg));
    pdw.on_new_pose(pose_msg_ptr7); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    cav_msgs::MobilityOperation msg7 = pdw.compose_arrival_message();
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
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg7.m_header.sender_id);
    ASSERT_FALSE(msg7.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("422", cargo_id);
    ASSERT_EQ("HOLDING_AREA", operation);
    ASSERT_EQ("40", action_id);
    ASSERT_NEAR(38.4139993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.2595583, vehicle_longitude, 0.001);

   // Create an "EXIT_PORT" MobilityOperationConstPtr for pdw
    cav_msgs::MobilityOperation mobility_operation_msg9;
    mobility_operation_msg9.strategy = "carma/port_drayage";
    mobility_operation_msg9.strategy_params = "{ \"cmv_id\": \"123\", \"destination\": { \"latitude\"\
        : \"38.6639993\", \"longitude\": \"-77.8395583\" }, \"operation\": \"EXIT_PORT\", \"action_id\"\
        : \"41\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr9(new cav_msgs::MobilityOperation(mobility_operation_msg9));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr9);

    // Test composeArrivalMessage for when CMV has arrived at the Port Exit
    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.6639993 +lon_0=-77.8395583 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std_msgs::StringConstPtr georeference_msg_ptr8(new std_msgs::String(georeference_msg));
    pdw.on_new_georeference(georeference_msg_ptr8);

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    geometry_msgs::PoseStampedConstPtr pose_msg_ptr8(new geometry_msgs::PoseStamped(pose_msg));
    pdw.on_new_pose(pose_msg_ptr8); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    cav_msgs::MobilityOperation msg8 = pdw.compose_arrival_message();
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
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg8.m_header.sender_id);
    ASSERT_FALSE(msg8.strategy_params.empty());
    ASSERT_EQ("123", cmv_id);
    ASSERT_TRUE(has_cargo);
    ASSERT_EQ("422", cargo_id);
    ASSERT_EQ("EXIT_PORT", operation);
    ASSERT_EQ("41", action_id);
    ASSERT_NEAR(38.6639993, vehicle_latitude, 0.001);
    ASSERT_NEAR(-77.8395583, vehicle_longitude, 0.001);

   // Create an "ENTER_STAGING_AREA" MobilityOperationConstPtr for pdw
    cav_msgs::MobilityOperation mobility_operation_msg10;
    mobility_operation_msg10.strategy = "carma/port_drayage";
    mobility_operation_msg10.strategy_params = "{ \"cmv_id\": \"123\", \"destination\": { \"latitude\"\
        : \"38.6889993\", \"longitude\": \"-77.8124583\" }, \"operation\": \"ENTER_STAGING_AREA\", \"action_id\"\
        : \"42\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr10(new cav_msgs::MobilityOperation(mobility_operation_msg10));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr10);

    // Test composeArrivalMessage for when CMV has arrived at the Staging Area Entrance
    // Set the pdw's map projector and its current pose
    base_proj = "+proj=tmerc +lat_0=38.6889993 +lon_0=-77.8124583 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                             "+no_defs";
    georeference_msg.data = base_proj;
    std_msgs::StringConstPtr georeference_msg_ptr9(new std_msgs::String(georeference_msg));
    pdw.on_new_georeference(georeference_msg_ptr9);

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    geometry_msgs::PoseStampedConstPtr pose_msg_ptr9(new geometry_msgs::PoseStamped(pose_msg));
    pdw.on_new_pose(pose_msg_ptr9); // Sets the host vehicle's current gps lat/lon position

    // Obtain the contents of the broadcasted message when the CMV arrives at the Port Entrance
    cav_msgs::MobilityOperation msg9 = pdw.compose_arrival_message();
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
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg9.m_header.sender_id);
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
    // Create PortDrayageWorker object with _cmv_id of "123"
    port_drayage_plugin::PortDrayageWorker pdw{
        "123", 
        "", // Empty string indicates CMV is not carrying cargo 
        "TEST_CARMA_HOST_ID", 
        true, // Flag indicating CMV's first destination; 'true' is Staging Area Entrance, 'false' is Port Entrance
        [](cav_msgs::MobilityOperation){}, 
        [](cav_msgs::UIInstructions){},
        1.0, 
        true, // Flag to enable port drayage operations
        [](cav_srvs::SetActiveRoute){return true;} 
    };

    // First 'operation' is for 'PICKUP'. Verify the created UI Instructions message:
    std::string current_operation = "PICKUP";
    std::string previous_operation = "";

    cav_msgs::UIInstructions ui_instructions_msg = pdw.compose_ui_instructions(current_operation, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "A new Port Drayage route with operation type 'PICKUP' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, cav_msgs::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");

    // Second received MobilityOperation message is for a 'DROPOFF' operation. The previous 'PICKUP' operation has been completed.
    current_operation = "DROPOFF";
    previous_operation = "PICKUP";

    ui_instructions_msg = pdw.compose_ui_instructions(current_operation, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "The pickup action was completed successfully. A new Port Drayage route with operation type 'DROPOFF' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, cav_msgs::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");

    // Third received MobilityOperation message is for a 'EXIT_STAGING_AREA' operation. The previous 'DROPOFF' operation has been completed.
    current_operation = "EXIT_STAGING_AREA";
    previous_operation = "DROPOFF";

    ui_instructions_msg = pdw.compose_ui_instructions(current_operation, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "The dropoff action was completed successfully. A new Port Drayage route with operation type 'EXIT_STAGING_AREA' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, cav_msgs::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");
    
    // Fourth received MobilityOperation message is for a 'ENTER_PORT' operation. The previous 'EXIT_STAGING_AREA' operation has been completed.
    current_operation = "ENTER_PORT";
    previous_operation = "EXIT_STAGING_AREA";

    ui_instructions_msg = pdw.compose_ui_instructions(current_operation, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "A new Port Drayage route with operation type 'ENTER_PORT' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, cav_msgs::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");

    // Fifth received MobilityOperation message is for a 'ENTER_PORT' operation. The previous 'EXIT_STAGING_AREA' operation has been completed.
    current_operation = "EXIT_PORT";
    previous_operation = "HOLDING_AREA";

    ui_instructions_msg = pdw.compose_ui_instructions(current_operation, previous_operation);

    ASSERT_EQ(ui_instructions_msg.msg, "The inspection was completed successfully. A new Port Drayage route with operation type 'EXIT_PORT' has been received. "
                                  "Select YES to engage the system on the route, or select NO to remain "
                                  "disengaged.");
    ASSERT_EQ(ui_instructions_msg.type, cav_msgs::UIInstructions::ACK_REQUIRED);
    ASSERT_EQ(ui_instructions_msg.response_service, "/guidance/set_guidance_active");
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
