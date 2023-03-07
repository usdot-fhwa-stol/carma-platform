/*
 * Copyright (C) 2022-2023 LEIDOS.
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

#include "approaching_emergency_vehicle_plugin/approaching_emergency_vehicle_plugin_node.hpp"
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>
#include <lanelet2_io/Io.h>

namespace approaching_emergency_vehicle_plugin{

    TEST(Testapproaching_emergency_vehicle_plugin, testStateMachineTransitions){

        // Create, configure, and activate worker_node (ApproachingEmergencyVehiclePlugin)
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<approaching_emergency_vehicle_plugin::ApproachingEmergencyVehiclePlugin>(options);

        worker_node->configure(); //Call configure state transition
        worker_node->activate();  //Call activate state transition to get not read for runtime
        
        // Test sequence of events that indicate the ERV approaches and passes the ego vehicle with the ego vehicle being 
        //      unable to lane change out of the path of the ERV
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::NO_APPROACHING_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::NO_APPROACHING_ERV);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::NO_APPROACHING_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::APPROACHING_ERV_IN_PATH);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::MOVING_OVER_FOR_APPROACHING_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_ABOUT_TO_PASS_IN_PATH);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::WAITING_FOR_APPROACHING_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_PASSING_IN_PATH);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_PASSED);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::NO_APPROACHING_ERV);

        // Test sequence of events that indicate the ERV approaches and passes the ego vehicle with the ego vehicle being 
        //      able to lane change out of the path of the ERV
        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::APPROACHING_ERV_IN_PATH);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::MOVING_OVER_FOR_APPROACHING_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::APPROACHING_ERV_NOT_IN_PATH);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::WAITING_FOR_APPROACHING_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_ABOUT_TO_PASS_NOT_IN_PATH);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::WAITING_FOR_APPROACHING_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_PASSING_NOT_IN_PATH);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_UPDATE_TIMEOUT);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::NO_APPROACHING_ERV);

        // Test sequence of events that indicate the ERV approaches and passes the ego vehicle with the ego vehicle always 
        //      being out of the path of the ERV
        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::APPROACHING_ERV_NOT_IN_PATH);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::WAITING_FOR_APPROACHING_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_ABOUT_TO_PASS_NOT_IN_PATH);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::WAITING_FOR_APPROACHING_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_PASSING_NOT_IN_PATH);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_PASSED);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::NO_APPROACHING_ERV);

        // Test sequence of events that indicate the ERV is first tracked when it is already too close for the ego
        //      vehicle to attempt to change lanes out of the ERV's path
        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_ABOUT_TO_PASS_IN_PATH);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::WAITING_FOR_APPROACHING_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_PASSING_IN_PATH);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV);

        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_PASSED);
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::NO_APPROACHING_ERV);
    }

    TEST(Testapproaching_emergency_vehicle_plugin, testRouteConflict){

        // Create, configure, and activate worker_node (ApproachingEmergencyVehiclePlugin)
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<approaching_emergency_vehicle_plugin::ApproachingEmergencyVehiclePlugin>(options);

        worker_node->configure(); //Call configure state transition
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // create semantic map with lane width 3.7 meters and lanelet length 25.0 meters (3 adjacent lanes; each 4 lanelets long)
        lanelet::LaneletMapPtr map = carma_wm::test::buildGuidanceTestMap(3.7, 25.0);

        // Create CARMA World Model for ego vehicle and set its route
        std::shared_ptr<carma_wm::CARMAWorldModel> ego_cmw = std::make_shared<carma_wm::CARMAWorldModel>();
        ego_cmw->setConfigSpeedLimit(10.0);
        ego_cmw->carma_wm::CARMAWorldModel::setMap(map);
        carma_wm::test::setRouteByIds({1212, 1213}, ego_cmw);
        worker_node->wm_ = ego_cmw;

        // Get the shortest path from the ego vehicle's route
        auto ego_shortest_path = ego_cmw->getRoute()->shortestPath();

        // Create CARMA World Model for ERV 
        std::shared_ptr<carma_wm::CARMAWorldModel> erv_cmw = std::make_shared<carma_wm::CARMAWorldModel>();
        erv_cmw->setConfigSpeedLimit(20.0);
        erv_cmw->carma_wm::CARMAWorldModel::setMap(map);

        // Build ERV's routing graph from map and set ERV's route so that it intersects the ego vehicle's future shortest path in lanelet 1212
        auto traffic_rules = erv_cmw->getTrafficRules();
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*erv_cmw->getMap(), *traffic_rules.get());
        lanelet::ConstLanelet lanelet_1210 = erv_cmw->getMap()->laneletLayer.get(1210);
        lanelet::ConstLanelet lanelet_1213 = erv_cmw->getMap()->laneletLayer.get(1213);
        lanelet::Optional<lanelet::routing::Route> erv_future_route = map_graph->getRoute(lanelet_1210, lanelet_1213);

        // Verify that ERV's future route intersects ego vehicle's future shortest path in lanelet 1212
        boost::optional<lanelet::ConstLanelet> intersecting_lanelet = worker_node->getRouteIntersectingLanelet(erv_future_route.get());
        ASSERT_TRUE(intersecting_lanelet);
        ASSERT_EQ((*intersecting_lanelet).id(), 1212);

        // Create ERV's current position in the map frame (first point on centerline of lanelet 1210)
        lanelet::ConstLineString2d lanelet_1210_centerline = lanelet::utils::to2D(lanelet_1210.centerline());
        lanelet::BasicPoint2d erv_current_position_in_map = lanelet_1210_centerline.front();

        // Set ERV speed faster than ego speed so that ERV will eventually pass the ego vehicle
        worker_node->current_speed_ = 10.0; // Set ego vehicle's current speed to 10 m/s
        double erv_speed = 20.0; // Set ERV's current speed to 20 m/s

        // Verify that ERV will pass ego vehicle in ~5 seconds (ERV is 50 meters behind ego vehicle and travelling 10 m/s faster)
        boost::optional<double> seconds_until_passing = worker_node->getSecondsUntilPassing(erv_future_route, erv_current_position_in_map, erv_speed, *intersecting_lanelet);
        ASSERT_TRUE(seconds_until_passing);
        ASSERT_NEAR(seconds_until_passing.get(), 5.0, 0.01);
    }

    TEST(Testapproaching_emergency_vehicle_plugin, testBSMProcessing){

        // Create, configure, and activate worker_node (ApproachingEmergencyVehiclePlugin)
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<approaching_emergency_vehicle_plugin::ApproachingEmergencyVehiclePlugin>(options);

        worker_node->configure(); //Call configure state transition
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Verify that plugin is not currently tracking an ERV
        ASSERT_EQ(worker_node->has_tracked_erv_, false);

        // Set ego speed to 10 m/s
        worker_node->current_speed_ = 10.0;

        // Set georeference for worker_node so that it can convert ERV BSM lat/lon coordinates to map coordinates
        std::string proj = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";
        std_msgs::msg::String str_msg;
        str_msg.data = proj;
        std::unique_ptr<std_msgs::msg::String> msg_ptr = std::make_unique<std_msgs::msg::String>(str_msg);
        worker_node->georeferenceCallback(std::move(msg_ptr)); 

        // Create a world model object for worker_node and set its map
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>(); 

        // Set projection
        int projector_type = 0;
        std::string target_frame;
        lanelet::ErrorMessages load_errors;

        // Load map file and parameters
        std::string file = "../../install_ros2/approaching_emergency_vehicle_plugin/share/approaching_emergency_vehicle_plugin/resource/town01_vector_map_1.osm";

        lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
        lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
        lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

        // Set map
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        // Build routing graph from map	
        auto traffic_rules = cmw->getTrafficRules();
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*cmw->getMap(), *traffic_rules.get());

        // Set ego vehicle's route 
        lanelet::ConstLanelet starting_lanelet = cmw->getMap()->laneletLayer.get(168);
        lanelet::ConstLanelet ending_lanelet = cmw->getMap()->laneletLayer.get(111);
        lanelet::Optional<lanelet::routing::Route> optional_route = map_graph->getRoute(starting_lanelet, ending_lanelet);
        lanelet::routing::Route route = std::move(*optional_route);
        carma_wm::LaneletRoutePtr route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
        cmw->setRoute(route_ptr);

        // Set worker_node's world model to cmw
        worker_node->wm_ = cmw;

        // Verify that worker_node's shortest path is 168->178->111
        auto ego_shortest_path = worker_node->wm_->getRoute()->shortestPath();
        ASSERT_EQ(ego_shortest_path.size(), 3);
        ASSERT_EQ(ego_shortest_path[0].id(), 168);
        ASSERT_EQ(ego_shortest_path[1].id(), 170);
        ASSERT_EQ(ego_shortest_path[2].id(), 111);

        // Set worker_node's route_state_ object to indicate ego vehicle is currently at the beginning of its shortest path in lanelet 168
        carma_planning_msgs::msg::RouteState route_state_msg;
        route_state_msg.lanelet_id = 168;
        route_state_msg.down_track = 0.0;
        std::unique_ptr<carma_planning_msgs::msg::RouteState> route_state_msg_ptr = std::make_unique<carma_planning_msgs::msg::RouteState>(route_state_msg);
        worker_node->routeStateCallback(std::move(route_state_msg_ptr)); 

        // Create ERV's BSM with current location and route destination points that results in an intersecting lanelet between
        //       ERV's future route and the ego vehicle's future shortest path
        carma_v2x_msgs::msg::BSM erv_bsm;
        erv_bsm.header.stamp = rclcpp::Time(0,0);
        erv_bsm.core_data.id = {1,2,3,4};
        erv_bsm.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::LATITUDE_AVAILABLE;
        erv_bsm.core_data.latitude = 48.9975384; 
        erv_bsm.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::LONGITUDE_AVAILABLE;
        erv_bsm.core_data.longitude = 8.0026469;
        erv_bsm.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::SPEED_AVAILABLE;
        erv_bsm.core_data.speed = 10.0;

        // Add Part II Content to ERV BSM to indicate that its sirens and lights are active
        erv_bsm.presence_vector |= carma_v2x_msgs::msg::BSM::HAS_PART_II;
        carma_v2x_msgs::msg::BSMPartIIExtension part_ii_special_vehicle_ext;
        part_ii_special_vehicle_ext.part_ii_id = carma_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT;
        part_ii_special_vehicle_ext.special_vehicle_extensions.presence_vector |= carma_v2x_msgs::msg::SpecialVehicleExtensions::HAS_VEHICLE_ALERTS;
        part_ii_special_vehicle_ext.special_vehicle_extensions.vehicle_alerts.siren_use.siren_in_use = j2735_v2x_msgs::msg::SirenInUse::IN_USE;
        part_ii_special_vehicle_ext.special_vehicle_extensions.vehicle_alerts.lights_use.lightbar_in_use = j2735_v2x_msgs::msg::LightbarInUse::IN_USE;
        erv_bsm.part_ii.push_back(part_ii_special_vehicle_ext);

        erv_bsm.presence_vector |= carma_v2x_msgs::msg::BSM::HAS_REGIONAL;
        carma_v2x_msgs::msg::BSMRegionalExtension regional_ext;
        regional_ext.regional_extension_id = carma_v2x_msgs::msg::BSMRegionalExtension::ROUTE_DESTINATIONS;

        carma_v2x_msgs::msg::Position3D position1;
        position1.latitude = 48.9980717;
        position1.longitude = 8.0026509;

        carma_v2x_msgs::msg::Position3D position2;
        position2.latitude = 48.9990511; 
        position2.longitude = 8.0020885; 

        regional_ext.route_destination_points.push_back(position1);
        regional_ext.route_destination_points.push_back(position2);
        erv_bsm.regional.push_back(regional_ext);
        
        // Verify that ego vehicle does not determine that ERV is approaching since both vehicles are travelling the same speed (10 m/s)
        std::unique_ptr<carma_v2x_msgs::msg::BSM> erv_bsm_ptr = std::make_unique<carma_v2x_msgs::msg::BSM>(erv_bsm);
        worker_node->incomingBsmCallback(std::move(erv_bsm_ptr)); 
        ASSERT_FALSE(worker_node->has_tracked_erv_);

        // Increase ERV's speed to twice the ego vehicle speed so that the ERV is considered to be approaching the ego vehicle
        erv_bsm.core_data.speed = 20.0;
        std::unique_ptr<carma_v2x_msgs::msg::BSM> erv_bsm_ptr2 = std::make_unique<carma_v2x_msgs::msg::BSM>(erv_bsm);
        worker_node->incomingBsmCallback(std::move(erv_bsm_ptr2)); 
        ASSERT_TRUE(worker_node->has_tracked_erv_);

        // Update timestamp of ERV BSM to a time that does not satisfy config_.bsm_processing_frequency and verify that tracked_erv_ doesn't update
        ASSERT_EQ(worker_node->tracked_erv_.latest_bsm_timestamp, rclcpp::Time(0, 0, worker_node->get_clock()->get_clock_type()));
        worker_node->config_.bsm_processing_frequency = 0.5; // (Hz) Only process ERV BSMs that are at least 2 seconds apart
        
        erv_bsm.header.stamp = rclcpp::Time(1, 0);
        std::unique_ptr<carma_v2x_msgs::msg::BSM> erv_bsm_ptr3 = std::make_unique<carma_v2x_msgs::msg::BSM>(erv_bsm);
        worker_node->incomingBsmCallback(std::move(erv_bsm_ptr3)); 
        ASSERT_EQ(worker_node->tracked_erv_.latest_bsm_timestamp, rclcpp::Time(0, 0, worker_node->get_clock()->get_clock_type()));

        // Update timestamp of ERV BSM to a time that does satisfy config_.bsm_processing_frequency and verify that tracked_erv_ updates
        erv_bsm.header.stamp = rclcpp::Time(2, 50);
        std::unique_ptr<carma_v2x_msgs::msg::BSM> erv_bsm_ptr4 = std::make_unique<carma_v2x_msgs::msg::BSM>(erv_bsm);
        worker_node->incomingBsmCallback(std::move(erv_bsm_ptr4)); 
        ASSERT_EQ(worker_node->tracked_erv_.latest_bsm_timestamp, rclcpp::Time(2, 50, worker_node->get_clock()->get_clock_type()));

        worker_node->config_.timeout_duration = 2.0; //(Seconds) Update timeout duration to decrease time of this unit test

        // Add Node to an executor and spin it to trigger timer callbacks
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(worker_node->get_node_base_interface());

        // Spin executor for 10 seconds
        auto end_time = std::chrono::system_clock::now() + std::chrono::seconds(3);
        while(std::chrono::system_clock::now() < end_time){
            executor.spin_once();
        }

        // Verify that the plugin no longer has an approaching ERV
        ASSERT_FALSE(worker_node->has_tracked_erv_);
    }

    TEST(Testapproaching_emergency_vehicle_plugin, testManeuverPlanWhenWaitingForApproachingErv){
        // Create, configure, and activate worker_node (ApproachingEmergencyVehiclePlugin)
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<approaching_emergency_vehicle_plugin::ApproachingEmergencyVehiclePlugin>(options);

        worker_node->configure(); //Call configure state transition
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Verify that initial state is NO_APPROACHING_ERV
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::NO_APPROACHING_ERV);

        worker_node->has_tracked_erv_ = true;
        worker_node->tracked_erv_.seconds_until_passing = 17.5; // Set value to trigger state machine transition to 'WAITING_FOR_APPROACHING_ERV' state
        worker_node->config_.minimal_plan_duration = 15.0; // (Seconds) Maneuver plan shall be at least 15.0 seconds long

        // Set georeference for worker_node so that it can convert ERV BSM lat/lon coordinates to map coordinates
        std::string proj = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";
        std_msgs::msg::String str_msg;
        str_msg.data = proj;
        std::unique_ptr<std_msgs::msg::String> msg_ptr = std::make_unique<std_msgs::msg::String>(str_msg);
        worker_node->georeferenceCallback(std::move(msg_ptr)); 

        // Create a world model object for worker_node and set its map
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>(); 

        // Set projection
        int projector_type = 0;
        std::string target_frame;
        lanelet::ErrorMessages load_errors;

        // Load map file and parameters
        std::string file = "../../install_ros2/approaching_emergency_vehicle_plugin/share/approaching_emergency_vehicle_plugin/resource/town01_vector_map_1.osm";

        lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
        lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
        lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

        // Set map
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        // Build routing graph from map	
        auto traffic_rules = cmw->getTrafficRules();
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*cmw->getMap(), *traffic_rules.get());

        // Place ego vehicle in left lane on route with shortest path 164->165->171->109->106
        lanelet::ConstLanelet starting_lanelet = cmw->getMap()->laneletLayer.get(164); 
        lanelet::ConstLanelet ending_lanelet = cmw->getMap()->laneletLayer.get(106);
        lanelet::Optional<lanelet::routing::Route> optional_route = map_graph->getRoute(starting_lanelet, ending_lanelet);
        lanelet::routing::Route route = std::move(*optional_route);
        carma_wm::LaneletRoutePtr route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
        cmw->setRoute(route_ptr);

        // Set worker_node's world model to cmw
        worker_node->wm_ = cmw;

        //**********************//
        // TEST 1: Verify correct ego vehicle lane-following maneuver plan in non-shortest-path lanelets on route when in 
        //         WAITING_FOR_APPROACHING_ERV state.
        //**********************//

        // Create plan maneuvers service request for worker_node
        auto req = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Request>();

        // Set maneuver plan service request's vehicle state parameters to place ego vehicle in right lane to verify that it can remain off its
        //     shortest path (but still on its route) when generating a plan that keeps the ego vehicle in its lane.
        req->veh_x = 585277.966793666; // Lanelet 167 (right adjacent to 164)
        req->veh_y = 5460258.19308606; // Lanelet 167 (right adjacent to 164)
        req->veh_downtrack = 88.11799880; // (meters) Matches downtrack of (veh_x, veh_y) on ego vehicle's route
        req->veh_logitudinal_velocity = 10.0; // (m/s)
        req->veh_lane_id = "167";

        auto resp = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();
        auto header_srv = std::make_shared<rmw_request_id_t>();
        worker_node->plan_maneuvers_callback(header_srv, req, resp);

        ASSERT_EQ(resp->new_plan.maneuvers.size(), 3);

        // Verify Maneuver 0 lane following parameters in lanelet 167
        ASSERT_EQ(resp->new_plan.maneuvers[0].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[0].lane_following_maneuver.start_time).seconds(), 0.0, 0.01); 
        ASSERT_NEAR(resp->new_plan.maneuvers[0].lane_following_maneuver.end_dist, 190.243, 0.01);
        ASSERT_NEAR(resp->new_plan.maneuvers[0].lane_following_maneuver.end_speed, 10.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[0].lane_following_maneuver.end_time).seconds(), 10.2125, 0.01); 
        std::vector<std::string> maneuver_0_lane_id {"167"};
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.lane_ids, maneuver_0_lane_id); 
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.planning_tactical_plugin, "inlanecruising_plugin");

        // Verify Maneuver 1 lane following parameters in lanelet 169
        ASSERT_EQ(resp->new_plan.maneuvers[1].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[1].lane_following_maneuver.start_time).seconds(), 10.2125, 0.01); 
        ASSERT_NEAR(resp->new_plan.maneuvers[1].lane_following_maneuver.end_dist, 206.164, 0.01);
        ASSERT_NEAR(resp->new_plan.maneuvers[1].lane_following_maneuver.end_speed, 10.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[1].lane_following_maneuver.end_time).seconds(), 11.8046, 0.01); 
        std::vector<std::string> maneuver_1_lane_id {"169"};
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_following_maneuver.lane_ids, maneuver_1_lane_id); 
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_following_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_following_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_following_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_following_maneuver.parameters.planning_tactical_plugin, "inlanecruising_plugin");

        // Verify Maneuver 2 lane following parameters in lanelet 168
        ASSERT_EQ(resp->new_plan.maneuvers[2].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[2].lane_following_maneuver.start_time).seconds(), 11.8046, 0.01); 
        ASSERT_NEAR(resp->new_plan.maneuvers[2].lane_following_maneuver.end_dist, 240.438, 0.01);
        ASSERT_NEAR(resp->new_plan.maneuvers[2].lane_following_maneuver.end_speed, 10.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[2].lane_following_maneuver.end_time).seconds(), 15.232, 0.01); 
        std::vector<std::string> maneuver_2_lane_id {"168"};
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.lane_ids, maneuver_2_lane_id); 
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.parameters.planning_tactical_plugin, "inlanecruising_plugin");

        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::WAITING_FOR_APPROACHING_ERV);

        //**********************//
        // TEST 2: Verify that an empty maneuver plan is generated when the lane-following maneuver plan would result in the
        //         ego vehicle passing the end of its route when in WAITING_FOR_APPROACHING_ERV state.
        //**********************//

        // Increase minimal plan duration so that maneuver plan will reach end of route, resulting in an empty maneuver plan
        worker_node->config_.minimal_plan_duration = 100.0; // (Seconds) Maneuver plan shall be at least 100.0 seconds long
        auto resp2 = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();
        worker_node->plan_maneuvers_callback(header_srv, req, resp2);
        ASSERT_EQ(resp2->new_plan.maneuvers.size(), 6);
        ASSERT_EQ(resp2->new_plan.maneuvers.back().stop_and_wait_maneuver.parameters.planning_tactical_plugin, "stop_and_wait_plugin");

        //**********************//
        // TEST 3: Verify that an empty maneuver plan is generated when the lane-following maneuver plan would result in the
        //         ego vehicle exiting its route when in WAITING_FOR_APPROACHING_ERV state.
        //**********************//
        lanelet::ConstLanelet ending_lanelet_2 = cmw->getMap()->laneletLayer.get(160);
        lanelet::Optional<lanelet::routing::Route> optional_route_2 = map_graph->getRoute(starting_lanelet, ending_lanelet_2);
        lanelet::routing::Route route_2 = std::move(*optional_route_2);
        carma_wm::LaneletRoutePtr route_ptr_2 = std::make_shared<lanelet::routing::Route>(std::move(route_2));
        cmw->setRoute(route_ptr_2);

        // Set worker_node's world model to cmw
        worker_node->wm_ = cmw;
        worker_node->config_.minimal_plan_duration = 100.0; // (Seconds) Maneuver plan shall be at least 100.0 seconds long
        auto resp3 = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();
        worker_node->plan_maneuvers_callback(header_srv, req, resp3);

        ASSERT_TRUE(resp3->new_plan.maneuvers.empty());
    }

    TEST(Testapproaching_emergency_vehicle_plugin, testManeuverPlanWhenSlowingDownForErv){
        // Create, configure, and activate worker_node (ApproachingEmergencyVehiclePlugin)
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<approaching_emergency_vehicle_plugin::ApproachingEmergencyVehiclePlugin>(options);

        worker_node->configure(); //Call configure state transition
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Verify that initial state is NO_APPROACHING_ERV
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::NO_APPROACHING_ERV);

        worker_node->has_tracked_erv_ = true;
        worker_node->tracked_erv_.seconds_until_passing = 10.0; // Set value to trigger state machine transition to 'SLOWING_DOWN_FOR_ERV' state
        worker_node->config_.minimal_plan_duration = 25.0; // (Seconds) Maneuver plan shall be at least 25.0 seconds long to enable multiple maneuvers for this test case
        worker_node->config_.speed_limit_reduction_during_passing = 10.0; // (m/s) Amount to reduce maneuver target speed by when in 'SLOWING_DOWN_FOR_ERV' state
        worker_node->config_.minimum_reduced_speed_limit = 5.0; // (m/s) Minimum target speed when in 'SLOWING_DOWN_FOR_ERV' state

        // Set georeference for worker_node so that it can convert ERV BSM lat/lon coordinates to map coordinates
        std::string proj = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";
        std_msgs::msg::String str_msg;
        str_msg.data = proj;
        std::unique_ptr<std_msgs::msg::String> msg_ptr = std::make_unique<std_msgs::msg::String>(str_msg);
        worker_node->georeferenceCallback(std::move(msg_ptr)); 

        // Create a world model object for worker_node and set its map
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>(); 

        // Set projection
        int projector_type = 0;
        std::string target_frame;
        lanelet::ErrorMessages load_errors;

        // Load map file and parameters
        std::string file = "../../install_ros2/approaching_emergency_vehicle_plugin/share/approaching_emergency_vehicle_plugin/resource/town01_vector_map_1.osm";

        lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
        lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
        lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

        // Set map
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        // Build routing graph from map	
        auto traffic_rules = cmw->getTrafficRules();
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*cmw->getMap(), *traffic_rules.get());

        // Place ego vehicle in left lane on route with shortest path 164->165->171->109->106
        lanelet::ConstLanelet starting_lanelet = cmw->getMap()->laneletLayer.get(164); 
        lanelet::ConstLanelet ending_lanelet = cmw->getMap()->laneletLayer.get(106);
        lanelet::Optional<lanelet::routing::Route> optional_route = map_graph->getRoute(starting_lanelet, ending_lanelet);
        lanelet::routing::Route route = std::move(*optional_route);
        carma_wm::LaneletRoutePtr route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
        cmw->setRoute(route_ptr);

        // Set worker_node's world model to cmw
        worker_node->wm_ = cmw;

        //**********************//
        // TEST 1: Verify correct ego vehicle lane-following maneuver plan in non-shortest-path lanelets on route when in 
        //         WAITING_FOR_APPROACHING_ERV state.
        //**********************//

        // Create plan maneuvers service request for worker_node
        auto req = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Request>();

        // Set maneuver plan service request's vehicle state parameters to place ego vehicle in right lane to verify that it can remain off its
        //     shortest path (but still on its route) when generating a plan that keeps the ego vehicle in its lane.
        req->veh_x = 585277.966793666; // Lanelet 167 (right adjacent to 164)
        req->veh_y = 5460258.19308606; // Lanelet 167 (right adjacent to 164)
        req->veh_downtrack = 88.11799880; // (meters) Matches downtrack of (veh_x, veh_y) on ego vehicle's route
        req->veh_logitudinal_velocity = 10.0; // (m/s)
        req->veh_lane_id = "167";

        auto resp = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();
        auto header_srv = std::make_shared<rmw_request_id_t>();
        worker_node->plan_maneuvers_callback(header_srv, req, resp);

        ASSERT_EQ(resp->new_plan.maneuvers.size(), 3);

        // Verify Maneuver 0 lane following parameters in lanelet 167
        ASSERT_EQ(resp->new_plan.maneuvers[0].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[0].lane_following_maneuver.start_time).seconds(), 0.0, 0.01); 
        ASSERT_NEAR(resp->new_plan.maneuvers[0].lane_following_maneuver.end_dist, 190.243, 0.01);
        ASSERT_NEAR(resp->new_plan.maneuvers[0].lane_following_maneuver.end_speed, 5.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[0].lane_following_maneuver.end_time).seconds(), 13.6167, 0.01); 
        std::vector<std::string> maneuver_0_lane_id {"167"};
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.lane_ids, maneuver_0_lane_id); 
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.planning_tactical_plugin, "inlanecruising_plugin");

        // Verify Maneuver 1 lane following parameters in lanelet 169
        ASSERT_EQ(resp->new_plan.maneuvers[1].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[1].lane_following_maneuver.start_time).seconds(), 13.6167, 0.01); 
        ASSERT_NEAR(resp->new_plan.maneuvers[1].lane_following_maneuver.end_dist, 206.164, 0.01);
        ASSERT_NEAR(resp->new_plan.maneuvers[1].lane_following_maneuver.end_speed, 5.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[1].lane_following_maneuver.end_time).seconds(), 16.8008, 0.01); 
        std::vector<std::string> maneuver_1_lane_id {"169"};
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_following_maneuver.lane_ids, maneuver_1_lane_id); 
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_following_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_following_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_following_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_following_maneuver.parameters.planning_tactical_plugin, "inlanecruising_plugin");

        // Verify Maneuver 2 lane following parameters in lanelet 168
        ASSERT_EQ(resp->new_plan.maneuvers[2].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[2].lane_following_maneuver.start_time).seconds(), 16.8008, 0.01); 
        ASSERT_NEAR(resp->new_plan.maneuvers[2].lane_following_maneuver.end_dist, 240.438, 0.01);
        ASSERT_NEAR(resp->new_plan.maneuvers[2].lane_following_maneuver.end_speed, 5.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[2].lane_following_maneuver.end_time).seconds(), 23.6558, 0.01); 
        std::vector<std::string> maneuver_2_lane_id {"168"};
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.lane_ids, maneuver_2_lane_id); 
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.parameters.planning_tactical_plugin, "inlanecruising_plugin");
    }

    TEST(Testapproaching_emergency_vehicle_plugin, testManeuverPlanWhenMovingOverForErv){
        // Create, configure, and activate worker_node (ApproachingEmergencyVehiclePlugin)
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<approaching_emergency_vehicle_plugin::ApproachingEmergencyVehiclePlugin>(options);

        worker_node->configure(); //Call configure state transition
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Verify that initial state is NO_APPROACHING_ERV
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::NO_APPROACHING_ERV);

        worker_node->has_tracked_erv_ = true;
        worker_node->tracked_erv_.seconds_until_passing = 40.0; // Set value to trigger state machine transition to 'MOVING_OVER_FOR_APPROACHING_ERV' state
        worker_node->config_.minimal_plan_duration = 15.0; // (Seconds) Maneuver plan shall be at least 25.0 seconds long to enable multiple maneuvers for this test case

        // Set georeference for worker_node so that it can convert ERV BSM lat/lon coordinates to map coordinates
        std::string proj = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";
        std_msgs::msg::String str_msg;
        str_msg.data = proj;
        std::unique_ptr<std_msgs::msg::String> msg_ptr = std::make_unique<std_msgs::msg::String>(str_msg);
        worker_node->georeferenceCallback(std::move(msg_ptr)); 

        // Create a world model object for worker_node and set its map
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>(); 

        // Set projection
        int projector_type = 0;
        std::string target_frame;
        lanelet::ErrorMessages load_errors;

        // Load map file and parameters
        std::string file = "../../install_ros2/approaching_emergency_vehicle_plugin/share/approaching_emergency_vehicle_plugin/resource/town01_vector_map_1.osm";

        lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
        lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
        lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

        // Set map
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        // Build routing graph from map	
        auto traffic_rules = cmw->getTrafficRules();
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*cmw->getMap(), *traffic_rules.get());

        // Place ego vehicle in left lane on route with shortest path 164->165->171->109->106->107
        lanelet::ConstLanelet starting_lanelet = cmw->getMap()->laneletLayer.get(164); 
        lanelet::ConstLanelet ending_lanelet = cmw->getMap()->laneletLayer.get(107);
        lanelet::Optional<lanelet::routing::Route> optional_route = map_graph->getRoute(starting_lanelet, ending_lanelet);
        lanelet::routing::Route route = std::move(*optional_route);
        carma_wm::LaneletRoutePtr route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
        cmw->setRoute(route_ptr);

        // Set worker_node's world model to cmw
        worker_node->wm_ = cmw;

        //**********************//
        // TEST 1: Ego vehicle is in rightmost lane and the approaching ERV is in the rightmost lane. Verify that ego vehicle generates
        //         left lane change maneuver when in MOVING_OVER_FOR_APPROACHING_ERV state.
        //**********************//

        // Create plan maneuvers service request for worker_node
        auto req = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Request>();
        
        worker_node->tracked_erv_.in_rightmost_lane = true;

        // Set maneuver plan service request's vehicle state parameters to place ego vehicle in right lane to verify that it can remain off its
        //     shortest path (but still on its route) when generating a plan that keeps the ego vehicle in its lane.
        req->veh_x = 585277.966793666; // Lanelet 167 (rightmost lane; right adjacent to 164)
        req->veh_y = 5460258.19308606; // Lanelet 167 (rightmost lane; right adjacent to 164)
        req->veh_downtrack = 88.11799880; // (meters) Matches downtrack of (veh_x, veh_y) on ego vehicle's route
        req->veh_logitudinal_velocity = 10.0; // (m/s)
        req->veh_lane_id = "167";

        auto resp = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();
        auto header_srv = std::make_shared<rmw_request_id_t>();
        worker_node->plan_maneuvers_callback(header_srv, req, resp);    

        ASSERT_TRUE(worker_node->has_planned_upcoming_lc_);

        // Verify Maneuver 0 lane following parameters in lanelet 167
        ASSERT_EQ(resp->new_plan.maneuvers[0].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[0].lane_following_maneuver.start_time).seconds(), 0.0, 0.01); 
        ASSERT_NEAR(resp->new_plan.maneuvers[0].lane_following_maneuver.end_dist, 190.243, 0.01);
        ASSERT_NEAR(resp->new_plan.maneuvers[0].lane_following_maneuver.end_speed, 10.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[0].lane_following_maneuver.end_time).seconds(), 10.2125, 0.01); 
        std::vector<std::string> maneuver_0_lane_id {"167"};
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.lane_ids, maneuver_0_lane_id); 
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.planning_tactical_plugin, "inlanecruising_plugin");

        // Verify Maneuver 1 lane change parameters for left lane change from lanelet 169 to lanelet 165
        ASSERT_EQ(resp->new_plan.maneuvers[1].type, carma_planning_msgs::msg::Maneuver::LANE_CHANGE);
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[1].lane_change_maneuver.start_time).seconds(), 10.2125, 0.01); 
        ASSERT_NEAR(resp->new_plan.maneuvers[1].lane_change_maneuver.start_dist, 190.243, 0.01);
        ASSERT_NEAR(resp->new_plan.maneuvers[1].lane_change_maneuver.start_speed, 10.0, 0.01); 
        ASSERT_NEAR(resp->new_plan.maneuvers[1].lane_change_maneuver.end_dist, 206.164, 0.01);
        ASSERT_NEAR(resp->new_plan.maneuvers[1].lane_change_maneuver.end_speed, 10.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[1].lane_change_maneuver.end_time).seconds(), 11.80458, 0.01); 
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_change_maneuver.starting_lane_id, "169");
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_change_maneuver.ending_lane_id, "165");
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_change_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_change_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_change_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp->new_plan.maneuvers[1].lane_change_maneuver.parameters.planning_tactical_plugin, "cooperative_lanechange");
        std::string lane_change_maneuver_id = resp->new_plan.maneuvers[1].lane_change_maneuver.parameters.maneuver_id;

        // Verify Maneuver 2 lane following parameters in lanelet 109
        ASSERT_EQ(resp->new_plan.maneuvers[2].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[2].lane_following_maneuver.start_time).seconds(), 11.80458, 0.01); 
        ASSERT_NEAR(resp->new_plan.maneuvers[2].lane_following_maneuver.start_dist, 206.164, 0.01);
        ASSERT_NEAR(resp->new_plan.maneuvers[2].lane_following_maneuver.start_speed, 10.0, 0.01); 
        ASSERT_NEAR(resp->new_plan.maneuvers[2].lane_following_maneuver.end_dist, 240.438, 0.01);
        ASSERT_NEAR(resp->new_plan.maneuvers[2].lane_following_maneuver.end_speed, 10.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp->new_plan.maneuvers[2].lane_following_maneuver.end_time).seconds(), 15.232, 0.01); 
        std::vector<std::string> maneuver_2_lane_id {"171"};
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.lane_ids, maneuver_2_lane_id); 
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp->new_plan.maneuvers[2].lane_following_maneuver.parameters.planning_tactical_plugin, "inlanecruising_plugin");

        //**********************//
        // TEST 2: Set maneuver plan service request that begins with ego vehicle in the middle of its lane change. Check that first maneuver is
        //         the same lane change maneuver from the previous maneuver plan, and also verify that lane-follow maneuvers after that lane change maneuver.
        //**********************//
        req->veh_x = 585325.842977; // Lanelet 169 (rightmost lane; right adjacent to 165)
        req->veh_y = 5460330.84628; // Lanelet 169 (rightmost lane; right adjacent to 165)
        req->veh_downtrack = 197.52434; // (meters) Matches downtrack of (veh_x, veh_y) on ego vehicle's route
        req->veh_logitudinal_velocity = 10.0; // (m/s)
        req->veh_lane_id = "169";

        auto resp2 = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();
        worker_node->plan_maneuvers_callback(header_srv, req, resp2); 

        ASSERT_EQ(resp2->new_plan.maneuvers.size(), 5);

        // Verify Maneuver 0 lane change parameters for left lane change from lanelet 169 to lanelet 165
        ASSERT_EQ(resp2->new_plan.maneuvers[0].type, carma_planning_msgs::msg::Maneuver::LANE_CHANGE);
        ASSERT_NEAR(rclcpp::Time(resp2->new_plan.maneuvers[0].lane_change_maneuver.start_time).seconds(), 0.0, 0.01); 
        ASSERT_NEAR(resp2->new_plan.maneuvers[0].lane_change_maneuver.start_dist, 190.243, 0.01);
        ASSERT_NEAR(resp2->new_plan.maneuvers[0].lane_change_maneuver.start_speed, 10.0, 0.01); 
        ASSERT_NEAR(resp2->new_plan.maneuvers[0].lane_change_maneuver.end_dist, 206.164, 0.01);
        ASSERT_NEAR(resp2->new_plan.maneuvers[0].lane_change_maneuver.end_speed, 10.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp2->new_plan.maneuvers[0].lane_change_maneuver.end_time).seconds(), 1.59208, 0.01); 
        ASSERT_EQ(resp2->new_plan.maneuvers[0].lane_change_maneuver.starting_lane_id, "169");
        ASSERT_EQ(resp2->new_plan.maneuvers[0].lane_change_maneuver.ending_lane_id, "165");
        ASSERT_EQ(resp2->new_plan.maneuvers[0].lane_change_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp2->new_plan.maneuvers[0].lane_change_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp2->new_plan.maneuvers[0].lane_change_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp2->new_plan.maneuvers[0].lane_change_maneuver.parameters.planning_tactical_plugin, "cooperative_lanechange");
        ASSERT_EQ(resp2->new_plan.maneuvers[0].lane_change_maneuver.parameters.maneuver_id, lane_change_maneuver_id);

        // Verify Maneuver 1 lane following parameters in lanelet 171
        ASSERT_EQ(resp2->new_plan.maneuvers[1].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);
        ASSERT_NEAR(rclcpp::Time(resp2->new_plan.maneuvers[1].lane_following_maneuver.start_time).seconds(), 1.59208, 0.01); 
        ASSERT_NEAR(resp2->new_plan.maneuvers[1].lane_following_maneuver.start_dist, 206.1637, 0.01);
        ASSERT_NEAR(resp2->new_plan.maneuvers[1].lane_following_maneuver.start_speed, 10.0, 0.01); 
        ASSERT_NEAR(resp2->new_plan.maneuvers[1].lane_following_maneuver.end_dist, 240.43874, 0.01);
        ASSERT_NEAR(resp2->new_plan.maneuvers[1].lane_following_maneuver.end_speed, 10.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp2->new_plan.maneuvers[1].lane_following_maneuver.end_time).seconds(), 5.01953, 0.01); 
        std::vector<std::string> maneuver_1_lane_id {"171"};
        ASSERT_EQ(resp2->new_plan.maneuvers[1].lane_following_maneuver.lane_ids, maneuver_1_lane_id); 
        ASSERT_EQ(resp2->new_plan.maneuvers[1].lane_following_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp2->new_plan.maneuvers[1].lane_following_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp2->new_plan.maneuvers[1].lane_following_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp2->new_plan.maneuvers[1].lane_following_maneuver.parameters.planning_tactical_plugin, "inlanecruising_plugin");

        // Verify Maneuver 2 lane following parameters in lanelet 109
        ASSERT_EQ(resp2->new_plan.maneuvers[2].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);
        ASSERT_NEAR(rclcpp::Time(resp2->new_plan.maneuvers[2].lane_following_maneuver.start_time).seconds(), 5.01953, 0.01); 
        ASSERT_NEAR(resp2->new_plan.maneuvers[2].lane_following_maneuver.start_dist, 240.43874, 0.01);
        ASSERT_NEAR(resp2->new_plan.maneuvers[2].lane_following_maneuver.start_speed, 10.0, 0.01); 
        ASSERT_NEAR(resp2->new_plan.maneuvers[2].lane_following_maneuver.end_dist, 255.52386, 0.01);
        ASSERT_NEAR(resp2->new_plan.maneuvers[2].lane_following_maneuver.end_speed, 10.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp2->new_plan.maneuvers[2].lane_following_maneuver.end_time).seconds(), 6.528, 0.01); 
        maneuver_2_lane_id[0] = "109";
        ASSERT_EQ(resp2->new_plan.maneuvers[2].lane_following_maneuver.lane_ids, maneuver_2_lane_id); 
        ASSERT_EQ(resp2->new_plan.maneuvers[2].lane_following_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp2->new_plan.maneuvers[2].lane_following_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp2->new_plan.maneuvers[2].lane_following_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp2->new_plan.maneuvers[2].lane_following_maneuver.parameters.planning_tactical_plugin, "inlanecruising_plugin");

        // Verify Maneuver 3 lane following parameters in lanelet 106
        ASSERT_EQ(resp2->new_plan.maneuvers[3].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);
        ASSERT_NEAR(rclcpp::Time(resp2->new_plan.maneuvers[3].lane_following_maneuver.start_time).seconds(), 6.528, 0.01); 
        ASSERT_NEAR(resp2->new_plan.maneuvers[3].lane_following_maneuver.start_dist, 255.52386, 0.01);
        ASSERT_NEAR(resp2->new_plan.maneuvers[3].lane_following_maneuver.start_speed, 10.0, 0.01); 
        ASSERT_NEAR(resp2->new_plan.maneuvers[3].lane_following_maneuver.end_dist, 554.941, 0.01);
        ASSERT_NEAR(resp2->new_plan.maneuvers[3].lane_following_maneuver.end_speed, 10.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp2->new_plan.maneuvers[3].lane_following_maneuver.end_time).seconds(), 36.4697, 0.01); 
        std::vector<std::string> maneuver_3_lane_id {"106"};
        ASSERT_EQ(resp2->new_plan.maneuvers[3].lane_following_maneuver.lane_ids, maneuver_3_lane_id); 
        ASSERT_EQ(resp2->new_plan.maneuvers[3].lane_following_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp2->new_plan.maneuvers[3].lane_following_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp2->new_plan.maneuvers[3].lane_following_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp2->new_plan.maneuvers[3].lane_following_maneuver.parameters.planning_tactical_plugin, "inlanecruising_plugin");

        // Verify Maneuver 4 stop and wait maneuver spans lanelets 106 and 107
        ASSERT_EQ(resp2->new_plan.maneuvers[4].type, carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT);
        ASSERT_NEAR(rclcpp::Time(resp2->new_plan.maneuvers[4].stop_and_wait_maneuver.start_time).seconds(), 36.4697, 0.01); 
        ASSERT_NEAR(resp2->new_plan.maneuvers[4].stop_and_wait_maneuver.start_dist, 554.941, 0.01);
        ASSERT_NEAR(resp2->new_plan.maneuvers[4].stop_and_wait_maneuver.start_speed, 10.0, 0.01); 
        ASSERT_NEAR(resp2->new_plan.maneuvers[4].stop_and_wait_maneuver.end_dist, 604.941, 0.01);
        ASSERT_NEAR(rclcpp::Time(resp2->new_plan.maneuvers[4].stop_and_wait_maneuver.end_time).seconds(), 46.4697, 0.01); 
        ASSERT_EQ(resp2->new_plan.maneuvers[4].stop_and_wait_maneuver.starting_lane_id, "106"); 
        ASSERT_EQ(resp2->new_plan.maneuvers[4].stop_and_wait_maneuver.ending_lane_id, "107"); 
        ASSERT_EQ(resp2->new_plan.maneuvers[4].stop_and_wait_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp2->new_plan.maneuvers[4].stop_and_wait_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp2->new_plan.maneuvers[4].stop_and_wait_maneuver.parameters.planning_tactical_plugin, "stop_and_wait_plugin");

        ASSERT_EQ(worker_node->has_planned_upcoming_lc_, true);

        //**********************//
        // TEST 3: Set maneuver plan service request that begins with ego vehicle positioned after its lane change. Check that first maneuver is
        //         a lane follow maneuver. Verify that all maneuvers include the expected parameters.
        //**********************//
        req->veh_x = 585314.2535; // Lanelet 169 (rightmost lane; right adjacent to 165)
        req->veh_y = 5460392.8554; // Lanelet 169 (rightmost lane; right adjacent to 165)
        req->veh_downtrack = 260.411; // (meters) Matches downtrack of (veh_x, veh_y) on ego vehicle's route
        req->veh_logitudinal_velocity = 10.0; // (m/s)
        req->veh_lane_id = "106";

        auto resp3 = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();
        worker_node->plan_maneuvers_callback(header_srv, req, resp3); 

        // Verify that node no longer has a planned upcoming lane change
        ASSERT_FALSE(worker_node->has_planned_upcoming_lc_);

        ASSERT_EQ(resp3->new_plan.maneuvers.size(), 2);

        // Verify Maneuver 0 lane following parameters in lanelet 106
        ASSERT_EQ(resp3->new_plan.maneuvers[0].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);
        ASSERT_NEAR(rclcpp::Time(resp3->new_plan.maneuvers[0].lane_following_maneuver.start_time).seconds(), 0.0, 0.01); 
        ASSERT_NEAR(resp3->new_plan.maneuvers[0].lane_following_maneuver.start_dist, 260.411, 0.01);
        ASSERT_NEAR(resp3->new_plan.maneuvers[0].lane_following_maneuver.start_speed, 10.0, 0.01); 
        ASSERT_NEAR(resp3->new_plan.maneuvers[0].lane_following_maneuver.end_dist, 554.941, 0.01);
        ASSERT_NEAR(resp3->new_plan.maneuvers[0].lane_following_maneuver.end_speed, 10.0, 0.01); 
        ASSERT_NEAR(rclcpp::Time(resp3->new_plan.maneuvers[0].lane_following_maneuver.end_time).seconds(), 29.453, 0.01); 
        maneuver_0_lane_id[0] = "106";
        ASSERT_EQ(resp3->new_plan.maneuvers[0].lane_following_maneuver.lane_ids, maneuver_0_lane_id); 
        ASSERT_EQ(resp3->new_plan.maneuvers[0].lane_following_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp3->new_plan.maneuvers[0].lane_following_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp3->new_plan.maneuvers[0].lane_following_maneuver.parameters.presence_vector, carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN);
        ASSERT_EQ(resp3->new_plan.maneuvers[0].lane_following_maneuver.parameters.planning_tactical_plugin, "inlanecruising_plugin");

        // Verify Maneuver 1 stop and wait maneuver spans lanelets 106 and 107
        ASSERT_EQ(resp3->new_plan.maneuvers[1].type, carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT);
        ASSERT_NEAR(rclcpp::Time(resp3->new_plan.maneuvers[1].stop_and_wait_maneuver.start_time).seconds(), 29.453, 0.01); 
        ASSERT_NEAR(resp3->new_plan.maneuvers[1].stop_and_wait_maneuver.start_dist, 554.941, 0.01);
        ASSERT_NEAR(resp3->new_plan.maneuvers[1].stop_and_wait_maneuver.start_speed, 10.0, 0.01); 
        ASSERT_NEAR(resp3->new_plan.maneuvers[1].stop_and_wait_maneuver.end_dist, 604.941, 0.01);
        ASSERT_NEAR(rclcpp::Time(resp3->new_plan.maneuvers[1].stop_and_wait_maneuver.end_time).seconds(), 39.453, 0.01); 
        ASSERT_EQ(resp3->new_plan.maneuvers[1].stop_and_wait_maneuver.starting_lane_id, "106"); 
        ASSERT_EQ(resp3->new_plan.maneuvers[1].stop_and_wait_maneuver.ending_lane_id, "107"); 
        ASSERT_EQ(resp3->new_plan.maneuvers[1].stop_and_wait_maneuver.parameters.negotiation_type, carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION);
        ASSERT_EQ(resp3->new_plan.maneuvers[1].stop_and_wait_maneuver.parameters.planning_strategic_plugin, "approaching_emergency_vehicle_plugin");
        ASSERT_EQ(resp3->new_plan.maneuvers[1].stop_and_wait_maneuver.parameters.planning_tactical_plugin, "stop_and_wait_plugin");
    }

    TEST(Testapproaching_emergency_vehicle_plugin, testWarningBroadcast){
        // Create, configure, and activate worker_node (ApproachingEmergencyVehiclePlugin)
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<approaching_emergency_vehicle_plugin::ApproachingEmergencyVehiclePlugin>(options);

        worker_node->configure(); //Call configure state transition
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Verify that initial state is NO_APPROACHING_ERV
        ASSERT_EQ(worker_node->transition_table_.getState(), ApproachingEmergencyVehicleState::NO_APPROACHING_ERV);

        worker_node->has_tracked_erv_ = true;
        worker_node->tracked_erv_.seconds_until_passing = 40.0; // Set value to trigger state machine transition to 'MOVING_OVER_FOR_APPROACHING_ERV' state
        worker_node->config_.minimal_plan_duration = 15.0; // (Seconds) Maneuver plan shall be at least 25.0 seconds long to enable multiple maneuvers for this test case

        // Set georeference for worker_node so that it can convert ERV BSM lat/lon coordinates to map coordinates
        std::string proj = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";
        std_msgs::msg::String str_msg;
        str_msg.data = proj;
        std::unique_ptr<std_msgs::msg::String> msg_ptr = std::make_unique<std_msgs::msg::String>(str_msg);
        worker_node->georeferenceCallback(std::move(msg_ptr)); 

        // Create a world model object for worker_node and set its map
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>(); 

        // Set projection
        int projector_type = 0;
        std::string target_frame;
        lanelet::ErrorMessages load_errors;

        // Load map file and parameters
        std::string file = "../../install_ros2/approaching_emergency_vehicle_plugin/share/approaching_emergency_vehicle_plugin/resource/town01_vector_map_1.osm";

        lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
        lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
        lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

        // Set map
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        // Build routing graph from map	
        auto traffic_rules = cmw->getTrafficRules();
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*cmw->getMap(), *traffic_rules.get());

        // Place ego vehicle in left lane on route with shortest path 164->165->171->109->106->107
        lanelet::ConstLanelet starting_lanelet = cmw->getMap()->laneletLayer.get(164); 
        lanelet::ConstLanelet ending_lanelet = cmw->getMap()->laneletLayer.get(107);
        lanelet::Optional<lanelet::routing::Route> optional_route = map_graph->getRoute(starting_lanelet, ending_lanelet);
        lanelet::routing::Route route = std::move(*optional_route);
        carma_wm::LaneletRoutePtr route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
        cmw->setRoute(route_ptr);

        // Set worker_node's world model to cmw
        worker_node->wm_ = cmw;
        
        // Set configuration parameters relevant to this unit test
        worker_node->config_.passing_threshold = 10.0; // Seconds
        worker_node->config_.warning_broadcast_frequency = 1; // Hz
        worker_node->config_.max_warning_broadcasts = 3;

        worker_node->has_tracked_erv_ = true;
        worker_node->tracked_erv_.in_rightmost_lane = true; // Plan maneuvers request will place ego vehicle in rightmost lane as well
        worker_node->tracked_erv_.seconds_until_passing = 10.0;

        // Create plan maneuvers service request for worker_node
        auto req = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Request>();

        // Set maneuver plan service request's vehicle state parameters to place ego vehicle in right lane to verify that it can remain off its
        //     shortest path (but still on its route) when generating a plan that keeps the ego vehicle in its lane.
        req->veh_x = 585277.966793666; // Lanelet 167 (rightmost lane; right adjacent to 164)
        req->veh_y = 5460258.19308606; // Lanelet 167 (rightmost lane; right adjacent to 164)
        req->veh_downtrack = 88.11799880; // (meters) Matches downtrack of (veh_x, veh_y) on ego vehicle's route
        req->veh_logitudinal_velocity = 10.0; // (m/s)
        req->veh_lane_id = "167";

        auto resp = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();
        auto header_srv = std::make_shared<rmw_request_id_t>();
        worker_node->plan_maneuvers_callback(header_srv, req, resp);   

        // Set parameters so that ERV remains tracked throughout this unit test
        worker_node->tracked_erv_.latest_update_time = worker_node->now(); 
        worker_node->config_.timeout_duration = 30.0; // (Seconds) Increase timeout duration so that ERV is still tracked throughout this unit test
        
        // Verify that the state transition resulting from the plan maneuvers request triggers the node to set should_broadcast_warnings to true
        ASSERT_EQ(worker_node->num_warnings_broadcasted_, 0);
        ASSERT_TRUE(worker_node->should_broadcast_warnings_);

        // Add Node to an executor and spin it to trigger timer callbacks
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(worker_node->get_node_base_interface());

        // Spin executor for 1 second
        auto end_time = std::chrono::system_clock::now() + std::chrono::seconds(1);
        while(std::chrono::system_clock::now() < end_time){
            executor.spin_once();
        }
        
        // Verify that node has broadcasted 2 (of 3) warning messages
        ASSERT_EQ(worker_node->num_warnings_broadcasted_, 2);

        // Spin executor for 2 seconds
        end_time = std::chrono::system_clock::now() + std::chrono::seconds(2);
        while(std::chrono::system_clock::now() < end_time){
            executor.spin_once();
        }

        // Verify that node has broadcasted all warning messages and counter has been reset to 0
        ASSERT_EQ(worker_node->num_warnings_broadcasted_, 0);
        ASSERT_FALSE(worker_node->should_broadcast_warnings_); // Node should no longer be broadcasting warnings since all have been broadcasted
    }

    TEST(Testapproaching_emergency_vehicle_plugin, testApproachingErvStatusMessage){

        // Create, configure, and activate worker_node (ApproachingEmergencyVehiclePlugin)
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<approaching_emergency_vehicle_plugin::ApproachingEmergencyVehiclePlugin>(options);

        worker_node->configure(); //Call configure state transition
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Generate status message after initial node activation (no ERV is being tracked) and verify the output
        carma_msgs::msg::UIInstructions status_msg = worker_node->generateApproachingErvStatusMessage();
        ASSERT_EQ(status_msg.type, carma_msgs::msg::UIInstructions::INFO);
        ASSERT_EQ(status_msg.msg.substr(0,22), "HAS_APPROACHING_ERV:0,");

        // Generate status message when an ERV is being tracked in state MOVING_OVER_FOR_APPROACHING_ERV (left lane change) and verify the output
        worker_node->has_tracked_erv_ = true;
        worker_node->tracked_erv_.seconds_until_passing = 11.56342;
        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::APPROACHING_ERV_IN_PATH);
        worker_node->has_planned_upcoming_lc_ = true;
        worker_node->upcoming_lc_params_.is_right_lane_change = false;

        status_msg = worker_node->generateApproachingErvStatusMessage();
        ASSERT_EQ(status_msg.type, carma_msgs::msg::UIInstructions::INFO);
        ASSERT_EQ(status_msg.msg, "HAS_APPROACHING_ERV:1,TIME_UNTIL_PASSING:11.6,EGO_VEHICLE_ACTION:Changing lanes to the left.");

        // Generate status message when an ERV is being tracked in state MOVING_OVER_FOR_APPROACHING_ERV (right lane change) and verify the output
        worker_node->has_planned_upcoming_lc_ = true;
        worker_node->upcoming_lc_params_.is_right_lane_change = true;

        status_msg = worker_node->generateApproachingErvStatusMessage();
        ASSERT_EQ(status_msg.type, carma_msgs::msg::UIInstructions::INFO);
        ASSERT_EQ(status_msg.msg, "HAS_APPROACHING_ERV:1,TIME_UNTIL_PASSING:11.6,EGO_VEHICLE_ACTION:Changing lanes to the right.");

        // Generate status message when an ERV is being tracked in state WAITING_FOR_APPROACHING_ERV and verify the output
        worker_node->has_tracked_erv_ = true;
        worker_node->tracked_erv_.seconds_until_passing = 11.563429203;
        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_ABOUT_TO_PASS_IN_PATH);

        status_msg = worker_node->generateApproachingErvStatusMessage();
        ASSERT_EQ(status_msg.type, carma_msgs::msg::UIInstructions::INFO);
        ASSERT_EQ(status_msg.msg, "HAS_APPROACHING_ERV:1,TIME_UNTIL_PASSING:11.6,EGO_VEHICLE_ACTION:Remaining in the current lane at the speed limit.");

        // Generate status message when an ERV is being tracked in state SLOWING_DOWN_FOR_ERV with ego vehicle travelling significantly above the target reduced speed, and verify the output
        worker_node->transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_PASSING_IN_PATH);
        worker_node->tracked_erv_.seconds_until_passing = 1.578;

        carma_planning_msgs::msg::ManeuverPlan maneuver_plan;
        carma_planning_msgs::msg::Maneuver maneuver;
        maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        maneuver.lane_following_maneuver.end_speed = 6.7056; // (m/s) equal to 15 mph
        maneuver_plan.maneuvers.push_back(maneuver);
        worker_node->latest_maneuver_plan_ = maneuver_plan;

        worker_node->current_speed_ = 15.0; // (m/s), not within threshold (config_.reduced_speed_buffer) of first maneuver's target speed (end_speed)

        status_msg = worker_node->generateApproachingErvStatusMessage();
        ASSERT_EQ(status_msg.type, carma_msgs::msg::UIInstructions::INFO);
        ASSERT_EQ(status_msg.msg, "HAS_APPROACHING_ERV:1,TIME_UNTIL_PASSING:1.6,EGO_VEHICLE_ACTION:Remaining in the current lane and slowing down to a reduced speed of 15 mph.");

        // Generate status message when an ERV is being tracked in state SLOWING_DOWN_FOR_ERV with ego vehicle travelling near the the target reduced speed, and verify the output
        worker_node->current_speed_ = 6.8; // (m/s), within threshold (config_.reduced_speed_buffer) of first maneuver's target speed (end_speed)

        status_msg = worker_node->generateApproachingErvStatusMessage();
        ASSERT_EQ(status_msg.type, carma_msgs::msg::UIInstructions::INFO);
        ASSERT_EQ(status_msg.msg, "HAS_APPROACHING_ERV:1,TIME_UNTIL_PASSING:1.6,EGO_VEHICLE_ACTION:Remaining in the current lane at a reduced speed of 15 mph.");

        // Generate status message when an ERV is being tracked in state SLOWING_DOWN_FOR_ERV without a generated maneuver plan, and verify an exception is thrown
        carma_planning_msgs::msg::ManeuverPlan empty_maneuver_plan;
        worker_node->latest_maneuver_plan_ = empty_maneuver_plan;

        EXPECT_THROW(worker_node->generateApproachingErvStatusMessage(), std::invalid_argument);
    }

} // namespace approaching_emergency_vehicle_plugin

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