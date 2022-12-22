/*
 * Copyright (C) 2022 LEIDOS.
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
        boost::optional<lanelet::ConstLanelet> intersecting_lanelet = worker_node->getRouteIntersectingLanelet(erv_future_route.get(), ego_shortest_path);
        ASSERT_TRUE(intersecting_lanelet);
        ASSERT_EQ((*intersecting_lanelet).id(), 1212);

        // Create ERV's current position in the map frame (first point on centerline of lanelet 1210)
        lanelet::ConstLineString2d lanelet_1210_centerline = lanelet::utils::to2D(lanelet_1210.centerline());
        lanelet::BasicPoint2d erv_current_position_in_map = lanelet_1210_centerline.front();

        // Set ERV speed faster than ego speed so that ERV will eventually pass the ego vehicle
        worker_node->current_speed_ = 10.0; // Set ego vehicle's current speed to 10 m/s
        double erv_speed = 20.0; // Set ERV's current speed to 20 m/s

        // Verify that ERV will pass ego vehicle in ~5 seconds (ERV is 50 meters behind ego vehicle and travelling 10 m/s faster)
        double seconds_until_passing = worker_node->getSecondsUntilPassing(erv_future_route, erv_current_position_in_map, erv_speed, *intersecting_lanelet);
        ASSERT_NEAR(seconds_until_passing, 5.0, 0.01);
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

        // Trigger a timeout on the currently tracked ERV and verify that the plugin no longer has an approaching ERV
        std::this_thread::sleep_for(std::chrono::seconds(10));
        worker_node->checkForErvTimeout();
        ASSERT_FALSE(worker_node->has_tracked_erv_);
    }

    // Test ERV BSM that times out
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