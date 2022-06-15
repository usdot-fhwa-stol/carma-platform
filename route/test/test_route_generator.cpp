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
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Traits.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>

#include "route/route_generator_worker.hpp"
#include "route/route_state_worker.hpp"
#include "route/route_node.hpp"

TEST(RouteGeneratorTest, testRouteVisualizerCenterLineParser)
{
    // Create a RouteGeneratorWorker for this test
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
    tf2_ros::Buffer tf2_buffer(clock->get_clock());
    route::RouteGeneratorWorker worker(tf2_buffer);
    worker.setLoggerInterface(node->get_node_logging_interface());

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    // If the output is an error about the geoReference field in the osm file, here is a correct example. If the lat/lon coordinates are already correct, simply add:
    // <geoReference>+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs</geoReference>

    // File location of osm file
    std::string file = "../../install_ros2/route/share/route/resource/map/vector_map.osm";   
    // Starting and ending lanelet IDs. It's easiest to grab these from JOSM
    lanelet::Id start_id = 1346;
    lanelet::Id end_id = 1351;

    // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

    // Grabs lanelet elements from the start and end IDs. Fails the unit test if there is no lanelet with the matching ID
    lanelet::Lanelet start_lanelet;
    lanelet::Lanelet end_lanelet;

    try {
        start_lanelet = map->laneletLayer.get(start_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified starting lanelet Id of " << start_id << " does not exist in the provided map.";
    }
    try {
        end_lanelet = map->laneletLayer.get(end_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified ending lanelet Id of " << end_id << " does not exist in the provided map.";
    }

    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
    
    // Create MarkerArray to test

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Time(0,0);
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;//
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "route_visualizer";

    marker.scale.x = 0.65;
    marker.scale.y = 0.65;
    marker.scale.z = 0.65;
    marker.frame_locked = true;

    marker.id = 0;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;

    geometry_msgs::msg::Point p1;
    p1.x = start_lanelet.centerline3d().front().x();
    p1.y = start_lanelet.centerline3d().front().y();

    marker.points.push_back(p1);

    geometry_msgs::msg::Point p2;
    p2.x = start_lanelet.centerline3d().back().x();
    p2.y = start_lanelet.centerline3d().back().y();

    marker.points.push_back(p2);

    // Computes the shortest path and prints the list of lanelet IDs to get from the start to the end. Can be manually confirmed in JOSM
    auto route = map_graph->getRoute(start_lanelet, end_lanelet);
    if(!route) {
        ASSERT_FALSE(true);
        std::cout << "Route not generated." << " ";
    } else {
        std::cout << "shortest path: \n";
        for(const auto& ll : route.get().shortestPath()) {
            std::cout << ll.id() << " ";
        }
        std::cout << "\n";
        auto test_msg = worker.composeRouteMarkerMsg(route);
        ASSERT_EQ(marker.points.size(), test_msg.points.size());
        EXPECT_NEAR(marker.points[0].x, test_msg.points[0].x, 10.0);
        EXPECT_NEAR(marker.points[0].y, test_msg.points[0].y, 10.0);
        EXPECT_NEAR(marker.points[1].x, test_msg.points[1].x, 10.0);
        EXPECT_NEAR(marker.points[1].y, test_msg.points[1].y, 10.0);
    }
}

TEST(RouteGeneratorTest, testLaneletRoutingVectorMap)
{
    // Create a RouteGeneratorWorker for this test
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
    tf2_ros::Buffer tf2_buffer(clock->get_clock());
    route::RouteGeneratorWorker worker(tf2_buffer);
    worker.setLoggerInterface(node->get_node_logging_interface());

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    // If the output is an error about the geoReference field in the osm file, here is a correct example. If the lat/lon coordinates are already correct, simply add:
    // <geoReference>+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs</geoReference>

    // File location of osm file
    std::string file = "../../install_ros2/route/share/route/resource/map/vector_map.osm";   
    // Starting and ending lanelet IDs. It's easiest to grab these from JOSM
    lanelet::Id start_id = 1346;
    lanelet::Id end_id = 1349;

    // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

    // Grabs lanelet elements from the start and end IDs. Fails the unit test if there is no lanelet with the matching ID
    lanelet::Lanelet start_lanelet;
    lanelet::Lanelet end_lanelet;

    try {
        start_lanelet = map->laneletLayer.get(start_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified starting lanelet Id of " << start_id << " does not exist in the provided map.";
    }
    try {
        end_lanelet = map->laneletLayer.get(end_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified ending lanelet Id of " << end_id << " does not exist in the provided map.";
    }

    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
    // Output graph for debugging
    map_graph->exportGraphViz("../routing.txt");

    // Computes the shortest path and prints the list of lanelet IDs to get from the start to the end. Can be manually confirmed in JOSM
    auto route = map_graph->getRoute(start_lanelet, end_lanelet);
    if(!route) {
        ASSERT_FALSE(true);
        std::cout << "Route not generated." << " ";
    } else {
        std::cout << "shortest path: \n";
        for(const auto& ll : route.get().shortestPath()) {
            std::cout << ll.id() << " ";
        }
        std::cout << "\n";
        carma_planning_msgs::msg::Route route_msg_ = worker.composeRouteMsg(route);


        ASSERT_TRUE(route_msg_.shortest_path_lanelet_ids.size() > 0);
        ASSERT_TRUE(route_msg_.route_path_lanelet_ids.size() > 0);
    }
}

TEST(RouteGeneratorTest, testLaneletRoutingTown02VectorMap)
{
    // Create a RouteGeneratorWorker for this test
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
    tf2_ros::Buffer tf2_buffer(clock->get_clock());
    route::RouteGeneratorWorker worker(tf2_buffer);
    worker.setLoggerInterface(node->get_node_logging_interface());

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    // If the output is an error about the geoReference field in the osm file, here is a correct example. If the lat/lon coordinates are already correct, simply add:
    // <geoReference>+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs</geoReference>

    // File location of osm file
    std::string file = "../../install_ros2/route/share/route/resource/map/town01_vector_map_1.osm";
    // Starting and ending lanelet IDs. It's easiest to grab these from JOSM
    lanelet::Id start_id = 101;
    lanelet::Id end_id = 111;
    /***
     * VALID PATHs (consists of lanenet ids): (This is also the shortest path because certain Lanelets missing)
     * 159->160->164->136->135->137->144->121; 
     * 159->160->164->136->135->137->144->118;
     * 168->170->111
     * 159->161->168->170->111
     * 167->169->168->170->111
     * 115->146->140->139->143->167->169->168->170->111 
     * 141->139->143->167->169->168->170->111 
     * 127->146->140->139->143->167->169->168->170->111 
     * 101->100->104->167->169->168->170->111 (a counter cLock circle) 
     * **/

    // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

    // Grabs lanelet elements from the start and end IDs. Fails the unit test if there is no lanelet with the matching ID
    lanelet::Lanelet start_lanelet;
    lanelet::Lanelet end_lanelet;

    try 
    {
        //get lanelet layer
        start_lanelet = map->laneletLayer.get(start_id);        
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified starting lanelet Id of " << start_id << " does not exist in the provided map.";
    }
    try {
        end_lanelet = map->laneletLayer.get(end_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified ending lanelet Id of " << end_id << " does not exist in the provided map.";
    }

    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
    // Output graph for debugging
    map_graph->exportGraphViz("../routing2.txt");

    // Computes the shortest path and prints the list of lanelet IDs to get from the start to the end. Can be manually confirmed in JOSM
    const auto route = map_graph->getRoute(start_lanelet, end_lanelet);
    if(!route) {
        ASSERT_FALSE(true);
        std::cout << "Route not generated." << " ";
    } else {
        std::cout << "shortest path: \n";
        for(const auto& ll : route.get().shortestPath()) {
            std::cout << ll.id() << " ";
        }
        std::cout << "\n";
        carma_planning_msgs::msg::Route route_msg_ = worker.composeRouteMsg(route);
        ASSERT_TRUE(route_msg_.shortest_path_lanelet_ids.size() > 0);
        ASSERT_TRUE(route_msg_.route_path_lanelet_ids.size() > 0);
    }
}

TEST(RouteGeneratorTest, test_crosstrack_error_check)
{
    // Create a RouteGeneratorWorker for this test
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
    tf2_ros::Buffer tf2_buffer(clock->get_clock());
    route::RouteGeneratorWorker worker(tf2_buffer);
    worker.setLoggerInterface(node->get_node_logging_interface());

    std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    //Create route msg
    carma_planning_msgs::msg::Route route_msg;

   // File location of osm file
    std::string file = "../../install_ros2/route/share/route/resource/map/town01_vector_map_1.osm";
     // Starting and ending lanelet IDs. It's easiest to grab these from JOSM
    lanelet::Id start_id = 101;
    lanelet::Id end_id = 111;

    //Load map parameters

    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);
    cmw->carma_wm::CARMAWorldModel::setMap(map);

     // Grabs lanelet elements from the start and end IDs. Fails the unit test if there is no lanelet with the matching ID
    lanelet::Lanelet start_lanelet;
    lanelet::Lanelet end_lanelet;

    try 
    {
        //get lanelet layer
        start_lanelet = map->laneletLayer.get(start_id);        
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified starting lanelet Id of " << start_id << " does not exist in the provided map.";
    }
    try {
        end_lanelet = map->laneletLayer.get(end_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified ending lanelet Id of " << end_id << " does not exist in the provided map.";
    }
    
    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
    const auto route = map_graph->getRoute(start_lanelet, end_lanelet);
    route_msg = worker.composeRouteMsg(route);
    ASSERT_TRUE(route_msg.route_path_lanelet_ids.size() > 0);
    
    //Test 1: Vehicle is out of bounds//

    //Assign vehicle position
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;

    worker.setWorldModelPtr(cmw);
    worker.setCrosstrackErrorCountMax(0);
    worker.setCrosstrackErrorDistance(1.0);
    
    std::shared_ptr<geometry_msgs::msg::PoseStamped> mpt(new geometry_msgs::msg::PoseStamped(msg));

    //Compare vehicle position to the route bounds //

    lanelet::BasicPoint2d current_loc(mpt->pose.position.x, mpt->pose.position.y);

    auto current_lanelet = worker.getClosestLaneletFromRouteLanelets(current_loc);

    // worker.pose_cb(mpt);

    lanelet::BasicPoint2d position;
    position.x()= msg.pose.position.x;
    position.y()= msg.pose.position.y;

    ASSERT_EQ(boost::geometry::within(position, start_lanelet.polygon2d()), false);
    ASSERT_EQ(worker.crosstrackErrorCheck(mpt, start_lanelet), true); //The vehicle will show crosstrack error, so the value should return true
    
    //Test 2: Vehicle is in bounds, no crosstrack error//

    //Use position values to show the case when there is no crosstrack error
    worker.setCrosstrackErrorDistance(1.0);

    //Assign vehicle position
    msg.pose.position.x = -9.45542;
    msg.pose.position.y = -182.324;

    position.x()= msg.pose.position.x;
    position.y()= msg.pose.position.y;
    
    std::shared_ptr<geometry_msgs::msg::PoseStamped> mpt2(new geometry_msgs::msg::PoseStamped(msg));

    ASSERT_EQ(boost::geometry::within(position, start_lanelet.polygon2d()), true);
    ASSERT_EQ(worker.crosstrackErrorCheck(mpt2, start_lanelet), false);

    //Test 3: Vehicle is out of bounds, and has exceeded the maximum number of consecutive timesteps outside of the route allowable before triggering LEFT_ROUTE//
    worker.setCrosstrackErrorCountMax(1);
    worker.setCrosstrackErrorDistance(1.0);

    position.x()= 0.0;
    position.y()= 0.0;

    ASSERT_EQ(boost::geometry::within(position, start_lanelet.polygon2d()), false);
    ASSERT_EQ(worker.crosstrackErrorCheck(mpt, start_lanelet), false); //The vehicle will show no crosstrack error, so the value should return false
    ASSERT_EQ(worker.crosstrackErrorCheck(mpt, start_lanelet), true); //The vehicle will show crosstrack error, so the value should return true
}

TEST(RouteGeneratorTest, test_set_active_route_cb)
{
    ////////////
    // Conduct tests for set_active_route_cb() using a route .csv file
    ////////////

    // Create a RouteGeneratorWorker for this test
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
    tf2_ros::Buffer tf2_buffer(clock->get_clock());
    route::RouteGeneratorWorker worker(tf2_buffer);
    worker.setLoggerInterface(node->get_node_logging_interface());

    std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();    
    worker.setRouteFilePath("../../install_ros2/route/share/route/resource/route/");
    worker.setWorldModelPtr(cmw);
    
    // Set projection
    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    // Load map file and parameters
    std::string file = "../../install_ros2/route/share/route/resource/map/town01_vector_map_1.osm";

    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

    // Set map
    cmw->carma_wm::CARMAWorldModel::setMap(map);

    // Set georeference
    std::string proj = "+proj=tmerc +lat_0=4.9000000000000000e+1 +lon_0=8.0000000000000000e+0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";
    std_msgs::msg::String str_msg;
    str_msg.data = proj;
    std::unique_ptr<std_msgs::msg::String> msg_ptr = std::make_unique<std_msgs::msg::String>(str_msg);
    worker.georeferenceCb(move(msg_ptr));  

    // Conduct tests for getAvailableRouteCb()
    std::shared_ptr<rmw_request_id_t> header;
    carma_planning_msgs::srv::GetAvailableRoutes::Request req;
    carma_planning_msgs::srv::GetAvailableRoutes::Response resp;
    auto req_ptr = std::make_shared<carma_planning_msgs::srv::GetAvailableRoutes::Request>(req);
    auto resp_ptr = std::make_shared<carma_planning_msgs::srv::GetAvailableRoutes::Response>(resp);
    
    ASSERT_TRUE(worker.getAvailableRouteCb(header, req_ptr, resp_ptr));
    ASSERT_EQ(5, resp_ptr->available_routes.size());

    for(auto i = 0; i < resp_ptr->available_routes.size();i++)    
    {
        if(resp_ptr->available_routes[i].route_id  == "Test_town01_route_1")
        {
            ASSERT_EQ("DEST3", resp_ptr->available_routes[i].route_name);
            auto gps_points = worker.loadRouteDestinationGpsPointsFromRouteId("Test_town01_route_1");
            auto map_points = worker.loadRouteDestinationsInMapFrame(gps_points);

            // TODO: temporarily disabled since map isnt loaded properly
            // ASSERT_EQ(3, map_points.size()); 
            // ASSERT_NEAR(-9.45542, map_points[0].x(), 0.001);
            // ASSERT_NEAR(-182.324, map_points[0].y(), 0.001);  
            // ASSERT_NEAR(72, map_points[0].z(), 0.001);
        }
    }
    
    //Assign vehicle position
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x = -9.45542;
    msg.pose.position.y = -182.324;
    //geometry_msgs::PoseStampedPtr mpt(new geometry_msgs::PoseStamped(msg));
    worker.vehicle_pose_ = msg;

    carma_planning_msgs::srv::SetActiveRoute::Request req2;
    carma_planning_msgs::srv::SetActiveRoute::Response resp2;
    for(auto i: resp_ptr->available_routes)
    {
        if(i.route_id  == "Test_town01_route_1")
        {
            req2.route_id = i.route_id;
            req2.choice = carma_planning_msgs::srv::SetActiveRoute::Request::ROUTE_ID;

            auto req_ptr2 = std::make_shared<carma_planning_msgs::srv::SetActiveRoute::Request>(req2);
            auto resp_ptr2 = std::make_shared<carma_planning_msgs::srv::SetActiveRoute::Response>(resp2);
            ASSERT_EQ(worker.setActiveRouteCb(header, req_ptr2, resp_ptr2), true);
            // TODO: temporarily disabled since map isnt loaded properly
            // ASSERT_EQ(resp2.error_status, cav_srvs::SetActiveRouteResponse::NO_ERROR);
        }
    }
    
    ////////////
    // Conduct tests for set_active_route_cb() using an array of destination points in the service request
    ////////////
    route::RouteGeneratorWorker worker2(tf2_buffer);
    std::unique_ptr<std_msgs::msg::String> msg_ptr2 = std::make_unique<std_msgs::msg::String>(str_msg);
    worker2.setLoggerInterface(node->get_node_logging_interface());
    worker2.georeferenceCb(move(msg_ptr2)); 
    worker2.setWorldModelPtr(cmw);
    worker2.setRouteFilePath("../../install_ros2/route/share/route/resource/route/");
    worker2.vehicle_pose_ = msg;
    
    // Create array of destination points for the SetActiveRoute request
    carma_v2x_msgs::msg::Position3D destination;
    destination.latitude = -10440.3912269;
    destination.longitude = -541.755427;
    destination.elevation_exists = false;
    
    // Create SetActiveRoute request and response, and set necessary fields in the request
    carma_planning_msgs::srv::SetActiveRoute::Request req3;
    carma_planning_msgs::srv::SetActiveRoute::Response resp3;
    req3.destination_points.push_back(destination);
    req3.choice = carma_planning_msgs::srv::SetActiveRoute::Request::DESTINATION_POINTS_ARRAY;

    auto req_ptr3 = std::make_shared<carma_planning_msgs::srv::SetActiveRoute::Request>(req3);
    auto resp_ptr3 = std::make_shared<carma_planning_msgs::srv::SetActiveRoute::Response>(resp3);
    ASSERT_EQ(worker2.setActiveRouteCb(header, req_ptr3, resp_ptr3), true);
    // TODO: temporarily disabled since map isnt loaded properly
    // ASSERT_EQ(resp4.error_status, cav_srvs::SetActiveRouteResponse::NO_ERROR);
}

TEST(RouteGeneratorTest, test_duplicate_lanelets_in_shortest_path)
{
    // Create a RouteGeneratorWorker for this test
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
    tf2_ros::Buffer tf2_buffer(clock->get_clock());
    route::RouteGeneratorWorker worker(tf2_buffer);
    worker.setLoggerInterface(node->get_node_logging_interface());

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    // File location of the osm file for this test case:
    std::string file = "../../install_ros2/route/share/route/resource/map/town01_vector_map_1.osm";

    // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

    // Starting and ending lanelet IDs (these were obtained from viewing the osm file in JOSM)
    lanelet::Id start_id = 111;
    lanelet::Id end_id = 111;
    lanelet::Lanelet start_lanelet = map->laneletLayer.get(start_id);
    lanelet::Lanelet end_lanelet = map->laneletLayer.get(end_id);

    // Create and populate the 'via_lanelets_vector':
    lanelet::ConstLanelets via_lanelets_vector;
    lanelet::Id middle_id = 167;
    via_lanelets_vector.push_back(map->laneletLayer.get(middle_id));

    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

    auto route_with_duplicates = map_graph->getRouteVia(start_lanelet, via_lanelets_vector, end_lanelet);

    // Test that the shortest path in 'route_with_duplicates' contains duplicate lanelet IDs
    // The shortest path is 111 -> 101 -> 100 -> 104 -> 167 -> 169 -> 168 -> 170 -> 111
    ASSERT_EQ(worker.checkForDuplicateLaneletsInShortestPath(route_with_duplicates.get()), true);

    // Change the ending lanelet ID and update the route so the shortest path does not contain duplicate lanelet IDs
    end_id = 170;
    end_lanelet = map->laneletLayer.get(end_id);
    auto route_without_duplicates = map_graph->getRouteVia(start_lanelet, via_lanelets_vector, end_lanelet);

    // Test that the shortest path in 'route_without_duplicates' does not contain duplicate Lanelet IDs
    // The shortest path is 111 -> 101 -> 100 -> 104 -> 167 -> 169 -> 168 -> 170
    ASSERT_EQ(worker.checkForDuplicateLaneletsInShortestPath(route_without_duplicates.get()), false);
}

TEST(RouteGeneratorTest, test_reroute_after_route_invalidation)
{
    // Create a RouteGeneratorWorker for this test
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
    tf2_ros::Buffer tf2_buffer(clock->get_clock());
    route::RouteGeneratorWorker worker(tf2_buffer);
    worker.setLoggerInterface(node->get_node_logging_interface());

    auto cmw = carma_wm::test::getGuidanceTestMap();
    worker.setWorldModelPtr(cmw);

    // set route here
    carma_wm::test::setRouteByIds({1200, 1201,1202,1203}, cmw);

    lanelet::BasicPoint2d end_point{1.85, 87.5};

    std::vector<lanelet::BasicPoint2d> dest_points;
    dest_points.push_back(end_point);

    auto route = worker.rerouteAfterRouteInvalidation(dest_points);

    ASSERT_EQ(dest_points.size(), 1);
    ASSERT_TRUE(!!route);
    ASSERT_EQ(route->shortestPath().size(), 4);
}

TEST(RouteGeneratorTest, test_setReroutingChecker)
{
    // Create a RouteGeneratorWorker for this test
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
    tf2_ros::Buffer tf2_buffer(clock->get_clock());
    route::RouteGeneratorWorker worker(tf2_buffer);
    worker.setLoggerInterface(node->get_node_logging_interface());
    
    bool flag = false;
    worker.setReroutingChecker([&]{
        flag = true;
        return flag;
    });
    EXPECT_NO_THROW(worker.reroutingChecker());
    ASSERT_EQ(true,worker.reroutingChecker());
}

TEST(RouteGeneratorTest, test_get_closest_lanelet_from_route_llts)
{
    // Create a RouteGeneratorWorker for this test
    auto node = std::make_shared<rclcpp::Node>("test_node");
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
    tf2_ros::Buffer tf2_buffer(clock->get_clock());
    route::RouteGeneratorWorker worker(tf2_buffer);
    worker.setLoggerInterface(node->get_node_logging_interface());

    std::shared_ptr<carma_wm::WMListener> wml;
    std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    geometry_msgs::msg::PoseStamped msg;

    //Create route msg
    carma_planning_msgs::msg::Route route_msg;

    // File location of osm file
    std::string file = "../../install_ros2/route/share/route/resource/map/town01_vector_map_1.osm";
    
    // Starting and ending lanelet IDs. It's easiest to grab these from JOSM
    lanelet::Id start_id = 101;
    lanelet::Id end_id = 111;

    // Load map parameters
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);
    cmw->carma_wm::CARMAWorldModel::setMap(map);

    // Grabs lanelet elements from the start and end IDs. Fails the unit test if there is no lanelet with the matching ID
    lanelet::Lanelet start_lanelet;
    lanelet::Lanelet end_lanelet;

    try 
    {
        //get lanelet layer
        start_lanelet = map->laneletLayer.get(start_id);        
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified starting lanelet Id of " << start_id << " does not exist in the provided map.";
    }
    try {
        end_lanelet = map->laneletLayer.get(end_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified ending lanelet Id of " << end_id << " does not exist in the provided map.";
    }

    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

    const auto route = map_graph->getRoute(start_lanelet, end_lanelet);
    route_msg = worker.composeRouteMsg(route);
    ASSERT_TRUE(route_msg.route_path_lanelet_ids.size() > 0);

    worker.setWorldModelPtr(cmw);
    worker.setCrosstrackErrorCountMax(0);
    worker.setCrosstrackErrorDistance(1.0);

     //Assign vehicle position
    msg.pose.position.x = -9.45542;
    msg.pose.position.y = -182.324;

    lanelet::BasicPoint2d position;
    position.x()= msg.pose.position.x;
    position.y()= msg.pose.position.y;

    auto mpt = std::make_shared<geometry_msgs::msg::PoseStamped>(msg);

    worker.addLanelet(start_lanelet);
    lanelet::ConstLanelet llt = worker.getClosestLaneletFromRouteLanelets(position);

    ASSERT_EQ(worker.crosstrackErrorCheck(mpt, llt), false);
    ASSERT_EQ(boost::geometry::within(position, llt.polygon2d()), true);
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