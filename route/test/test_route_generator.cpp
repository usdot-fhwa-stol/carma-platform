/*
 * Copyright (C) 2020 LEIDOS.
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

#include "route_generator_worker.h"
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
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

/*
Using this file:
    1) Libraries you should have: carma-base, carma-config, carma-msgs (integration/routing branch), 
        carma-platform (integration/routing branch), carma-utils, caram-web-ui, opendrive2lanelet.
    2) Then the test can be built with the command: 
        ./carma_build -c /workspaces/carma_ws/carma/ -a /workspaces/carma_ws/autoware.ai/ -x -m "--only-pkg-with-deps route"
    3) Update the osm file location and starting/ending IDs to match the file you want to test
    4) Run the unit test with
        catkin_make run_tests_route
    5) Confirm that the test passed and that the list of lanelet IDs does traverse from the start to the end
*/

TEST(RouteGeneratorTest, testLaneletRouting)
{
    tf2_ros::Buffer tf_buffer;
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker(tf_buffer);

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    // If the output is an error about the geoReference field in the osm file, here is a correct example. If the lat/lon coordinates are already correct, simply add:
    // <geoReference>+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs</geoReference>

    // File location of osm file
    std::string file = "../resource/map/test_vector_map.osm";
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
    } else {
        std::cout << "shortest path: \n";
        for(const auto& ll : route.get().shortestPath()) {
            std::cout << ll.id() << " ";
        }
        std::cout << "\n";
    }
}

TEST(RouteGeneratorTest, testReadRouteFile)
{
    tf2_ros::Buffer tf_buffer;
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker(tf_buffer);
    worker.set_route_file_path("../../src/route/resource/route/");
    cav_srvs::GetAvailableRoutesRequest req;
    cav_srvs::GetAvailableRoutesResponse resp;
    ASSERT_TRUE(worker.get_available_route_cb(req, resp));
    ASSERT_EQ("tfhrc_test_route", resp.availableRoutes.front().route_id);
    ASSERT_EQ("C-HUB", resp.availableRoutes.front().route_name);
    ASSERT_EQ(1, resp.availableRoutes.size());
    auto points = worker.load_route_destinations_in_ecef("tfhrc_test_route");
    ASSERT_EQ(5, points.size());
    ASSERT_NEAR(1106580, points[0].getX(), 5.0);
    ASSERT_NEAR(894697, points[0].getY(), 5.0);
    ASSERT_NEAR(-6196590, points[0].getZ(), 5.0);
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
