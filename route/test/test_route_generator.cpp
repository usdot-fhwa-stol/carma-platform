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
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(RouteGeneratorTest, testLaneletRouting)
{
    tf2_ros::Buffer tf_buffer;
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker(tf_buffer, wm);
    // lanelet::LaneletMapPtr map = lanelet::load("/home/qswawrq/Desktop/TFHRC.osm",
    //     lanelet::projection::LocalFrameProjector("EPSG:4326", "+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 +units=m +vunits=m"));
    lanelet::LaneletMapPtr map = lanelet::load("../../src/route/resource/map/test_vector_map.osm", lanelet::Origin({0, 0}));
    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::BasicPoint2d start(0.5, 0.5);
    std::vector<lanelet::BasicPoint2d> via;
    // via.emplace_back(lanelet::BasicPoint2d(-158.0, 533.5));
    //via.emplace_back(lanelet::BasicPoint2d(-17.5, 322.0));
    lanelet::BasicPoint2d end(0.5, 1.5);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
    // Output graph for debugging
    // map_graph->exportGraphViz("/home/qswawrq/Desktop/routing.txt");
    auto route = worker.routing(start, via, end, const_map, std::move(map_graph));
    if(!route) {
        ASSERT_FALSE(true);
    } else {
        std::cout << "shortest path: \n";
        for(const auto& ll : route.get().shortestPath()) {
            std::cout << ll.id() << " ";
        }
    }
}

TEST(RouteGeneratorTest, testReadRouteFile)
{
    tf2_ros::Buffer tf_buffer;
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker(tf_buffer, wm);
    worker.set_route_file_path("../../src/route/resource/route/");
    cav_srvs::GetAvailableRoutesRequest req;
    cav_srvs::GetAvailableRoutesResponse resp;
    ASSERT_TRUE(worker.get_available_route_cb(req, resp));
    ASSERT_EQ("tfhrc_test_route", resp.availableRoutes.front().route_name);
    ASSERT_EQ(1, resp.availableRoutes.size());
    auto points = worker.load_route_destinationsin_ecef("tfhrc_test_route");
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