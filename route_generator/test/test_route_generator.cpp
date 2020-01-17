/*
 * Copyright (C) 2019 LEIDOS.
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

#include "route_generator.h"
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

TEST(RouteGeneratorTest, testReadFileFunction)
{
    std::vector<std::string> file_names = RouteGenerator::read_route_names("/home/qswawrq/CARMAWorkspace/src/route_generator/resource/");
    for(int i = 0; i < file_names.size(); ++i)
    {
        std::string expect_file_name = "route" + std::to_string(i + 1) + ".csv";
        EXPECT_EQ(file_names[i], expect_file_name);
    }

}

TEST(RouteGeneratorTest, testLaneletRouting)
{
    // lanelet::LaneletMapPtr map = lanelet::load("/home/qswawrq/Desktop/TFHRC.osm",
    //     lanelet::projection::LocalFrameProjector("EPSG:4326", "+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 +units=m +vunits=m"));
    lanelet::LaneletMapPtr map = lanelet::load("/home/qswawrq/Desktop/TFHRC_3.osm", lanelet::Origin({0, 0}));
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
    auto route = RouteGenerator::routing(start, via, end, const_map, std::move(map_graph));
    if(!route) {
        ASSERT_FALSE(true);
    } else {
        std::cout << "shortest path: \n";
        for(const auto& ll : route.get().shortestPath()) {
            std::cout << ll.id() << " ";
        }
    }
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}