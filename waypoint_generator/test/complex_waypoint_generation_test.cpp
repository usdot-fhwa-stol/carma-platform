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

#include <gmock/gmock.h>
#include <iostream>
#include <waypoint_generator/waypoint_generator.hpp>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_core/Attribute.h>
#include <carma_wm/Geometry.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_wm/WMTestLibForGuidance.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tuple>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/Traits.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <carma_wm/MapConformer.h>
#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

using namespace waypoint_generator;
using namespace lanelet::units::literals;

/*
Using this file:
    1) Set the file path to your OSM file
    2) Set the route_ids to the route you wish to test

    NOTE: The test is disabled by default. Enable it by removing the DISABLED_ prefix from the test name
*/

TEST(WaypointGeneratorTest, DISABLED_test_full_waypoint_generation)
{
    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    // File location of osm file
    std::string file = "/workspaces/carma_ws/carma/src/carma-platform/waypoint_generator/AOI_1_TFHRC_faster_pretty.osm";    
    // The route ids that will form the route used
    std::vector<lanelet::Id> route_ids = { 130, 111, 110, 113, 135, 138 };

    // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

    lanelet::MapConformer::ensureCompliance(map);

    WaypointGeneratorConfig config;
    std::shared_ptr<CARMAWorldModel> wm = std::make_shared<CARMAWorldModel>();
    wm->setMap(map);

    bool wp_published;
    autoware_msgs::LaneArray published_wps;
    WaypointGenerator wpg(wm, config, [&](auto msg) {published_wps = msg; wp_published = true;});
    
    auto routing_graph = wm->getMapRoutingGraph();

    // Output graph for debugging
    routing_graph->exportGraphViz("/workspaces/carma_ws/carma/src/carma-platform/waypoint_generator/routing.viz");

    carma_wm::test::setRouteByIds(route_ids, wm);

    wpg.new_route_callback();



}

}