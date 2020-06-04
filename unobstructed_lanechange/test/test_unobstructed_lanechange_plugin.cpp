/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include "unobstructed_lanechange.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/CARMAWorldModel.h>
#include "TestHelpers.h"






TEST(UnobstructedLaneChangePluginTest, testCreateLaneChangeTrajectory1)
{
    std::vector<double> start_point = {0.0, 0.0};
    std::vector<double> end_point = {10.0, 3.0};
    unobstructed_lanechange::UnobstructedLaneChangePlugin lc;
    std::vector<cav_msgs::TrajectoryPlanPoint> res = lc.create_lanechange_trajectory(start_point, end_point);
    EXPECT_EQ(60, res.size());
    EXPECT_NEAR(0.0, res[0].target_time, 0.01);
    EXPECT_NEAR(0.0, res[0].x, 0.05);
    EXPECT_NEAR(0.0, res[0].y, 0.05);
    EXPECT_NEAR(2.5, res[15].x, 0.05);
    EXPECT_NEAR(0.75, res[15].y, 0.05);
    EXPECT_NEAR(5.0, res[30].x, 0.05);
    EXPECT_NEAR(1.5, res[30].y, 0.05);
    EXPECT_NEAR(7.5, res[45].x, 0.05);
    EXPECT_NEAR(2.25, res[45].y, 0.05);
    EXPECT_NEAR(9.85, res[59].x, 0.05);
    EXPECT_NEAR(2.95, res[59].y, 0.05);
}






TEST(UnobstructedLaneChangePluginTest, testextract1)
{
    std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();

    auto pl1 = carma_wm::getPoint(0, 0, 0);
    auto pl2 = carma_wm::getPoint(0, 1, 0);
    auto pl3 = carma_wm::getPoint(0, 2, 0);
    auto pr1 = carma_wm::getPoint(1, 0, 0);
    auto pr2 = carma_wm::getPoint(1, 1, 0);
    auto pr3 = carma_wm::getPoint(1, 2, 0);

    std::vector<lanelet::Point3d> left_1 = { pl1, pl2 };
    std::vector<lanelet::Point3d> right_1 = { pr1, pr2 };
    auto ll_1 = carma_wm::getLanelet(left_1, right_1, lanelet::AttributeValueString::SolidSolid, lanelet::AttributeValueString::Dashed);

    std::vector<lanelet::Point3d> left_2 = { pl2, pl3 };
    std::vector<lanelet::Point3d> right_2 = { pr2, pr3 };
    auto ll_2 = carma_wm::getLanelet(left_2, right_2);

    // 1. Confirm all pointers are false (done above)
    // Ensure that none of the returned pointers are valid if the map has not been set
    ASSERT_FALSE((bool)cmw->getMap());
    ASSERT_FALSE((bool)cmw->getRoute());
    ASSERT_FALSE((bool)cmw->getMapRoutingGraph());

    // 2. Build map but do not assign
    // Create basic map and verify that the map and routing graph can be build, but the route remains false
    lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1, ll_2 }, {});
    // 3. Build routing graph but do not assign
    // Build routing graph from map
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

    auto id = cmw->getRoute()->shortestPath()[0].id();


    std::string id_ = std::to_string(id);
    double dt_ = 1;
    unobstructed_lanechange::UnobstructedLaneChangePlugin lc;
    std::vector<double> res = lc.extract_point_from_lanelet(cmw, id_, dt_);
    
    

    EXPECT_EQ(2, res.size());
    EXPECT_NEAR(0.5, res[0], 0.05);
    EXPECT_NEAR(0, res[1], 0.05);
}





// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}