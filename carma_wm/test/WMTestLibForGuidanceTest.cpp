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
#include <carma_wm/CARMAWorldModel.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <tf2/LinearMath/Quaternion.h>
#include "TestHelpers.h"
#include <lanelet2_extension/regulatory_elements/PassingControlLine.h>
#include <carma_wm/WMTestLibForGuidance.h>
#include <ros/ros.h>

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_wm
{
namespace test
{
TEST(WMTestLibForGuidanceTest, getGuidanceTestMap)
{
    auto cmw = getGuidanceTestMap();
    // Overall sanity check
    ASSERT_TRUE((bool)cmw->getMap());	
    ASSERT_TRUE((bool)cmw->getRoute());	
    ASSERT_TRUE((bool)cmw->getMapRoutingGraph());

    // Sanity check on lanelets
    ASSERT_EQ(12, cmw->getMap()->laneletLayer.size());

    // Sanity check on the default route
    EXPECT_NO_THROW(cmw->getRoute()->checkValidity());
    EXPECT_EQ(cmw->getRoute()->shortestPath().size(), 4);
    EXPECT_EQ((cmw->getRoute()->shortestPath().begin()+ 1)->id(), 1201);
    EXPECT_EQ((cmw->getRoute()->shortestPath().begin() + 2)->id(), 1202);

    // Check default obstacle
    ASSERT_EQ(cmw->getRoadwayObjects().size(), 1);
    ASSERT_EQ(cmw->getRoadwayObjects().begin()->lanelet_id, 1212);
    ASSERT_EQ(cmw->getRoadwayObjects().begin()->down_track, 12.5);   // half of default lanelet length
    ASSERT_EQ(cmw->getRoadwayObjects().begin()->cross_track, 0);     // right at the center

    // Check the default speed limit
    ASSERT_TRUE(cmw->getMap()->regulatoryElementLayer.size() >= 12); // 12+ regems because each lanelet has one speedlimit at least
    ASSERT_EQ(cmw->getMap()->laneletLayer.get(1200).regulatoryElementsAs<lanelet::DigitalSpeedLimit>().size(), 1);
    ASSERT_NEAR(cmw->getMap()->laneletLayer.get(1200).regulatoryElementsAs<lanelet::DigitalSpeedLimit>().begin()->get()->getSpeedLimit().value(), 11.176, 0.0001); // 25mph is this meter/s

    cmw = getGuidanceTestMap(3.7,25,{NO_OBSTACLE});
    ASSERT_EQ(cmw->getRoadwayObjects().size(), 0);

    cmw = getGuidanceTestMap(3.7,25,{NO_OBSTACLE, NO_SPEED_LIMIT});
    ASSERT_EQ(cmw->getMap()->regulatoryElementLayer.size(), 0);
}

TEST(WMTestLibForGuidanceTest, buildGuidanceTestMap)
{
    auto map = buildGuidanceTestMap(1,1); // unit length lanelets for easy testing as some developers might prefer
    // Sanity check on lanelets
    ASSERT_EQ(12, map->laneletLayer.size());
    ASSERT_EQ(map->laneletLayer.get(1200).leftBound()[1].y(), 1);
}

TEST(WMTestLibForGuidanceTest, addObstacle)
{
    auto cmw = getGuidanceTestMap(1,1);
    addObstacle(0.5,0.5, cmw, {{0.25,1.5}, {0.5,2.5}}, 0.75, 0.75);
    ASSERT_EQ(cmw->getRoadwayObjects().size(), 2);
    // Check if it is correctly filling raw values
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.pose.pose.position.x, 0.5);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.pose.pose.position.y, 0.5);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.pose.pose.position.z, 0);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.pose.pose.orientation.x, 0);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.pose.pose.orientation.y, 0);
    ASSERT_NEAR(cmw->getRoadwayObjects()[1].object.pose.pose.orientation.z, 0.707108, 0.0001);
    ASSERT_NEAR(cmw->getRoadwayObjects()[1].object.pose.pose.orientation.w, 0.707108, 0.0001);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.size.x, 0.75);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.size.y, 0.75);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.size.z, 1);

    // predicted raw values
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.predictions.size(), 2);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.predictions[0].header.stamp.nsec, 1000);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.predictions[1].header.stamp.nsec, 2000);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.predictions[0].predicted_position.position.y, 1.5);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].object.predictions[0].header.stamp.nsec, 1000);

    // check calculated values
    ASSERT_EQ(cmw->getRoadwayObjects()[1].lanelet_id, 1200);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].down_track, 0.5);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].cross_track, 0);
    
    // check calculated pred values
    ASSERT_EQ(cmw->getRoadwayObjects()[1].predicted_lanelet_ids[0], 1201);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].predicted_down_tracks[0], 0.5);
    ASSERT_EQ(cmw->getRoadwayObjects()[1].predicted_cross_tracks[0], -0.25);

    // Check the other way of adding obstacle
    carma_wm::TrackPos tp = {0.5, 0};
    carma_wm::TrackPos tp_pred = {1.5, -0.25};

    addObstacle(tp, 1200, cmw, {tp_pred}, 0.75, 0.75);
    // check calculated values
    ASSERT_EQ(cmw->getRoadwayObjects()[2].lanelet_id, 1200);
    ASSERT_EQ(cmw->getRoadwayObjects()[2].down_track, 0.5);
    ASSERT_EQ(cmw->getRoadwayObjects()[2].cross_track, 0);
    
    // check calculated pred values
    ASSERT_EQ(cmw->getRoadwayObjects()[2].predicted_lanelet_ids[0], 1201);
    ASSERT_EQ(cmw->getRoadwayObjects()[2].predicted_down_tracks[0], 0.5);
    ASSERT_EQ(cmw->getRoadwayObjects()[2].predicted_cross_tracks[0], -0.25);
}

TEST(WMTestLibForGuidanceTest, setRouteByIds)
{
    auto cmw = getGuidanceTestMap();
    EXPECT_THROW(setRouteByIds({1200}, cmw), lanelet::InvalidInputError);
    EXPECT_NO_THROW(setRouteByIds({1200,1210,1220,1221,1222,1223}, cmw));
    ASSERT_EQ(cmw->getRoute()->shortestPath().size(), 6);
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1200)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1210)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1220)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1221)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1222)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1223)));
    
    EXPECT_NO_THROW(setRouteByIds({1200,1210}, cmw));
    ASSERT_EQ(cmw->getRoute()->shortestPath().size(), 2);
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1200)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1210)));

    EXPECT_NO_THROW(setRouteByIds({1200,1201,1211,1212,1222,1223}, cmw));
    ASSERT_EQ(cmw->getRoute()->shortestPath().size(), 6);
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1200)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1201)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1211)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1212)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1222)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1223)));

    EXPECT_NO_THROW(setRouteByIds({1200,1201,1211,1221,1222,1223}, cmw));
    ASSERT_EQ(cmw->getRoute()->shortestPath().size(), 6);
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1200)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1223)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1211)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1221)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1222)));
    ASSERT_TRUE(cmw->getRoute()->contains(cmw->getMap()->laneletLayer.get(1220)));
}

TEST(WMTestLibForGuidanceTest, setRouteByLanelets)
{
    auto cmw = getGuidanceTestMap();
    auto ll_1200 = cmw->getMap()->laneletLayer.get(1200);
    auto ll_1210 = cmw->getMap()->laneletLayer.get(1210);
    auto ll_1220 = cmw->getMap()->laneletLayer.get(1220);
    auto ll_1221 = cmw->getMap()->laneletLayer.get(1221);
    auto ll_1222 = cmw->getMap()->laneletLayer.get(1222);
    auto ll_1223 = cmw->getMap()->laneletLayer.get(1223);
    auto ll_1211 = cmw->getMap()->laneletLayer.get(1211);
    auto ll_1212 = cmw->getMap()->laneletLayer.get(1212);
    auto ll_1201 = cmw->getMap()->laneletLayer.get(1201);

    EXPECT_THROW(setRouteByLanelets({ll_1200}, cmw), lanelet::InvalidInputError);
    EXPECT_NO_THROW(setRouteByLanelets({ll_1200,ll_1210,ll_1220,ll_1221,ll_1222,ll_1223}, cmw));
    ASSERT_EQ(cmw->getRoute()->shortestPath().size(), 6);
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1200));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1210));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1220));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1221));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1222));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1223));
    
    EXPECT_NO_THROW(setRouteByLanelets({ll_1200,ll_1210}, cmw));
    ASSERT_EQ(cmw->getRoute()->shortestPath().size(), 2);
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1200));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1210));

    EXPECT_NO_THROW(setRouteByLanelets({ll_1200,ll_1201,ll_1211,ll_1212,ll_1222,ll_1223}, cmw));
    ASSERT_EQ(cmw->getRoute()->shortestPath().size(), 6);
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1200));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1201));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1211));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1212));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1222));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1223));

    EXPECT_NO_THROW(setRouteByLanelets({ll_1200,ll_1201,ll_1211,ll_1221,ll_1222,ll_1223}, cmw));
    ASSERT_EQ(cmw->getRoute()->shortestPath().size(), 6);
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1200));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1223));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1211));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1221));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1222));
    ASSERT_TRUE(cmw->getRoute()->contains(ll_1220));
}

TEST(WMTestLibForGuidanceTest, setSpeedLimit)
{
    auto cmw = getGuidanceTestMap(3.7, 25.0, {DEFAULT_OBSTACLE, NO_SPEED_LIMIT});
    auto llt = cmw->getMutableMap()->laneletLayer.get(1200);
    // create existing speed limit to overwrite
    lanelet::DigitalSpeedLimitPtr sl = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::utils::getId(), 50_mph, {llt}, {},
                                                     { lanelet::Participants::VehicleCar }));
    cmw->getMutableMap()->update(llt, sl);
    ASSERT_EQ(cmw->getMutableMap()->regulatoryElementLayer.size(), 1);
    setSpeedLimit(25_mph, cmw);
    ASSERT_EQ(cmw->getMutableMap()->regulatoryElementLayer.size(), 13); // old speed limit exists but is not assigned to any llt
    ASSERT_NEAR(cmw->getMutableMap()->laneletLayer.get(1200).regulatoryElementsAs<lanelet::DigitalSpeedLimit>()[0]->getSpeedLimit().value(), 11.176, 0.0001); 
}
}  // namespace test
}  // namespace carma_wm
