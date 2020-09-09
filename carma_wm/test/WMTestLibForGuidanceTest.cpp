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
    ASSERT_TRUE((bool)cmw->getMap());	
    ASSERT_TRUE((bool)cmw->getRoute());	
    ASSERT_TRUE((bool)cmw->getMapRoutingGraph());
}
}  // namespace test
}  // namespace carma_wm
