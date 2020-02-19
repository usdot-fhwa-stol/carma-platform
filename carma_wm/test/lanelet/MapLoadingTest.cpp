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

#include <gmock/gmock.h>
#include <iostream>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>

#include <carma_wm/lanelet/RegionAccessRule.h>
#include <carma_wm/lanelet/DigitalSpeedLimit.h>
#include <carma_wm/lanelet/DirectionOfTravel.h>
#include <carma_wm/lanelet/PassingControlLine.h>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/utility/Units.h>
#include "../TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace lanelet
{
using namespace lanelet::units::literals;

TEST(MapLoadingTest, mapLoadingTest)
{
  lanelet::LaneletMapPtr map = lanelet::load("resources/test_map.osm", lanelet::Origin({ 0, 0 }));

  auto ll_1 = map->laneletLayer.find(1349);

  auto sl = (*ll_1).regulatoryElementsAs<DigitalSpeedLimit>();

  ASSERT_EQ(1, sl.size());
  ASSERT_EQ(5_mph, sl[0]->getSpeedLimit());

  auto control_lines = (*ll_1).regulatoryElementsAs<PassingControlLine>();

  ASSERT_EQ(2, control_lines.size());
  // TODO control line check
  ASSERT_TRUE(
      PassingControlLine::boundPassable(ll_1->leftBound(), control_lines, true, lanelet::Participants::VehicleCar));
  ASSERT_TRUE(
      PassingControlLine::boundPassable(ll_1->leftBound(), control_lines, false, lanelet::Participants::VehicleCar));

  ASSERT_FALSE(
      PassingControlLine::boundPassable(ll_1->rightBound(), control_lines, true, lanelet::Participants::VehicleCar));
  ASSERT_FALSE(
      PassingControlLine::boundPassable(ll_1->rightBound(), control_lines, false, lanelet::Participants::VehicleCar));

  auto dot = (*ll_1).regulatoryElementsAs<DirectionOfTravel>();

  ASSERT_EQ(1, dot.size());
  ASSERT_TRUE(dot[0]->isOneWay());

  auto rar = (*ll_1).regulatoryElementsAs<RegionAccessRule>();

  ASSERT_EQ(1, dot.size());
  ASSERT_TRUE(rar[0]->accessable("vehicle:car"));
  ASSERT_FALSE(rar[0]->accessable("vehicle"));
}

}  // namespace lanelet
