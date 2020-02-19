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
#include <carma_wm/lanelet/DigitalSpeedLimit.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
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

TEST(DigitalSpeedLimit, digitalSpeedLimit)
{
  auto pl1 = carma_wm::getPoint(0, 0, 0);
  auto pl2 = carma_wm::getPoint(0, 1, 0);
  auto pl3 = carma_wm::getPoint(0, 2, 0);
  auto pr1 = carma_wm::getPoint(1, 0, 0);
  auto pr2 = carma_wm::getPoint(1, 1, 0);
  auto pr3 = carma_wm::getPoint(1, 2, 0);

  auto p4 = carma_wm::getPoint(0, 3, 0);
  auto p5 = carma_wm::getPoint(1, 3, 0);

  std::vector<lanelet::Point3d> left_1 = { pl1, pl2 };
  std::vector<lanelet::Point3d> right_1 = { pr1, pr2 };
  auto ll_1 = carma_wm::getLanelet(left_1, right_1);

  std::vector<lanelet::Point3d> left_2 = { pl2, pl3 };
  std::vector<lanelet::Point3d> right_2 = { pr2, pr3 };
  auto ll_2 = carma_wm::getLanelet(left_2, right_2);

  // Add an area
  lanelet::LineString3d area_loop(lanelet::utils::getId(), { pl3, p4, p5, pr3 });

  area_loop.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
  area_loop.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Dashed;

  lanelet::Area area(lanelet::utils::getId(), { area_loop });

  area.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::Multipolygon;
  area.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  area.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
  area.attributes()[lanelet::AttributeNamesString::ParticipantVehicle] = "yes";

  DigitalSpeedLimit dsl(DigitalSpeedLimit::buildData(lanelet::utils::getId(), 5_kmh, { ll_1, ll_2 }, { area },
                                                     { lanelet::Participants::VehicleCar }));

  ASSERT_EQ(2, dsl.getLanelets().size());
  ASSERT_EQ(1, dsl.getAreas().size());
  ASSERT_FALSE(dsl.appliesTo(lanelet::Participants::Vehicle));
  ASSERT_TRUE(dsl.appliesTo(lanelet::Participants::VehicleCar));
  ASSERT_TRUE(dsl.appliesTo(lanelet::Participants::VehicleCarElectric));  // Test acceptance of sub type

  ASSERT_EQ(5_kmh, dsl.getSpeedLimit());
}

}  // namespace lanelet
