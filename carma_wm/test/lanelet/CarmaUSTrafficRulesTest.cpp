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
#include <lanelet2_core/utility/Units.h>
#include <carma_wm/lanelet/CarmaUSTrafficRules.h>
#include <lanelet2_core/geometry/LineString.h>
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

TEST(CarmaUSTrafficRules, carmaUSTrafficRules)
{
  // Build the map to use
  auto map = carma_wm::getDisjointRouteWithArea();

  Lanelet ll_1 = *(map->laneletLayer.find(10000));
  Lanelet ll_2 = *(map->laneletLayer.find(10001));
  Lanelet ll_3 = *(map->laneletLayer.find(10002));
  Lanelet ll_4 = *(map->laneletLayer.find(10003));
  Area area = *(map->areaLayer.find(10004));

  // 1. Set speed limit for all 3 lanelets to 25
  std::shared_ptr<DigitalSpeedLimit> dsl_lanelet(new DigitalSpeedLimit(DigitalSpeedLimit::buildData(
      lanelet::utils::getId(), 25_kmh, { ll_1, ll_2, ll_3, ll_4 }, {}, { lanelet::Participants::Vehicle })));
  ll_1.addRegulatoryElement(dsl_lanelet);
  ll_2.addRegulatoryElement(dsl_lanelet);
  ll_3.addRegulatoryElement(dsl_lanelet);
  ll_4.addRegulatoryElement(dsl_lanelet);
  // 2. set speed limit for area to 20
  std::shared_ptr<DigitalSpeedLimit> dsl_area(new DigitalSpeedLimit(
      DigitalSpeedLimit::buildData(lanelet::utils::getId(), 20_kmh, {}, { area }, { lanelet::Participants::Vehicle })));
  area.addRegulatoryElement(dsl_area);
  // 3. set lanelet passable by vehicle
  std::shared_ptr<RegionAccessRule> rar_lanelet(new RegionAccessRule(RegionAccessRule::buildData(
      lanelet::utils::getId(), { ll_1, ll_2, ll_3, ll_4 }, {}, { lanelet::Participants::Vehicle })));
  ll_1.addRegulatoryElement(rar_lanelet);
  ll_2.addRegulatoryElement(rar_lanelet);
  ll_3.addRegulatoryElement(rar_lanelet);
  ll_4.addRegulatoryElement(rar_lanelet);
  // 4. set area passable by car
  std::shared_ptr<RegionAccessRule> rar_area(new RegionAccessRule(
      RegionAccessRule::buildData(lanelet::utils::getId(), {}, { area }, { lanelet::Participants::VehicleCar })));
  area.addRegulatoryElement(rar_area);
  // 5. set direction of travel for first two lanelets to one way.
  std::shared_ptr<DirectionOfTravel> dot_oneway(new DirectionOfTravel(DirectionOfTravel::buildData(
      lanelet::utils::getId(), { ll_1, ll_2, ll_3 }, DirectionOfTravel::OneWay, { lanelet::Participants::Vehicle })));
  ll_1.addRegulatoryElement(dot_oneway);
  ll_2.addRegulatoryElement(dot_oneway);
  ll_3.addRegulatoryElement(dot_oneway);
  // 6. set direction of travel for last lanelet to bi directional
  std::shared_ptr<DirectionOfTravel> dot_bi(new DirectionOfTravel(DirectionOfTravel::buildData(
      lanelet::utils::getId(), { ll_4 }, DirectionOfTravel::BiDirectional, { lanelet::Participants::Vehicle })));
  ll_4.addRegulatoryElement(dot_bi);
  // 7. create non-passable control line on left of first lanelet
  std::shared_ptr<PassingControlLine> pcl_left_1(
      new PassingControlLine(PassingControlLine::buildData(lanelet::utils::getId(), { ll_1.leftBound() }, {}, {})));
  ll_1.addRegulatoryElement(pcl_left_1);

  // 8. create passable control line from either side on first lanelet
  std::shared_ptr<PassingControlLine> pcl_right_1(new PassingControlLine(
      PassingControlLine::buildData(lanelet::utils::getId(), { ll_1.rightBound() }, { lanelet::Participants::Vehicle },
                                    { lanelet::Participants::Vehicle })));
  ll_1.addRegulatoryElement(pcl_right_1);
  ll_2.addRegulatoryElement(pcl_right_1);
  // 9. set non-passable control line on last lanelets
  std::shared_ptr<PassingControlLine> pcl_left_2(new PassingControlLine(
      PassingControlLine::buildData(lanelet::utils::getId(), { ll_3.leftBound(), ll_4.leftBound() }, {}, {})));
  std::shared_ptr<PassingControlLine> pcl_right_2(new PassingControlLine(PassingControlLine::buildData(
      lanelet::utils::getId(), { ll_2.rightBound(), ll_3.rightBound(), ll_4.rightBound() }, {}, {})));
  ll_3.addRegulatoryElement(pcl_left_2);
  ll_4.addRegulatoryElement(pcl_left_2);

  ll_2.addRegulatoryElement(pcl_right_2);
  ll_3.addRegulatoryElement(pcl_right_2);
  ll_4.addRegulatoryElement(pcl_right_2);
  // 10. set passable control line from either side for area lanelet bound
  std::shared_ptr<PassingControlLine> pcl_area(new PassingControlLine(
      PassingControlLine::buildData(lanelet::utils::getId(), { area.outerBound()[1] },
                                    { lanelet::Participants::Vehicle }, { lanelet::Participants::Vehicle })));
  area.addRegulatoryElement(pcl_area);

  // Done Building Map
  // Actual testing starts here

  // Tests for a generic vehicle
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::traffic_rules::CarmaUSTrafficRules::Location, lanelet::Participants::Vehicle);

  // bool canPass(const ConstLanelet& lanelet) const override;
  ASSERT_TRUE(traffic_rules->canPass(ll_1));
  ASSERT_TRUE(traffic_rules->canPass(ll_2));
  ASSERT_TRUE(traffic_rules->canPass(ll_3));
  ASSERT_TRUE(traffic_rules->canPass(ll_4));

  // bool canPass(const ConstArea& area) const override;
  ASSERT_FALSE(traffic_rules->canPass(area));

  // bool canPass(const ConstLanelet& from, const ConstLanelet& to) const override;
  ASSERT_FALSE(traffic_rules->canPass(ll_1, ll_2));
  ASSERT_FALSE(traffic_rules->canPass(ll_1, ll_3));
  ASSERT_TRUE(traffic_rules->canPass(ll_2, ll_3));
  ASSERT_TRUE(traffic_rules->canPass(ll_3, ll_4));

  // bool canPass(const ConstLanelet& from, const ConstArea& to) const override;
  ASSERT_FALSE(traffic_rules->canPass(ll_2, area));
  ASSERT_FALSE(traffic_rules->canPass(ll_4, area));

  // bool canPass(const ConstArea& from, const ConstLanelet& to) const override;
  ASSERT_FALSE(traffic_rules->canPass(area, ll_4));
  ASSERT_FALSE(traffic_rules->canPass(area, ll_2));

  // bool canPass(const ConstArea& from, const ConstArea& to) const override;
  ASSERT_FALSE(traffic_rules->canPass(area, area));

  // bool canChangeLane(const ConstLanelet& from, const ConstLanelet& to) const override;
  ASSERT_FALSE(traffic_rules->canChangeLane(ll_1, ll_3));
  ASSERT_TRUE(traffic_rules->canChangeLane(ll_1, ll_2));
  ASSERT_TRUE(traffic_rules->canChangeLane(ll_2, ll_1));

  // SpeedLimitInformation speedLimit(const ConstLanelet& lanelet) const override;

  ASSERT_EQ(25_kmh, traffic_rules->speedLimit(ll_1).speedLimit);
  ASSERT_EQ(25_kmh, traffic_rules->speedLimit(ll_2).speedLimit);
  ASSERT_EQ(25_kmh, traffic_rules->speedLimit(ll_3).speedLimit);
  ASSERT_EQ(25_kmh, traffic_rules->speedLimit(ll_4).speedLimit);

  // SpeedLimitInformation speedLimit(const ConstArea& area) const override;
  ASSERT_EQ(20_kmh, traffic_rules->speedLimit(area).speedLimit);

  // bool isOneWay(const ConstLanelet& lanelet) const override;
  ASSERT_TRUE(traffic_rules->isOneWay(ll_1));
  ASSERT_TRUE(traffic_rules->isOneWay(ll_2));
  ASSERT_TRUE(traffic_rules->isOneWay(ll_3));
  ASSERT_FALSE(traffic_rules->isOneWay(ll_4));

  // bool hasDynamicRules(const ConstLanelet& lanelet) const override;
  ASSERT_TRUE(traffic_rules->hasDynamicRules(ll_1));

  // Tests for a car
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules_car = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::traffic_rules::CarmaUSTrafficRules::Location, lanelet::Participants::VehicleCar);

  ASSERT_TRUE(traffic_rules_car->canPass(ll_4, area));

  ASSERT_TRUE(traffic_rules_car->canPass(area, ll_4));
}

}  // namespace lanelet
