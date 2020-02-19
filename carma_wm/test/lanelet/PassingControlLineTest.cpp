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
#include <carma_wm/lanelet/PassingControlLine.h>
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
TEST(PassingControlLineTest, passingControlLine)
{
  auto pl1 = carma_wm::getPoint(0, 0, 0);
  auto pl2 = carma_wm::getPoint(0, 1, 0);
  auto pr1 = carma_wm::getPoint(1, 0, 0);
  auto pr2 = carma_wm::getPoint(1, 1, 0);

  std::vector<lanelet::Point3d> left_1 = { pl1, pl2 };
  std::vector<lanelet::Point3d> right_1 = { pr1, pr2 };
  auto ll_1 = carma_wm::getLanelet(left_1, right_1, lanelet::AttributeValueString::SolidDashed,
                                   lanelet::AttributeValueString::Dashed);

  // Creat passing control line for solid dashed line
  std::shared_ptr<PassingControlLine> pcl(new PassingControlLine(PassingControlLine::buildData(
      lanelet::utils::getId(), { ll_1.leftBound() }, {}, { lanelet::Participants::Vehicle })));

  ll_1.addRegulatoryElement(pcl);

  LineStrings3d nonconstLine = pcl->controlLine();
  ASSERT_EQ(1, nonconstLine.size());

  ASSERT_FALSE(pcl->passableFromLeft(lanelet::Participants::Vehicle));
  ASSERT_TRUE(pcl->passableFromRight(lanelet::Participants::Vehicle));
  ;

  ASSERT_FALSE(PassingControlLine::boundPassable(ll_1.leftBound(), ll_1.regulatoryElementsAs<PassingControlLine>(),
                                                 true, lanelet::Participants::Vehicle));
  ASSERT_TRUE(PassingControlLine::boundPassable(ll_1.leftBound(), ll_1.regulatoryElementsAs<PassingControlLine>(),
                                                false, lanelet::Participants::Vehicle));

  // Check inverted bound
  ASSERT_TRUE(PassingControlLine::boundPassable(ll_1.leftBound().invert(),
                                                ll_1.regulatoryElementsAs<PassingControlLine>(), true,
                                                lanelet::Participants::Vehicle));
  ASSERT_FALSE(PassingControlLine::boundPassable(ll_1.leftBound().invert(),
                                                 ll_1.regulatoryElementsAs<PassingControlLine>(), false,
                                                 lanelet::Participants::Vehicle));
}

}  // namespace lanelet
