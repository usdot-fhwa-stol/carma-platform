#pragma once
/*
 * Copyright (C) 2021 LEIDOS.
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
#include <ros/console.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_wm/WMTestLibForGuidance.h>

// Unit tests for transition table
namespace wz_strategic_plugin
{
/**
 * \brief TODO
 */
class WorkZoneTestFixture : public ::testing::Test
{
  /**
   *  - getGuidanceTestMap gives a simple one way, 3 lane map (25mph speed limit) with one static prebaked obstacle and
   *      4 lanelets in a lane (if 2 stripes make up one lanelet):
   *
   *        |1203|1213|1223|
   *        | _  _  _  _  _|
   *        |1202| Ob |1222|
   *        | _  _  _  _  _|
   *        |1201|1211|1221|    num   = lanelet id hardcoded for easier testing
   *        | _  _  _  _  _|    |     = lane lines
   *        |1200|1210|1220|    - - - = Lanelet boundary
   *        |              |    O     = Default Obstacle
   *        ****************
   *           START_LINE
   */

protected:
  void SetUp() override
  {
    carma_wm::test::MapOptions options;
    options.lane_length_ = 25;
    options.lane_width_ = 3.7;
    options.speed_limit_ = carma_wm::test::MapOptions::SpeedLimit::DEFAULT;
    options.obstacle_ = carma_wm::test::MapOptions::Obstacle::NONE;

    cmw_ = carma_wm::test::getGuidanceTestMap(options);

    carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, cmw_);
  }

  // void TearDown() override {}

  std::shared_ptr<carma_wm::CARMAWorldModel> cmw_;
};

}  // namespace wz_strategic_plugin