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

#include <light_controlled_intersection_plugin.h>
#include <carma_wm/WMTestLibForGuidance.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/CARMAWorldModel.h>
#include <cav_msgs/LaneFollowingManeuver.h>
#include <cav_msgs/VehicleState.h>
#include <cav_srvs/PlanManeuversRequest.h>
#include <cav_srvs/PlanManeuversResponse.h>

namespace light_controlled_intersection_transit_plugin
{
TEST(LCIStrategicPluginTest, determineSpeedProfileCasetest)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  LightControlledIntersectionTacticalPluginConfig config;
  config.vehicle_accel_limit= 2;
  config.vehicle_accel_limit_multiplier= 1;
  config.vehicle_decel_limit= 1;
  config.vehicle_decel_limit_multiplier= 1;
  
  LightControlledIntersectionTacticalPlugin lci(wm, config, [&](auto msg) {});

  auto case_num1 = lci.determineSpeedProfileCase(10, 3, 9, 2.5);

  EXPECT_EQ(3, case_num1);

  auto case_num2 = lci.determineSpeedProfileCase(10, 5, 4, 1.4);

  EXPECT_EQ(4, case_num2);

  auto case_num3 = lci.determineSpeedProfileCase(2, 1, 1, 2);

  EXPECT_EQ(2, case_num3);

  auto case_num4 = lci.determineSpeedProfileCase(5, 5, 1, 1.4);

  EXPECT_EQ(1, case_num4);
}

}

// Run all the tests
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  ROSCONSOLE_AUTOINIT;
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) { // Change to Debug to enable debug logs
    ros::console::notifyLoggerLevelsChanged();
  }
  auto res = RUN_ALL_TESTS();
  
  return res;
}