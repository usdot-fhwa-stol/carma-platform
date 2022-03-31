/*
 * Copyright (C) 2022 LEIDOS.
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

 TEST(LCITacticalPluginTest, apply_accel_cruise_decel_speed_profile_test)
 {
   std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
   LightControlledIntersectionTacticalPluginConfig config;
   config.vehicle_accel_limit= 1;
   config.vehicle_accel_limit_multiplier= 1;
   config.vehicle_decel_limit= 1;
   config.vehicle_decel_limit_multiplier= 1;

   LightControlledIntersectionTacticalPlugin lci(wm, config, [&](auto msg) {});
   auto map = carma_wm::test::buildGuidanceTestMap(2, 2);
   wm->setMap(map);
   carma_wm::test::setSpeedLimit(15_mph, wm);
   /**
    * Total route length should be 100m
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
   carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

   PointSpeedPair p;
   std::vector<PointSpeedPair> points_and_target_speeds;

   lci.speed_limit_ = 1;
   double a_accel = 1;
   double a_decel = -1;
   double dist_decel = 2;
   double dist_accel = 2;
   double dist_cruise = 2;
   double speed_before_accel = -1;
   double speed_before_decel = 2;
   SpeedProfileCase case_num = static_cast<SpeedProfileCase>(1);
   double starting_downtrack = 0;
   double ending_downtrack = 8;
   double departure_speed = 0;
   double scheduled_entry_time = 6;
   double entry_dist = ending_downtrack - starting_downtrack;
   double starting_speed = 0;
   int n = 9;
   double step = ending_downtrack / (n - 1);
   for (auto i = 0; i < n; i++)
   {
     p.point = {0.5, i * step};
     p.speed = 5;
     points_and_target_speeds.push_back(p);
   }
    // change speed profile depending on algorithm case starting from maneuver start_dist
   if(case_num == ACCEL_CRUISE_DECEL || case_num == ACCEL_DECEL){
     // acceleration (cruising if needed) then deceleration to reach desired intersection entry speed/time according to algorithm doc
     lci.apply_accel_cruise_decel_speed_profile(wm, points_and_target_speeds, starting_downtrack, entry_dist, starting_speed, speed_before_decel, 
                                             departure_speed, dist_accel, dist_cruise, dist_decel, a_accel, a_decel);
   }
   else if(case_num == DECEL_ACCEL || case_num == DECEL_CRUISE_ACCEL)
   {
     // deceleration (cruising if needed) then acceleration to reach desired intersection entry speed/time according to algorithm doc
     lci.apply_decel_cruise_accel_speed_profile(wm, points_and_target_speeds, starting_downtrack, entry_dist, starting_speed, speed_before_accel, 
                                             departure_speed, dist_accel, dist_cruise, dist_decel, a_accel, a_decel);
   }
   EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
   EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);
 }

} // namespace light_controlled_intersection_transit_plugin

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