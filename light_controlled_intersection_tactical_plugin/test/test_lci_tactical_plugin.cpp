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
TEST(LCITacticalPluginTest, determineSpeedProfileCasetest)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  LightControlledIntersectionTacticalPluginConfig config;
  config.minimum_speed = 2;
  config.vehicle_accel_limit= 2;
  config.vehicle_accel_limit_multiplier= 1;
  config.vehicle_decel_limit= 1;
  config.vehicle_decel_limit_multiplier= 1;


  LightControlledIntersectionTacticalPlugin lci(wm, config, [&](auto msg) {});
  lci.speed_limit_ = 10;

  auto case_num1 = lci.determineSpeedProfileCase(10, 12, 9, 8);

  EXPECT_EQ(3, case_num1);

  auto case_num2 = lci.determineSpeedProfileCase(10, 12, 12, 1);

  EXPECT_EQ(4, case_num2);

  auto case_num3 = lci.determineSpeedProfileCase(10, 9, 1, 2);

  EXPECT_EQ(2, case_num3);

  auto case_num4 = lci.determineSpeedProfileCase(10, 9, 11, 1.4);

  EXPECT_EQ(1, case_num4);
}

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
  
  // ACCEL-CRUISE START
 
  lci.speed_limit_ = 1;
  double departure_speed = 1;
  double start_dist = 0;
  double end_dist = 5;
  double remaining_dist = end_dist - start_dist;
  double starting_speed = 0; 
  double remaining_time = 8;
  int n = 11;
  double step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = 5;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  
  double speed_before_decel = lci.calcSpeedBeforeDecel(remaining_time, remaining_dist, starting_speed, departure_speed);
  
  lci.apply_accel_cruise_decel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_decel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // ACCEL-CRUISE END

  // CRUISE DECEL START
  points_and_target_speeds = {};
  lci.speed_limit_ = 1;
  departure_speed = 0.5;
  start_dist = 0;
  end_dist = 5;
  remaining_dist = end_dist - start_dist;
  starting_speed = 1; 
  remaining_time = 6;
  n = 11;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = 5;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_decel = lci.calcSpeedBeforeDecel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_accel_cruise_decel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_decel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // CRUISE DECEL end

  // ACCEL DECEL START
  points_and_target_speeds = {};
  lci.speed_limit_ = 1;
  departure_speed = 0.5;
  start_dist = 0;
  end_dist = 5;
  remaining_dist = end_dist - start_dist;
  starting_speed = 0; 
  remaining_time = 10;
  n = 11;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = 5;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_decel = lci.calcSpeedBeforeDecel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_accel_cruise_decel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_decel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // ACCEL DECEL END

  // ACCEL CRUISE DECEL START
  points_and_target_speeds = {};
  lci.speed_limit_ = 1;
  departure_speed = 0.8;
  start_dist = 0;
  end_dist = 5;
  remaining_dist = end_dist - start_dist;
  starting_speed = 0; 
  remaining_time = 8;
  n = 11;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = 5;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_decel = lci.calcSpeedBeforeDecel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_accel_cruise_decel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_decel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // ACCEL CRUISE DECEL END

  // ONLY CRUISE START
  points_and_target_speeds = {};
  lci.speed_limit_ = 1;
  departure_speed = lci.speed_limit_;
  start_dist = 0;
  end_dist = 5;
  remaining_dist = end_dist - start_dist;
  starting_speed = lci.speed_limit_; 
  remaining_time = 5;
  n = 11;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = lci.speed_limit_;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_decel = lci.calcSpeedBeforeDecel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_accel_cruise_decel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_decel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // ONLY CRUISE END

  // BUFFERED ACCEL CRUISE DECEL START 
  points_and_target_speeds = {};
  lci.speed_limit_ = 1;
  departure_speed = 0.8;
  start_dist = 1;
  end_dist = 6;
  starting_speed = 0; 
  remaining_time = 8;
  remaining_dist = end_dist - start_dist;
  n = 13;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = lci.speed_limit_;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_decel = lci.calcSpeedBeforeDecel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_accel_cruise_decel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_decel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // BUFFERED ACCEL CRUISE DECEL END

  // SACRIFICE START NEGATIVE ACCEL PART START
  points_and_target_speeds = {};
  lci.speed_limit_ = 15;
  departure_speed = 15;
  start_dist = 0;
  end_dist = 225;
  starting_speed = 0; 
  remaining_time = 12.5;
  remaining_dist = end_dist - start_dist;
  n = 11;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = lci.speed_limit_;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_decel = lci.calcSpeedBeforeDecel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_accel_cruise_decel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_decel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // SACRIFICE START NEGATIVE ACCEL PART END

  // SACRIFICE START NEGATIVE ACCEL PART START
  points_and_target_speeds = {};
  lci.speed_limit_ = 15;
  departure_speed = 10;
  start_dist = 0;
  end_dist = 225;
  starting_speed = 0; 
  remaining_time = 12.5;
  remaining_dist = end_dist - start_dist;
  n = 11;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = lci.speed_limit_;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_decel = lci.calcSpeedBeforeDecel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_accel_cruise_decel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_decel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // SACRIFICE START NEGATIVE ACCEL PART END
}

TEST(LCITacticalPluginTest, apply_decel_cruise_accel_speed_profile_test)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  LightControlledIntersectionTacticalPluginConfig config;
  config.vehicle_accel_limit= 1;
  config.vehicle_accel_limit_multiplier= 1;
  config.vehicle_decel_limit= 1;
  config.vehicle_decel_limit_multiplier= 1;
  config.minimum_speed = 1;
  
  
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
  
  // DECEL-CRUISE START
 
  double departure_speed = 1;
  double start_dist = 0;
  double end_dist = 11;
  double remaining_dist = end_dist - start_dist;
  double starting_speed = 2; 
  double remaining_time = 8;
  int n = 11;
  double step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = starting_speed;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  
  double speed_before_accel = lci.calcSpeedBeforeAccel(remaining_time, remaining_dist, starting_speed, departure_speed);
  
  lci.apply_decel_cruise_accel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_accel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // DECEL-CRUISE START

  // CRUISE ACCEL START
  points_and_target_speeds = {};
  departure_speed = 2;
  start_dist = 0;
  end_dist = 11;
  remaining_dist = end_dist - start_dist;
  starting_speed = 1; 
  remaining_time = 8;
  n = 11;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = 5;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_accel = lci.calcSpeedBeforeAccel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_decel_cruise_accel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_accel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // CRUISE ACCEL end
  
  
  // DECEL ACCEL START
  points_and_target_speeds = {};
  lci.speed_limit_ = 3;
  departure_speed = 2;
  start_dist = 0;
  end_dist = 11;
  remaining_dist = end_dist - start_dist;
  starting_speed = 2; 
  remaining_time = 5.6;
  n = 11;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = 5;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_accel = lci.calcSpeedBeforeAccel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_decel_cruise_accel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_accel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);
  
  // DECEL ACCEL END
  
  // DECEL CRUISE ACCEL START
  points_and_target_speeds = {};
  departure_speed = 1.5;
  start_dist = 0;
  end_dist = 11;
  remaining_dist = end_dist - start_dist;
  starting_speed = 2; 
  remaining_time = 7;
  n = 11;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = 5;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_accel = lci.calcSpeedBeforeAccel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_decel_cruise_accel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_accel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);
  
  // DECEL CRUISE ACCEL END
  
  // ONLY CRUISE START
  points_and_target_speeds = {};
  departure_speed = lci.speed_limit_;
  start_dist = 0;
  end_dist = 5;
  remaining_dist = end_dist - start_dist;
  starting_speed = lci.speed_limit_; 
  remaining_time = 5;
  n = 11;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = lci.speed_limit_;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_accel = lci.calcSpeedBeforeAccel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_decel_cruise_accel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_accel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);
  
  // ONLY CRUISE END
  
  // BUFFERED DECEL CRUISE ACCEL START 
  points_and_target_speeds = {};
  departure_speed = 1.5;
  start_dist = 1;
  end_dist = 12;
  starting_speed = 2; 
  remaining_time = 7;
  remaining_dist = end_dist - start_dist;
  n = 13;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = lci.speed_limit_;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_accel = lci.calcSpeedBeforeAccel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_decel_cruise_accel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_accel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // BUFFERED DECEL CRUISE ACCEL END
  
  
  // SACRIFICE START NEGATIVE ACCEL PART START
  points_and_target_speeds = {};
  departure_speed = 3;
  start_dist = 0;
  end_dist = 10;
  starting_speed = 2; 
  remaining_time = 12.5;
  remaining_dist = end_dist - start_dist;
  n = 11;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = lci.speed_limit_;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_accel = lci.calcSpeedBeforeAccel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_decel_cruise_accel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_accel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // SACRIFICE START NEGATIVE ACCEL PART START
  
  // SACRIFICE START POSITIVE DECEL PART START
  points_and_target_speeds = {};
  departure_speed = 2;
  start_dist = 0;
  end_dist = 10;
  starting_speed = 2; 
  remaining_time = 15;
  remaining_dist = end_dist - start_dist;
  n = 11;
  step = end_dist / (n - 1);
  for (auto i = 0; i < n; i++)
  {
    p.point = {0.5, i * step};
    p.speed = lci.speed_limit_;
    points_and_target_speeds.push_back(p);
  }

  ROS_DEBUG_STREAM("\n Estimated time: " << lci.calcEstimatedEntryTimeLeft(remaining_dist, starting_speed, departure_speed ) 
                                         << ", scheduled time: " << remaining_time);
  speed_before_accel = lci.calcSpeedBeforeAccel(remaining_time, remaining_dist, starting_speed, departure_speed);

  lci.apply_decel_cruise_accel_speed_profile(wm, points_and_target_speeds, start_dist, end_dist, 
                                    remaining_time, starting_speed, speed_before_accel, departure_speed);

  EXPECT_NEAR(points_and_target_speeds.front().speed, starting_speed, 0.001);
  EXPECT_NEAR(points_and_target_speeds.back().speed, departure_speed, 0.001);

  // SACRIFICE START POSITIVE DECEL PART END
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