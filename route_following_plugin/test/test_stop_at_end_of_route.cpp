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

#include "route_following_plugin.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <string>

namespace route_following_plugin
{
/**
 * NOTE: This file contains test cases labeled by number
 *        These test cases refer to the various scenarios shown here route_following_plugin/resource/media/StopAndWaitTestCases.png
 */ 
class StopAndWaitTestFixture : public ::testing::Test
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

TEST_F(StopAndWaitTestFixture, CaseOne)
{
  auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
  worker->wm_ = cmw_;  // Set world model from test fixture

  double entry_speed = 10.0;
  double long_accel_limit = 2.0;
  double lat_accel_limit = 1.0;
  double min_maneuver_length = 10.0;
  double route_end_downtrack = 100.0;

  carma_planning_msgs::msg::Maneuver m1;
  m1.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  m1.lane_following_maneuver.start_dist = 0;
  m1.lane_following_maneuver.end_dist = 40;
  m1.lane_following_maneuver.start_speed = entry_speed;
  m1.lane_following_maneuver.end_speed = entry_speed;
  m1.lane_following_maneuver.lane_ids = { "1200", "1201" };

  auto result = worker->addStopAndWaitAtRouteEnd({ m1 }, route_end_downtrack, entry_speed, long_accel_limit,
                                                lat_accel_limit, min_maneuver_length);

  ASSERT_EQ(3, result.size());

  // m1
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING, result[0].type);
  ASSERT_NEAR(0.0, result[0].lane_following_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(40.0, result[0].lane_following_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_following_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_following_maneuver.end_speed, 0.00001);
  ASSERT_EQ(2, result[0].lane_following_maneuver.lane_ids.size());
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[0].compare("1200") == 0);
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[1].compare("1201") == 0);

  // Extra lane follow
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING, result[1].type);
  ASSERT_NEAR(40.0, result[1].lane_following_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(75.0, result[1].lane_following_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[1].lane_following_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(entry_speed, result[1].lane_following_maneuver.end_speed, 0.00001);
  ASSERT_EQ(2, result[1].lane_following_maneuver.lane_ids.size());
  ASSERT_TRUE(result[1].lane_following_maneuver.lane_ids[0].compare("1201") == 0);
  ASSERT_TRUE(result[1].lane_following_maneuver.lane_ids[1].compare("1202") == 0);

  // Stop And Wait
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT, result[2].type);
  ASSERT_NEAR(75.0, result[2].stop_and_wait_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(100.0, result[2].stop_and_wait_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[2].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_TRUE(result[2].stop_and_wait_maneuver.starting_lane_id.compare("1203") == 0);
  ASSERT_TRUE(result[2].stop_and_wait_maneuver.ending_lane_id.compare("1203") == 0);
}

TEST_F(StopAndWaitTestFixture, CaseTwo)
{
  auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
  worker->wm_ = cmw_;  // Set world model from test fixture

  double entry_speed = 10.0;
  double long_accel_limit = 2.0;
  double lat_accel_limit = 1.0;
  double min_maneuver_length = 10.0;
  double route_end_downtrack = 100.0;

  carma_planning_msgs::msg::Maneuver m1;
  m1.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  m1.lane_following_maneuver.start_dist = 0;
  m1.lane_following_maneuver.end_dist = 70;
  m1.lane_following_maneuver.start_speed = entry_speed;
  m1.lane_following_maneuver.end_speed = entry_speed;
  m1.lane_following_maneuver.lane_ids = { "1200", "1201", "1202" };

  auto result = worker->addStopAndWaitAtRouteEnd({ m1 }, route_end_downtrack, entry_speed, long_accel_limit,
                                                lat_accel_limit, min_maneuver_length);

  ASSERT_EQ(2, result.size());

  // m1
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING, result[0].type);
  ASSERT_NEAR(0.0, result[0].lane_following_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(70.0, result[0].lane_following_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_following_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_following_maneuver.end_speed, 0.00001);
  ASSERT_EQ(3, result[0].lane_following_maneuver.lane_ids.size());
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[0].compare("1200") == 0);
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[1].compare("1201") == 0);
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[2].compare("1202") == 0);

  // Stop And Wait
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT, result[1].type);
  ASSERT_NEAR(70.0, result[1].stop_and_wait_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(100.0, result[1].stop_and_wait_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[1].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_TRUE(result[1].stop_and_wait_maneuver.starting_lane_id.compare("1202") == 0);
  ASSERT_TRUE(result[1].stop_and_wait_maneuver.ending_lane_id.compare("1203") == 0);
}

TEST_F(StopAndWaitTestFixture, CaseThree)
{
  auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
  worker->wm_ = cmw_;  // Set world model from test fixture

  double entry_speed = 10.0;
  double long_accel_limit = 2.0;
  double lat_accel_limit = 1.0;
  double min_maneuver_length = 10.0;
  double route_end_downtrack = 100.0;

  carma_planning_msgs::msg::Maneuver m1;
  m1.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  m1.lane_following_maneuver.start_dist = 0;
  m1.lane_following_maneuver.end_dist = 78;
  m1.lane_following_maneuver.start_speed = entry_speed;
  m1.lane_following_maneuver.end_speed = entry_speed;
  m1.lane_following_maneuver.lane_ids = { "1200", "1201", "1202", "1203" };

  auto result = worker->addStopAndWaitAtRouteEnd({ m1 }, route_end_downtrack, entry_speed, long_accel_limit,
                                                lat_accel_limit, min_maneuver_length);

  ASSERT_EQ(2, result.size());

  // m1
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING, result[0].type);
  ASSERT_NEAR(0.0, result[0].lane_following_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(75.0, result[0].lane_following_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_following_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_following_maneuver.end_speed, 0.00001);
  ASSERT_EQ(3, result[0].lane_following_maneuver.lane_ids.size());
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[0].compare("1200") == 0);
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[1].compare("1201") == 0);
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[2].compare("1202") == 0);

  // Stop And Wait
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT, result[1].type);
  ASSERT_NEAR(75.0, result[1].stop_and_wait_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(100.0, result[1].stop_and_wait_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[1].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_TRUE(result[1].stop_and_wait_maneuver.starting_lane_id.compare("1203") == 0);
  ASSERT_TRUE(result[1].stop_and_wait_maneuver.ending_lane_id.compare("1203") == 0);
}

TEST_F(StopAndWaitTestFixture, CaseFour)
{
  auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
  worker->wm_ = cmw_;  // Set world model from test fixture

  double entry_speed = 10.0;
  double long_accel_limit = 2.0;
  double lat_accel_limit = 1.0;
  double min_maneuver_length = 10.0;
  double route_end_downtrack = 100.0;

  carma_planning_msgs::msg::Maneuver m1;
  m1.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  m1.lane_following_maneuver.start_dist = 0;
  m1.lane_following_maneuver.end_dist = 78;
  m1.lane_following_maneuver.start_speed = entry_speed;
  m1.lane_following_maneuver.end_speed = entry_speed;
  m1.lane_following_maneuver.lane_ids = { "1200", "1201", "1202", "1203" };

  carma_planning_msgs::msg::Maneuver m2;
  m2.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  m2.lane_following_maneuver.start_dist = 78;
  m2.lane_following_maneuver.end_dist = 90;
  m2.lane_following_maneuver.start_speed = entry_speed;
  m2.lane_following_maneuver.end_speed = entry_speed;
  m2.lane_following_maneuver.lane_ids = { "1203" };

  auto result = worker->addStopAndWaitAtRouteEnd({ m1, m2 }, route_end_downtrack, entry_speed, long_accel_limit,
                                                lat_accel_limit, min_maneuver_length);

  ASSERT_EQ(2, result.size()); // M2 should have been dropped

  // m1
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING, result[0].type);
  ASSERT_NEAR(0.0, result[0].lane_following_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(75.0, result[0].lane_following_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_following_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_following_maneuver.end_speed, 0.00001);
  ASSERT_EQ(3, result[0].lane_following_maneuver.lane_ids.size());
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[0].compare("1200") == 0);
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[1].compare("1201") == 0);
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[2].compare("1202") == 0);

  // Stop And Wait
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT, result[1].type);
  ASSERT_NEAR(75.0, result[1].stop_and_wait_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(100.0, result[1].stop_and_wait_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[1].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_TRUE(result[1].stop_and_wait_maneuver.starting_lane_id.compare("1203") == 0);
  ASSERT_TRUE(result[1].stop_and_wait_maneuver.ending_lane_id.compare("1203") == 0);
}

TEST_F(StopAndWaitTestFixture, CaseFive)
{
  auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
  worker->wm_ = cmw_;  // Set world model from test fixture

  double entry_speed = 10.0;
  double long_accel_limit = 2.0;
  double lat_accel_limit = 1.0;
  double min_maneuver_length = 10.0;
  double route_end_downtrack = 100.0;

  carma_planning_msgs::msg::Maneuver m1;
  m1.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  m1.lane_following_maneuver.start_dist = 0;
  m1.lane_following_maneuver.end_dist = 70;
  m1.lane_following_maneuver.start_speed = entry_speed;
  m1.lane_following_maneuver.end_speed = entry_speed;
  m1.lane_following_maneuver.lane_ids = { "1200", "1201", "1202" };

  carma_planning_msgs::msg::Maneuver m2;
  m2.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  m2.lane_following_maneuver.start_dist = 70;
  m2.lane_following_maneuver.end_dist = 90;
  m2.lane_following_maneuver.start_speed = entry_speed;
  m2.lane_following_maneuver.end_speed = entry_speed;
  m2.lane_following_maneuver.lane_ids = { "1202", "1203" };

  auto result = worker->addStopAndWaitAtRouteEnd({ m1, m2 }, route_end_downtrack, entry_speed, long_accel_limit,
                                                lat_accel_limit, min_maneuver_length);

  ASSERT_EQ(2, result.size()); // M2 should have been dropped

  // m1
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING, result[0].type);
  ASSERT_NEAR(0.0, result[0].lane_following_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(70.0, result[0].lane_following_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_following_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_following_maneuver.end_speed, 0.00001);
  ASSERT_EQ(3, result[0].lane_following_maneuver.lane_ids.size());
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[0].compare("1200") == 0);
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[1].compare("1201") == 0);
  ASSERT_TRUE(result[0].lane_following_maneuver.lane_ids[2].compare("1202") == 0);

  // Stop And Wait should have been extended
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT, result[1].type);
  ASSERT_NEAR(70.0, result[1].stop_and_wait_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(100.0, result[1].stop_and_wait_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[1].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_TRUE(result[1].stop_and_wait_maneuver.starting_lane_id.compare("1202") == 0);
  ASSERT_TRUE(result[1].stop_and_wait_maneuver.ending_lane_id.compare("1203") == 0);
}

TEST_F(StopAndWaitTestFixture, CaseSix)
{
  auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
  worker->wm_ = cmw_;  // Set world model from test fixture

  double entry_speed = 10.0;
  double long_accel_limit = 2.0;
  double lat_accel_limit = 1.0;
  double min_maneuver_length = 10.0;
  double route_end_downtrack = 100.0;

  carma_planning_msgs::msg::Maneuver m1;
  m1.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
  m1.lane_change_maneuver.start_dist = 50;
  m1.lane_change_maneuver.end_dist = 70;
  m1.lane_change_maneuver.start_speed = entry_speed;
  m1.lane_change_maneuver.end_speed = entry_speed;
  m1.lane_change_maneuver.starting_lane_id = "1202";
  m1.lane_change_maneuver.ending_lane_id = "1202";

  auto result = worker->addStopAndWaitAtRouteEnd({ m1 }, route_end_downtrack, entry_speed, long_accel_limit,
                                                lat_accel_limit, min_maneuver_length);

  ASSERT_EQ(2, result.size());

  // m1
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_CHANGE, result[0].type);
  ASSERT_NEAR(50.0, result[0].lane_change_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(70.0, result[0].lane_change_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_change_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_change_maneuver.end_speed, 0.00001);
  ASSERT_TRUE(result[0].lane_change_maneuver.starting_lane_id.compare("1202") == 0);
  ASSERT_TRUE(result[0].lane_change_maneuver.ending_lane_id.compare("1202") == 0);

  // Stop And Wait should have been extended
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT, result[1].type);
  ASSERT_NEAR(70.0, result[1].stop_and_wait_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(100.0, result[1].stop_and_wait_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[1].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_TRUE(result[1].stop_and_wait_maneuver.starting_lane_id.compare("1202") == 0);
  ASSERT_TRUE(result[1].stop_and_wait_maneuver.ending_lane_id.compare("1203") == 0);
}

TEST_F(StopAndWaitTestFixture, CaseSeven)
{
  auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
  worker->wm_ = cmw_;  // Set world model from test fixture

  double entry_speed = 10.0;
  double long_accel_limit = 2.0;
  double lat_accel_limit = 1.0;
  double min_maneuver_length = 10.0;
  double route_end_downtrack = 100.0;

  carma_planning_msgs::msg::Maneuver m1;
  m1.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
  m1.lane_change_maneuver.start_dist = 30;
  m1.lane_change_maneuver.end_dist = 58;
  m1.lane_change_maneuver.start_speed = entry_speed;
  m1.lane_change_maneuver.end_speed = entry_speed;
  m1.lane_change_maneuver.starting_lane_id = "1201";
  m1.lane_change_maneuver.ending_lane_id = "1202";

  auto result = worker->addStopAndWaitAtRouteEnd({ m1 }, route_end_downtrack, entry_speed, long_accel_limit,
                                                lat_accel_limit, min_maneuver_length);

  ASSERT_EQ(3, result.size());

  // m1
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_CHANGE, result[0].type);
  ASSERT_NEAR(30.0, result[0].lane_change_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(58.0, result[0].lane_change_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_change_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_change_maneuver.end_speed, 0.00001);
  ASSERT_TRUE(result[0].lane_change_maneuver.starting_lane_id.compare("1201") == 0);
  ASSERT_TRUE(result[0].lane_change_maneuver.ending_lane_id.compare("1202") == 0);

// Extra lane follow
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING, result[1].type);
  ASSERT_NEAR(58.0, result[1].lane_following_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(75.0, result[1].lane_following_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[1].lane_following_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(entry_speed, result[1].lane_following_maneuver.end_speed, 0.00001);
  ASSERT_EQ(1, result[1].lane_following_maneuver.lane_ids.size());
  ASSERT_TRUE(result[1].lane_following_maneuver.lane_ids[0].compare("1202") == 0);

  // Stop And Wait 
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT, result[2].type);
  ASSERT_NEAR(75.0, result[2].stop_and_wait_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(100.0, result[2].stop_and_wait_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[2].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_TRUE(result[2].stop_and_wait_maneuver.starting_lane_id.compare("1203") == 0);
  ASSERT_TRUE(result[2].stop_and_wait_maneuver.ending_lane_id.compare("1203") == 0);
}

TEST_F(StopAndWaitTestFixture, CaseEight)
{
  auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
  worker->wm_ = cmw_;  // Set world model from test fixture

  double entry_speed = 10.0;
  double long_accel_limit = 2.0;
  double lat_accel_limit = 1.0;
  double min_maneuver_length = 10.0;
  double route_end_downtrack = 100.0;

  carma_planning_msgs::msg::Maneuver m1;
  m1.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
  m1.lane_change_maneuver.start_dist = 50.01;
  m1.lane_change_maneuver.end_dist = 85;
  m1.lane_change_maneuver.start_speed = entry_speed;
  m1.lane_change_maneuver.end_speed = entry_speed;
  m1.lane_change_maneuver.starting_lane_id = "1202";
  m1.lane_change_maneuver.ending_lane_id = "1203";

  auto result = worker->addStopAndWaitAtRouteEnd({ m1 }, route_end_downtrack, entry_speed, long_accel_limit,
                                                lat_accel_limit, min_maneuver_length);

  ASSERT_EQ(2, result.size());

  // m1
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_CHANGE, result[0].type);
  ASSERT_NEAR(50.01, result[0].lane_change_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(75.0, result[0].lane_change_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_change_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].lane_change_maneuver.end_speed, 0.00001);
  ASSERT_TRUE(result[0].lane_change_maneuver.starting_lane_id.compare("1202") == 0);
  ASSERT_TRUE(result[0].lane_change_maneuver.ending_lane_id.compare("1202") == 0);

  // Stop And Wait 
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT, result[1].type);
  ASSERT_NEAR(75.0, result[1].stop_and_wait_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(100.0, result[1].stop_and_wait_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[1].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_TRUE(result[1].stop_and_wait_maneuver.starting_lane_id.compare("1203") == 0);
  ASSERT_TRUE(result[1].stop_and_wait_maneuver.ending_lane_id.compare("1203") == 0);
}

TEST_F(StopAndWaitTestFixture, CaseNine)
{
  auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
  worker->wm_ = cmw_;  // Set world model from test fixture

  double entry_speed = 10.0;
  double long_accel_limit = 2.0;
  double lat_accel_limit = 1.0;
  double min_maneuver_length = 10.0;
  double route_end_downtrack = 100.0;

  carma_planning_msgs::msg::Maneuver m1;
  m1.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
  m1.lane_change_maneuver.start_dist = 70;
  m1.lane_change_maneuver.end_dist = 90;
  m1.lane_change_maneuver.start_speed = entry_speed;
  m1.lane_change_maneuver.end_speed = entry_speed;
  m1.lane_change_maneuver.starting_lane_id = "1202";
  m1.lane_change_maneuver.ending_lane_id = "1203";

  // Cannot shrink lane change maneuver this much. Expect exception to be thrown
  ASSERT_THROW(worker->addStopAndWaitAtRouteEnd({ m1 }, route_end_downtrack, entry_speed, long_accel_limit,
                                                lat_accel_limit, min_maneuver_length), std::invalid_argument);
}

TEST_F(StopAndWaitTestFixture, CaseTen)
{
  auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
  worker->wm_ = cmw_;  // Set world model from test fixture

  double entry_speed = 10.0;
  double long_accel_limit = 2.0;
  double lat_accel_limit = 1.0;
  double min_maneuver_length = 10.0;
  double route_end_downtrack = 100.0;

  auto result = worker->addStopAndWaitAtRouteEnd({ }, route_end_downtrack, entry_speed, long_accel_limit,
                                                lat_accel_limit, min_maneuver_length);

  ASSERT_EQ(1, result.size());

  // Stop And Wait
  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT, result[0].type);
  ASSERT_NEAR(75.0, result[0].stop_and_wait_maneuver.start_dist, 0.00001);
  ASSERT_NEAR(100.0, result[0].stop_and_wait_maneuver.end_dist, 0.00001);
  ASSERT_NEAR(entry_speed, result[0].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_TRUE(result[0].stop_and_wait_maneuver.starting_lane_id.compare("1203") == 0);
  ASSERT_TRUE(result[0].stop_and_wait_maneuver.ending_lane_id.compare("1203") == 0);
}

}  // namespace route_following_plugin

