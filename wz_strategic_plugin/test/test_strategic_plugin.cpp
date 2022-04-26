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

#include <gtest/gtest.h>
#include <ros/console.h>
#include "test_fixture.h"
#include "wz_strategic_plugin/wz_strategic_plugin.h"
#include "wz_strategic_plugin/wz_strategic_plugin_config.h"

// Unit tests for strategic plugin
namespace wz_strategic_plugin
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
TEST_F(WorkZoneTestFixture, planManeuverCb)
{
  WzStrategicPluginConfig config;
  WzStrategicPlugin wzp(cmw_, config);

  cav_srvs::PlanManeuversRequest req;
  cav_srvs::PlanManeuversResponse resp;

  // Light will be located on lanelet 1200 (25m) and control lanelet 1202, 1203
  lanelet::Id traffic_light_id = lanelet::utils::getId();
  carma_wm::test::addTrafficLight(cmw_, traffic_light_id, {1201}, { 1202 });

  ROS_DEBUG("Out of range test ");
  req.header.stamp = ros::Time(0);
  req.veh_x = 1.85;
  req.veh_y = 1.0; // Out of range of light which should kick in at 8.36567466667
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 11.176;
  req.veh_lane_id = "1200";
  
  wzp.planManeuverCb(req, resp);
  ROS_ERROR_STREAM("resp.newplanmaneuvers size: "<<resp.new_plan.maneuvers.size());

  ASSERT_TRUE(resp.new_plan.maneuvers.empty());

  ROS_DEBUG("In range test ");
  req = cav_srvs::PlanManeuversRequest();
  req.veh_x = 1.85;
  req.veh_y = 10.0; // In approach range
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 11.176;
  req.veh_lane_id = "1200";

  wzp.planManeuverCb(req, resp);

  ASSERT_EQ(1, resp.new_plan.maneuvers.size());
  ASSERT_EQ(cav_msgs::Maneuver::STOP_AND_WAIT, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(10.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_dist );
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_time.toSec(), 0.00001);
  ASSERT_NEAR(11.176, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(config.min_maneuver_planning_period, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.end_time.toSec(), 0.001);
  ASSERT_NEAR(45.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.end_dist, 0.0001);
  ASSERT_TRUE(resp.new_plan.maneuvers[0].stop_and_wait_maneuver.starting_lane_id.compare("1200") == 0);
  ASSERT_TRUE(resp.new_plan.maneuvers[0].stop_and_wait_maneuver.ending_lane_id.compare("1202") == 0);

  ROS_DEBUG("Waiting test ");
  req = cav_srvs::PlanManeuversRequest();
  ros::Time::setNow(ros::Time(6.0));
  req.header.stamp = ros::Time(6.0); // In red phase
  req.veh_x = 1.85;
  req.veh_y = 49.0; // At one meter infront of stop bar at a stop
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 0.0;
  req.veh_lane_id = "1201";

  wzp.planManeuverCb(req, resp);

  ASSERT_EQ(1, resp.new_plan.maneuvers.size());
  ASSERT_EQ(cav_msgs::Maneuver::STOP_AND_WAIT, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(39.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_dist );
  ASSERT_NEAR(6.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_time.toSec(), 0.00001);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_NEAR((ros::Time(6.0) + ros::Duration(config.min_maneuver_planning_period)).toSec(), resp.new_plan.maneuvers[0].stop_and_wait_maneuver.end_time.toSec(), 0.001);
  ASSERT_NEAR(50.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.end_dist, 0.0001);
  ASSERT_TRUE(resp.new_plan.maneuvers[0].stop_and_wait_maneuver.starting_lane_id.compare("1201") == 0);
  ASSERT_TRUE(resp.new_plan.maneuvers[0].stop_and_wait_maneuver.ending_lane_id.compare("1201") == 0);
  
  ROS_DEBUG("Departing test ");
  req = cav_srvs::PlanManeuversRequest();
  ros::Time::setNow(ros::Time(25.0));
  req.header.stamp = ros::Time(25.0); // In green phase
  req.veh_x = 1.85;
  req.veh_y = 49.0; // At one meter infront of stop bar at a stop
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 0.0;
  req.veh_lane_id = "1201";

  wzp.planManeuverCb(req, resp);

  ASSERT_EQ(1, resp.new_plan.maneuvers.size());
  ASSERT_EQ(cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(49.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_dist );
  ASSERT_NEAR(25.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_time.toSec(), 0.00001);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_speed, 0.00001);
  ASSERT_NEAR((ros::Time(25.0) + ros::Duration(4.473872536388049)).toSec(), resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.end_time.toSec(), 0.25);
  ASSERT_NEAR(75.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.end_dist, 0.0001);
  ASSERT_NEAR(11.176, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.end_speed, 0.00001);
  ASSERT_TRUE(resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.starting_lane_id.compare("1201") == 0);
  ASSERT_TRUE(resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.ending_lane_id.compare("1203") == 0);

  ROS_DEBUG("Exit test ");
  req = cav_srvs::PlanManeuversRequest();
  ros::Time::setNow(ros::Time(30.0));
  req.header.stamp = ros::Time(30.0); // In green phase
  req.veh_x = 1.85;
  req.veh_y = 76.0; // past intersection
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 11.176;
  req.veh_lane_id = "1203";

  wzp.planManeuverCb(req, resp);

  ASSERT_TRUE(resp.new_plan.maneuvers.empty());


  ROS_DEBUG("Reset to evaluate flow throw behavior");
  req = cav_srvs::PlanManeuversRequest();
  ros::Time::setNow(ros::Time(25.0));
  req.header.stamp = ros::Time(25.0); // In green phase
  req.veh_x = 1.85;
  req.veh_y = 10.0; // In approach range
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 11.176;
  req.veh_lane_id = "1200";

  wzp.planManeuverCb(req, resp);

  ASSERT_EQ(2, resp.new_plan.maneuvers.size());

  ASSERT_EQ(cav_msgs::Maneuver::LANE_FOLLOWING, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(10.0, resp.new_plan.maneuvers[0].lane_following_maneuver.start_dist);
  ASSERT_NEAR(25.0, resp.new_plan.maneuvers[0].lane_following_maneuver.start_time.toSec(), 0.00001);
  ASSERT_NEAR(11.176, resp.new_plan.maneuvers[0].lane_following_maneuver.start_speed, 0.00001);
  ASSERT_NEAR((ros::Time(25.0) + ros::Duration(3.13171080888)).toSec(), resp.new_plan.maneuvers[0].lane_following_maneuver.end_time.toSec(), 0.25);
  ASSERT_NEAR(45.0, resp.new_plan.maneuvers[0].lane_following_maneuver.end_dist, 0.0001);
  ASSERT_NEAR(11.176, resp.new_plan.maneuvers[0].lane_following_maneuver.end_speed, 0.00001);
  ASSERT_EQ(3, resp.new_plan.maneuvers[0].lane_following_maneuver.lane_ids.size());
  ASSERT_TRUE(resp.new_plan.maneuvers[0].lane_following_maneuver.lane_ids[0].compare("1200") == 0);
  ASSERT_TRUE(resp.new_plan.maneuvers[0].lane_following_maneuver.lane_ids[1].compare("1201") == 0);


  ASSERT_EQ(cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT, resp.new_plan.maneuvers[1].type);
  ASSERT_EQ(45.0, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.start_dist );
  ASSERT_NEAR((ros::Time(25.0) + ros::Duration(3.13171080888)).toSec(), resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.start_time.toSec(), 0.25);
  ASSERT_NEAR(11.176, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.start_speed, 0.00001);
  ASSERT_NEAR((ros::Time(25.0) + ros::Duration(2.23693629205 + 3.13171080888)).toSec(), resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.end_time.toSec(), 0.25);
  ASSERT_NEAR(75.0, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.end_dist, 0.0001);
  ASSERT_NEAR(11.176, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.end_speed, 0.00001);
  ASSERT_TRUE(resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.starting_lane_id.compare("1202") == 0);
  ASSERT_TRUE(resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.ending_lane_id.compare("1202") == 0);
}
}  // namespace wz_strategic_plugin