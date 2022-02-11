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
#include "lci_strategic_plugin/lci_strategic_plugin.h"
#include "lci_strategic_plugin/lci_strategic_plugin_config.h"

// Unit tests for strategic plugin
namespace lci_strategic_plugin
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
TEST_F(LCIStrategicTestFixture, planManeuverCb)
{
  LCIStrategicPluginConfig config;
  LCIStrategicPlugin lcip(cmw_, config);

  cav_srvs::PlanManeuversRequest req;
  cav_srvs::PlanManeuversResponse resp;

  // Light will be located on lanelet 1200 (300m) and control lanelet 1202, 1203
  lanelet::Id traffic_light_id = lanelet::utils::getId();
  carma_wm::test::addTrafficLight(cmw_, traffic_light_id, 1200, { 1203 });

  ROS_WARN("Out of range test ");
  req.header.stamp = ros::Time(0);
  req.veh_x = 1.85;
  req.veh_y = 1.0; // Out of range of light which should kick in at 100 meters at 11.176 m/s
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 11.176;
  req.veh_lane_id = "1200";
  
  lcip.planManeuverCb(req, resp);

  ASSERT_TRUE(resp.new_plan.maneuvers.empty());

  ROS_WARN(">>>>>>>>>>>>>>>>>>>>>>>>>>>In range test: GREEN");
  req = cav_srvs::PlanManeuversRequest();
  req.veh_x = 1.85;
  req.veh_y = 130; // In approach range
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 11.176;
  req.veh_lane_id = "1200";

  lcip.planManeuverCb(req, resp);

  ASSERT_EQ(2, resp.new_plan.maneuvers.size());
  ASSERT_EQ(cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(130.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_dist );
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_time.toSec(), 0.00001);
  ASSERT_NEAR(req.veh_logitudinal_velocity, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(26.2, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.end_time.toSec(), 0.001);
  ASSERT_NEAR(295, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.end_dist, 0.0001);
  // check trajectory smoothing parameters:
  ASSERT_EQ("Carma/light_controlled_intersection", resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.string_valued_meta_data.front());
  ASSERT_NEAR(0.717332, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[0], 0.001);
  ASSERT_NEAR(-0.717332, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[1], 0.001);
  ASSERT_NEAR(83.5783, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[2], 0.001);
  ASSERT_NEAR(2.84336, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[3], 0.001);
  ASSERT_NEAR(83.5783, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[4], 0.001);
  ASSERT_NEAR(2.2352, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[5], 0.001);
  ASSERT_NEAR(-1, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[6], 0.001);
  ASSERT_EQ(4, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.int_valued_meta_data[0]);

  ASSERT_EQ(cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT, resp.new_plan.maneuvers[1].type);
  ASSERT_EQ(295, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.start_dist );
  ASSERT_NEAR(26.2, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.start_time.toSec(), 0.00001);
  ASSERT_NEAR(11.176, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(79.88647, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.end_time.toSec(), 0.001);
  ASSERT_NEAR(900, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.end_dist, 0.0001);

  ROS_WARN(">>>>>>>>>>>>>>>>>>>>>>>>>>>In range test: RED");
  req = cav_srvs::PlanManeuversRequest();
  req.veh_x = 1.85;
  req.veh_y = 225; // In approach range
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 11.176;
  req.veh_lane_id = "1200";

  lcip.planManeuverCb(req, resp);

  ASSERT_EQ(1, resp.new_plan.maneuvers.size());
  ASSERT_EQ(cav_msgs::Maneuver::STOP_AND_WAIT, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(req.veh_y, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_dist );
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_time.toSec(), 0.00001);
  ASSERT_NEAR(req.veh_logitudinal_velocity, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(15.1, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.end_time.toSec(), 0.001);
  ASSERT_NEAR(295, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.end_dist, 0.0001);

  ROS_WARN(">>>>>>>>>>>>>>>>>>>>>>>Waiting test ");
  req = cav_srvs::PlanManeuversRequest();
  ros::Time::setNow(ros::Time(6.0));
  req.header.stamp = ros::Time(6.0); // In red phase
  req.veh_x = 1.85;
  req.veh_y = 299.0; // At one meter infront of stop bar at a stop
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 0.0;
  req.veh_lane_id = "1201";

  lcip.planManeuverCb(req, resp);

  ASSERT_EQ(1, resp.new_plan.maneuvers.size());
  ASSERT_EQ(cav_msgs::Maneuver::STOP_AND_WAIT, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(289.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_dist );
  ASSERT_NEAR(6.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_time.toSec(), 0.00001);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_speed, 0.00001);
  ASSERT_NEAR((ros::Time(6.0) + ros::Duration(config.min_maneuver_planning_period)).toSec(), resp.new_plan.maneuvers[0].stop_and_wait_maneuver.end_time.toSec(), 0.001);
  ASSERT_NEAR(300.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.end_dist, 0.0001);
  ASSERT_TRUE(resp.new_plan.maneuvers[0].stop_and_wait_maneuver.starting_lane_id.compare("1201") == 0);
  ASSERT_TRUE(resp.new_plan.maneuvers[0].stop_and_wait_maneuver.ending_lane_id.compare("1201") == 0);
  
  ROS_WARN(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Departing test ");
  req = cav_srvs::PlanManeuversRequest();
  ros::Time::setNow(ros::Time(25.0));
  req.header.stamp = ros::Time(25.0); // In green phase
  req.veh_x = 1.85;
  req.veh_y = 299.0; // At one meter infront of stop bar at a stop
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 0.0;
  req.veh_lane_id = "1201";

  lcip.planManeuverCb(req, resp);

  ASSERT_EQ(1, resp.new_plan.maneuvers.size());
  ASSERT_EQ(cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(299.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_dist );
  ASSERT_NEAR(25.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_time.toSec(), 0.00001);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(ros::Time(132.55189).toSec(), resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.end_time.toSec(), 0.25);
  ASSERT_NEAR(900.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.end_dist, 0.0001);
  ASSERT_NEAR(11.176, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.end_speed, 0.00001);
  ASSERT_TRUE(resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.starting_lane_id.compare("1200") == 0);
  ASSERT_TRUE(resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.ending_lane_id.compare("1203") == 0);

  ROS_WARN(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Exit test ");
  req = cav_srvs::PlanManeuversRequest();
  ros::Time::setNow(ros::Time(30.0));
  req.header.stamp = ros::Time(30.0); // In green phase
  req.veh_x = 1.85;
  req.veh_y = 910; // past intersection
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 11.176;
  req.veh_lane_id = "1203";

  lcip.planManeuverCb(req, resp);

  ASSERT_TRUE(resp.new_plan.maneuvers.empty());

  ROS_WARN(">>>>>>>>>>>>>>>>>>>>>>>>>>>RESET: In range test: RED with 2 maneuvers");
  req = cav_srvs::PlanManeuversRequest();
  ros::Time::setNow(ros::Time(15.0));
  req.header.stamp = ros::Time(15.0);
  req.veh_x = 1.85;
  req.veh_y = 230; // In approach range
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 11.176;
  req.veh_lane_id = "1200";

  lcip.planManeuverCb(req, resp);

  ASSERT_EQ(2, resp.new_plan.maneuvers.size());
  ASSERT_EQ(cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(req.veh_y, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_dist);
  ASSERT_NEAR(15.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_time.toSec(), 0.00001);
  ASSERT_NEAR(req.veh_logitudinal_velocity, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(20.8446, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.end_time.toSec(), 0.001);
  ASSERT_NEAR(259.7, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.end_dist, 0.01);
  // trajectory smoothing part
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[0], 0.001);
  ASSERT_NEAR(-1.5, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[1], 0.001);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[2], 0.001);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[3], 0.001);
  ASSERT_NEAR(29.6998297, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[4], 0.001);
  ASSERT_NEAR(2.40904, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[5], 0.001);
  ASSERT_NEAR(-1, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[6], 0.001);
  ASSERT_EQ(3, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.int_valued_meta_data[0]);

  ASSERT_EQ(cav_msgs::Maneuver::STOP_AND_WAIT, resp.new_plan.maneuvers[1].type);
  ASSERT_NEAR(259.7, resp.new_plan.maneuvers[1].stop_and_wait_maneuver.start_dist, 0.1);
  ASSERT_NEAR(20.8446, resp.new_plan.maneuvers[1].stop_and_wait_maneuver.start_time.toSec(), 0.01);
  ASSERT_NEAR(2.40904, resp.new_plan.maneuvers[1].stop_and_wait_maneuver.start_speed, 0.01);
  ASSERT_NEAR(35.9446, resp.new_plan.maneuvers[1].stop_and_wait_maneuver.end_time.toSec(), 0.01);
  ASSERT_NEAR(295, resp.new_plan.maneuvers[1].stop_and_wait_maneuver.end_dist, 0.1);
  ASSERT_NEAR(0.0957666, resp.new_plan.maneuvers[1].stop_and_wait_maneuver.parameters.float_valued_meta_data[1] , 0.001);

  ROS_WARN(">>>>>>>>>>>>>>>>>>>>>>>>>>>RESET: In range test: GREEN: Algo failed and NOT able to stop");
  req = cav_srvs::PlanManeuversRequest();
  req.veh_x = 1.85;
  req.veh_y = 275; // In approach range
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 11.176;
  req.veh_lane_id = "1200";

  lcip.planManeuverCb(req, resp);

  ASSERT_EQ(2, resp.new_plan.maneuvers.size());
  ASSERT_EQ(cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(req.veh_y, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_dist);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_time.toSec(), 0.00001);
  ASSERT_NEAR(req.veh_logitudinal_velocity, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.start_speed, 0.00001);
  ASSERT_NEAR(2.7412, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.end_time.toSec(), 0.001);
  ASSERT_NEAR(295, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.end_dist, 0.01);
  // trajectory smoothing part
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[0], 0.001);
  ASSERT_NEAR(-1.5, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[1], 0.001);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[2], 0.001);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[3], 0.001);
  ASSERT_NEAR(25, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[4], 0.001);
  ASSERT_NEAR(-1, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[5], 0.001);
  ASSERT_NEAR(11.176, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.float_valued_meta_data[6], 0.001);
  ASSERT_EQ(2, resp.new_plan.maneuvers[0].intersection_transit_straight_maneuver.parameters.int_valued_meta_data[0]);
  
}
}  // namespace lci_strategic_plugin