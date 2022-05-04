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
#include "sci_strategic_plugin.h"
#include "sci_strategic_plugin_config.h"
#include <carma_wm/CARMAWorldModel.h>
#include <carma_wm/WMTestLibForGuidance.h>


// Unit tests for strategic plugin helper methods
using namespace sci_strategic_plugin;

TEST(SCIStrategicPluginTest, getDiscoveryMsg)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm;
  SCIStrategicPluginConfig config;
  config.strategic_plugin_name = "test name";
  SCIStrategicPlugin sci(wm, config);

  auto msg = sci.getDiscoveryMsg();
  ASSERT_TRUE(msg.name.compare("test name") == 0);
  ASSERT_TRUE(msg.version_id.compare("v1.0") == 0);
  ASSERT_TRUE(msg.available);
  ASSERT_TRUE(msg.activated);
  ASSERT_EQ(msg.type, cav_msgs::Plugin::STRATEGIC);
  ASSERT_TRUE(msg.capability.compare("strategic_plan/plan_maneuvers") == 0);
}


TEST(SCIStrategicPluginTest, composeLaneFollowingManeuverMessage)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm;
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);

  auto result =
      sci.composeLaneFollowingManeuverMessage(1, 10.2, 20.4, 5, 10, ros::Time(1.2), 1.0, { 1200, 1201 });

  ASSERT_EQ(cav_msgs::Maneuver::LANE_FOLLOWING, result.type);
  ASSERT_EQ(cav_msgs::ManeuverParameters::NO_NEGOTIATION, result.lane_following_maneuver.parameters.negotiation_type);
  ASSERT_EQ(cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN | cav_msgs::ManeuverParameters::HAS_INT_META_DATA | 
  cav_msgs::ManeuverParameters::HAS_FLOAT_META_DATA | cav_msgs::ManeuverParameters::HAS_STRING_META_DATA,
            result.lane_following_maneuver.parameters.presence_vector);
  ASSERT_TRUE(config.lane_following_plugin_name.compare(
                  result.lane_following_maneuver.parameters.planning_tactical_plugin) == 0);
  ASSERT_TRUE(
      config.strategic_plugin_name.compare(result.lane_following_maneuver.parameters.planning_strategic_plugin) == 0);

  ASSERT_EQ(10.2, result.lane_following_maneuver.start_dist);
  ASSERT_EQ(20.4, result.lane_following_maneuver.end_dist);
  ASSERT_EQ(5, result.lane_following_maneuver.start_speed);
  ASSERT_EQ(10, result.lane_following_maneuver.end_speed);
  ASSERT_EQ(ros::Time(1.2), result.lane_following_maneuver.start_time);
  ASSERT_EQ(ros::Time(1.2) + ros::Duration(1.0), result.lane_following_maneuver.end_time);
  ASSERT_EQ(2, result.lane_following_maneuver.lane_ids.size());
  ASSERT_TRUE(result.lane_following_maneuver.lane_ids[0].compare("1200") == 0);
  ASSERT_TRUE(result.lane_following_maneuver.lane_ids[1].compare("1201") == 0);
  ASSERT_TRUE(result.lane_following_maneuver.parameters.int_valued_meta_data[0] == 1);
}

TEST(SCIStrategicPluginTest, composeIntersectionTransitMessage)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm;
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);
  TurnDirection intersection_turn_direction = TurnDirection::Straight;

  auto result = sci.composeIntersectionTransitMessage(10.2, 20.4, 5, 10, ros::Time(1.2), ros::Time(2.2), intersection_turn_direction, 1200, 1201);

  ASSERT_EQ(cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT, result.type);
  ASSERT_EQ(cav_msgs::ManeuverParameters::NO_NEGOTIATION,
            result.intersection_transit_straight_maneuver.parameters.negotiation_type);
  ASSERT_EQ(cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN,
            result.intersection_transit_straight_maneuver.parameters.presence_vector);
  ASSERT_TRUE(config.intersection_transit_plugin_name.compare(
                  result.intersection_transit_straight_maneuver.parameters.planning_tactical_plugin) == 0);
  ASSERT_TRUE(config.strategic_plugin_name.compare(
                  result.intersection_transit_straight_maneuver.parameters.planning_strategic_plugin) == 0);

  ASSERT_EQ(10.2, result.intersection_transit_straight_maneuver.start_dist);
  ASSERT_EQ(20.4, result.intersection_transit_straight_maneuver.end_dist);
  ASSERT_EQ(5, result.intersection_transit_straight_maneuver.start_speed);
  ASSERT_EQ(10, result.intersection_transit_straight_maneuver.end_speed);
  ASSERT_EQ(ros::Time(1.2), result.intersection_transit_straight_maneuver.start_time);
  ASSERT_EQ(ros::Time(2.2), result.intersection_transit_straight_maneuver.end_time);
  ASSERT_TRUE(result.intersection_transit_straight_maneuver.starting_lane_id.compare("1200") == 0);
  ASSERT_TRUE(result.intersection_transit_straight_maneuver.ending_lane_id.compare("1201") == 0);
}

TEST(SCIStrategicPluginTest, composeStopAndWaitManeuverMessage)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm;
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);

  auto result = sci.composeStopAndWaitManeuverMessage(10.2, 20.4, 5, 1200, 1201, 0.56, ros::Time(1.2), ros::Time(2.2));

  ASSERT_EQ(cav_msgs::Maneuver::STOP_AND_WAIT, result.type);
  ASSERT_EQ(cav_msgs::ManeuverParameters::NO_NEGOTIATION, result.stop_and_wait_maneuver.parameters.negotiation_type);
  ASSERT_EQ(cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN | cav_msgs::ManeuverParameters::HAS_FLOAT_META_DATA,
            result.stop_and_wait_maneuver.parameters.presence_vector);
  ASSERT_TRUE(config.stop_and_wait_plugin_name.compare(
                  result.stop_and_wait_maneuver.parameters.planning_tactical_plugin) == 0);
  ASSERT_TRUE(
      config.strategic_plugin_name.compare(result.stop_and_wait_maneuver.parameters.planning_strategic_plugin) == 0);

  ASSERT_EQ(10.2, result.stop_and_wait_maneuver.start_dist);
  ASSERT_EQ(20.4, result.stop_and_wait_maneuver.end_dist);
  ASSERT_EQ(5, result.stop_and_wait_maneuver.start_speed);
  ASSERT_EQ(ros::Time(1.2), result.stop_and_wait_maneuver.start_time);
  ASSERT_EQ(ros::Time(2.2), result.stop_and_wait_maneuver.end_time);
  ASSERT_EQ(0.56, result.stop_and_wait_maneuver.parameters.float_valued_meta_data[1]);
  ASSERT_TRUE(result.stop_and_wait_maneuver.starting_lane_id.compare("1200") == 0);
  ASSERT_TRUE(result.stop_and_wait_maneuver.ending_lane_id.compare("1201") == 0);
}



TEST(SCIStrategicPluginTest, findSpeedLimit)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm;
  carma_wm::test::MapOptions options;
  options.lane_length_ = 25;
  options.lane_width_ = 3.7;
  options.speed_limit_ = carma_wm::test::MapOptions::SpeedLimit::DEFAULT;

  wm = carma_wm::test::getGuidanceTestMap(options);
  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);

  auto ll_iterator = wm->getMap()->laneletLayer.find(1200);
  if (ll_iterator == wm->getMap()->laneletLayer.end())
  FAIL() << "Expected lanelet not present in map. Unit test may not be structured correctly";
  
  ASSERT_NEAR(11.176, sci.findSpeedLimit(*ll_iterator), 0.00001);
}

TEST(SCIStrategicPluginTest, moboperationcbtest)
{
  cav_msgs::MobilityOperation msg;
  msg.strategy = "Carma/stop_controlled_intersection";

  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);

  ASSERT_EQ(sci.approaching_stop_controlled_interction_, false);
  auto msg_ptr = boost::make_shared<const cav_msgs::MobilityOperation>(msg);
  sci.mobilityOperationCb(msg_ptr);

  ASSERT_EQ(sci.approaching_stop_controlled_interction_, true);

}

TEST(SCIStrategicPluginTest, parseStrategyParamstest)
{
  
  cav_msgs::MobilityOperation msg;
  msg.strategy_params =  "st:16000,et:32000,dt:48000,dp:1,access:0";

  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);

  sci.parseStrategyParams(msg.strategy_params);

  
  EXPECT_EQ(16000, sci.scheduled_stop_time_);
  EXPECT_EQ(32000, sci.scheduled_enter_time_);
  EXPECT_EQ(48000, sci.scheduled_depart_time_);
  EXPECT_EQ(1, sci.scheduled_departure_position_);
  EXPECT_EQ(false, sci.is_allowed_int_);

  cav_msgs::MobilityOperation outgoing_msg = sci.generateMobilityOperation();
  EXPECT_EQ(outgoing_msg.strategy, "Carma/stop_controlled_intersection");
  EXPECT_EQ(outgoing_msg.m_header.sender_id, config.vehicle_id);
  std::cout << "strategy_param: " << outgoing_msg.strategy_params << std::endl;
}

TEST(SCIStrategicPluginTest, calcEstimatedStopTimetest)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);

  double stop_time = sci.calcEstimatedStopTime(25, 13);

  EXPECT_NEAR(3.84, stop_time, 0.01);
}

TEST(SCIStrategicPluginTest, calc_speed_before_deceltest)
{

  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);

  double stop_speed = sci.calc_speed_before_decel(20, 250, 10);

  EXPECT_NEAR(21.5, stop_speed, 0.2);
}

TEST(SCIStrategicPluginTest, determine_speed_profile_casetest)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);

  int case_num1 = sci.determine_speed_profile_case(50, 15, 40, 10);

  EXPECT_EQ(3, case_num1);

  int case_num2 = sci.determine_speed_profile_case(100, 13, 11, 10);

  EXPECT_EQ(2, case_num2);

  int case_num3 = sci.determine_speed_profile_case(100, 13, 11, 20);

  EXPECT_EQ(1, case_num3);

}

TEST(SCIStrategicPluginTest, caseOneSpeedProfiletest)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);

  std::vector<double> metadata{};

  sci.caseOneSpeedProfile(17, 12, 44, &metadata);

  EXPECT_NEAR(0.5, metadata[0], 0.01);
  EXPECT_NEAR(-0.5, metadata[1], 0.01);
  EXPECT_NEAR(10, metadata[2], 0.01);
  EXPECT_NEAR(34, metadata[3], 0.01);
}

TEST(SCIStrategicPluginTest, caseTwoSpeedProfiletest)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);

  std::vector<double> metadata{};

  sci.caseTwoSpeedProfile(250, 21.2, 10, 20, 15, &metadata);

  EXPECT_NEAR(1, metadata[0], 0.01);
  EXPECT_NEAR(1, metadata[1], 0.01);
  EXPECT_NEAR(5, metadata[2], 0.01);
  EXPECT_NEAR(-15, metadata[3], 0.01);
  EXPECT_NEAR(12, metadata[4], 0.01);
}

TEST(SCIStrategicPluginTest, caseThreeSpeedProfiletest)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);

  double dec_val = sci.caseThreeSpeedProfile(50, 5, 30);

  EXPECT_NEAR(-1.83, dec_val, 0.01);
}

TEST(SCIStrategicPluginTest, testIntersectionturndirection)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(wm, config);

  double dec_val = sci.caseThreeSpeedProfile(50, 5, 30);

  EXPECT_NEAR(-1.83, dec_val, 0.01);
}

// The map in this unit test does not support turn direction and therefore it is disabled. 
// The test can be run if the turn direction detection logic (lines 461-467) is commented.

TEST(SCIStrategicPluginTest, DISABLE_maneuvercbtest)
{
  lanelet::Id id{1200};
  // intersection id
  lanelet::Id int_id{1};
  lanelet::Point3d p1, p2, p3, p4, p5, p6;
  lanelet::LineString3d ls1, ls2, ls3, ls4, ls5, ls6;
  lanelet::Lanelet ll1, ll2, ll3, ll4;

  p1 = lanelet::Point3d(++id, 0., 10., 0.);
  p2 = lanelet::Point3d(++id, 10., 10., 0.);
  p3 = lanelet::Point3d(++id, 10., 10., 0.);
  p4 = lanelet::Point3d(++id, 60., 10., 0.);
  p5 = lanelet::Point3d(++id, 60., 60., 0.);
  p6 = lanelet::Point3d(++id, 60., 60., 0.);

  ls1 = lanelet::LineString3d(++id, lanelet::Points3d{p1, p2});
  ls2 = lanelet::LineString3d(++id, lanelet::Points3d{p2, p3});
  ls3 = lanelet::LineString3d(++id, lanelet::Points3d{p3, p4});
  ls4 = lanelet::LineString3d(++id, lanelet::Points3d{p4, p5});
  ls5 = lanelet::LineString3d(++id, lanelet::Points3d{p5, p6});


  ll1 = lanelet::Lanelet(++id, ls1, ls2);
  std::cout << "ll1.id()  " << ll1.id() << std::endl;
  ll2 = lanelet::Lanelet(++id, ls2, ls3);
  std::cout << "ll2.id()  " << ll2.id() << std::endl;
  ll3 = lanelet::Lanelet(++id, ls3, ls4);
  std::cout << "ll3.id()  " << ll3.id() << std::endl;
  ll4 = lanelet::Lanelet(++id, ls4, ls5);

  carma_wm::CARMAWorldModel cmw;
  lanelet::LaneletMapPtr map;
  // Create a complete map
  carma_wm::test::MapOptions mp(1,1);
  auto cmw_ptr = carma_wm::test::getGuidanceTestMap(mp);

  std::shared_ptr<lanelet::AllWayStop> row = lanelet::AllWayStop::make(int_id, lanelet::AttributeMap(), {{ll1, ls1}, {ll3, ls4}});
  cmw_ptr->getMutableMap()->update(cmw_ptr->getMutableMap()->laneletLayer.get(1200), row);

  carma_wm::test::setRouteByIds({1200, 1201, 1202, 1203}, cmw_ptr);


  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  SCIStrategicPluginConfig config;
  SCIStrategicPlugin sci(cmw_ptr, config);

  sci.current_downtrack_ = 1.0;
  // pose callback test
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.pose.position.x = 1.0;
  pose_msg.pose.position.y = 1.0;
  auto msg = boost::make_shared<const geometry_msgs::PoseStamped>(pose_msg);
  sci.currentPoseCb(msg);
  ASSERT_NEAR(1.0, sci.current_downtrack_, 0.1);


  sci.approaching_stop_controlled_interction_ = true;
  sci.street_msg_timestamp_ = 2000;
  sci.scheduled_stop_time_ = 2500;
  sci.scheduled_enter_time_ = 5000;
  sci.scheduled_depart_time_ = 7000;
  

  cav_srvs::PlanManeuversRequest req;
  cav_srvs::PlanManeuversResponse resp;

  // approaching intersection
  req = cav_srvs::PlanManeuversRequest();
  req.veh_x = 1.85;
  req.veh_y = 1.0; 
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 11.176;
  req.veh_lane_id = "1200";

  

  sci.planManeuverCb(req, resp);

  ASSERT_EQ(1, resp.new_plan.maneuvers.size());
  ASSERT_EQ(resp.new_plan.maneuvers[0].lane_following_maneuver.lane_ids[0], "1200");
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].lane_following_maneuver.end_speed, 0.00001);
  // case 3
  ASSERT_EQ(2, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.int_valued_meta_data[0]);

  // at the stop line
  cav_srvs::PlanManeuversRequest req1;
  cav_srvs::PlanManeuversResponse resp1;

  sci.current_downtrack_ = 9;
  req1 = cav_srvs::PlanManeuversRequest();
  req1.veh_x = 9.85;
  req1.veh_y = 2.0; 
  req1.veh_downtrack = req.veh_y;
  req1.veh_logitudinal_velocity = 0.0;
  req1.veh_lane_id = "1209";

  sci.scheduled_enter_time_ = 7000;

  sci.planManeuverCb(req1, resp1);
  ASSERT_EQ(1, resp1.new_plan.maneuvers.size());
  ASSERT_EQ(resp1.new_plan.maneuvers[0].stop_and_wait_maneuver.starting_lane_id, "1212");
  ASSERT_EQ(resp1.new_plan.maneuvers[0].stop_and_wait_maneuver.ending_lane_id, "1212");


}
