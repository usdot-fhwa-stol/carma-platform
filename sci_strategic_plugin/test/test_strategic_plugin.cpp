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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "sci_strategic_plugin.hpp"
#include "sci_strategic_plugin_config.hpp"
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>


// Unit tests for strategic plugin helper methods
namespace sci_strategic_plugin
{

TEST(SCIStrategicPluginTest, composeLaneFollowingManeuverMessage)
{
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  SCIStrategicPluginConfig config;
  auto result =
      sci_node->composeLaneFollowingManeuverMessage(1, 10.2, 20.4, 5, 10, rclcpp::Time(1.2*1e9), 1.0, { 1200, 1201 });

  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING, result.type);
  ASSERT_EQ(carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION, result.lane_following_maneuver.parameters.negotiation_type);
  ASSERT_EQ(carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN | carma_planning_msgs::msg::ManeuverParameters::HAS_INT_META_DATA | 
  carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA | carma_planning_msgs::msg::ManeuverParameters::HAS_STRING_META_DATA,
            result.lane_following_maneuver.parameters.presence_vector);
  ASSERT_TRUE(config.lane_following_plugin_name.compare(
                  result.lane_following_maneuver.parameters.planning_tactical_plugin) == 0);
  ASSERT_TRUE(
      config.strategic_plugin_name.compare(result.lane_following_maneuver.parameters.planning_strategic_plugin) == 0);

  ASSERT_EQ(10.2, result.lane_following_maneuver.start_dist);
  ASSERT_EQ(20.4, result.lane_following_maneuver.end_dist);
  ASSERT_EQ(5, result.lane_following_maneuver.start_speed);
  ASSERT_EQ(10, result.lane_following_maneuver.end_speed);
  ASSERT_EQ(rclcpp::Time(1.2*1e9, RCL_ROS_TIME), result.lane_following_maneuver.start_time);
  ASSERT_EQ(rclcpp::Time(1.2*1e9, RCL_ROS_TIME) + rclcpp::Duration(1.0*1e9), result.lane_following_maneuver.end_time);
  ASSERT_EQ(2, result.lane_following_maneuver.lane_ids.size());
  ASSERT_TRUE(result.lane_following_maneuver.lane_ids[0].compare("1200") == 0);
  ASSERT_TRUE(result.lane_following_maneuver.lane_ids[1].compare("1201") == 0);
  ASSERT_TRUE(result.lane_following_maneuver.parameters.int_valued_meta_data[0] == 1);
}

TEST(SCIStrategicPluginTest, composeIntersectionTransitMessage)
{
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  SCIStrategicPluginConfig config;
  TurnDirection intersection_turn_direction = TurnDirection::Straight;

  auto result = sci_node->composeIntersectionTransitMessage(10.2, 20.4, 5, 10, rclcpp::Time(1.2*1e9), rclcpp::Time(2.2*1e9), intersection_turn_direction, 1200, 1201);

  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT, result.type);
  ASSERT_EQ(carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION,
            result.intersection_transit_straight_maneuver.parameters.negotiation_type);
  ASSERT_EQ(carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN,
            result.intersection_transit_straight_maneuver.parameters.presence_vector);
  ASSERT_TRUE(config.intersection_transit_plugin_name.compare(
                  result.intersection_transit_straight_maneuver.parameters.planning_tactical_plugin) == 0);
  ASSERT_TRUE(config.strategic_plugin_name.compare(
                  result.intersection_transit_straight_maneuver.parameters.planning_strategic_plugin) == 0);

  ASSERT_EQ(10.2, result.intersection_transit_straight_maneuver.start_dist);
  ASSERT_EQ(20.4, result.intersection_transit_straight_maneuver.end_dist);
  ASSERT_EQ(5, result.intersection_transit_straight_maneuver.start_speed);
  ASSERT_EQ(10, result.intersection_transit_straight_maneuver.end_speed);
  ASSERT_EQ(rclcpp::Time(1.2*1e9, RCL_ROS_TIME), result.intersection_transit_straight_maneuver.start_time);
  ASSERT_EQ(rclcpp::Time(2.2*1e9, RCL_ROS_TIME), result.intersection_transit_straight_maneuver.end_time);
  ASSERT_TRUE(result.intersection_transit_straight_maneuver.starting_lane_id.compare("1200") == 0);
  ASSERT_TRUE(result.intersection_transit_straight_maneuver.ending_lane_id.compare("1201") == 0);
}

TEST(SCIStrategicPluginTest, composeStopAndWaitManeuverMessage)
{
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  SCIStrategicPluginConfig config;
  auto result = sci_node->composeStopAndWaitManeuverMessage(10.2, 20.4, 5, 1200, 1201, 0.56, rclcpp::Time(1.2*1e9), rclcpp::Time(2.2*1e9));

  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT, result.type);
  ASSERT_EQ(carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION, result.stop_and_wait_maneuver.parameters.negotiation_type);
  ASSERT_EQ(carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN | carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA,
            result.stop_and_wait_maneuver.parameters.presence_vector);
  ASSERT_TRUE(config.stop_and_wait_plugin_name.compare(
                  result.stop_and_wait_maneuver.parameters.planning_tactical_plugin) == 0);
  ASSERT_TRUE(
      config.strategic_plugin_name.compare(result.stop_and_wait_maneuver.parameters.planning_strategic_plugin) == 0);

  ASSERT_EQ(10.2, result.stop_and_wait_maneuver.start_dist);
  ASSERT_EQ(20.4, result.stop_and_wait_maneuver.end_dist);
  ASSERT_EQ(5, result.stop_and_wait_maneuver.start_speed);
  ASSERT_EQ(rclcpp::Time(1.2*1e9, RCL_ROS_TIME), result.stop_and_wait_maneuver.start_time);
  ASSERT_EQ(rclcpp::Time(2.2*1e9, RCL_ROS_TIME), result.stop_and_wait_maneuver.end_time);
  ASSERT_EQ(0.56, result.stop_and_wait_maneuver.parameters.float_valued_meta_data[1]);
  ASSERT_TRUE(result.stop_and_wait_maneuver.starting_lane_id.compare("1200") == 0);
  ASSERT_TRUE(result.stop_and_wait_maneuver.ending_lane_id.compare("1201") == 0);
}

TEST(SCIStrategicPluginTest, findSpeedLimit)
{
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  
  std::shared_ptr<carma_wm::CARMAWorldModel> wm;
  carma_wm::test::MapOptions options;
  options.lane_length_ = 25;
  options.lane_width_ = 3.7;
  options.speed_limit_ = carma_wm::test::MapOptions::SpeedLimit::DEFAULT;

  wm = carma_wm::test::getGuidanceTestMap(options);
  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

  sci_node->set_wm(wm);

  auto ll_iterator = wm->getMap()->laneletLayer.find(1200);
  if (ll_iterator == wm->getMap()->laneletLayer.end())
  FAIL() << "Expected lanelet not present in map. Unit test may not be structured correctly";
  
  ASSERT_NEAR(11.176, sci_node->findSpeedLimit(*ll_iterator), 0.00001);
}

TEST(SCIStrategicPluginTest, moboperationcbtest)
{
  carma_v2x_msgs::msg::MobilityOperation msg;
  msg.strategy = "Carma/stop_controlled_intersection";

  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  sci_node->set_wm(wm);
  ASSERT_EQ(sci_node->approaching_stop_controlled_interction_, false);
  auto msg_ptr = std::make_unique<carma_v2x_msgs::msg::MobilityOperation>(msg);
  sci_node->mobilityOperationCb(std::move(msg_ptr));

  ASSERT_EQ(sci_node->approaching_stop_controlled_interction_, true);

}

TEST(SCIStrategicPluginTest, parseStrategyParamstest)
{
  
  carma_v2x_msgs::msg::MobilityOperation msg;
  msg.strategy_params =  "st:16000,et:32000,dt:48000,dp:1,access:0";

  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  SCIStrategicPluginConfig config;
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  sci_node->set_wm(wm);

  sci_node->parseStrategyParams(msg.strategy_params);

  
  EXPECT_EQ(16000, sci_node->scheduled_stop_time_);
  EXPECT_EQ(32000, sci_node->scheduled_enter_time_);
  EXPECT_EQ(48000, sci_node->scheduled_depart_time_);
  EXPECT_EQ(1, sci_node->scheduled_departure_position_);
  EXPECT_EQ(false, sci_node->is_allowed_int_);

  carma_v2x_msgs::msg::MobilityOperation outgoing_msg = sci_node->generateMobilityOperation();
  EXPECT_EQ(outgoing_msg.strategy, "Carma/stop_controlled_intersection");
  EXPECT_EQ(outgoing_msg.m_header.sender_id, config.vehicle_id);
  std::cout << "strategy_param: " << outgoing_msg.strategy_params << std::endl;
}

TEST(SCIStrategicPluginTest, calcEstimatedStopTimetest)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  sci_node->set_wm(wm);

  double stop_time = sci_node->calcEstimatedStopTime(25, 13);

  EXPECT_NEAR(3.84, stop_time, 0.01);
}

TEST(SCIStrategicPluginTest, calc_speed_before_deceltest)
{

  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  sci_node->set_wm(wm);

  double stop_speed = sci_node->calc_speed_before_decel(20, 250, 10);

  EXPECT_NEAR(21.5, stop_speed, 0.2);
}

TEST(SCIStrategicPluginTest, determine_speed_profile_casetest)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  sci_node->set_wm(wm);

  int case_num1 = sci_node->determine_speed_profile_case(50, 15, 40, 10);

  EXPECT_EQ(3, case_num1);

  int case_num2 = sci_node->determine_speed_profile_case(100, 13, 11, 10);

  EXPECT_EQ(2, case_num2);

  int case_num3 = sci_node->determine_speed_profile_case(100, 13, 11, 20);

  EXPECT_EQ(1, case_num3);

}

TEST(SCIStrategicPluginTest, caseOneSpeedProfiletest)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  sci_node->set_wm(wm);

  std::vector<double> metadata{};

  sci_node->caseOneSpeedProfile(17, 12, 44, &metadata);

  EXPECT_NEAR(0.5, metadata[0], 0.01);
  EXPECT_NEAR(-0.5, metadata[1], 0.01);
  EXPECT_NEAR(10, metadata[2], 0.01);
  EXPECT_NEAR(34, metadata[3], 0.01);
}

TEST(SCIStrategicPluginTest, caseTwoSpeedProfiletest)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  sci_node->set_wm(wm);

  std::vector<double> metadata{};

  sci_node->caseTwoSpeedProfile(250, 21.2, 10, 20, 15, &metadata);

  EXPECT_NEAR(1, metadata[0], 0.01);
  EXPECT_NEAR(1, metadata[1], 0.01);
  EXPECT_NEAR(5, metadata[2], 0.01);
  EXPECT_NEAR(-15, metadata[3], 0.01);
  EXPECT_NEAR(12, metadata[4], 0.01);
}

TEST(SCIStrategicPluginTest, caseThreeSpeedProfiletest)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  sci_node->set_wm(wm);

  double dec_val = sci_node->caseThreeSpeedProfile(50, 5, 30);

  EXPECT_NEAR(-1.83, dec_val, 0.01);
}

TEST(SCIStrategicPluginTest, testIntersectionturndirection)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  sci_node->set_wm(wm);

  double dec_val = sci_node->caseThreeSpeedProfile(50, 5, 30);

  EXPECT_NEAR(-1.83, dec_val, 0.01);
}

// The map in this unit test does not support turn direction and therefore it is disabled. 
// The test can be run if the turn direction detection logic (lines 461-467) is commented.

TEST(SCIStrategicPluginTest, DISABLED_maneuvercbtest)
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

  auto sci_node = std::make_shared<sci_strategic_plugin::SCIStrategicPlugin>(rclcpp::NodeOptions());
  sci_node->configure();
  sci_node->activate();
  sci_node->set_wm(cmw_ptr);

  sci_node->current_downtrack_ = 1.0;
  // pose callback test
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.pose.position.x = 1.0;
  pose_msg.pose.position.y = 1.0;
  auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
  sci_node->currentPoseCb(std::move(msg));
  ASSERT_NEAR(1.0, sci_node->current_downtrack_, 0.1);

  sci_node->approaching_stop_controlled_interction_ = true;
  sci_node->street_msg_timestamp_ = 2000;
  sci_node->scheduled_stop_time_ = 2500;
  sci_node->scheduled_enter_time_ = 5000;
  sci_node->scheduled_depart_time_ = 7000;

  auto srv_header = std::make_shared<rmw_request_id_t>();
  auto req = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Request>();
  auto resp = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();

  // approaching intersection
  req->veh_x = 1.85;
  req->veh_y = 1.0; 
  req->veh_downtrack = req->veh_y;
  req->veh_logitudinal_velocity = 11.176;
  req->veh_lane_id = "1200";

  sci_node->plan_maneuvers_callback(srv_header, req, resp);

  ASSERT_EQ(1, resp->new_plan.maneuvers.size());

  ASSERT_EQ(resp->new_plan.maneuvers[0].lane_following_maneuver.lane_ids[0], "1200");

  ASSERT_NEAR(0.0, resp->new_plan.maneuvers[0].lane_following_maneuver.end_speed, 0.00001);

  // case 3
  ASSERT_EQ(2, resp->new_plan.maneuvers[0].lane_following_maneuver.parameters.int_valued_meta_data[0]);


  // at the stop line
  auto srv_header1 = std::make_shared<rmw_request_id_t>();
  auto req1 = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Request>();
  auto resp1 = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();

  sci_node->current_downtrack_ = 9;
  req1->veh_x = 9.85;
  req1->veh_y = 2.0; 
  req1->veh_downtrack = req->veh_y;
  req1->veh_logitudinal_velocity = 0.0;
  req1->veh_lane_id = "1209";

  sci_node->scheduled_enter_time_ = 7000;
  sci_node->plan_maneuvers_callback(srv_header1, req1, resp1);
  ASSERT_EQ(1, resp1->new_plan.maneuvers.size());
  ASSERT_EQ(resp1->new_plan.maneuvers[0].stop_and_wait_maneuver.starting_lane_id, "1212");
  ASSERT_EQ(resp1->new_plan.maneuvers[0].stop_and_wait_maneuver.ending_lane_id, "1212");


}



} // namespace sci_strategic_plugin