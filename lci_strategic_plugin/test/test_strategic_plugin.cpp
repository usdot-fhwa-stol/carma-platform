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
  carma_wm::test::addTrafficLight(cmw_, traffic_light_id, {1200}, { 1203 });

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
  ASSERT_EQ(cav_msgs::Maneuver::LANE_FOLLOWING, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(130.0, resp.new_plan.maneuvers[0].lane_following_maneuver.start_dist );
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].lane_following_maneuver.start_time.toSec(), 0.01);
  ASSERT_NEAR(req.veh_logitudinal_velocity, resp.new_plan.maneuvers[0].lane_following_maneuver.start_speed, 0.01);
  ASSERT_NEAR(24.01, resp.new_plan.maneuvers[0].lane_following_maneuver.end_time.toSec(), 0.01);
  ASSERT_NEAR(300, resp.new_plan.maneuvers[0].lane_following_maneuver.end_dist, 0.0001);
  // check trajectory smoothing parameters:
  ASSERT_EQ("signalized", resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.string_valued_meta_data.front());
  ASSERT_NEAR(0.6823, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[0], 0.01);
  ASSERT_NEAR(-0.6823, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[1], 0.01);
  ASSERT_NEAR(85.00, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[2], 0.01);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[3], 0.01);
  ASSERT_NEAR(85.00, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[4], 0.01);
  ASSERT_NEAR(2.98476634, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[5], 0.01);
  ASSERT_NEAR(-1, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[6], 0.001);
  ASSERT_EQ(3, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.int_valued_meta_data[0]);

  ASSERT_EQ(cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT, resp.new_plan.maneuvers[1].type);
  ASSERT_EQ(300, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.start_dist );
  ASSERT_NEAR(24.010000000000002, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.start_time.toSec(), 0.01);
  ASSERT_NEAR(11.176, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.start_speed, 0.01);
  ASSERT_NEAR(77.6964710090, resp.new_plan.maneuvers[1].intersection_transit_straight_maneuver.end_time.toSec(), 0.001);
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
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_time.toSec(), 0.01);
  ASSERT_NEAR(req.veh_logitudinal_velocity, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_speed, 0.01);
  ASSERT_NEAR(15.1, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.end_time.toSec(), 0.001);
  ASSERT_NEAR(300, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.end_dist, 0.0001);

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
  ASSERT_NEAR(6.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_time.toSec(), 0.01);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].stop_and_wait_maneuver.start_speed, 0.01);
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
  ASSERT_EQ(cav_msgs::Maneuver::LANE_FOLLOWING, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(299.0, resp.new_plan.maneuvers[0].lane_following_maneuver.start_dist );
  ASSERT_NEAR(25.0, resp.new_plan.maneuvers[0].lane_following_maneuver.start_time.toSec(), 0.01);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].lane_following_maneuver.start_speed, 0.01);
  ASSERT_NEAR(ros::Time(132.55189).toSec(), resp.new_plan.maneuvers[0].lane_following_maneuver.end_time.toSec(), 0.25);
  ASSERT_NEAR(900.0, resp.new_plan.maneuvers[0].lane_following_maneuver.end_dist, 0.0001);
  ASSERT_NEAR(11.176, resp.new_plan.maneuvers[0].lane_following_maneuver.end_speed, 0.01);

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
  ASSERT_EQ(cav_msgs::Maneuver::LANE_FOLLOWING, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(req.veh_y, resp.new_plan.maneuvers[0].lane_following_maneuver.start_dist);
  ASSERT_NEAR(15.0, resp.new_plan.maneuvers[0].lane_following_maneuver.start_time.toSec(), 0.01);
  ASSERT_NEAR(req.veh_logitudinal_velocity, resp.new_plan.maneuvers[0].lane_following_maneuver.start_speed, 0.01);
  ASSERT_NEAR(20.8446, resp.new_plan.maneuvers[0].lane_following_maneuver.end_time.toSec(), 0.001);
  ASSERT_NEAR(259.7, resp.new_plan.maneuvers[0].lane_following_maneuver.end_dist, 0.01);
  // trajectory smoothing part
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[0], 0.001);
  ASSERT_NEAR(-1.5, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[1], 0.001);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[2], 0.001);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[3], 0.001);
  ASSERT_NEAR(29.6998297, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[4], 0.001);
  ASSERT_NEAR(2.40904, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[5], 0.001);
  ASSERT_NEAR(-1, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[6], 0.001);
  ASSERT_EQ(3, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.int_valued_meta_data[0]);

  ASSERT_EQ(cav_msgs::Maneuver::STOP_AND_WAIT, resp.new_plan.maneuvers[1].type);
  ASSERT_NEAR(259.7, resp.new_plan.maneuvers[1].stop_and_wait_maneuver.start_dist, 0.1);
  ASSERT_NEAR(20.8446, resp.new_plan.maneuvers[1].stop_and_wait_maneuver.start_time.toSec(), 0.01);
  ASSERT_NEAR(2.40904, resp.new_plan.maneuvers[1].stop_and_wait_maneuver.start_speed, 0.01);
  ASSERT_NEAR(35.9446, resp.new_plan.maneuvers[1].stop_and_wait_maneuver.end_time.toSec(), 0.01);
  ASSERT_NEAR(300, resp.new_plan.maneuvers[1].stop_and_wait_maneuver.end_dist, 0.1);
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
  ASSERT_EQ(cav_msgs::Maneuver::LANE_FOLLOWING, resp.new_plan.maneuvers[0].type);
  ASSERT_EQ(req.veh_y, resp.new_plan.maneuvers[0].lane_following_maneuver.start_dist);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].lane_following_maneuver.start_time.toSec(), 0.01);
  ASSERT_NEAR(req.veh_logitudinal_velocity, resp.new_plan.maneuvers[0].lane_following_maneuver.start_speed, 0.01);
  ASSERT_NEAR(2.7412, resp.new_plan.maneuvers[0].lane_following_maneuver.end_time.toSec(), 0.001);
  ASSERT_NEAR(300, resp.new_plan.maneuvers[0].lane_following_maneuver.end_dist, 0.01);
  // trajectory smoothing part
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[0], 0.001);
  ASSERT_NEAR(-1.5, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[1], 0.001);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[2], 0.001);
  ASSERT_NEAR(0.0, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[3], 0.001);
  ASSERT_NEAR(25, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[4], 0.001);
  ASSERT_NEAR(-1, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[5], 0.001);
  ASSERT_NEAR(11.176, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.float_valued_meta_data[6], 0.001);
  ASSERT_EQ(2, resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.int_valued_meta_data[0]);
  
}

TEST_F(LCIStrategicTestFixture, determine_speed_profile_case)
{
  LCIStrategicPluginConfig config;
  LCIStrategicPlugin lcip(cmw_, config);

  auto profile_case = lcip.determine_speed_profile_case(10, 5, 3, 0, 5);

  EXPECT_EQ(profile_case,SpeedProfileCase::ACCEL_DECEL);

  profile_case = lcip.determine_speed_profile_case(10, 5, 6, 0, 5);
 
  EXPECT_EQ(profile_case,SpeedProfileCase::ACCEL_CRUISE_DECEL);

  profile_case = lcip.determine_speed_profile_case(5, 10, 0, 1, 5);

  EXPECT_EQ(profile_case,SpeedProfileCase::DECEL_CRUISE_ACCEL);

  profile_case = lcip.determine_speed_profile_case(5, 10, 0, 3, 5);

  EXPECT_EQ(profile_case,SpeedProfileCase::DECEL_ACCEL);

  TrajectorySmoothingParameters params;
  params.a_accel = 999;
  params.a_decel = 999;
  params.dist_accel = 999;
  params.dist_cruise = 999;
  params.dist_decel = 999;
  params.speed_before_accel = 999;
  

  params.is_algorithm_successful = false;
  params.case_num = SpeedProfileCase::ACCEL_DECEL;
  auto mvr = lcip.composeTrajectorySmoothingManeuverMessage(0, 0, 0, 0,ros::Time(0), ros::Time(0), params);

  bool is_successful = mvr.lane_following_maneuver.parameters.int_valued_meta_data[1];
  SpeedProfileCase new_case = static_cast<SpeedProfileCase>(mvr.lane_following_maneuver.parameters.int_valued_meta_data[0]);

  ASSERT_EQ(new_case, SpeedProfileCase::ACCEL_DECEL);
  ASSERT_TRUE(false);
} 

TEST_F(LCIStrategicTestFixture, inflection_speeds_calc)
{
  LCIStrategicPluginConfig config;
  LCIStrategicPlugin lcip(cmw_, config);

  auto speed = lcip.calc_speed_before_decel(2, 2, 5, 5);

  EXPECT_NEAR(speed, 5, 0.001);

  speed = lcip.calc_speed_before_accel(2, 2, 5, 5);

  EXPECT_NEAR(speed, -3, 0.001);

  speed = lcip.get_inflection_speed_value(5, 4, 2, 10, 2, 4, 1, -1);

  EXPECT_NEAR(speed, 10, 0.001);

  speed = lcip.get_inflection_speed_value(3, 4, 2, 10, 2, 4, 1, -1);

  EXPECT_NEAR(speed, 3.1622, 0.001);

  speed = lcip.get_inflection_speed_value(1, 4, 2, 10, 2, 4, 1, -1);

  EXPECT_NEAR(speed, 2.449, 0.001);

  speed = lcip.get_inflection_speed_value(1, 4, 2, 10, 5, 4, 1, -1);

  EXPECT_NEAR(speed,  4.796, 0.001);
}

TEST_F(LCIStrategicTestFixture, get_nearest_green_entry_time)
{
  LCIStrategicPluginConfig config;
  LCIStrategicPlugin lcip(cmw_, config);

  // Light will be located on lanelet 1200 (300m) and control lanelet 1202, 1203
  lanelet::Id traffic_light_id = lanelet::utils::getId();
  carma_wm::test::addTrafficLight(cmw_, traffic_light_id, {1200}, { 1203 });

  auto signal = cmw_->getMutableMap()->laneletLayer.get(1200).regulatoryElementsAs<lanelet::CarmaTrafficSignal>().front();
  auto time = lcip.get_nearest_green_entry_time(ros::Time(10), ros::Time(15), signal, 0);

  EXPECT_EQ(ros::Time(24), time);

  time = lcip.get_nearest_green_entry_time(ros::Time(10), ros::Time(28), signal, 0);

  EXPECT_EQ(ros::Time(28), time);

  time = lcip.get_nearest_green_entry_time(ros::Time(10), ros::Time(45), signal, 0);

  EXPECT_EQ(ros::Time(68), time);

  time = lcip.get_nearest_green_entry_time(ros::Time(10), ros::Time(44), signal, 50);

  EXPECT_EQ(ros::Time(122), time);
}


TEST_F(LCIStrategicTestFixture, handleFailureCase)
{
  LCIStrategicPluginConfig config;
  config.vehicle_accel_limit = 1;
  config.vehicle_accel_limit_multiplier = 1;
  config.vehicle_decel_limit_multiplier = 1;
  config.vehicle_decel_limit= 1;


  LCIStrategicPlugin lcip(cmw_, config);

  auto params = lcip.handleFailureCase(5, 10, 12, 0);
  
  EXPECT_NEAR(params.a_accel, 1, 0.001);
  EXPECT_NEAR(params.a_decel, 0, 0.001);
  EXPECT_NEAR(params.speed_before_accel, 5, 0.001);
  EXPECT_NEAR(params.speed_before_decel, -1, 0.001);
  EXPECT_NEAR(params.dist_accel, 12, 0.001);
  EXPECT_NEAR(params.dist_cruise, 0, 0.001);
  EXPECT_NEAR(params.dist_decel, 0, 0.001);
  EXPECT_NEAR(params.modified_departure_speed, 7, 0.01);
  EXPECT_NEAR(params.modified_remaining_time, 2, 0.01);
  EXPECT_EQ(params.case_num, SpeedProfileCase::DECEL_ACCEL);

  params = lcip.handleFailureCase(5, 0, 8, 0);

  EXPECT_NEAR(params.a_accel, 0, 0.001);
  EXPECT_NEAR(params.a_decel, -1, 0.001);
  EXPECT_NEAR(params.speed_before_accel, -1, 0.001);
  EXPECT_NEAR(params.speed_before_decel, 5, 0.001);
  EXPECT_NEAR(params.dist_accel, 0, 0.001);
  EXPECT_NEAR(params.dist_cruise, 0, 0.001);
  EXPECT_NEAR(params.dist_decel, 8, 0.001);
  EXPECT_NEAR(params.modified_departure_speed, 3, 0.01);
  EXPECT_NEAR(params.modified_remaining_time, 2, 0.01);
  EXPECT_EQ(params.case_num, SpeedProfileCase::ACCEL_DECEL);

  EXPECT_THROW(lcip.handleFailureCase(5 ,0, 15, 0), std::invalid_argument);

} 

TEST(LCIStrategicPluginTest, moboperationcbtest)
{
  cav_msgs::MobilityOperation msg;
  msg.strategy = "signalized";

  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  LCIStrategicPluginConfig config;
  LCIStrategicPlugin lcip(wm, config);

  ASSERT_EQ(lcip.approaching_light_controlled_interction_, false);
  auto msg_ptr = boost::make_shared<const cav_msgs::MobilityOperation>(msg);
  lcip.mobilityOperationCb(msg_ptr);

  ASSERT_EQ(lcip.approaching_light_controlled_interction_, true);
}

TEST(LCIStrategicPluginTest, parseStrategyParamstest)
{
  cav_msgs::MobilityOperation msg;
  msg.strategy_params =  "et:32000";

  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  LCIStrategicPluginConfig config;
  LCIStrategicPlugin lcip(wm, config);

  lcip.parseStrategyParams(msg.strategy_params);

  
  EXPECT_EQ(16000, lcip.scheduled_stop_time_);
  EXPECT_EQ(32000, lcip.scheduled_enter_time_);

  cav_msgs::MobilityOperation outgoing_msg = lcip.generateMobilityOperation();
  EXPECT_EQ(outgoing_msg.strategy, "signalized");
  EXPECT_EQ(outgoing_msg.m_header.sender_id, config.vehicle_id);
  std::cout << "strategy_param: " << outgoing_msg.strategy_params << std::endl;
}

}  // namespace lci_strategic_plugin