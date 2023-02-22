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


/*
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
  ASSERT_EQ("Carma/signalized_intersection", resp.new_plan.maneuvers[0].lane_following_maneuver.parameters.string_valued_meta_data.front());
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

TEST_F(LCIStrategicTestFixture, handleFailureCaseHelper)
{
  carma_wm::test::setSpeedLimit(25_mph, cmw_);

  // Light will be located on lanelet 1200 (300m) and control lanelet 1202, 1203
  lanelet::Id traffic_light_id = lanelet::utils::getId();
  carma_wm::test::addTrafficLight(cmw_, traffic_light_id, {1200}, { 1203 });

  LCIStrategicPluginConfig config;
  config.vehicle_accel_limit = 2;
  config.vehicle_accel_limit_multiplier = 1;
  config.vehicle_decel_limit_multiplier = 1;
  config.vehicle_decel_limit= 2;
  config.green_light_time_buffer = 1.0;
  ros::Time::setNow(ros::Time(0));
  LCIStrategicPlugin lcip(cmw_, config);

  auto signal = cmw_->getMutableMap()->laneletLayer.get(1200).regulatoryElementsAs<lanelet::CarmaTrafficSignal>().front();
  
  ////////// CASE 1 ////////////////
  // Traj upper 1

  double green_start_time = 1.0;
  double green_end_time = 5.0;
  double remaining_distance = 12.0;
  double current_time = 0.0;

  signal->fixed_cycle_duration = lanelet::time::durationFromSec(0.0); //dynamic
  signal->recorded_time_stamps = {};
  signal->recorded_start_time_stamps = {};
  signal->recorded_time_stamps.push_back(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(boost::posix_time::from_time_t(green_start_time), lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN));
  signal->recorded_start_time_stamps.push_back(boost::posix_time::from_time_t(0.0));
  signal->recorded_time_stamps.push_back(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(boost::posix_time::from_time_t(green_end_time), lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED));
  signal->recorded_start_time_stamps.push_back(boost::posix_time::from_time_t(green_start_time));
  
  TrajectoryParams params;

  params = lcip.handleFailureCaseHelper(signal, 
                                            current_time, 
                                            8.0, 
                                            11.0, 
                                            11.176, 
                                            remaining_distance, 
                                            remaining_distance);

  EXPECT_NEAR(params.t0_, 0.0, 0.01);
  EXPECT_NEAR(params.v0_, 8, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, 1.291503, 0.01);
  EXPECT_NEAR(params.v1_, 10.583, 0.01);
  EXPECT_NEAR(params.x1_, 12, 0.01);
  EXPECT_NEAR(params.a1_, 2, 0.01);
  EXPECT_NEAR(params.t2_, 1.291, 0.01);
  EXPECT_NEAR(params.v2_, 10.583, 0.01);
  EXPECT_NEAR(params.x2_, 12, 0.01);
  EXPECT_NEAR(params.a2_, 2, 0.01);
  EXPECT_NEAR(params.t3_, 1.291, 0.01);
  EXPECT_NEAR(params.v3_, 10.583, 0.01);
  EXPECT_NEAR(params.x3_, 12, 0.01);
  EXPECT_NEAR(params.a3_, 2, 0.01);

    // Traj upper 2
  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = 3.0;

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        8.0, 
                                        11.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, current_time, 0.01);
  EXPECT_NEAR(params.v0_, 8, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, current_time + 1.291503, 0.01);
  EXPECT_NEAR(params.v1_, 10.583, 0.01);
  EXPECT_NEAR(params.x1_, 12, 0.01);
  EXPECT_NEAR(params.a1_, 2, 0.01);
  EXPECT_NEAR(params.t2_, current_time + 1.291, 0.01);
  EXPECT_NEAR(params.v2_, 10.583, 0.01);
  EXPECT_NEAR(params.x2_, 12, 0.01);
  EXPECT_NEAR(params.a2_, 2, 0.01);
  EXPECT_NEAR(params.t3_, current_time + 1.291, 0.01);
  EXPECT_NEAR(params.v3_, 10.583, 0.01);
  EXPECT_NEAR(params.x3_, 12, 0.01);
  EXPECT_NEAR(params.a3_, 2, 0.01);



  // Traj lower 

  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = -0.5; // unrealistic time, only used for unit test purpose

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        8.0, 
                                        11.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, current_time, 0.01);
  EXPECT_NEAR(params.v0_, 8, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, current_time + 2.0, 0.01);
  EXPECT_NEAR(params.v1_, 4.0, 0.01);
  EXPECT_NEAR(params.x1_, 12, 0.01);
  EXPECT_NEAR(params.a1_, -2, 0.01);
  EXPECT_NEAR(params.t2_, current_time + 2.0, 0.01);
  EXPECT_NEAR(params.v2_, 4.0, 0.01);
  EXPECT_NEAR(params.x2_, 12, 0.01);
  EXPECT_NEAR(params.a2_, -2, 0.01);
  EXPECT_NEAR(params.t3_, current_time + 2.0, 0.01);
  EXPECT_NEAR(params.v3_, 4.0, 0.01);
  EXPECT_NEAR(params.x3_, 12, 0.01);
  EXPECT_NEAR(params.a3_, -2, 0.01);


  // Traj failure

  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = 4.0; 

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        8.0, 
                                        11.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, 0.0, 0.01);
  EXPECT_NEAR(params.v0_, 0.0, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, 0.0, 0.01);
  EXPECT_NEAR(params.v1_, 0.0, 0.01);
  EXPECT_NEAR(params.x1_, 0, 0.01);
  EXPECT_NEAR(params.a1_, 0, 0.01);

  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = 6.0;

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        8.0, 
                                        11.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, 0.0, 0.01);
  EXPECT_NEAR(params.v0_, 0.0, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, 0.0, 0.01);
  EXPECT_NEAR(params.v1_, 0.0, 0.01);
  EXPECT_NEAR(params.x1_, 0, 0.01);
  EXPECT_NEAR(params.a1_, 0, 0.01);

    
  ////////// CASE 2 ////////////////

  // Traj upper GREEN

  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = 2.0;

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        9.0, 
                                        11.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, current_time, 0.01);
  EXPECT_NEAR(params.v0_, 9, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, current_time + 1.0, 0.01);
  EXPECT_NEAR(params.v1_, 11.0, 0.01);
  EXPECT_NEAR(params.x1_, 10, 0.01);
  EXPECT_NEAR(params.a1_, 2.0, 0.01);
  EXPECT_NEAR(params.t2_, current_time + 1.1818, 0.01);
  EXPECT_NEAR(params.v2_, 11.0, 0.01);
  EXPECT_NEAR(params.x2_, 12, 0.01);
  EXPECT_NEAR(params.a2_, 0.0, 0.01);
  EXPECT_NEAR(params.t3_, current_time + 1.1818, 0.01);
  EXPECT_NEAR(params.v3_, 11.0, 0.01);
  EXPECT_NEAR(params.x3_, 12, 0.01);
  EXPECT_NEAR(params.a3_, 0.0, 0.01);

  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = 0.0;

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        9.0, 
                                        11.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, current_time, 0.01);
  EXPECT_NEAR(params.v0_, 9, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, current_time + 1.0, 0.01);
  EXPECT_NEAR(params.v1_, 11.0, 0.01);
  EXPECT_NEAR(params.x1_, 10, 0.01);
  EXPECT_NEAR(params.a1_, 2.0, 0.01);
  EXPECT_NEAR(params.t2_, current_time + 1.1818, 0.01);
  EXPECT_NEAR(params.v2_, 11.0, 0.01);
  EXPECT_NEAR(params.x2_, 12, 0.01);
  EXPECT_NEAR(params.a2_, 0.0, 0.01);
  EXPECT_NEAR(params.t3_, current_time + 1.1818, 0.01);
  EXPECT_NEAR(params.v3_, 11.0, 0.01);
  EXPECT_NEAR(params.x3_, 12, 0.01);
  EXPECT_NEAR(params.a3_, 0.0, 0.01);

   // Traj lower GREEN

  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = -0.5; //unrealistic time only for unit test only

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        9.0, 
                                        11.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, current_time, 0.01);
  EXPECT_NEAR(params.v0_, 9, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, current_time + 1.6277, 0.01);
  EXPECT_NEAR(params.v1_, 5.74456, 0.01);
  EXPECT_NEAR(params.x1_, 12, 0.01);
  EXPECT_NEAR(params.a1_, -2.0, 0.01);
  EXPECT_NEAR(params.t2_, current_time + 1.6277, 0.01);
  EXPECT_NEAR(params.v2_, 5.74456, 0.01);
  EXPECT_NEAR(params.x2_, 12, 0.01);
  EXPECT_NEAR(params.a2_, -2.0, 0.01);
  EXPECT_NEAR(params.t3_, current_time + 1.6277, 0.01);
  EXPECT_NEAR(params.v3_, 5.74456, 0.01);
  EXPECT_NEAR(params.x3_, 12, 0.01);
  EXPECT_NEAR(params.a3_, -2.0, 0.01);

  // TRAJ FAILURE

  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = 4.0;

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        9.0, 
                                        11.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, 0.0, 0.01);
  EXPECT_NEAR(params.v0_, 0.0, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, 0.0, 0.01);
  EXPECT_NEAR(params.v1_, 0.0, 0.01);
  EXPECT_NEAR(params.x1_, 0, 0.01);
  EXPECT_NEAR(params.a1_, 0, 0.01);

  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = 7.0;

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        9.0, 
                                        11.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, 0.0, 0.01);
  EXPECT_NEAR(params.v0_, 0.0, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, 0.0, 0.01);
  EXPECT_NEAR(params.v1_, 0.0, 0.01);
  EXPECT_NEAR(params.x1_, 0, 0.01);
  EXPECT_NEAR(params.a1_, 0, 0.01);

  ////////// CASE 3 ////////////////
  
  // Traj upper GREEN  
  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = 0.0;

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        9.0, 
                                        7.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, current_time, 0.01);
  EXPECT_NEAR(params.v0_, 9, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, current_time + 0.4444, 0.01);
  EXPECT_NEAR(params.v1_, 9.0, 0.01);
  EXPECT_NEAR(params.x1_, 4.0, 0.01);
  EXPECT_NEAR(params.a1_, 0.0, 0.01);
  EXPECT_NEAR(params.t2_, current_time + 1.4444, 0.01);
  EXPECT_NEAR(params.v2_, 7.0, 0.01);
  EXPECT_NEAR(params.x2_, 12, 0.01);
  EXPECT_NEAR(params.a2_, -2.0, 0.01);
  EXPECT_NEAR(params.t3_, current_time + 1.4444, 0.01);
  EXPECT_NEAR(params.v3_, 7.0, 0.01);
  EXPECT_NEAR(params.x3_, 12, 0.01);
  EXPECT_NEAR(params.a3_, -2.0, 0.01);

  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = 2.0;

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        9.0, 
                                        7.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, current_time, 0.01);
  EXPECT_NEAR(params.v0_, 9, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, current_time + 0.4444, 0.01);
  EXPECT_NEAR(params.v1_, 9.0, 0.01);
  EXPECT_NEAR(params.x1_, 4.0, 0.01);
  EXPECT_NEAR(params.a1_, 0.0, 0.01);
  EXPECT_NEAR(params.t2_, current_time + 1.4444, 0.01);
  EXPECT_NEAR(params.v2_, 7.0, 0.01);
  EXPECT_NEAR(params.x2_, 12, 0.01);
  EXPECT_NEAR(params.a2_, -2.0, 0.01);
  EXPECT_NEAR(params.t3_, current_time + 1.4444, 0.01);
  EXPECT_NEAR(params.v3_, 7.0, 0.01);
  EXPECT_NEAR(params.x3_, 12, 0.01);
  EXPECT_NEAR(params.a3_, -2.0, 0.01);

  // Traj lower GREEN
  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = -0.5; //unrealistic time only for unit test only

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        9.0, 
                                        7.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, current_time, 0.01);
  EXPECT_NEAR(params.v0_, 9, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, current_time + 1.6277, 0.01);
  EXPECT_NEAR(params.v1_, 5.74456, 0.01);
  EXPECT_NEAR(params.x1_, 12, 0.01);
  EXPECT_NEAR(params.a1_, -2.0, 0.01);
  EXPECT_NEAR(params.t2_, current_time + 1.6277, 0.01);
  EXPECT_NEAR(params.v2_, 5.74456, 0.01);
  EXPECT_NEAR(params.x2_, 12, 0.01);
  EXPECT_NEAR(params.a2_, -2.0, 0.01);
  EXPECT_NEAR(params.t3_, current_time + 1.6277, 0.01);
  EXPECT_NEAR(params.v3_, 5.74456, 0.01);
  EXPECT_NEAR(params.x3_, 12, 0.01);
  EXPECT_NEAR(params.a3_, -2.0, 0.01);

  ////////// CASE 4 ////////////////
  
  // Traj upper GREEN  
  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = 0.0;

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        10.0, 
                                        7.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, current_time, 0.01);
  EXPECT_NEAR(params.v0_, 10.0, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, current_time + 1.39445, 0.01);
  EXPECT_NEAR(params.v1_, 7.211, 0.01);
  EXPECT_NEAR(params.x1_, 12.0, 0.01);
  EXPECT_NEAR(params.a1_, -2.0, 0.01);
  EXPECT_NEAR(params.t2_, current_time + 1.39445, 0.01);
  EXPECT_NEAR(params.v2_, 7.211, 0.01);
  EXPECT_NEAR(params.x2_, 12.0, 0.01);
  EXPECT_NEAR(params.a2_, -2.0, 0.01);
  EXPECT_NEAR(params.t3_, current_time + 1.39445, 0.01);
  EXPECT_NEAR(params.v3_, 7.211, 0.01);
  EXPECT_NEAR(params.x3_, 12.0, 0.01);
  EXPECT_NEAR(params.a3_, -2.0, 0.01);

  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = 2.0;

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        10.0, 
                                        7.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, current_time, 0.01);
  EXPECT_NEAR(params.v0_, 10.0, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, current_time + 1.39445, 0.01);
  EXPECT_NEAR(params.v1_, 7.211, 0.01);
  EXPECT_NEAR(params.x1_, 12.0, 0.01);
  EXPECT_NEAR(params.a1_, -2.0, 0.01);
  EXPECT_NEAR(params.t2_, current_time + 1.39445, 0.01);
  EXPECT_NEAR(params.v2_, 7.211, 0.01);
  EXPECT_NEAR(params.x2_, 12.0, 0.01);
  EXPECT_NEAR(params.a2_, -2.0, 0.01);
  EXPECT_NEAR(params.t3_, current_time + 1.39445, 0.01);
  EXPECT_NEAR(params.v3_, 7.211, 0.01);
  EXPECT_NEAR(params.x3_, 12.0, 0.01);
  EXPECT_NEAR(params.a3_, -2.0, 0.01);

  // Traj lower GREEN
  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = 4; 

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        10.0, 
                                        7.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, 0.0, 0.01);
  EXPECT_NEAR(params.v0_, 0.0, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, 0.0, 0.01);
  EXPECT_NEAR(params.v1_, 0.0, 0.01);
  EXPECT_NEAR(params.x1_, 0, 0.01);
  EXPECT_NEAR(params.a1_, 0, 0.01);

  green_start_time = 1.0;
  green_end_time = 5.0;
  remaining_distance = 12.0;
  current_time = -0.5; //unrealistic time only for unit test

  params = lcip.handleFailureCaseHelper(signal, 
                                        current_time, 
                                        10.0, 
                                        7.0, 
                                        11.176, 
                                        remaining_distance, 
                                        remaining_distance);

  EXPECT_NEAR(params.t0_, 0.0, 0.01);
  EXPECT_NEAR(params.v0_, 0.0, 0.01);
  EXPECT_NEAR(params.x0_, 0, 0.01);
  EXPECT_NEAR(params.t1_, 0.0, 0.01);
  EXPECT_NEAR(params.v1_, 0.0, 0.01);
  EXPECT_NEAR(params.x1_, 0, 0.01);
  EXPECT_NEAR(params.a1_, 0, 0.01);

} 

TEST(LCIStrategicPluginTest, moboperationcbtest)
{
  cav_msgs::MobilityOperation msg;
  msg.strategy = "Carma/signalized_intersection";

  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  LCIStrategicPluginConfig config;
  LCIStrategicPlugin lcip(wm, config);

  ASSERT_EQ(lcip.approaching_light_controlled_intersection_, false);
  auto msg_ptr = boost::make_shared<const cav_msgs::MobilityOperation>(msg);
  lcip.mobilityOperationCb(msg_ptr);

  ASSERT_EQ(lcip.approaching_light_controlled_intersection_, true);
}

TEST(LCIStrategicPluginTest, parseStrategyParamstest)
{
  cav_msgs::MobilityOperation msg;
  msg.strategy_params =  "et:32000";

  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  LCIStrategicPluginConfig config;
  LCIStrategicPlugin lcip(wm, config);

  lcip.parseStrategyParams(msg.strategy_params);

  EXPECT_EQ(32000, lcip.scheduled_enter_time_);

  cav_msgs::MobilityOperation outgoing_msg = lcip.generateMobilityOperation();
  EXPECT_EQ(outgoing_msg.strategy, "Carma/signalized_intersection");
  EXPECT_EQ(outgoing_msg.m_header.sender_id, config.vehicle_id);
  std::cout << "strategy_param: " << outgoing_msg.strategy_params << std::endl;
}

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
TEST_F(LCIStrategicTestFixture, planWhenETInTBD)
{
  LCIStrategicPluginConfig config;
  config.enable_carma_streets_connection = true;
  config.green_light_time_buffer = 1.0;
  LCIStrategicPlugin lcip(cmw_, config);

  cav_srvs::PlanManeuversRequest req;
  cav_srvs::PlanManeuversResponse resp;

  // Light will be located on lanelet 1200 (300m) and control lanelet 1202, 1203
  lanelet::Id traffic_light_id = lanelet::utils::getId();
  carma_wm::test::addTrafficLight(cmw_, traffic_light_id, {1200}, { 1203 });

  req.header.stamp = ros::Time(6); // GREEN ends at 7, but 1.3 sec minimum to intersection
  req.veh_x = 1.85;
  req.veh_y = 285; // traffic light is at 300
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 10.0;
  req.veh_lane_id = "1200";
  lcip.scheduled_enter_time_ = 20000; // 20s in TBD
  auto signal = cmw_->getMutableMap()->laneletLayer.get(1200).regulatoryElementsAs<lanelet::CarmaTrafficSignal>().front();
  LCIStrategicPlugin::VehicleState current_state = lcip.extractInitialState(req);

  double green_start_time = 3.0;
  double green_end_time = 7.0;

  signal->fixed_cycle_duration = lanelet::time::durationFromSec(0.0); //dynamic
  signal->recorded_time_stamps = {};
  signal->recorded_start_time_stamps = {};
  signal->recorded_time_stamps.push_back(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(boost::posix_time::from_time_t(green_start_time), lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN));
  signal->recorded_start_time_stamps.push_back(boost::posix_time::from_time_t(0.0));
  signal->recorded_time_stamps.push_back(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(boost::posix_time::from_time_t(green_end_time), lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED));
  signal->recorded_start_time_stamps.push_back(boost::posix_time::from_time_t(green_start_time));

  ////////// CASE 1: When close to intersection check for basic red light violation ////////////////
  lcip.last_case_num_ = TSCase::CASE_1; //simulating when vehicle is speeding up while ET goes into TBD
  lcip.planWhenAPPROACHING(req, resp, current_state, signal, cmw_->getMutableMap()->laneletLayer.get(1200), cmw_->getMutableMap()->laneletLayer.get(1203), cmw_->getMutableMap()->laneletLayer.get(1200));

  ASSERT_FALSE(resp.new_plan.maneuvers.empty());
  ASSERT_TRUE(resp.new_plan.maneuvers.front().type == cav_msgs::Maneuver::STOP_AND_WAIT);
  ASSERT_NEAR(resp.new_plan.maneuvers.front().stop_and_wait_maneuver.parameters.float_valued_meta_data[1], 3.0, 0.01);
 
  ////////// CASE 2: Check edge cases when not red light violating ////////////////
  resp.new_plan.maneuvers = {};
  req.header.stamp = ros::Time(0);
  req.veh_x = 1.85;
  req.veh_y = 270; // traffic light is at 300 
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 9.5;
  req.veh_lane_id = "1200";
  lcip.scheduled_enter_time_ = 20000; // 20s in TBD
  current_state = lcip.extractInitialState(req);

  lcip.last_case_num_ = TSCase::CASE_1; //simulating when vehicle is speeding up while ET goes into TBD
  
  lcip.planWhenAPPROACHING(req, resp, current_state, signal, cmw_->getMutableMap()->laneletLayer.get(1200), cmw_->getMutableMap()->laneletLayer.get(1203), cmw_->getMutableMap()->laneletLayer.get(1200));

  ASSERT_FALSE(resp.new_plan.maneuvers.empty());
  ASSERT_TRUE(resp.new_plan.maneuvers.front().type == cav_msgs::Maneuver::LANE_FOLLOWING); // not stop because edge case

  ////////// CASE 3: When far from desired stopping distance and ET in TBD, keep going ////////////////
  resp.new_plan.maneuvers = {};
  req.header.stamp = ros::Time(2);
  req.veh_x = 1.85;
  req.veh_y = 200; // traffic light is at 300 
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 10.0;
  req.veh_lane_id = "1200";
  lcip.scheduled_enter_time_ = 20000; // 20s in TBD
  current_state = lcip.extractInitialState(req);

  lcip.last_case_num_ = TSCase::CASE_1; //simulating when vehicle is speeding up while ET goes into TBD
  
  lcip.planWhenAPPROACHING(req, resp, current_state, signal, cmw_->getMutableMap()->laneletLayer.get(1200), cmw_->getMutableMap()->laneletLayer.get(1203), cmw_->getMutableMap()->laneletLayer.get(1200));

  ASSERT_FALSE(resp.new_plan.maneuvers.empty());
  ASSERT_TRUE(resp.new_plan.maneuvers.front().type == cav_msgs::Maneuver::LANE_FOLLOWING); //not stop

  ////////// CASE 4: Actual run from platform ////////////////
  LCIStrategicPluginConfig config_real;
  config_real.enable_carma_streets_connection = true;
  config_real.green_light_time_buffer = 1.0;
  config_real.deceleration_fraction = 0.7;  // actual was 0.8 which failed. 
  config_real.vehicle_decel_limit_multiplier = 1.0;
  config_real.vehicle_accel_limit_multiplier = 1.0;
  config_real.desired_distance_to_stop_buffer = 15.0;
  config_real.stopping_location_buffer = 8.0;

  LCIStrategicPlugin lcip_real(cmw_, config_real);

  green_start_time = 8087.69;
  green_end_time = 8098.6;

  signal->recorded_time_stamps = {};
  signal->recorded_start_time_stamps = {};
  signal->recorded_time_stamps.push_back(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(boost::posix_time::from_time_t(green_start_time), lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN));
  signal->recorded_start_time_stamps.push_back(boost::posix_time::from_time_t(0.0));
  signal->recorded_time_stamps.push_back(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(boost::posix_time::from_time_t(green_end_time), lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED));
  signal->recorded_start_time_stamps.push_back(boost::posix_time::from_time_t(green_start_time));

  resp.new_plan.maneuvers = {};
  req.header.stamp = ros::Time(8097.49);
  req.veh_x = 1.85;
  req.veh_y = 300.0 - 49.63;         //unit test light is at 300, actual downtrack of the run at 113.479 with 49.63meters left
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 10.402;
  req.veh_lane_id = "1200";
  lcip_real.scheduled_enter_time_ = 8103635; // 8103.635 
  current_state = lcip_real.extractInitialState(req);

  lcip_real.last_case_num_ = TSCase::CASE_3; //simulating when vehicle is speeding up while ET goes into TBD
  
  lcip_real.planWhenAPPROACHING(req, resp, current_state, signal, cmw_->getMutableMap()->laneletLayer.get(1200), cmw_->getMutableMap()->laneletLayer.get(1203), cmw_->getMutableMap()->laneletLayer.get(1200));

  ASSERT_FALSE(resp.new_plan.maneuvers.empty());
  ASSERT_TRUE(resp.new_plan.maneuvers.front().type == cav_msgs::Maneuver::STOP_AND_WAIT); //not stop
  ASSERT_NEAR(resp.new_plan.maneuvers.front().stop_and_wait_maneuver.parameters.float_valued_meta_data[1], 2.0, 0.01);

  // approximately 3sec later assuming it was not able to slow down, check emergency stopping
  resp.new_plan.maneuvers = {};
  req.header.stamp = ros::Time(8100.49);
  req.veh_x = 1.85;
  req.veh_y = 300.0 - 14.0;         //unit test light is at 300
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 9.402;
  req.veh_lane_id = "1200";
  lcip_real.scheduled_enter_time_ = 8103635; // 8103.635 
  current_state = lcip_real.extractInitialState(req);

  lcip_real.last_case_num_ = TSCase::STOPPING;
  
  lcip_real.planWhenAPPROACHING(req, resp, current_state, signal, cmw_->getMutableMap()->laneletLayer.get(1200), cmw_->getMutableMap()->laneletLayer.get(1203), cmw_->getMutableMap()->laneletLayer.get(1200));

  ASSERT_FALSE(resp.new_plan.maneuvers.empty());
  ASSERT_TRUE(resp.new_plan.maneuvers.front().type == cav_msgs::Maneuver::STOP_AND_WAIT); //not stop
  ASSERT_NEAR(resp.new_plan.maneuvers.front().stop_and_wait_maneuver.parameters.float_valued_meta_data[1], 4.0, 0.01);

  ///////// CASE 5. Hypothetical run that can make the GREEN using handleFailure ///////////
  LCIStrategicPluginConfig config_failure;
  config_failure.vehicle_accel_limit = 2;
  config_failure.vehicle_accel_limit_multiplier = 1;
  config_failure.vehicle_decel_limit_multiplier = 1;
  config_failure.vehicle_decel_limit= 2;
  config_failure.green_light_time_buffer = 1.0;
  ros::Time::setNow(ros::Time(0));
  LCIStrategicPlugin lcip_failure(cmw_, config_failure);

  ////////// CASE 1 //////////////// copy from test HandleFailureCaseHelper
  // Traj upper 1

  green_start_time = 1.0;
  green_end_time = 5.0;

  signal->fixed_cycle_duration = lanelet::time::durationFromSec(0.0); //dynamic
  signal->recorded_time_stamps = {};
  signal->recorded_start_time_stamps = {};
  signal->recorded_time_stamps.push_back(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(boost::posix_time::from_time_t(green_start_time), lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN));
  signal->recorded_start_time_stamps.push_back(boost::posix_time::from_time_t(0.0));
  signal->recorded_time_stamps.push_back(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(boost::posix_time::from_time_t(green_end_time), lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED));
  signal->recorded_start_time_stamps.push_back(boost::posix_time::from_time_t(green_start_time));

  resp.new_plan.maneuvers = {};
  req.header.stamp = ros::Time(2.0);
  req.veh_x = 1.85;
  req.veh_y = 300.0 - 12.0;         //unit test light is at 300
  req.veh_downtrack = req.veh_y;
  req.veh_logitudinal_velocity = 9.0;
  req.veh_lane_id = "1200";
  lcip_failure.scheduled_enter_time_ = 20000; // 20.0 
  current_state = lcip_failure.extractInitialState(req);

  lcip_failure.last_case_num_ = TSCase::CASE_1;
  
  lcip_failure.planWhenAPPROACHING(req, resp, current_state, signal, cmw_->getMutableMap()->laneletLayer.get(1200), cmw_->getMutableMap()->laneletLayer.get(1203), cmw_->getMutableMap()->laneletLayer.get(1200));

  ASSERT_FALSE(resp.new_plan.maneuvers.empty());
  ASSERT_TRUE(resp.new_plan.maneuvers.front().type == cav_msgs::Maneuver::LANE_FOLLOWING); //not stop

}


}  // namespace lci_strategic_plugin