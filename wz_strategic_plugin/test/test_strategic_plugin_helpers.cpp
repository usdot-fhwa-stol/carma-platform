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

// Unit tests for strategic plugin helper methods
namespace wz_strategic_plugin
{
TEST_F(WorkZoneTestFixture, getDiscoveryMsg)
{
  WzStrategicPluginConfig config;
  config.strategic_plugin_name = "test name";
  WzStrategicPlugin wzp(cmw_, config);

  auto msg = wzp.getDiscoveryMsg();
  ASSERT_TRUE(msg.name.compare("test name") == 0);
  ASSERT_TRUE(msg.version_id.compare("v1.0") == 0);
  ASSERT_TRUE(msg.available);
  ASSERT_TRUE(msg.activated);
  ASSERT_EQ(msg.type, cav_msgs::Plugin::STRATEGIC);
  ASSERT_TRUE(msg.capability.compare("strategic_plan/plan_maneuvers") == 0);
}

TEST_F(WorkZoneTestFixture, supportedLightState)
{
  WzStrategicPluginConfig config;
  WzStrategicPlugin wzp(cmw_, config);

  ASSERT_TRUE(wzp.supportedLightState(lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED));
  ASSERT_TRUE(wzp.supportedLightState(lanelet::CarmaTrafficSignalState::PROTECTED_CLEARANCE));
  ASSERT_TRUE(wzp.supportedLightState(lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN));

  ASSERT_FALSE(wzp.supportedLightState(lanelet::CarmaTrafficSignalState::UNAVAILABLE));
  ASSERT_FALSE(wzp.supportedLightState(lanelet::CarmaTrafficSignalState::DARK));
  ASSERT_FALSE(wzp.supportedLightState(lanelet::CarmaTrafficSignalState::STOP_THEN_PROCEED));
  ASSERT_FALSE(wzp.supportedLightState(lanelet::CarmaTrafficSignalState::PRE_MOVEMENT));
  ASSERT_FALSE(wzp.supportedLightState(lanelet::CarmaTrafficSignalState::PERMISSIVE_MOVEMENT_ALLOWED));
  ASSERT_FALSE(wzp.supportedLightState(lanelet::CarmaTrafficSignalState::PERMISSIVE_CLEARANCE));
  ASSERT_FALSE(wzp.supportedLightState(lanelet::CarmaTrafficSignalState::CAUTION_CONFLICTING_TRAFFIC));
}

TEST_F(WorkZoneTestFixture, estimate_distance_to_stop)
{
  WzStrategicPluginConfig config;
  WzStrategicPlugin wzp(cmw_, config);

  ASSERT_NEAR(wzp.estimate_distance_to_stop(8.5, 1.3), 27.788461538461537, 0.00001);
}

TEST_F(WorkZoneTestFixture, estimate_time_to_stop)
{
  WzStrategicPluginConfig config;
  WzStrategicPlugin wzp(cmw_, config);

  ASSERT_NEAR(wzp.estimate_time_to_stop(27.788461538461537, 8.5), 6.538461538461538, 0.00001);
}

TEST_F(WorkZoneTestFixture, extractInitialState)
{
  WzStrategicPluginConfig config;
  WzStrategicPlugin wzp(cmw_, config);

  cav_srvs::PlanManeuversRequest req;
  req.header.stamp = ros::Time(5.0);
  req.veh_downtrack = 23.5;
  req.veh_logitudinal_velocity = 10.2;
  req.veh_lane_id = "1002";

  WzStrategicPlugin::VehicleState result = wzp.extractInitialState(req);

  ASSERT_EQ(req.header.stamp.toSec(), result.stamp.toSec());
  ASSERT_EQ(req.veh_downtrack, result.downtrack);
  ASSERT_EQ(req.veh_logitudinal_velocity, result.speed);
  ASSERT_TRUE(req.veh_lane_id.compare(std::to_string(result.lane_id)) == 0);
}

TEST_F(WorkZoneTestFixture, validLightState)
{
  WzStrategicPluginConfig config;
  WzStrategicPlugin wzp(cmw_, config);
  boost::posix_time::ptime dummy_time;
  ASSERT_TRUE(wzp.validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED), ros::Time(1)));
  ASSERT_TRUE(wzp.validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::PROTECTED_CLEARANCE), ros::Time(1)));
  ASSERT_TRUE(wzp.validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN), ros::Time(1)));
  
  ASSERT_FALSE(wzp.validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::UNAVAILABLE), ros::Time(1)));
  ASSERT_FALSE(wzp.validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::DARK), ros::Time(1)));
  ASSERT_FALSE(wzp.validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::STOP_THEN_PROCEED), ros::Time(1)));
  ASSERT_FALSE(wzp.validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::PRE_MOVEMENT), ros::Time(1)));
  ASSERT_FALSE(wzp.validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::PERMISSIVE_MOVEMENT_ALLOWED), ros::Time(1)));
  ASSERT_FALSE(wzp.validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::PERMISSIVE_CLEARANCE), ros::Time(1)));
  ASSERT_FALSE(wzp.validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::CAUTION_CONFLICTING_TRAFFIC), ros::Time(1)));
  ASSERT_FALSE(wzp.validLightState(boost::none, ros::Time(1)));
}

TEST_F(WorkZoneTestFixture, getLaneletsBetweenWithException)
{
  WzStrategicPluginConfig config;
  WzStrategicPlugin wzp(cmw_, config);

  auto result = wzp.getLaneletsBetweenWithException(24, 26, true);

  ASSERT_EQ(2, result.size());
  ASSERT_EQ(1200, result[0].id());
  ASSERT_EQ(1201, result[1].id());

  result = wzp.getLaneletsBetweenWithException(24, 24, true);

  ASSERT_EQ(1, result.size());
  ASSERT_EQ(1200, result[0].id());

  ASSERT_THROW(wzp.getLaneletsBetweenWithException(24, 23, true), std::invalid_argument);
}

TEST_F(WorkZoneTestFixture, composeLaneFollowingManeuverMessage)
{
  WzStrategicPluginConfig config;
  WzStrategicPlugin wzp(cmw_, config);

  auto result =
      wzp.composeLaneFollowingManeuverMessage(10.2, 20.4, 5, 10, ros::Time(1.2), ros::Time(2.2), { 1200, 1201 });

  ASSERT_EQ(cav_msgs::Maneuver::LANE_FOLLOWING, result.type);
  ASSERT_EQ(cav_msgs::ManeuverParameters::NO_NEGOTIATION, result.lane_following_maneuver.parameters.negotiation_type);
  ASSERT_EQ(cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN,
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
  ASSERT_EQ(ros::Time(2.2), result.lane_following_maneuver.end_time);
  ASSERT_EQ(2, result.lane_following_maneuver.lane_ids.size());
  ASSERT_TRUE(result.lane_following_maneuver.lane_ids[0].compare("1200") == 0);
  ASSERT_TRUE(result.lane_following_maneuver.lane_ids[1].compare("1201") == 0);
}

TEST_F(WorkZoneTestFixture, composeStopAndWaitManeuverMessage)
{
  WzStrategicPluginConfig config;
  WzStrategicPlugin wzp(cmw_, config);

  double stopping_acceleration = 1.0;
  auto result = wzp.composeStopAndWaitManeuverMessage(10.2, 20.4, 5, 1200, 1201, ros::Time(1.2), ros::Time(2.2), stopping_acceleration);

  ASSERT_EQ(cav_msgs::Maneuver::STOP_AND_WAIT, result.type);
  ASSERT_EQ(cav_msgs::ManeuverParameters::NO_NEGOTIATION, result.stop_and_wait_maneuver.parameters.negotiation_type);
  ASSERT_EQ(cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN | cav_msgs::ManeuverParameters::HAS_FLOAT_META_DATA,
            result.stop_and_wait_maneuver.parameters.presence_vector);  // TODO diff
  ASSERT_TRUE(config.stop_and_wait_plugin_name.compare(
                  result.stop_and_wait_maneuver.parameters.planning_tactical_plugin) == 0);
  ASSERT_TRUE(
      config.strategic_plugin_name.compare(result.stop_and_wait_maneuver.parameters.planning_strategic_plugin) == 0);

  ASSERT_EQ(10.2, result.stop_and_wait_maneuver.start_dist);
  ASSERT_EQ(20.4, result.stop_and_wait_maneuver.end_dist);
  ASSERT_EQ(5, result.stop_and_wait_maneuver.start_speed);
  ASSERT_EQ(ros::Time(1.2), result.stop_and_wait_maneuver.start_time);
  ASSERT_EQ(ros::Time(2.2), result.stop_and_wait_maneuver.end_time);
  ASSERT_TRUE(result.stop_and_wait_maneuver.starting_lane_id.compare("1200") == 0);
  ASSERT_TRUE(result.stop_and_wait_maneuver.ending_lane_id.compare("1201") == 0);
}

TEST_F(WorkZoneTestFixture, composeIntersectionTransitMessage)
{
  WzStrategicPluginConfig config;
  WzStrategicPlugin wzp(cmw_, config);

  auto result = wzp.composeIntersectionTransitMessage(10.2, 20.4, 5, 10, ros::Time(1.2), ros::Time(2.2), 1200, 1201);

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

TEST_F(WorkZoneTestFixture, findSpeedLimit)
{
  WzStrategicPluginConfig config;
  WzStrategicPlugin wzp(cmw_, config);

  auto ll_iterator = cmw_->getMap()->laneletLayer.find(1200);
  if (ll_iterator == cmw_->getMap()->laneletLayer.end())
    FAIL() << "Expected lanelet not present in map. Unit test may not be structured correctly";
  
  ASSERT_NEAR(11.176, wzp.findSpeedLimit(*ll_iterator), 0.00001);
}

}  // namespace wz_strategic_plugin