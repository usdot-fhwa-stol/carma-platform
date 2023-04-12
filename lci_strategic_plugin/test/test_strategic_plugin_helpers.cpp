/*
 * Copyright (C) 2023 LEIDOS.
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

#include "test_fixture.hpp"
#include "lci_strategic_plugin/lci_strategic_plugin.hpp"
#include "lci_strategic_plugin/lci_strategic_plugin_config.hpp"

// Unit tests for strategic plugin helper methods
namespace lci_strategic_plugin
{

TEST_F(LCIStrategicTestFixture, supportedLightState)
{
  LCIStrategicPluginConfig config;
    auto lcip = std::make_shared<lci_strategic_plugin::LCIStrategicPlugin>(rclcpp::NodeOptions());

  lcip->wm_ = cmw_;
  lcip->config_ = config;

  ASSERT_TRUE(lcip->supportedLightState(lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED));
  ASSERT_TRUE(lcip->supportedLightState(lanelet::CarmaTrafficSignalState::PROTECTED_CLEARANCE));
  ASSERT_TRUE(lcip->supportedLightState(lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN));

  ASSERT_FALSE(lcip->supportedLightState(lanelet::CarmaTrafficSignalState::UNAVAILABLE));
  ASSERT_FALSE(lcip->supportedLightState(lanelet::CarmaTrafficSignalState::DARK));
  ASSERT_FALSE(lcip->supportedLightState(lanelet::CarmaTrafficSignalState::STOP_THEN_PROCEED));
  ASSERT_FALSE(lcip->supportedLightState(lanelet::CarmaTrafficSignalState::PRE_MOVEMENT));
  ASSERT_FALSE(lcip->supportedLightState(lanelet::CarmaTrafficSignalState::PERMISSIVE_MOVEMENT_ALLOWED));
  ASSERT_FALSE(lcip->supportedLightState(lanelet::CarmaTrafficSignalState::PERMISSIVE_CLEARANCE));
  ASSERT_FALSE(lcip->supportedLightState(lanelet::CarmaTrafficSignalState::CAUTION_CONFLICTING_TRAFFIC));
}

TEST_F(LCIStrategicTestFixture, estimate_distance_to_stop)
{
  LCIStrategicPluginConfig config;
      auto lcip = std::make_shared<lci_strategic_plugin::LCIStrategicPlugin>(rclcpp::NodeOptions());

  lcip->wm_ = cmw_;
  lcip->config_ = config;

  ASSERT_NEAR(lcip->estimate_distance_to_stop(8.5, 1.3), 27.788461538461537, 0.00001);
}

TEST_F(LCIStrategicTestFixture, estimate_time_to_stop)
{
  LCIStrategicPluginConfig config;
      auto lcip = std::make_shared<lci_strategic_plugin::LCIStrategicPlugin>(rclcpp::NodeOptions());

  lcip->wm_ = cmw_;
  lcip->config_ = config;

  ASSERT_NEAR(lcip->estimate_time_to_stop(27.788461538461537, 8.5), 6.538461538461538, 0.00001);
}

TEST_F(LCIStrategicTestFixture, extractInitialState)
{
  LCIStrategicPluginConfig config;
  auto lcip = std::make_shared<lci_strategic_plugin::LCIStrategicPlugin>(rclcpp::NodeOptions());

  lcip->wm_ = cmw_;
  lcip->config_ = config;

  carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Request>();
  req->header.stamp = rclcpp::Time(1e9 * 5.0);
  req->veh_downtrack = 23.5;
  req->veh_logitudinal_velocity = 10.2;
  req->veh_lane_id = "1002";

  LCIStrategicPlugin::VehicleState result = lcip->extractInitialState(req);

  ASSERT_EQ(rclcpp::Time(req->header.stamp, RCL_SYSTEM_TIME).seconds(), rclcpp::Time(result.stamp).seconds());
  ASSERT_EQ(req->veh_downtrack, result.downtrack);
  ASSERT_EQ(req->veh_logitudinal_velocity, result.speed);
  ASSERT_TRUE(req->veh_lane_id.compare(std::to_string(result.lane_id)) == 0);
}

TEST_F(LCIStrategicTestFixture, validLightState)
{
  LCIStrategicPluginConfig config;
      auto lcip = std::make_shared<lci_strategic_plugin::LCIStrategicPlugin>(rclcpp::NodeOptions());

  lcip->wm_ = cmw_;
  lcip->config_ = config;
  boost::posix_time::ptime dummy_time;
  ASSERT_TRUE(lcip->validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED), rclcpp::Time(1e9 * 1)));
  ASSERT_TRUE(lcip->validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::PROTECTED_CLEARANCE), rclcpp::Time(1e9 * 1)));
  ASSERT_TRUE(lcip->validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN), rclcpp::Time(1e9 * 1)));
  
  ASSERT_FALSE(lcip->validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::UNAVAILABLE), rclcpp::Time(1e9 * 1)));
  ASSERT_FALSE(lcip->validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::DARK), rclcpp::Time(1e9 * 1)));
  ASSERT_FALSE(lcip->validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::STOP_THEN_PROCEED), rclcpp::Time(1e9 * 1)));
  ASSERT_FALSE(lcip->validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::PRE_MOVEMENT), rclcpp::Time(1e9 * 1)));
  ASSERT_FALSE(lcip->validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::PERMISSIVE_MOVEMENT_ALLOWED), rclcpp::Time(1e9 * 1)));
  ASSERT_FALSE(lcip->validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::PERMISSIVE_CLEARANCE), rclcpp::Time(1e9 * 1)));
  ASSERT_FALSE(lcip->validLightState(std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(dummy_time, lanelet::CarmaTrafficSignalState::CAUTION_CONFLICTING_TRAFFIC), rclcpp::Time(1e9 * 1)));
  ASSERT_FALSE(lcip->validLightState(boost::none, rclcpp::Time(1e9 * 1)));
}

TEST_F(LCIStrategicTestFixture, getLaneletsBetweenWithException)
{
  LCIStrategicPluginConfig config;
      auto lcip = std::make_shared<lci_strategic_plugin::LCIStrategicPlugin>(rclcpp::NodeOptions());

  lcip->wm_ = cmw_;
  lcip->config_ = config;

  auto result = lcip->getLaneletsBetweenWithException(290, 310, true);

  ASSERT_EQ(2, result.size());
  ASSERT_EQ(1200, result[0].id());
  ASSERT_EQ(1201, result[1].id());

  result = lcip->getLaneletsBetweenWithException(290, 290, true);

  ASSERT_EQ(1, result.size());
  ASSERT_EQ(1200, result[0].id());

  ASSERT_THROW(lcip->getLaneletsBetweenWithException(240, 230, true), std::invalid_argument);
}

TEST_F(LCIStrategicTestFixture, composeTrajectorySmoothingManeuverMessage)
{
  LCIStrategicPluginConfig config;
      auto lcip = std::make_shared<lci_strategic_plugin::LCIStrategicPlugin>(rclcpp::NodeOptions());

  lcip->wm_ = cmw_;
  lcip->config_ = config;
  TrajectoryParams tsp;

  auto result =
      lcip->composeTrajectorySmoothingManeuverMessage(10.2, 20.4, {}, 5, 10, rclcpp::Time(1e9 * 1.2), rclcpp::Time(1e9 * 2.2), tsp);

  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING, result.type);
  ASSERT_EQ(carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION, result.lane_following_maneuver.parameters.negotiation_type);
  ASSERT_EQ(carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN | carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA | carma_planning_msgs::msg::ManeuverParameters::HAS_INT_META_DATA,
            result.lane_following_maneuver.parameters.presence_vector);
  ASSERT_TRUE(config.lane_following_plugin_name.compare(
                  result.lane_following_maneuver.parameters.planning_tactical_plugin) == 0);
  ASSERT_TRUE(
      config.strategic_plugin_name.compare(result.lane_following_maneuver.parameters.planning_strategic_plugin) == 0);

  ASSERT_EQ(10.2, result.lane_following_maneuver.start_dist);
  ASSERT_EQ(20.4, result.lane_following_maneuver.end_dist);
  ASSERT_EQ(5, result.lane_following_maneuver.start_speed);
  ASSERT_EQ(10, result.lane_following_maneuver.end_speed);
  ASSERT_EQ(rclcpp::Time(1e9 * 1.2, RCL_ROS_TIME), rclcpp::Time(result.lane_following_maneuver.start_time, RCL_ROS_TIME));
  ASSERT_EQ(rclcpp::Time(1e9 * 2.2, RCL_ROS_TIME), rclcpp::Time(result.lane_following_maneuver.end_time, RCL_ROS_TIME));
}

TEST_F(LCIStrategicTestFixture, composeStopAndWaitManeuverMessage)
{
  LCIStrategicPluginConfig config;
      auto lcip = std::make_shared<lci_strategic_plugin::LCIStrategicPlugin>(rclcpp::NodeOptions());

  lcip->wm_ = cmw_;
  lcip->config_ = config;

  double stopping_acceleration = 1.0;
  auto result = lcip->composeStopAndWaitManeuverMessage(10.2, 20.4, 5, 1200, 1201, rclcpp::Time(1e9 * 1.2), rclcpp::Time(1e9 * 2.2), stopping_acceleration);

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
  ASSERT_EQ(rclcpp::Time(1e9 * 1.2, RCL_ROS_TIME), rclcpp::Time(result.stop_and_wait_maneuver.start_time,  RCL_ROS_TIME));
  ASSERT_EQ(rclcpp::Time(1e9 * 2.2, RCL_ROS_TIME), rclcpp::Time(result.stop_and_wait_maneuver.end_time,  RCL_ROS_TIME));
  ASSERT_TRUE(result.stop_and_wait_maneuver.starting_lane_id.compare("1200") == 0);
  ASSERT_TRUE(result.stop_and_wait_maneuver.ending_lane_id.compare("1201") == 0);
}

TEST_F(LCIStrategicTestFixture, composeIntersectionTransitMessage)
{
  LCIStrategicPluginConfig config;
      auto lcip = std::make_shared<lci_strategic_plugin::LCIStrategicPlugin>(rclcpp::NodeOptions());

  lcip->wm_ = cmw_;
  lcip->config_ = config;

  auto result = lcip->composeIntersectionTransitMessage(10.2, 20.4, 5, 10, rclcpp::Time(1e9 * 1.2), rclcpp::Time(1e9 * 2.2), 1200, 1201);

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
  ASSERT_EQ(rclcpp::Time(1e9 * 1.2, RCL_ROS_TIME), rclcpp::Time(result.intersection_transit_straight_maneuver.start_time, RCL_ROS_TIME));
  ASSERT_EQ(rclcpp::Time(1e9 * 2.2, RCL_ROS_TIME), rclcpp::Time(result.intersection_transit_straight_maneuver.end_time, RCL_ROS_TIME));
  ASSERT_TRUE(result.intersection_transit_straight_maneuver.starting_lane_id.compare("1200") == 0);
  ASSERT_TRUE(result.intersection_transit_straight_maneuver.ending_lane_id.compare("1201") == 0);
}

TEST_F(LCIStrategicTestFixture, findSpeedLimit)
{
  LCIStrategicPluginConfig config;
      auto lcip = std::make_shared<lci_strategic_plugin::LCIStrategicPlugin>(rclcpp::NodeOptions());

  lcip->wm_ = cmw_;
  lcip->config_ = config;

  auto ll_iterator = cmw_->getMap()->laneletLayer.find(1200);
  if (ll_iterator == cmw_->getMap()->laneletLayer.end())
    FAIL() << "Expected lanelet not present in map. Unit test may not be structured correctly";
  
  ASSERT_NEAR(11.176, lcip->findSpeedLimit(*ll_iterator), 0.00001);
}

}  // namespace lci_strategic_plugin