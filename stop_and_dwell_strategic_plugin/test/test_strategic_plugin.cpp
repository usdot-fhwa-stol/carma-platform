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
#include <rclcpp/rclcpp.hpp>
#include "stop_and_dwell_strategic_plugin.hpp"
#include "stop_and_dwell_strategic_plugin_config.hpp"
#include <carma_wm/CARMAWorldModel.hpp>
#include <carma_wm/WMTestLibForGuidance.hpp>


// Unit tests for strategic plugin helper methods
namespace stop_and_dwell_strategic_plugin
{

TEST(StopAndDwellStrategicPluginTest, composeLaneFollowingManeuverMessage)
{
  auto sd_node = std::make_shared<stop_and_dwell_strategic_plugin::StopAndDwellStrategicPlugin>(rclcpp::NodeOptions());
  sd_node->configure();
  sd_node->activate();
  StopAndDwellStrategicPluginConfig config;
  auto result =
      sd_node->composeLaneFollowingManeuverMessage(1, 10.2, 20.4, 5, 10, rclcpp::Time(1.2*1e9), 1.0, { 1200, 1201 });

  ASSERT_EQ(carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING, result.type);
  ASSERT_EQ(carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION, result.lane_following_maneuver.parameters.negotiation_type);
  ASSERT_EQ(carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN,
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
}

TEST(StopAndDwellStrategicPluginTest, composeStopAndWaitManeuverMessage)
{
  auto sd_node = std::make_shared<stop_and_dwell_strategic_plugin::StopAndDwellStrategicPlugin>(rclcpp::NodeOptions());
  sd_node->configure();
  sd_node->activate();
  StopAndDwellStrategicPluginConfig config;
  auto result = sd_node->composeStopAndWaitManeuverMessage(10.2, 20.4, 5, 1200, 1201, 0.56, rclcpp::Time(1.2*1e9), rclcpp::Time(2.2*1e9));

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

TEST(StopAndDwellStrategicPluginTest, findSpeedLimit)
{
  auto sd_node = std::make_shared<stop_and_dwell_strategic_plugin::StopAndDwellStrategicPlugin>(rclcpp::NodeOptions());
  sd_node->configure();
  sd_node->activate();
  
  std::shared_ptr<carma_wm::CARMAWorldModel> wm;
  carma_wm::test::MapOptions options;
  options.lane_length_ = 25;
  options.lane_width_ = 3.7;
  options.speed_limit_ = carma_wm::test::MapOptions::SpeedLimit::DEFAULT;

  wm = carma_wm::test::getGuidanceTestMap(options);
  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

  sd_node->set_wm(wm);

  auto ll_iterator = wm->getMap()->laneletLayer.find(1200);
  if (ll_iterator == wm->getMap()->laneletLayer.end())
  FAIL() << "Expected lanelet not present in map. Unit test may not be structured correctly";
  
  ASSERT_NEAR(11.176, sd_node->findSpeedLimit(*ll_iterator), 0.00001);
}

TEST(StopAndDwellStrategicPluginTest, maneuvercbtest)
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

}

} // namespace stop_and_dwell_strategic_plugin