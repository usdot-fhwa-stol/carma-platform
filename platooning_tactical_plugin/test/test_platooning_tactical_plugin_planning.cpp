/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include <platooning_tactical_plugin/platooning_tactical_plugin.h>
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>
#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/Traits.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <carma_wm_ros2/MapConformer.hpp>
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <unsupported/Eigen/Splines>
#include <carma_ros2_utils/containers/containers.hpp>
#include <carma_ros2_utils/timers/testing/TestTimerFactory.hpp>

typedef Eigen::Spline<float, 2> Spline2d;


using namespace lanelet::units::literals;
namespace platooning_tactical_plugin
{
TEST(PlatooningTacticalPluginTest, testPlanningCallback)
{
  PlatooningTacticalPluginConfig config;
  config.default_downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  PlatooningTacticalPlugin plugin(wm, config,
    std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());

  auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);

  wm->setMap(map);
  carma_wm::test::setSpeedLimit(15_mph, wm);

  /**
   * Total route length should be 100m
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

  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

  carma_planning_msgs::srv::PlanTrajectory::Request req;
  req.vehicle_state.x_pos_global = 1.5;
  req.vehicle_state.y_pos_global = 5;
  req.vehicle_state.orientation = 0;
  req.vehicle_state.longitudinal_vel = 0.0;

  carma_planning_msgs::msg::Maneuver maneuver;
  maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  maneuver.lane_following_maneuver.parameters.negotiation_type = carma_planning_msgs::msg::ManeuverParameters::GENERAL_NEGOTIATION; 
  maneuver.lane_following_maneuver.lane_ids = {"1200"};
  maneuver.lane_following_maneuver.start_dist = 5.0;
  maneuver.lane_following_maneuver.start_time = rclcpp::Time(0.0*1e9);
  maneuver.lane_following_maneuver.start_speed = 0.0;

  maneuver.lane_following_maneuver.end_dist = 14.98835712;
  maneuver.lane_following_maneuver.end_speed = 6.7056;
  maneuver.lane_following_maneuver.end_time = rclcpp::Time(4.4704*1e9);

  carma_planning_msgs::msg::Maneuver maneuver2;
  maneuver2.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  maneuver2.lane_following_maneuver.parameters.negotiation_type = carma_planning_msgs::msg::ManeuverParameters::PLATOONING; 
  maneuver2.lane_following_maneuver.lane_ids = {"1200"};
  maneuver2.lane_following_maneuver.start_dist = 14.98835712;
  maneuver2.lane_following_maneuver.start_speed = 6.7056;
  maneuver2.lane_following_maneuver.start_time = rclcpp::Time(4.4704*1e9);

  maneuver2.lane_following_maneuver.end_dist = 14.98835712 + 50.0;
  maneuver2.lane_following_maneuver.end_speed = 6.7056;
  maneuver2.lane_following_maneuver.end_time = rclcpp::Time((4.4704 + 7.45645430685) * 1e9);

  req.maneuver_plan.maneuvers.push_back(maneuver);
  req.maneuver_plan.maneuvers.push_back(maneuver2);

  carma_planning_msgs::srv::PlanTrajectory::Response resp;

  plugin.plan_trajectory_cb(req, resp);


}

TEST(PlatooningTacticalPluginTest, testPlanningCallbackexception)
{
  PlatooningTacticalPluginConfig config;
  config.default_downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  PlatooningTacticalPlugin plugin(wm, config,
    std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());

  auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);

  wm->setMap(map);
  carma_wm::test::setSpeedLimit(15_mph, wm);

  /**
   * Total route length should be 100m
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

  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

  carma_planning_msgs::srv::PlanTrajectory::Request req;
  req.vehicle_state.x_pos_global = 1.5;
  req.vehicle_state.y_pos_global = 5;
  req.vehicle_state.orientation = 0;
  req.vehicle_state.longitudinal_vel = 0.0;

  carma_planning_msgs::msg::Maneuver maneuver;
  maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  maneuver.lane_following_maneuver.lane_ids = {"1200"};
  maneuver.lane_following_maneuver.start_dist = 5.0;
  maneuver.lane_following_maneuver.start_time = rclcpp::Time(0.0*1e9);
  maneuver.lane_following_maneuver.start_speed = 6.7056;

  maneuver.lane_following_maneuver.end_dist = 14.98835712;
  maneuver.lane_following_maneuver.end_speed = 6.7056;
  maneuver.lane_following_maneuver.end_time = rclcpp::Time(4.4704*1e9);

  carma_planning_msgs::msg::Maneuver maneuver2;
  maneuver2.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
  maneuver2.lane_following_maneuver.lane_ids = {"1200"};
  maneuver2.lane_following_maneuver.start_dist = 14.98835712;
  maneuver2.lane_following_maneuver.start_speed = 6.7056;
  maneuver2.lane_following_maneuver.start_time = rclcpp::Time(4.4704*1e9);

  maneuver2.lane_following_maneuver.end_dist = 14.98835712 + 50.0;
  maneuver2.lane_following_maneuver.end_speed = 6.7056;
  maneuver2.lane_following_maneuver.end_time = rclcpp::Time((4.4704 + 7.45645430685)*1e9);

  req.maneuver_plan.maneuvers.push_back(maneuver);
  req.maneuver_plan.maneuvers.push_back(maneuver2);

  carma_planning_msgs::srv::PlanTrajectory::Response resp;

  // plugin.plan_trajectory_cb(req, resp);

  // EXPECT_THROW(plugin.plan_trajectory_cb(req, resp), std::invalid_argument);

}


}  // namespace platooning_tactical_plugin
