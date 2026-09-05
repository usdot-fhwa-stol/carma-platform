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

#include <yield_plugin/yield_plugin.hpp>
#include <yield_plugin/yield_plugin_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <carma_wm/CARMAWorldModel.hpp>
#include <math.h>
#include <boost/property_tree/json_parser.hpp>
#include <carma_wm/WMTestLibForGuidance.hpp>
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
#include <carma_wm/MapConformer.hpp>
#include <unsupported/Eigen/Splines>
#include <tf2/LinearMath/Vector3.h>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_perception_msgs/msg/predicted_state.hpp>
#include <cmath>
#include <cstdint>
#include <limits>

using namespace yield_plugin;


TEST(YieldPluginTest, test_polynomial_calc)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[](const auto& msg) {}, [](const auto& msg) {});

  std::vector<double> coeff;
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);

  double result = plugin.polynomial_calc(coeff, 0);
  EXPECT_EQ(2, result);

  result = plugin.polynomial_calc(coeff, 1);
  EXPECT_EQ(12, result);

  result = plugin.polynomial_calc(coeff, 2);
  EXPECT_EQ(126, result);

  result = plugin.polynomial_calc(coeff, 3);
  EXPECT_EQ(728, result);
}

TEST(YieldPluginTest, test_polynomial_calc_derivative)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[](const auto& msg) {}, [](const auto& msg) {});

  std::vector<double> coeff;
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);

  double result = plugin.polynomial_calc_d(coeff, 0);
  EXPECT_EQ(2, result);

  result = plugin.polynomial_calc_d(coeff, 1);
  EXPECT_EQ(30, result);

  result = plugin.polynomial_calc_d(coeff, 2);
  EXPECT_EQ(258, result);

  result = plugin.polynomial_calc_d(coeff, 3);
  EXPECT_EQ(1094, result);
}

TEST(YieldPluginTest, MaxTrajectorySpeed)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[](const auto& msg) {}, [](const auto& msg) {});

  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory_points;

  rclcpp::Time startTime(1.0);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_1;
  point_1.x = 0.0;
  point_1.y = 0.0;
  point_1.target_time = startTime;
  point_1.lane_id = "1";
  trajectory_points.push_back(point_1);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_2;
  point_2.x = 5.0;
  point_2.y = 0.0;
  point_2.target_time = startTime + rclcpp::Duration::from_nanoseconds(1*1e9);
  point_2.lane_id = "1";
  trajectory_points.push_back(point_2);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_3;
  point_3.x = 10.0;
  point_3.y = 0.0;
  point_3.target_time = startTime + rclcpp::Duration::from_nanoseconds(2*1e9);
  point_3.lane_id = "1";
  trajectory_points.push_back(point_3);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_4;
  point_4.x = 15.0;
  point_4.y = 0.0;
  point_4.target_time = startTime + rclcpp::Duration::from_nanoseconds(3*1e9);
  point_4.lane_id = "1";
  trajectory_points.push_back(point_4);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_5;
  point_5.x = 20.0;
  point_5.y = 0.0;
  point_5.target_time = startTime + rclcpp::Duration::from_nanoseconds(4*1e9);
  point_5.lane_id = "1";
  trajectory_points.push_back(point_5);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_6;
  point_6.x = 25.0;
  point_6.y = 0.0;
  point_6.target_time = startTime + rclcpp::Duration::from_nanoseconds(5*1e9);
  point_6.lane_id = "1";
  trajectory_points.push_back(point_6);


 carma_planning_msgs::msg::TrajectoryPlanPoint point_7;
  point_7.x = 40.0;
  point_7.y = 0.0;
  point_7.target_time = startTime + rclcpp::Duration::from_nanoseconds(6*1e9);
  point_7.lane_id = "1";
  trajectory_points.push_back(point_7);

  double result = plugin.max_trajectory_speed(trajectory_points, rclcpp::Time(point_7.target_time).seconds());
  EXPECT_EQ(15, result);

  EXPECT_EQ(5, plugin.max_trajectory_speed(
    {trajectory_points.at(0), trajectory_points.at(1)},
    rclcpp::Time(point_2.target_time).seconds()));
  EXPECT_EQ(0, plugin.max_trajectory_speed({}, 0.0));
  EXPECT_EQ(0, plugin.max_trajectory_speed({point_1}, 0.0));

  point_2.target_time = point_1.target_time;
  EXPECT_EQ(0, plugin.max_trajectory_speed(
    {point_1, point_2}, rclcpp::Time(point_2.target_time).seconds()));
}

TEST(YieldPluginTest, CollisionCheckTrajectoryUsesObservedSpeed)
{
  YieldPluginConfig config;
  config.vehicle_acceleration_limit = 1.0;
  config.vehicle_deceleration_limit = 1.0;

  auto wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());
  YieldPlugin plugin(nh, wm, config, [](const auto& msg) {}, [](const auto& msg) {});

  carma_planning_msgs::msg::TrajectoryPlan upstream;
  upstream.header.stamp = rclcpp::Time(9'000'000'000LL);
  upstream.initial_longitudinal_velocity = 4.5;

  auto add_point = [&upstream](double y, int64_t time_ns) {
    carma_planning_msgs::msg::TrajectoryPlanPoint point;
    point.x = 10.0;
    point.y = y;
    point.target_time = rclcpp::Time(time_ns);
    upstream.trajectory_points.push_back(point);
  };

  add_point(0.0, 10'000'000'000LL);
  add_point(4.5, 11'000'000'000LL);
  add_point(9.0, 12'000'000'000LL);
  add_point(13.5, 13'000'000'000LL);

  const auto upstream_before = upstream;
  EXPECT_EQ(plugin.get_collision_check_trajectory(upstream, 4.5), upstream);

  const auto corrected = plugin.get_collision_check_trajectory(upstream, 2.0);
  ASSERT_EQ(corrected.trajectory_points.size(), upstream.trajectory_points.size());
  EXPECT_EQ(upstream, upstream_before);
  EXPECT_EQ(corrected.header, upstream.header);
  EXPECT_EQ(
    corrected.trajectory_points.front().target_time,
    upstream.trajectory_points.front().target_time);
  EXPECT_DOUBLE_EQ(corrected.initial_longitudinal_velocity, 2.0);

  const double first_corrected_dt =
    (rclcpp::Time(corrected.trajectory_points.at(1).target_time) -
     rclcpp::Time(corrected.trajectory_points.at(0).target_time)).seconds();

  // v1 = sqrt(v0^2 + 2ad) = sqrt(13), then dt = 2d / (v0 + v1).
  EXPECT_NEAR(first_corrected_dt, 9.0 / (2.0 + std::sqrt(13.0)), 1e-6);
  EXPECT_GT(
    rclcpp::Time(corrected.trajectory_points.at(1).target_time),
    rclcpp::Time(upstream.trajectory_points.at(1).target_time));

  const auto corrected_from_rest =
    plugin.get_collision_check_trajectory(upstream, 0.0);
  const double first_dt_from_rest =
    (rclcpp::Time(corrected_from_rest.trajectory_points.at(1).target_time) -
     rclcpp::Time(corrected_from_rest.trajectory_points.at(0).target_time)).seconds();
  EXPECT_NEAR(first_dt_from_rest, 3.0, 1e-6);

  double previous_speed = 2.0;
  for (size_t i = 0; i + 1 < corrected.trajectory_points.size(); ++i)
  {
    const auto& start = corrected.trajectory_points.at(i);
    const auto& end = corrected.trajectory_points.at(i + 1);
    const double distance = std::hypot(end.x - start.x, end.y - start.y);
    const double duration =
      (rclcpp::Time(end.target_time) - rclcpp::Time(start.target_time)).seconds();
    ASSERT_GT(duration, 0.0);
    const double next_speed = 2.0 * distance / duration - previous_speed;
    EXPECT_LE(
      next_speed * next_speed - previous_speed * previous_speed,
      2.0 * config.vehicle_acceleration_limit * distance + 1e-5);
    previous_speed = next_speed;
  }

  auto slower_upstream = upstream;
  slower_upstream.initial_longitudinal_velocity = 2.0;
  for (size_t i = 0; i < slower_upstream.trajectory_points.size(); ++i)
  {
    slower_upstream.trajectory_points.at(i).y = 2.0 * i;
  }

  const auto decelerating =
    plugin.get_collision_check_trajectory(slower_upstream, 4.5);
  const double first_decelerating_dt =
    (rclcpp::Time(decelerating.trajectory_points.at(1).target_time) -
     rclcpp::Time(decelerating.trajectory_points.at(0).target_time)).seconds();
  ASSERT_TRUE(std::isfinite(first_decelerating_dt));
  ASSERT_GT(first_decelerating_dt, 0.0);
  const double first_decelerating_speed =
    4.0 / first_decelerating_dt - 4.5;
  EXPECT_NEAR(first_decelerating_speed, std::sqrt(16.25), 1e-6);

  // Preserve a measured overspeed rather than rewriting the initial state to
  // satisfy an unreachable upstream stop. The reachable endpoint remains fast.
  auto emergency_stop_upstream = upstream;
  emergency_stop_upstream.initial_longitudinal_velocity = 2.0;
  emergency_stop_upstream.trajectory_points.resize(2);
  emergency_stop_upstream.trajectory_points.at(1).y = 1.0;

  const auto emergency_stop =
    plugin.get_collision_check_trajectory(emergency_stop_upstream, 10.0);
  EXPECT_DOUBLE_EQ(emergency_stop.initial_longitudinal_velocity, 10.0);
  const double emergency_stop_dt =
    (rclcpp::Time(emergency_stop.trajectory_points.at(1).target_time) -
     rclcpp::Time(emergency_stop.trajectory_points.at(0).target_time)).seconds();
  ASSERT_TRUE(std::isfinite(emergency_stop_dt));
  ASSERT_GT(emergency_stop_dt, 0.0);
  const double emergency_stop_endpoint_speed =
    2.0 / emergency_stop_dt - 10.0;
  EXPECT_NEAR(emergency_stop_endpoint_speed, std::sqrt(98.0), 1e-6);

  // A zero-speed mismatch that would produce an infinite segment duration
  // falls back to the unchanged upstream trajectory before conversion.
  EXPECT_EQ(
    plugin.get_collision_check_trajectory(emergency_stop_upstream, 0.0),
    emergency_stop_upstream);

  auto stopped_upstream = upstream;
  stopped_upstream.initial_longitudinal_velocity = 0.0;
  EXPECT_EQ(
    plugin.get_collision_check_trajectory(stopped_upstream, 0.0),
    stopped_upstream);
}

TEST(YieldPluginTest, CorrectedInitialSpeedRecoversCrossingCollision)
{
  auto wm = std::make_shared<carma_wm::CARMAWorldModel>();
  wm->setMap(carma_wm::test::buildGuidanceTestMap(100, 100));
  carma_wm::test::setRouteByIds({1200, 1201, 1202, 1203}, wm);

  YieldPluginConfig config;
  config.vehicle_acceleration_limit = 1.0;
  config.vehicle_deceleration_limit = 3.0;
  config.intervehicle_collision_distance_in_m = 0.05;
  config.collision_check_radius_in_m = 100.0;
  config.obstacle_zero_speed_threshold_in_ms = 0.01;
  config.minimum_safety_gap_in_meters = 0.1;
  config.vehicle_length = 0.1;

  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());
  YieldPlugin plugin(nh, wm, config, [](const auto& msg) {}, [](const auto& msg) {});
  plugin.update_route_llt_cache();

  carma_planning_msgs::msg::TrajectoryPlan upstream;
  upstream.initial_longitudinal_velocity = 4.5;

  auto add_ego_point = [&upstream](double y, int64_t time_ns) {
    carma_planning_msgs::msg::TrajectoryPlanPoint point;
    point.x = 10.0;
    point.y = y;
    point.target_time = rclcpp::Time(time_ns);
    upstream.trajectory_points.push_back(point);
  };

  add_ego_point(0.0, 10'000'000'000LL);
  add_ego_point(4.5, 11'000'000'000LL);
  add_ego_point(9.0, 12'000'000'000LL);
  add_ego_point(13.5, 13'000'000'000LL);

  const auto upstream_before = upstream;
  EXPECT_EQ(
    plugin.update_traj_for_object(upstream, {}, 4.5, 2.0),
    upstream_before);

  std::vector<carma_perception_msgs::msg::PredictedState> predictions;
  auto add_prediction = [&predictions](double x, int64_t time_ns) {
    carma_perception_msgs::msg::PredictedState state;
    state.header.stamp = rclcpp::Time(time_ns);
    state.predicted_position.position.x = x;
    state.predicted_position.position.y = 4.5;
    state.predicted_velocity.linear.x = 0.1;
    predictions.push_back(state);
  };

  // The obstacle crosses x=10 at t=11.6. The raw 4.5 m/s plan places
  // the ego vehicle at y=7.2, while the reachable plan places it at y~=4.48.
  add_prediction(9.9, 10'600'000'000LL);
  add_prediction(10.0, 11'600'000'000LL);
  add_prediction(10.1, 12'600'000'000LL);

  EXPECT_FALSE(plugin.get_collision(upstream, predictions, 0.05, 4.5).has_value());

  const auto corrected = plugin.get_collision_check_trajectory(upstream, 2.0);
  EXPECT_EQ(upstream, upstream_before);

  const double corrected_end =
    rclcpp::Time(corrected.trajectory_points.back().target_time).seconds();
  const double corrected_max_speed =
    plugin.max_trajectory_speed(corrected.trajectory_points, corrected_end);
  const auto collision =
    plugin.get_collision(corrected, predictions, 0.05, corrected_max_speed);

  ASSERT_TRUE(collision.has_value());
  EXPECT_NEAR(collision->collision_time.seconds(), 11.6, 1e-6);

  carma_perception_msgs::msg::ExternalObject obstacle;
  obstacle.id = 42;
  obstacle.header.stamp = predictions.front().header.stamp;
  obstacle.pose.pose.position = predictions.front().predicted_position.position;
  obstacle.velocity.twist.linear.x = 0.1;
  obstacle.predictions.assign(predictions.cbegin() + 1, predictions.cend());

  EXPECT_EQ(
    plugin.update_traj_for_object(upstream, {obstacle}, 4.5, 4.5),
    upstream_before);
  const auto yielded =
    plugin.update_traj_for_object(upstream, {obstacle}, 4.5, 2.0);
  EXPECT_NE(yielded, upstream_before);
  EXPECT_EQ(upstream, upstream_before);
}

TEST(YieldPluginTest, CollisionCheckTrajectoryRejectsMalformedInput)
{
  YieldPluginConfig config;
  auto wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());
  YieldPlugin plugin(nh, wm, config, [](const auto& msg) {}, [](const auto& msg) {});

  carma_planning_msgs::msg::TrajectoryPlan valid;
  valid.initial_longitudinal_velocity = 4.5;
  for (size_t i = 0; i < 3; ++i)
  {
    carma_planning_msgs::msg::TrajectoryPlanPoint point;
    point.x = 10.0;
    point.y = 4.5 * i;
    point.target_time =
      rclcpp::Time(static_cast<int64_t>((10.0 + i) * 1e9));
    valid.trajectory_points.push_back(point);
  }

  auto non_increasing_time = valid;
  non_increasing_time.trajectory_points.at(2).target_time =
    non_increasing_time.trajectory_points.at(1).target_time;
  EXPECT_EQ(
    plugin.get_collision_check_trajectory(non_increasing_time, 2.0),
    non_increasing_time);

  auto zero_distance = valid;
  zero_distance.trajectory_points.at(1).x =
    zero_distance.trajectory_points.at(0).x;
  zero_distance.trajectory_points.at(1).y =
    zero_distance.trajectory_points.at(0).y;
  EXPECT_EQ(
    plugin.get_collision_check_trajectory(zero_distance, 2.0),
    zero_distance);

  EXPECT_EQ(
    plugin.get_collision_check_trajectory(
      valid, std::numeric_limits<double>::quiet_NaN()),
    valid);
  EXPECT_EQ(plugin.get_collision_check_trajectory(valid, -1.0), valid);

  auto invalid_config = config;
  invalid_config.vehicle_acceleration_limit = 0.0;
  YieldPlugin invalid_config_plugin(
    nh, wm, invalid_config, [](const auto& msg) {}, [](const auto& msg) {});
  EXPECT_EQ(
    invalid_config_plugin.get_collision_check_trajectory(valid, 2.0),
    valid);
}

TEST(YieldPluginTest, CollisionCheckDoesNotExtrapolatePastEgoSegment)
{
  auto wm = std::make_shared<carma_wm::CARMAWorldModel>();
  wm->setMap(carma_wm::test::buildGuidanceTestMap(100, 100));
  carma_wm::test::setRouteByIds({1200, 1201, 1202, 1203}, wm);

  YieldPluginConfig config;
  config.intervehicle_collision_distance_in_m = 0.1;
  config.collision_check_radius_in_m = 100.0;

  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());
  YieldPlugin plugin(nh, wm, config, [](const auto& msg) {}, [](const auto& msg) {});
  plugin.update_route_llt_cache();

  carma_planning_msgs::msg::TrajectoryPlan ego;
  for (size_t i = 0; i < 2; ++i)
  {
    carma_planning_msgs::msg::TrajectoryPlanPoint point;
    point.x = 10.0;
    point.y = i;
    point.target_time =
      rclcpp::Time(static_cast<int64_t>((10.0 + i) * 1e9));
    ego.trajectory_points.push_back(point);
  }

  std::vector<carma_perception_msgs::msg::PredictedState> predictions;
  for (size_t i = 0; i < 2; ++i)
  {
    carma_perception_msgs::msg::PredictedState state;
    state.header.stamp =
      rclcpp::Time(static_cast<int64_t>((12.0 + i) * 1e9));
    state.predicted_position.position.x = 10.0;
    state.predicted_position.position.y = 2.0 + i;
    state.predicted_velocity.linear.y = 1.0;
    predictions.push_back(state);
  }

  // These positions lie on the infinite extension of the ego segment, but
  // occur after that segment ends and therefore cannot be a collision.
  EXPECT_FALSE(plugin.get_collision(ego, predictions, 0.1, 1.0).has_value());
}

TEST(YieldPluginTest, test_update_traj)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto map = carma_wm::test::buildGuidanceTestMap(100,100);

  wm->setMap(map);
  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

  YieldPluginConfig config;
  config.vehicle_length = 4;
  config.vehicle_width = 2;
  config.vehicle_height = 1;

  // std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[](const auto& msg) {}, [](const auto& msg) {});

  carma_perception_msgs::msg::ExternalObjectList rwol;
  carma_planning_msgs::msg::TrajectoryPlan tp;

  rclcpp::Time startTime(1.0);

  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_1;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_2;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_3;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_4;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_5;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_6;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_7;

  trajectory_point_1.x = 10.0;
  trajectory_point_1.y = 0.0001;
  trajectory_point_1.target_time = rclcpp::Time(0);

  trajectory_point_2.x = 10.0;
  trajectory_point_2.y = 20.0;
  trajectory_point_2.target_time = rclcpp::Time(1,0);

  trajectory_point_3.x = 10.0;
  trajectory_point_3.y = 30.0;
  trajectory_point_3.target_time = rclcpp::Time(2,0);

  trajectory_point_4.x = 10.0;
  trajectory_point_4.y = 40.0;
  trajectory_point_4.target_time = rclcpp::Time(3,0);

  trajectory_point_5.x = 10.0;
  trajectory_point_5.y = 50.0;
  trajectory_point_5.target_time = rclcpp::Time(4,0);

  trajectory_point_6.x = 10.0;
  trajectory_point_6.y = 60.0;
  trajectory_point_6.target_time = rclcpp::Time(5,0);

  trajectory_point_7.x = 10.0;
  trajectory_point_7.y = 70.0;
  trajectory_point_7.target_time = rclcpp::Time(6,0);

  tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};

  carma_perception_msgs::msg::ExternalObject rwo_1;

  tf2::Quaternion tf_orientation;
  tf_orientation.setRPY(0, 0, 1.5708);

  rwo_1.pose.pose.position.x = 60;
  rwo_1.pose.pose.position.y = 50;
  rwo_1.pose.pose.position.z = 0;

  rwo_1.pose.pose.orientation.x = tf_orientation.getX();
  rwo_1.pose.pose.orientation.y = tf_orientation.getY();
  rwo_1.pose.pose.orientation.z = tf_orientation.getZ();
  rwo_1.pose.pose.orientation.w = tf_orientation.getW();

  rwo_1.size.x = 1;
  rwo_1.size.y = 1;
  rwo_1.size.z = 1;

  carma_perception_msgs::msg::PredictedState ps_1;
  ps_1.header.stamp.sec = 1;

  ps_1.predicted_position.position.x = 10;
  ps_1.predicted_position.position.y = 10;
  ps_1.predicted_position.position.z = 0;

  ps_1.predicted_position.orientation.x = tf_orientation.getX();
  ps_1.predicted_position.orientation.y = tf_orientation.getY();
  ps_1.predicted_position.orientation.z = tf_orientation.getZ();
  ps_1.predicted_position.orientation.w = tf_orientation.getW();

  carma_perception_msgs::msg::PredictedState ps_2;
  ps_2.header.stamp.sec = 2;

  ps_2.predicted_position.position.x = 10;
  ps_2.predicted_position.position.y = 20;
  ps_2.predicted_position.position.z = 0;

  ps_2.predicted_position.orientation.x = tf_orientation.getX();
  ps_2.predicted_position.orientation.y = tf_orientation.getY();
  ps_2.predicted_position.orientation.z = tf_orientation.getZ();
  ps_2.predicted_position.orientation.w = tf_orientation.getW();

  carma_perception_msgs::msg::PredictedState ps_3;
  ps_3.header.stamp.sec = 3;

  ps_3.predicted_position.position.x = 10;
  ps_3.predicted_position.position.y = 30;
  ps_3.predicted_position.position.z = 0;

  ps_3.predicted_position.orientation.x = tf_orientation.getX();
  ps_3.predicted_position.orientation.y = tf_orientation.getY();
  ps_3.predicted_position.orientation.z = tf_orientation.getZ();
  ps_3.predicted_position.orientation.w = tf_orientation.getW();

  rwo_1.predictions = {ps_1,ps_2,ps_3};
  rwo_1.velocity.twist.linear.x = 10.0;

  rwol.objects = {rwo_1};

  std::vector<carma_perception_msgs::msg::ExternalObject> rw_objs;

  rw_objs.push_back(rwo_1);

  carma_planning_msgs::msg::TrajectoryPlan tp_new =
    plugin.update_traj_for_object(tp, rw_objs, 10.0);

  EXPECT_EQ(7, tp.trajectory_points.size());
}

TEST(YieldPluginTest, get_collision)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto map = carma_wm::test::buildGuidanceTestMap(100,100);

  wm->setMap(map);
  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

  YieldPluginConfig config;
  config.vehicle_length = 4;
  config.vehicle_width = 2;
  config.vehicle_height = 1;
  config.collision_check_radius_in_m = 90;

  // std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[](const auto& msg) {}, [](const auto& msg) {});

  carma_perception_msgs::msg::ExternalObjectList rwol;
  carma_planning_msgs::msg::TrajectoryPlan tp;

  EXPECT_EQ(
    plugin.update_traj_for_object(tp, {}, 0.0, 0.0),
    tp); //should return unchanged if received invalid trajectory

  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_1;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_2;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_3;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_4;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_5;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_6;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_7;

  trajectory_point_1.x = 10.0;
  trajectory_point_1.y = 0.0001;
  trajectory_point_1.target_time = rclcpp::Time(0);

  trajectory_point_2.x = 10.0;
  trajectory_point_2.y = 20.0;
  trajectory_point_2.target_time = rclcpp::Time(1,0);

  trajectory_point_3.x = 10.0;
  trajectory_point_3.y = 30.0;
  trajectory_point_3.target_time = rclcpp::Time(2,0);

  trajectory_point_4.x = 10.0;
  trajectory_point_4.y = 40.0;
  trajectory_point_4.target_time = rclcpp::Time(3,0);

  trajectory_point_5.x = 10.0;
  trajectory_point_5.y = 50.0;
  trajectory_point_5.target_time = rclcpp::Time(4,0);

  trajectory_point_6.x = 10.0;
  trajectory_point_6.y = 60.0;
  trajectory_point_6.target_time = rclcpp::Time(5,0);

  trajectory_point_7.x = 10.0;
  trajectory_point_7.y = 70.0;
  trajectory_point_7.target_time = rclcpp::Time(6,0);

  tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};

  carma_perception_msgs::msg::ExternalObject rwo_1;

  // Set route but also test no throw
  EXPECT_NO_THROW(plugin.update_traj_for_object(tp, {}, 0.0, 0.0));

  // ON ROUTE, BUT NO COLLISION DUE TO BEING AHEAD

  tf2::Quaternion tf_orientation;
  tf_orientation.setRPY(0, 0, 1.5708);

  rwo_1.pose.pose.position.x = 60;
  rwo_1.pose.pose.position.y = 50;
  rwo_1.pose.pose.position.z = 0;

  rwo_1.pose.pose.orientation.x = tf_orientation.getX();
  rwo_1.pose.pose.orientation.y = tf_orientation.getY();
  rwo_1.pose.pose.orientation.z = tf_orientation.getZ();
  rwo_1.pose.pose.orientation.w = tf_orientation.getW();

  rwo_1.size.x = 1;
  rwo_1.size.y = 1;
  rwo_1.size.z = 1;

  carma_perception_msgs::msg::PredictedState ps_1;
  ps_1.header.stamp.sec = 1;

  ps_1.predicted_position.position.x = 10;
  ps_1.predicted_position.position.y = 10;
  ps_1.predicted_position.position.z = 0;

  ps_1.predicted_position.orientation.x = tf_orientation.getX();
  ps_1.predicted_position.orientation.y = tf_orientation.getY();
  ps_1.predicted_position.orientation.z = tf_orientation.getZ();
  ps_1.predicted_position.orientation.w = tf_orientation.getW();

  carma_perception_msgs::msg::PredictedState ps_2;
  ps_2.header.stamp.sec = 2;

  ps_2.predicted_position.position.x = 10;
  ps_2.predicted_position.position.y = 20;
  ps_2.predicted_position.position.z = 0;

  ps_2.predicted_position.orientation.x = tf_orientation.getX();
  ps_2.predicted_position.orientation.y = tf_orientation.getY();
  ps_2.predicted_position.orientation.z = tf_orientation.getZ();
  ps_2.predicted_position.orientation.w = tf_orientation.getW();

  carma_perception_msgs::msg::PredictedState ps_3;
  ps_3.header.stamp.sec = 3;

  ps_3.predicted_position.position.x = 10;
  ps_3.predicted_position.position.y = 30;
  ps_3.predicted_position.position.z = 0;

  ps_3.predicted_position.orientation.x = tf_orientation.getX();
  ps_3.predicted_position.orientation.y = tf_orientation.getY();
  ps_3.predicted_position.orientation.z = tf_orientation.getZ();
  ps_3.predicted_position.orientation.w = tf_orientation.getW();

  rwo_1.predictions = {ps_1,ps_2,ps_3};
  rwo_1.velocity.twist.linear.x = 10.0;

  auto collision_result = plugin.get_collision(tp, rwo_1.predictions, 6, 10);

  ASSERT_TRUE(collision_result == std::nullopt);

  // DETECT COLLISION

  ps_1.header.stamp.sec = 1;

  ps_1.predicted_position.position.x = 10;
  ps_1.predicted_position.position.y = 30;
  ps_1.predicted_position.position.z = 0;
  ps_1.predicted_velocity.linear.x = 0;
  ps_1.predicted_velocity.linear.y = 1;

  ps_2.header.stamp.sec = 2;

  ps_2.predicted_position.position.x = 10;
  ps_2.predicted_position.position.y = 31;
  ps_2.predicted_position.position.z = 0;
  ps_2.predicted_velocity = ps_1.predicted_velocity;

  ps_3.header.stamp.sec = 3;

  ps_3.predicted_position.position.x = 10;
  ps_3.predicted_position.position.y = 32;
  ps_3.predicted_position.position.z = 0;
  ps_3.predicted_velocity = ps_1.predicted_velocity;

  rwo_1.predictions = {ps_1,ps_2,ps_3};

  collision_result = plugin.get_collision(tp, rwo_1.predictions, 6, 10);
  ASSERT_TRUE(collision_result != std::nullopt);
  ASSERT_TRUE(collision_result.value().collision_time == rclcpp::Time(2, 0, collision_result.value().collision_time.get_clock_type()));

  // STATES ARE NOT ON ROUTE
  ps_1.header.stamp.sec = 1;

  ps_1.predicted_position.position.x = 20;
  ps_1.predicted_position.position.y = 30;
  ps_1.predicted_position.position.z = 0;

  ps_2.header.stamp.sec = 2;

  ps_2.predicted_position.position.x = 20;
  ps_2.predicted_position.position.y = 31;
  ps_2.predicted_position.position.z = 0;

  rwo_1.predictions = {ps_1,ps_2};

  collision_result = plugin.get_collision(tp, rwo_1.predictions, 6, 10);
  ASSERT_TRUE(collision_result == std::nullopt);

  // STATES ARE ON THE ROUTE, BUT TOO FAR AWAY
  ps_1.header.stamp.sec = 1;

  ps_1.predicted_position.position.x = 10;
  ps_1.predicted_position.position.y = 95;
  ps_1.predicted_position.position.z = 0;

  ps_2.header.stamp.sec = 2;

  ps_2.predicted_position.position.x = 10;
  ps_2.predicted_position.position.y = 96;
  ps_2.predicted_position.position.z = 0;

  rwo_1.predictions = {ps_1,ps_2};

  collision_result = plugin.get_collision(tp, rwo_1.predictions, 6, 10);
  ASSERT_TRUE(collision_result == std::nullopt);

  // STATES ARE ON THE ROUTE, BUT ALREADY PASSED
  // HOWEVER, AS A NAIVE TEST, IT SHOULD STILL RETURN COLLISION (can be removed if algorithm gets improved)
  ps_1.header.stamp.sec = 2;

  ps_1.predicted_position.position.x = 10;
  ps_1.predicted_position.position.y = 25;
  ps_1.predicted_position.position.z = 0;
  ps_1.predicted_velocity.linear.x = 0;
  ps_1.predicted_velocity.linear.y = 10;

  ps_2.header.stamp.sec = 3;

  ps_2.predicted_position.position.x = 10;
  ps_2.predicted_position.position.y = 35;
  ps_2.predicted_position.position.z = 0;
  ps_2.predicted_velocity = ps_1.predicted_velocity;

  rwo_1.predictions = {ps_1,ps_2};

  collision_result = plugin.get_collision(tp, rwo_1.predictions, 6, 10);
  ASSERT_TRUE(collision_result != std::nullopt);
}

TEST(YieldPluginTest, is_object_behind_vehicle)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto map = carma_wm::test::buildGuidanceTestMap(100,100);

  wm->setMap(map);
  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

  YieldPluginConfig config;
  config.vehicle_length = 4;
  config.vehicle_width = 2;
  config.vehicle_height = 1;
  config.collision_check_radius_in_m = 90;

  // std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[](const auto& msg) {}, [](const auto& msg) {});

  carma_perception_msgs::msg::ExternalObjectList rwol;
  carma_planning_msgs::msg::TrajectoryPlan tp;

  EXPECT_EQ(
    plugin.update_traj_for_object(tp, {}, 0.0, 0.0),
    tp); //should return unchanged if received invalid trajectory

  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_1;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_2;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_3;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_4;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_5;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_6;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_7;

  trajectory_point_1.x = 10.0;
  trajectory_point_1.y = 0.0001;
  trajectory_point_1.target_time = rclcpp::Time(0);

  trajectory_point_2.x = 10.0;
  trajectory_point_2.y = 20.0;
  trajectory_point_2.target_time = rclcpp::Time(1,0);

  trajectory_point_3.x = 10.0;
  trajectory_point_3.y = 30.0;
  trajectory_point_3.target_time = rclcpp::Time(2,0);

  trajectory_point_4.x = 10.0;
  trajectory_point_4.y = 40.0;
  trajectory_point_4.target_time = rclcpp::Time(3,0);

  trajectory_point_5.x = 10.0;
  trajectory_point_5.y = 50.0;
  trajectory_point_5.target_time = rclcpp::Time(4,0);

  trajectory_point_6.x = 10.0;
  trajectory_point_6.y = 60.0;
  trajectory_point_6.target_time = rclcpp::Time(5,0);

  trajectory_point_7.x = 10.0;
  trajectory_point_7.y = 70.0;
  trajectory_point_7.target_time = rclcpp::Time(6,0);

  tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};

  carma_perception_msgs::msg::ExternalObject rwo_1;

  // Set route but also test no throw
  EXPECT_NO_THROW(plugin.update_traj_for_object(tp, {}, 0.0, 0.0));
  // not actually passed
  ASSERT_FALSE(plugin.is_object_behind_vehicle(rwo_1.id, rclcpp::Time(9,0), 12.0, 11.0)); //dummy time and id

  // passed but needs confirmation
  ASSERT_FALSE(plugin.is_object_behind_vehicle(rwo_1.id, rclcpp::Time(9,0), 14.0, 11.0));
  ASSERT_FALSE(plugin.is_object_behind_vehicle(rwo_1.id, rclcpp::Time(9,0), 14.0, 11.0));
  ASSERT_FALSE(plugin.is_object_behind_vehicle(rwo_1.id, rclcpp::Time(9,0), 14.0, 11.0));
  ASSERT_FALSE(plugin.is_object_behind_vehicle(rwo_1.id, rclcpp::Time(9,0), 14.0, 11.0));
  ASSERT_TRUE(plugin.is_object_behind_vehicle(rwo_1.id, rclcpp::Time(9,0), 14.0, 11.0)); //need consecutive 5 times required to confirm vehicle passed
}

TEST(YieldPluginTest, test_update_traj2)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[](const auto& msg) {}, [](const auto& msg) {});

  carma_planning_msgs::msg::TrajectoryPlan original_tp;

  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_1;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_2;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_3;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_4;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_5;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_6;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_7;

  trajectory_point_1.x = 1.0;
  trajectory_point_1.y = 1.0;
  trajectory_point_1.target_time = rclcpp::Time(0);

  trajectory_point_2.x = 5.0;
  trajectory_point_2.y = 1.0;
  trajectory_point_2.target_time = rclcpp::Time(1,0);

  trajectory_point_3.x = 10.0;
  trajectory_point_3.y = 1.0;
  trajectory_point_3.target_time = rclcpp::Time(2,0);

  trajectory_point_4.x = 15.0;
  trajectory_point_4.y = 1.0;
  trajectory_point_4.target_time = rclcpp::Time(3,0);

  trajectory_point_5.x = 20.0;
  trajectory_point_5.y = 1.0;
  trajectory_point_5.target_time = rclcpp::Time(4,0);

  trajectory_point_6.x = 25.0;
  trajectory_point_6.y = 1.0;
  trajectory_point_6.target_time = rclcpp::Time(5,0);

  trajectory_point_7.x = 30.0;
  trajectory_point_7.y = 1.0;
  trajectory_point_7.target_time = rclcpp::Time(6,0);

  original_tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};


  double initial_pos = 0.0;
  double goal_pos = 35.0;
  double current_speed_ = 10.0;
  double goal_velocity = 5.0;
  double initial_accel = 0.0;
  double goal_accel = 0.0;
  double initial_time = 0.0;
  double tp = 5;

  std::vector<double> values = quintic_coefficient_calculator::quintic_coefficient_calculator(initial_pos,
                                                                                              goal_pos,
                                                                                              current_speed_,
                                                                                              goal_velocity,
                                                                                              initial_accel,
                                                                                              goal_accel,
                                                                                              initial_time,
                                                                                              tp);

  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> new_trajectory_points;
  new_trajectory_points.push_back(original_tp.trajectory_points[0]);

  std::vector<double> original_traj_downtracks = plugin.get_relative_downtracks(original_tp);

  std::vector<double> new_speeds;
  std::vector<double> distance_travelled;


  for(size_t i = 1; i < original_tp.trajectory_points.size() ; i++ )
  {
      double traj_target_time = i * tp / original_tp.trajectory_points.size();
      double dt_dist = plugin.polynomial_calc(values, traj_target_time);
      distance_travelled.push_back(dt_dist);
      double dv = plugin.polynomial_calc_d(values, traj_target_time);
      new_speeds.push_back(dv);
      carma_planning_msgs::msg::TrajectoryPlanPoint new_tpp;
      new_tpp.x = original_tp.trajectory_points[i].x;
      new_tpp.y = original_tp.trajectory_points[i].y;
      new_tpp.target_time = rclcpp::Time(new_trajectory_points[0].target_time) + rclcpp::Duration::from_nanoseconds((original_traj_downtracks[i]/dv)*1e9);
      new_trajectory_points.push_back(new_tpp);
  }

  // speeds are decreasing
  EXPECT_EQ(6, new_speeds.size());
  EXPECT_TRUE(new_speeds[5] <= new_speeds[4]);
  EXPECT_TRUE(new_speeds[3] <= new_speeds[1]);


  EXPECT_EQ(6, distance_travelled.size());
  // did not exceed goal position
  EXPECT_TRUE(distance_travelled[5] < goal_pos );
}


TEST(YieldPluginTest, test_update_traj_stop)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[](const auto& msg) {}, [](const auto& msg) {});

  carma_planning_msgs::msg::TrajectoryPlan original_tp;

  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_1;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_2;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_3;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_4;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_5;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_6;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_7;

  trajectory_point_1.x = 1.0;
  trajectory_point_1.y = 1.0;
  trajectory_point_1.target_time = rclcpp::Time(0);

  trajectory_point_2.x = 5.0;
  trajectory_point_2.y = 1.0;
  trajectory_point_2.target_time = rclcpp::Time(1,0);

  trajectory_point_3.x = 10.0;
  trajectory_point_3.y = 1.0;
  trajectory_point_3.target_time = rclcpp::Time(2,0);

  trajectory_point_4.x = 15.0;
  trajectory_point_4.y = 1.0;
  trajectory_point_4.target_time = rclcpp::Time(3,0);

  trajectory_point_5.x = 20.0;
  trajectory_point_5.y = 1.0;
  trajectory_point_5.target_time = rclcpp::Time(4,0);

  trajectory_point_6.x = 25.0;
  trajectory_point_6.y = 1.0;
  trajectory_point_6.target_time = rclcpp::Time(5,0);

  trajectory_point_7.x = 30.0;
  trajectory_point_7.y = 1.0;
  trajectory_point_7.target_time = rclcpp::Time(6,0);

  original_tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};



  // When the lead vehicle is stopped

  double initial_pos = 0.0;
  double goal_pos = 15.0;
  double current_speed_ = 10.0;
  double goal_velocity = 0.0;
  double initial_accel = 0.0;
  double goal_accel = 0.0;
  double initial_time = 0.0;
  double tp = 5;

  std::vector<double> values = quintic_coefficient_calculator::quintic_coefficient_calculator(initial_pos,
                                                                                              goal_pos,
                                                                                              current_speed_,
                                                                                              goal_velocity,
                                                                                              initial_accel,
                                                                                              goal_accel,
                                                                                              initial_time,
                                                                                              tp);

  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> new_trajectory_points;
  new_trajectory_points.push_back(original_tp.trajectory_points[0]);
  std::vector<double> original_traj_downtracks = plugin.get_relative_downtracks(original_tp);

  std::vector<double> new_speeds;
  std::vector<double> distance_travelled;
  for(size_t i = 1; i < original_tp.trajectory_points.size() ; i++ )
  {
      double traj_target_time = i * tp / original_tp.trajectory_points.size();
      double dt_dist = plugin.polynomial_calc(values, traj_target_time);
      double dv = plugin.polynomial_calc_d(values, traj_target_time);
      carma_planning_msgs::msg::TrajectoryPlanPoint new_tpp;

      if (dv >= 1.0)
        {
          RCLCPP_WARN(rclcpp::get_logger("yield_plugin"),"target speed is positive");
          if (dv >= current_speed_){
            dv = current_speed_;
          }
          // trajectory point is copied to move all the available information, then its target time is updated
          new_tpp = original_tp.trajectory_points[i];
          new_tpp.target_time = rclcpp::Time(new_trajectory_points[i-1].target_time) + rclcpp::Duration::from_nanoseconds((original_traj_downtracks[i]/dv)*1e9);
          new_trajectory_points.push_back(new_tpp);
        }
        else
        {
          RCLCPP_WARN(rclcpp::get_logger("yield_plugin"),"target speed is zero");
          new_tpp = new_trajectory_points[i-1];
          new_tpp.target_time = rclcpp::Time(new_trajectory_points[0].target_time) + rclcpp::Duration::from_nanoseconds(traj_target_time*1e9);
          new_trajectory_points.push_back(new_tpp);
        }
      new_speeds.push_back(dv);
  }


  EXPECT_EQ(original_tp.trajectory_points.size(), new_trajectory_points.size());
  // Trajectory point location same as previous point
  EXPECT_EQ(new_trajectory_points[5].x, new_trajectory_points[4].x);
  // Trajectory point time is greater than previous point
  EXPECT_TRUE(rclcpp::Time(new_trajectory_points[5].target_time) > rclcpp::Time(new_trajectory_points[4].target_time));
}

TEST(YieldPluginTest, jmt_traj)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[](const auto& msg) {}, [](const auto& msg) {});

  carma_planning_msgs::msg::TrajectoryPlan original_tp;

  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_1;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_2;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_3;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_4;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_5;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_6;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_7;

  trajectory_point_1.x = 1.0;
  trajectory_point_1.y = 1.0;
  trajectory_point_1.target_time = rclcpp::Time(0);

  trajectory_point_2.x = 5.0;
  trajectory_point_2.y = 1.0;
  trajectory_point_2.target_time = rclcpp::Time(1,0);

  trajectory_point_3.x = 10.0;
  trajectory_point_3.y = 1.0;
  trajectory_point_3.target_time = rclcpp::Time(2,0);

  trajectory_point_4.x = 15.0;
  trajectory_point_4.y = 1.0;
  trajectory_point_4.target_time = rclcpp::Time(3,0);

  trajectory_point_5.x = 20.0;
  trajectory_point_5.y = 1.0;
  trajectory_point_5.target_time = rclcpp::Time(4,0);

  trajectory_point_6.x = 25.0;
  trajectory_point_6.y = 1.0;
  trajectory_point_6.target_time = rclcpp::Time(5,0);

  trajectory_point_7.x = 30.0;
  trajectory_point_7.y = 1.0;
  trajectory_point_7.target_time = rclcpp::Time(6,0);

  original_tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};


  // When the lead vehicle is stopped

  double initial_pos = 0.0;
  double goal_pos = 35.0;
  double initial_velocity = 10.0;
  double goal_velocity = 5.0;
  double initial_accel = 0.0;
  double goal_accel = 0.0;
  double initial_time = 0.0;
  double tp = 5;

  carma_planning_msgs::msg::TrajectoryPlan jmt_traj = plugin.generate_JMT_trajectory(original_tp, initial_pos, goal_pos, initial_velocity, goal_velocity, tp, rclcpp::Time(original_tp.trajectory_points.back().target_time).seconds());

  EXPECT_EQ(jmt_traj.trajectory_points.size(), original_tp.trajectory_points.size());
  EXPECT_LE(rclcpp::Time(jmt_traj.trajectory_points[2].target_time), rclcpp::Time(original_tp.trajectory_points[2].target_time));

  initial_pos = 9.5;
  goal_pos = 35.0;
  initial_velocity = 9.0;
  goal_velocity = 5.0;
  initial_accel = -1.0;
  goal_accel = 0.0;
  initial_time = 1.0;
  tp = 5;

  jmt_traj = plugin.generate_JMT_trajectory(original_tp, initial_pos, goal_pos, initial_velocity, goal_velocity, tp, rclcpp::Time(original_tp.trajectory_points.back().target_time).seconds());
  // New trajectory's points should have future target_time as it slows down further
  EXPECT_EQ(jmt_traj.trajectory_points.size(), original_tp.trajectory_points.size());
  EXPECT_LE(rclcpp::Time(original_tp.trajectory_points[2].target_time).seconds(), rclcpp::Time(jmt_traj.trajectory_points[2].target_time).seconds());
  EXPECT_LE(rclcpp::Time(original_tp.trajectory_points[4].target_time).seconds(), rclcpp::Time(jmt_traj.trajectory_points[4].target_time).seconds());
  EXPECT_LE(rclcpp::Time(original_tp.trajectory_points[5].target_time).seconds(), rclcpp::Time(jmt_traj.trajectory_points[5].target_time).seconds());
}

TEST(YieldPluginTest, min_digital_gap)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto map = carma_wm::test::buildGuidanceTestMap(3,3);

  auto ll_1200 = map->laneletLayer.get(1200);

  double min_gap = 12;

  auto regulatory_element = std::make_shared<lanelet::DigitalMinimumGap>(lanelet::DigitalMinimumGap::buildData(lanelet::utils::getId(), min_gap, {ll_1200}, { },
                                                     { lanelet::Participants::VehicleCar }));
  ll_1200.addRegulatoryElement(regulatory_element);
  map->add(regulatory_element);
  wm->setMap(map);
  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

  YieldPluginConfig config;
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[](const auto& msg) {}, [](const auto& msg) {});

  carma_planning_msgs::msg::TrajectoryPlan original_tp;

  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_1;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_2;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_3;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_4;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_5;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_6;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_7;

  trajectory_point_1.x = 1.0;
  trajectory_point_1.y = 1.0;
  trajectory_point_1.target_time = rclcpp::Time(0);

  trajectory_point_2.x = 1.0;
  trajectory_point_2.y = 1.5;
  trajectory_point_2.target_time = rclcpp::Time(1,0);

  trajectory_point_3.x = 1.0;
  trajectory_point_3.y = 2.0;
  trajectory_point_3.target_time = rclcpp::Time(2,0);

  trajectory_point_4.x = 1.0;
  trajectory_point_4.y = 2.5;
  trajectory_point_4.target_time = rclcpp::Time(3,0);

  original_tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4};

  double gap = plugin.check_traj_for_digital_min_gap(original_tp);

  EXPECT_EQ(gap, min_gap);

}
