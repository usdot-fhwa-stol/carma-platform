/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include "stop_and_wait_plugin.h"
#include "stop_and_wait_config.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <carma_wm/WMTestLibForGuidance.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_core/geometry/LineString.h>
#include <string>
#include <cav_msgs/Maneuver.h>
#include <carma_utils/containers/containers.h>
#include <cav_msgs/VehicleState.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <fstream>
#include <cav_srvs/PlanTrajectory.h>
#include <sstream>
#include <ros/package.h>
#include <unordered_set>

namespace stop_and_wait_plugin
{
TEST(StopandWait, TestStopandWaitPlanning)
{
  ros::Time::setNow(ros::Time(0.0));
  StopandWaitConfig config;

  config.minimal_trajectory_duration = 6.0;    // Trajectory length in seconds
  config.stop_timestep = 0.1;                  // Size of timesteps between stopped trajectory points
  config.trajectory_step_size = 1;                  // Amount to downsample input lanelet centerline data.
  config.accel_limit_multiplier = 0.5;         // Multiplier to compine with actual accel limit for target planning
  config.accel_limit = 2.0;                    // Longitudinal acceleration limit of the vehicle

  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  StopandWait plugin(wm, config, [&](auto msg) {});

  auto map = carma_wm::test::buildGuidanceTestMap(3.7, 20);

  wm->setMap(map);
  carma_wm::test::setSpeedLimit(20_mph, wm);

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

  cav_srvs::PlanTrajectoryRequest req;
  req.vehicle_state.x_pos_global = 1.5;
  req.vehicle_state.y_pos_global = 5;
  req.vehicle_state.orientation = 3.14/2;
  req.vehicle_state.longitudinal_vel = 8.9408; // 20 mph
  req.header.stamp = ros::Time(0.0);

  cav_msgs::Maneuver maneuver;
  maneuver.type = cav_msgs::Maneuver::STOP_AND_WAIT;
  maneuver.stop_and_wait_maneuver.start_dist = 5.0;
  maneuver.stop_and_wait_maneuver.start_time = ros::Time(0.0);
  maneuver.stop_and_wait_maneuver.start_speed = 8.9408;

  maneuver.stop_and_wait_maneuver.end_dist = 55;
  maneuver.stop_and_wait_maneuver.end_time = ros::Time(11.175999999999998);

  req.maneuver_plan.maneuvers.push_back(maneuver);
  req.maneuver_index_to_plan = 0;

  cav_srvs::PlanTrajectoryResponse resp;
  plugin.plan_trajectory_cb(req, resp);

  double dist = 0;
  double vel = resp.trajectory_plan.initial_longitudinal_velocity;
  bool first= true;
  lanelet::BasicPoint2d prev_point;
  ros::Time prev_time = ros::Time(0.0);
  for (auto point : resp.trajectory_plan.trajectory_points) {
    lanelet::BasicPoint2d p(point.x, point.y);
    ROS_INFO_STREAM("Y: " << point.y);
    double delta = 0;
    if (first) {
      first = false;
    } else {
      delta = lanelet::geometry::distance2d(p, prev_point);
      dist += delta;
      ROS_INFO_STREAM("delta: " << delta << " timediff: " << (point.target_time - prev_time).toSec() << "pre_vel: " << vel);
      vel = (2.0 * delta / (point.target_time - prev_time).toSec()) - vel;
    }
    ROS_ERROR_STREAM("point time: " << point.target_time.toSec() << " dist: " << dist << " vel: " << vel);
    prev_point = p;
    prev_time = point.target_time;
  }
}
}  // namespace stop_and_wait_plugin

// Run all the tests
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  ROSCONSOLE_AUTOINIT;
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) { // Change to Debug to enable debug logs
    ros::console::notifyLoggerLevelsChanged();
  }
  auto res = RUN_ALL_TESTS();
  
  return res;
}