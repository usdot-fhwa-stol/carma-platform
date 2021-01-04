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

#include <inlanecruising_plugin/inlanecruising_plugin.h>
#include <carma_wm/WMTestLibForGuidance.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/Traits.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <carma_wm/MapConformer.h>
#include <carma_wm/CARMAWorldModel.h>
#include <ros/console.h>
#include <unsupported/Eigen/Splines>

typedef Eigen::Spline<float, 3> Spline3d;

using namespace lanelet::units::literals;
namespace inlanecruising_plugin
{


TEST(InLaneCruisingPluginTest, DISABLED_testPlanningCallback)
{
  InLaneCruisingPluginConfig config;
  config.downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

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

  cav_srvs::PlanTrajectoryRequest req;
  req.vehicle_state.X_pos_global = 1.5;
  req.vehicle_state.Y_pos_global = 5;
  req.vehicle_state.orientation = 0;
  req.vehicle_state.longitudinal_vel = 0.0;

  cav_msgs::Maneuver maneuver;
  maneuver.type = cav_msgs::Maneuver::LANE_FOLLOWING;
  maneuver.lane_following_maneuver.lane_id = 1200;
  maneuver.lane_following_maneuver.start_dist = 5.0;
  maneuver.lane_following_maneuver.start_time = ros::Time(0.0);
  maneuver.lane_following_maneuver.start_speed = 0.0;

  maneuver.lane_following_maneuver.end_dist = 14.98835712;
  maneuver.lane_following_maneuver.end_speed = 6.7056;
  maneuver.lane_following_maneuver.end_time = ros::Time(4.4704);

  cav_msgs::Maneuver maneuver2;
  maneuver2.type = cav_msgs::Maneuver::LANE_FOLLOWING;
  maneuver2.lane_following_maneuver.lane_id = 1200;
  maneuver2.lane_following_maneuver.start_dist = 14.98835712;
  maneuver2.lane_following_maneuver.start_speed = 6.7056;
  maneuver2.lane_following_maneuver.start_time = ros::Time(4.4704);

  maneuver2.lane_following_maneuver.end_dist = 14.98835712 + 50.0;
  maneuver2.lane_following_maneuver.end_speed = 6.7056;
  maneuver2.lane_following_maneuver.end_time = ros::Time(4.4704 + 7.45645430685);

  req.maneuver_plan.maneuvers.push_back(maneuver);
  req.maneuver_plan.maneuvers.push_back(maneuver2);

  cav_srvs::PlanTrajectoryResponse resp;

  plugin.plan_trajectory_cb(req, resp);

}

/*
Using this file:
    1) Set the file path to your OSM file
    2) Set the route_ids to the route you wish to test

    NOTE: The test is disabled by default. Enable it by removing the DISABLED_ prefix from the test name
*/

TEST(WaypointGeneratorTest, test_full_generation)
{
  
    
    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    // File location of osm file
    std::string file = "/workspaces/carma/AOI_1_TFHRC_faster_pretty.osm";    
    // The route ids that will form the route used
    std::vector<lanelet::Id> route_ids = { 130, 111, 110, 113, 135, 137, 170, 144, 143, 145, 140 };

    // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

    lanelet::MapConformer::ensureCompliance(map, 80_mph);

    InLaneCruisingPluginConfig config;
    config.lateral_accel_limit = 1.5;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    wm->setMap(map);

    InLaneCruisingPlugin inlc(wm, config, [&](auto msg) {});
    
    auto routing_graph = wm->getMapRoutingGraph();

    // Output graph for debugging
    //routing_graph->exportGraphViz("/workspaces/carma_ws/carma/src/carma-platform/waypoint_generator/routing.viz");

    carma_wm::test::setRouteByIds(route_ids, wm);

    auto p = wm->getMap()->laneletLayer.get(130).centerline()[3];
    ROS_WARN_STREAM("Start Point: " << p.x() << ", " << p.y());

    // -159.666, 521.683

  cav_srvs::PlanTrajectoryRequest req;

  req.vehicle_state.X_pos_global = -107;
  req.vehicle_state.Y_pos_global = 311.904;
  req.vehicle_state.orientation = -2.7570977;
  req.vehicle_state.longitudinal_vel = 0.0;

  cav_msgs::Maneuver maneuver;
  maneuver.type = cav_msgs::Maneuver::LANE_FOLLOWING;
  maneuver.lane_following_maneuver.lane_id = 110;
  maneuver.lane_following_maneuver.start_dist = 14.98835712 + 45+ 180;
  maneuver.lane_following_maneuver.start_time = ros::Time(0.0);
  maneuver.lane_following_maneuver.start_speed = 0.0;

  maneuver.lane_following_maneuver.end_dist = 14.98835712 + 50.0 + 45 + 200;
  maneuver.lane_following_maneuver.end_speed = 6.7056;
  maneuver.lane_following_maneuver.end_time = ros::Time(8);

  cav_msgs::Maneuver maneuver2;
  maneuver2.type = cav_msgs::Maneuver::LANE_FOLLOWING;
  maneuver2.lane_following_maneuver.lane_id = 110;
  maneuver2.lane_following_maneuver.start_dist = 14.98835712 + 45+ 202;
  maneuver2.lane_following_maneuver.start_speed = 6.7056;
  maneuver2.lane_following_maneuver.start_time = ros::Time(4.4704);

  maneuver2.lane_following_maneuver.end_dist = 14.98835712 + 50.0 + 45 + 250;
  maneuver2.lane_following_maneuver.end_speed = 6.7056;
  maneuver2.lane_following_maneuver.end_time = ros::Time(4.4704 + 7.45645430685 + 37.31);

  req.maneuver_plan.maneuvers.push_back(maneuver);
  req.maneuver_plan.maneuvers.push_back(maneuver2);

  cav_srvs::PlanTrajectoryResponse resp;

  inlc.plan_trajectory_cb(req, resp);

  for (auto pt: resp.trajectory_plan.trajectory_points)
  {
    std::cerr << "x:" << pt.x << "y: " << pt.y << "t: " << pt.target_time << std::endl;
  }
  
  

  /*
  std::vector<Eigen::VectorXf> waypoints;
  
  Eigen::Vector3f po1(2,3,4);
  Eigen::Vector3f po2(2,5,4);
  Eigen::Vector3f po3(2,8,9);
  Eigen::Vector3f po4(2,8,23);
  Eigen::Vector3f po5(2,3.5,25);
  Eigen::Vector3f po6(2,3,25);
  Eigen::Vector3f po7(2,2.5,26);
  Eigen::Vector3f po8(2,2.25,27);
  Eigen::Vector3f po9(2,2.0,28);
  Eigen::Vector3f po10(2,1.5,30);
  Eigen::Vector3f po11(2,1.0,32);
  Eigen::Vector3f po12(2,1.25,34);
  Eigen::Vector3f po13(2,2.0,35);
  Eigen::Vector3f po14(2,4.0,35);
  Eigen::Vector3f po15(2,5.0,35.5);
  Eigen::Vector3f po16(2,6.0,36);
  Eigen::Vector3f po17(2,7.0,50);
  Eigen::Vector3f po18(2,6.5,48);
  Eigen::Vector3f po19(2,4.0,43);
  
                        //x,    y
  // Eigen::Vector3f po1(2, 1    ,5);
  // Eigen::Vector3f po2(2,  2   ,5);
  // Eigen::Vector3f po3(2,  3   ,5);
  // Eigen::Vector3f po4(2,  4    ,5);
  // Eigen::Vector3f po5(2,  5     ,5);
  // Eigen::Vector3f po6(2,  10    ,5);
  // Eigen::Vector3f po7(2,  7     ,5);
  // Eigen::Vector3f po8(2,  8    ,5);
  // Eigen::Vector3f po9(2,  9     ,5);
  // Eigen::Vector3f po10(2, 10    ,5);
  // Eigen::Vector3f po11(2, 11    ,5);
  // Eigen::Vector3f po12(2, 12    ,5);
  // Eigen::Vector3f po13(2, 13 , 5);
  // Eigen::Vector3f po14(2, 14     ,5);
  // Eigen::Vector3f po15(2, 15    ,5);
  // Eigen::Vector3f po16(2, 16     ,5);
  // Eigen::Vector3f po17(2, 17     ,5);
  // Eigen::Vector3f po18(2, 18     ,5);
  // Eigen::Vector3f po19(2, 19     ,5);
// 
  /*
  waypoints.push_back(po1);
  waypoints.push_back(po2);
  waypoints.push_back(po3);
  waypoints.push_back(po4);
  waypoints.push_back(po5);
  waypoints.push_back(po6);
  waypoints.push_back(po7);
  waypoints.push_back(po8);
  waypoints.push_back(po9);
  waypoints.push_back(po10);
  waypoints.push_back(po11);
  waypoints.push_back(po12);
  waypoints.push_back(po13);
  waypoints.push_back(po14);
  waypoints.push_back(po15);
  waypoints.push_back(po16);
  waypoints.push_back(po17);
  waypoints.push_back(po18);
  waypoints.push_back(po19);


  // The degree of the interpolating spline needs to be one less than the number of points
  // that are fitted to the spline.
  Eigen::MatrixXf points(3, waypoints.size());
  int row_index = 0;
  for(auto const way_point : waypoints){
      points.col(row_index) << way_point[0], way_point[1], way_point[2];
      row_index++;
  }
  Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, 2);
  float time_ = 0;
  for(int i=0; i<100; i++){
      time_ += 1.0/(100*1.0);
      Eigen::VectorXf values = spline(time_);
      std::cout<< values << std::endl;
      std::cout<< "x:" << values.x() << "y:" << values.y() << "z:" << values.z() << std::endl;
  }
  */

}

};  // namespace inlanecruising_plugin

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::Time::init();
    ROSCONSOLE_AUTOINIT;
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
      ros::console::notifyLoggerLevelsChanged();
    }
    return RUN_ALL_TESTS();
}