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
#include <carma_utils/containers/containers.h>
#include <tf/LinearMath/Vector3.h>


#include <ros/ros.h>
#include <string>
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <sstream>
#include <carma_utils/containers/containers.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <inlanecruising_plugin/smoothing/SplineI.h>
#include <inlanecruising_plugin/smoothing/BSpline.h>
#include <inlanecruising_plugin/inlanecruising_plugin.h>
#include <inlanecruising_plugin/log/log.h>
#include <inlanecruising_plugin/smoothing/filters.h>
#include <unordered_set>


typedef Eigen::Spline<double, 2> Spline2d;

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

TEST(WaypointGeneratorTest, DISABLED_test_full_generation)
{
  
 int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;

  // File location of osm file
  std::string file = "/workspaces/carma/AOI_1_TFHRC_faster_pretty.osm";    
  // The route ids that will form the route used
  std::vector<lanelet::Id> route_ids = { 130, 111, 110, 113, 135, 138 };

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


}

TEST(WaypointGeneratorTest, DISABLED_test_compute_fit_full_generation)
{
  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;

  // File location of osm file
  std::string file = "/workspaces/carma/AOI_1_TFHRC_faster_pretty.osm";    
  // The route ids that will form the route used
  std::vector<lanelet::Id> route_ids = { 130, 129 };

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

  carma_wm::test::setRouteByIds(route_ids, wm);

  auto route = wm->getRoute()->shortestPath();
  std::vector<lanelet::ConstLanelet> lanelets;
  for (auto llt : route)
  {
    lanelets.push_back(llt);
  }
  // Get centerline
  lanelet::BasicLineString2d route_geometry = carma_wm::geometry::concatenate_lanelets(lanelets);
  
  // Downsample
  std::vector<lanelet::BasicPoint2d> downsampled_points;
  downsampled_points.reserve((route_geometry.size() / config.downsample_ratio) + 1);

  for (int i = 0; i < route_geometry.size(); i += config.downsample_ratio)
  {
    downsampled_points.push_back(route_geometry[i]);
    // Uncomment to print and check if this original map matches with the generated one below 
    // ROS_INFO_STREAM("Original point: x: " << route_geometry[i].x() << "y: " << route_geometry[i].y());
  }

  std::unique_ptr<smoothing::SplineI> fit_curve = inlc.compute_fit(downsampled_points);
  
  std::vector<lanelet::BasicPoint2d> spline_points;

  // Following logic is written for BSpline library. Switch with appropriate call of the new library if different.
  double parameter = 0.0;
  for(int i=0; i< downsampled_points.size(); i++){
    lanelet::BasicPoint2d pt = (*fit_curve)(parameter);
    // Uncomment to print and check if this generated map matches with the original one above 
    // ROS_INFO_STREAM("BSpline point: x: " << values.x() << "y: " << values.y());
    spline_points.push_back(pt);
    parameter += 1.0/(downsampled_points.size()*1.0);
  }

  ASSERT_EQ(spline_points.size(), downsampled_points.size());
  int error_count = 0;
  
  for(int i = 1; i < downsampled_points.size(); i ++)
  {
    // tag as erroneous if directions of generated points are not within 5 degree of those of original points
    tf::Vector3 original_vector(downsampled_points[i].x() - downsampled_points[i-1].x(), 
                      downsampled_points[i].y() - downsampled_points[i-1].y(), 0);
    original_vector.setZ(0);
    tf::Vector3 spline_vector(spline_points[i].x() - spline_points[i-1].x(), 
                      spline_points[i].y() - spline_points[i-1].y(), 0);
    spline_vector.setZ(0);
    double angle_in_rad = std::fabs(tf::tfAngle(original_vector, spline_vector));
    if (angle_in_rad > 0.08)  error_count ++;
  }

  // We say it is passing if there is less than 10% error in total number of points
  ROS_INFO_STREAM("Total points above 5 degree difference in their direction:" << error_count 
    << ", which is " << (double)error_count/(double)downsampled_points.size()*100 << "% of total");
  ASSERT_TRUE((double)error_count/(double)downsampled_points.size() < 0.1); 
}


TEST(WaypointGeneratorTest, DISABLED_test_compute_fit_full_generation_fraction)
{
  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;

  // File location of osm file
  std::string file = "/workspaces/carma/AOI_1_TFHRC_faster_pretty.osm";    
  // The route ids that will form the route used
  std::vector<lanelet::Id> route_ids = { 130, 129 };

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

  // INCLUDE POINTS HERE
  lanelet::BasicPoint2d pt1{-183.704,	478.671 }; 
  lanelet::BasicPoint2d pt2{-186.631,	477.492 }; 
  lanelet::BasicPoint2d pt3{-189.386,	476.233 }; 
  lanelet::BasicPoint2d pt4{-192.147,	474.967 }; 
  lanelet::BasicPoint2d pt5{-194.906,	473.67 }; 
  lanelet::BasicPoint2d pt6{-197.643,	472.343 }; 
  lanelet::BasicPoint2d pt7{-200.364,	471.012 }; 
  lanelet::BasicPoint2d pt8{-203.085,	469.681 }; 
  lanelet::BasicPoint2d pt9{-205.805,	468.349 }; 
  lanelet::BasicPoint2d pt10{-208.526,	467.018 }; 
  lanelet::BasicPoint2d pt12{-211.247,	465.686 }; 
  lanelet::BasicPoint2d pt13{-213.994,	464.278 }; 
  lanelet::BasicPoint2d pt14{-216.678,	462.79 }; 
  lanelet::BasicPoint2d pt15{-219.305,	461.219 }; 
  lanelet::BasicPoint2d pt16{-221.881,	459.563 }; 
  lanelet::BasicPoint2d pt17{-224.45	,458.016 }; 
  lanelet::BasicPoint2d pt18{-227.165,	456.806 }; 
  lanelet::BasicPoint2d pt19{-230.195,	454.988 }; 
  lanelet::BasicPoint2d pt20{-232.68	,452.626 }; 
  lanelet::BasicPoint2d pt21{-234.847,	450.304 }; 
  lanelet::BasicPoint2d pt22{-236.861,	448.008 }; 
  lanelet::BasicPoint2d pt23{-238.816,	445.664 }; 
  lanelet::BasicPoint2d pt24{-240.711,	443.272 }; 

  std::vector<lanelet::BasicPoint2d> downsampled_points;
  downsampled_points.push_back(pt1);  
  downsampled_points.push_back(pt2);  
  downsampled_points.push_back(pt3);  
  downsampled_points.push_back(pt4);  
  downsampled_points.push_back(pt5);  
  downsampled_points.push_back(pt6);  
  downsampled_points.push_back(pt7);  
  downsampled_points.push_back(pt8);  
  downsampled_points.push_back(pt9);  
  downsampled_points.push_back(pt10);  
  downsampled_points.push_back(pt12);  
  downsampled_points.push_back(pt13);  
  downsampled_points.push_back(pt14);  
  downsampled_points.push_back(pt15);  
  downsampled_points.push_back(pt16);  
  downsampled_points.push_back(pt17);  
  downsampled_points.push_back(pt18);  
  downsampled_points.push_back(pt19);  
  downsampled_points.push_back(pt20);  
  downsampled_points.push_back(pt21);  
  downsampled_points.push_back(pt22);  
  downsampled_points.push_back(pt23);  
  downsampled_points.push_back(pt24);  

  std::unique_ptr<smoothing::SplineI> fit_curve = inlc.compute_fit(downsampled_points);
  
  std::vector<lanelet::BasicPoint2d> spline_points;

  // Following logic is written for BSpline library. Switch with appropriate call of the new library if different.
  double parameter = 0.0;
  for(int i=0; i< downsampled_points.size(); i++){
    double c = inlc.compute_curvature_at(fit_curve, parameter);
    // Uncomment to print and check if this generated map matches with the original one above 
    lanelet::BasicPoint2d pt = (*fit_curve)(parameter);
    //ROS_INFO_STREAM("BSpline point: x: " << pt.x() << "y: " << pt.y());
    ROS_DEBUG_STREAM("c1: " << c);
    spline_points.push_back(pt);
    parameter += 1.0/(double)downsampled_points.size();
  }
  //std::unique_ptr<smoothing::SplineI> fit_curve_2 = inlc.compute_fit(spline_points);
  parameter = 0.0;
  for(int i=0; i< 68; i++){
    double c = inlc.compute_curvature_at(fit_curve, parameter);
    // Uncomment to print and check if this generated map matches with the original one above 
    lanelet::BasicPoint2d pt = (*fit_curve)(parameter);
    //ROS_INFO_STREAM("BSpline point: x: " << pt.x() << "y: " << pt.y());
    ROS_DEBUG_STREAM("c2: " << c);
    parameter += 1.0/(double)68;
  }
  //double c = inlc.compute_curvature_at(fit_curve, parameter);
  //ROS_DEBUG_STREAM("c: " << c);

  //lanelet::BasicPoint2d pt = (*fit_curve)(parameter);
  //spline_points.push_back(pt);


  //ASSERT_EQ(spline_points.size(), downsampled_points.size());
  int error_count = 0;
  /*
  for(int i = 1; i < downsampled_points.size(); i ++)
  {
    // tag as erroneous if directions of generated points are not within 5 degree of those of original points
    tf::Vector3 original_vector(downsampled_points[i].x() - downsampled_points[i-1].x(), 
                      downsampled_points[i].y() - downsampled_points[i-1].y(), 0);
    original_vector.setZ(0);
    tf::Vector3 spline_vector(spline_points[i].x() - spline_points[i-1].x(), 
                      spline_points[i].y() - spline_points[i-1].y(), 0);
    spline_vector.setZ(0);
    double angle_in_rad = std::fabs(tf::tfAngle(original_vector, spline_vector));
    if (angle_in_rad > 0.08)  error_count ++;
  }
  
  // We say it is passing if there is less than 10% error in total number of points
  ROS_INFO_STREAM("Total points above 5 degree difference in their direction:" << error_count 
    << ", which is " << (double)error_count/(double)downsampled_points.size()*100 << "% of total");
  ASSERT_TRUE((double)error_count/(double)downsampled_points.size() < 0.1); 
  */
}

TEST(WaypointGeneratorTest, inlane_similar_fraction)
{
  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;

  // File location of osm file
  std::string file = "/workspaces/carma/AOI_1_TFHRC_faster_pretty.osm";    
  // The route ids that will form the route used
  std::vector<lanelet::Id> route_ids = { 130, 129 };

  // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
  lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

  lanelet::MapConformer::ensureCompliance(map, 80_mph);

  InLaneCruisingPluginConfig config;
  config.lateral_accel_limit = 2.5;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  wm->setMap(map);

  InLaneCruisingPlugin inlc(wm, config, [&](auto msg) {});
  
  auto routing_graph = wm->getMapRoutingGraph();

  // INCLUDE POINTS HERE
  lanelet::BasicPoint2d pt1{-213.994,  	464.278 }; 
  lanelet::BasicPoint2d pt2{-216.678,  	462.79 }; 
  lanelet::BasicPoint2d pt3{-219.305,  	461.219 }; 
  lanelet::BasicPoint2d pt4{-221.881,  	459.563 }; 
  lanelet::BasicPoint2d pt5{-224.45,  458.016 }; 
  lanelet::BasicPoint2d pt6{-227.165,  	456.806 }; 
  lanelet::BasicPoint2d pt7{-230.195,  	454.988 }; 
  lanelet::BasicPoint2d pt8{-232.68,  	452.626 }; 
  lanelet::BasicPoint2d pt9{-234.847,  	450.304 }; 
  lanelet::BasicPoint2d pt10{-236.861,  	448.008 }; 
  lanelet::BasicPoint2d pt12{-238.816,  	445.664 }; 
  lanelet::BasicPoint2d pt13{-240.711,  	443.272 }; 
  lanelet::BasicPoint2d pt14{-242.549,  440.887 }; 
  lanelet::BasicPoint2d pt15{-244.404,  	438.483 }; 
  lanelet::BasicPoint2d pt16{-246.316,  	435.974 }; 
  lanelet::BasicPoint2d pt17{-248.16,  433.394 }; 
  lanelet::BasicPoint2d pt18{-249.918,  	430.706 }; 
  lanelet::BasicPoint2d pt19{-250.341,  	426.332 }; 
  lanelet::BasicPoint2d pt20{-248.233,  423.067 }; 
  lanelet::BasicPoint2d pt21{-244.199,  	421.018 }; 
  lanelet::BasicPoint2d pt22{-238.807,  	422.086 }; 
  //lanelet::BasicPoint2d pt23{-238.816,	445.664 }; 
 // lanelet::BasicPoint2d pt24{-240.711,	443.272 }; 

  std::vector<lanelet::BasicPoint2d> curve_points;
  curve_points.push_back(pt1);  
  curve_points.push_back(pt2);  
  curve_points.push_back(pt3);  
  curve_points.push_back(pt4);  
  curve_points.push_back(pt5);  
  curve_points.push_back(pt6);  
  curve_points.push_back(pt7);  
  curve_points.push_back(pt8);  
  curve_points.push_back(pt9);  
  curve_points.push_back(pt10);  
  curve_points.push_back(pt12);  
  curve_points.push_back(pt13);  
  curve_points.push_back(pt14);  
  curve_points.push_back(pt15);  
  curve_points.push_back(pt16);  
  curve_points.push_back(pt17);  
  curve_points.push_back(pt18);  
  curve_points.push_back(pt19);  
  curve_points.push_back(pt20);  
  curve_points.push_back(pt21);  
  curve_points.push_back(pt22);  
 // curve_points.push_back(pt23);  
 // curve_points.push_back(pt24);  
    cav_msgs::VehicleState state;
  state.X_pos_global = -227.215;
  state.Y_pos_global = 456.56;
  state.longitudinal_vel =  4.42212;
  state.yaw_rate = -2.58804;
  std::vector<double> speed_limits;
  for (int i = 0; i <curve_points.size(); i ++)
  {
    speed_limits.push_back(8.9408);
  }
   ///////////////
   // SIMULATION START
   ///////////////

  std::vector<double> final_yaw_values;
  std::vector<double> final_actual_speeds;
  std::vector<lanelet::BasicPoint2d> all_sampling_points;

  std::unique_ptr<smoothing::SplineI> fit_curve = inlc.compute_fit(curve_points); // Compute splines based on curve points
  if (!fit_curve)
  {
    throw std::invalid_argument("Could not fit a spline curve along the given trajectory!");
  }

  ROS_DEBUG("Got fit");

  ROS_DEBUG_STREAM("speed_limits.size() " << speed_limits.size());

  std::vector<lanelet::BasicPoint2d> sampling_points;
  sampling_points.reserve(1 + curve_points.size() * 2);

  std::vector<double> distributed_speed_limits;
  distributed_speed_limits.reserve(1 + curve_points.size() * 2);

  // compute total length of the trajectory to get correct number of points 
  // we expect using curve_resample_step_size
  std::vector<double> downtracks_raw = carma_wm::geometry::compute_arc_lengths(curve_points);

  int total_step_along_curve = static_cast<int>(downtracks_raw.back() / config.curve_resample_step_size);

  int current_speed_index = 0;
  size_t total_point_size = curve_points.size();

  double step_threshold_for_next_speed = (double)total_step_along_curve / (double)total_point_size;
  double scaled_steps_along_curve = 0.0; // from 0 (start) to 1 (end) for the whole trajectory
  std::vector<double> better_curvature;
  better_curvature.reserve(1 + curve_points.size() * 2);
    
  for (size_t steps_along_curve = 0; steps_along_curve < total_step_along_curve; steps_along_curve++) // Resample curve at tighter resolution
  {
    lanelet::BasicPoint2d p = (*fit_curve)(scaled_steps_along_curve);
    
    sampling_points.push_back(p);
    double c = inlc.compute_curvature_at(fit_curve, scaled_steps_along_curve);
    better_curvature.push_back(c);
    if ((double)steps_along_curve > step_threshold_for_next_speed)
    {
      step_threshold_for_next_speed += (double)total_step_along_curve / (double) total_point_size;
      current_speed_index ++;
    }
    distributed_speed_limits.push_back(speed_limits[current_speed_index]); // Identify speed limits for resampled points
    scaled_steps_along_curve += 1.0/total_step_along_curve; //adding steps_along_curve_step_size
  }

  std::vector<double> yaw_values = carma_wm::geometry::compute_tangent_orientations(sampling_points);

  std::vector<double> curvatures = better_curvature;

  log::printDoublesPerLineWithPrefix("curvatures[i]: ", curvatures);

  std::vector<double> ideal_speeds =
      trajectory_utils::constrained_speeds_for_curvatures(curvatures, config.lateral_accel_limit);

  log::printDoublesPerLineWithPrefix("ideal_speeds: ", ideal_speeds);

  std::vector<double> actual_speeds = inlc.apply_speed_limits(ideal_speeds, distributed_speed_limits);

  final_yaw_values.insert(final_yaw_values.end(), yaw_values.begin(), yaw_values.end());
  all_sampling_points.insert(all_sampling_points.end(), sampling_points.begin(), sampling_points.end());
  final_actual_speeds.insert(final_actual_speeds.end(), actual_speeds.begin(), actual_speeds.end());
  
  // Add current vehicle point to front of the trajectory
  int nearest_pt_index = inlc.get_nearest_point_index(all_sampling_points, state);
  ROS_DEBUG_STREAM("Curvature right now: " << better_curvature[nearest_pt_index] << ", at x: " << state.X_pos_global << ", y: " << state.Y_pos_global);

  std::vector<lanelet::BasicPoint2d> future_basic_points(all_sampling_points.begin() + nearest_pt_index + 1,
                                            all_sampling_points.end());  // Points in front of current vehicle position

  std::vector<double> future_speeds(final_actual_speeds.begin() + nearest_pt_index + 1,
                                            final_actual_speeds.end());  // Points in front of current vehicle position
  std::vector<double> future_yaw(final_yaw_values.begin() + nearest_pt_index + 1,
                                            final_yaw_values.end());  // Points in front of current vehicle position
  final_actual_speeds = future_speeds;
  all_sampling_points = future_basic_points;
  final_yaw_values = future_yaw;

  lanelet::BasicPoint2d cur_veh_point(state.X_pos_global, state.Y_pos_global);

  all_sampling_points.insert(all_sampling_points.begin(),
                             cur_veh_point);  // Add current vehicle position to front of sample points

  final_actual_speeds.insert(final_actual_speeds.begin(), state.longitudinal_vel);

  final_yaw_values.insert(final_yaw_values.begin(), state.orientation);

  // Compute points to local downtracks
  std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(all_sampling_points);

  // Apply accel limits
  final_actual_speeds = inlc.optimize_speed(downtracks, final_actual_speeds, config.max_accel);
  final_actual_speeds = smoothing::moving_average_filter(final_actual_speeds, config.moving_average_window_size);
  log::printDoublesPerLineWithPrefix("post_average[i]: ", final_actual_speeds);

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