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

#include <stop_controlled_intersection_plugin.hpp>
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <carma_planning_msgs/msg/lane_following_maneuver.hpp>
#include <carma_planning_msgs/msg/vehicle_state.hpp>
#include <carma_planning_msgs/srv/plan_maneuvers.hpp>

namespace stop_controlled_intersection_tactical_plugin
{
  TEST(StopControlledIntersectionTacticalPlugin, TestSCIPlanning_case_one)
  {
    //Test Stop controlled Intersection tactical plugin generation
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    auto node = std::make_shared<stop_controlled_intersection_tactical_plugin::StopControlledIntersectionTacticalPlugin>(rclcpp::NodeOptions());
    node->configure();
    node->activate();
    
    auto map = carma_wm::test::buildGuidanceTestMap(3.7, 50);

    wm->setMap(map);
    
    carma_wm::test::setSpeedLimit(40_mph, wm);
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

    carma_wm::test::setRouteByIds({1200, 1201, 1202, 1203}, wm);

    node->wm_ = wm;

    //Create a request and maneuver that meets case 1 criteria
    //In order to be case 1 - estimated_stop_time > scheduled_stop_time and speed_before_stop < speed_limit
    //speed_before_stop
    auto req  = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Request>();
    req->vehicle_state.x_pos_global = 1.5;
    req->vehicle_state.y_pos_global = 5.0;
    req->vehicle_state.orientation = 0;
    req->vehicle_state.longitudinal_vel = 11.176;

    carma_planning_msgs::msg::Maneuver maneuver;
    maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
    maneuver.lane_following_maneuver.start_dist = req->vehicle_state.y_pos_global;
    maneuver.lane_following_maneuver.start_time = rclcpp::Time(0.0);
    maneuver.lane_following_maneuver.start_speed = 11.176;  //25 mph in mps

    maneuver.lane_following_maneuver.end_dist = 100.0;
    maneuver.lane_following_maneuver.end_speed = 0.0;

    //Enter meta data
    maneuver.lane_following_maneuver.parameters.int_valued_meta_data.push_back(1);    //Case number
    maneuver.lane_following_maneuver.parameters.string_valued_meta_data.push_back("Carma/stop_controlled_intersection");
    //Float meta data list - a_acc, a_dec, t_acc, t_dec, speed_before_decel
    
    //Calculate speed before decel - The speed the vehicle should accelerate before slowing down
    //Assuming the a_dec to be 0.5m/s^2, the vehicle should be able to stop at that deceleration- over total_dist/2
    //So accelerate half way and decelerate the rest
    double a_dec = -1.5;
    double assumed_dec_dist = maneuver.lane_following_maneuver.end_dist/2;
    double speed_before_dec = sqrt(2*std::abs(a_dec)*assumed_dec_dist);
    
    double a_acc = (pow(speed_before_dec,2) - pow(maneuver.lane_following_maneuver.start_speed,2))/(2*(maneuver.lane_following_maneuver.end_dist - assumed_dec_dist));
    double t_acc = (speed_before_dec - maneuver.lane_following_maneuver.start_speed)/a_acc;
    double t_dec = (speed_before_dec)/std::abs(a_dec);

    maneuver.lane_following_maneuver.end_time = rclcpp::Time(maneuver.lane_following_maneuver.start_time) + rclcpp::Duration((t_acc + t_dec)*1e9);

    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(a_acc);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(a_dec);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(t_acc);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(t_dec);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(speed_before_dec);

    req->maneuver_plan.maneuvers.push_back(maneuver);
    req->maneuver_index_to_plan = 0;

    auto resp  = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Response>();
    auto header_srv = std::make_shared<rmw_request_id_t>();
    node->plan_trajectory_callback(header_srv, req, resp);

    EXPECT_EQ(0, resp->related_maneuvers.back());

    //Test maneuvers_to_points
    std::vector<carma_planning_msgs::msg::Maneuver> maneuver_plan;
    maneuver_plan.push_back(maneuver);
    std::vector<PointSpeedPair> points_and_target_speeds = node->maneuvers_to_points(maneuver_plan, wm, req->vehicle_state);
    EXPECT_EQ(points_and_target_speeds[0].speed, req->vehicle_state.longitudinal_vel);

    //Test create_case_one_speed_profile
    //get geometry points from maneuvers_to_points
    std::vector<lanelet::BasicPoint2d> geometry_profile;
    std::vector<double> speeds;
    basic_autonomy::waypoint_generation:: split_point_speed_pairs(points_and_target_speeds, &geometry_profile, &speeds);
    std::vector<PointSpeedPair> case_one_profile = node->create_case_one_speed_profile(wm, maneuver, geometry_profile,  req->vehicle_state.longitudinal_vel, req->vehicle_state);
    //Ensure acceleration and deceleration is happening
    double dist_to_cover = maneuver.lane_following_maneuver.end_dist - maneuver.lane_following_maneuver.start_dist;
    double total_dist_covered = 0;
  
    double prev_speed = case_one_profile[0].speed;
    lanelet::BasicPoint2d prev_point = case_one_profile[0].point;
    double current_speed;
    lanelet::BasicPoint2d current_point;
    
    for(int i = 0;i < case_one_profile.size();i++){
      current_point = case_one_profile[i].point;
      current_speed = case_one_profile[i].speed;
      double delta_d = lanelet::geometry::distance2d(prev_point, current_point);
      total_dist_covered += delta_d;   
      if(total_dist_covered < 50.1){ //According to test conditions acceleration till approx 50 meters 
        EXPECT_TRUE(current_speed >= prev_speed);
      }
      else{
        EXPECT_TRUE(current_speed <= prev_speed);
      }

      prev_speed = current_speed;
      prev_point = current_point;
    }


    //Test compose_trajectory_from_centerline
    req->header.stamp = rclcpp::Time(0.0);
    std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory = node->compose_trajectory_from_centerline(points_and_target_speeds, req->vehicle_state, req->header.stamp);
    EXPECT_TRUE(trajectory.size() > 2);
  }

  TEST(StopControlledIntersectionTacticalPlugin, TestSCIPlanning_case_two){
    //Test Stop controlled Intersection tactical plugin generation
    
    StopControlledIntersectionTacticalPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    auto node = std::make_shared<stop_controlled_intersection_tactical_plugin::StopControlledIntersectionTacticalPlugin>(rclcpp::NodeOptions());
    node->configure();
    node->activate();
    
    auto map = carma_wm::test::buildGuidanceTestMap(3.7, 50);

    wm->setMap(map);
    carma_wm::test::setSpeedLimit(30_mph, wm);
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

    carma_wm::test::setRouteByIds({1200, 1201, 1202, 1203}, wm);   
    node->wm_ = wm;
    
    //Create a request and maneuver that meets case 2 criteria
    //speed_before_stop
     auto req  = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Request>();
     req->vehicle_state.x_pos_global = 1.5;
     req->vehicle_state.y_pos_global = 5.0;
     req->vehicle_state.orientation = 0;
     req->vehicle_state.longitudinal_vel = 11.176;

     carma_planning_msgs::msg::Maneuver maneuver;
     maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
     maneuver.lane_following_maneuver.start_dist = req->vehicle_state.y_pos_global;
     maneuver.lane_following_maneuver.start_time = rclcpp::Time(0.0);
     maneuver.lane_following_maneuver.start_speed = 11.176;  //25 mph in mps

     maneuver.lane_following_maneuver.end_dist = 100.0;
     maneuver.lane_following_maneuver.end_speed = 0.0;

    //Enter meta data
    maneuver.lane_following_maneuver.parameters.int_valued_meta_data.push_back(2);    //Case number
    maneuver.lane_following_maneuver.parameters.string_valued_meta_data.push_back("Carma/stop_controlled_intersection");

    //Float meta data list - a_acc, a_dec, t_acc, t_dec, t_cruise, speed_before_decel
    
    //Calculate speed before decel - The speed the vehicle should accelerate and then cruise before slowing down
    //Assuming the a_dec to be 2m/s^2, the vehicle should be able to stop at that deceleration starting from speed limit

    double a_dec = -2.0;
    double speed_before_decel = 13.4112; //30_mph in mps (Speed Limit)
    double dist_decel = pow(speed_before_decel, 2)/(2*std::abs(a_dec));
    //Fix Cruising distance
    double dist_cruising = 20.0;
    double  dist_acc = (maneuver.lane_following_maneuver.end_dist - maneuver.lane_following_maneuver.start_dist) - dist_decel - dist_cruising;
    double a_acc = (pow(speed_before_decel,2) - pow(maneuver.lane_following_maneuver.start_speed, 2))/(2*dist_acc);
    double t_acc = (speed_before_decel - maneuver.lane_following_maneuver.start_speed)/a_acc;
    double t_dec = speed_before_decel/std::abs(a_dec);
    double t_cruising = dist_cruising/speed_before_decel;

    maneuver.lane_following_maneuver.end_time = rclcpp::Time(maneuver.lane_following_maneuver.start_time) + rclcpp::Duration((t_acc + t_cruising + t_dec)*1e9);

    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(a_acc);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(a_dec);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(t_acc);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(t_dec);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(t_cruising);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(speed_before_decel);

    req->maneuver_plan.maneuvers.push_back(maneuver);
    req->maneuver_index_to_plan = 0;

    auto resp  = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Response>();

    auto header_srv = std::make_shared<rmw_request_id_t>();
    node->plan_trajectory_callback(header_srv, req, resp);
    EXPECT_EQ(0, resp->related_maneuvers.back());

    //Test create_case_two_speed_profile
    std::vector<lanelet::BasicPoint2d> route_geometry_points = wm->sampleRoutePoints(
            std::min(maneuver.lane_following_maneuver.start_dist + 1.0, maneuver.lane_following_maneuver.end_dist), 
            maneuver.lane_following_maneuver.end_dist, 1.0);
    
    std::vector<PointSpeedPair> case_two_profile = node->create_case_two_speed_profile(wm, maneuver, route_geometry_points, req->vehicle_state.longitudinal_vel);

    double prev_speed = case_two_profile[0].speed;
    for(int i = 0;i< case_two_profile.size();i++){
      double current_speed = case_two_profile[i].speed;
      if(i <= 31){
        EXPECT_TRUE(current_speed >= prev_speed);
      }
      else if(i > 31 && i < 52){
        EXPECT_TRUE(current_speed == prev_speed);
      }
      else{
        EXPECT_TRUE(current_speed < prev_speed);
      }
      
      prev_speed = current_speed;
    }

  }

  TEST(StopControlledIntersectionTacticalPlugin, TestSCIPlanning_case_three){
    //Test Stop controlled Intersection tactical plugin generation

    StopControlledIntersectionTacticalPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    auto node = std::make_shared<stop_controlled_intersection_tactical_plugin::StopControlledIntersectionTacticalPlugin>(rclcpp::NodeOptions());
    node->configure();
    node->activate();
    
    auto map = carma_wm::test::buildGuidanceTestMap(3.7, 50);

    wm->setMap(map);
    carma_wm::test::setSpeedLimit(30_mph, wm);
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

    carma_wm::test::setRouteByIds({1200, 1201, 1202, 1203}, wm);   
    node->wm_ = wm;
    //Create a request and maneuver that meets case 3 criteria
    //In order to be case 2 - estimated_stop_time > scheduled_stop_time and speed_before_decel =  speed_limit
    //speed_before_stop
    auto req  = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Request>();
    req->vehicle_state.x_pos_global = 1.5;
    req->vehicle_state.y_pos_global = 5.0;
    req->vehicle_state.orientation = 0;
    req->vehicle_state.longitudinal_vel = 11.176;

    carma_planning_msgs::msg::Maneuver maneuver;
    maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
    maneuver.lane_following_maneuver.start_dist = req->vehicle_state.y_pos_global;
    maneuver.lane_following_maneuver.start_time = rclcpp::Time(0.0);
    maneuver.lane_following_maneuver.start_speed = 11.176;  //25 mph in mps

    maneuver.lane_following_maneuver.end_dist = 100.0;
    maneuver.lane_following_maneuver.end_speed = 0.0;

    //Enter meta data
    maneuver.lane_following_maneuver.parameters.int_valued_meta_data.push_back(3);    //Case number
    maneuver.lane_following_maneuver.parameters.string_valued_meta_data.push_back("Carma/stop_controlled_intersection");

    //Float meta data list - a_dec
    double a_dec = -pow(maneuver.lane_following_maneuver.start_speed, 2)/(2*(maneuver.lane_following_maneuver.end_dist - maneuver.lane_following_maneuver.start_dist));
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(a_dec);

    //Test create_case_three_speed_profile
    std::vector<lanelet::BasicPoint2d> route_geometry_points = wm->sampleRoutePoints(
            std::min(maneuver.lane_following_maneuver.start_dist , maneuver.lane_following_maneuver.end_dist), 
            maneuver.lane_following_maneuver.end_dist, 1.0);
    
    std::vector<PointSpeedPair> case_three_profile = node->create_case_three_speed_profile(wm, maneuver, route_geometry_points, req->vehicle_state.longitudinal_vel);
    double prev_speed = case_three_profile.front().speed;
    for(int i = 1;i <case_three_profile.size();i++){
      double current_speed = case_three_profile[i].speed;
      EXPECT_TRUE(current_speed <= prev_speed);
      
      prev_speed = current_speed;
    }
    EXPECT_TRUE(case_three_profile.back().speed < 0.5); //Last speed is less than 0.5mps
  }

}

/*!
* \brief Main entrypoint for unit tests
*/
int main (int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  //Initialize ROS
  rclcpp::init(argc, argv);
  auto ret = rcutils_logging_set_logger_level(
          rclcpp::get_logger("stop_controlled_intersection_tactical_plugin").get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  bool success = RUN_ALL_TESTS();

  //shutdown ROS
  rclcpp::shutdown();

  return success;
}