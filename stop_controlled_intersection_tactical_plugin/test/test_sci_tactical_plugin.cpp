/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include <stop_controlled_intersection_plugin.h>
#include <carma_wm/WMTestLibForGuidance.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/CARMAWorldModel.h>
#include <cav_msgs/LaneFollowingManeuver.h>
#include <cav_msgs/VehicleState.h>
#include <cav_srvs/PlanManeuversRequest.h>
#include <cav_srvs/PlanManeuversResponse.h>

namespace stop_controlled_intersection_transit_plugin
{
    TEST(StopControlledIntersectionTacticalPlugin, TestSCIPlanning_case_one)
    {
      //Test Stop controlled Intersection tactical plugin generation
      ros::Time::setNow(ros::Time(0.0));
      StopControlledIntersectionTacticalPluginConfig config;
      std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
      StopControlledIntersectionTacticalPlugin plugin(wm, config, [&](auto msg) {});
      
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
    
    //Create a request and maneuver that meets case 1 criteria
    //In order to be case 1 - estimated_stop_time > scheduled_stop_time and speed_before_stop < speed_limit
    //speed_before_stop
     cav_srvs::PlanTrajectoryRequest req;
     req.vehicle_state.X_pos_global = 1.5;
     req.vehicle_state.Y_pos_global = 5.0;
     req.vehicle_state.orientation = 0;
     req.vehicle_state.longitudinal_vel = 11.176;

     cav_msgs::Maneuver maneuver;
     maneuver.type = cav_msgs::Maneuver::LANE_FOLLOWING;
     maneuver.lane_following_maneuver.start_dist = req.vehicle_state.Y_pos_global;
     maneuver.lane_following_maneuver.start_time = ros::Time(0.0);
     maneuver.lane_following_maneuver.start_speed = 11.176;  //25 mph in mps

     maneuver.lane_following_maneuver.end_dist = 100.0;
     maneuver.lane_following_maneuver.end_speed = 0.0;

    //Enter meta data
    maneuver.lane_following_maneuver.parameters.int_valued_meta_data.push_back(1);    //Case number
    maneuver.lane_following_maneuver.parameters.string_valued_meta_data.push_back("Carma/stop_controlled_intersection");
    //Float meta data list - a_acc, a_dec, t_acc, t_dec, speed_before_decel
    
    //Calculate speed before decel - The speed the vehicle should accelerate to before slowing down
    //Assuming the a_dec to be 2m/s^2, the vehicle should be able to stop at that deceleration- over total_dist/2
    //So accelerate half way and decelerate the rest
    double a_dec = -0.5;
    double assumed_dec_dist = maneuver.lane_following_maneuver.end_dist/2;
    double speed_before_dec = sqrt(2*std::abs(a_dec)*assumed_dec_dist);
    
    double a_acc = (pow(speed_before_dec,2) - pow(maneuver.lane_following_maneuver.start_speed,2))/(2*(maneuver.lane_following_maneuver.end_dist - assumed_dec_dist));
    double t_acc = (speed_before_dec - maneuver.lane_following_maneuver.start_speed)/a_acc;
    double t_dec = (speed_before_dec)/std::abs(a_dec);

    maneuver.lane_following_maneuver.end_time = maneuver.lane_following_maneuver.start_time + ros::Duration(t_acc + t_dec);

    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(a_acc);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(a_dec);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(t_acc);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(t_dec);
    maneuver.lane_following_maneuver.parameters.float_valued_meta_data.push_back(speed_before_dec);

    req.maneuver_plan.maneuvers.push_back(maneuver);
    req.maneuver_index_to_plan = 0;

    cav_srvs::PlanTrajectoryResponse resp;

    plugin.plan_trajectory_cb(req, resp);

    EXPECT_EQ(0, resp.related_maneuvers.back());

    //Test maneuvers_to_points
    std::vector<cav_msgs::Maneuver> maneuver_plan;
    maneuver_plan.push_back(maneuver);
    std::vector<PointSpeedPair> points_and_target_speeds = plugin.maneuvers_to_points(maneuver_plan, wm, req.vehicle_state);
    EXPECT_EQ(points_and_target_speeds[0].speed, req.vehicle_state.longitudinal_vel);

    //Test compose_trajectory_from_centerline
    req.header.stamp = ros::Time(0.0);
    std::vector<cav_msgs::TrajectoryPlanPoint> trajectory = plugin.compose_trajectory_from_centerline(points_and_target_speeds, req.vehicle_state, req.header.stamp);
    EXPECT_TRUE(trajectory.size() > 2);
  }

}

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