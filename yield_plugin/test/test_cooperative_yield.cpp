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

#include <yield_plugin/yield_plugin.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/CARMAWorldModel.h>
#include <math.h>
#include <tf/LinearMath/Vector3.h>
#include <boost/property_tree/json_parser.hpp>


using namespace yield_plugin;


TEST(YieldPluginTest, compose_mobility_response)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  YieldPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
  cav_msgs::MobilityResponse resp = plugin.compose_mobility_response("recicpient_id", "plan_id", true);

  EXPECT_EQ(resp.m_header.recipient_id, "recicpient_id");
  EXPECT_EQ(resp.m_header.plan_id, "plan_id");
  EXPECT_TRUE(resp.is_accepted);
}


TEST(YieldPluginTest, test_detect_trajectories_intersection)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  YieldPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});

  std::vector<lanelet::BasicPoint2d> v1, v2;

  lanelet::BasicPoint2d p1, p2, p3;
  p1.x() = 1;
  p1.y() = 1;
  p2.x() = 2;
  p2.y() = 2;
  p3.x() = 3;
  p3.y() = 3;
  v1 = {p1, p2, p3};

  
  lanelet::BasicPoint2d p4;
  p4.x() = 2.5;
  p4.y() = 0;
  lanelet::BasicPoint2d p5;
  p5.x() = 0;
  p5.y() = 2.5;
  v2 = {p4, p5};

  std::vector<std::pair<int, lanelet::BasicPoint2d>> output = plugin.detect_trajectories_intersection(v1, v2);
  EXPECT_EQ(output.size(), 2);
  EXPECT_EQ(output[0].first, 0);
  EXPECT_EQ(output[0].second.x(), 2.5);

}

TEST(YieldPluginTest, test_update_clc_trajectory)
{
    YieldPluginConfig config;
    config.safety_collision_time_gap = 0.1;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    YieldPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
    

    
    lanelet::BasicPoint2d p1(10, 0);
    lanelet::BasicPoint2d p2(20, 0);
    lanelet::BasicPoint2d p3(30, 0);
    lanelet::BasicPoint2d p4(40, 0);
    std::vector<lanelet::BasicPoint2d> incoming_traj = {p1, p2, p3, p4};

    double req_speed = 5;
    double req_time = 7;
    double req_stamp = 5;
    plugin.set_incoming_request_info(incoming_traj, req_speed, req_time, req_stamp);

    cav_msgs::TrajectoryPlan original_tp;

    cav_msgs::TrajectoryPlanPoint trajectory_point_1;
    cav_msgs::TrajectoryPlanPoint trajectory_point_2;
    cav_msgs::TrajectoryPlanPoint trajectory_point_3;
    cav_msgs::TrajectoryPlanPoint trajectory_point_4;
    cav_msgs::TrajectoryPlanPoint trajectory_point_5;
    cav_msgs::TrajectoryPlanPoint trajectory_point_6;
    cav_msgs::TrajectoryPlanPoint trajectory_point_7;

    trajectory_point_1.x = 20.0;
    trajectory_point_1.y = -40.0;
    trajectory_point_1.target_time = ros::Time(0);

    trajectory_point_2.x = 20.0;
    trajectory_point_2.y = -30.0;
    trajectory_point_2.target_time = ros::Time(1);

    trajectory_point_3.x = 20.0;
    trajectory_point_3.y = -20.0;
    trajectory_point_3.target_time = ros::Time(2);
    
    trajectory_point_4.x = 20.0;
    trajectory_point_4.y = -10.0;
    trajectory_point_4.target_time = ros::Time(3);

    trajectory_point_5.x = 20.0;
    trajectory_point_5.y = 0.0;
    trajectory_point_5.target_time = ros::Time(4);

    trajectory_point_6.x = 20.0;
    trajectory_point_6.y = 10.0;
    trajectory_point_6.target_time = ros::Time(5);

    trajectory_point_7.x = 20.0;
    trajectory_point_7.y = 20.0;
    trajectory_point_7.target_time = ros::Time(6);
    
    original_tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};

    double current_speed = 10;
    cav_msgs::TrajectoryPlan yield_plan = plugin.update_traj_for_cooperative_behavior(original_tp, current_speed);

    EXPECT_EQ(yield_plan.trajectory_points.size(), original_tp.trajectory_points.size());
    // slow down confirmed, since target time for trajectories has increaded.
    EXPECT_TRUE(yield_plan.trajectory_points[2].target_time > original_tp.trajectory_points[2].target_time);
    EXPECT_TRUE(yield_plan.trajectory_points[3].target_time > original_tp.trajectory_points[3].target_time);
}

TEST(YieldPluginTest, test_traj_cb)
{
    YieldPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    YieldPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});

    cav_msgs::TrajectoryPlan original_tp;

    cav_msgs::TrajectoryPlanPoint trajectory_point_1;
    cav_msgs::TrajectoryPlanPoint trajectory_point_2;
    cav_msgs::TrajectoryPlanPoint trajectory_point_3;
    cav_msgs::TrajectoryPlanPoint trajectory_point_4;
    cav_msgs::TrajectoryPlanPoint trajectory_point_5;
    cav_msgs::TrajectoryPlanPoint trajectory_point_6;
    cav_msgs::TrajectoryPlanPoint trajectory_point_7;

    trajectory_point_1.x = 20.0;
    trajectory_point_1.y = -40.0;
    trajectory_point_1.target_time = ros::Time(0);

    trajectory_point_2.x = 20.0;
    trajectory_point_2.y = -30.0;
    trajectory_point_2.target_time = ros::Time(1);

    trajectory_point_3.x = 20.0;
    trajectory_point_3.y = -20.0;
    trajectory_point_3.target_time = ros::Time(2);
    
    trajectory_point_4.x = 20.0;
    trajectory_point_4.y = -10.0;
    trajectory_point_4.target_time = ros::Time(3);

    trajectory_point_5.x = 20.0;
    trajectory_point_5.y = 0.0;
    trajectory_point_5.target_time = ros::Time(4);

    trajectory_point_6.x = 20.0;
    trajectory_point_6.y = 10.0;
    trajectory_point_6.target_time = ros::Time(5);

    trajectory_point_7.x = 20.0;
    trajectory_point_7.y = 20.0;
    trajectory_point_7.target_time = ros::Time(60);
    
    original_tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};

    cav_srvs::PlanTrajectoryRequest req;
    req.vehicle_state.x_pos_global = 1.5;
    req.vehicle_state.y_pos_global = 5;
    req.vehicle_state.orientation = 0;
    req.vehicle_state.longitudinal_vel = 0.0;

    req.initial_trajectory_plan = original_tp;

    cav_srvs::PlanTrajectoryResponse resp;

    plugin.plan_trajectory_cb(req, resp);

}

