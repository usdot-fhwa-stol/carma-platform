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

#include <inlanecruising_plugin/object_avoidance.h>
#include <inlanecruising_plugin/inlanecruising_plugin.h>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/CARMAWorldModel.h>
#include <math.h>
#include <tf/LinearMath/Vector3.h>

using namespace inlanecruising_plugin;
TEST(InLaneCruisingPluginTest, test_polynomial_calc)
{

    std::vector<double> coeff;

    coeff.push_back(2.0);
    coeff.push_back(2.0);
    coeff.push_back(2.0);
    coeff.push_back(2.0);
    coeff.push_back(2.0);    
    coeff.push_back(2.0);    

    // double result = ap.polynomial_calc(coeff, 0);
    // EXPECT_EQ(2, result);

    // result = ap.polynomial_calc(coeff, 1);
    // EXPECT_EQ(12, result);

    // result = ap.polynomial_calc(coeff, 2);
    // EXPECT_EQ(126, result);

    // result = ap.polynomial_calc(coeff, 3);
    // EXPECT_EQ(728, result);
}

TEST(InLaneCruisingPluginTest, MaxTrajectorySpeed)
{

    std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points;

    ros::Time startTime(1.0);

    cav_msgs::TrajectoryPlanPoint point_1;
    point_1.x = 0.0;
    point_1.y = 0.0;
    point_1.target_time = startTime;
    point_1.lane_id = "1";
    trajectory_points.push_back(point_1);

    cav_msgs::TrajectoryPlanPoint point_2;
    point_2.x = 5.0;
    point_2.y = 0.0;
    point_2.target_time = startTime + ros::Duration(1);;
    point_2.lane_id = "1";
    trajectory_points.push_back(point_2);

    cav_msgs::TrajectoryPlanPoint point_3;
    point_3.x = 10.0;
    point_3.y = 0.0;
    point_3.target_time = startTime + ros::Duration(2);;
    point_3.lane_id = "1";
    trajectory_points.push_back(point_3);

    // cav_msgs::TrajectoryPlanPoint point_4;
    // point_4.x = 15.0;
    // point_4.y = 0.0;
    // point_4.target_time = 3000.0;
    // point_4.lane_id = "1";
    // trajectory_points.push_back(point_4);

    // cav_msgs::TrajectoryPlanPoint point_5;
    // point_5.x = 20.0;
    // point_5.y = 0.0;
    // point_5.target_time = 4000.0;
    // point_5.lane_id = "1";
    // trajectory_points.push_back(point_5);

    // cav_msgs::TrajectoryPlanPoint point_6;
    // point_6.x = 25.0;
    // point_6.y = 0.0;
    // point_6.target_time = 5000.0;
    // point_6.lane_id = "1";
    // trajectory_points.push_back(point_6);


    // cav_msgs::TrajectoryPlanPoint point_7;
    // point_7.x = 40.0;
    // point_7.y = 0.0;
    // point_7.target_time = 6000.0;
    // point_7.lane_id = "1";
    // trajectory_points.push_back(point_7);

    // autoware_plugin::AutowarePlugin ap;
    // double result = ap.max_trajectory_speed(trajectory_points);
    // EXPECT_EQ(5, result);

}