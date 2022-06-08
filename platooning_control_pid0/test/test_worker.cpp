
/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
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

------------------------------------------------------------------------------*/

#include "platoon_control_worker.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/math/constants/constants.hpp>

const double PI = boost::math::double_constants::pi;
const double TWO_PI = 2.0*PI;


TEST(PlatonControlWorkerTest, test_set_config)
{
    
}

TEST(PlatoonControlWorkerTest, test_find_nearest_point)
{
    platoon_control_pid0::PlatoonControlWorker pcw;
    std::vector<cav_msgs::TrajectoryPlanPoint> traj;

    cav_msgs::TrajectoryPlanPoint p0, p1, p2;
    p0.x = 1.0;
    p0.y = 2.0;
    traj.push_back(p0);
    p1.x = 1.0;
    p1.y = 3.0;
    traj.push_back(p1);
    p2.x = 1.0;
    p2.y = 4.0;
    traj.push_back(p2);
    pcw.unit_test_set_traj(traj);
    EXPECT_NEAR(3.0, pcw.unit_test_get_traj_py(1), 0.01);

    pcw.unit_test_set_pose(1.5, 0.0, 1.04*PI/2.0);
    pcw.find_nearest_point();
    EXPECT_EQ(0, pcw.get_tp_index());

    pcw.unit_test_set_pose(1.6, 2.2, 0.98*PI/2.0);
    pcw.find_nearest_point();
    EXPECT_EQ(1, pcw.get_tp_index());

    pcw.unit_test_set_pose(0.82, 3.9, 0.85*PI/2.0);
    pcw.find_nearest_point();
    EXPECT_EQ(2, pcw.get_tp_index());
}

TEST(PlatoonControlWorkerTest, test_calculate_cross_track1)
{
    platoon_control_pid0::PlatoonControlWorker pcw;
    std::vector<cav_msgs::TrajectoryPlanPoint> traj;

    cav_msgs::TrajectoryPlanPoint p0, p1, p2;
    p0.x = 1.0;
    p0.y = 2.0;
    traj.push_back(p0);
    p1.x = 1.0;
    p1.y = 3.0;
    traj.push_back(p1);
    p2.x = 1.0;
    p2.y = 4.0;
    traj.push_back(p2);
    pcw.unit_test_set_traj(traj);
    EXPECT_NEAR(3.0, pcw.unit_test_get_traj_py(1), 0.01);
    
    pcw.unit_test_set_pose(1.5, 0.0, 1.04*PI/2.0);
    pcw.find_nearest_point();
    EXPECT_EQ(0, pcw.get_tp_index());
    double cte = pcw.calculate_cross_track();
    EXPECT_NEAR(-0.5, cte, 0.01);

    pcw.unit_test_set_pose(1.6, 2.2, 0.98*PI/2.0);
    pcw.find_nearest_point();
    EXPECT_EQ(1, pcw.get_tp_index());
    cte = pcw.calculate_cross_track();
    EXPECT_NEAR(-0.6, cte, 0.01);

    pcw.unit_test_set_pose(0.82, 3.9, 0.85*PI/2.0);
    pcw.find_nearest_point();
    EXPECT_EQ(2, pcw.get_tp_index());
    cte = pcw.calculate_cross_track();
    EXPECT_NEAR(0.18, cte, 0.01);
}

TEST(PlatoonControlWorkerTest, test_calculate_cross_track2)
{
    platoon_control_pid0::PlatoonControlWorker pcw;
    std::vector<cav_msgs::TrajectoryPlanPoint> traj;

    cav_msgs::TrajectoryPlanPoint p0, p1, p2;
    p0.x = -1.0;
    p0.y = -2.0;
    traj.push_back(p0);
    p1.x = 0.0;
    p1.y = -1.0;
    traj.push_back(p1);
    p2.x = 1.0;
    p2.y = 0.0;
    traj.push_back(p2);
    pcw.unit_test_set_traj(traj);
    EXPECT_NEAR(-1.0, pcw.unit_test_get_traj_py(1), 0.01);
    
    pcw.unit_test_set_pose(-0.9, -0.2, 0.3*PI/2.0); //left of track, heading toward right side of track
    pcw.find_nearest_point();
    EXPECT_EQ(1, pcw.get_tp_index());
    double cte = pcw.calculate_cross_track();
    EXPECT_NEAR(1.20, cte, 0.01);
}

TEST(PlatoonControlWorkerTest, test_calc_desired_heading)
{
    platoon_control_pid0::PlatoonControlWorker pcw;
    std::vector<cav_msgs::TrajectoryPlanPoint> traj;

    cav_msgs::TrajectoryPlanPoint p0, p1, p2, p3;
    p0.x = -1.0;
    p0.y = -2.0;
    p0.yaw = 0.7854;
    traj.push_back(p0);
    p1.x = 0.0;
    p1.y = -1.0;
    p1.yaw = 0.7854;
    traj.push_back(p1);
    p2.x = 1.0;
    p2.y = 0.0;
    p2.yaw = 0.7854;
    traj.push_back(p2);
    p3.x = 3.0;
    p3.y = 0.8;
    p3.yaw = 0.6111;
    traj.push_back(p3);
    pcw.unit_test_set_traj(traj);
    EXPECT_NEAR(-1.0, pcw.unit_test_get_traj_py(1), 0.01);
    EXPECT_NEAR(0.0, pcw.unit_test_get_traj_py(2), 0.01);
    EXPECT_NEAR(0.8, pcw.unit_test_get_traj_py(3), 0.01);

    // first test - vehicle heading almost same as TP1, no lookahead
    pcw.unit_test_set_heading_lookahead(0);
    pcw.unit_test_set_pose(-0.9, -0.2, 0.79);
    pcw.find_nearest_point();
    EXPECT_EQ(1, pcw.get_tp_index());
    EXPECT_NEAR(0.785, pcw.calc_desired_heading(), 0.01);

    // 2nd test - vehicle heading almost same as TP1, lookahead defined
    pcw.unit_test_set_heading_lookahead(2);
    pcw.unit_test_set_pose(-0.9, -0.2, 0.79);
    pcw.find_nearest_point();
    EXPECT_EQ(1, pcw.get_tp_index());
    EXPECT_NEAR(0.611, pcw.calc_desired_heading(), 0.01);

    // 3rd test - vehicle heading farther right, no lookahead
    pcw.unit_test_set_heading_lookahead(0);
    pcw.unit_test_set_pose(-0.9, -0.2, 0.6);
    pcw.find_nearest_point();
    EXPECT_EQ(1, pcw.get_tp_index());
    EXPECT_NEAR(0.785, pcw.calc_desired_heading(), 0.01);

    // 4th test - vehicle heading farther right, lookahead defined
    pcw.unit_test_set_heading_lookahead(2);
    pcw.unit_test_set_pose(-0.9, -0.2, 0.6);
    pcw.find_nearest_point();
    EXPECT_EQ(1, pcw.get_tp_index());
    EXPECT_NEAR(0.611, pcw.calc_desired_heading(), 0.01);
}

/*
TEST(PlatoonControlWorkerTest, test1)
{
    platoon_control_pid0::PlatoonControlWorker pcw;
    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 2.0;
    pcw.generateSpeed(point);
    EXPECT_NEAR(0, pcw.speedCmd_, 0.1);
}

TEST(PlatoonControlWorkerTest, test11)
{
    platoon_control_pid0::PlatoonLeaderInfo leader;
    platoon_control_pid0::PlatoonControlWorker pcw;
    leader.staticId = "";
    leader.leaderIndex = 0;
    leader.NumberOfVehicleInFront = 1;
    pcw.setLeader(leader);
    cav_msgs::TrajectoryPlanPoint point;
    point.x = 30.0;
    point.y = 20.0;
    pcw.currentSpeed = 10.0;
    pcw.lastCmdSpeed = 10;
    pcw.generateSpeed(point);
    EXPECT_NEAR(10.0, pcw.getLastSpeedCommand(), 0.5);
}

TEST(PlatoonControlWorkerTest, test2)
{

    platoon_control_pid0::PlatoonControlWorker pcw;
    platoon_control_pid0::PlatoonLeaderInfo leader;
    leader.commandSpeed = 10;
    leader.vehicleSpeed = 10;
    leader.vehiclePosition = 50;
    leader.staticId = "id";
    leader.leaderIndex = 0;
    leader.NumberOfVehicleInFront = 1;
    pcw.setLeader(leader);

    cav_msgs::TrajectoryPlanPoint point;
    point.x = 30.0;
    point.y = 20.0;
    pcw.currentSpeed = 10.0;
    pcw.lastCmdSpeed = 10;
    pcw.generateSpeed(point);
    EXPECT_NEAR(9.75, pcw.getLastSpeedCommand(), 0.5);


    cav_msgs::TrajectoryPlanPoint point2;
    point2.x = 30.0;
    point2.y = 40.0;
    pcw.generateSpeed(point2);
    EXPECT_NEAR(10, pcw.getLastSpeedCommand(), 0.5);

    cav_msgs::TrajectoryPlanPoint point3;
    point3.x = 50.0;
    point3.y = 60.0;
    pcw.generateSpeed(point3);
    EXPECT_NEAR(10.25, pcw.getLastSpeedCommand(), 0.5);

}

TEST(PlatoonControlWorkerTest, test3)
{

    platoon_control_pid0::PlatoonControlWorker pcw;
    platoon_control_pid0::PlatoonLeaderInfo leader;
    leader.commandSpeed = 10;
    leader.vehicleSpeed = 10;
    leader.vehiclePosition = 50;
    leader.staticId = "id";
    leader.leaderIndex = 0;
    leader.NumberOfVehicleInFront = 2;
    pcw.setLeader(leader);

    cav_msgs::TrajectoryPlanPoint point;
    point.x = 30.0;
    point.y = 15.0;
    pcw.currentSpeed = 10.0;
    pcw.lastCmdSpeed = 10;
    pcw.generateSpeed(point);
    EXPECT_NEAR(10.25, pcw.getLastSpeedCommand(), 0.5);


    cav_msgs::TrajectoryPlanPoint point2;
    point2.x = 50.0;
    point2.y = 60.0;
    pcw.platoon_leader.vehiclePosition = 51;
    pcw.generateSpeed(point2);
    EXPECT_NEAR(10.5, pcw.getLastSpeedCommand(), 0.5);

    cav_msgs::TrajectoryPlanPoint point3;
    point3.x = 50.0;
    point3.y = 60.0;
    pcw.platoon_leader.vehiclePosition = 49;
    pcw.generateSpeed(point3);
    EXPECT_NEAR(10.25, pcw.getLastSpeedCommand(), 0.5);

    }

TEST(PlatoonControlWorkerTest, test_steer)
{
    platoon_control_pid0::PlatoonControlWorker pcw;
    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 2.0;
    pcw.generateSteer(point);
    EXPECT_NEAR(0, pcw.steerCmd_, 0.1);
}
*/