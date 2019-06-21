/*
 * Copyright (C) 2018-2019 LEIDOS.
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

#include "pure_pursuit_wrapper/pure_pursuit_wrapper_worker.hpp"

#include <gtest/gtest.h>
#include <iostream>


TEST(TrajectoryPlanPointToWaypointConverterTest, test1)
{  

    geometry_msgs::PoseStamped current_pose;
    cav_msgs::TrajectoryPlanPoint tpp;
    pure_pursuit_wrapper::PurePursuitWrapperWorker ppww;
    autoware_msgs::Waypoint waypoint;
    double current_time;
    
    current_time = 0;
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;

    tpp.x = 10;
    tpp.y = 10;
    tpp.target_time = 10000;

    waypoint = ppww.TrajectoryPlanPointToWaypointConverter(current_time, current_pose, tpp);

    double v_x = waypoint.twist.twist.linear.x;
    double correct_v_x = 3.6;

    EXPECT_EQ(correct_v_x, v_x);

    current_time = 0;
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;

    tpp.x = 0;
    tpp.y = 0;
    tpp.target_time = 0;

    waypoint = ppww.TrajectoryPlanPointToWaypointConverter(current_time, current_pose, tpp);

    v_x = waypoint.twist.twist.linear.x;
    correct_v_x = 0;

    EXPECT_EQ(correct_v_x, v_x);

    current_time = 0;
    current_pose.pose.position.x = 10.0;
    current_pose.pose.position.y = 10.0;

    tpp.x = 0;
    tpp.y = 0;
    tpp.target_time = 10000;

    waypoint = ppww.TrajectoryPlanPointToWaypointConverter(current_time, current_pose, tpp);

    v_x = waypoint.twist.twist.linear.x;
    correct_v_x = -3.6;

    EXPECT_EQ(correct_v_x, v_x);

}

int main(int argc, char**argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
