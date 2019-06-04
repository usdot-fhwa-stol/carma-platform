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

    geometry_msgs::PoseStamped pose;
    cav_msgs::TrajectoryPlanPoint tpp;

    double current_time = 0;

    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;

    tpp.x = 10;
    tpp.y = 10;
    tpp.target_time = 10000;

    pure_pursuit_wrapper::PurePursuitWrapperWorker ppww;

    autoware_msgs::Waypoint waypoint = ppww.TrajectoryPlanPointToWaypointConverter(current_time, pose, tpp);

    double v_x = waypoint.twist.twist.linear.x;
    double correct_v_x = -1;

    EXPECT_EQ(v_x, correct_v_x);
}

int main(int argc, char**argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
