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

#include "unobstructed_lanechange.h"
#include <gtest/gtest.h>
#include <ros/ros.h>



TEST(UnobstructedLaneChangePluginTest, testCreateLaneChangeTrajectory1)
{
    std::vector<double> start_point = {0.0, 0.0};
    std::vector<double> end_point = {10.0, 3.0};
    unobstructed_lanechange::UnobstructedLaneChangePlugin lc;
    std::vector<cav_msgs::TrajectoryPlanPoint> res = lc.create_lanechange_trajectory(start_point, end_point);
    EXPECT_EQ(60, res.size());
    EXPECT_NEAR(0.0, res[0].target_time, 0.01);
    EXPECT_NEAR(0.0, res[0].x, 0.05);
    EXPECT_NEAR(0.0, res[0].y, 0.05);
    EXPECT_NEAR(2.5, res[15].x, 0.05);
    EXPECT_NEAR(0.75, res[15].y, 0.05);
    EXPECT_NEAR(5.0, res[30].x, 0.05);
    EXPECT_NEAR(1.5, res[30].y, 0.05);
    EXPECT_NEAR(7.5, res[45].x, 0.05);
    EXPECT_NEAR(2.25, res[45].y, 0.05);
    EXPECT_NEAR(9.85, res[59].x, 0.05);
    EXPECT_NEAR(2.95, res[59].y, 0.05);
    EXPECT_NEAR(1.0, res[59].target_time, 0.01);
}


// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}