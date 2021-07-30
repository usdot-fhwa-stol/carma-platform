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

#include "mobilitypath_publisher.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(MobilityPathPublicationTest, test1)
{
    ros::Time::init();
    cav_msgs::TrajectoryPlan plan;
    cav_msgs::TrajectoryPlanPoint p1;
    p1.x = 20.0;
    p1.y = 0.0;
    p1.target_time = ros::Time(0,0);

    cav_msgs::TrajectoryPlanPoint p2;
    p2.x = 19.0;
    p2.y = 0.0;
    p2.target_time = ros::Time(0,1);

    plan.trajectory_points.push_back(p1);
    plan.trajectory_points.push_back(p2);
    
    mobilitypath_publisher::MobilityPathPublication worker;

    std::string base_proj = "+proj=tmerc +lat_0=0.0 +lon_0=0.0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                        "+no_defs";
    std_msgs::String msg;
    msg.data = base_proj;
    std_msgs::StringConstPtr msg_ptr(new std_msgs::String(msg));
    worker.georeference_cb(msg_ptr);  // Set projection

    auto res = worker.mobilityPathMessageGenerator(plan);
    ASSERT_EQ(1, res.trajectory.offsets.size());

    ASSERT_EQ(637813699.0, res.trajectory.location.ecef_x);
    ASSERT_EQ(1999.0, res.trajectory.location.ecef_y);
    ASSERT_EQ(0.0, res.trajectory.location.ecef_z);

    ASSERT_EQ(res.trajectory.offsets[0].offset_x, 0);
    ASSERT_EQ(res.trajectory.offsets[0].offset_y, -100);
    ASSERT_EQ(res.trajectory.offsets[0].offset_z, 0);
}

TEST(MobilityPathPublicationTest, test2)
{
    ros::Time::init();
    cav_msgs::TrajectoryPlan plan;
    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1;
    point.y = 1;
    point.target_time = ros::Time(0,1);
    plan.trajectory_points.push_back(point);

    mobilitypath_publisher::MobilityPathPublication worker;

    auto res = worker.mobilityPathMessageGenerator(plan);

    // Check values are unset as georeference was not provided
    ASSERT_EQ(0, res.trajectory.location.ecef_x);
    ASSERT_EQ(0, res.trajectory.location.ecef_y);
    ASSERT_EQ(0, res.trajectory.location.ecef_z);
    EXPECT_EQ(0, res.trajectory.offsets.size());
    
}


// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}