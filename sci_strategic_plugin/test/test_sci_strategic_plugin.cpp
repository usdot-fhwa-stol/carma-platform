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

#include <sci_strategic_plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>

TEST(SCIStrategicPlugin, UnitTest1)
{
    ros::CARMANodeHandle nh;
    ros::Publisher mob_op_pub = nh.advertise<cav_msgs::MobilityOperation>("incoming_mobility_operation", 5);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 5);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    ros::spinOnce(); 
    
    EXPECT_EQ(1, mob_op_pub.getNumSubscribers());
    EXPECT_EQ(1, pose_pub.getNumSubscribers());
}

int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_sci_strategic_plugin");
    auto res = RUN_ALL_TESTS();
    return res;
}