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

#include "vehicle_status_generator_worker.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(VehicleStatusWorkerTest, testMsgCount)
{
    vehicle_status_generator::VehicleStatusGeneratorWorker worker;
    EXPECT_EQ(0, worker.getNextMsgCount());
    for(int i = 1; i < 127; i++)
    {
        worker.getNextMsgCount();
    }
    EXPECT_EQ(127, worker.getNextMsgCount());
    EXPECT_EQ(0, worker.getNextMsgCount());
}

TEST(VehicleStatusWorkerTest, testMsgId)
{
    vehicle_status_generator::VehicleStatusGeneratorWorker worker;
    ros::Time time1(10, 0);
    std::vector<uint8_t> msgId1 = worker.getMsgId(time1);
    ros::Time time2(11, 0);
    std::vector<uint8_t> msgId2 = worker.getMsgId(time2);
    ros::Time time3(310, 0);
    std::vector<uint8_t> msgId3 = worker.getMsgId(time3);
    EXPECT_TRUE(msgId1 == msgId2);
    EXPECT_TRUE(msgId2 != msgId3);
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
