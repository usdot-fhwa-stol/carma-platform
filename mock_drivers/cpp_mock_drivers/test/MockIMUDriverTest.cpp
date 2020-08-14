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

#include <gmock/gmock.h>
#include "cpp_mock_drivers/MockIMUDriver.h"
#include <cav_msgs/BagData.h>


namespace mock_drivers{

    TEST(MockIMUDriver, Constructor){
        MockIMUDriver d1;
        MockIMUDriver d2(false);
        MockIMUDriver d3(true);

        ASSERT_EQ(d1.getMockDriverNode().isDummy(), false);
        ASSERT_EQ(d2.getMockDriverNode().isDummy(), false);
        ASSERT_EQ(d3.getMockDriverNode().isDummy(), true);
    }

    TEST(MockIMUDriver, pubCallbacks){
        MockIMUDriver d(true);

        cav_msgs::BagData::ConstPtr test_msg_ptr(new cav_msgs::BagData());
        ASSERT_TRUE((*test_msg_ptr).header.stamp.isZero());

        d.parserCB(test_msg_ptr);

        std::vector<std::string> test_str_vector = d.getMockDriverNode().getTopics();
        std::vector<ros::Time> test_time_vector = d.getMockDriverNode().getTimeStamps();

        ASSERT_EQ(test_str_vector[0], "raw_data");

        // Give a range because the nanoseconds go too fast for the test to pass if its assert equal
        ros::Duration range(0.001);
        
        EXPECT_TRUE((test_time_vector[0] > ros::Time::now() - range) && (test_time_vector[0] < ros::Time::now() + range));
    }

    TEST(MockIMUDriver, run){
        MockIMUDriver d(true);
        ASSERT_EQ(d.run(), 0);
    }
}