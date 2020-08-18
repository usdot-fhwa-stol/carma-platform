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
#include "cpp_mock_drivers/MockControllerDriver.h"
#include <carma_simulation_msgs/BagData.h>


namespace mock_drivers{

    TEST(MockControllerDriver, Constructor){
        MockControllerDriver d1;
        MockControllerDriver d2(false);
        MockControllerDriver d3(true);

        ASSERT_EQ(d1.getMockDriverNode().isDummy(), false);
        ASSERT_EQ(d2.getMockDriverNode().isDummy(), false);
        ASSERT_EQ(d3.getMockDriverNode().isDummy(), true);
    }

    TEST(MockControllerDriver, pubCallbacks){
        MockControllerDriver d(true);

        carma_simulation_msgs::BagData::ConstPtr test_msg_ptr(new carma_simulation_msgs::BagData());
        ASSERT_TRUE((*test_msg_ptr).header.stamp.isZero());

        d.parserCB(test_msg_ptr);

        std::vector<std::string> test_str_vector = d.getMockDriverNode().getTopics();
        std::vector<ros::Time> test_time_vector = d.getMockDriverNode().getTimeStamps();

        ASSERT_EQ(test_str_vector[0], "robot_status");
    }

    TEST(MockControllerDriver, srvCallbacks){
        MockControllerDriver d(true);
        cav_srvs::SetEnableRobotic srv;

        EXPECT_TRUE(d.enableRoboticSrv(srv.request, srv.response));
    }

    TEST(MockControllerDriver, run){
        MockControllerDriver d(true);
        ASSERT_EQ(d.run(), 0);
    }
}