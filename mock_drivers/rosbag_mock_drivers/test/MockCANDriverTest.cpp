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
#include "rosbag_mock_drivers/MockCANDriver.h"
#include <carma_simulation_msgs/BagData.h>


namespace mock_drivers{

    TEST(MockCANDriver, Constructor){
        MockCANDriver d1;
        MockCANDriver d2(false);
        MockCANDriver d3(true);

        ASSERT_EQ(d1.getMockDriverNode().isDummy(), false);
        ASSERT_EQ(d2.getMockDriverNode().isDummy(), false);
        ASSERT_EQ(d3.getMockDriverNode().isDummy(), true);
    }

    TEST(MockCANDriver, pubCallbacks){
        MockCANDriver d(true);

        carma_simulation_msgs::BagData::ConstPtr test_msg_ptr(new carma_simulation_msgs::BagData());
        ASSERT_TRUE((*test_msg_ptr).header.stamp.isZero());

        d.parserCB(test_msg_ptr);

        std::vector<std::string> test_str_vector = d.getMockDriverNode().getTopics();
        std::vector<ros::Time> test_time_vector = d.getMockDriverNode().getTimeStamps();

        ASSERT_EQ(test_str_vector[0], "/hardware_interface/can/acc_engaged");
        ASSERT_EQ(test_str_vector[1], "/hardware_interface/can/acceleration");
        ASSERT_EQ(test_str_vector[2], "/hardware_interface/can/antilock_brakes_active");
        ASSERT_EQ(test_str_vector[3], "/hardware_interface/can/brake_lights");
        ASSERT_EQ(test_str_vector[4], "/hardware_interface/can/brake_position");
        ASSERT_EQ(test_str_vector[5], "/hardware_interface/can/engine_speed");
        ASSERT_EQ(test_str_vector[6], "/hardware_interface/can/fuel_flow");
        ASSERT_EQ(test_str_vector[7], "/hardware_interface/can/odometer");
        ASSERT_EQ(test_str_vector[8], "/hardware_interface/can/parking_brake");
        ASSERT_EQ(test_str_vector[9], "/hardware_interface/can/speed");
        ASSERT_EQ(test_str_vector[10], "/hardware_interface/can/stability_ctrl_active");
        ASSERT_EQ(test_str_vector[11], "/hardware_interface/can/stability_ctrl_enabled");
        ASSERT_EQ(test_str_vector[12], "/hardware_interface/can/steering_wheel_angle");
        ASSERT_EQ(test_str_vector[13], "/hardware_interface/can/throttle_position");
        ASSERT_EQ(test_str_vector[14], "/hardware_interface/can/traction_ctrl_active");
        ASSERT_EQ(test_str_vector[15], "/hardware_interface/can/traction_ctrl_enabled");
        ASSERT_EQ(test_str_vector[16], "/hardware_interface/can/transmission_state");
        ASSERT_EQ(test_str_vector[17], "/hardware_interface/can/turn_signal_state");
        ASSERT_EQ(test_str_vector[18], "/hardware_interface/can/vehicle/twist");
        ASSERT_EQ(test_str_vector[19], "/hardware_interface/can/vehicle_status");
        ASSERT_EQ(test_str_vector[20], "/hardware_interface/can/velocity_accel");

        // Give a range because the nanoseconds go too fast for the test to pass if its assert equal
        ros::Duration range(0.0001);
        
        EXPECT_TRUE((test_time_vector[0] > ros::Time::now() - range) && (test_time_vector[0] < ros::Time::now() + range));
        EXPECT_TRUE((test_time_vector[1] > ros::Time::now() - range) && (test_time_vector[1] < ros::Time::now() + range));
        EXPECT_TRUE((test_time_vector[2] > ros::Time::now() - range) && (test_time_vector[2] < ros::Time::now() + range));
    }

    TEST(MockCANDriver, driver_discovery){
        MockCANDriver d(true);

        ASSERT_TRUE(d.driverDiscovery());
    }

    TEST(MockCANDriver, run){
        MockCANDriver d(true);

        ASSERT_EQ(d.run(), 0);
    }
}