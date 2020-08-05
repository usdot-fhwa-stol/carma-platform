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

#include "TestMockDriver.h"
#include "std_msgs/String.h"


namespace mock_drivers{

    void TestMockDriver::chatterCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("I heard: [%s]", msg->data.c_str());
    }

    TestMockDriver::TestMockDriver(){
        MockDriverNode mock_driver_node_();

            // ROSComms<std_msgs::String> test_pub_;
            // ROSComms<std_msgs::String, const std_msgs::String::ConstPtr&> test_sub_;
            // CommTypes ct, bool latch, int qs, string t

        ROSComms<std_msgs::String> test_pub_(CommTypes::pub, false, 100, "mock_pub");
        
        // std::function<void(const std_msgs::String::ConstPtr&)> mock_ptr = std::bind(&TestMockDriver::chatterCallback, this, std::placeholders::_1);
        // ROSComms<const std_msgs::String::ConstPtr&> test_sub_(mock_ptr, CommTypes::sub, false, 100, "mock_sub");
    }

    int TestMockDriver::run(){
        
        boost::shared_ptr<ROSComms<std_msgs::String>> pub_ptr(&test_pub_);
        // boost::shared_ptr<ROSComms<const std_msgs::String::ConstPtr&>> sub_ptr(&test_sub_);

        mock_driver_node_.addComms<boost::shared_ptr<ROSComms<std_msgs::String>>>(pub_ptr);
        // mock_driver_node_.addComms<boost::shared_ptr<ROSComms<const std_msgs::String::ConstPtr&>>>(sub_ptr);

        // mock_driver_node_.spin(2);
        return 0;
    }
}