/*
 * Copyright (C) 2022 LEIDOS.
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

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include "carma_cloud_client/carma_cloud_client_node.hpp"


TEST(Testcarma_cloud_client, test_xml_conversion){

    rclcpp::NodeOptions options;
    carma_cloud_client::CarmaCloudClient plugin(options);

    carma_v2x_msgs::msg::TrafficControlRequest request_msg;
    request_msg.choice = carma_v2x_msgs::msg::TrafficControlRequest::TCRV01;
    request_msg.tcr_v01.reqseq = 123;
    request_msg.tcr_v01.reqid.id = {0, 1, 2, 3, 4, 5, 6, 7};
    carma_v2x_msgs::msg::TrafficControlBounds b1;
    b1.oldest = b1.oldest.set__nanosec(500000000);
    b1.reflat = 45.0;// 45 deg
    b1.reflon = 40.0;// 40 deg
    
    for(int i = 0; i < 3; i++)
    {
        b1.offsets[i].deltax = 1000;
        b1.offsets[i].deltay = 100;
    }

    request_msg.tcr_v01.bounds.push_back(b1);
    char xml_str[10000]; 
    plugin.XMLconversion(xml_str, request_msg);
    // The resulting xml is printed.
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
} 