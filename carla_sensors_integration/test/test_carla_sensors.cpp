/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include <carla_sensors_integration.h>

TEST(CarlaSensorsTest, testLIDARCallback)
{
    carla_sensors::CarlaSensors test1;

    sensor_msgs::PointCloud2 lidar_test;

    lidar_test.data.push_back(2);
    lidar_test.data.push_back(4);
    lidar_test.data.push_back(6);

    sensor_msgs::PointField field1, field2;

    field1.datatype = 1;
    field1.name = "Field1";

    field2.datatype = 3;
    field2.name = "Field2";

    lidar_test.fields.push_back(field1);
    lidar_test.fields.push_back(field2);

    lidar_test.height = 1;
    lidar_test.is_bigendian = true;
    lidar_test.is_dense = true;
    lidar_test.point_step = 4;
    lidar_test.row_step = 1;
    lidar_test.width = 3;

    test1.point_cloud_cb(lidar_test);
    auto result_msg = test1.get_lidar_msg();

    ASSERT_FALSE(result_msg.data.size() == 0);
    ASSERT_EQ(result_msg.data.size(), 3);
    ASSERT_EQ(result_msg.fields.size, 0);
    


    
}

TEST(CarlaSensorsTest, testImageRawCallback)
{
       carla_sensors::CarlaSensors test;

    sensor_msgs::Image raw_image_msg;

    raw_image_msg.data.push_back(12);
    raw_image_msg.data.push_back(14);
    raw_image_msg.data.push_back(16);
    raw_image_msg.data.push_back(18);

    raw_image_msg.encoding = "TEST";
    raw_image_msg.height = 4;
    raw_image_msg.is_bigendian = true;
    raw_image_msg.width = 5;

    test.image_raw_cb(raw_image_msg);

    auto result_msg = test.get_image_raw_msg();

    ASSERT_EQ(result_msg.data.size(), 4);
    ASSERT_EQ(result_msg.encoding, "TEST");
    ASSERT_EQ(result_msg.height, 4);
    ASSERT_EQ(result_msg.width, 5);
    ASSERT_TRUE(result_msg.is_bigendian);

   


}

TEST(CarlaSensorsTest, testImageColorCallback)
{

     carla_sensors::CarlaSensors test;

    sensor_msgs::Image color_image_msg;

    color_image_msg.data.push_back(12);
    color_image_msg.data.push_back(14);
    color_image_msg.data.push_back(16);
    color_image_msg.data.push_back(18);

    color_image_msg.encoding = "TEST";
    color_image_msg.height = 4;
    color_image_msg.is_bigendian = true;
    color_image_msg.width = 5;

    test.image_color_cb(color_image_msg);

    auto result_msg = test.get_image_color_msg();

    ASSERT_EQ(result_msg.data.size(), 4);
    ASSERT_EQ(result_msg.encoding, "TEST");
    ASSERT_EQ(result_msg.height, 4);
    ASSERT_EQ(result_msg.width, 5);
    ASSERT_TRUE(result_msg.is_bigendian);
    
}

TEST(CarlaSensorsTest, testImageRectCallback)
{
   carla_sensors::CarlaSensors test;

    sensor_msgs::Image rect_image_msg;

    rect_image_msg.data.push_back(12);
    rect_image_msg.data.push_back(14);
    rect_image_msg.data.push_back(16);
    rect_image_msg.data.push_back(18);

    rect_image_msg.encoding = "TEST";
    rect_image_msg.height = 4;
    rect_image_msg.is_bigendian = true;
    rect_image_msg.width = 5;

    test.image_rect_cb(rect_image_msg);

    auto result_msg = test.get_image_raw_msg();

    ASSERT_EQ(result_msg.data.size(), 4);
    ASSERT_EQ(result_msg.encoding, "TEST");
    ASSERT_EQ(result_msg.height, 4);
    ASSERT_EQ(result_msg.width, 5);
    ASSERT_TRUE(result_msg.is_bigendian);
}

TEST(CarlaSensorsTest, testGNSSCallback)
{
    carla_sensors::CarlaSensors test1;

    sensor_msgs::NavSatFix gnss_test;
    gnss_test.altitude = 5.0;
    gnss_test.latitude = 54.0;
    gnss_test.longitude = 65.0;
    gnss_test.position_covariance = {23.0, 24.0, 25.0};
    gnss_test.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;

    test1.gnss_fixed_fused_cb(gnss_test);

    auto result_msg = test1.get_gnss_fixed_msg();

    ASSERT_EQ(result_msg.altitude, 5.0);
    ASSERT_EQ(result_msg.latitude, 54.0);
    ASSERT_EQ(result_msg.longitude, 65.0);

    ROS_INFO_STREAM("TESTS PASSED");


}