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
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <lanelet2_core/utility/Units.h>
#include <chrono>
#include <pcl/common/generate.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include "points_map_filter/points_map_filter_node.hpp"

namespace {
using namespace lanelet::units::literals;
lanelet::Lanelet getLanelet(lanelet::LineString3d &left_ls, lanelet::LineString3d &right_ls,
                            const lanelet::Attribute &left_sub_type = lanelet::AttributeValueString::SolidSolid,
                            const lanelet::Attribute &right_sub_type = lanelet::AttributeValueString::Solid)
{
    left_ls.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
    left_ls.attributes()[lanelet::AttributeName::Subtype] = left_sub_type;

    right_ls.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
    right_ls.attributes()[lanelet::AttributeName::Subtype] = right_sub_type;

    lanelet::Lanelet ll;
    ll.setId(lanelet::utils::getId());
    ll.setLeftBound(left_ls);
    ll.setRightBound(right_ls);

    ll.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::Lanelet;
    ll.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
    ll.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
    ll.attributes()[lanelet::AttributeName::OneWay] = "yes";
    ll.attributes()[lanelet::AttributeName::Dynamic] = "no";

    return ll;
}

/*
 * NOTE: The following function is taken from the Autoware.ai system's 
 *       lanelet2_extension library and is ported here as a ROS2 version
 *       Once this functionality is fully ported to ROS2 this function 
 *       can be replaced with a library call
 * 
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 */

void toBinMsg(const lanelet::LaneletMapPtr& map, autoware_lanelet2_msgs::msg::MapBin* msg)
{
  if (msg == nullptr)
  {
    return;
  }

  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *map;
  auto id_counter = lanelet::utils::getId();
  oa << id_counter;

  std::string data_str(ss.str());

  msg->data.clear();
  msg->data.assign(data_str.begin(), data_str.end());
}

}



TEST(Testpoints_map_filter, filter_test)
{
    std::vector<lanelet::Point3d> left_pts;
    std::vector<lanelet::Point3d> right_pts;

    lanelet::Point3d p1(lanelet::utils::getId(), 0, 0, 0);
    lanelet::Point3d p2(lanelet::utils::getId(), 0, 3, 0);
    lanelet::Point3d p3(lanelet::utils::getId(), 3, 3, 0);
    lanelet::Point3d p4(lanelet::utils::getId(), 3, 0, 0);

    //expecting cells with positions [-1.5, 1.5] [1.5, 4.5]

    lanelet::LineString3d left_ls(lanelet::utils::getId(), {p1, p2});

    lanelet::LineString3d right_ls(lanelet::utils::getId(), {p4, p3});

    auto ll = getLanelet(left_ls, right_ls);

    lanelet::LaneletMapPtr map = lanelet::utils::createMap({ll}, {});



    std::vector<std::string> remaps; // Remaps to keep topics separate from other tests
    remaps.push_back("--ros-args");
    remaps.push_back("-r");
    remaps.push_back("filtered_points:=/points_filter_test/filtered_points");
    remaps.push_back("-r");
    remaps.push_back("points_raw:=/points_filter_test/points_raw");
    remaps.push_back("-r");
    remaps.push_back("lanelet2_map:=/points_filter_test/lanelet2_map");
    remaps.push_back("-r");
    remaps.push_back("__node:=points_filter_test_points_map_filter");

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.arguments(remaps);

    auto worker_node = std::make_shared<points_map_filter::Node>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime



    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    for (double x = -2.9; x < 7; x += 1.5) {
        pcl::PointXYZI p;
        p.x = x;
        p.y = x;
        p.z = 0;
        cloud->points.push_back(p);
    }

    ASSERT_EQ(cloud->points.size(), 7u);


    std::unique_ptr<autoware_lanelet2_msgs::msg::MapBin> map_msg = std::make_unique<autoware_lanelet2_msgs::msg::MapBin>();
    
    toBinMsg(map, map_msg.get());

    worker_node->map_callback(move(map_msg)); // Manually drive topic callbacks



    sensor_msgs::msg::PointCloud2 result;
    auto sub = worker_node->create_subscription<sensor_msgs::msg::PointCloud2>("/points_filter_test/filtered_points", 1,
        [&](sensor_msgs::msg::PointCloud2::UniquePtr msg)
        {
            result = *msg;
        });


    sensor_msgs::msg::PointCloud2 input_points;
    pcl::toROSMsg(*cloud, input_points);

    std::unique_ptr<sensor_msgs::msg::PointCloud2> input_points_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>(input_points);

    worker_node->points_callback(move(input_points_ptr));

    // Provide some time for publication to occur
    std::this_thread::sleep_for(std::chrono::seconds(2));

    rclcpp::spin_some(worker_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger

    auto output_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    pcl::moveFromROSMsg(result, *output_cloud);

    ASSERT_EQ(output_cloud->points.size(), 4u);

    for (auto p : output_cloud->points) {
        std::cerr << "Point" << std::endl;
        float x = p.x;
        float y = p.y;
        float z = 0.0;

        if (!( -0.00001 <= x && x <= 6.0000001
            && -0.00001 <= y && y <= 6.0000001
            && z == 0.0))
        {
            FAIL() << "Detected points outside of map's approximate bounds";
        }
        
    }


}

TEST(Testpoints_map_filter, linker_test)
{
    std::cerr << "TEST" << std::endl;
    std::vector<lanelet::Point3d> left_pts;
    std::vector<lanelet::Point3d> right_pts;

    lanelet::Point3d p1(lanelet::utils::getId(), 0, 0, 0);
    lanelet::Point3d p2(lanelet::utils::getId(), 0, 3, 0);
    lanelet::Point3d p3(lanelet::utils::getId(), 3, 3, 0);
    lanelet::Point3d p4(lanelet::utils::getId(), 3, 0, 0);

    //expecting cells with positions [-1.5, 1.5] [1.5, 4.5]

    lanelet::LineString3d left_ls(lanelet::utils::getId(), {p1, p2});

    lanelet::LineString3d right_ls(lanelet::utils::getId(), {p4, p3});

    auto ll = getLanelet(left_ls, right_ls);

    lanelet::LaneletMapPtr map = lanelet::utils::createMap({ll}, {});


    std::vector<std::string> remaps; // Remaps to keep topics separate from other tests
    remaps.push_back("--ros-args");
    remaps.push_back("-r");
    remaps.push_back("filtered_points:=/points_filter_test/filtered_points2");
    remaps.push_back("-r");
    remaps.push_back("points_raw:=/points_filter_test/points_raw2");
    remaps.push_back("-r");
    remaps.push_back("lanelet2_map:=/points_filter_test/lanelet2_map2");
    remaps.push_back("-r");
    remaps.push_back("__node:=points_filter_test_points_map_filter2");

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.arguments(remaps);

    auto worker_node = std::make_shared<points_map_filter::Node>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime



    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    for (double x = -2.9; x < 7; x += 1.5) {
        pcl::PointXYZI p;
        p.x = x;
        p.y = x;
        p.z = 0;
        cloud->points.push_back(p);
    }

    ASSERT_EQ(cloud->points.size(), 7u);


    std::unique_ptr<autoware_lanelet2_msgs::msg::MapBin> map_msg = std::make_unique<autoware_lanelet2_msgs::msg::MapBin>();
    
    toBinMsg(map, map_msg.get());

    map_msg->data = {
        22, 0, 0, 0, 0, 0, 0, 0, 115, 101, 114, 105, 97, 108, 105, 122, 97, 116, 105, 111, 110, 58, 58, 97, 114, 99, 104, 105, 118, 101, 17, 0, 4, 8, 4, 8, 1, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 3, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -22, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 64, 0, 0, 0, 0, 0, 0, 8, 64, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 1, 0, 0, 0, -21, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 2, 0, 0, 0, -23, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 64, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 0, 0, 0, -24, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 7, 0, 1, 0, 0, 0, 0, 4, 0, 0, 0, -19, 3, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 115, 117, 98, 116, 121, 112, 101, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 115, 111, 108, 105, 100, 4, 0, 0, 0, 0, 0, 0, 0, 116, 121, 112, 101, 9, 0, 0, 0, 0, 0, 0, 0, 108, 105, 110, 101, 95, 116, 104, 105, 110, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 1, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 7, 0, 5, 0, 0, 0, -20, 3, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 115, 117, 98, 116, 121, 112, 101, 11, 0, 0, 0, 0, 0, 0, 0, 115, 111, 108, 105, 100, 95, 115, 111, 108, 105, 100, 4, 0, 0, 0, 0, 0, 0, 0, 116, 121, 112, 101, 9, 0, 0, 0, 0, 0, 0, 0, 108, 105, 110, 101, 95, 116, 104, 105, 110, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 0, 0, 0, 3, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 13, 0, 1, 0, 0, 0, 0, 6, 0, 0, 0, -18, 3, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 100, 121, 110, 97, 109, 105, 99, 2, 0, 0, 0, 0, 0, 0, 0, 110, 111, 8, 0, 0, 0, 0, 0, 0, 0, 108, 111, 99, 97, 116, 105, 111, 110, 5, 0, 0, 0, 0, 0, 0, 0, 117, 114, 98, 97, 110, 7, 0, 0, 0, 0, 0, 0, 0, 111, 110, 101, 95, 119, 97, 121, 3, 0, 0, 0, 0, 0, 0, 0, 121, 101, 115, 7, 0, 0, 0, 0, 0, 0, 0, 115, 117, 98, 116, 121, 112, 101, 4, 0, 0, 0, 0, 0, 0, 0, 114, 111, 97, 100, 4, 0, 0, 0, 0, 0, 0, 0, 116, 121, 112, 101, 7, 0, 0, 0, 0, 0, 0, 0, 108, 97, 110, 101, 108, 101, 116, 0, 7, 0, 5, 0, 0, 0, 0, 7, 0, 4, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, -17, 3, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 16, 0, 1, 0, 0, 0, 0, 7, 0, 0, 0, -17, 3, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 108, 105, 109, 105, 116, 9, 0, 0, 0, 0, 0, 0, 0, 51, 50, 46, 49, 56, 54, 56, 56, 48, 23, 0, 0, 0, 0, 0, 0, 0, 112, 97, 114, 116, 105, 99, 105, 112, 97, 110, 116, 58, 118, 101, 104, 105, 99, 108, 101, 58, 99, 97, 114, 3, 0, 0, 0, 0, 0, 0, 0, 121, 101, 115, 6, 0, 0, 0, 0, 0, 0, 0, 114, 101, 97, 115, 111, 110, 2, 0, 0, 0, 0, 0, 0, 0, 78, 65, 7, 0, 0, 0, 0, 0, 0, 0, 115, 117, 98, 116, 121, 112, 101, 19, 0, 0, 0, 0, 0, 0, 0, 100, 105, 103, 105, 116, 97, 108, 95, 115, 112, 101, 101, 100, 95, 108, 105, 109, 105, 116, 4, 0, 0, 0, 0, 0, 0, 0, 116, 121, 112, 101, 18, 0, 0, 0, 0, 0, 0, 0, 114, 101, 103, 117, 108, 97, 116, 111, 114, 121, 95, 101, 108, 101, 109, 101, 110, 116, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 114, 101, 102, 101, 114, 115, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 8, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 6, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, -17, 3, 0, 0, 0, 0, 0, 0, -16, 3, 0, 0, 0, 0, 0, 0
    };

    //std::cerr << std::endl << std::endl << std::to_string(map_msg->data) << std::endl;

    worker_node->map_callback(move(map_msg)); // Manually drive topic callbacks



    sensor_msgs::msg::PointCloud2 result;
    auto sub = worker_node->create_subscription<sensor_msgs::msg::PointCloud2>("/points_filter_test/filtered_points2", 1,
        [&](sensor_msgs::msg::PointCloud2::UniquePtr msg)
        {
            result = *msg;
        });


    sensor_msgs::msg::PointCloud2 input_points;
    pcl::toROSMsg(*cloud, input_points);

    std::unique_ptr<sensor_msgs::msg::PointCloud2> input_points_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>(input_points);

    worker_node->points_callback(move(input_points_ptr));

    // Provide some time for publication to occur
    std::this_thread::sleep_for(std::chrono::seconds(2));

    rclcpp::spin_some(worker_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger

    auto output_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    pcl::moveFromROSMsg(result, *output_cloud);

    ASSERT_EQ(output_cloud->points.size(), 4u);

    for (auto p : output_cloud->points) {
        std::cerr << "Point" << std::endl;
        float x = p.x;
        float y = p.y;
        float z = 0.0;

        if (!( -0.00001 <= x && x <= 6.0000001
            && -0.00001 <= y && y <= 6.0000001
            && z == 0.0))
        {
            FAIL() << "Detected points outside of map's approximate bounds";
        }
        
    }


}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
}