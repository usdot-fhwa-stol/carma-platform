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



// TODO for USER: Implement a real test using GTest
TEST(Testpoints_map_filter, example_test)
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


    auto sl = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::utils::getId(), 20_mph, {ll},
      {}, { lanelet::Participants::VehicleCar }));

      ll.addRegulatoryElement(sl);
      map->add(sl);//Add DigitalSpeedLimit data to the map

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