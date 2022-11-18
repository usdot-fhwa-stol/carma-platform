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

TEST(Testcarma_cloud_client, test_xml_parse_full){

    rclcpp::NodeOptions options;
    carma_cloud_client::CarmaCloudClient plugin(options);

    std::string xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><TrafficControlMessage><tcmV01><reqid>C7C9A13FE6AC464E</reqid><reqseq>0</reqseq><msgtot>11</msgtot><msgnum>8</msgnum><id>00308202879d343fea29d21a5181f099</id><updated>0</updated><package><label>Close Lane Small Vehicles</label><tcids><Id128b>00308202879d343fea29d21a5181f099</Id128b></tcids></package><params><vclasses><micromobile/><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27707632</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><closed><notopen/></closed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27707632</reftime><reflon>-818330861</reflon><reflat>281182920</reflat><refelv>0</refelv><refwidth>397</refwidth><heading>3403</heading><nodes><PathNode><x>2</x><y>0</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>-1</width></PathNode><PathNode><x>-406</x><y>1443</y><width>1</width></PathNode><PathNode><x>-407</x><y>1444</y><width>2</width></PathNode><PathNode><x>-406</x><y>1443</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>0</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage>";
    
    carma_v2x_msgs::msg::TrafficControlMessage tcm_msg = plugin.parseTCMXML(xml);
    
    // std::cout << "bool " << tcm_msg.tcm_v01.reqseq <<std::endl;
    ASSERT_TRUE(tcm_msg.tcm_v01.package_exists);
    ASSERT_TRUE(tcm_msg.tcm_v01.package.label_exists);
    std::cout<<"label: "<<tcm_msg.tcm_v01.package.label<<std::endl;
    // Close Lane Small Vehicles
    
    ASSERT_TRUE(tcm_msg.tcm_v01.params_exists);
    ASSERT_EQ(tcm_msg.tcm_v01.params.vclasses.size(), 14);
    // ASSERT_TRUE(tcm_msg.tcm_v01.params.regulatory);
    
    ASSERT_TRUE(tcm_msg.tcm_v01.geometry_exists);
    ASSERT_EQ(tcm_msg.tcm_v01.geometry.nodes.size(), 6);

}

unsigned char parse_hex(char c)
    {
        if ('0' <= c && c <= '9') return c - '0';
        if ('A' <= c && c <= 'F') return c - 'A' + 10;
        if ('a' <= c && c <= 'f') return c - 'a' + 10;
        std::abort();
    }

TEST(Testcarma_cloud_client, test_xml_parse2){

    rclcpp::NodeOptions options;
    carma_cloud_client::CarmaCloudClient plugin(options);


    std::string xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><TrafficControlMessage><tcmV01><reqid>C7C9A13FE6AC464E</reqid><reqseq>0</reqseq><msgtot>11</msgtot><msgnum>8</msgnum><id>00308202879d343fea29d21a5181f099</id><updated>0</updated><package><label>Close Lane Small Vehicles</label><tcids><Id128b>00308202879d343fea29d21a5181f099</Id128b></tcids></package><params><vclasses><micromobile/><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27707632</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><closed><notopen/></closed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27707632</reftime><reflon>-818330861</reflon><reflat>281182920</reflat><refelv>0</refelv><refwidth>397</refwidth><heading>3403</heading><nodes><PathNode><x>2</x><y>0</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>-1</width></PathNode><PathNode><x>-406</x><y>1443</y><width>1</width></PathNode><PathNode><x>-407</x><y>1444</y><width>2</width></PathNode><PathNode><x>-406</x><y>1443</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>0</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage>";
    boost::property_tree::ptree tree;
    
    try 
    {
      std :: stringstream ss; 
      ss << xml;
      read_xml(ss, tree);
    } 
    catch (boost::property_tree::xml_parser_error &e) 
    {
      std :: cout << "Failed to parse the xml string." << e.what();
    } 
    catch (...) {
      std :: cout << "Failed !!!";
    }

    int reqseq = tree.get<int>("TrafficControlMessage.tcmV01.reqseq");
    std::cout << "reqseq " << reqseq <<std::endl;

    std::string req = tree.get<std::string>("TrafficControlMessage.tcmV01.reqid");
    std::cout << "req " << req <<std::endl;

    std::string label = tree.get<std::string>("TrafficControlMessage.tcmV01.package.label");
    std::cout << "label " << label <<std::endl;


    int start = tree.get<int>("TrafficControlMessage.tcmV01.params.schedule.start");
    std::cout << "start " << start <<std::endl;

    boost::property_tree::ptree::const_assoc_iterator  it = tree.find("package");
    if( it == tree.not_found() )
    {
        std::cout << "packageee " <<std::endl;
    }
    else
    {
        std::cout << "no " <<std::endl;
    }

    if (tree.count("package")!=0)
    {
        std::cout << "packageee2 " <<std::endl;
    }
    else
    {
        std::cout << "no " <<std::endl;
    }
    
    std::string reqq = "C7";

    uint8_t nn = std::stoull(reqq, 0, 16);
    std::cout << "nn " << std::to_string(nn) <<std::endl;

    for (auto& item : tree.get_child("TrafficControlMessage.tcmV01.reqid"))
    {
        uint8_t hex = item.second.get_value<uint8_t>();
        std::cout << "hex " << std::to_string(hex) <<std::endl;
    }
    
    std::string vc = tree.get<std::string>("TrafficControlMessage.tcmV01.params.vclasses");
    std::cout << "vc " << vc.size() <<std::endl;

    std::string cl = tree.get<std::string>("TrafficControlMessage.tcmV01.params.detail.closed");
    std::cout << "cl " << cl <<std::endl;

    auto child = tree.get_child_optional( "TrafficControlMessage.tcmV01.params.detail.maxspeed" );
    if( !child )
    {
        std::cout << "no max speed"<<std::endl;
    }

    boost::optional<float> v = tree.get_optional<float>("TrafficControlMessage.tcmV01.params.detail.maxspeed");
    if( !v )
    {
        std::cout << "no max speed2"<<std::endl;
    }

    auto tree_package = tree.get_child_optional("TrafficControlMessage.tcmV01.package");
    if (tree_package)
    {
        auto tree1 = tree_package.get();
        std::cout << "found package"<<std::endl;
        std::string tcids_string = tree.get<std::string>("TrafficControlMessage.tcmV01.package.tcids");
        std::cout << "found package tcid" << tcids_string <<std::endl;
    }

    for (auto& item : tree.get_child("TrafficControlMessage.tcmV01.params.vclasses"))
    {
        std::cout << "vc " << item.first  <<std::endl;
    }

    float node_x = tree.get<float>("TrafficControlMessage.tcmV01.geometry.nodes.PathNode.x");
    std::cout << "node_x " << node_x <<std::endl;

    for (auto& item : tree.get_child("TrafficControlMessage.tcmV01.params.vclasses"))
    {
        std::cout << "item.first " << item.first << std::endl;
    }

    for (auto& item : tree.get_child("TrafficControlMessage.tcmV01.geometry.nodes"))
    {
        std::cout << "node " << item.first << std::endl;

        for (auto& which : item.second)
        {
            if (which.first == "x")
            {
                float xx = which.second.get_value<float>();
                std::cout << "xx " << xx << std::endl;
            }
        }
    }
    // BOOST_FOREACH(  boost::property_tree::ptree::value_type const& v, tree.get_child("TrafficControlMessage.tcmV01.params.vclasses") )
    // {
    //     std::cout << "vc " << v <<std::endl;
    // }
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