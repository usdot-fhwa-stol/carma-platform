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

TEST(Testcarma_cloud_client, test_xml_list){

    rclcpp::NodeOptions options;
    carma_cloud_client::CarmaCloudClient plugin(options);

    std::string xml_list = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><TrafficControlMessageList><TrafficControlMessage><tcmV01><reqid>0102030405060708</reqid><reqseq>0</reqseq><msgtot>6</msgtot><msgnum>1</msgnum><id>001698403caedb603139c0f158992a7d</id><updated>0</updated><package><label>platform test</label><tcids><Id128b>001698403caedb603139c0f158992a7d</Id128b></tcids></package><params><vclasses><micromobile/><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27813460</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><closed><notopen/></closed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27813460</reftime><reflon>-771498705</reflon><reflat>389551653</reflat><refelv>0</refelv><refwidth>382</refwidth><heading>3312</heading><nodes><PathNode><x>1</x><y>0</y><width>0</width></PathNode><PathNode><x>-1260</x><y>802</y><width>3</width></PathNode><PathNode><x>-1176</x><y>923</y><width>2</width></PathNode><PathNode><x>-248</x><y>226</y><width>-2</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage><TrafficControlMessage><tcmV01><reqid>0102030405060708</reqid><reqseq>0</reqseq><msgtot>6</msgtot><msgnum>2</msgnum><id>0052b25d169a4a00c71c038fa70abbd7</id><updated>0</updated><package><label>workzone</label><tcids><Id128b>0052b25d169a4a00c71c038fa70abbd7</Id128b></tcids></package><params><vclasses><micromobile/><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27830621</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><maxspeed>45</maxspeed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27830621</reftime><reflon>-771509819</reflon><reflat>389557957</reflat><refelv>0</refelv><refwidth>413</refwidth><heading>3312</heading><nodes><PathNode><x>1</x><y>0</y><width>0</width></PathNode><PathNode><x>322</x><y>18</y><width>-16</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage><TrafficControlMessage><tcmV01><reqid>0102030405060708</reqid><reqseq>0</reqseq><msgtot>6</msgtot><msgnum>3</msgnum><id>00242a9dc147efc795dbb8a5dda83e33</id><updated>0</updated><package><label>workzone</label><tcids><Id128b>00242a9dc147efc795dbb8a5dda83e33</Id128b></tcids></package><params><vclasses><micromobile/><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27830622</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><maxspeed>45</maxspeed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27830622</reftime><reflon>-771509776</reflon><reflat>389557959</reflat><refelv>0</refelv><refwidth>411</refwidth><heading>3312</heading><nodes><PathNode><x>1</x><y>0</y><width>-1</width></PathNode><PathNode><x>1488</x><y>-38</y><width>-31</width></PathNode><PathNode><x>1426</x><y>-421</y><width>16</width></PathNode><PathNode><x>1281</x><y>-765</y><width>18</width></PathNode><PathNode><x>1104</x><y>-1003</y><width>-37</width></PathNode><PathNode><x>749</x><y>-1153</y><width>-3</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage><TrafficControlMessage><tcmV01><reqid>0102030405060708</reqid><reqseq>0</reqseq><msgtot>6</msgtot><msgnum>4</msgnum><id>0033d7ce1c56cbe32b0f94d1d4d0d23e</id><updated>0</updated><package><label>workzone</label><tcids><Id128b>0033d7ce1c56cbe32b0f94d1d4d0d23e</Id128b></tcids></package><params><vclasses><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27830632</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><maxspeed>45</maxspeed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27830632</reftime><reflon>-771503828</reflon><reflat>389554968</reflat><refelv>0</refelv><refwidth>371</refwidth><heading>3312</heading><nodes><PathNode><x>1</x><y>0</y><width>0</width></PathNode><PathNode><x>917</x><y>-1182</y><width>2</width></PathNode><PathNode><x>1027</x><y>-1090</y><width>2</width></PathNode><PathNode><x>1074</x><y>-1040</y><width>3</width></PathNode><PathNode><x>1167</x><y>-930</y><width>2</width></PathNode><PathNode><x>1264</x><y>-800</y><width>2</width></PathNode><PathNode><x>1334</x><y>-673</y><width>3</width></PathNode><PathNode><x>1384</x><y>-571</y><width>4</width></PathNode><PathNode><x>1322</x><y>-524</y><width>1</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage><TrafficControlMessage><tcmV01><reqid>0102030405060708</reqid><reqseq>0</reqseq><msgtot>6</msgtot><msgnum>5</msgnum><id>00b270eac7e965b98fbdc283006e41dd</id><updated>0</updated><package><label>workzone</label><tcids><Id128b>00b270eac7e965b98fbdc283006e41dd</Id128b></tcids></package><params><vclasses><micromobile/><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27818963</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><maxspeed>90</maxspeed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27818963</reftime><reflon>-771490953</reflon><reflat>389549263</reflat><refelv>0</refelv><refwidth>396</refwidth><heading>3312</heading><nodes><PathNode><x>1</x><y>0</y><width>0</width></PathNode><PathNode><x>1476</x><y>-248</y><width>15</width></PathNode><PathNode><x>1484</x><y>-190</y><width>22</width></PathNode><PathNode><x>1489</x><y>-132</y><width>18</width></PathNode><PathNode><x>1493</x><y>-67</y><width>5</width></PathNode><PathNode><x>1494</x><y>-20</y><width>-8</width></PathNode><PathNode><x>1492</x><y>83</y><width>-12</width></PathNode><PathNode><x>1490</x><y>148</y><width>-6</width></PathNode><PathNode><x>1484</x><y>206</y><width>-2</width></PathNode><PathNode><x>1475</x><y>248</y><width>-9</width></PathNode><PathNode><x>1040</x><y>207</y><width>-3</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage><TrafficControlMessage><tcmV01><reqid>0102030405060708</reqid><reqseq>0</reqseq><msgtot>6</msgtot><msgnum>6</msgnum><id>007dfe6e1f6d35f8f72d6ff4832d043a</id><updated>0</updated><package><label>workzone</label><tcids><Id128b>007dfe6e1f6d35f8f72d6ff4832d043a</Id128b></tcids></package><params><vclasses><micromobile/><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27817693</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><maxspeed>9</maxspeed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27817693</reftime><reflon>-771484526</reflon><reflat>389548802</reflat><refelv>0</refelv><refwidth>450</refwidth><heading>3312</heading><nodes><PathNode><x>1</x><y>0</y><width>0</width></PathNode><PathNode><x>1494</x><y>60</y><width>-10</width></PathNode><PathNode><x>1490</x><y>134</y><width>-6</width></PathNode><PathNode><x>1484</x><y>208</y><width>-6</width></PathNode><PathNode><x>1478</x><y>236</y><width>-5</width></PathNode><PathNode><x>1469</x><y>287</y><width>-9</width></PathNode><PathNode><x>1468</x><y>298</y><width>-4</width></PathNode><PathNode><x>939</x><y>191</y><width>-2</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage></TrafficControlMessageList>";
    
    boost::property_tree::ptree list_tree;
    std :: stringstream ss; 
    ss << xml_list;
    read_xml(ss, list_tree);

    auto child_tcm_list = list_tree.get_child_optional("TrafficControlMessageList");

    std::vector<j2735_v2x_msgs::msg::TrafficControlMessage> parsed_tcms;

    if (!child_tcm_list)
    {
      std::cerr<<"no list"<<std::endl;
    }
    else
    {
        
      auto tcm_list = child_tcm_list.get();

      BOOST_FOREACH(auto &node, list_tree.get_child("TrafficControlMessageList"))
      {
    
        j2735_v2x_msgs::msg::TrafficControlMessage parsed_tcm = plugin.parseTCMXML(node.second);
        ASSERT_EQ(parsed_tcm.choice, j2735_v2x_msgs::msg::TrafficControlMessage::TCMV01);
        parsed_tcms.push_back(parsed_tcm);

      }

    }

    ASSERT_EQ(parsed_tcms.size(), 6);

}

TEST(Testcarma_cloud_client, test_xml_parse_geometry){

    rclcpp::NodeOptions options;
    carma_cloud_client::CarmaCloudClient plugin(options);

    std::string xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><TrafficControlMessage><tcmV01><reqid>C7C9A13FE6AC464E</reqid><reqseq>0</reqseq><msgtot>11</msgtot><msgnum>8</msgnum><id>00308202879d343fea29d21a5181f099</id><updated>0</updated><package><label>Close Lane Small Vehicles</label><tcids><Id128b>00308202879d343fea29d21a5181f099</Id128b></tcids></package><params><vclasses><micromobile/><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27707632</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><closed><notopen/></closed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27707632</reftime><reflon>-818330861</reflon><reflat>281182920</reflat><refelv>0</refelv><refwidth>397</refwidth><heading>3403</heading><nodes><PathNode><x>2</x><y>0</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>-1</width></PathNode><PathNode><x>-406</x><y>1443</y><width>1</width></PathNode><PathNode><x>-407</x><y>1444</y><width>2</width></PathNode><PathNode><x>-406</x><y>1443</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>0</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage>";
    
    boost::property_tree::ptree tree;
    std :: stringstream ss; 
    ss << xml;
    read_xml(ss, tree);

    j2735_v2x_msgs::msg::TrafficControlGeometry tcm_geometry = plugin.parse_geometry(tree.get_child("TrafficControlMessage.tcmV01.geometry"));
    ASSERT_EQ(tcm_geometry.proj, "epsg:3785");
    ASSERT_EQ(tcm_geometry.datum, "WGS84");
    ASSERT_EQ(tcm_geometry.reflat, 281182920);
}

TEST(Testcarma_cloud_client, test_xml_parse_params){

    rclcpp::NodeOptions options;
    carma_cloud_client::CarmaCloudClient plugin(options);

    std::string xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><TrafficControlMessage><tcmV01><reqid>C7C9A13FE6AC464E</reqid><reqseq>0</reqseq><msgtot>11</msgtot><msgnum>8</msgnum><id>00308202879d343fea29d21a5181f099</id><updated>0</updated><package><label>Close Lane Small Vehicles</label><tcids><Id128b>00308202879d343fea29d21a5181f099</Id128b></tcids></package><params><vclasses><micromobile/><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27707632</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><closed><notopen/></closed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27707632</reftime><reflon>-818330861</reflon><reflat>281182920</reflat><refelv>0</refelv><refwidth>397</refwidth><heading>3403</heading><nodes><PathNode><x>2</x><y>0</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>-1</width></PathNode><PathNode><x>-406</x><y>1443</y><width>1</width></PathNode><PathNode><x>-407</x><y>1444</y><width>2</width></PathNode><PathNode><x>-406</x><y>1443</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>0</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage>";
    
    boost::property_tree::ptree tree;
    std :: stringstream ss; 
    ss << xml;
    read_xml(ss, tree);

    j2735_v2x_msgs::msg::TrafficControlParams tcm_params = plugin.parse_params(tree.get_child("TrafficControlMessage.tcmV01.params"));
    ASSERT_EQ(tcm_params.vclasses.size(), 14);
    ASSERT_TRUE(tcm_params.regulatory);

    j2735_v2x_msgs::msg::TrafficControlSchedule tcm_schedule = plugin.parse_schedule(tree.get_child("TrafficControlMessage.tcmV01.params"));
    ASSERT_EQ(tcm_schedule.start, 27707632);
    ASSERT_EQ(tcm_schedule.dow.dow.size(), 7);
    ASSERT_FALSE(tcm_schedule.between_exists);
    ASSERT_FALSE(tcm_schedule.repeat_exists);
}

TEST(Testcarma_cloud_client, test_xml_parse_package){

    rclcpp::NodeOptions options;
    carma_cloud_client::CarmaCloudClient plugin(options);

    std::string xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><TrafficControlMessage><tcmV01><reqid>C7C9A13FE6AC464E</reqid><reqseq>0</reqseq><msgtot>11</msgtot><msgnum>8</msgnum><id>00308202879d343fea29d21a5181f099</id><updated>0</updated><package><label>Close Lane Small Vehicles</label><tcids><Id128b>00308202879d343fea29d21a5181f099</Id128b></tcids></package><params><vclasses><micromobile/><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27707632</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><closed><notopen/></closed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27707632</reftime><reflon>-818330861</reflon><reflat>281182920</reflat><refelv>0</refelv><refwidth>397</refwidth><heading>3403</heading><nodes><PathNode><x>2</x><y>0</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>-1</width></PathNode><PathNode><x>-406</x><y>1443</y><width>1</width></PathNode><PathNode><x>-407</x><y>1444</y><width>2</width></PathNode><PathNode><x>-406</x><y>1443</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>0</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage>";
    
    boost::property_tree::ptree tree;
    std :: stringstream ss; 
    ss << xml;
    read_xml(ss, tree);

    j2735_v2x_msgs::msg::TrafficControlPackage tcm_package = plugin.parse_package(tree.get_child("TrafficControlMessage.tcmV01.package"));
    ASSERT_EQ(tcm_package.label, "Close Lane Small Vehicles");
}

TEST(Testcarma_cloud_client, test_xml_parse_detail){

    rclcpp::NodeOptions options;
    carma_cloud_client::CarmaCloudClient plugin(options);

    std::string xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><TrafficControlMessage><tcmV01><reqid>C7C9A13FE6AC464E</reqid><reqseq>0</reqseq><msgtot>11</msgtot><msgnum>8</msgnum><id>00308202879d343fea29d21a5181f099</id><updated>0</updated><package><label>Close Lane Small Vehicles</label><tcids><Id128b>00308202879d343fea29d21a5181f099</Id128b></tcids></package><params><vclasses><micromobile/><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27707632</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><closed><notopen/></closed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27707632</reftime><reflon>-818330861</reflon><reflat>281182920</reflat><refelv>0</refelv><refwidth>397</refwidth><heading>3403</heading><nodes><PathNode><x>2</x><y>0</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>-1</width></PathNode><PathNode><x>-406</x><y>1443</y><width>1</width></PathNode><PathNode><x>-407</x><y>1444</y><width>2</width></PathNode><PathNode><x>-406</x><y>1443</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>0</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage>";
    
    boost::property_tree::ptree tree;
    std :: stringstream ss; 
    ss << xml;
    read_xml(ss, tree);

    j2735_v2x_msgs::msg::TrafficControlDetail tcm_detail = plugin.parse_detail(tree.get_child("TrafficControlMessage.tcmV01.params"));
    ASSERT_EQ(tcm_detail.closed, j2735_v2x_msgs::msg::TrafficControlDetail::CLOSED);
}

TEST(Testcarma_cloud_client, test_xml_parse_full){

    rclcpp::NodeOptions options;
    carma_cloud_client::CarmaCloudClient plugin(options);

    std::string xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><TrafficControlMessage><tcmV01><reqid>C7C9A13FE6AC464E</reqid><reqseq>0</reqseq><msgtot>11</msgtot><msgnum>8</msgnum><id>00308202879d343fea29d21a5181f099</id><updated>0</updated><package><label>Close Lane Small Vehicles</label><tcids><Id128b>00308202879d343fea29d21a5181f099</Id128b></tcids></package><params><vclasses><micromobile/><motorcycle/><passenger-car/><light-truck-van/><bus/><two-axle-six-tire-single-unit-truck/><three-axle-single-unit-truck/><four-or-more-axle-single-unit-truck/><four-or-fewer-axle-single-trailer-truck/><five-axle-single-trailer-truck/><six-or-more-axle-single-trailer-truck/><five-or-fewer-axle-multi-trailer-truck/><six-axle-multi-trailer-truck/><seven-or-more-axle-multi-trailer-truck/></vclasses><schedule><start>27707632</start><end>153722867280912</end><dow>1111111</dow></schedule><regulatory><true/></regulatory><detail><closed><notopen/></closed></detail></params><geometry><proj>epsg:3785</proj><datum>WGS84</datum><reftime>27707632</reftime><reflon>-818330861</reflon><reflat>281182920</reflat><refelv>0</refelv><refwidth>397</refwidth><heading>3403</heading><nodes><PathNode><x>2</x><y>0</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>-1</width></PathNode><PathNode><x>-406</x><y>1443</y><width>1</width></PathNode><PathNode><x>-407</x><y>1444</y><width>2</width></PathNode><PathNode><x>-406</x><y>1443</y><width>0</width></PathNode><PathNode><x>-406</x><y>1444</y><width>0</width></PathNode></nodes></geometry></tcmV01></TrafficControlMessage>";
    
    boost::property_tree::ptree tree;
    std :: stringstream ss; 
    ss << xml;
    read_xml(ss, tree);

    j2735_v2x_msgs::msg::TrafficControlMessage tcm_msg = plugin.parseTCMXML(tree.get_child("TrafficControlMessage"));
    
    ASSERT_TRUE(tcm_msg.tcm_v01.package_exists);
    ASSERT_TRUE(tcm_msg.tcm_v01.package.label_exists);
    // Close Lane Small Vehicles
    
    ASSERT_TRUE(tcm_msg.tcm_v01.params_exists);
    ASSERT_EQ(tcm_msg.tcm_v01.params.vclasses.size(), 14);
    // ASSERT_TRUE(tcm_msg.tcm_v01.params.regulatory);
    
    ASSERT_TRUE(tcm_msg.tcm_v01.geometry_exists);
    ASSERT_EQ(tcm_msg.tcm_v01.geometry.nodes.size(), 6);

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