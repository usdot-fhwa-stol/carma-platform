/*
 * Copyright (C) 2021 LEIDOS.
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
#include <carma_wm/TrafficControl.h>
#include <carma_wm/SignalizedIntersectionManager.h>
#include <autoware_lanelet2_ros_interface/utility/message_conversion.h>
#include <memory>
#include <chrono>
#include <ctime>
#include <atomic>
#include "TestHelpers.h"
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>

#include <cav_msgs/TrafficControlMessage.h>
#include <carma_wm/WMTestLibForGuidance.h>

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_wm

{

TEST(SignalizedIntersectionManger, DISABLED_convertLaneToLaneletId9945)
{
  
  // File to process. Path is relative to test folder
  std::string file = "resource/TFHRC_03.10.22.xodr.osm";
  // Id of lanelet to start combing from
  lanelet::Id starting_id = 113;
  // Side to combine. If LEFT than the left lanelet left edge will be used for the left edge of the right lanelet
  // (intially the starting_id lanelet). Vice-versa for RIGHT.

  ///////////
  // START OF LOGIC
  ///////////

  // Write new map to file
  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;
  // Parse geo reference info from the original lanelet map (.osm)
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);

  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());

  lanelet::LaneletMapPtr lanelet_map = lanelet::load(file, local_projector, &load_errors);

  if (lanelet_map->laneletLayer.size() == 0)
  {
    FAIL() << "Input map does not contain any lanelets";
  }

  carma_wm::CARMAWorldModel cwm;
  cwm.setMap(lanelet_map);
  auto routing_graph = cwm.getMapRoutingGraph();

  carma_wm::SignalizedIntersectionManager sim;
  ROS_ERROR_STREAM("2");
  //std::string georeference = "+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sim.setTargetFrame(target_frame);
    
  std::unordered_map<uint8_t, lanelet::Id> entry;
  std::unordered_map<uint8_t, lanelet::Id> exit;

  cav_msgs::IntersectionGeometry intersection; //ref_point lat, lon, el = 0;
  intersection.id.id = 9945;

  cav_msgs::GenericLane lane;
  lane.lane_id = 1;
  j2735_msgs::Connection connection;
  connection.signal_group = 1;
  connection.connecting_lane.lane = 181;

  lane.connect_to_list.push_back(connection);

  cav_msgs::NodeXY node;
  
  // lane 1
  lane.lane_id = 1;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 2u; //ingress
  node.delta.x = -17.4;
  node.delta.y = -5.33;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -32.38; //offset from previous
  node.delta.y = -6.72;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -24.94; //offset from previous
  node.delta.y = -3.76;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -9.13; //offset from previous
  node.delta.y = -0.64;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -9.01; //offset from previous
  node.delta.y = -0.12;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -16.04; //offset from previous
  node.delta.y = 0.12;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -14.88; //offset from previous
  node.delta.y = 0.93;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -9.18; //offset from previous
  node.delta.y = 0.87;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -20.69; //offset from previous
  node.delta.y = 3.47;

  lane.node_list.nodes.node_set_xy.push_back(node);
  intersection.lane_list.push_back(lane);

  // lane 8 14329
  lane.lane_id = 8;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 1u; //ingress
  node.delta.x = -18.10;
  node.delta.y = -1.98;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -32.38; //offset from previous
  node.delta.y = -6.72;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -25.75; //offset from previous
  node.delta.y = -5.62;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -20.11; //offset from previous
  node.delta.y = -3.47;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 17.61; //offset from previous
  node.delta.y = -1.79;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -6.39; //offset from previous
  node.delta.y = 0.52;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -13.83; //offset from previous
  node.delta.y = 0.06;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -12.50; //offset from previous
  node.delta.y = 1.39;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -20.92; //offset from previous
  node.delta.y = 3.47;

  lane.node_list.nodes.node_set_xy.push_back(node);
  intersection.lane_list.push_back(lane);

  // lane 5 EGRESS, 4663 but MISSES! 
  lane.lane_id = 5;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 2u; //ingress

  node.delta.x = 16.08;
  node.delta.y = 4.97;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 6.92; //offset from previous
  node.delta.y = 2.03;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 5.35; //offset from previous
  node.delta.y = 1.62;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 4.53; //offset from previous
  node.delta.y = 2.08;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 3.78; //offset from previous
  node.delta.y = 2.37;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 3.25; //offset from previous
  node.delta.y = 1.91;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 2.67; //offset from previous
  node.delta.y = 2.95;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 1.45; //offset from previous
  node.delta.y = 2.60;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 1.57; //offset from previous
  node.delta.y = 3.01;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 0.35; //offset from previous
  node.delta.y = 0.87;

  lane.node_list.nodes.node_set_xy.push_back(node);
  intersection.lane_list.push_back(lane);

  // lane 4 6703

  lane.lane_id = 4;
  lane.node_list = {};
  lane.connect_to_list = {};
  lane.lane_attributes.directional_use.lane_direction = 1u; //ingress

  node.delta.x = 17.06;
  node.delta.y = 1.84;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 7.32; //offset from previous
  node.delta.y = 2.37;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 6.34; //offset from previous
  node.delta.y = 2.49;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 5.52; //offset from previous
  node.delta.y = 2.03;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 4.59; //offset from previous
  node.delta.y = 2.43;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 3.84; //offset from previous
  node.delta.y = 3.07;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 2.44; //offset from previous
  node.delta.y = 3.70;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 1.34; //offset from previous
  node.delta.y = 2.32;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 1.57; //offset from previous
  node.delta.y = 3.30;

  lane.node_list.nodes.node_set_xy.push_back(node);
  intersection.lane_list.push_back(lane);
  
  // lane 2 LANELET: 9135, but misses!
  lane.lane_id = 2;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 1u; //ingress

  node.delta.x = 1.14;
  node.delta.y = -11.99;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 0.06; //offset from previous
  node.delta.y = -2.32;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -0.46; //offset from previous
  node.delta.y = -2.32;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -0.87; //offset from previous
  node.delta.y = -2.49;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -1.28; //offset from previous
  node.delta.y = -2.32;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -2.21; //offset from previous
  node.delta.y = -1.97;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -3.78; //offset from previous
  node.delta.y = -1.97;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -4.94; //offset from previous
  node.delta.y = -1.97;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -10.0; //offset from previous
  node.delta.y = -2.84;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -3.72; //offset from previous
  node.delta.y = -0.93;

  lane.node_list.nodes.node_set_xy.push_back(node);
  intersection.lane_list.push_back(lane);

  // lane 3  // LANELET 12121
  lane.lane_id = 3;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 2u; //ingress

  node.delta.x = 4.04;
  node.delta.y = -11.88;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -0.29; //offset from previous
  node.delta.y = -4.75;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -0.99; //offset from previous
  node.delta.y = -3.82;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -1.05; //offset from previous
  node.delta.y = -2.03;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -2.03; //offset from previous
  node.delta.y = -2.43;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -2.85; //offset from previous
  node.delta.y = -1.97;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -3.20; //offset from previous
  node.delta.y = -1.39;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -4.13; //offset from previous
  node.delta.y = -1.62;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -5.06; //offset from previous
  node.delta.y = -1.62;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -6.22; //offset from previous
  node.delta.y = -1.74;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -3.31; //offset from previous
  node.delta.y = -1.04;

  lane.node_list.nodes.node_set_xy.push_back(node);
  intersection.lane_list.push_back(lane);

  // lane 6
  lane.lane_id = 6;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 1u; //ingress

  node.delta.x = -0.37;
  node.delta.y = 12.78;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -1.45; //offset from previous
  node.delta.y = 7.76;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -2.96; //offset from previous
  node.delta.y = 7.87;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -2.79; //offset from previous
  node.delta.y =  4.17;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -3.25; //offset from previous
  node.delta.y = 3.18;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -4.30; //offset from previous
  node.delta.y = 2.72;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -5.00; //offset from previous
  node.delta.y = 2.43;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -7.21; //offset from previous
  node.delta.y = 3.65;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -4.71; //offset from previous
  node.delta.y = 2.03;

  lane.node_list.nodes.node_set_xy.push_back(node);
  intersection.lane_list.push_back(lane);
  
   // lane 7
  lane.lane_id = 7;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 2u; //ingress

  node.delta.x = -3.34;
  node.delta.y = 12.26;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -1.51; //offset from previous
  node.delta.y = 7.18;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -1.98; //offset from previous
  node.delta.y = 5.15;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -2.32; //offset from previous
  node.delta.y =  4.46;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -3.49; //offset from previous
  node.delta.y = 3.88;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -5.29; //offset from previous
  node.delta.y = 3.47;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -3.89; //offset from previous
  node.delta.y = 1.79;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -6.39; //offset from previous
  node.delta.y = 2.55;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -5.93; //offset from previous
  node.delta.y = 2.66;

  lane.node_list.nodes.node_set_xy.push_back(node);

  intersection.lane_list.push_back(lane);

  intersection.ref_point.latitude = 38.9550489;
  intersection.ref_point.longitude = -77.1473752;


  sim.convertLaneToLaneletId(entry, exit, intersection, lanelet_map, cwm.getMapRoutingGraph());
  //for (auto iter = sim.signal_group_to_entry_lanelet_ids_.begin(); iter != sim.signal_group_to_entry_lanelet_ids_.end(); iter ++)
  //{
  //  ROS_ERROR_STREAM("Entry lanelet: " << iter->second);
  //}


  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_.size(), 1);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_.size(), 1);

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_[1].size(), 1);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_[1].size(), 1);
  EXPECT_EQ(entry.size(), 1);
  EXPECT_EQ(exit.size(), 1);
}


TEST(SignalizedIntersectionManger, convertLaneToLaneletId9709)
{
  
  // File to process. Path is relative to test folder
  std::string file = "resource/TFHRC_03.10.22.xodr.osm";
  // Id of lanelet to start combing from
  lanelet::Id starting_id = 113;
  // Side to combine. If LEFT than the left lanelet left edge will be used for the left edge of the right lanelet
  // (intially the starting_id lanelet). Vice-versa for RIGHT.

  ///////////
  // START OF LOGIC
  ///////////

  // Write new map to file
  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;
  // Parse geo reference info from the original lanelet map (.osm)
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);

  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());

  lanelet::LaneletMapPtr lanelet_map = lanelet::load(file, local_projector, &load_errors);

  if (lanelet_map->laneletLayer.size() == 0)
  {
    FAIL() << "Input map does not contain any lanelets";
  }

  carma_wm::CARMAWorldModel cwm;
  cwm.setMap(lanelet_map);
  auto routing_graph = cwm.getMapRoutingGraph();

  carma_wm::SignalizedIntersectionManager sim;
  
  ROS_ERROR_STREAM("2");
  //std::string georeference = "+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sim.setTargetFrame(target_frame);
    
  std::unordered_map<uint8_t, lanelet::Id> entry;
  std::unordered_map<uint8_t, lanelet::Id> exit;

  cav_msgs::IntersectionGeometry intersection; //ref_point lat, lon, el = 0;
  intersection.id.id = 9709;
  std::pair<double, double> correction = {-0.5, -1.0};
  sim.intersection_coord_correction_[intersection.id.id] = correction;

  cav_msgs::GenericLane lane;
  lane.lane_id = 1;
  j2735_msgs::Connection connection;
  connection.signal_group = 1;
  connection.connecting_lane.lane = 181;

  lane.connect_to_list.push_back(connection);

  cav_msgs::NodeXY node;
  
  // lane 1
  lane.lane_id = 1;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 2u; //ingress
  node.delta.x = -5.23;
  node.delta.y = -12.94;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -3.60; //offset from previous
  node.delta.y = -7.24;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -6.224; //offset from previous
  node.delta.y = -11.11;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -2.09; //offset from previous
  node.delta.y = -6.54;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 0.76; //offset from previous
  node.delta.y = -5.79;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 3.66; //offset from previous
  node.delta.y = -5.09;

  lane.node_list.nodes.node_set_xy.push_back(node);

  intersection.lane_list.push_back(lane);

  // lane 8 14329
  lane.lane_id = 5;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 1u; //ingress
  node.delta.x = -2.5;
  node.delta.y = -5.27;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -3.78; //offset from previous
  node.delta.y = -7.81;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -2.44; //offset from previous
  node.delta.y = -6.19;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -1.74; //offset from previous
  node.delta.y = -4.28;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 0.29; //offset from previous
  node.delta.y = -7.29;

  lane.node_list.nodes.node_set_xy.push_back(node);

  intersection.lane_list.push_back(lane);

  // lane 5 EGRESS, 4663 but MISSES! 
  lane.lane_id = 6;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 1u; //ingress

  node.delta.x = 15.23;
  node.delta.y = -5.18;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 5.81; //offset from previous
  node.delta.y = -1.33;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 10.35; //offset from previous
  node.delta.y = -1.62;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 9.53; //offset from previous
  node.delta.y = -1.22;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 7.09; //offset from previous
  node.delta.y = -0.41;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 13.60; //offset from previous
  node.delta.y = -0.98;

  lane.node_list.nodes.node_set_xy.push_back(node);

  intersection.lane_list.push_back(lane);

  // lane 4 6703

  lane.lane_id = 2;
  lane.node_list = {};
  lane.connect_to_list = {};
  lane.lane_attributes.directional_use.lane_direction = 2u; //ingress

  node.delta.x = 16.04;
  node.delta.y = -1.82;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 5.93; //offset from previous
  node.delta.y = -1.39;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 9.24; //offset from previous
  node.delta.y = -1.39;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 8.37; //offset from previous
  node.delta.y = -1.10;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 8.72; //offset from previous
  node.delta.y = -0.93;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 13.48; //offset from previous
  node.delta.y = -0.64;

  lane.node_list.nodes.node_set_xy.push_back(node);

  intersection.lane_list.push_back(lane);
  
  // lane 7 LANELET: 9135, but misses!
  lane.lane_id = 7;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 1u; //ingress

  node.delta.x = 8.72;
  node.delta.y = 13.63;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 2.32; //offset from previous
  node.delta.y = 4.98;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 3.08; //offset from previous
  node.delta.y = 8.05;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 3.31; //offset from previous
  node.delta.y = 7.41;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 2.5; //offset from previous
  node.delta.y = 7.12;

  lane.node_list.nodes.node_set_xy.push_back(node);

  intersection.lane_list.push_back(lane);

  // lane 3  // LANELET 12121
  lane.lane_id = 3;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 2u; //ingress

  node.delta.x = 5.06;
  node.delta.y = 14.67;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 3.49; //offset from previous
  node.delta.y = 7.35;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 3.72; //offset from previous
  node.delta.y = 8.97;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 2.85; //offset from previous
  node.delta.y = 6.83;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 2.15; //offset from previous
  node.delta.y = 4.80;

  lane.node_list.nodes.node_set_xy.push_back(node);

  
  intersection.lane_list.push_back(lane);

  // lane 8
  lane.lane_id = 8;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 1u; //ingress

  node.delta.x = -15.40;
  node.delta.y = 7.03;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -7.15; //offset from previous
  node.delta.y = 2.14;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -8.08; //offset from previous
  node.delta.y = 3.59;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -5.11; //offset from previous
  node.delta.y =  2.55;

  lane.node_list.nodes.node_set_xy.push_back(node);

  intersection.lane_list.push_back(lane);
  
   // lane 4
  lane.lane_id = 4;
  lane.node_list = {};
  lane.connect_to_list = {};

  lane.lane_attributes.directional_use.lane_direction = 2u; //ingress

  node.delta.x = -16.54;
  node.delta.y = 3.68;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -4.65; //offset from previous
  node.delta.y = 3.68;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -4.65; //offset from previous
  node.delta.y = 1.39;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -6.51; //offset from previous
  node.delta.y =  2.08;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -6.39; //offset from previous
  node.delta.y = 2.32;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -7.50; //offset from previous
  node.delta.y = 3.13;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = -7.61; //offset from previous
  node.delta.y = 4.92;

  lane.node_list.nodes.node_set_xy.push_back(node);

  intersection.lane_list.push_back(lane);

  intersection.ref_point.latitude = 38.9549844;
  intersection.ref_point.longitude = -77.1493239;


  sim.convertLaneToLaneletId(entry, exit, intersection, lanelet_map, cwm.getMapRoutingGraph());
  //for (auto iter = sim.signal_group_to_entry_lanelet_ids_.begin(); iter != sim.signal_group_to_entry_lanelet_ids_.end(); iter ++)
  //{
  //  ROS_ERROR_STREAM("Entry lanelet: " << iter->second);
  //}


  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_.size(), 1);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_.size(), 1);

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_[1].size(), 1);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_[1].size(), 1);
  EXPECT_EQ(entry.size(), 1);
  EXPECT_EQ(exit.size(), 1);
}


TEST(SignalizedIntersectionManger, DISABLED_createIntersectionFromMapMsg)
{
  /* | 173| 183| 193|
  *  | _  _  _  _  _|
  *  | 172| Ob | 192|
  *  | _  _  _  _  _|
  *  | 171| 181| 191|    num   = lanelet id hardcoded for easier testing
  *  | _  _  _  _  _|    |     = lane lines
  *  | 170| 180| 190|    - - - = Lanelet boundary
  *  |              |    O     = Default Obstacle
  *  ****************
  *     START_LINE
  */
  carma_wm::test::MapOptions options;
  options.lane_length_ = 25;
  options.lane_width_ = 5;
  auto cwm = carma_wm::test::getGuidanceTestMap(options);
  auto lanelet_map = cwm->getMutableMap();

  carma_wm::SignalizedIntersectionManager sim;
  std::string georeference = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sim.setTargetFrame(georeference);
    
  cav_msgs::MapData map_msg;
  cav_msgs::IntersectionGeometry intersection; //ref_point lat, lon, el = 0;
  intersection.id.id = 9001;

  cav_msgs::GenericLane lane;
  lane.lane_id = 180;

  lane.lane_attributes.directional_use.lane_direction = 1u; //ingress
  j2735_msgs::Connection connection;
  connection.signal_group = 1;
  connection.connecting_lane.lane = 190;

  lane.connect_to_list.push_back(connection);

  cav_msgs::NodeXY node;
  node.delta.x = 7.5;
  node.delta.y = 12.5;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 0; //offset from previous
  node.delta.y = -2.5;

  lane.node_list.nodes.node_set_xy.push_back(node);

  intersection.lane_list.push_back(lane);

  lane.lane_id = 190;
  
  lane.lane_attributes.directional_use.lane_direction = 2u; // egress imagining intersection 
                                                            // entering 1210 from left and out through 1220
  lane.node_list = {};
  lane.connect_to_list = {};

  node.delta.x = 12.5;
  node.delta.y = 12.5;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 0; //offset from previous
  node.delta.y = 2.5;

  lane.node_list.nodes.node_set_xy.push_back(node);
  intersection.lane_list.push_back(lane);
  map_msg.intersections.push_back(intersection);

  std::vector<std::shared_ptr<lanelet::SignalizedIntersection>> intersections;
  std::vector<std::shared_ptr<lanelet::CarmaTrafficSignal>> traffic_signals;

  sim.createIntersectionFromMapMsg(intersections, traffic_signals, map_msg, lanelet_map, cwm->getMapRoutingGraph());

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_.size(), 1);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_.size(), 1);

  EXPECT_EQ(*sim.signal_group_to_entry_lanelet_ids_[1].begin(), 1210);
  EXPECT_EQ(*sim.signal_group_to_exit_lanelet_ids_[1].begin(), 1220);
  EXPECT_EQ(sim.signal_group_to_traffic_light_id_[1], traffic_signals.front()->id());
  EXPECT_EQ(sim.intersection_id_to_regem_id_.size(), 1);
  EXPECT_EQ(intersections.size(), 1);
  EXPECT_EQ(traffic_signals.size(), 1);
  
}


TEST(SignalizedIntersectionManger, DISABLED_matchSignalizedIntersection)
{
  /* | 173| 183| 193|
  *  | _  _  _  _  _|
  *  | 172| Ob | 192|
  *  | _  _  _  _  _|
  *  | 171| 181| 191|    num   = lanelet id hardcoded for easier testing
  *  | _  _  _  _  _|    |     = lane lines
  *  | 170| 180| 190|    - - - = Lanelet boundary
  *  |              |    O     = Default Obstacle
  *  ****************
  *     START_LINE
  */
  
  auto lanelet_map = carma_wm::test::buildGuidanceTestMap(5.0, 25.0);

  carma_wm::SignalizedIntersectionManager sim;
  std::string georeference = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sim.setTargetFrame(georeference);
  
  lanelet::Id intersection_id = lanelet::utils::getId();
  auto intersection = std::make_shared<lanelet::SignalizedIntersection>(lanelet::SignalizedIntersection::buildData(intersection_id, 
                                                                        {lanelet_map->laneletLayer.get(180)}, {lanelet_map->laneletLayer.get(190)}, {}));
  
  lanelet_map->update({lanelet_map->laneletLayer.get(180)}, intersection);

  lanelet::Id queried_id = sim.matchSignalizedIntersection({lanelet_map->laneletLayer.get(180)}, {lanelet_map->laneletLayer.get(190)});

  EXPECT_EQ(queried_id, intersection_id);

}
TEST(SignalizedIntersectionManger, DISABLED_createTrafficSignalUsingSGID)
{
  /* | 173| 183| 193|
  *  | _  _  _  _  _|
  *  | 172| Ob | 192|
  *  | _  _  _  _  _|
  *  | 171| 181| 191|    num   = lanelet id hardcoded for easier testing
  *  | _  _  _  _  _|    |     = lane lines
  *  | 170| 180| 190|    - - - = Lanelet boundary
  *  |              |    O     = Default Obstacle
  *  ****************
  *     START_LINE
  */
  
  auto lanelet_map = carma_wm::test::buildGuidanceTestMap(5.0, 25.0);

  carma_wm::SignalizedIntersectionManager sim;
  std::string georeference = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sim.setTargetFrame(georeference);
  
  auto signal = sim.createTrafficSignalUsingSGID(1, {lanelet_map->laneletLayer.get(1210)}, {lanelet_map->laneletLayer.get(190)});

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_.size(), 1);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_.size(), 1);
  EXPECT_EQ(sim.signal_group_to_traffic_light_id_[1], signal->id());
}
TEST(SignalizedIntersectionManger, DISABLED_identifyInteriorLanelets)
{
  /* | 173| 183| 193|
  *  | _  _  _  _  _|
  *  | 172| Ob | 192|
  *  | _  _  _  _  _|
  *  | 171| 181| 191|    num   = lanelet id hardcoded for easier testing
  *  | _  _  _  _  _|    |     = lane lines
  *  | 170| 180| 190|    - - - = Lanelet boundary
  *  |              |    O     = Default Obstacle
  *  ****************
  *     START_LINE
  */
  
  auto lanelet_map = carma_wm::test::buildGuidanceTestMap(5.0, 25.0);

  carma_wm::SignalizedIntersectionManager sim;
  std::string georeference = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sim.setTargetFrame(georeference);
  
  auto interior = sim.identifyInteriorLanelets({lanelet_map->laneletLayer.get(173),lanelet_map->laneletLayer.get(181), lanelet_map->laneletLayer.get(193)}, lanelet_map);

  EXPECT_EQ(interior.size(), 4);


}


}  // namespace carma_wm_ctrl