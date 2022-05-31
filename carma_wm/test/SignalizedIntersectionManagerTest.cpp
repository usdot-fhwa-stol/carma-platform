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

TEST(SignalizedIntersectionManger, convertLaneToLaneletId)
{
  
  /* |1203|1213|1223|
  *  | _  _  _  _  _|
  *  |1202| Ob |1222|
  *  | _  _  _  _  _|
  *  |1201|1211|1221|    num   = lanelet id hardcoded for easier testing
  *  | _  _  _  _  _|    |     = lane lines
  *  |1200|1210|1220|    - - - = Lanelet boundary
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
    
  std::unordered_map<uint8_t, lanelet::Id> entry;
  std::unordered_map<uint8_t, lanelet::Id> exit;

  cav_msgs::IntersectionGeometry intersection; //ref_point lat, lon, el = 0;
  intersection.id.id = 9001;

  cav_msgs::GenericLane lane;
  lane.lane_id = (uint8_t)1210;
  lane.lane_attributes.directional_use.lane_direction = 2u; //ingress
  j2735_msgs::Connection connection;
  connection.signal_group = 1;
  connection.connecting_lane.lane = 1211;

  lane.connect_to_list.push_back(connection);

  cav_msgs::NodeXY node;
  node.delta.x = 7.5;
  node.delta.y = 25.0;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 0.0; //offset from previous
  node.delta.y = -0.5;

  lane.node_list.nodes.node_set_xy.push_back(node);

  intersection.lane_list.push_back(lane);

  lane.lane_id = (uint8_t)1211;
  lane.lane_attributes.directional_use.lane_direction = 1u; // egress imagining intersection 
                                                            // entering 1210 from left and out through 1220
  lane.node_list = {};
  lane.connect_to_list = {};

  node.delta.x = 7.5;
  node.delta.y = 37.5;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 0.0; //offset from previous
  node.delta.y = 50.0;

  lane.node_list.nodes.node_set_xy.push_back(node);
  intersection.lane_list.push_back(lane);

  sim.convertLaneToLaneletId(entry, exit, intersection, lanelet_map, cwm->getMapRoutingGraph());

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_.size(), 1);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_.size(), 1);

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_[1].size(), 1);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_[1].size(), 1);
  EXPECT_EQ(entry.size(), 1);
  EXPECT_EQ(exit.size(), 1);
}

TEST(SignalizedIntersectionManger, createIntersectionFromMapMsg)
{
  /* |1203|1213|1223|
  *  | _  _  _  _  _|
  *  |1202| Ob |1222|
  *  | _  _  _  _  _|
  *  |1201|1211|1221|    num   = lanelet id hardcoded for easier testing
  *  | _  _  _  _  _|    |     = lane lines
  *  |1200|1210|1220|    - - - = Lanelet boundary
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
  lane.lane_id = (uint8_t)1210;
  lane.lane_attributes.directional_use.lane_direction = 2u; //ingress
  j2735_msgs::Connection connection;
  connection.signal_group = 1;
  connection.connecting_lane.lane = 1220;

  lane.connect_to_list.push_back(connection);

  cav_msgs::NodeXY node;
  node.delta.x = 7.5;
  node.delta.y = 12.5;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 0; //offset from previous
  node.delta.y = -2.5;

  lane.node_list.nodes.node_set_xy.push_back(node);

  intersection.lane_list.push_back(lane);

  lane.lane_id = (uint8_t)1220;
  lane.lane_attributes.directional_use.lane_direction = 1u; // egress imagining intersection 
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


TEST(SignalizedIntersectionManger, matchSignalizedIntersection)
{
  /* |1203|1213|1223|
  *  | _  _  _  _  _|
  *  |1202| Ob |1222|
  *  | _  _  _  _  _|
  *  |1201|1211|1221|    num   = lanelet id hardcoded for easier testing
  *  | _  _  _  _  _|    |     = lane lines
  *  |1200|1210|1220|    - - - = Lanelet boundary
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
                                                                        {lanelet_map->laneletLayer.get(1210)}, {lanelet_map->laneletLayer.get(1220)}, {}));
  
  lanelet_map->update({lanelet_map->laneletLayer.get(1210)}, intersection);

  lanelet::Id queried_id = sim.matchSignalizedIntersection({lanelet_map->laneletLayer.get(1210)}, {lanelet_map->laneletLayer.get(1220)});

  EXPECT_EQ(queried_id, intersection_id);

}
TEST(SignalizedIntersectionManger, createTrafficSignalUsingSGID)
{
  /* |1203|1213|1223|
  *  | _  _  _  _  _|
  *  |1202| Ob |1222|
  *  | _  _  _  _  _|
  *  |1201|1211|1221|    num   = lanelet id hardcoded for easier testing
  *  | _  _  _  _  _|    |     = lane lines
  *  |1200|1210|1220|    - - - = Lanelet boundary
  *  |              |    O     = Default Obstacle
  *  ****************
  *     START_LINE
  */
  
  auto lanelet_map = carma_wm::test::buildGuidanceTestMap(5.0, 25.0);

  carma_wm::SignalizedIntersectionManager sim;
  std::string georeference = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sim.setTargetFrame(georeference);
  
  auto signal = sim.createTrafficSignalUsingSGID(1, {lanelet_map->laneletLayer.get(1210)}, {lanelet_map->laneletLayer.get(1220)});

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_.size(), 1);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_.size(), 1);
  EXPECT_EQ(sim.signal_group_to_traffic_light_id_[1], signal->id());
}
TEST(SignalizedIntersectionManger, identifyInteriorLanelets)
{
  /* |1203|1213|1223|
  *  | _  _  _  _  _|
  *  |1202| Ob |1222|
  *  | _  _  _  _  _|
  *  |1201|1211|1221|    num   = lanelet id hardcoded for easier testing
  *  | _  _  _  _  _|    |     = lane lines
  *  |1200|1210|1220|    - - - = Lanelet boundary
  *  |              |    O     = Default Obstacle
  *  ****************
  *     START_LINE
  */
  
  auto lanelet_map = carma_wm::test::buildGuidanceTestMap(5.0, 25.0);

  carma_wm::SignalizedIntersectionManager sim;
  std::string georeference = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sim.setTargetFrame(georeference);
  
  auto interior = sim.identifyInteriorLanelets({lanelet_map->laneletLayer.get(1203),lanelet_map->laneletLayer.get(1211), lanelet_map->laneletLayer.get(1223)}, lanelet_map);

  EXPECT_EQ(interior.size(), 4);


}


}  // namespace carma_wm_ctrl