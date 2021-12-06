/*
 * Copyright (C) 2020-2021 LEIDOS.
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
  
  auto lanelet_map = carma_wm::test::buildGuidanceTestMap(5.0, 25.0);
  ROS_DEBUG_STREAM("Left x: " << lanelet_map->laneletLayer.get(1210).leftBound2d().front().x());
  ROS_DEBUG_STREAM("Left y: " << lanelet_map->laneletLayer.get(1210).leftBound2d().front().y());
  ROS_DEBUG_STREAM("Right x: " << lanelet_map->laneletLayer.get(1210).rightBound2d().front().x());
  ROS_DEBUG_STREAM("Right y: " << lanelet_map->laneletLayer.get(1210).rightBound2d().front().y());
  ROS_DEBUG_STREAM("=====");

  ROS_DEBUG_STREAM("Left x: " << lanelet_map->laneletLayer.get(1210).leftBound2d().back().x());
  ROS_DEBUG_STREAM("Left y: " << lanelet_map->laneletLayer.get(1210).leftBound2d().back().y());
  ROS_DEBUG_STREAM("Right x: " << lanelet_map->laneletLayer.get(1210).rightBound2d().back().x());
  ROS_DEBUG_STREAM("Right y: " << lanelet_map->laneletLayer.get(1210).rightBound2d().back().y());

  carma_wm::SignalizedIntersectionManager sim;
  std::string georeference = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sim.setTargetFrame(georeference);
    
  std::unordered_map<uint8_t, lanelet::Id> entry;
  std::unordered_map<uint8_t, lanelet::Id> exit;

  cav_msgs::IntersectionGeometry intersection; //ref_point lat, lon, el = 0;
  intersection.id.id = 9001;

  cav_msgs::GenericLane lane;
  lane.lane_id = 1210;
  lane.lane_attributes.directional_use.lane_direction = 1u; //ingress
  j2735_msgs::Connection connection;
  connection.signal_group = 1;
  connection.connecting_lane.lane = 1220;

  lane.connect_to_list.push_back(connection);

  cav_msgs::NodeXY node;
  node.delta.x = 5;
  node.delta.y = 12.5;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 2.5; //offset from previous
  node.delta.y = 0;

  lane.node_list.nodes.node_set_xy.push_back(node);

  intersection.lane_list.push_back(lane);

  lane.lane_id = 1220;
  lane.lane_attributes.directional_use.lane_direction = 2u; // egress imagining intersection 
                                                            // entering 1210 from left and out through 1220
  lane.node_list = {};
  lane.connect_to_list = {};

  node.delta.x = 10;
  node.delta.y = 12.5;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 2.5; //offset from previous
  node.delta.y = 0;

  lane.node_list.nodes.node_set_xy.push_back(node);
  intersection.lane_list.push_back(lane);

  sim.convertLaneToLaneletId(entry, exit, intersection, lanelet_map);

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_.size(), 1);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_.size(), 1);

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_[1].size(), 1);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_[1].size(), 1);
  EXPECT_EQ(entry.size(), 1);
  EXPECT_EQ(exit.size(), 1);
}

TEST(SignalizedIntersectionManger, createIntersectionFromMapMsg)
{
  

}


TEST(SignalizedIntersectionManger, matchSignalizedIntersection)
{
  

}
TEST(SignalizedIntersectionManger, createTrafficSignalUsingSGID)
{
  

}
TEST(SignalizedIntersectionManger, identifyInteriorLanelets)
{
  

}


}  // namespace carma_wm_ctrl