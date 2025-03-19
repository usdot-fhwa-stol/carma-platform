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
#include <carma_wm/TrafficControl.hpp>
#include <carma_wm/SignalizedIntersectionManager.hpp>
#include <autoware_lanelet2_ros2_interface/utility/message_conversion.hpp>
#include <memory>
#include <chrono>
#include <ctime>
#include <atomic>
#include "TestHelpers.hpp"

#include <carma_v2x_msgs/msg/traffic_control_message.hpp>
#include <carma_wm/WMTestLibForGuidance.hpp>


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

  carma_v2x_msgs::msg::IntersectionGeometry intersection; //ref_point lat, lon, el = 0;
  intersection.id.id = 9001;

  carma_v2x_msgs::msg::GenericLane lane;
  lane.lane_id = (uint8_t)1210;
  lane.lane_attributes.directional_use.lane_direction = 2u; //ingress
  j2735_v2x_msgs::msg::Connection connection;
  connection.signal_group = 1;
  connection.connecting_lane.lane = 1211;

  lane.connect_to_list.push_back(connection);

  carma_v2x_msgs::msg::NodeXY node;
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
  lane.node_list = carma_v2x_msgs::msg::NodeListXY();
  lane.connect_to_list = {};

  node.delta.x = 7.5;
  node.delta.y = 37.5;

  lane.node_list.nodes.node_set_xy.push_back(node);

  node.delta.x = 0.0; //offset from previous
  node.delta.y = 50.0;

  lane.node_list.nodes.node_set_xy.push_back(node);
  intersection.lane_list.push_back(lane);

  sim.convertLaneToLaneletId(entry, exit, intersection, lanelet_map, cwm->getMapRoutingGraph());

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_.size(), 1u);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_.size(), 1u);

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_[1].size(), 1u);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_[1].size(), 1u);
  EXPECT_EQ(entry.size(), 1u);
  EXPECT_EQ(exit.size(), 1u);
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

  carma_v2x_msgs::msg::MapData map_msg;
  carma_v2x_msgs::msg::IntersectionGeometry intersection; //ref_point lat, lon, el = 0;
  intersection.id.id = 9001;

  carma_v2x_msgs::msg::GenericLane lane;
  lane.lane_id = (uint8_t)1210;
  lane.lane_attributes.directional_use.lane_direction = 2u; //ingress
  j2735_v2x_msgs::msg::Connection connection;
  connection.signal_group = 1;
  connection.connecting_lane.lane = 1220;

  lane.connect_to_list.push_back(connection);

  carma_v2x_msgs::msg::NodeXY node;
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
  lane.node_list = carma_v2x_msgs::msg::NodeListXY();
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

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_.size(), 1u);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_.size(), 1u);

  EXPECT_EQ(*sim.signal_group_to_entry_lanelet_ids_[1].begin(), 1210);
  EXPECT_EQ(*sim.signal_group_to_exit_lanelet_ids_[1].begin(), 1220);
  EXPECT_EQ(sim.signal_group_to_traffic_light_id_[1], traffic_signals.front()->id());
  EXPECT_EQ(sim.intersection_id_to_regem_id_.size(), 1u);
  EXPECT_EQ(intersections.size(), 1u);
  EXPECT_EQ(traffic_signals.size(), 1u);

}

TEST(SignalizedIntersectionManger, processSpatFromMsg)
{
  auto pl1 = carma_wm::getPoint(0, 0, 0);
  auto pl2 = carma_wm::getPoint(0, 1, 0);
  auto pl3 = carma_wm::getPoint(0, 2, 0);
  auto pr1 = carma_wm::getPoint(1, 0, 0);
  auto pr2 = carma_wm::getPoint(1, 1, 0);
  auto pr3 = carma_wm::getPoint(1, 2, 0);
  std::vector<lanelet::Point3d> left_1 = { pl1, pl2, pl3 };
  std::vector<lanelet::Point3d> right_1 = { pr1, pr2, pr3 };
  auto ll_1 = carma_wm::getLanelet(left_1, right_1, lanelet::AttributeValueString::SolidDashed,lanelet::AttributeValueString::Dashed);
  lanelet::Id traffic_light_id = lanelet::utils::getId();
  lanelet::LineString3d virtual_stop_line(lanelet::utils::getId(), {pl2, pr2});
  // Creat passing control line for solid dashed line
  std::shared_ptr<lanelet::CarmaTrafficSignal> traffic_light(new lanelet::CarmaTrafficSignal(lanelet::CarmaTrafficSignal::buildData(traffic_light_id, { virtual_stop_line }, { ll_1 }, { ll_1 })));
  traffic_light->revision_ = 0;
  ll_1.addRegulatoryElement(traffic_light);
  auto map_unique_ptr = lanelet::utils::createMap({ ll_1 }, {});
  // Turn unique ptr into shared so we can easily manipulate
  std::shared_ptr<lanelet::LaneletMap> map = std::move(map_unique_ptr);
  map->add(traffic_light);

  uint16_t intersection_id=1;
  uint8_t signal_group_id=1;
  carma_wm::SignalizedIntersectionManager sim;
  sim.intersection_id_to_regem_id_[intersection_id] = 1001;
  sim.signal_group_to_traffic_light_id_[signal_group_id] = traffic_light_id;

  // create sample SPAT.msg and fill its entries
  carma_v2x_msgs::msg::SPAT spat;
  carma_v2x_msgs::msg::IntersectionState state;
  state.id.id = 1;
  state.revision = 0;
  carma_v2x_msgs::msg::MovementState movement;
  movement.signal_group = 1;
  carma_v2x_msgs::msg::MovementEvent event;

  // call the processSpatFromMsg with that msg 1
  event.event_state.movement_phase_state = 5;
  event.timing.min_end_time = 20;
  event.timing.start_time = 0;
  movement.movement_event_list.push_back(event);
  state.movement_list.push_back(movement);
  spat.intersection_state_list.push_back(state);
  sim.processSpatFromMsg(spat, map);
  sim.phase_type_ = carma_wm::signalized_intersection_manager::PHASE_TYPE::DYNAMIC; //dynamic
  auto lights1 = map->laneletLayer.get(ll_1.id()).regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
  // By default, traffic_signal shouldn't have fixed_cycle_duration
  EXPECT_EQ(lanelet::time::durationFromSec(0), lights1[0]->fixed_cycle_duration);

  // Do nothing if received SPAT of another intersection
  state.id.id = 2;
  spat.intersection_state_list.push_back(state);
  sim.processSpatFromMsg(spat, map);
  EXPECT_EQ(lanelet::time::durationFromSec(0), lights1[0]->fixed_cycle_duration);
  EXPECT_EQ(1, lights1[0]->recorded_time_stamps.size());
  EXPECT_EQ(1, lights1[0]->recorded_start_time_stamps.size());

  // Valid dynamic spat
  state.id.id = 1;
  spat.intersection_state_list = {};
  spat.intersection_state_list.push_back(state);
  sim.processSpatFromMsg(spat, map);
  EXPECT_EQ(lanelet::time::durationFromSec(0), lights1[0]->fixed_cycle_duration);
  EXPECT_EQ(1, lights1[0]->recorded_time_stamps.size());
  EXPECT_EQ(1, lights1[0]->recorded_start_time_stamps.size());
  EXPECT_EQ(20, lanelet::time::toSec(lights1[0]->recorded_time_stamps.front().first));
  EXPECT_EQ(0, lanelet::time::toSec(lights1[0]->recorded_start_time_stamps.front()));
  EXPECT_EQ(lanelet::CarmaTrafficSignalState::PERMISSIVE_MOVEMENT_ALLOWED, lights1[0]->recorded_time_stamps.front().second);

  // Empty movement, stay the same
  state.id.id = 1;
  carma_v2x_msgs::msg::MovementState empty_movement;
  spat.intersection_state_list[0].movement_list[0] = empty_movement;
  sim.processSpatFromMsg(spat, map);
  EXPECT_EQ(1, lights1[0]->recorded_time_stamps.size());
  EXPECT_EQ(1, lights1[0]->recorded_start_time_stamps.size());

  // Multiple states
  // first state
  spat.intersection_state_list[0] = state;
  // second state
  event.event_state.movement_phase_state = 3;
  event.timing.min_end_time = 40;
  event.timing.start_time = 20;
  movement.movement_event_list.push_back(event);
  state.movement_list.push_back(movement);
  spat.intersection_state_list.push_back(state);
  sim.processSpatFromMsg(spat, map);
  EXPECT_EQ(2, lights1[0]->recorded_time_stamps.size());
  EXPECT_EQ(2, lights1[0]->recorded_start_time_stamps.size());
  EXPECT_NEAR(20.0, lanelet::time::toSec(lights1[0]->recorded_time_stamps.front().first), 0.0001);
  EXPECT_NEAR(0.0, lanelet::time::toSec(lights1[0]->recorded_start_time_stamps.front()),  0.0001);
  EXPECT_EQ(lanelet::CarmaTrafficSignalState::PERMISSIVE_MOVEMENT_ALLOWED, lights1[0]->recorded_time_stamps.front().second);
  EXPECT_NEAR(40.0, lanelet::time::toSec(lights1[0]->recorded_time_stamps.back().first),  0.0001);
  EXPECT_NEAR(20.0, lanelet::time::toSec(lights1[0]->recorded_start_time_stamps.back()),  0.0001);
  EXPECT_EQ(lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN, lights1[0]->recorded_time_stamps.back().second);
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

  EXPECT_EQ(sim.signal_group_to_entry_lanelet_ids_.size(), 1u);
  EXPECT_EQ(sim.signal_group_to_exit_lanelet_ids_.size(), 1u);
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

  EXPECT_EQ(interior.size(), 4u);


}

TEST(SignalizedIntersectionManger, getTrafficSignalId)
{
  carma_wm::SignalizedIntersectionManager sim;
  uint16_t intersection_id=1;
  uint8_t signal_group_id=1;
  sim.intersection_id_to_regem_id_[intersection_id] = 1001;
  sim.signal_group_to_traffic_light_id_[signal_group_id] = 1000;

  EXPECT_EQ(sim.getTrafficSignalId(intersection_id, signal_group_id), 1000);
}

}  // namespace carma_wm_ctrl