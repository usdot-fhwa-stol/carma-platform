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
#include <iostream>
#include <autoware_lanelet2_ros2_interface/utility/message_conversion.hpp>
#include <../src/WMListenerWorker.hpp>
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <boost/archive/binary_oarchive.hpp>
#include <sstream>
#include <string>
#include "TestHelpers.hpp"
#include <carma_wm_ros2/MapConformer.hpp>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <autoware_lanelet2_ros2_interface/utility/query.hpp>
#include <autoware_lanelet2_ros2_interface/utility/utilities.hpp>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>


namespace carma_wm
{
TEST(WMListenerWorkerTest, constructor)
{
  WMListenerWorker wmlw;

  ASSERT_TRUE((bool)wmlw.getWorldModel());
}

TEST(WMListenerWorkerTest, mapCallback)
{
  CARMAWorldModel cwm;

  addStraightRoute(cwm);

  ASSERT_TRUE((bool)cwm.getMap());
  ASSERT_TRUE((bool)cwm.getRoute());
  ASSERT_TRUE((bool)cwm.getMapRoutingGraph());

  auto map_ptr = lanelet::utils::removeConst(cwm.getMap());

  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map_ptr, &msg);

  WMListenerWorker wmlw;

  ///// Test map not set
  ASSERT_FALSE((bool)(wmlw.getWorldModel()->getMap()));

  ///// Test Map callback without user callback
  wmlw.mapCallback(std::make_unique<autoware_lanelet2_msgs::msg::MapBin>(msg));
  ASSERT_TRUE((bool)(wmlw.getWorldModel()->getMap()));

  ///// Test user defined callback
  bool flag = false;
  ASSERT_FALSE(flag);

  wmlw.setMapCallback([&flag]() { flag = true; });

  wmlw.mapCallback(std::make_unique<autoware_lanelet2_msgs::msg::MapBin>(msg));

  ASSERT_TRUE(flag);
}

TEST(WMListenerWorkerTest, routeCallback)
{
  WMListenerWorker wmlw;

  CARMAWorldModel cwm;

  addStraightRoute(cwm);

  ASSERT_TRUE((bool)cwm.getMap());
  ASSERT_TRUE((bool)cwm.getRoute());
  ASSERT_TRUE((bool)cwm.getMapRoutingGraph());

  auto map_ptr = lanelet::utils::removeConst(cwm.getMap());

  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map_ptr, &msg);

  carma_planning_msgs::msg::Route route_msg;
  route_msg.shortest_path_lanelet_ids.push_back(cwm.getRoute()->shortestPath()[0].id());
  route_msg.shortest_path_lanelet_ids.push_back(cwm.getRoute()->shortestPath()[1].id());

  //// Test route callback without map to verify no crash
  wmlw.routeCallback(std::make_unique<carma_planning_msgs::msg::Route>(route_msg));

  wmlw.mapCallback(std::make_unique<autoware_lanelet2_msgs::msg::MapBin>(msg));

  ///// Test without user defined route callback
  wmlw.routeCallback(std::make_unique<carma_planning_msgs::msg::Route>(route_msg));

  bool flag = false;

  ///// Test without user defined route callback
  wmlw.routeCallback(std::make_unique<carma_planning_msgs::msg::Route>(route_msg));

  ASSERT_FALSE(flag);

  ///// Test with user defined route callback
  wmlw.setRouteCallback([&flag]() { flag = true; });

  wmlw.routeCallback(std::make_unique<carma_planning_msgs::msg::Route>(route_msg));

  ASSERT_TRUE(flag);

  ///test route_node_flag_ and rerouting_flag_
  autoware_lanelet2_msgs::msg::MapBin geofence_msg;
  geofence_msg.invalidates_route=true;

  wmlw.mapUpdateCallback(geofence_msg);
  wmlw.enableUpdatesWithoutRoute();
  wmlw.routeCallback(std::make_unique<carma_planning_msgs::msg::Route>(route_msg));
  
}

TEST(WMListenerWorkerTest, mapUpdateCallback)
{
  // build the geofence msg to test the mapUpdateCallback
  using namespace lanelet::units::literals;
  // add a lanelet
  auto p1 = getPoint(0, 0, 0);
  auto p2 = getPoint(0, 1, 0);
  auto p3 = getPoint(1, 1, 0);
  auto p4 = getPoint(1, 0, 0);
  lanelet::LineString3d left_ls_1(lanelet::utils::getId(), { p1, p2 });
  lanelet::LineString3d right_ls_1(lanelet::utils::getId(), { p4, p3 });

  auto ll_1 = getLanelet(left_ls_1, right_ls_1, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);
  auto ll_2 = getLanelet(left_ls_1, right_ls_1, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);
  // add regems

  lanelet::DigitalSpeedLimitPtr speed_limit_old = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(9000, 5_mph, {ll_1}, {},
                                                     { lanelet::Participants::VehicleCar }));
  lanelet::DigitalSpeedLimitPtr speed_limit_new = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(9001, 5_mph, {ll_1}, {},
                                                     { lanelet::Participants::VehicleCar }));

  // Create the geofence object
  auto gf_ptr = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  gf_ptr->id_ = boost::uuids::random_generator()();
  gf_ptr->remove_list_.push_back(std::make_pair(ll_1.id(), speed_limit_old));
  gf_ptr->update_list_.push_back(std::make_pair(ll_1.id(), speed_limit_new));

  // from broadcaster
  autoware_lanelet2_msgs::msg::MapBin gf_obj_msg;
  auto received_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_, {ll_2}));
  carma_wm::toBinMsg(received_data, &gf_obj_msg);

  // create a listener
  WMListenerWorker wmlw; 
  // create basic map
  ll_1.addRegulatoryElement(speed_limit_old);
  lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1 }, { });
  autoware_lanelet2_msgs::msg::MapBin map_msg;
  lanelet::utils::conversion::toBinMsg(map, &map_msg);

  wmlw.mapCallback(std::make_unique<autoware_lanelet2_msgs::msg::MapBin>(map_msg));

  // make sure it had old speed limit before
  auto regems = wmlw.getWorldModel()->getMap()->laneletLayer.get(ll_1.id()).regulatoryElements();
  ASSERT_EQ(regems.size(), 1);
  ASSERT_EQ(regems[0]->id(), speed_limit_old->id());
  ASSERT_NE(wmlw.getWorldModel()->getMap()->regulatoryElementLayer.find(speed_limit_old->id()), 
            wmlw.getWorldModel()->getMap()->regulatoryElementLayer.end());

  // test the MapUpdateCallback
  wmlw.mapUpdateCallback(gf_obj_msg);
  
  // check if the map has the new speed limit now
  regems = wmlw.getWorldModel()->getMap()->laneletLayer.get(ll_1.id()).regulatoryElements();
  ASSERT_EQ(regems.size(), 1);
  ASSERT_EQ(regems[0]->id(), speed_limit_new->id());
  // and it is queryable:
  ASSERT_NE(wmlw.getWorldModel()->getMap()->regulatoryElementLayer.find(speed_limit_new->id()), 
            wmlw.getWorldModel()->getMap()->regulatoryElementLayer.end());
  ASSERT_EQ(wmlw.getWorldModel()->getMap()->laneletLayer.findUsages(speed_limit_new).size(), 0); // same element not not queryable here because different node
                                                                                                 // using different data address. This should be 0
  auto new_regem_correct_data = wmlw.getWorldModel()->getMap()->regulatoryElementLayer.get(speed_limit_new->id());
  ASSERT_EQ(wmlw.getWorldModel()->getMap()->laneletLayer.findUsages(new_regem_correct_data).size(), 1); // now queryable here because of same element with correct data address
  ASSERT_EQ(wmlw.getWorldModel()->getMap()->laneletLayer.findUsages(new_regem_correct_data)[0].id(), ll_1.id());
  // check if the map has the new lanelet now
  ASSERT_EQ(wmlw.getWorldModel()->getMap()->laneletLayer.size(), 2);
  ASSERT_TRUE(wmlw.getWorldModel()->getMap()->laneletLayer.exists(ll_2.id()));
  
  // but old regem should still be there as the updater doesn't completely delete old regems, but only sever the connections
  auto regem_old_correct_data = wmlw.getWorldModel()->getMap()->regulatoryElementLayer.get(speed_limit_old->id());
  ASSERT_NE(wmlw.getWorldModel()->getMap()->regulatoryElementLayer.find(speed_limit_old->id()), 
            wmlw.getWorldModel()->getMap()->regulatoryElementLayer.end());
  ASSERT_EQ(wmlw.getWorldModel()->getMap()->laneletLayer.findUsages(regem_old_correct_data).size(), 0);
  ASSERT_EQ(wmlw.getWorldModel()->getMap()->laneletLayer.findUsages(speed_limit_old).size(), 0);
  
  // now the change should be reversable
  gf_ptr->update_list_ = {};
  gf_ptr->remove_list_ = {};
  gf_ptr->update_list_.push_back(std::make_pair(ll_1.id(), speed_limit_old));
  gf_ptr->remove_list_.push_back(std::make_pair(ll_1.id(), speed_limit_new));

  // reverse ros msg from broadcaster
  autoware_lanelet2_msgs::msg::MapBin gf_reverse_msg;
  auto reverse_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_, {}));
  carma_wm::toBinMsg(reverse_data, &gf_reverse_msg);
  gf_reverse_msg.seq_id +=2;

  // test the MapUpdateCallback reverse
  auto gf_rev_msg_ptr =  boost::make_shared<autoware_lanelet2_msgs::msg::MapBin>(gf_reverse_msg);
  gf_obj_msg.seq_id ++;
  EXPECT_THROW(wmlw.mapUpdateCallback(gf_obj_msg), lanelet::InvalidInputError); // because we are trying update the exact same llt and regem relationship again
  wmlw.mapUpdateCallback(gf_reverse_msg);

  // check above conditions again on old speed
  regems = wmlw.getWorldModel()->getMap()->laneletLayer.get(ll_1.id()).regulatoryElements();
  ASSERT_EQ(regems.size(), 1);
  ASSERT_EQ(regems[0]->id(), speed_limit_old->id()); //TODO- fix
  // and it is queryable:
  ASSERT_NE(wmlw.getWorldModel()->getMap()->regulatoryElementLayer.find(speed_limit_old->id()), 
            wmlw.getWorldModel()->getMap()->regulatoryElementLayer.end());

  // old_speed_limit's data is also stored at a different address from the one we created locally because
  // we serialized the whole map and deserialized before setting the map.
  ASSERT_EQ(wmlw.getWorldModel()->getMap()->regulatoryElementLayer.get(speed_limit_old->id()), regem_old_correct_data); //TODO- fix
  ASSERT_EQ(wmlw.getWorldModel()->getMap()->laneletLayer.findUsages(regem_old_correct_data).size(), 1);
  ASSERT_EQ(wmlw.getWorldModel()->getMap()->laneletLayer.findUsages(regem_old_correct_data)[0].id(), ll_1.id()); //TODO- fix
  
}

TEST(WMListenerWorkerTest, setConfigSpeedLimitTest)
{
  WMListenerWorker wmlw;

  bool flag = false;
  double cL = 24.0;
  ///// Test without user defined config limit
  wmlw.setConfigSpeedLimit(cL);

  double current_cl;

  current_cl = wmlw.getConfigSpeedLimit();

  ASSERT_EQ(cL, current_cl);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("WMListenerWorkerTest"), "config_speed_limit = "<< current_cl);

}

TEST(WMListenerWorkerTest, getVehicleParticipationTypeTest)
{
  WMListenerWorker wmlw;

  bool flag = false;
  std::string pt = lanelet::Participants::VehicleTruck;
  ///// Test without user defined config limit
  wmlw.setVehicleParticipationType(pt);

  std::string current_pt;

  current_pt = wmlw.getVehicleParticipationType();

  ASSERT_EQ(pt, current_pt);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("WMListenerWorkerTest"),"Participant = "<< current_pt);

}

TEST(WMListenerWorkerTest, checkIfReRoutingNeeded1)
{
  WMListenerWorker wmlw;
  ASSERT_EQ(false, wmlw.checkIfReRoutingNeeded());
}

TEST(WMListenerWorkerTest, checkIfReRoutingNeeded2)
{
  WMListenerWorker wmlw;
  CARMAWorldModel cwm;

  addStraightRoute(cwm);

  auto map_ptr = lanelet::utils::removeConst(cwm.getMap());

  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map_ptr, &msg);

  wmlw.mapCallback(std::make_unique<autoware_lanelet2_msgs::msg::MapBin>(msg));

  carma_planning_msgs::msg::Route route_msg;
  route_msg.shortest_path_lanelet_ids.push_back(cwm.getRoute()->shortestPath()[0].id());
  route_msg.shortest_path_lanelet_ids.push_back(cwm.getRoute()->shortestPath()[1].id());
  wmlw.routeCallback(std::make_unique<carma_planning_msgs::msg::Route>(route_msg));

  autoware_lanelet2_msgs::msg::MapBin geofence_msg;
  geofence_msg.invalidates_route=true;

  wmlw.mapUpdateCallback(geofence_msg);
  ASSERT_EQ(true, wmlw.checkIfReRoutingNeeded());
}

}  // namespace carma_wm
