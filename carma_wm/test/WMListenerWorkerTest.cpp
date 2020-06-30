/*
 * Copyright (C) 2019 LEIDOS.
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
#include <iostream>
#include <lanelet2_extension/utility/message_conversion.h>
#include <../src/WMListenerWorker.h>
#include <carma_wm/CARMAWorldModel.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <boost/archive/binary_oarchive.hpp>
#include <sstream>
#include <string>
#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

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

  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map_ptr, &msg);

  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  WMListenerWorker wmlw;

  ///// Test map not set
  ASSERT_FALSE((bool)(wmlw.getWorldModel()->getMap()));

  ///// Test Map callback without user callback
  wmlw.mapCallback(map_msg_ptr);
  ASSERT_TRUE((bool)(wmlw.getWorldModel()->getMap()));

  ///// Test user defined callback
  bool flag = false;
  ASSERT_FALSE(flag);

  wmlw.setMapCallback([&flag]() { flag = true; });

  wmlw.mapCallback(map_msg_ptr);

  ASSERT_TRUE(flag);
}

TEST(WMListenerWorkerTest, routeCallback)
{
  WMListenerWorker wmlw;

  bool flag = false;

  ///// Test without user defined route callback
  wmlw.routeCallback();

  ASSERT_FALSE(flag);

  ///// Test with user defined route callback
  wmlw.setRouteCallback([&flag]() { flag = true; });

  wmlw.routeCallback();

  ASSERT_TRUE(flag);
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
  autoware_lanelet2_msgs::MapBin gf_obj_msg;
  auto received_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_));
  carma_wm::toBinMsg(received_data, &gf_obj_msg);

  // create a listener
  WMListenerWorker wmlw; 
  // create basic map
  ll_1.addRegulatoryElement(speed_limit_old);
  lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1 }, { });
  autoware_lanelet2_msgs::MapBin map_msg;
  lanelet::utils::conversion::toBinMsg(map, &map_msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(map_msg));
  wmlw.mapCallback(map_msg_ptr);

  // make sure it had old speed limit before
  auto regems = wmlw.getWorldModel()->getMap()->laneletLayer.get(ll_1.id()).regulatoryElements();
  ASSERT_EQ(regems.size(), 1);
  ASSERT_EQ(regems[0]->id(), speed_limit_old->id());
  ASSERT_NE(wmlw.getWorldModel()->getMap()->regulatoryElementLayer.find(speed_limit_old->id()), 
            wmlw.getWorldModel()->getMap()->regulatoryElementLayer.end());

  // test the MapUpdateCallback
  auto gf_msg_ptr =  boost::make_shared<const autoware_lanelet2_msgs::MapBin>(gf_obj_msg);
  wmlw.mapUpdateCallback(gf_msg_ptr);
  
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
  // but old regem should still be there as the updater doesn't completely delete old regems, but sever the connections
  ASSERT_NE(wmlw.getWorldModel()->getMap()->regulatoryElementLayer.find(speed_limit_old->id()), 
            wmlw.getWorldModel()->getMap()->regulatoryElementLayer.end());


  // now the change should be reversable
  gf_ptr->update_list_ = {};
  gf_ptr->remove_list_ = {};
  gf_ptr->update_list_.push_back(std::make_pair(ll_1.id(), speed_limit_old));
  gf_ptr->remove_list_.push_back(std::make_pair(ll_1.id(), speed_limit_new));

  // reverse ros msg from broadcaster
  autoware_lanelet2_msgs::MapBin gf_reverse_msg;
  auto reverse_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_));
  carma_wm::toBinMsg(reverse_data, &gf_reverse_msg);

  // test the MapUpdateCallback reverse
  auto gf_rev_msg_ptr =  boost::make_shared<const autoware_lanelet2_msgs::MapBin>(gf_reverse_msg);
  EXPECT_THROW(wmlw.mapUpdateCallback(gf_msg_ptr), lanelet::InvalidInputError); // because we are trying update the exact same llt and regem relationship again
  wmlw.mapUpdateCallback(gf_rev_msg_ptr);

  // check above conditions again on old speed
  regems = wmlw.getWorldModel()->getMap()->laneletLayer.get(ll_1.id()).regulatoryElements();
  ASSERT_EQ(regems.size(), 1);
  ASSERT_EQ(regems[0]->id(), speed_limit_old->id());
  // and it is queryable:
  ASSERT_NE(wmlw.getWorldModel()->getMap()->regulatoryElementLayer.find(speed_limit_old->id()), 
            wmlw.getWorldModel()->getMap()->regulatoryElementLayer.end());

  // old_speed_limit's data is also stored at a different address from the one we created because
  // we serialized the whole map and deserialized before setting the map.
  auto regem_old_correct_data = wmlw.getWorldModel()->getMap()->regulatoryElementLayer.get(speed_limit_old->id());
  ASSERT_EQ(wmlw.getWorldModel()->getMap()->regulatoryElementLayer.get(speed_limit_old->id()), regem_old_correct_data);
  ASSERT_EQ(wmlw.getWorldModel()->getMap()->laneletLayer.findUsages(regem_old_correct_data).size(), 1);
  ASSERT_EQ(wmlw.getWorldModel()->getMap()->laneletLayer.findUsages(regem_old_correct_data)[0].id(), ll_1.id());
}


}  // namespace carma_wm