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
#include <../src/CARMAWorldModel.h>
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
}  // namespace carma_wm