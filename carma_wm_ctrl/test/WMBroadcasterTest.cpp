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
#include <carma_wm_ctrl/GeofenceSchedule.h>
#include <carma_wm_ctrl/Geofence.h>
#include <carma_wm_ctrl/GeofenceScheduler.h>
#include <carma_wm_ctrl/ROSTimerFactory.h>
#include <carma_wm_ctrl/WMBroadcaster.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <memory>
#include <chrono>
#include <ctime>
#include <atomic>
#include "TestHelpers.h"
#include "TestTimer.h"
#include "TestTimerFactory.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_wm_ctrl
{
TEST(WMBroadcaster, Constructor)
{
  WMBroadcaster([](const autoware_lanelet2_msgs::MapBin& map_bin) {},
                std::make_unique<TestTimerFactory>());  // Create broadcaster with test timers. Having this check helps
                                                        // verify that the timers do not crash on destruction
}

TEST(WMBroadcaster, baseMapCallback)
{
  ros::Time::setNow(ros::Time(0));  // Set current time

  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(4, map->laneletLayer.size());  // Verify the map can be decoded

        base_map_call_count++;
      },
      std::make_unique<TestTimerFactory>());

  // Get and convert map to binary message
  auto map = carma_wm::getDisjointRouteMap();

  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);

  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(map_msg_ptr);

  ASSERT_EQ(1, base_map_call_count);
}

// Since the actual logic for adding geofences to the map has not yet been added
// this unit test has to be manually verified by looking for the following to log messages
// First "Adding active geofence to the map with geofence id: 1"
// Second "Removing inactive geofence to the map with geofence id: 1"
// Once said logic is added this unit test should be updated
TEST(WMBroadcaster, geofenceCallback)
{
  // Test adding then evaluate if the calls to active and inactive are done correctly
  Geofence gf;
  gf.id_ = 1;
  gf.schedule = GeofenceSchedule(ros::Time(1),  // Schedule between 1 and 8
                                 ros::Time(8),
                                 ros::Duration(2),    // Start's at 2
                                 ros::Duration(3.1),  // Ends at by 3.1
                                 ros::Duration(1),    // Duration of 1 and interval of two so active durations are (2-3)
                                 ros::Duration(2));
  ros::Time::setNow(ros::Time(0));  // Set current time

  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(4, map->laneletLayer.size());  // Verify the map can be decoded

        base_map_call_count++;
      },
      std::make_unique<TestTimerFactory>());

  // Get and convert map to binary message
  auto map = carma_wm::getDisjointRouteMap();

  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);

  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(map_msg_ptr);

  ASSERT_EQ(1, base_map_call_count);

  // Verify adding geofence call
  wmb.geofenceCallback(gf);

  ros::Time::setNow(ros::Time(2.1));  // Set current time

  std::atomic<uint32_t> temp(0);
  carma_wm::waitForEqOrTimeout(3.0, 1, temp);

  ros::Time::setNow(ros::Time(3.1));  // Set current time

  carma_wm::waitForEqOrTimeout(3.0, 1, temp);
}

}  // namespace carma_wm_ctrl