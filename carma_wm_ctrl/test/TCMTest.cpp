/*
 * Copyright (C) 2020 LEIDOS.
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
#include <carma_wm/WMTestLibForGuidance.h>
#include <carma_wm_ctrl/GeofenceSchedule.h>
#include <carma_wm_ctrl/Geofence.h>
#include <carma_wm/TrafficControl.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <carma_wm_ctrl/GeofenceScheduler.h>
#include <carma_wm_ctrl/WMBroadcaster.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <memory>
#include <chrono>
#include <ctime>
#include <atomic>
#include <carma_utils/testing/TestHelpers.h>
#include <carma_utils/timers/testing/TestTimer.h>
#include <carma_utils/timers/testing/TestTimerFactory.h>
#include <algorithm>

#include <cav_msgs/Route.h>
#include <cav_msgs/TrafficControlMessage.h>
#include <cav_msgs/CheckActiveGeofence.h>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/functional/hash.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <cav_msgs/TrafficControlMessage.h>

#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

using carma_utils::timers::testing::TestTimer;
using carma_utils::timers::testing::TestTimerFactory;

namespace carma_wm_ctrl

{
/**
 * This test is for manually debugging the geofence callback when an actual map is available. It can be left disabled in most cases
 */ 
TEST(WMBroadcaster, DISABLED_geofenceCallback)
{  //
  WMBroadcaster wmb([](const autoware_lanelet2_msgs::MapBin& map_bin) {},
                    [](const autoware_lanelet2_msgs::MapBin& map_bin) {},
                    [](const cav_msgs::TrafficControlRequest& control_msg_pub_) {},
                    [](const cav_msgs::CheckActiveGeofence& active_pub_) {},
                    std::make_unique<TestTimerFactory>());  // Create broadcaster with test timers. Having this check
                                                            // helps verify that the timers do not crash on destruction

  wmb.setConfigSpeedLimit(35.0);
  wmb.setMaxLaneWidth(4.0);

  //

  // File to process. Path is relative to test folder
  std::string file = "/workspaces/carma_ws/carma/src/carma-platform/carma_wm_ctrl/test/resource/"
                     "Summit_Point_split_25mph_verification_for_test.osm";

  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;
  // Parse geo reference info from the original lanelet map (.osm)
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);

  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());

  lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

  if (map->laneletLayer.size() == 0)
  {
    FAIL() << "Input map does not contain any lanelets";
  }

  std_msgs::String str_msg;
  str_msg.data = target_frame;
  ROS_WARN_STREAM("Projection: " << target_frame);
  wmb.geoReferenceCallback(str_msg);

  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);

  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  wmb.baseMapCallback(map_msg_ptr);


  cav_msgs::TrafficControlMessage tcm;
  tcm.choice = 1;
  tcm.tcmV01.reqid.id = {0,0,0,0,0,0,0,0}; // Original id was {254, 16, 166, 86, 224, 213, 73, 249} changed to broadcast to support testing
  tcm.tcmV01.reqseq = 0;
  tcm.tcmV01.msgtot = 2;
  tcm.tcmV01.msgnum = 1;
  tcm.tcmV01.id.id = {245, 58, 198, 220, 227, 49, 222, 135, 14, 191, 107, 175, 134, 207, 23, 89};
  tcm.tcmV01.updated.sec = 0;
  tcm.tcmV01.updated.nsec = 0;
  tcm.tcmV01.package.label = "weather";
  tcm.tcmV01.package.label_exists = true;
  j2735_msgs::Id128b gid;
  gid.id = { 245, 58, 198, 220, 227, 49, 222, 135, 14, 191, 107, 175, 134, 207, 23, 89 };
  tcm.tcmV01.package.tcids = { gid };
  tcm.tcmV01.package_exists = true;
  j2735_msgs::TrafficControlVehClass a,b,c,d,e,f,g,h,i,j,k,l,m;
  a.vehicle_class = 4;
  b.vehicle_class = 5;
  c.vehicle_class = 6;
  d.vehicle_class = 7;
  e.vehicle_class = 8;
  f.vehicle_class = 9;
  g.vehicle_class = 10;
  h.vehicle_class = 11;
  i.vehicle_class = 12;
  j.vehicle_class = 13;
  k.vehicle_class = 14;
  l.vehicle_class = 15;
  m.vehicle_class = 16;
  tcm.tcmV01.params.vclasses = { a,b,c,d,e,f,g,h,i,j,k,l,m };
  tcm.tcmV01.params.schedule.start.sec = 1624476360;
  tcm.tcmV01.params.schedule.start.nsec = 0;
  tcm.tcmV01.params.schedule.dow.dow = { 1,1,1,1,1,1,1 };
  tcm.tcmV01.params.schedule.dow_exists = true;
  tcm.tcmV01.params.regulatory = true;
  tcm.tcmV01.params.detail.choice = 5;
  tcm.tcmV01.params.detail.closed = 1;
  tcm.tcmV01.params_exists = true;
  tcm.tcmV01.geometry.proj = "epsg:3785";
  tcm.tcmV01.geometry.datum = "WGS84";
  tcm.tcmV01.geometry.reftime.sec = 2891441912;
  tcm.tcmV01.geometry.reftime.nsec = 0;
  tcm.tcmV01.geometry.reflon = -77.9696101;
  tcm.tcmV01.geometry.reflat = 39.2339986;
  tcm.tcmV01.geometry.refelv = -409.600006104;
  tcm.tcmV01.geometry.heading = 331.299987793;

  cav_msgs::PathNode aa,bb,cc;
  aa.x = 0.0;
  aa.y = 0.0;
  aa.width = 0.0;
  aa.width_exists = true;

  bb.x = -15.09;
  bb.y = -44.58;
  bb.width = -0.0399999991059;
  bb.width_exists = true;

  cc.x = -15.08;
  cc.y = -44.58;
  cc.width = -0.00999999977648;
  cc.width_exists = true;

  tcm.tcmV01.geometry.nodes = { aa, bb, cc };
  tcm.tcmV01.geometry_exists = true;

  wmb.geofenceCallback(tcm);

}
}  // namespace carma_wm_ctrl
