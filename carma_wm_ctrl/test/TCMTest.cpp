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
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>
#include <carma_wm_ctrl/GeofenceSchedule.hpp>
#include <carma_wm_ctrl/Geofence.hpp>
#include <carma_wm_ros2/TrafficControl.hpp>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <carma_wm_ctrl/GeofenceScheduler.hpp>
#include <carma_wm_ctrl/WMBroadcaster.hpp>
#include <autoware_lanelet2_ros2_interface/utility/message_conversion.hpp>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <memory>
#include <chrono>
#include <ctime>
#include <atomic>
#include <carma_ros2_utils/testing/TestHelpers.hpp>
#include <carma_ros2_utils/timers/testing/TestTimer.hpp>
#include <carma_ros2_utils/timers/testing/TestTimerFactory.hpp>
#include <algorithm>

#include <carma_planning_msgs/msg/route.hpp>
#include <carma_v2x_msgs/msg/traffic_control_message.hpp>
#include <carma_perception_msgs/msg/check_active_geofence.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/functional/hash.hpp>
#include <geometry_msgs/msg/pose_stamped.h>

#include "TestHelpers.hpp"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

using carma_ros2_utils::timers::testing::TestTimer;
using carma_ros2_utils::timers::testing::TestTimerFactory;

namespace carma_wm_ctrl

{
/**
 * This test is for manually debugging the geofence callback when an actual map is available. It can be left disabled in most cases
 */ 
TEST(WMBroadcaster, DISABLED_geofenceCallback)
{  //
  WMBroadcaster wmb([](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {},
                    [](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {},
                    [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_) {},
                    [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_) {},
                    std::make_unique<TestTimerFactory>(),
                    [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});  // Create broadcaster with test timers. Having this check
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

  std_msgs::msg::String str_msg;
  str_msg.data = target_frame;
  RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Projection: " << target_frame);
  wmb.geoReferenceCallback(str_msg);

  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);

  autoware_lanelet2_msgs::msg::MapBin::SharedPtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));

  wmb.baseMapCallback(map_msg_ptr);


  carma_v2x_msgs::msg::TrafficControlMessage tcm;
  tcm.choice = 1;
  tcm.tcm_v01.reqid.id = {0,0,0,0,0,0,0,0}; // Original id was {254, 16, 166, 86, 224, 213, 73, 249} changed to broadcast to support testing
  tcm.tcm_v01.reqseq = 0;
  tcm.tcm_v01.msgtot = 2;
  tcm.tcm_v01.msgnum = 1;
  tcm.tcm_v01.id.id = {245, 58, 198, 220, 227, 49, 222, 135, 14, 191, 107, 175, 134, 207, 23, 89};
  tcm.tcm_v01.updated.sec = 0;
  tcm.tcm_v01.updated.nsec = 0;
  tcm.tcm_v01.package.label = "weather";
  tcm.tcm_v01.package.label_exists = true;
  j2735_v2x_msgs::msg::Id128b gid;
  gid.id = { 245, 58, 198, 220, 227, 49, 222, 135, 14, 191, 107, 175, 134, 207, 23, 89 };
  tcm.tcm_v01.package.tcids = { gid };
  tcm.tcm_v01.package_exists = true;
  j2735_v2x_msgs::msg::TrafficControlVehClass a,b,c,d,e,f,g,h,i,j,k,l,m;
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
  tcm.tcm_v01.params.vclasses = { a,b,c,d,e,f,g,h,i,j,k,l,m };
  tcm.tcm_v01.params.schedule.start.sec = 1624476360;
  tcm.tcm_v01.params.schedule.start.nsec = 0;
  tcm.tcm_v01.params.schedule.dow.dow = { 1,1,1,1,1,1,1 };
  tcm.tcm_v01.params.schedule.dow_exists = true;
  tcm.tcm_v01.params.regulatory = true;
  tcm.tcm_v01.params.detail.choice = 5;
  tcm.tcm_v01.params.detail.closed = 1;
  tcm.tcm_v01.params_exists = true;
  tcm.tcm_v01.geometry.proj = "epsg:3785";
  tcm.tcm_v01.geometry.datum = "WGS84";
  tcm.tcm_v01.geometry.reftime.sec = 2891441912;
  tcm.tcm_v01.geometry.reftime.nsec = 0;
  tcm.tcm_v01.geometry.reflon = -77.9696101;
  tcm.tcm_v01.geometry.reflat = 39.2339986;
  tcm.tcm_v01.geometry.refelv = -409.600006104;
  tcm.tcm_v01.geometry.heading = 331.299987793;

  carma_v2x_msgs::msg::PathNode aa,bb,cc;
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

  tcm.tcm_v01.geometry.nodes = { aa, bb, cc };
  tcm.tcm_v01.geometry_exists = true;

  wmb.geofenceCallback(tcm);

}
}  // namespace carma_wm_ctrl
