/*
 * Copyright (C) 2020-2022 LEIDOS.
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
#include <carma_wm/WMTestLibForGuidance.hpp>
#include <lanelet2_extension/time/TimeConversion.h>
#include <carma_wm_ctrl/GeofenceSchedule.hpp>
#include <carma_wm_ctrl/Geofence.hpp>
#include <carma_wm/TrafficControl.hpp>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <carma_wm_ctrl/GeofenceScheduler.hpp>
#include <carma_wm_ctrl/WMBroadcaster.hpp>
#include <autoware_lanelet2_ros2_interface/utility/message_conversion.hpp>
#include <lanelet2_extension/io/autoware_osm_parser.h>
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
#include <builtin_interfaces/msg/time.hpp>


#include "TestHelpers.hpp"

using carma_ros2_utils::timers::testing::TestTimer;
using carma_ros2_utils::timers::testing::TestTimerFactory;

namespace carma_wm_ctrl

{

TEST(WMBroadcaster, Constructor)
{
  WMBroadcaster([](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {}, [](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {},
   [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){}, [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){}, std::make_shared<TestTimerFactory>(),[](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});  // Create broadcaster with test timers. Having this check helps
                                                        // verify that the timers do not crash on destruction
}
TEST(WMBroadcaster, baseMapCallback)
{
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(4, map->laneletLayer.size());  // Verify the map can be decoded

        base_map_call_count++;
      }, [](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {}, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  // Get and convert map to binary message
  auto map = carma_wm::getDisjointRouteMap();

  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);

  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(std::move(map_msg_ptr));

  ASSERT_EQ(1, base_map_call_count);
}

// here test the proj string transform test
TEST(WMBroadcaster, getAffectedLaneletOrAreasFromTransform)
{
  using namespace lanelet::units::literals;
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(4, map->laneletLayer.size());  // Verify the map can be decoded

        base_map_call_count++;
      }, [](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {}, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  //////
  // Get and convert map to binary message
  /////
  auto map = carma_wm::getDisjointRouteMap();
  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));

  // Set the map
  wmb.baseMapCallback(std::move(map_msg_ptr));
  ASSERT_EQ(1, base_map_call_count);

  // Setting georeferences
  // geofence's origin (0,0) is at base_map's (10,10)
  std::string base_map_proj_string, geofence_proj_string;
  std_msgs::msg::String base_map_proj;
  base_map_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  geofence_proj_string = "+proj=tmerc +lat_0=39.46645851394806215 +lon_0=-76.16907903057393980 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  base_map_proj.data = base_map_proj_string;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(base_map_proj));

  // create the geofence request
  carma_v2x_msgs::msg::TrafficControlMessageV01 gf_msg;
  gf_msg.geometry.proj = geofence_proj_string;
  // set the points
  carma_v2x_msgs::msg::PathNode pt;
  // check points that are inside lanelets
  pt.x = -8.5; pt.y = -9.5; pt.z = 0; // straight geofence line across 2 lanelets
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 0.0; pt.y = 1.0; pt.z = 0; //-8.5 -8.5
  gf_msg.geometry.nodes.push_back(pt);
  gf_msg.geometry.datum  = "+proj=tmerc +lat_0=39.46645851394806215 +lon_0=-76.16907903057393980 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  auto gf_pts = wmb.getPointsInLocalFrame(gf_msg);
  lanelet::ConstLaneletOrAreas affected_parts = wmb.getAffectedLaneletOrAreas(gf_pts);
  ASSERT_EQ(affected_parts.size(), 2);
  ASSERT_EQ(affected_parts[0].id(), 10001);
  ASSERT_EQ(affected_parts[1].id(), 10002);
  // check points that are outside, on the edge, and on the point that makes up the lanelets
  pt.x = -11.5; pt.y = -1.5; pt.z = 0; // -20, -10
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 11.0; pt.y = 1.5; pt.z = 0; // -9 -8.5
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 9; pt.y = 8.5; pt.z = 0; // 0 0
  gf_msg.geometry.nodes.push_back(pt);
  
  gf_pts = wmb.getPointsInLocalFrame(gf_msg);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_pts);
  ASSERT_EQ(affected_parts.size(), 2); // newly added ones should not be considered to be on the lanelet
}

// here test assuming the georeference proj strings are the same
TEST(WMBroadcaster, getAffectedLaneletOrAreasOnlyLogic)
{
  using namespace lanelet::units::literals;
  // Set the environment  
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        
        base_map_call_count++;
      }, [](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {}, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  //////
  // Get and convert map to binary message
  /////

  auto map = carma_wm::getBroadcasterTestMap();

  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));

  carma_v2x_msgs::msg::TrafficControlMessageV01 gf_msg;
  // Check if error are correctly being thrown
  EXPECT_THROW(wmb.getPointsInLocalFrame(gf_msg), lanelet::InvalidObjectStateError);
  // Set the map
  wmb.baseMapCallback(std::move(map_msg_ptr));
  ASSERT_EQ(1, base_map_call_count);
  
  EXPECT_THROW(wmb.getPointsInLocalFrame(gf_msg), lanelet::InvalidObjectStateError);
  
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::msg::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  gf_msg.geometry.datum = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(sample_proj_string));

  // create the control message's relevant parts
  gf_msg.geometry.proj = proj_string;
  // set the points
  carma_v2x_msgs::msg::PathNode pt;
  // check points that are inside lanelets
  pt.x = 1.75; pt.y = 0.5; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  auto gf_pts = wmb.getPointsInLocalFrame(gf_msg);

  lanelet::ConstLaneletOrAreas affected_parts = wmb.getAffectedLaneletOrAreas(gf_pts);
  ASSERT_EQ(affected_parts.size(), 0); // this is 0 because there will never be geofence with only 1 pt
                                       // if there is, it won't apply to the map as it doesn't have any direction information, 
                                       // which makes it confusing for overlapping lanelets
  pt.x = 0.0; pt.y = -0.05; pt.z = 0; //1.75 0.45
  gf_msg.geometry.nodes.push_back(pt);
  gf_pts = wmb.getPointsInLocalFrame(gf_msg);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_pts);
  ASSERT_EQ(affected_parts.size(), 0); // although there are two points in the same lanelet,
                                       // lanelet and the two points are not in the same direction
  gf_msg.geometry.nodes.pop_back();
  pt.x = 0.0; pt.y = 0.05; pt.z = 0; //1.75 0.55
  gf_msg.geometry.nodes.push_back(pt);
  gf_pts = wmb.getPointsInLocalFrame(gf_msg);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_pts);
  ASSERT_EQ(affected_parts.size(), 1); // because two points are in one geofence, it will be recorded now
  gf_msg.geometry.nodes.pop_back();
  gf_msg.geometry.nodes.pop_back();
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;    // first of series geofence points across multiple lanelets 0.5 0.5
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 0.0; pt.y = 0.6; pt.z = 0;    // adding point in the next lanelet 0.5 1.1
  gf_msg.geometry.nodes.push_back(pt);
  gf_pts = wmb.getPointsInLocalFrame(gf_msg);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_pts);
  ASSERT_EQ(affected_parts.size(), 3);    // although (0.5,1.1) is in another overlapping lanelet (llt_unreg)
                                          // that lanelet is disjoint/doesnt have same direction/not successor of the any lanelet
  pt.x = 1.0; pt.y = 1.0; pt.z = 0;    // adding further points in different lanelet narrowing down our direction 1.5 2.1
  gf_msg.geometry.nodes.push_back(pt);
  gf_pts = wmb.getPointsInLocalFrame(gf_msg);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_pts);
  ASSERT_EQ(affected_parts.size(), 3);    // now they are actually 3 different lanelets because we changed direction
  std::vector<lanelet::Id> affected_parts_ids;
  for (auto i = 0; i < affected_parts.size(); i ++)
  {
    affected_parts_ids.push_back(affected_parts[i].id());
  }
  ASSERT_TRUE(std::find(affected_parts_ids.begin(), affected_parts_ids.end(), 10003) != affected_parts_ids.end()); //due to race condition
  ASSERT_TRUE(std::find(affected_parts_ids.begin(), affected_parts_ids.end(), 10000) != affected_parts_ids.end());
  ASSERT_TRUE(std::find(affected_parts_ids.begin(), affected_parts_ids.end(), 10006) != affected_parts_ids.end());

  // check points that are outside, on the edge, and on the point that makes up the lanelets
  gf_msg.geometry.nodes = {};
  pt.x = 0.5; pt.y = 0; pt.z = 0; //0.5 0
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 0; pt.z = 0; // 1 0
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 9; pt.y = 10; pt.z = 0; // 10 10
  gf_msg.geometry.nodes.push_back(pt);
  
  gf_pts = wmb.getPointsInLocalFrame(gf_msg);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_pts);
  ASSERT_EQ(affected_parts.size(), 2); // they should not be considered to be on the lanelet
}

TEST(WMBroadcaster, geofenceCallback)
{
  // Test adding then evaluate if the calls to active and inactive are done correctly
  auto gf = std::make_shared<Geofence>();

  boost::uuids::uuid curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  gf->id_ = curr_id;

  gf->schedules.push_back(carma_wm_ctrl::GeofenceSchedule(rclcpp::Time(1e9),  // Schedule between 1 and 8
                                 rclcpp::Time(8e9),
                                 rclcpp::Duration(2e9),    // Starts at 2
                                 rclcpp::Duration(1.1e9),  // Ends at by 3.1
                                 rclcpp::Duration(0),    // 0 offset for repetition start, so still starts at 2
                                 rclcpp::Duration(1e9),    // Duration of 1 and interval of 2 so active durations are (2-3)
                                 rclcpp::Duration(2e9)));
  // convert to ros msg
  carma_v2x_msgs::msg::TrafficControlMessageV01 msg_v01;
  std::copy(gf->id_.begin(),  gf->id_.end(), msg_v01.id.id.begin());
  msg_v01.params.schedule.start = gf->schedules[0].schedule_start_;
  msg_v01.params.schedule.end = gf->schedules[0].schedule_end_;
  msg_v01.params.schedule.end_exists = true;
  msg_v01.params.schedule.between_exists = true;
  msg_v01.params.schedule.repeat_exists = true;
  
  carma_v2x_msgs::msg::DailySchedule daily_schedule;
  daily_schedule.begin = gf->schedules[0].control_start_;
  daily_schedule.duration = gf->schedules[0].control_duration_;
  msg_v01.params.schedule.between.push_back(daily_schedule);
  msg_v01.params.schedule.repeat.offset =  gf->schedules[0].control_offset_;
  msg_v01.params.schedule.repeat.span =  gf->schedules[0].control_span_;
  msg_v01.params.schedule.repeat.period =  gf->schedules[0].control_period_;
  msg_v01.params.schedule.end_exists = true;
  msg_v01.params.schedule.between_exists = true;
  msg_v01.params.schedule.repeat_exists = true;

  // variables needed to test
  size_t base_map_call_count = 0;
  std::atomic<uint32_t> active_call_count(0);
  std::atomic<std::size_t> last_active_gf(0);

  auto timer = std::make_shared<TestTimerFactory>();

  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(4, map->laneletLayer.size());  // Verify the map can be decoded
        base_map_call_count++;
      },
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {
        auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
        carma_wm::fromBinMsg(geofence_bin, data_received);
        ASSERT_EQ(data_received->id_, curr_id);
        ASSERT_EQ(data_received->remove_list_.size(), 0);
        ASSERT_EQ(data_received->update_list_.size(), 0);
        active_call_count.store(active_call_count.load() + 1);
        // atomic is not working for boost::uuids::uuid, so hash it
        last_active_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
        
      }, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      timer, [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  // Verify that nothing is happening when reserved

 // Get and convert map to binary message
  auto map = carma_wm::getDisjointRouteMap();
  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(std::move(map_msg_ptr));
  ASSERT_EQ(1, base_map_call_count);

  // Setting georeferences
  std::string base_map_proj_string, geofence_proj_string;
  std_msgs::msg::String base_map_proj;
  base_map_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  geofence_proj_string = "+proj=tmerc +lat_0=39.46645851394806215 +lon_0=-76.16907903057393980 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs"; //different proj
  base_map_proj.data = base_map_proj_string;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(base_map_proj));

  carma_v2x_msgs::msg::TrafficControlMessage gf_msg;
  gf_msg.tcm_v01.geometry.datum = geofence_proj_string;
  gf_msg.choice = carma_v2x_msgs::msg::TrafficControlMessage::RESERVED;

  ASSERT_EQ(0, active_call_count.load());
  ASSERT_EQ(0, last_active_gf.load());

  timer->setNow(rclcpp::Time(1.0e9));  // Set current time
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg));
  
  ASSERT_EQ(0, active_call_count.load());
  ASSERT_EQ(0, last_active_gf.load());




  timer->setNow(rclcpp::Time(2.1e9));// Geofence should have started by now
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg)); 

  ASSERT_EQ(0, active_call_count.load());
  ASSERT_EQ(0, last_active_gf.load());
  
  
  gf_msg.choice = carma_v2x_msgs::msg::TrafficControlMessage::TCMV01;
  // create the geofence request
  msg_v01.geometry.proj = geofence_proj_string;
  msg_v01.geometry.datum = geofence_proj_string;
  gf_msg.tcm_v01 = msg_v01;

  // every control message needs associated control request id
  carma_planning_msgs::msg::Route route_msg;
  route_msg.route_path_lanelet_ids.push_back(10000);
  std::shared_ptr<j2735_v2x_msgs::msg::Id64b> req_id = std::make_shared<j2735_v2x_msgs::msg::Id64b>(j2735_v2x_msgs::msg::Id64b());
  wmb.controlRequestFromRoute(route_msg, req_id);
  gf_msg.tcm_v01.reqid = *req_id;

  // check geofence with no applicable points
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg));
  timer->setNow(rclcpp::Time(100.0e9)); // set it past control_end so that it does't trigger again

  ASSERT_FALSE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(0, active_call_count.load());
  
  // set the points
  carma_v2x_msgs::msg::PathNode pt;
  // check points that are inside lanelets
  pt.x = -8.5; pt.y = -9.5; pt.z = 0; // straight geofence line across 2 lanelets
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 0.0; pt.y = 1.0; pt.z = 0; // offset from last point
  msg_v01.geometry.nodes.push_back(pt);
  // update id to continue testing
  curr_id = boost::uuids::random_generator()(); 
  curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), msg_v01.id.id.begin());
  msg_v01.reqid = *req_id;
  gf_msg.tcm_v01 = msg_v01;

  timer->setNow(rclcpp::Time(2.1e9)); // set active
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg));
  timer->setNow(rclcpp::Time(100.0e9)); // set it past control_end so that it does't trigger again

  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(1, active_call_count.load());

  // update id to continue testing
  curr_id = boost::uuids::random_generator()(); 
  curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(),  curr_id.end(), msg_v01.id.id.begin());
  gf_msg.tcm_v01 = msg_v01;
  timer->setNow(rclcpp::Time(2.1e9)); // set active
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg));
  timer->setNow(rclcpp::Time(100.0e9)); // set it past control_end so that it does't trigger again
  
  // check if different geofence id is working
  timer->setNow(rclcpp::Time(3.0e9)); // right before finishing at 3.1
  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(2, active_call_count.load());
}
  
TEST(WMBroadcaster, routeCallbackMessage) 
{
  carma_planning_msgs::msg::Route route_msg;
  
  route_msg.route_path_lanelet_ids.push_back(1346);
  route_msg.route_path_lanelet_ids.push_back(1349);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "This is a test: ");
  
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback  
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(2, map->laneletLayer.size());  // Verify the map can be decoded

        base_map_call_count++;
      }, [](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {}, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});
 
  
  //Test throw exceptions
  ASSERT_THROW(wmb.routeCallbackMessage(std::make_unique<carma_planning_msgs::msg::Route>(route_msg)), lanelet::InvalidObjectStateError);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Throw Exceptions Test Passed.");
  
  // Load vector map from a file start 
  std::string file = "resource/test_vector_map1.osm";
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
  // apply loaded map to WMBroadcaster
  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(std::move(map_msg_ptr));
  ASSERT_EQ(1, base_map_call_count);

  //Test target_frame value
  std_msgs::msg::String target;
  target.data = target_frame;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(target));
  ASSERT_FALSE(target_frame.empty());
  // loading end
  
  carma_v2x_msgs::msg::TrafficControlRequest coRe;

  ///// Test without user defined route callback
  coRe = wmb.controlRequestFromRoute(route_msg);

  ASSERT_TRUE(coRe.tcr_v01.bounds.size() > 0);

}

TEST(WMBroadcaster, addAndRemoveGeofence)
{
  using namespace lanelet::units::literals;
  // Set the environment  
  size_t base_map_call_count = 0;
  size_t map_update_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        base_map_call_count++;
      }, 
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map update callback
        map_update_call_count++;
      }, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  //////
  // Set up the map (add relevant regulatory elements)
  /////
  auto map = carma_wm::getBroadcasterTestMap();
  ASSERT_EQ(map->regulatoryElementLayer.size(), 0);
  // add regems
  lanelet::DigitalSpeedLimitPtr old_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::InvalId, 5_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  ASSERT_TRUE(old_speed_limit->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 0);
  map->update(map->laneletLayer.get(10000), old_speed_limit); // added a speed limit to first llt
  
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 1);
  ASSERT_TRUE(map->regulatoryElementLayer.exists(old_speed_limit->id()));
  ASSERT_EQ(map->regulatoryElementLayer.size(), 1);
  ASSERT_EQ(map->laneletLayer.findUsages(old_speed_limit).size(), 1);
  ASSERT_EQ(map->laneletLayer.find(10000)->regulatoryElements().front()->id(), old_speed_limit->id());//should be 10045 old speed limit's id
  
  
  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));
  // Set the map
  wmb.baseMapCallback(std::move(map_msg_ptr));
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::msg::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(sample_proj_string));

  // Create the geofence object
  auto gf_ptr = std::make_shared<carma_wm_ctrl::Geofence>(carma_wm_ctrl::Geofence());
  gf_ptr->id_ = boost::uuids::random_generator()();
  
  carma_v2x_msgs::msg::TrafficControlMessageV01 gf_msg;
  gf_msg.geometry.datum = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  lanelet::DigitalSpeedLimitPtr new_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(map->regulatoryElementLayer.uniqueId(), 10_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  gf_ptr->regulatory_element_ = new_speed_limit;
  // create the control message's relevant parts to fill the object
  gf_msg.geometry.proj = proj_string;
  // set the points
  carma_v2x_msgs::msg::PathNode pt;
  // check points that are inside lanelets
  gf_msg.geometry.nodes = {};
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 1.5; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  gf_ptr->gf_pts = wmb.getPointsInLocalFrame(gf_msg);
  gf_ptr->affected_parts_ = wmb.getAffectedLaneletOrAreas(gf_ptr->gf_pts);
  gf_ptr->msg_ = gf_msg;

  ASSERT_EQ(gf_ptr->affected_parts_.size(), 3);
  ASSERT_EQ(gf_ptr->affected_parts_[0].id(), 10000);
  ASSERT_EQ(gf_ptr->affected_parts_[0].regulatoryElements()[0]->id(), old_speed_limit->id()); // old speed limit
  ASSERT_EQ(gf_ptr->affected_parts_[0].regulatoryElements().size(), 4); // old speed limit and other map conforming regulations
  // process the geofence and change the map

  wmb.addGeofence(gf_ptr);
  ASSERT_EQ(map_update_call_count, 1);
  /*Analyze prev_regems_.size()*/
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 3);
  ASSERT_EQ(gf_ptr->prev_regems_[0].first, 10000);
  ASSERT_EQ(gf_ptr->prev_regems_[0].second->id(), old_speed_limit->id());

  // now suppose the geofence is finished being used, we have to revert the changes
  wmb.removeGeofence(gf_ptr);
  ASSERT_EQ(map_update_call_count, 2);
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 0);

  // we can check if the removeGeofence worked, by using addGeofence again and if the original is there again
  wmb.addGeofence(gf_ptr);
  ASSERT_EQ(map_update_call_count, 3);
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 3);
  ASSERT_EQ(gf_ptr->prev_regems_[0].first, 10000);
  ASSERT_EQ(gf_ptr->prev_regems_[0].second->id(), old_speed_limit->id());

}

TEST(WMBroadcaster, GeofenceBinMsgTest)
{
  using namespace lanelet::units::literals;
  // Set the environment  
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        base_map_call_count++;
      },
      [](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {}, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});
  
  /////
  // Set up the map (add relevant regulatory elements)
  /////
  auto map = carma_wm::getBroadcasterTestMap();
  ASSERT_EQ(map->regulatoryElementLayer.size(), 0);
  // add regems
  lanelet::DigitalSpeedLimitPtr old_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::InvalId, 5_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  ASSERT_TRUE(old_speed_limit->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 0);
  map->update(map->laneletLayer.get(10000), old_speed_limit); // added a speed limit to first llt
  
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 1);
  ASSERT_TRUE(map->regulatoryElementLayer.exists(old_speed_limit->id()));
  ASSERT_EQ(map->regulatoryElementLayer.size(), 1);
  ASSERT_EQ(map->laneletLayer.findUsages(old_speed_limit).size(), 1);
  ASSERT_EQ(map->laneletLayer.find(10000)->regulatoryElements().front()->id(), old_speed_limit->id());//should be 10045 old speed limit's id
  
  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));
  // Set the map
  wmb.baseMapCallback(std::move(map_msg_ptr));
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::msg::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(sample_proj_string));

  // Create the geofence object
  auto gf_ptr = std::make_shared<carma_wm_ctrl::Geofence>(carma_wm_ctrl::Geofence());
  gf_ptr->id_ = boost::uuids::random_generator()();
  
  carma_v2x_msgs::msg::TrafficControlMessageV01 gf_msg;
  gf_msg.geometry.datum = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  lanelet::DigitalSpeedLimitPtr new_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(map->regulatoryElementLayer.uniqueId(), 10_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  gf_ptr->regulatory_element_ = new_speed_limit;
  // create the control message's relevant parts to fill the object
  gf_msg.geometry.proj = proj_string;
  // set the points
  carma_v2x_msgs::msg::PathNode pt;
  // check points that are inside lanelets
  gf_msg.geometry.nodes = {};
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 0.0; pt.y = 1.0; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  gf_ptr->gf_pts = wmb.getPointsInLocalFrame(gf_msg);
  gf_ptr->affected_parts_ = wmb.getAffectedLaneletOrAreas(gf_ptr->gf_pts);
  gf_ptr->msg_ = gf_msg;

  ASSERT_EQ(gf_ptr->affected_parts_.size(), 3);
  ASSERT_EQ(gf_ptr->affected_parts_[0].id(), 10000);
  ASSERT_EQ(gf_ptr->affected_parts_[0].regulatoryElements()[0]->id(), old_speed_limit->id()); // old speed limit
  ASSERT_EQ(gf_ptr->affected_parts_[0].regulatoryElements().size(), 4); // old speed limit and other map conforming regulations
  // process the geofence and change the map

  // flow for adding geofence to the map
  wmb.addGeofence(gf_ptr);
  // from broadcaster
  autoware_lanelet2_msgs::msg::MapBin gf_obj_msg;

  auto send_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_, {}));
  carma_wm::toBinMsg(send_data, &gf_obj_msg);
  // at map users
  auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  carma_wm::fromBinMsg(gf_obj_msg, data_received);
  ASSERT_EQ(data_received->id_, gf_ptr->id_);
  ASSERT_EQ(gf_ptr->remove_list_.size(), 3);
  ASSERT_EQ(data_received->remove_list_.size(), 3); // old_speed_limit
  ASSERT_TRUE(data_received->remove_list_[0].second->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) ==0 );
  ASSERT_EQ(data_received->update_list_.size(), 3); // geofence tags 2 lanelets
  ASSERT_EQ(data_received->update_list_[0].first, 10000);

  // we can see that the gf_ptr->now would have the prev speed limit of 5_mph that affected llt 10000
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 3);
  ASSERT_EQ(gf_ptr->prev_regems_[0].first, 10000);
  ASSERT_EQ(gf_ptr->prev_regems_[0].second->id(), old_speed_limit->id());

  // now suppose the geofence is finished being used, we have to revert the changes
  wmb.removeGeofence(gf_ptr);
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 0); // should be reset
  // from broadcaster
  autoware_lanelet2_msgs::msg::MapBin gf_msg_revert;
  auto send_data_revert = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_, {}));
  carma_wm::toBinMsg(send_data_revert, &gf_msg_revert);
  // at map users
  auto rec_data_revert = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  carma_wm::fromBinMsg(gf_msg_revert, rec_data_revert);

  // previously added update_list_ should be tagged for removal, vice versa
  ASSERT_EQ(rec_data_revert->remove_list_.size(), 3);
  ASSERT_EQ(rec_data_revert->remove_list_.size(), data_received->update_list_.size());
  ASSERT_EQ(rec_data_revert->update_list_.size(), data_received->remove_list_.size());
  ASSERT_EQ(rec_data_revert->update_list_.size(), 3);
  ASSERT_EQ(rec_data_revert->update_list_[0].first, 10000);
  ASSERT_EQ(rec_data_revert->update_list_[0].second->id(), old_speed_limit->id());
  
}

TEST(WMBroadcaster, RegulatoryPCLTest)
{
  // Test adding then evaluate if the calls to active and inactive are done correctly
  auto gf_ptr = std::make_shared<Geofence>();

  boost::uuids::uuid curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  gf_ptr->id_ = curr_id;

  gf_ptr->schedules.push_back(carma_wm_ctrl::GeofenceSchedule(rclcpp::Time(1e9),  // Schedule between 1 and 8
                                 rclcpp::Time(8e9),
                                 rclcpp::Duration(2e9),    // Starts at 2
                                 rclcpp::Duration(1.1e9),  // Ends at by 3.1
                                 rclcpp::Duration(0),    // 0 offset for repetition start, so still starts at 2
                                 rclcpp::Duration(1e9),    // Duration of 1 and interval of two so active durations are (2-3)
                                 rclcpp::Duration(2e9)));
  // convert to ros msg
  carma_v2x_msgs::msg::TrafficControlMessageV01 msg_v01;
  std::copy(gf_ptr->id_.begin(),  gf_ptr->id_.end(), msg_v01.id.id.begin());
  msg_v01.params.schedule.start = gf_ptr->schedules[0].schedule_start_;
  msg_v01.params.schedule.end = gf_ptr->schedules[0].schedule_end_;
  carma_v2x_msgs::msg::DailySchedule daily_schedule;
  daily_schedule.begin = gf_ptr->schedules[0].control_start_;
  daily_schedule.duration = gf_ptr->schedules[0].control_duration_;
  msg_v01.params.schedule.between.push_back(daily_schedule);
  msg_v01.params.schedule.repeat.offset =  gf_ptr->schedules[0].control_offset_;
  msg_v01.params.schedule.repeat.span =  gf_ptr->schedules[0].control_span_;
  msg_v01.params.schedule.repeat.period =  gf_ptr->schedules[0].control_period_;
  msg_v01.params.schedule.end_exists = true;
  msg_v01.params.schedule.between_exists = true;
  msg_v01.params.schedule.repeat_exists = true;

  // variables needed to test
  size_t base_map_call_count = 0;
  std::atomic<uint32_t> active_call_count(0);
  std::atomic<std::size_t> last_active_gf(0);
  bool testing_forward_direction = false;
  bool testing_reverse_direction = false;

  // Get and convert map to binary message
  auto map = carma_wm::getBroadcasterTestMap();
  
  ASSERT_EQ(map->regulatoryElementLayer.size(), 0);
  // add regems
  lanelet::PassingControlLinePtr old_pcl = std::make_shared<lanelet::PassingControlLine>(lanelet::PassingControlLine::buildData(10082, {map->laneletLayer.get(10000).leftBound()}, {},
                                                     { lanelet::Participants::VehicleCar }));
  ASSERT_TRUE(old_pcl->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::PassingControlLine::RuleName) == 0);
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 0);
  map->update(map->laneletLayer.get(10000), old_pcl); // added a passing control line
  map->update(map->laneletLayer.get(10007), old_pcl);
  ASSERT_EQ(map->laneletLayer.find(10000)->regulatoryElements().front()->id(), old_pcl->id());
  ASSERT_EQ(map->laneletLayer.find(10007)->regulatoryElements().front()->id(), old_pcl->id());
  ASSERT_FALSE(old_pcl->passableFromLeft(lanelet::Participants::VehicleCar));
  ASSERT_TRUE(old_pcl->passableFromRight(lanelet::Participants::VehicleCar));

  auto timer = std::make_shared<TestTimerFactory>();
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        base_map_call_count++;
      },
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {
        auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
        carma_wm::fromBinMsg(geofence_bin, data_received);

        ASSERT_EQ(data_received->id_, curr_id);

        // gather the new control lines from update list to test
        std::vector<lanelet::PassingControlLinePtr> control_lines;
        for (auto pair : data_received->update_list_)
        {
          auto factory_pcl = lanelet::RegulatoryElementFactory::create(pair.second->attribute(lanelet::AttributeName::Subtype).value(),
                                                            std::const_pointer_cast<lanelet::RegulatoryElementData>(pair.second->constData()));
          lanelet::PassingControlLinePtr control_line = std::dynamic_pointer_cast<lanelet::PassingControlLine>(factory_pcl);
          control_lines.push_back(control_line);
        }
        
        if (testing_forward_direction)
        {
          ASSERT_FALSE(lanelet::PassingControlLine::boundPassable(map->laneletLayer.get(10000).leftBound(), control_lines,
                                                 true, lanelet::Participants::VehicleCar));
          ASSERT_TRUE(lanelet::PassingControlLine::boundPassable(map->laneletLayer.get(10000).leftBound(), control_lines,
                                                      false, lanelet::Participants::VehicleCar));
        }
        if (testing_reverse_direction)
        {
          ASSERT_FALSE(lanelet::PassingControlLine::boundPassable(map->laneletLayer.get(10000).leftBound(), control_lines, true,
                                                      lanelet::Participants::VehicleCar));
          ASSERT_TRUE(lanelet::PassingControlLine::boundPassable(map->laneletLayer.get(10000).leftBound(), control_lines, false,
                                                      lanelet::Participants::VehicleCar));
        }
       
        active_call_count.store(active_call_count.load() + 1);
        // atomic is not working for boost::uuids::uuid, so hash it
        last_active_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
      }, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      timer, [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

 autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));
  // Set the map
  wmb.baseMapCallback(std::move(map_msg_ptr));
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::msg::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(sample_proj_string));

  // set the accessibility
  msg_v01.params_exists = true;
  msg_v01.geometry.datum = proj_string;
  j2735_v2x_msgs::msg::TrafficControlVehClass veh_class;
  veh_class.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::PASSENGER_CAR;
  msg_v01.params.vclasses.push_back(veh_class);
  msg_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::LATAFFINITY_CHOICE;
  msg_v01.params.detail.lataffinity = carma_v2x_msgs::msg::TrafficControlDetail::LEFT; // applies to the left boundaries of the 
  msg_v01.params.detail.latperm[0] = carma_v2x_msgs::msg::TrafficControlDetail::NONE; // not accessible from left
  msg_v01.params.detail.latperm[1] = carma_v2x_msgs::msg::TrafficControlDetail::PERMITTED; // accessible from right

  // create the control message's relevant parts to fill the object
  msg_v01.geometry.proj = proj_string;
  // set the points
  carma_v2x_msgs::msg::PathNode pt;
  // check points that are inside lanelets, thauto gf_ptr = std::make_shared<Geofence>();ese correspond to id 10000, 10007
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;  
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 1.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);

  // register the geofence
  carma_v2x_msgs::msg::TrafficControlMessage gf_msg;
  gf_msg.tcm_v01.geometry.datum = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  gf_msg.choice = carma_v2x_msgs::msg::TrafficControlMessage::TCMV01;

  // every control message needs associated control request id
  carma_planning_msgs::msg::Route route_msg;
  route_msg.route_path_lanelet_ids.push_back(10000);
  std::shared_ptr<j2735_v2x_msgs::msg::Id64b> req_id = std::make_shared<j2735_v2x_msgs::msg::Id64b>(j2735_v2x_msgs::msg::Id64b());
  wmb.controlRequestFromRoute(route_msg, req_id);
  msg_v01.reqid = *req_id;

  gf_msg.tcm_v01 = msg_v01;
  testing_forward_direction = true;

  timer->setNow(rclcpp::Time(2.1e9));  // Set current time
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg));
  timer->setNow(rclcpp::Time(100.0e9));  // Set current time

  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(1, active_call_count.load());

  testing_forward_direction = false;
  testing_reverse_direction = true;

  // update id to continue testing
  curr_id = boost::uuids::random_generator()(); 
  curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), msg_v01.id.id.begin());
  msg_v01.geometry.nodes = {};
  pt.x = 0.5; pt.y = 0.75; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  gf_msg.tcm_v01 = msg_v01;

  timer->setNow(rclcpp::Time(2.1e9));  // Set current time
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg));
  timer->setNow(rclcpp::Time(100.0e9));  // Set current time
  
  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(2, active_call_count.load());
}

TEST(WMBroadcaster, geofenceFromMsgTest)
{
  using namespace lanelet::units::literals;
  // Start creating ROS msg
  carma_v2x_msgs::msg::TrafficControlMessageV01 msg_v01;
  boost::uuids::uuid curr_id = boost::uuids::random_generator()(); 
  std::copy(curr_id.begin(),  curr_id.end(), msg_v01.id.id.begin());
  msg_v01.params.schedule.start = rclcpp::Time(1);  // Schedule between 1 ...
  msg_v01.params.schedule.end = rclcpp::Time(8);    // and 8
  carma_v2x_msgs::msg::DailySchedule daily_schedule;
  daily_schedule.begin = rclcpp::Duration(2);       // Starts at 2
  daily_schedule.duration = rclcpp::Duration(1.1);  // Ends at by 3.1
  msg_v01.params.schedule.between.push_back(daily_schedule);
  msg_v01.params.schedule.repeat.offset =  rclcpp::Duration(0);  // 0 offset for repetition start, so still starts at 2
  msg_v01.params.schedule.repeat.span = rclcpp::Duration(1);     // Duration of 1 and interval of two so active durations are (2-3)
  msg_v01.params.schedule.repeat.period =  rclcpp::Duration(2);
  msg_v01.params.schedule.end_exists = true;
  msg_v01.params.schedule.between_exists = true;
  msg_v01.params.schedule.repeat_exists = true;

  // Get map and convert map to binary message
  auto map = carma_wm::getBroadcasterTestMap();

  // Set a basic environment  
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        base_map_call_count++;
      },
      [](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {}, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));
  // Set the map
  wmb.baseMapCallback(std::move(map_msg_ptr));
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::msg::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(sample_proj_string));

  // create rest of control message's relevant parts to fill the object
  msg_v01.geometry.proj = proj_string;
  msg_v01.geometry.datum = proj_string;
  // set the points
  carma_v2x_msgs::msg::PathNode pt;
  // check points that are inside lanelets, these correspond to id 10000, 10007
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;  
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 1.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  msg_v01.params_exists = true;

  // Create geofence pointer that will be used throughout this test case
  auto gf_ptr = std::make_shared<Geofence>();

  // test restricted lane (trucks and buses will not have access)
  msg_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::RESTRICTED_CHOICE;
  j2735_v2x_msgs::msg::TrafficControlVehClass restricted_veh_class1;
  j2735_v2x_msgs::msg::TrafficControlVehClass restricted_veh_class2;
  restricted_veh_class1.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::TWO_AXLE_SIX_TIRE_SINGLE_UNIT_TRUCK;
  restricted_veh_class2.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::BUS;
  msg_v01.params.vclasses.push_back(restricted_veh_class1);
  msg_v01.params.vclasses.push_back(restricted_veh_class2);
  wmb.geofenceFromMsg(gf_ptr, msg_v01);

  lanelet::RegionAccessRulePtr region_access_rule = std::dynamic_pointer_cast<lanelet::RegionAccessRule>(gf_ptr->regulatory_element_);
  ASSERT_EQ(region_access_rule->accessable(lanelet::Participants::VehicleBus), false);   // Bus does not have access
  ASSERT_EQ(region_access_rule->accessable(lanelet::Participants::VehicleTruck), false); // Truck does not have access
  ASSERT_EQ(region_access_rule->accessable(lanelet::Participants::VehicleCar), true);    // Car has access
  msg_v01.params.vclasses = {}; // Clear the set vclasses for msg_v01
  
  // test maxspeed - config limit inactive
  msg_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE;
  msg_v01.params.detail.maxspeed = 99;

  lanelet::Velocity limit = 80_mph;

  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  ASSERT_TRUE(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  lanelet::DigitalSpeedLimitPtr max_speed = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(gf_ptr->regulatory_element_);

  ASSERT_NEAR(max_speed->speed_limit_.value(), limit.value(),0.0001) ;//Check that the maximum speed limit is not larger than 80_mph
  ASSERT_GE(max_speed->speed_limit_, 0_mph);//Check that the maximum speed limit is not smaller than 0_mph
  RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Maximum speed limit is valid (1).");

  msg_v01.params.detail.maxspeed = -4;

  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  ASSERT_TRUE(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  max_speed = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(gf_ptr->regulatory_element_);
  ASSERT_GE(max_speed->speed_limit_, 0_mph);//Check that the maximum speed limit is not smaller than 0_mph
  RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Maximum speed limit is valid(2).");


  // test minspeed - config limit inactive
  msg_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::MINSPEED_CHOICE;
  msg_v01.params.detail.minspeed = -4.0;


  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  ASSERT_TRUE(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  lanelet::DigitalSpeedLimitPtr min_speed = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(gf_ptr->regulatory_element_);
  ASSERT_GE(min_speed->speed_limit_, 0_mph);
  RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Minimum speed limit is valid.(1)");


   msg_v01.params.detail.minspeed = 99.0;


  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  ASSERT_TRUE(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  min_speed = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(gf_ptr->regulatory_element_);
  ASSERT_NEAR(min_speed->speed_limit_.value(), limit.value(), 0.0001) ;//Check that the minimum speed limit is not larger than 80_mph
  ASSERT_GE(min_speed->speed_limit_, 0_mph);
  RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Minimum speed limit is valid.(2)");

  

// test maxspeed - config limit active
  wmb.setConfigSpeedLimit(55.0);//Set the config speed limit
  msg_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE;
  msg_v01.params.detail.maxspeed = 100.0;
  auto gf_ptr2 = std::make_shared<Geofence>();
  wmb.geofenceFromMsg(gf_ptr2, msg_v01);
  ASSERT_TRUE(gf_ptr2->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  lanelet::DigitalSpeedLimitPtr max_speed_cL = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(gf_ptr2->regulatory_element_);
  //ASSERT_NEAR(max_speed->speed_limit_.value(), 22.352, 0.00001);
  ASSERT_LE(max_speed_cL->speed_limit_, 80_mph);//Check that the maximum speed limit is not larger than 80_mph
  ASSERT_EQ(max_speed_cL->speed_limit_, 55_mph);//Check that the maximum speed limit is equal to the configured limit
  RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Maximum speed limit (config_limit enabled) is valid.");

   // test minspeed - config limit active
  wmb.setConfigSpeedLimit(55.0);//Set the config speed limit
  msg_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::MINSPEED_CHOICE;
  msg_v01.params.detail.minspeed = 0;
  wmb.geofenceFromMsg(gf_ptr2,msg_v01);
  ASSERT_TRUE(gf_ptr2->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  lanelet::DigitalSpeedLimitPtr min_speed_cL = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(gf_ptr2->regulatory_element_);
 // ASSERT_NEAR(min_speed->speed_limit_.value(), 22.352,  0.00001);
  ASSERT_LE(min_speed_cL->speed_limit_, 80_mph);//Check that the maximum speed limit is not larger than 80_mph
  ASSERT_GE(min_speed_cL->speed_limit_, 0_mph);
  ASSERT_EQ(min_speed_cL->speed_limit_, 55_mph);
  RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Minimum speed limit (config_limit enabled) is valid.");




  // TEST passing control line
  // Test lataffinity
  msg_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::LATAFFINITY_CHOICE;
  msg_v01.params.detail.lataffinity = carma_v2x_msgs::msg::TrafficControlDetail::LEFT; // applies to the left boundaries of the 
  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  ASSERT_TRUE(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::PassingControlLine::RuleName) == 0);
  ASSERT_TRUE(gf_ptr->pcl_affects_left_);

  msg_v01.params.detail.lataffinity = carma_v2x_msgs::msg::TrafficControlDetail::RIGHT; // applies to the right boundaries of the 
  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  ASSERT_TRUE(gf_ptr->pcl_affects_right_);

  msg_v01.params.detail.latperm[0] = carma_v2x_msgs::msg::TrafficControlDetail::NONE; // not accessible from left
  msg_v01.params.detail.latperm[1] = carma_v2x_msgs::msg::TrafficControlDetail::PERMITTED; // accessible from right

  // Test Participants
  j2735_v2x_msgs::msg::TrafficControlVehClass veh_class;
  veh_class.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::ANY;
  msg_v01.params.vclasses.push_back(veh_class);
  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  lanelet::PassingControlLinePtr pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_EQ(pcl->right_participants_.size(), 3);

  msg_v01.params.vclasses = {};
  veh_class.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::PEDESTRIAN;
  msg_v01.params.vclasses.push_back(veh_class);
  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::Pedestrian) == 0);
  
  msg_v01.params.vclasses = {};
  veh_class.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::BICYCLE;
  msg_v01.params.vclasses.push_back(veh_class);
  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::Bicycle) == 0);
  
  msg_v01.params.vclasses = {};
  veh_class.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::MOTORCYCLE;
  msg_v01.params.vclasses.push_back(veh_class);
  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::VehicleMotorcycle) == 0);
  
  msg_v01.params.vclasses = {};
  veh_class.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::BUS;
  msg_v01.params.vclasses.push_back(veh_class);
  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::VehicleBus) == 0);

  msg_v01.params.vclasses = {};
  veh_class.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::LIGHT_TRUCK_VAN;
  msg_v01.params.vclasses.push_back(veh_class);
  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::VehicleCar) == 0);

  msg_v01.params.vclasses = {};
  veh_class.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::THREE_AXLE_SINGLE_UNIT_TRUCK;
  msg_v01.params.vclasses.push_back(veh_class);
  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::VehicleTruck) == 0);
  ASSERT_EQ(pcl->left_participants_.size(), 0);
  
  msg_v01.params.detail.latperm[0] = carma_v2x_msgs::msg::TrafficControlDetail::PERMITTED; // accessible from left
  msg_v01.params.detail.latperm[1] = carma_v2x_msgs::msg::TrafficControlDetail::NONE; // not accessible from right
  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->left_participants_.begin()->data(), lanelet::Participants::VehicleTruck) == 0);
  ASSERT_EQ(pcl->right_participants_.size(), 0);

  msg_v01.params.detail.latperm[0] = carma_v2x_msgs::msg::TrafficControlDetail::EMERGENCYONLY; 
  msg_v01.params.detail.latperm[1] = carma_v2x_msgs::msg::TrafficControlDetail::EMERGENCYONLY; 
  wmb.geofenceFromMsg(gf_ptr, msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::VehicleEmergency) == 0);
  ASSERT_TRUE(strcmp(pcl->left_participants_.begin()->data(), lanelet::Participants::VehicleEmergency) == 0);
  
}

TEST(WMBroadcaster, distToNearestActiveGeofence)
{
   // Test adding then evaluate if the calls to active and inactive are done correctly
  auto gf = std::make_shared<Geofence>();

  boost::uuids::uuid curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  gf->id_ = curr_id;

  gf->schedules.push_back(carma_wm_ctrl::GeofenceSchedule(rclcpp::Time(1e9),  // Schedule between 1 and 8
                                 rclcpp::Time(8e9),
                                 rclcpp::Duration(2e9),    // Starts at 2
                                 rclcpp::Duration(1.1e9),  // Ends at by 3.1
                                 rclcpp::Duration(0),    // 0 offset for repetition start, so still starts at 2
                                 rclcpp::Duration(1e9),    // Duration of 1 and interval of two so active durations are (2-3)
                                 rclcpp::Duration(2e9)));
  // convert to ros msg
  carma_v2x_msgs::msg::TrafficControlMessageV01 msg_v01;
  std::copy(gf->id_.begin(),  gf->id_.end(), msg_v01.id.id.begin());
  msg_v01.params.schedule.start = gf->schedules[0].schedule_start_;
  msg_v01.params.schedule.end = gf->schedules[0].schedule_end_;
  carma_v2x_msgs::msg::DailySchedule daily_schedule;
  daily_schedule.begin = gf->schedules[0].control_start_;
  daily_schedule.duration = gf->schedules[0].control_duration_;
  msg_v01.params.schedule.between.push_back(daily_schedule);
  msg_v01.params.schedule.repeat.offset =  gf->schedules[0].control_offset_;
  msg_v01.params.schedule.repeat.span =  gf->schedules[0].control_span_;
  msg_v01.params.schedule.repeat.period =  gf->schedules[0].control_period_;
  msg_v01.params.schedule.end_exists = true;
  msg_v01.params.schedule.between_exists = true;
  msg_v01.params.schedule.repeat_exists = true;

  // variables needed to test
  size_t base_map_call_count = 0;
  std::atomic<uint32_t> map_update_call_count(0);
  std::atomic<std::size_t> last_active_gf(0);
  std::atomic<std::size_t> last_inactive_gf(0);
  bool activated = false;

  auto timer = std::make_shared<TestTimerFactory>();

  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
      },
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {
        auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
        carma_wm::fromBinMsg(geofence_bin, data_received);
        map_update_call_count.store(map_update_call_count.load() + 1);
        // atomic is not working for boost::uuids::uuid, so hash it
        if (activated)
          last_active_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
        if (!activated)
          last_inactive_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
      }, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      timer, [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  // Get and convert map to binary message
  auto map = carma_wm::getDisjointRouteMap();
  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(std::move(map_msg_ptr));

  // Setting georeferences
  // geofence's origin (0,0) is at base_map's (10,10)
  std::string base_map_proj_string, geofence_proj_string;
  std_msgs::msg::String base_map_proj;
  base_map_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  geofence_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  base_map_proj.data = base_map_proj_string;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(base_map_proj));
  
  carma_v2x_msgs::msg::TrafficControlMessage gf_msg;
  gf_msg.choice = carma_v2x_msgs::msg::TrafficControlMessage::TCMV01;
  // create the geofence request
  msg_v01.geometry.proj = geofence_proj_string;
  msg_v01.geometry.datum = geofence_proj_string;


  // every control message needs associated control request id
  carma_planning_msgs::msg::Route route_msg;
  route_msg.route_path_lanelet_ids.push_back(10000);
  route_msg.route_path_lanelet_ids.push_back(10001);
  route_msg.route_path_lanelet_ids.push_back(10002);
  route_msg.route_path_lanelet_ids.push_back(10003);
  std::shared_ptr<j2735_v2x_msgs::msg::Id64b> req_id = std::make_shared<j2735_v2x_msgs::msg::Id64b>(j2735_v2x_msgs::msg::Id64b());
  wmb.controlRequestFromRoute(route_msg, req_id);
  msg_v01.reqid = *req_id;

  // set the points
  carma_v2x_msgs::msg::PathNode pt;
  // check points that are inside lanelets
  pt.x = 1.5; pt.y = 1.5; pt.z = 0; // straight geofence line across 2 lanelets
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 1.5; pt.y = 2.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  // update id to continue testing
  curr_id = boost::uuids::random_generator()(); 
  curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), msg_v01.id.id.begin());

  msg_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE;
  msg_v01.params.detail.maxspeed = 50;
  gf_msg.tcm_v01 = msg_v01;

  // Make sure the geofence is active now
  activated = true;
  timer->setNow(rclcpp::Time(2.1e9));  // Set current time so that geofence is active
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg));
  
  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(1, map_update_call_count.load());

  lanelet::BasicPoint2d curr_pos = {1.5,0.5};
  double nearest_gf_dist = wmb.distToNearestActiveGeofence(curr_pos);
  ASSERT_NEAR(nearest_gf_dist, 0.5, 0.0001);

  curr_pos = {0.5,0.5};
  nearest_gf_dist = wmb.distToNearestActiveGeofence(curr_pos);
  ASSERT_NEAR(nearest_gf_dist, 1.5, 0.0001);

  curr_pos = {1.5,1.5};  // it is currently on an active geofence
  nearest_gf_dist = wmb.distToNearestActiveGeofence(curr_pos);
  ASSERT_NEAR(nearest_gf_dist, 0.0, 0.0001);  // it should point the next

  curr_pos = {1.5,3.5};  // it is currently not on any lanelet
  EXPECT_THROW(wmb.distToNearestActiveGeofence(curr_pos), std::invalid_argument);

  activated = false;
  timer->setNow(rclcpp::Time(3.2e9));  // Geofences deactivate now
  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_inactive_gf));
  ASSERT_EQ(2, map_update_call_count.load());

  curr_pos = {1.5,1.5};  // it is currently on an active geofence
  nearest_gf_dist = wmb.distToNearestActiveGeofence(curr_pos);
  ASSERT_NEAR(nearest_gf_dist, 0, 0.0001);  // it should point the next
}

TEST(WMBroadcaster, addRegionAccessRule)
{
  auto gf_ptr = std::make_shared<Geofence>();
  auto map = carma_wm::getBroadcasterTestMap();

  std::vector<lanelet::Lanelet> affected_llts {map->laneletLayer.get(map->laneletLayer.begin()->id())};

  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {},
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {},
      [&](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [&](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  carma_v2x_msgs::msg::TrafficControlMessageV01 msg_v01;
  carma_v2x_msgs::msg::TrafficControlMessageV01 msg_v02;
  j2735_v2x_msgs::msg::TrafficControlVehClass participant1,participant2;
  participant1.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::PASSENGER_CAR;
  msg_v01.params.vclasses.push_back(participant1);

  wmb.addRegionAccessRule(gf_ptr,msg_v01,affected_llts);

  ASSERT_EQ(gf_ptr->invalidate_route_,true);

  participant2.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::PEDESTRIAN;
  msg_v02.params.vclasses = {};
  msg_v02.params.vclasses.push_back(participant2);
  msg_v02.package.label = "Move over law";
  gf_ptr = std::make_shared<Geofence>();
  wmb.addRegionAccessRule(gf_ptr,msg_v02,affected_llts);

  ASSERT_EQ(gf_ptr->invalidate_route_,false);

  ASSERT_TRUE(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::RegionAccessRule::RuleName) == 0);
  lanelet::RegionAccessRulePtr region_cess_reg = std::dynamic_pointer_cast<lanelet::RegionAccessRule>(gf_ptr->regulatory_element_);
  ASSERT_EQ(region_cess_reg->getReason(),"Move over law") ;
}


TEST(WMBroadcaster, addRegionMinimumGap)
{
  auto gf_ptr = std::make_shared<Geofence>();
  auto map = carma_wm::getBroadcasterTestMap();

  std::vector<lanelet::Lanelet> affected_llts {map->laneletLayer.get(map->laneletLayer.begin()->id())};

  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {},
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {},
      [&](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [&](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  carma_v2x_msgs::msg::TrafficControlMessageV01 msg_v01;
  carma_v2x_msgs::msg::TrafficControlMessageV01 msg_v02;
  j2735_v2x_msgs::msg::TrafficControlVehClass participant1,participant2;
  participant1.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::PASSENGER_CAR;
  msg_v01.params.vclasses.push_back(participant1);
  double min_gap =  12;
  wmb.addRegionMinimumGap(gf_ptr, msg_v01,min_gap,affected_llts, {});

  ASSERT_TRUE(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalMinimumGap::RuleName) == 0);
  lanelet::DigitalMinimumGapPtr min_gap_reg = std::dynamic_pointer_cast<lanelet::DigitalMinimumGap>(gf_ptr->regulatory_element_);
  ASSERT_NEAR(min_gap_reg->getMinimumGap(),min_gap, 0.0001) ;
  auto result = wmb.participantsChecker(msg_v01);
  ASSERT_EQ(result.size(), 1);
}

TEST(WMBroadcaster, invertParticipants)
{
  auto gf_ptr = std::make_shared<Geofence>();
  auto map = carma_wm::getBroadcasterTestMap();

  std::vector<lanelet::Lanelet> affected_llts {map->laneletLayer.get(map->laneletLayer.begin()->id())};

  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {},
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {},
      [&](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [&](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});
  
  std::vector<std::string> participants;
  auto result = wmb.invertParticipants(participants);
  ASSERT_EQ(result.size(), 6);
}

TEST(WMBroadcaster, currentLocationCallback)
{

   // Test adding then evaluate if the calls to active and inactive are done correctly
  auto gf = std::make_shared<Geofence>();

  boost::uuids::uuid curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  gf->id_ = curr_id;

  gf->schedules.push_back(carma_wm_ctrl::GeofenceSchedule(rclcpp::Time(1e9),  // Schedule between 1 and 8
                                 rclcpp::Time(8e9),
                                 rclcpp::Duration(2e9),    // Starts at 2
                                 rclcpp::Duration(1.1e9),  // Ends at by 3.1
                                 rclcpp::Duration(0),    // 0 offset for repetition start, so still starts at 2
                                 rclcpp::Duration(1e9),    // Duration of 1 and interval of two so active durations are (2-3)
                                 rclcpp::Duration(2e9)));
  // convert to ros msg
  carma_v2x_msgs::msg::TrafficControlMessageV01 msg_v01;
  std::copy(gf->id_.begin(),  gf->id_.end(), msg_v01.id.id.begin());

  msg_v01.params.schedule.start = gf->schedules[0].schedule_start_;
  msg_v01.params.schedule.end = gf->schedules[0].schedule_end_;
  carma_v2x_msgs::msg::DailySchedule daily_schedule;
  daily_schedule.begin = gf->schedules[0].control_start_;
  daily_schedule.duration = gf->schedules[0].control_duration_;
  msg_v01.params.schedule.between.push_back(daily_schedule);
  msg_v01.params.schedule.repeat.offset =  gf->schedules[0].control_offset_;
  msg_v01.params.schedule.repeat.span =  gf->schedules[0].control_span_;
  msg_v01.params.schedule.repeat.period =  gf->schedules[0].control_period_;
  msg_v01.params.schedule.end_exists = true;
  msg_v01.params.schedule.between_exists = true;
  msg_v01.params.schedule.repeat_exists = true;

 // variables needed to test
  std::atomic<uint32_t> map_update_call_count(0);
  std::atomic<std::size_t> last_active_gf(0);
  std::atomic<std::size_t> last_inactive_gf(0);
  bool activated = false;

//Create input message
  geometry_msgs::msg::PoseStamped input_msg;

  //Input message coordinates
  input_msg.pose.position.x = 1.5;
  input_msg.pose.position.y = 1.5;
  input_msg.pose.position.z = 0.0;

 size_t base_map_call_count = 0;

 auto timer = std::make_shared<TestTimerFactory>();

  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
      },
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {
        auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
        carma_wm::fromBinMsg(geofence_bin, data_received);
        map_update_call_count.store(map_update_call_count.load() + 1);
        // atomic is not working for boost::uuids::uuid, so hash it
        if (activated)
          last_active_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
        if (!activated)
          last_inactive_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
      }, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      timer, [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Throw Exceptions Test Passed.");

  // Get and convert map to binary message
  auto map = carma_wm::getDisjointRouteMap();
  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(std::move(map_msg_ptr));

  // Setting georeferences
  // geofence's origin (0,0) is at base_map's (10,10)
  std::string base_map_proj_string, geofence_proj_string;
  std_msgs::msg::String base_map_proj;
  base_map_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  geofence_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  base_map_proj.data = base_map_proj_string;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(base_map_proj));
  
  carma_v2x_msgs::msg::TrafficControlMessage gf_msg;
  gf_msg.choice = carma_v2x_msgs::msg::TrafficControlMessage::TCMV01;
  // create the geofence request
  msg_v01.geometry.proj = geofence_proj_string;
  msg_v01.geometry.datum = geofence_proj_string;

  // every control message needs associated control request id
  carma_planning_msgs::msg::Route route_msg;
  route_msg.route_path_lanelet_ids.push_back(10000);
  route_msg.route_path_lanelet_ids.push_back(10001);
  route_msg.route_path_lanelet_ids.push_back(10002);
  route_msg.route_path_lanelet_ids.push_back(10003);
  std::shared_ptr<j2735_v2x_msgs::msg::Id64b> req_id = std::make_shared<j2735_v2x_msgs::msg::Id64b>(j2735_v2x_msgs::msg::Id64b());
  wmb.controlRequestFromRoute(route_msg, req_id);
  msg_v01.reqid = *req_id;

  // set the points
  carma_v2x_msgs::msg::PathNode pt;
  // check points that are inside lanelets
  pt.x = 1.5; pt.y = 1.5; pt.z = 0; // straight geofence line across 2 lanelets
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 1.5; pt.y = 2.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  // update id to continue testing
  curr_id = boost::uuids::random_generator()(); 
  curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), msg_v01.id.id.begin());

  msg_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE;
  msg_v01.params.detail.maxspeed = 50;
  gf_msg.tcm_v01 = msg_v01;

  // Make sure the geofence is active now
  activated = true;
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg));
  timer->setNow(rclcpp::Time(2.1e9));  // Set current time so that geofence is active
  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(1, map_update_call_count.load());

 // wmb.addGeofence(gf);

  std::unordered_set<lanelet::Id> active_geofence_llt_ids;
 // active_geofence_llt_ids.insert(gf->id_);

  carma_perception_msgs::msg::CheckActiveGeofence check = wmb.checkActiveGeofenceLogic(input_msg);
  ASSERT_GE(check.distance_to_next_geofence, 0);
  EXPECT_TRUE(check.type > 0);
  EXPECT_TRUE(check.is_on_active_geofence);

  activated = false;
  timer->setNow(rclcpp::Time(3.2e9));  // Geofences deactivate now
  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_inactive_gf));
  ASSERT_EQ(2, map_update_call_count.load());
}

TEST(WMBroadcaster, checkActiveGeofenceLogicTest)
{
   // Create geofence pointer
  auto gf = std::make_shared<Geofence>();
  gf->schedules.push_back(carma_wm_ctrl::GeofenceSchedule(rclcpp::Time(1e9),  // Schedule between 1 and 8
                                 rclcpp::Time(8e9),
                                 rclcpp::Duration(2e9),    // Starts at 2
                                 rclcpp::Duration(1.1e9),  // Ends at by 3.1
                                 rclcpp::Duration(0),    // 0 offset for repetition start, so still starts at 2
                                 rclcpp::Duration(1e9),    // Duration of 1 and interval of two so active durations are (2-3)
                                 rclcpp::Duration(2e9)));

  // Convert the geofence pointer into a TrafficControlMessageV01 message
  carma_v2x_msgs::msg::TrafficControlMessageV01 msg_v01;
  msg_v01.params.schedule.start = gf->schedules[0].schedule_start_;
  msg_v01.params.schedule.end = gf->schedules[0].schedule_end_;
  carma_v2x_msgs::msg::DailySchedule daily_schedule;
  daily_schedule.begin = gf->schedules[0].control_start_;
  daily_schedule.duration = gf->schedules[0].control_duration_;
  msg_v01.params.schedule.between.push_back(daily_schedule);
  msg_v01.params.schedule.repeat.offset =  gf->schedules[0].control_offset_;
  msg_v01.params.schedule.repeat.span =  gf->schedules[0].control_span_;
  msg_v01.params.schedule.repeat.period =  gf->schedules[0].control_period_;
  msg_v01.params.schedule.end_exists = true;
  msg_v01.params.schedule.between_exists = true;
  msg_v01.params.schedule.repeat_exists = true;

  // Initialize variables required for this test
  std::atomic<uint32_t> map_update_call_count(0);
  std::atomic<std::size_t> last_active_gf(0);
  bool activated = false;

  // Create WMBroadcaster object
  auto timer = std::make_shared<TestTimerFactory>();

  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
      },
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {
        auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
        carma_wm::fromBinMsg(geofence_bin, data_received);
        map_update_call_count.store(map_update_call_count.load() + 1);
        // atomic is not working for boost::uuids::uuid, so hash it
        if (activated)
          last_active_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
      }, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      timer, [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  // Get and convert map to binary message
  auto map = carma_wm::test::buildGuidanceTestMap(3.7, 25);
  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(std::move(map_msg_ptr));

  // Setting georeferences, otherwise geofenceCallback() will throw exception
  // geofence's origin (0,0) is at base_map's (10,10)
  std::string base_map_proj_string, geofence_proj_string;
  std_msgs::msg::String base_map_proj;
  base_map_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  geofence_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  base_map_proj.data = base_map_proj_string;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(base_map_proj));

  carma_v2x_msgs::msg::TrafficControlMessage gf_msg;
  gf_msg.choice = carma_v2x_msgs::msg::TrafficControlMessage::TCMV01;
  msg_v01.geometry.proj = geofence_proj_string;
  msg_v01.geometry.datum = geofence_proj_string;
  // Obtain a control request ID for the TrafficControlMessage
  carma_planning_msgs::msg::Route route_msg;
  route_msg.route_path_lanelet_ids.push_back(1200);
  route_msg.route_path_lanelet_ids.push_back(1201);
  route_msg.route_path_lanelet_ids.push_back(1202);
  route_msg.route_path_lanelet_ids.push_back(1203);
  std::shared_ptr<j2735_v2x_msgs::msg::Id64b> req_id = std::make_shared<j2735_v2x_msgs::msg::Id64b>(j2735_v2x_msgs::msg::Id64b());
  wmb.controlRequestFromRoute(route_msg, req_id);
  msg_v01.reqid = *req_id;

  // Set geofence 1's TrafficControlMessage points for lanelets 1200 and 1201
  carma_v2x_msgs::msg::PathNode pt;
  pt.x = 1.5; pt.y = 15; pt.z = 0; // Point in lanelet 1200
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 0.0; pt.y = 30; pt.z = 0; // Point in lanelet 1201
  msg_v01.geometry.nodes.push_back(pt);

  // Set an advisory speed limit for geofence 1 (Lanelets 1200 and 1201)
  msg_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE;
  msg_v01.params.detail.maxspeed = 22.352; // 50 mph

  // Set the ID for geofence 1
  boost::uuids::uuid curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed_gf1 = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), msg_v01.id.id.begin());
  gf_msg.tcm_v01 = msg_v01;

  // Create geofence 2 with a prescribed minimum gap (Lanelets 1200 and 1201)
  auto gf_msg2 = gf_msg;
  gf_msg2.tcm_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::MINHDWY_CHOICE;
  gf_msg2.tcm_v01.params.detail.minhdwy = 5;

  // Set the ID for geofence 2
  curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed_gf2 = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), gf_msg2.tcm_v01.id.id.begin());

  // Create geofence 3 with a lane closure (Lanelets 1210 and 1211)
  auto gf_msg3 = gf_msg;
  gf_msg3.tcm_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::CLOSED_CHOICE;
  gf_msg3.tcm_v01.params.detail.closed = carma_v2x_msgs::msg::TrafficControlDetail::CLOSED;
  gf_msg3.tcm_v01.package.label = "MOVE OVER LAW";
  j2735_v2x_msgs::msg::TrafficControlVehClass veh_type;
  veh_type.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::ANY;
  gf_msg3.tcm_v01.params.vclasses.push_back(veh_type);

  // Set geofence 3's TrafficControlMessage points for lanelets 1210 and 1211
  pt.x = 4.5; pt.y = 15; pt.z = 0; // Point in lanelet 1210
  gf_msg3.tcm_v01.geometry.nodes[0] = pt;
  pt.x = 0.0; pt.y = 30; pt.z = 0; // Point in lanelet 1211
  gf_msg3.tcm_v01.geometry.nodes[1] = pt;

  // Set the ID for geofence 3
  curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed_gf3 = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), gf_msg3.tcm_v01.id.id.begin());

  // Create geofence 4 with an advisory speed limit (Lanelets 1220 and 1221)
  auto gf_msg4 = gf_msg;
  gf_msg4.tcm_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE;
  gf_msg4.tcm_v01.params.detail.maxspeed = 22.352; // 50 mph

  // Set geofence 4's TrafficControlMessage points for lanelets 1220 and 1221
  pt.x = 10.0; pt.y = 15; pt.z = 0; // Point in lanelet 1220
  gf_msg4.tcm_v01.geometry.nodes[0] = pt;
  pt.x = 0.0; pt.y = 30; pt.z = 0; // Point in lanelet 1221
  gf_msg4.tcm_v01.geometry.nodes[1] = pt;

  // Set the ID for geofence 4
  curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed_gf4 = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), gf_msg4.tcm_v01.id.id.begin());

  // Create geofence 5 with a prescribed minimum gap (Lanelets 1220 and 1221)
  auto gf_msg5 = gf_msg4;
  gf_msg5.tcm_v01.params.detail.choice = carma_v2x_msgs::msg::TrafficControlDetail::MINHDWY_CHOICE;
  gf_msg5.tcm_v01.params.detail.minhdwy = 5;

  // Set the ID for geofence 5
  curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed_gf5 = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), gf_msg5.tcm_v01.id.id.begin());

  // Make sure the geofences are active now
  timer->setNow(rclcpp::Time(2.1e9));
  activated = true;

  // Set callback for geofence 1 
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg));

  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed_gf1, last_active_gf));
  ASSERT_EQ(1, map_update_call_count.load());

  // Set callback for geofence 2
  timer->setNow(rclcpp::Time(2.1e9));
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg2));

  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed_gf2, last_active_gf));
  ASSERT_EQ(2, map_update_call_count.load());

  // Set callback for geofence 3
  timer->setNow(rclcpp::Time(2.1e9));
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg3));

  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed_gf3, last_active_gf));
  ASSERT_EQ(3, map_update_call_count.load());

  // Set callback for geofence 4
  timer->setNow(rclcpp::Time(2.1e9));
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg4));

  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed_gf4, last_active_gf));
  ASSERT_EQ(4, map_update_call_count.load());

  // Set callback for geofence 5
  timer->setNow(rclcpp::Time(2.1e9));
  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg5));

  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed_gf5, last_active_gf));
  ASSERT_EQ(5, map_update_call_count.load());

  // Create current vehicle pose message for Lanelet 1200
  geometry_msgs::msg::PoseStamped current_vehicle_pose;
  current_vehicle_pose.pose.position.x = 1.5;
  current_vehicle_pose.pose.position.y = 10.5;
  current_vehicle_pose.pose.position.z = 0.0;

  // Check active geofence result for Lanelet 1200
  carma_perception_msgs::msg::CheckActiveGeofence check = wmb.checkActiveGeofenceLogic(current_vehicle_pose);
  ASSERT_GE(check.distance_to_next_geofence, 0);
  EXPECT_TRUE(check.is_on_active_geofence);
  ASSERT_NEAR(check.value, 22.352, 0.0001); // Advisory Speed Limit (50 mph in m/s)
  ASSERT_NEAR(check.advisory_speed, 22.352, 0.0001); // 50 mph in m/s
  ASSERT_EQ(check.minimum_gap, 5);
  ASSERT_EQ(check.type, 2); // Type 2 is "LANE_CLOSED" since adjacent right lane is closed
  ASSERT_EQ(check.reason, "");  // does not have reason for 1220 lanelet

  // Update current vehicle pose message for Lanelet 1210
  current_vehicle_pose.pose.position.x = 4.5;

  // Check active geofence result for Lanelet 1210
  check = wmb.checkActiveGeofenceLogic(current_vehicle_pose);
  ASSERT_GE(check.distance_to_next_geofence, 0);
  EXPECT_TRUE(check.is_on_active_geofence);
  ASSERT_NEAR(check.value, 35.7632, 0.00001); // Advisory Speed Limit (matches original map speed limit)
  ASSERT_NEAR(check.advisory_speed, 35.7632, 0.00001); // Matches original map speed limit
  ASSERT_EQ(check.minimum_gap, 0); // Not populated
  ASSERT_EQ(check.type, 2); // Type 2 is "LANE_CLOSED"
  ASSERT_EQ(check.reason, "MOVE OVER LAW"); // set in gf_msg3

  // Create current vehicle pose message for Lanelet 1220
  current_vehicle_pose.pose.position.x = 10.0;

  // Check active geofence result for Lanelet 1220
  check = wmb.checkActiveGeofenceLogic(current_vehicle_pose);
  ASSERT_GE(check.distance_to_next_geofence, 0);
  EXPECT_TRUE(check.is_on_active_geofence);
  ASSERT_NEAR(check.value, 22.352, 0.0001); // Advisory Speed Limit (50 mph in m/s)
  ASSERT_NEAR(check.advisory_speed, 22.352, 0.0001); // 50 mph in m/s
  ASSERT_EQ(check.minimum_gap, 5);
  ASSERT_EQ(check.type, 2); // Type 2 is "LANE_CLOSED" since adjacent left lane is closed
  ASSERT_EQ(check.reason, ""); // 1220 lanelet was not set reason as copied from gf_msg

  // Create current vehicle pose message for Lanelet 1203 
  current_vehicle_pose.pose.position.x = 1.5;
  current_vehicle_pose.pose.position.y = 85;

  // Check active geofence result for Lanelet 1203 
  check = wmb.checkActiveGeofenceLogic(current_vehicle_pose);
  EXPECT_FALSE(check.is_on_active_geofence);
}

TEST(WMBroadcaster, RegionAccessRuleTest)
{
  // Test adding then evaluate if the calls to active and inactive are done correctly
  auto gf_ptr = std::make_shared<Geofence>();

  boost::uuids::uuid curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  gf_ptr->id_ = curr_id;

  gf_ptr->schedules.push_back(carma_wm_ctrl::GeofenceSchedule(rclcpp::Time(1e9),  // Schedule between 1 and 8
                                 rclcpp::Time(8e9),
                                 rclcpp::Duration(2e9),    // Starts at 2
                                 rclcpp::Duration(1.1e9),  // Ends at by 3.1
                                 rclcpp::Duration(0),    // 0 offset for repetition start, so still starts at 2
                                 rclcpp::Duration(1e9),    // Duration of 1 and interval of two so active durations are (2-3)
                                 rclcpp::Duration(2e9)));
  // convert to ros msg
  carma_v2x_msgs::msg::TrafficControlMessageV01 msg_v01;
  std::copy(gf_ptr->id_.begin(),  gf_ptr->id_.end(), msg_v01.id.id.begin());
  msg_v01.params.schedule.start = gf_ptr->schedules[0].schedule_start_;
  msg_v01.params.schedule.end = gf_ptr->schedules[0].schedule_end_;
  carma_v2x_msgs::msg::DailySchedule daily_schedule;
  daily_schedule.begin = gf_ptr->schedules[0].control_start_;
  daily_schedule.duration = gf_ptr->schedules[0].control_duration_;
  msg_v01.params.schedule.between.push_back(daily_schedule);
  msg_v01.params.schedule.repeat.offset =  gf_ptr->schedules[0].control_offset_;
  msg_v01.params.schedule.repeat.span =  gf_ptr->schedules[0].control_span_;
  msg_v01.params.schedule.repeat.period =  gf_ptr->schedules[0].control_period_;
  msg_v01.params.schedule.end_exists = true;
  msg_v01.params.schedule.between_exists = true;
  msg_v01.params.schedule.repeat_exists = true;

  // variables needed to test
  size_t base_map_call_count = 0;
  std::atomic<uint32_t> active_call_count(0);
  std::atomic<std::size_t> last_active_gf(0);
  bool testing_forward_direction = false;
  bool testing_reverse_direction = false;

  // Get and convert map to binary message
  auto map = carma_wm::getBroadcasterTestMap();
  
  ASSERT_EQ(map->regulatoryElementLayer.size(), 0);

  const std::string& reason = "Move over law";
  // add regems
  lanelet::RegionAccessRulePtr old_reg = std::make_shared<lanelet::RegionAccessRule>(lanelet::RegionAccessRule::buildData(10082, {map->laneletLayer.get(10000)},{},
                                                     { lanelet::Participants::VehicleCar }, reason));
  ASSERT_TRUE(old_reg->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::RegionAccessRule::RuleName) == 0);
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 0);
  map->update(map->laneletLayer.get(10000), old_reg); // added a passing control line
  map->update(map->laneletLayer.get(10007), old_reg);
  ASSERT_EQ(map->laneletLayer.find(10000)->regulatoryElements().front()->id(), old_reg->id());
  ASSERT_EQ(map->laneletLayer.find(10007)->regulatoryElements().front()->id(), old_reg->id());
  ASSERT_FALSE(old_reg->accessable(lanelet::Participants::VehicleBus));
  ASSERT_TRUE(old_reg->accessable(lanelet::Participants::VehicleCar));
  lanelet::RegionAccessRulePtr accessRuleReg =  std::dynamic_pointer_cast<lanelet::RegionAccessRule>
                    (map->regulatoryElementLayer.get(map->laneletLayer.find(10000)->regulatoryElements().front()->id()));
  ASSERT_EQ(accessRuleReg->getReason(),"Move over law");

  lanelet::RegionAccessRulePtr accessRuleReg2 =  std::dynamic_pointer_cast<lanelet::RegionAccessRule>
  (map->regulatoryElementLayer.get(map->laneletLayer.find(10007)->regulatoryElements().front()->id()));
  ASSERT_EQ(accessRuleReg->getReason(),"Move over law");

  auto timer = std::make_shared<TestTimerFactory>();

  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        base_map_call_count++;
      },
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {
        auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
        carma_wm::fromBinMsg(geofence_bin, data_received);

        ASSERT_EQ(data_received->id_, curr_id);

        // gather the new control lines from update list to test
        for (auto pair : data_received->update_list_)
        {
          auto factory_pcl = lanelet::RegulatoryElementFactory::create(pair.second->attribute(lanelet::AttributeName::Subtype).value(),
                                                            std::const_pointer_cast<lanelet::RegulatoryElementData>(pair.second->constData()));
          lanelet::RegionAccessRulePtr region_acc = std::dynamic_pointer_cast<lanelet::RegionAccessRule>(factory_pcl);
          ASSERT_TRUE(region_acc->accessable(lanelet::Participants::VehicleBus));
          ASSERT_FALSE(region_acc->accessable(lanelet::Participants::VehicleCar));
        }
             
        active_call_count.store(active_call_count.load() + 1);
        // atomic is not working for boost::uuids::uuid, so hash it
        last_active_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
      }, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      timer, [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));
  // Set the map
  wmb.baseMapCallback(std::move(map_msg_ptr));
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::msg::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(sample_proj_string));

  // set the accessibility
  msg_v01.geometry_exists=true;
  msg_v01.params_exists=true;
  j2735_v2x_msgs::msg::TrafficControlVehClass veh_type;
  veh_type.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::PASSENGER_CAR; 
  msg_v01.params.vclasses.push_back(veh_type);
  msg_v01.params.detail.choice=carma_v2x_msgs::msg::TrafficControlDetail::CLOSED_CHOICE;
  msg_v01.params.detail.closed=carma_v2x_msgs::msg::TrafficControlDetail::CLOSED;
  msg_v01.params.detail.minhdwy=5;

  // create the control message's relevant parts to fill the object
  msg_v01.geometry.proj = proj_string;
  msg_v01.geometry.datum = proj_string;

  // set the points
  carma_v2x_msgs::msg::PathNode pt;
  // check points that are inside lanelets, thauto gf_ptr = std::make_shared<Geofence>();ese correspond to id 10000, 10007
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;  
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 1.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);

  // register the geofence
  carma_v2x_msgs::msg::TrafficControlMessage gf_msg;
  gf_msg.choice = carma_v2x_msgs::msg::TrafficControlMessage::TCMV01;

  // every control message needs associated control request id
  carma_planning_msgs::msg::Route route_msg;
  route_msg.route_path_lanelet_ids.push_back(10000);
  std::shared_ptr<j2735_v2x_msgs::msg::Id64b> req_id = std::make_shared<j2735_v2x_msgs::msg::Id64b>(j2735_v2x_msgs::msg::Id64b());
  wmb.controlRequestFromRoute(route_msg, req_id);
  msg_v01.reqid = *req_id;

  gf_msg.tcm_v01 = msg_v01;
  testing_forward_direction = true;

  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg));

  timer->setNow(rclcpp::Time(2.1e9));  // Set current time
  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(1, active_call_count.load());

  testing_forward_direction = false;
  testing_reverse_direction = true;

  // update id to continue testing
  curr_id = boost::uuids::random_generator()(); 
  curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), msg_v01.id.id.begin());
  msg_v01.geometry.nodes = {};
  pt.x = 0.5; pt.y = 0.75; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  gf_msg.tcm_v01 = msg_v01;

  wmb.geofenceCallback(std::make_unique<carma_v2x_msgs::msg::TrafficControlMessage>(gf_msg));
  timer->setNow(rclcpp::Time(2.1e9));  // Set current time
  
  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(2, active_call_count.load());
}

TEST(WMBroadcaster, generate32BitId)
{
  
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {},
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {}, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  std::string label = "TYPE:SIG_WZ,INT_ID:0001,SG_ID:001";
  auto bits = wmb.generate32BitId(label);
  EXPECT_EQ(bits, 257);
}

TEST(WMBroadcaster, splitLaneletWithRatio)
{
  // Create WMBroadcaster object
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {},
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {
      }, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  // Get and convert map to binary message with 26 points of 1 meter in-between distances
  auto map = carma_wm::test::buildGuidanceTestMap(5, 25, 25);
  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));
  RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Error messages below are expected...");
  // Trigger basemap callback
  wmb.baseMapCallback(std::move(map_msg_ptr));
  auto first_lanelet = map->laneletLayer.get(1200);
  EXPECT_THROW(wmb.splitLaneletWithRatio({}, first_lanelet, 0.5), lanelet::InvalidInputError);

  // check front ratio TOO CLOSE (0.5 meter error for 25 meter lanelet)
  auto copy_lanelet = wmb.splitLaneletWithRatio({0.99}, first_lanelet, 0.5);
  EXPECT_EQ(copy_lanelet.size(), 1);
  EXPECT_NE(copy_lanelet.front().id(), first_lanelet.id());

  // multiple front TOO CLOSE ratios
  copy_lanelet = wmb.splitLaneletWithRatio({0.001, 0.002, 0.003, 0.004, 0.005}, first_lanelet, 0.5);
  EXPECT_EQ(copy_lanelet.size(), 1);
  EXPECT_NE(copy_lanelet.front().id(), first_lanelet.id());

  copy_lanelet = wmb.splitLaneletWithRatio({0.01}, first_lanelet, 0.5);
  EXPECT_EQ(copy_lanelet.size(), 1);
  EXPECT_NE(copy_lanelet.front().id(), first_lanelet.id());

  // check one valid ratio
  copy_lanelet = wmb.splitLaneletWithRatio({0.6}, first_lanelet, 0.5);
  EXPECT_EQ(copy_lanelet.size(), 2);
  EXPECT_NE(copy_lanelet.front().id(), first_lanelet.id());
  EXPECT_NE(copy_lanelet.back().id(), first_lanelet.id());
  
  // check connection of two new lanelets using ids
  EXPECT_EQ(copy_lanelet.front().leftBound2d().front().id(), first_lanelet.leftBound2d().front().id());
  EXPECT_EQ(copy_lanelet.front().rightBound2d().front().id(), first_lanelet.rightBound2d().front().id());
  EXPECT_EQ(copy_lanelet.back().leftBound2d().back().id(), first_lanelet.leftBound2d().back().id());
  EXPECT_EQ(copy_lanelet.back().rightBound2d().back().id(), first_lanelet.rightBound2d().back().id());
  EXPECT_EQ(copy_lanelet.front().leftBound2d().back().id(), copy_lanelet.back().leftBound2d().front().id());
  EXPECT_EQ(copy_lanelet.front().rightBound2d().back().id(), copy_lanelet.back().rightBound2d().front().id());
  
  // check if matches previous original lanelet using geometry
  EXPECT_NEAR(lanelet::geometry::distance2d(copy_lanelet.front().leftBound2d().front().basicPoint2d(), first_lanelet.leftBound2d().front().basicPoint2d()), 0.0, 0.0001);
  EXPECT_NEAR(lanelet::geometry::distance2d(copy_lanelet.front().rightBound2d().front().basicPoint2d(), first_lanelet.rightBound2d().front().basicPoint2d()), 0.0, 0.0001);
  EXPECT_NEAR(lanelet::geometry::distance2d(copy_lanelet.back().leftBound2d().back().basicPoint2d(), first_lanelet.leftBound2d().back().basicPoint2d()), 0.0, 0.0001);
  EXPECT_NEAR(lanelet::geometry::distance2d(copy_lanelet.back().rightBound2d().back().basicPoint2d(), first_lanelet.rightBound2d().back().basicPoint2d()), 0.0, 0.0001);

  // check multiple valid ratios
  copy_lanelet = wmb.splitLaneletWithRatio({0.3, 0.6}, first_lanelet, 0.5);
  EXPECT_EQ(copy_lanelet.size(), 3);
  // check connection of two new lanelets
  EXPECT_NEAR(lanelet::geometry::distance2d(copy_lanelet.front().leftBound2d().back().basicPoint2d(), copy_lanelet[1].leftBound2d().front().basicPoint2d()), 0.0, 0.0001);
  EXPECT_NEAR(lanelet::geometry::distance2d(copy_lanelet[1].rightBound2d().back().basicPoint2d(), copy_lanelet.back().rightBound2d().front().basicPoint2d()), 0.0, 0.0001);
  // check if matches previous original lanelet
  EXPECT_NEAR(lanelet::geometry::distance2d(copy_lanelet.front().leftBound2d().front().basicPoint2d(), first_lanelet.leftBound2d().front().basicPoint2d()), 0.0, 0.0001);
  EXPECT_NEAR(lanelet::geometry::distance2d(copy_lanelet.front().rightBound2d().front().basicPoint2d(), first_lanelet.rightBound2d().front().basicPoint2d()), 0.0, 0.0001);
  EXPECT_NEAR(lanelet::geometry::distance2d(copy_lanelet.back().leftBound2d().back().basicPoint2d(), first_lanelet.leftBound2d().back().basicPoint2d()), 0.0, 0.0001);
  EXPECT_NEAR(lanelet::geometry::distance2d(copy_lanelet.back().rightBound2d().back().basicPoint2d(), first_lanelet.rightBound2d().back().basicPoint2d()), 0.0, 0.0001);

  // check multiple valid ratios of mixed
  copy_lanelet = wmb.splitLaneletWithRatio({0.3, 0.6, 0.99}, first_lanelet, 0.5);
  EXPECT_EQ(copy_lanelet.size(), 3);

  // check multiple valid ratios of mixed and not sorted
  copy_lanelet = wmb.splitLaneletWithRatio({0.3, 0.01, 0.6, 0.99}, first_lanelet, 0.5);
  EXPECT_EQ(copy_lanelet.size(), 3);
} 

TEST(WMBroadcaster, splitLaneletWithPoint)
{
  // Create WMBroadcaster object
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {},
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {
      }, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  // Get and convert map to binary message with 26 points of 1 meter in-between distances
  auto map = carma_wm::test::buildGuidanceTestMap(5, 25, 25);
  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(std::move(map_msg_ptr));
  auto first_lanelet = map->laneletLayer.get(1200);
  RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Error messages below are expected...");
  EXPECT_THROW(wmb.splitLaneletWithPoint({}, first_lanelet, 0.5), lanelet::InvalidInputError);

  // check front ratio TOO CLOSE (0.5 meter error for 25 meter lanelet)
  lanelet::BasicPoint2d point{2.5, 0.4};
  auto copy_lanelet = wmb.splitLaneletWithPoint({point}, first_lanelet, 0.5);
  EXPECT_EQ(copy_lanelet.size(), 1);
  EXPECT_NE(copy_lanelet.front().id(), first_lanelet.id());

  // check back TOO CLOSE ratio (0.5 meter error for 25 meter lanelet)
  lanelet::BasicPoint2d point1{2.5, 24.7};
  copy_lanelet = wmb.splitLaneletWithPoint({point1}, first_lanelet, 0.5);
  EXPECT_EQ(copy_lanelet.size(), 1);
  EXPECT_NE(copy_lanelet.front().id(), first_lanelet.id());

  // check one valid ratio
  lanelet::BasicPoint2d point2{2.5, 24.0};
  copy_lanelet = wmb.splitLaneletWithPoint({point2}, first_lanelet, 0.5);
  EXPECT_EQ(copy_lanelet.size(), 2);
  EXPECT_NE(copy_lanelet.front().id(), first_lanelet.id());
  EXPECT_NE(copy_lanelet.back().id(), first_lanelet.id());

  // check one valid ratio
  lanelet::BasicPoint2d point3{2.5, 10.0};
  copy_lanelet = wmb.splitLaneletWithPoint({point3}, first_lanelet, 0.5);
  EXPECT_EQ(copy_lanelet.size(), 2);
  EXPECT_NE(copy_lanelet.front().id(), first_lanelet.id());
  EXPECT_NE(copy_lanelet.back().id(), first_lanelet.id());
  EXPECT_NEAR(carma_wm::geometry::trackPos(first_lanelet, copy_lanelet.front().centerline2d().back().basicPoint2d()).downtrack, 10.0, 0.001);
  EXPECT_NEAR(carma_wm::geometry::trackPos(first_lanelet, copy_lanelet.back().centerline2d().front().basicPoint2d()).downtrack, 10.0, 0.001);

} 

TEST(WMBroadcaster, preprocessWorkzoneGeometry)
{
  // TESTING WORLD IS IN wmb_;
  // Create WMBroadcaster object
  WMBroadcaster wmb_(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {},
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {
      }, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  // create opposite direction road on the left lane
  auto map = carma_wm::test::buildGuidanceTestMap(5, 25, 25);

  auto ll_1 = carma_wm::test::getLanelet(9900, map->laneletLayer.get(1200).rightBound3d().invert(),map->laneletLayer.get(1200).leftBound3d().invert(),lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed);
  auto ll_2 = carma_wm::test::getLanelet(9901, map->laneletLayer.get(1201).rightBound3d().invert(),map->laneletLayer.get(1201).leftBound3d().invert(),lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed);
  auto ll_3 = carma_wm::test::getLanelet(9902, map->laneletLayer.get(1202).rightBound3d().invert(),map->laneletLayer.get(1202).leftBound3d().invert(),lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed);
  auto ll_4 = carma_wm::test::getLanelet(9903, map->laneletLayer.get(1203).rightBound3d().invert(),map->laneletLayer.get(1203).leftBound3d().invert(),lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed);
  
  std::vector<lanelet::Point3d> pts0;
  std::vector<lanelet::Point3d> pts1;
  std::vector<lanelet::Point3d> pts2;
  
  pts0.push_back(map->laneletLayer.get(1203).leftBound3d().back());
  pts0.push_back(carma_wm::test::getPoint(0.0, 101.0, 0));
  pts0.push_back(carma_wm::test::getPoint(0.0, 102.0, 0));
  pts0.push_back(carma_wm::test::getPoint(0.0, 103.0, 0));
  pts0.push_back(carma_wm::test::getPoint(0.0, 104.0, 0));
  pts0.push_back(carma_wm::test::getPoint(0.0, 105.0, 0));
  
  pts1.push_back(map->laneletLayer.get(1203).rightBound3d().back());
  pts1.push_back(carma_wm::test::getPoint(5.0, 101.0, 0));
  pts1.push_back(carma_wm::test::getPoint(5.0, 102.0, 0));
  pts1.push_back(carma_wm::test::getPoint(5.0, 103.0, 0));
  pts1.push_back(carma_wm::test::getPoint(5.0, 104.0, 0));
  pts1.push_back(carma_wm::test::getPoint(5.0, 105.0, 0));
  
  pts2.push_back(map->laneletLayer.get(1213).rightBound3d().back());
  pts2.push_back(carma_wm::test::getPoint(10.0, 101.0, 0));
  pts2.push_back(carma_wm::test::getPoint(10.0, 102.0, 0));
  pts2.push_back(carma_wm::test::getPoint(10.0, 103.0, 0));
  pts2.push_back(carma_wm::test::getPoint(10.0, 104.0, 0));
  pts2.push_back(carma_wm::test::getPoint(10.0, 105.0, 0));
  
  lanelet::LineString3d ls00(lanelet::utils::getId(), pts0);
  lanelet::LineString3d ls01(lanelet::utils::getId(), pts1);
  lanelet::LineString3d ls02(lanelet::utils::getId(), pts2);

  auto ll_9 = carma_wm::test::getLanelet(9904, ls01.invert(),ls00.invert(), lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Solid);
  auto ll_10 = carma_wm::test::getLanelet(9914, ls01,ls02,lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Solid);
  
  map->add(ll_1);
  map->add(ll_2);
  map->add(ll_3);
  map->add(ll_4); 
  map->add(ll_9);
  map->add(ll_10); 

  /**       START
   *        |9904|9914|
   *        | _  _  _ |_  _
   *        |9903|1213|1223|
   *        | _  _  _  _  _|
   *        |9902|1212|1222|
   *        | _  _  _  _  _|
   *        |9901|1211|1221|    num   = lanelet id hardcoded for easier testing
   *        | _  _  _  _  _|    |     = lane lines
   *        |9900|1210|1220|    - - - = Lanelet boundary
   *        |              |    lanelets starting with 990* are opposite direction
   *        ****************
   *           START_LINE
   */

  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));

  // Trigger basemap callback
  wmb_.baseMapCallback(std::move(map_msg_ptr));
  wmb_.setErrorDistance(0.5);
  
  std::shared_ptr<std::vector<lanelet::Lanelet>> parallel_llts = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());
  std::shared_ptr<std::vector<lanelet::Lanelet>> middle_opposite_lanelets = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());
  std::unordered_map<uint8_t, std::shared_ptr<carma_wm_ctrl::Geofence>> work_zone_geofence_cache;
  
  std::shared_ptr<Geofence> gf_ptr1 =  std::make_shared<Geofence>();
  std::shared_ptr<Geofence> gf_ptr2 =  std::make_shared<Geofence>();
  std::shared_ptr<Geofence> gf_ptr3 =  std::make_shared<Geofence>();
  std::shared_ptr<Geofence> gf_ptr4 =  std::make_shared<Geofence>();

  gf_ptr1->label_ = "TYPE:SIG_WZ,INT_ID:1000,SG_ID:235";
  gf_ptr2->label_ = "TYPE:SIG_WZ,INT_ID:1000,SG_ID:235";
  gf_ptr3->label_ = "TYPE:SIG_WZ,INT_ID:1000,SG_ID:235";
  gf_ptr4->label_ = "TYPE:SIG_WZ,INT_ID:1000,SG_ID:235";
  
  /**
   *        |9904|9914|
   *        | _  _  _ |_  _
   *        |9903|    |1223|
   *        | _  _ o   _  _|
   *        |    | o  |1222|
   *        | _r  _c_  _  _|
   *        |  r | c  |1221|    t   = taperright points
   *        | _  _ t_  _  _|    o   = openright points
   *        |9900| t  |1220|    c   = closed points
   *        |              |    r   = reverse points
   *        ****************
   *           START_LINE
   */

  std::vector<lanelet::Point3d> taper_right_pts = {};
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 12.5, 0));
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 25.0, 0));
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 37.5, 0));

  std::vector<lanelet::Point3d> reverse_pts = {};
  reverse_pts.push_back(carma_wm::test::getPoint(2.5, 62.5, 0)); // notice the direction
  reverse_pts.push_back(carma_wm::test::getPoint(2.5, 37.5, 0));

  std::vector<lanelet::Point3d> open_right_pts = {};
  open_right_pts.push_back(carma_wm::test::getPoint(7.5, 62.5, 0));
  open_right_pts.push_back(carma_wm::test::getPoint(7.5, 75.0, 0));
  open_right_pts.push_back(carma_wm::test::getPoint(7.5, 87.5, 0));

  std::vector<lanelet::Point3d> closed = {};
  closed.push_back(carma_wm::test::getPoint(7.5, 40.0, 0));
  closed.push_back(carma_wm::test::getPoint(7.5, 50.0, 0));
  closed.push_back(carma_wm::test::getPoint(7.5, 60.0, 0));

  gf_ptr1->gf_pts = taper_right_pts;
  gf_ptr2->gf_pts = reverse_pts;
  gf_ptr3->gf_pts = open_right_pts;
  gf_ptr4->gf_pts = closed;

  gf_ptr1->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr1->gf_pts);
  gf_ptr2->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr2->gf_pts);
  gf_ptr4->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr4->gf_pts);
  gf_ptr3->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr3->gf_pts);

  work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT] = gf_ptr1;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->affected_parts_.size(), 2);
  work_zone_geofence_cache[WorkZoneSection::REVERSE] = gf_ptr2;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::REVERSE]->affected_parts_.size(), 2);
  work_zone_geofence_cache[WorkZoneSection::OPENRIGHT] = gf_ptr3;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->affected_parts_.size(), 2);
  work_zone_geofence_cache[WorkZoneSection::CLOSED] = gf_ptr4;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::CLOSED]->affected_parts_.size(), 2);
  // get parallel and opposite lanelets
  
  wmb_.preprocessWorkzoneGeometry(work_zone_geofence_cache, parallel_llts, middle_opposite_lanelets);
  
  EXPECT_EQ(parallel_llts->size(), 4);
  EXPECT_EQ(middle_opposite_lanelets->size(), 2);

  // Must be in certain order (taperright):
  EXPECT_NEAR((*(parallel_llts.get()))[0].leftBound2d().front().basicPoint2d().x(), 5.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[0].leftBound2d().front().basicPoint2d().y(), 0.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[0].rightBound2d().back().basicPoint2d().y(), 13.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[1].rightBound2d().back().basicPoint2d().y(), 25.0, 0.0001);

  // Must be in certain order (openright):
  EXPECT_NEAR((*(parallel_llts.get()))[2].leftBound2d().front().basicPoint2d().x(), 5.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[2].leftBound2d().front().basicPoint2d().y(), 75.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[2].rightBound2d().back().basicPoint2d().y(), 88.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[3].rightBound2d().back().basicPoint2d().y(), 100.0, 0.0001);

  // Must be in certain order (reverse, note that it is still in opposite direction):
  EXPECT_NEAR((*(middle_opposite_lanelets.get()))[0].leftBound2d().front().basicPoint2d().x(), 5.0, 0.0001);
  EXPECT_NEAR((*(middle_opposite_lanelets.get()))[0].leftBound2d().front().basicPoint2d().y(), 62.0, 0.0001);
  EXPECT_NEAR((*(middle_opposite_lanelets.get()))[1].rightBound2d().back().basicPoint2d().y(), 37.0, 0.0001);


  /**
   *        |9904|9914|
   *        | _  _ o_ |_  _
   *        |9903| o  |1223|
   *        | _r _ c   _  _|
   *        |  r | c  |1222|
   *        | _r  _c_  _  _|
   *        |  r | c  |1221|    t   = taperright points
   *        | _  _ t_  _  _|    o   = openright points
   *        |9900| t  |1220|    c   = closed points
   *        |              |    r   = reverse points
   *        ****************
   *           START_LINE
   */
  parallel_llts = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());
  middle_opposite_lanelets = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());

  taper_right_pts = {};
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 12.5, 0));
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 25.0, 0));
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 37.5, 0));

  reverse_pts = {};
  reverse_pts.push_back(carma_wm::test::getPoint(2.5, 87.5, 0)); // notice the direction
  reverse_pts.push_back(carma_wm::test::getPoint(2.5, 62.5, 0)); 
  reverse_pts.push_back(carma_wm::test::getPoint(2.5, 37.5, 0));

  open_right_pts = {};
  open_right_pts.push_back(carma_wm::test::getPoint(7.5, 87.5, 0));
  open_right_pts.push_back(carma_wm::test::getPoint(7.5, 102.5, 0));

  closed = {};
  closed.push_back(carma_wm::test::getPoint(7.5, 40.0, 0));
  closed.push_back(carma_wm::test::getPoint(7.5, 50.0, 0));
  closed.push_back(carma_wm::test::getPoint(7.5, 60.0, 0));
  closed.push_back(carma_wm::test::getPoint(7.5, 80.0, 0));

  gf_ptr1->gf_pts = taper_right_pts;
  gf_ptr2->gf_pts = reverse_pts;
  gf_ptr3->gf_pts = open_right_pts;
  gf_ptr4->gf_pts = closed;

  gf_ptr1->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr1->gf_pts);
  gf_ptr2->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr2->gf_pts);
  gf_ptr4->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr4->gf_pts);
  gf_ptr3->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr3->gf_pts);

  work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT] = gf_ptr1;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->affected_parts_.size(), 2);
  work_zone_geofence_cache[WorkZoneSection::REVERSE] = gf_ptr2;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::REVERSE]->affected_parts_.size(), 3);
  work_zone_geofence_cache[WorkZoneSection::OPENRIGHT] = gf_ptr3;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->affected_parts_.size(), 2);
  work_zone_geofence_cache[WorkZoneSection::CLOSED] = gf_ptr4;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::CLOSED]->affected_parts_.size(), 3);
  
  // get parallel and opposite lanelets
  wmb_.preprocessWorkzoneGeometry(work_zone_geofence_cache, parallel_llts, middle_opposite_lanelets);
  
  EXPECT_EQ(parallel_llts->size(), 4);
  EXPECT_EQ(middle_opposite_lanelets->size(), 3);

  // Must be in certain order (taperright):
  EXPECT_NEAR((*(parallel_llts.get()))[0].leftBound2d().front().basicPoint2d().x(), 5.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[0].leftBound2d().front().basicPoint2d().y(), 0.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[0].rightBound2d().back().basicPoint2d().y(), 13.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[1].rightBound2d().back().basicPoint2d().y(), 25.0, 0.0001);

  // Must be in certain order (openright):
  EXPECT_NEAR((*(parallel_llts.get()))[2].leftBound2d().front().basicPoint2d().x(), 5.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[2].leftBound2d().front().basicPoint2d().y(), 100.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[2].rightBound2d().back().basicPoint2d().y(), 103.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[3].rightBound2d().back().basicPoint2d().y(), 105.0, 0.0001);

  // Must be in certain order (reverse, note that it is still in opposite direction):
  EXPECT_NEAR((*(middle_opposite_lanelets.get()))[0].leftBound2d().front().basicPoint2d().x(), 5.0, 0.0001);
  EXPECT_NEAR((*(middle_opposite_lanelets.get()))[0].leftBound2d().front().basicPoint2d().y(), 87.0, 0.0001);
  EXPECT_NEAR((*(middle_opposite_lanelets.get())).back().rightBound2d().back().basicPoint2d().y(), 37.0, 0.0001);

  /**
   *        |9904|9914|
   *        | _  | o_ |_  _
   *        |9903| o  |1223|
   *        | _  | o _|_  _|
   *        |  o | o  |    |
   *        |  r | c  |1222|
   *        |  r | c  |    | 
   *        | _t_|_t_ |_  _|
   *        |9901| t  |1221|    t   = taperright points
   *        | _ _|_t_ |_  _|    o   = openright points
   *        |9900| t  |1220|    c   = closed points
   *        |              |    r   = reverse points
   *        ****************
   *           START_LINE
   */
  parallel_llts = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());
  middle_opposite_lanelets = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());

  taper_right_pts = {};
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 12.5, 0));
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 25.0, 0));
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 37.5, 0));
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 72.5, 0));

  reverse_pts = {};
  reverse_pts.push_back(carma_wm::test::getPoint(2.5, 70.0, 0)); // notice the direction
  reverse_pts.push_back(carma_wm::test::getPoint(2.5, 55.0, 0)); 

  open_right_pts = {};
  open_right_pts.push_back(carma_wm::test::getPoint(7.5, 72.5, 0));
  open_right_pts.push_back(carma_wm::test::getPoint(7.5, 87.5, 0));
  open_right_pts.push_back(carma_wm::test::getPoint(7.5, 102.5, 0));

  closed = {};
  closed.push_back(carma_wm::test::getPoint(7.5, 55.0, 0)); 
  closed.push_back(carma_wm::test::getPoint(7.5, 70.0, 0)); 

  gf_ptr1->gf_pts = taper_right_pts;
  gf_ptr2->gf_pts = reverse_pts;
  gf_ptr3->gf_pts = open_right_pts;
  gf_ptr4->gf_pts = closed;

  
  gf_ptr1->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr1->gf_pts);
  
  gf_ptr2->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr2->gf_pts);
  
  gf_ptr4->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr4->gf_pts);
  
  gf_ptr3->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr3->gf_pts);

  work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT] = gf_ptr1;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->affected_parts_.size(), 3);
  work_zone_geofence_cache[WorkZoneSection::REVERSE] = gf_ptr2;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::REVERSE]->affected_parts_.size(), 1);
  work_zone_geofence_cache[WorkZoneSection::OPENRIGHT] = gf_ptr3;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->affected_parts_.size(), 3);
  work_zone_geofence_cache[WorkZoneSection::CLOSED] = gf_ptr4;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::CLOSED]->affected_parts_.size(), 1);

  // get parallel and opposite lanelets
  wmb_.preprocessWorkzoneGeometry(work_zone_geofence_cache, parallel_llts, middle_opposite_lanelets);
  
  EXPECT_EQ(parallel_llts->size(), 4);
  EXPECT_EQ(middle_opposite_lanelets->size(), 1);

  // Must be in certain order (taperright):
  EXPECT_NEAR((*(parallel_llts.get()))[0].leftBound2d().front().basicPoint2d().x(), 5.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[0].leftBound2d().front().basicPoint2d().y(), 0.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[0].rightBound2d().back().basicPoint2d().y(), 13.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[1].rightBound2d().back().basicPoint2d().y(), 25.0, 0.0001);

  // Must be in certain order (openright):
  EXPECT_NEAR((*(parallel_llts.get()))[2].leftBound2d().front().basicPoint2d().x(), 5.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[2].leftBound2d().front().basicPoint2d().y(), 100.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[2].rightBound2d().back().basicPoint2d().y(), 103.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[3].rightBound2d().back().basicPoint2d().y(), 105.0, 0.0001);

  // Must be in certain order (reverse, note that it is still in opposite direction):
  EXPECT_NEAR((*(middle_opposite_lanelets.get()))[0].leftBound2d().front().basicPoint2d().x(), 5.0, 0.0001);
  EXPECT_NEAR((*(middle_opposite_lanelets.get()))[0].leftBound2d().front().basicPoint2d().y(), 70.0, 0.0001);
  EXPECT_NEAR((*(middle_opposite_lanelets.get())).back().rightBound2d().back().basicPoint2d().y(), 55.0, 0.0001);

  /**
   *        |9904|9914|
   *        | _  |  _ |_  _
   *        |9903| o  |1223|
   *        | _  | o _|_  _|
   *        |  r | c  |    |
   *        |  r | c  |1222|
   *        |  r | c  |    | 
   *        | _r_|_c_ |_  _|
   *        |9901| t  |1221|    t   = taperright points
   *        | _ _|_t_ |_  _|    o   = openright points
   *        |9900|    |1220|    c   = closed points
   *        |              |    r   = reverse points
   *        ****************
   *           START_LINE
   */
  parallel_llts = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());
  middle_opposite_lanelets = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());
  wmb_.setErrorDistance(1.0);
  taper_right_pts = {};
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 25.5, 0));
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 37.5, 0));

  reverse_pts = {};
  reverse_pts.push_back(carma_wm::test::getPoint(2.5, 74.5, 0)); // notice the direction
  reverse_pts.push_back(carma_wm::test::getPoint(2.5, 50.5, 0)); 

  open_right_pts = {};
  open_right_pts.push_back(carma_wm::test::getPoint(7.5, 75.5, 0));
  open_right_pts.push_back(carma_wm::test::getPoint(7.5, 99.5, 0));

  closed = {};
  closed.push_back(carma_wm::test::getPoint(7.5, 50.5, 0)); 
  closed.push_back(carma_wm::test::getPoint(7.5, 74.5, 0)); 

  gf_ptr1->gf_pts = taper_right_pts;
  gf_ptr2->gf_pts = reverse_pts;
  gf_ptr3->gf_pts = open_right_pts;
  gf_ptr4->gf_pts = closed;

  
  gf_ptr1->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr1->gf_pts);
  
  gf_ptr2->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr2->gf_pts);
  
  gf_ptr4->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr4->gf_pts);
  
  gf_ptr3->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr3->gf_pts);

  work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT] = gf_ptr1;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->affected_parts_.size(), 1);
  work_zone_geofence_cache[WorkZoneSection::REVERSE] = gf_ptr2;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::REVERSE]->affected_parts_.size(), 1);
  work_zone_geofence_cache[WorkZoneSection::OPENRIGHT] = gf_ptr3;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->affected_parts_.size(), 1);
  work_zone_geofence_cache[WorkZoneSection::CLOSED] = gf_ptr4;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::CLOSED]->affected_parts_.size(), 1);

  // get parallel and opposite lanelets
  wmb_.preprocessWorkzoneGeometry(work_zone_geofence_cache, parallel_llts, middle_opposite_lanelets);
  
  EXPECT_EQ(parallel_llts->size(), 2);
  EXPECT_EQ(middle_opposite_lanelets->size(), 1);

  // Must be in certain order (taperright):
  EXPECT_NEAR((*(parallel_llts.get()))[0].leftBound2d().front().basicPoint2d().x(), 5.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[0].leftBound2d().front().basicPoint2d().y(), 0.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[0].rightBound2d().back().basicPoint2d().y(), 25.0, 0.0001);

  // Must be in certain order (openright):
  EXPECT_NEAR((*(parallel_llts.get()))[1].leftBound2d().front().basicPoint2d().x(), 5.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[1].leftBound2d().front().basicPoint2d().y(), 100.0, 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[1].rightBound2d().back().basicPoint2d().y(), 105.0, 0.0001);

  // Must be in certain order (reverse, note that it is still in opposite direction):
  EXPECT_NEAR((*(middle_opposite_lanelets.get()))[0].leftBound2d().front().basicPoint2d().x(), 5.0, 0.0001);
  EXPECT_NEAR((*(middle_opposite_lanelets.get()))[0].leftBound2d().front().basicPoint2d().y(), 75.0, 0.0001);
  EXPECT_NEAR((*(middle_opposite_lanelets.get())).back().rightBound2d().back().basicPoint2d().y(), 50.0, 0.0001);
} 

TEST(WMBroadcaster, createWorkzoneGeometry)
{
  /////////////////
  // CREATE WORLD
  /////////////////

  // Create WMBroadcaster object
  WMBroadcaster wmb_(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {},
      [&](const autoware_lanelet2_msgs::msg::MapBin& geofence_bin) {
      }, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});

  // create opposite direction road on the left lane
  auto map = carma_wm::test::buildGuidanceTestMap(5, 25, 25);

  auto ll_1 = carma_wm::test::getLanelet(9900, map->laneletLayer.get(1200).rightBound3d().invert(),map->laneletLayer.get(1200).leftBound3d().invert(),lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed);
  auto ll_2 = carma_wm::test::getLanelet(9901, map->laneletLayer.get(1201).rightBound3d().invert(),map->laneletLayer.get(1201).leftBound3d().invert(),lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed);
  auto ll_3 = carma_wm::test::getLanelet(9902, map->laneletLayer.get(1202).rightBound3d().invert(),map->laneletLayer.get(1202).leftBound3d().invert(),lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed);
  auto ll_4 = carma_wm::test::getLanelet(9903, map->laneletLayer.get(1203).rightBound3d().invert(),map->laneletLayer.get(1203).leftBound3d().invert(),lanelet::AttributeValueString::Solid, lanelet::AttributeValueString::Dashed);
  
  std::vector<lanelet::Point3d> pts0;
  std::vector<lanelet::Point3d> pts1;
  std::vector<lanelet::Point3d> pts2;
  
  pts0.push_back(map->laneletLayer.get(1203).leftBound3d().back());
  pts0.push_back(carma_wm::test::getPoint(0.0, 101.0, 0));
  pts0.push_back(carma_wm::test::getPoint(0.0, 102.0, 0));
  pts0.push_back(carma_wm::test::getPoint(0.0, 103.0, 0));
  pts0.push_back(carma_wm::test::getPoint(0.0, 104.0, 0));
  pts0.push_back(carma_wm::test::getPoint(0.0, 105.0, 0));
  
  pts1.push_back(map->laneletLayer.get(1203).rightBound3d().back());
  pts1.push_back(carma_wm::test::getPoint(5.0, 101.0, 0));
  pts1.push_back(carma_wm::test::getPoint(5.0, 102.0, 0));
  pts1.push_back(carma_wm::test::getPoint(5.0, 103.0, 0));
  pts1.push_back(carma_wm::test::getPoint(5.0, 104.0, 0));
  pts1.push_back(carma_wm::test::getPoint(5.0, 105.0, 0));
  
  pts2.push_back(map->laneletLayer.get(1213).rightBound3d().back());
  pts2.push_back(carma_wm::test::getPoint(10.0, 101.0, 0));
  pts2.push_back(carma_wm::test::getPoint(10.0, 102.0, 0));
  pts2.push_back(carma_wm::test::getPoint(10.0, 103.0, 0));
  pts2.push_back(carma_wm::test::getPoint(10.0, 104.0, 0));
  pts2.push_back(carma_wm::test::getPoint(10.0, 105.0, 0));
  
  lanelet::LineString3d ls00(lanelet::utils::getId(), pts0);
  lanelet::LineString3d ls01(lanelet::utils::getId(), pts1);
  lanelet::LineString3d ls02(lanelet::utils::getId(), pts2);

  auto ll_9 = carma_wm::test::getLanelet(9904, ls01.invert(),ls00.invert(), lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Solid);
  auto ll_10 = carma_wm::test::getLanelet(9914, ls01,ls02,lanelet::AttributeValueString::Dashed, lanelet::AttributeValueString::Solid);
  
  map->add(ll_1);
  map->add(ll_2);
  map->add(ll_3);
  map->add(ll_4); 
  map->add(ll_9);
  map->add(ll_10); 

  /**       START
   *        |9904|9914|
   *        | _  _  _ |_  _
   *        |9903|1213|1223|
   *        | _  _  _  _  _|
   *        |9902|1212|1222|
   *        | _  _  _  _  _|
   *        |9901|1211|1221|    num   = lanelet id hardcoded for easier testing
   *        | _  _  _  _  _|    |     = lane lines
   *        |9900|1210|1220|    - - - = Lanelet boundary
   *        |              |    lanelets starting with 990* are opposite direction
   *        ****************
   *           START_LINE
   */

  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));

  // Trigger basemap callback
  wmb_.baseMapCallback(std::move(map_msg_ptr));
  wmb_.setErrorDistance(0.5);
  
  /////////////////
  // CREATE REQUEST
  /////////////////

  std::shared_ptr<std::vector<lanelet::Lanelet>> parallel_llts = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());
  std::shared_ptr<std::vector<lanelet::Lanelet>> middle_opposite_lanelets = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());
  std::unordered_map<uint8_t, std::shared_ptr<carma_wm_ctrl::Geofence>> work_zone_geofence_cache;
  
  std::shared_ptr<Geofence> gf_ptr1 =  std::make_shared<Geofence>();
  std::shared_ptr<Geofence> gf_ptr2 =  std::make_shared<Geofence>();
  std::shared_ptr<Geofence> gf_ptr3 =  std::make_shared<Geofence>();
  std::shared_ptr<Geofence> gf_ptr4 =  std::make_shared<Geofence>();

  gf_ptr1->label_ = "TYPE:SIG_WZ,INT_ID:1000,SG_ID:235";
  gf_ptr2->label_ = "TYPE:SIG_WZ,INT_ID:1000,SG_ID:235";
  gf_ptr3->label_ = "TYPE:SIG_WZ,INT_ID:1000,SG_ID:235";
  gf_ptr4->label_ = "TYPE:SIG_WZ,INT_ID:1000,SG_ID:235";
  
  /**
   *        |9904|9914|
   *        | _  _  _ |_  _
   *        |9903|    |1223|
   *        | _  _ o   _  _|
   *        |  r | c  |1222|
   *        | _r  _c_  _  _|
   *        |    | t  |1221|    t   = taperright points
   *        | _  _ _  _   _|    o   = openright points
   *        |9900|1210|1220|    c   = closed points
   *        |              |    r   = reverse points
   *        ****************
   *           START_LINE
   */

  std::vector<lanelet::Point3d> taper_right_pts = {};

  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 37.5, 0));
  taper_right_pts.push_back(carma_wm::test::getPoint(7.5, 49.5, 0));

  std::vector<lanelet::Point3d> reverse_pts = {};
  reverse_pts.push_back(carma_wm::test::getPoint(2.5, 74.9, 0)); // notice the direction
  reverse_pts.push_back(carma_wm::test::getPoint(2.5, 50.1, 0));

  std::vector<lanelet::Point3d> open_right_pts = {};
  open_right_pts.push_back(carma_wm::test::getPoint(7.5, 75.1, 0));
  open_right_pts.push_back(carma_wm::test::getPoint(7.5, 87.5, 0));

  std::vector<lanelet::Point3d> closed = {};
  closed.push_back(carma_wm::test::getPoint(7.5, 51.0, 0));
  closed.push_back(carma_wm::test::getPoint(7.5, 74.0, 0));

  gf_ptr1->gf_pts = taper_right_pts;
  gf_ptr2->gf_pts = reverse_pts;
  gf_ptr3->gf_pts = open_right_pts;
  gf_ptr4->gf_pts = closed;

  gf_ptr1->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr1->gf_pts);
  gf_ptr2->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr2->gf_pts);
  gf_ptr4->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr4->gf_pts);
  gf_ptr3->affected_parts_ = wmb_.getAffectedLaneletOrAreas(gf_ptr3->gf_pts);

  work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT] = gf_ptr1;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->affected_parts_.size(), 1);
  work_zone_geofence_cache[WorkZoneSection::REVERSE] = gf_ptr2;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::REVERSE]->affected_parts_.size(), 1);
  work_zone_geofence_cache[WorkZoneSection::OPENRIGHT] = gf_ptr3;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->affected_parts_.size(), 1);
  work_zone_geofence_cache[WorkZoneSection::CLOSED] = gf_ptr4;
  EXPECT_EQ( work_zone_geofence_cache[WorkZoneSection::CLOSED]->affected_parts_.size(), 1);
  
  /////////////////
  // TESTS PREPROCESS
  /////////////////

  // get parallel and opposite lanelets
  wmb_.preprocessWorkzoneGeometry(work_zone_geofence_cache, parallel_llts, middle_opposite_lanelets);
  
  EXPECT_EQ(parallel_llts->size(), 4);
  EXPECT_EQ(middle_opposite_lanelets->size(), 1);

  // Must be in certain order (taperright):
  EXPECT_NEAR((*(parallel_llts.get()))[0].leftBound2d().front().basicPoint2d().x(), map->laneletLayer.get(1210).leftBound2d().back().basicPoint2d().x(), 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[0].leftBound2d().front().basicPoint2d().y(), map->laneletLayer.get(1210).leftBound2d().back().basicPoint2d().y(), 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[0].rightBound2d().front().basicPoint2d().x(), map->laneletLayer.get(1210).rightBound2d().back().basicPoint2d().x(), 0.0001);
  EXPECT_NEAR((*(parallel_llts.get()))[0].rightBound2d().front().basicPoint2d().y(), map->laneletLayer.get(1210).rightBound2d().back().basicPoint2d().y(), 0.0001);

  /////////////////
  // TEST START / CARMA_WM_CTRL part
  /////////////////

  // Create the workzone geometry
  auto gf_ptr = wmb_.createWorkzoneGeometry(work_zone_geofence_cache, parallel_llts->front(), parallel_llts->back(), middle_opposite_lanelets);

  EXPECT_EQ(gf_ptr->lanelet_additions_.size(), 7);

  EXPECT_EQ(gf_ptr->lanelet_additions_[0].leftBound2d().back().id(), gf_ptr->lanelet_additions_[1].leftBound2d().front().id());
  EXPECT_EQ(gf_ptr->lanelet_additions_[1].leftBound2d().back().id(), gf_ptr->lanelet_additions_[2].leftBound2d().front().id());
  EXPECT_EQ(gf_ptr->lanelet_additions_[2].leftBound2d().back().id(), gf_ptr->lanelet_additions_[3].leftBound2d().front().id());
  EXPECT_EQ(gf_ptr->lanelet_additions_[3].leftBound2d().back().id(), gf_ptr->lanelet_additions_[4].leftBound2d().front().id());
  EXPECT_EQ(gf_ptr->lanelet_additions_[0].rightBound2d().back().id(), gf_ptr->lanelet_additions_[1].rightBound2d().front().id());
  EXPECT_EQ(gf_ptr->lanelet_additions_[1].rightBound2d().back().id(), gf_ptr->lanelet_additions_[2].rightBound2d().front().id());
  EXPECT_EQ(gf_ptr->lanelet_additions_[2].rightBound2d().back().id(), gf_ptr->lanelet_additions_[3].rightBound2d().front().id());
  EXPECT_EQ(gf_ptr->lanelet_additions_[3].rightBound2d().back().id(), gf_ptr->lanelet_additions_[4].rightBound2d().front().id());

  /////////////////
  // TEST START / CARM_WM part
  /////////////////

  // update the map with new lanelets (mapUpdateCallback should follow this pattern as well)
  for(auto llt : gf_ptr->lanelet_additions_)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Adding llt with id:" << llt.id());
    auto left = llt.leftBound3d(); //new lanelet coming in
    for (int i = 0; i < left.size(); i ++)
    {
      if (map->pointLayer.exists(left[i].id())) //rewrite the memory address of new pts with that of local
      {
        llt.leftBound3d()[i] = map->pointLayer.get(left[i].id());
      }
    }
    auto right = llt.rightBound3d(); //new lanelet coming in
    for (int i = 0; i < right.size(); i ++)
    {
      if (map->pointLayer.exists(right[i].id())) //rewrite the memory address of new pts with that of local
      {
        llt.rightBound3d()[i] = map->pointLayer.get(right[i].id());
      }
    }
    // add the llt into the map
    EXPECT_NO_THROW(map->add(llt));
  }
  
  // update the list
  auto factory_pcl = lanelet::RegulatoryElementFactory::create(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value(),
                                                          std::const_pointer_cast<lanelet::RegulatoryElementData>(gf_ptr->regulatory_element_->constData()));
  lanelet::RegionAccessRulePtr rar = std::dynamic_pointer_cast<lanelet::RegionAccessRule>(factory_pcl);

  for (auto lanelet_or_area : gf_ptr->affected_parts_)
  {
    map->update(map->laneletLayer.get(lanelet_or_area.lanelet().get().id()), rar);
  }
  
  // this is part of world building process as it was convenient to put it here. not part of the request
  map->update(map->laneletLayer.get(1200), rar);
  map->update(map->laneletLayer.get(1201), rar);
  map->update(map->laneletLayer.get(1202), rar);

  // check routability
  std::shared_ptr<carma_wm::CARMAWorldModel> cmw=std::make_shared<carma_wm::CARMAWorldModel>();
  cmw->setMap(map, 0, true);
  auto map_graph = cmw->getMapRoutingGraph();
  // enable below to debug in rviz graph
  // map_graph->exportGraphViz("../routing.rviz");
  auto route_ = map_graph->getRoute(cmw->getMutableMap()->laneletLayer.get(1210), cmw->getMutableMap()->laneletLayer.get(9914));
  
  // check if memory addresses of connecting points of lanelets actually match after the update
  EXPECT_EQ(cmw->getMutableMap()->laneletLayer.get(gf_ptr->lanelet_additions_[0].id()).leftBound2d().front().constData(), cmw->getMutableMap()->laneletLayer.get(1210).leftBound2d().back().constData());
  EXPECT_EQ(cmw->getMutableMap()->laneletLayer.get(1211).leftBound2d().front().constData(), cmw->getMutableMap()->laneletLayer.get(1210).leftBound2d().back().constData());
  
  EXPECT_EQ(gf_ptr->lanelet_additions_[0].leftBound2d().front().id(), cmw->getMutableMap()->laneletLayer.get(1210).leftBound2d().back().id());
  EXPECT_EQ(gf_ptr->lanelet_additions_[0].rightBound2d().front().id(), cmw->getMutableMap()->laneletLayer.get(1210).rightBound2d().back().id());
  EXPECT_EQ(gf_ptr->lanelet_additions_[4].leftBound2d().back().id(), cmw->getMutableMap()->laneletLayer.get(9914).leftBound2d().front().id());
  EXPECT_EQ(gf_ptr->lanelet_additions_[4].rightBound2d().back().id(), cmw->getMutableMap()->laneletLayer.get(9914).rightBound2d().front().id());

  EXPECT_TRUE(!!route_);
  EXPECT_EQ(route_.get().shortestPath().size(), 7);

  // check if outdated lanelets blocked
  auto route1_ = map_graph->getRoute(map->laneletLayer.get(1210), map->laneletLayer.get(1211));
  ASSERT_FALSE(!!route1_);

}

TEST(WMBroadcaster, WMBroadcaster_VehicleParticipation_Test)
{

  using namespace lanelet::units::literals;

carma_wm::CARMAWorldModel wml;

  // Set the environment  
  size_t base_map_call_count = 0;
  size_t map_update_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        base_map_call_count++;
      }, 
      [&](const autoware_lanelet2_msgs::msg::MapBin& map_bin) {
        // Publish map update callback
        map_update_call_count++;
      }, [](const carma_v2x_msgs::msg::TrafficControlRequest& control_msg_pub_){},
      [](const carma_perception_msgs::msg::CheckActiveGeofence& active_pub_){},
      std::make_shared<TestTimerFactory>(), [](const carma_v2x_msgs::msg::MobilityOperation& tcm_ack_pub_){});


/*Test that Vehicle Participation Type Value is added before baseMapCallback*/
std::string p1 = lanelet::Participants::VehicleCar;
wml.setVehicleParticipationType(p1);

auto value = wmb.getVehicleParticipationType();

ASSERT_EQ(value, p1);

  /*Test Vehicle Participation Type Values in Map **/

    //////
  // Set up the map (add relevant regulatory elements)
  /////
  auto map = carma_wm::getBroadcasterTestMap();
  ASSERT_EQ(map->regulatoryElementLayer.size(), 0);
  // add regems

  //OLD SPEED LIMIT LOADED: This will be assigned to a VehicleTruck participant
  std::string participant1 = lanelet::Participants::VehicleTruck;
  lanelet::DigitalSpeedLimitPtr old_speed_limit1 = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::InvalId, 5_mph, {}, {},
                                                     { participant1}));
  ASSERT_TRUE(old_speed_limit1->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 0);
  map->update(map->laneletLayer.get(10000), old_speed_limit1); // added a speed limit to first llt

  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 1);
  ASSERT_TRUE(map->regulatoryElementLayer.exists(old_speed_limit1->id()));
  ASSERT_EQ(map->regulatoryElementLayer.size(), 1);
  ASSERT_EQ(map->laneletLayer.findUsages(old_speed_limit1).size(), 1);
  ASSERT_EQ(map->laneletLayer.find(10000)->regulatoryElements().front()->id(), old_speed_limit1->id());//should be 10045 old speed limit's id
  ASSERT_EQ(map->laneletLayer.find(10000)->regulatoryElements().front(), old_speed_limit1);

  lanelet::DigitalSpeedLimitPtr test_map_elem = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(map->laneletLayer.find(10000)->regulatoryElements().front());

  auto val2 = wmb.getVehicleParticipationType();

  ASSERT_EQ(test_map_elem->speed_limit_.value(), old_speed_limit1->speed_limit_.value());
  ASSERT_EQ(test_map_elem->participants_.begin()->data(), old_speed_limit1->participants_.begin()->data());

  autoware_lanelet2_msgs::msg::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr(new autoware_lanelet2_msgs::msg::MapBin(msg));
  // Set the map
  wmb.baseMapCallback(std::move(map_msg_ptr));
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::msg::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  
  /*ADD NEW REGELEM TO MAP WITH NEW SL and VPT*/

  lanelet::DigitalSpeedLimitPtr new_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(map->regulatoryElementLayer.uniqueId(), 10_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  map->update(map->laneletLayer.get(10000), new_speed_limit); // add a new speed limit to first llt
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 2);
  ASSERT_TRUE(map->regulatoryElementLayer.exists(new_speed_limit->id()));
  ASSERT_EQ(map->regulatoryElementLayer.size(), 2);
  ASSERT_EQ(map->laneletLayer.findUsages(new_speed_limit).size(), 1);
  ASSERT_EQ(map->laneletLayer.find(10000)->regulatoryElements()[1]->id(), new_speed_limit->id());
  test_map_elem = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(map->laneletLayer.find(10000)->regulatoryElements()[1]);
  ASSERT_EQ(test_map_elem->speed_limit_.value(), new_speed_limit->speed_limit_.value());
  ASSERT_EQ(test_map_elem->participants_.begin()->data(), new_speed_limit->participants_.begin()->data());

  // Set the map
  autoware_lanelet2_msgs::msg::MapBin msg1;
  lanelet::utils::conversion::toBinMsg(map, &msg1);
  autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg_ptr1(new autoware_lanelet2_msgs::msg::MapBin(msg1));
  wmb.baseMapCallback(std::move(map_msg_ptr1));

  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(std::make_unique<std_msgs::msg::String>(sample_proj_string));

  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Map Vehicle Participation Type Test Complete.");

  

}

}  // namespace carma_wm_ctrl
