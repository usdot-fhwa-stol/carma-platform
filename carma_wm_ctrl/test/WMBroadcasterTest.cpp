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
#include <carma_wm_ctrl/GeofenceSchedule.h>
#include <carma_wm_ctrl/Geofence.h>
#include <carma_wm/TrafficControl.h>
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

#include <cav_msgs/ControlMessage.h>
#include <cav_msgs/DaySchedule.h>
#include <cav_msgs/Schedule.h>
#include <cav_msgs/ScheduleParams.h>

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
  WMBroadcaster([](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const autoware_lanelet2_msgs::MapBin& map_bin) {},
   [](const cav_msgs::RouteConstPtr& route_callmsg_pub_){}, std::make_unique<TestTimerFactory>());  // Create broadcaster with test timers. Having this check helps
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
      }, [](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const cav_msgs::RouteConstPtr& route_callmsg_pub_){},
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

// here test the proj string transform test
TEST(WMBroadcaster, getAffectedLaneletOrAreasFromTransform)
{
  using namespace lanelet::units::literals;
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(4, map->laneletLayer.size());  // Verify the map can be decoded

        base_map_call_count++;
      }, [](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const cav_msgs::RouteConstPtr& route_callmsg_pub_){},
      std::make_unique<TestTimerFactory>());

  //////
  // Get and convert map to binary message
  /////
  auto map = carma_wm::getDisjointRouteMap();
  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  // Set the map
  wmb.baseMapCallback(map_msg_ptr);
  ASSERT_EQ(1, base_map_call_count);

  // Setting georeferences
  // geofence's origin (0,0) is at base_map's (10,10)
  std::string base_map_proj_string, geofence_proj_string;
  std_msgs::String base_map_proj;
  base_map_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  geofence_proj_string = "+proj=tmerc +lat_0=39.46645851394806215 +lon_0=-76.16907903057393980 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  base_map_proj.data = base_map_proj_string;
  wmb.geoReferenceCallback(base_map_proj);

  // create the geofence request
  cav_msgs::ControlMessage gf_msg;
  gf_msg.proj = geofence_proj_string;
  // set the points
  cav_msgs::Point pt;
  // check points that are inside lanelets
  pt.x = -8.5; pt.y = -9.5; pt.z = 0; // straight geofence line across 2 lanelets
  gf_msg.points.push_back(pt);
  pt.x = -8.5; pt.y = -8.5; pt.z = 0;
  gf_msg.points.push_back(pt);
  
  lanelet::ConstLaneletOrAreas affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 2);
  ASSERT_EQ(affected_parts[0].id(), 10002);
  ASSERT_EQ(affected_parts[1].id(), 10001);
  // check points that are outside, on the edge, and on the point that makes up the lanelets
  pt.x = -20; pt.y = -10; pt.z = 0;
  gf_msg.points.push_back(pt);
  pt.x = -9; pt.y = -8.5; pt.z = 0;
  gf_msg.points.push_back(pt);
  pt.x = 0; pt.y = 0; pt.z = 0;
  gf_msg.points.push_back(pt);
  
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 2); // newly added ones should not be considered to be on the lanelet
}

// here test assuming the georeference proj strings are the same
TEST(WMBroadcaster, getAffectedLaneletOrAreasOnlyLogic)
{
  using namespace lanelet::units::literals;
  // Set the environment  
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(4, map->laneletLayer.size());  // Verify the map can be decoded

        base_map_call_count++;
      }, [](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const cav_msgs::RouteConstPtr& route_callmsg_pub_){},
      std::make_unique<TestTimerFactory>());

  //////
  // Get and convert map to binary message
  /////
  auto map = carma_wm::getBroadcasterTestMap();

  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  cav_msgs::ControlMessage gf_msg;
  // Check if error are correctly being thrown
  EXPECT_THROW(wmb.getAffectedLaneletOrAreas(gf_msg), lanelet::InvalidObjectStateError);
  // Set the map
  wmb.baseMapCallback(map_msg_ptr);
  ASSERT_EQ(1, base_map_call_count);
  
  EXPECT_THROW(wmb.getAffectedLaneletOrAreas(gf_msg), lanelet::InvalidObjectStateError);
  
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(sample_proj_string);

  // create the control message's relevant parts
  gf_msg.proj = proj_string;
  // set the points
  cav_msgs::Point pt;
  // check points that are inside lanelets
  pt.x = 1.75; pt.y = 0.5; pt.z = 0;
  gf_msg.points.push_back(pt);
  lanelet::ConstLaneletOrAreas affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 0); // this is 0 because there will never be geofence with only 1 pt
                                       // if there is, it won't apply to the map as it doesn't have any direction information, 
                                       // which makes it confusing for overlapping lanelets
  pt.x = 1.75; pt.y = 0.45; pt.z = 0;
  gf_msg.points.push_back(pt);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 0); // although there are two points in the same lanelet,
                                       // lanelet and the two points are not in the same direction

  gf_msg.points.pop_back();
  pt.x = 1.75; pt.y = 0.55; pt.z = 0;
  gf_msg.points.push_back(pt);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 1); // because two points are in one geofence, it will be recorded now
  gf_msg.points.pop_back();
  gf_msg.points.pop_back();

  pt.x = 0.5; pt.y = 0.5; pt.z = 0;    // first of series geofence points across multiple lanelets
  gf_msg.points.push_back(pt);
  pt.x = 0.5; pt.y = 1.1; pt.z = 0;    // adding point in the next lanelet
  gf_msg.points.push_back(pt);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg); 
  ASSERT_EQ(affected_parts.size(), 3);    // although (0.5,1.1) is in another overlapping lanelet (llt_unreg)
                                          // that lanelet is disjoint/doesnt have same direction/not successor of the any lanelet
  
  pt.x = 1.5; pt.y = 2.1; pt.z = 0;    // adding further points in different lanelet narrowing down our direction
  gf_msg.points.push_back(pt);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 3);    // now they are actually 3 different lanelets because we changed direction
  ASSERT_EQ(affected_parts[0].id(), 10003);
  ASSERT_EQ(affected_parts[1].id(), 10006);
  ASSERT_EQ(affected_parts[2].id(), 10000);

  // check points that are outside, on the edge, and on the point that makes up the lanelets
  pt.x = 0.5; pt.y = 0; pt.z = 0;
  gf_msg.points.push_back(pt);
  pt.x = 1.0; pt.y = 0; pt.z = 0;
  gf_msg.points.push_back(pt);
  pt.x = 10; pt.y = 10; pt.z = 0;
  gf_msg.points.push_back(pt);
  
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 2); // they should not be considered to be on the lanelet
}

// Since the actual logic for adding geofences to the map has not yet been added
// this unit test has to be manually verified by looking for the following to log messages
// First "Adding active geofence to the map with geofence id: 1"
// Second "Removing inactive geofence to the map with geofence id: 1"
// Once said logic is added this unit test should be updated
TEST(WMBroadcaster, geofenceCallback)
{
  // Test adding then evaluate if the calls to active and inactive are done correctly
  carma_wm_ctrl::Geofence gf;
  gf.id_ = boost::uuids::random_generator()();
  gf.schedule = carma_wm_ctrl::GeofenceSchedule(ros::Time(1),  // Schedule between 1 and 8
                                 ros::Time(8),
                                 ros::Duration(2),    // Starts at 2
                                 ros::Duration(3.1),  // Ends at by 3.1
                                 ros::Duration(1),    // Duration of 1 and interval of two so active durations are (2-3)
                                 ros::Duration(2));
  // convert to ros msg
  cav_msgs::ControlMessage gf_msg;
  std::copy(gf.id_.begin(),  gf.id_.end(), gf_msg.id.begin());
  gf_msg.schedule.start = gf.schedule.schedule_start_;
  gf_msg.schedule.end = gf.schedule.schedule_end_;
  gf_msg.schedule.between.start =  gf.schedule.control_start_;
  gf_msg.schedule.between.end =  gf.schedule.control_end_;
  gf_msg.schedule.repeat.duration =  gf.schedule.control_duration_;
  gf_msg.schedule.repeat.interval =  gf.schedule.control_interval_;

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
      [&](const autoware_lanelet2_msgs::MapBin& geofence_bin) {
        auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
        carma_wm::fromBinMsg(geofence_bin, data_received);
        ASSERT_EQ(data_received->id_, gf.id_);
        ASSERT_EQ(data_received->remove_list_.size(), 0);
        ASSERT_EQ(data_received->update_list_.size(), 0);
      }, [](const cav_msgs::RouteConstPtr& route_callmsg_pub_){},
      std::make_unique<TestTimerFactory>());

  // Get and convert map to binary message
  auto map = carma_wm::getDisjointRouteMap();
  
  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);

  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(map_msg_ptr);
  ASSERT_EQ(1, base_map_call_count);

  // Setting georefernce otherwise, geofenceCallback will throw exception
  std_msgs::String sample_proj_string;
  sample_proj_string.data = "sample_proj_string"; // it doesn't have to be set correctly for this test
  ROS_INFO_STREAM("Below proj_create: unrecognized format is expected");
  wmb.geoReferenceCallback(sample_proj_string);

  // Verify adding geofence call
  wmb.geofenceCallback(gf_msg); 

  ros::Time::setNow(ros::Time(2.1));  // Set current time

  std::atomic<std::size_t> temp(0);
  carma_wm::waitForEqOrTimeout(3.0, 1, temp);

  ros::Time::setNow(ros::Time(3.1));  // Set current time

  carma_wm::waitForEqOrTimeout(3.0, 1, temp);
}
  
  
TEST(WMBroadcaster, routeCallbackMessage) 
{
  cav_msgs::Route route_msg;
  cav_msgs::RouteConstPtr rpt(new cav_msgs::Route(route_msg));
  
  route_msg.route_path_lanelet_ids.push_back(220); //Add ids to the route_path_lanelet_ids array
  route_msg.route_path_lanelet_ids.push_back(232);
  route_msg.route_path_lanelet_ids.push_back(248);
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback  
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(4, map->laneletLayer.size());  // Verify the map can be decoded

        base_map_call_count++;
      }, [](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const cav_msgs::RouteConstPtr& route_callmsg_pub_){},
      std::make_unique<TestTimerFactory>());
 

  bool flag = false;
  
  ///// Test without user defined route callback
  wmb.routeCallbackMessage(rpt);

  ASSERT_FALSE(flag);
}


TEST(WMBroadcaster, addAndRemoveGeofence)
{
  using namespace lanelet::units::literals;
  // Set the environment  
  size_t base_map_call_count = 0;
  size_t map_update_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        base_map_call_count++;
      }, 
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map update callback
        map_update_call_count++;
      }, [](const cav_msgs::RouteConstPtr& route_callmsg_pub_){},
      std::make_unique<TestTimerFactory>());

  //////
  // Set up the map (add relevant regulatory elements)
  /////
  auto map = carma_wm::getBroadcasterTestMap();
  ASSERT_EQ(map->regulatoryElementLayer.size(), 0);
  // add regems
  lanelet::DigitalSpeedLimitPtr old_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::InvalId, 5_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  ASSERT_EQ(old_speed_limit->attribute(lanelet::AttributeName::Subtype).value(), lanelet::DigitalSpeedLimit::RuleName);
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 0);
  map->update(map->laneletLayer.get(10000), old_speed_limit); // added a speed limit to first llt
  
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 1);
  ASSERT_TRUE(map->regulatoryElementLayer.exists(old_speed_limit->id()));
  ASSERT_EQ(map->regulatoryElementLayer.size(), 1);
  ASSERT_EQ(map->laneletLayer.findUsages(old_speed_limit).size(), 1);
  ASSERT_EQ(map->laneletLayer.find(10000)->regulatoryElements().front()->id(), old_speed_limit->id());//should be 10045 old speed limit's id
  
  
  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));
  // Set the map
  wmb.baseMapCallback(map_msg_ptr);
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(sample_proj_string);

  // Create the geofence object
  auto gf_ptr = std::make_shared<carma_wm_ctrl::Geofence>(carma_wm_ctrl::Geofence());
  gf_ptr->id_ = boost::uuids::random_generator()();
  cav_msgs::ControlMessage gf_msg;
  lanelet::DigitalSpeedLimitPtr new_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(map->regulatoryElementLayer.uniqueId(), 10_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  gf_ptr->min_speed_limit_ = new_speed_limit;
  // create the control message's relevant parts to fill the object
  gf_msg.proj = proj_string;
  // set the points
  cav_msgs::Point pt;
  // check points that are inside lanelets
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;
  gf_msg.points.push_back(pt);
  pt.x = 0.5; pt.y = 1.5; pt.z = 0;
  gf_msg.points.push_back(pt);
  gf_ptr->affected_parts_ = wmb.getAffectedLaneletOrAreas(gf_msg);

  ASSERT_EQ(gf_ptr->affected_parts_.size(), 2);
  ASSERT_EQ(gf_ptr->affected_parts_[1].id(), 10000);
  ASSERT_EQ(gf_ptr->affected_parts_[1].regulatoryElements()[0]->id(), old_speed_limit->id()); // old speed limit
  ASSERT_EQ(gf_ptr->affected_parts_[1].regulatoryElements().size(), 4); // old speed limit and other map conforming regulations
  // process the geofence and change the map

  wmb.addGeofence(gf_ptr);
  ASSERT_EQ(map_update_call_count, 1);
  // we can see that the gf_ptr->now would have the prev speed limit of 5_mph that affected llt 10000
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 1);
  ASSERT_EQ(gf_ptr->prev_regems_[0].first, 10000);
  ASSERT_EQ(gf_ptr->prev_regems_[0].second->id(), old_speed_limit->id());

  // now suppose the geofence is finished being used, we have to revert the changes
  wmb.removeGeofence(gf_ptr);
  ASSERT_EQ(map_update_call_count, 2);
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 0);

  // we can check if the removeGeofence worked, by using addGeofence again and if the original is there again
  wmb.addGeofence(gf_ptr);
  ASSERT_EQ(map_update_call_count, 3);
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 1);
  ASSERT_EQ(gf_ptr->prev_regems_[0].first, 10000);
  ASSERT_EQ(gf_ptr->prev_regems_[0].second->id(), old_speed_limit->id());

}

TEST(WMBroadcaster, GeofenceBinMsgTest)
{
using namespace lanelet::units::literals;
  // Set the environment  
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        base_map_call_count++;
      },
      [](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const cav_msgs::RouteConstPtr& route_callmsg_pub_){},
      std::make_unique<TestTimerFactory>());
  
  /////
  // Set up the map (add relevant regulatory elements)
  /////
  auto map = carma_wm::getBroadcasterTestMap();
  ASSERT_EQ(map->regulatoryElementLayer.size(), 0);
  // add regems
  lanelet::DigitalSpeedLimitPtr old_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::InvalId, 5_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  ASSERT_EQ(old_speed_limit->attribute(lanelet::AttributeName::Subtype).value(), lanelet::DigitalSpeedLimit::RuleName);
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 0);
  map->update(map->laneletLayer.get(10000), old_speed_limit); // added a speed limit to first llt
  
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 1);
  ASSERT_TRUE(map->regulatoryElementLayer.exists(old_speed_limit->id()));
  ASSERT_EQ(map->regulatoryElementLayer.size(), 1);
  ASSERT_EQ(map->laneletLayer.findUsages(old_speed_limit).size(), 1);
  ASSERT_EQ(map->laneletLayer.find(10000)->regulatoryElements().front()->id(), old_speed_limit->id());//should be 10045 old speed limit's id
  
  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));
  // Set the map
  wmb.baseMapCallback(map_msg_ptr);
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(sample_proj_string);

  // Create the geofence object
  auto gf_ptr = std::make_shared<carma_wm_ctrl::Geofence>(carma_wm_ctrl::Geofence());
  gf_ptr->id_ = boost::uuids::random_generator()();
  cav_msgs::ControlMessage gf_msg;
  lanelet::DigitalSpeedLimitPtr new_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(map->regulatoryElementLayer.uniqueId(), 10_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  gf_ptr->min_speed_limit_ = new_speed_limit;
  // create the control message's relevant parts to fill the object
  gf_msg.proj = proj_string;
  // set the points
  cav_msgs::Point pt;
  // check points that are inside lanelets
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;
  gf_msg.points.push_back(pt);
  pt.x = 0.5; pt.y = 1.5; pt.z = 0;
  gf_msg.points.push_back(pt);
  gf_ptr->affected_parts_ = wmb.getAffectedLaneletOrAreas(gf_msg);

  ASSERT_EQ(gf_ptr->affected_parts_.size(), 2);
  ASSERT_EQ(gf_ptr->affected_parts_[1].id(), 10000);
  ASSERT_EQ(gf_ptr->affected_parts_[1].regulatoryElements()[0]->id(), old_speed_limit->id()); // old speed limit
  ASSERT_EQ(gf_ptr->affected_parts_[1].regulatoryElements().size(), 4); // old speed limit and other map conforming regulations
  // process the geofence and change the map

  // flow for adding geofence to the map
  wmb.addGeofence(gf_ptr);
  // from broadcaster
  autoware_lanelet2_msgs::MapBin gf_obj_msg;

  auto send_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_));
  carma_wm::toBinMsg(send_data, &gf_obj_msg);
  // at map users
  auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  carma_wm::fromBinMsg(gf_obj_msg, data_received);
  ASSERT_EQ(data_received->id_, gf_ptr->id_);
  ASSERT_EQ(gf_ptr->remove_list_.size(), 1);
  ASSERT_EQ(data_received->remove_list_.size(), 1); // old_speed_limit
  ASSERT_EQ(data_received->remove_list_[0].second->attribute(lanelet::AttributeName::Subtype).value(), lanelet::DigitalSpeedLimit::RuleName );
  ASSERT_EQ(data_received->update_list_.size(), 2); // geofence tags 2 lanelets
  ASSERT_EQ(data_received->update_list_[1].first, 10000);

  // we can see that the gf_ptr->now would have the prev speed limit of 5_mph that affected llt 10000
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 1);
  ASSERT_EQ(gf_ptr->prev_regems_[0].first, 10000);
  ASSERT_EQ(gf_ptr->prev_regems_[0].second->id(), old_speed_limit->id());

  // now suppose the geofence is finished being used, we have to revert the changes
  wmb.removeGeofence(gf_ptr);
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 0); // should be reset
  // from broadcaster
  autoware_lanelet2_msgs::MapBin gf_msg_revert;
  auto send_data_revert = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_));
  carma_wm::toBinMsg(send_data_revert, &gf_msg_revert);
  // at map users
  auto rec_data_revert = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  carma_wm::fromBinMsg(gf_msg_revert, rec_data_revert);

  // previously added update_list_ should be tagged for removal, vice versa
  ASSERT_EQ(rec_data_revert->remove_list_.size(), 2);
  ASSERT_EQ(rec_data_revert->remove_list_.size(), data_received->update_list_.size());
  ASSERT_EQ(rec_data_revert->update_list_.size(), data_received->remove_list_.size());
  ASSERT_EQ(rec_data_revert->update_list_.size(), 1);
  ASSERT_EQ(rec_data_revert->update_list_[0].first, 10000);
  ASSERT_EQ(rec_data_revert->update_list_[0].second->id(), old_speed_limit->id());
  
}
}  // namespace carma_wm_ctrl
