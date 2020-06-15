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

inline lanelet::LaneletMapPtr getTestMap()
{
  
  using namespace carma_wm;
  // 1. Construct map
  auto p1 = getPoint(0, 0, 0);
  auto p2 = getPoint(0, 1, 0);
  auto p3 = getPoint(1, 1, 0);
  auto p4 = getPoint(1, 2, 0);
  auto p5 = getPoint(1, 0, 0);
  auto p6 = getPoint(2, 0, 0);
  auto p7 = getPoint(2, 1, 0);
  auto p8 = getPoint(2, 2, 0);
  auto p9 = getPoint(1, 3, 0);
  auto p10 = getPoint(2, 3, 0);
  auto p11 = getPoint(1, 4, 0);  // Points for areas
  auto p12 = getPoint(2, 4, 0);
  lanelet::LineString3d left_ls_1(lanelet::utils::getId(), { p1, p2 });
  lanelet::LineString3d right_ls_1(lanelet::utils::getId(), { p5, p3 });
  auto ll_1 = getLanelet(10000, left_ls_1, right_ls_1, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);

  lanelet::LineString3d right_ls_2(lanelet::utils::getId(), { p6, p7 });
  auto ll_2 = getLanelet(10001, right_ls_1, right_ls_2, lanelet::AttributeValueString::Dashed,
                         lanelet::AttributeValueString::Solid);

  lanelet::LineString3d left_ls_3(lanelet::utils::getId(), { p3, p4 });
  lanelet::LineString3d right_ls_3(lanelet::utils::getId(), { p7, p8 });
  auto ll_3 = getLanelet(10002, left_ls_3, right_ls_3, lanelet::AttributeValueString::Solid,
                         lanelet::AttributeValueString::Solid);

  // Add two way linestring
  lanelet::LineString3d left_ls_4(lanelet::utils::getId(), { p4, p9 });
  lanelet::LineString3d right_ls_4(lanelet::utils::getId(), { p8, p10 });
  auto ll_4 = getLanelet(10003, left_ls_4, right_ls_4, lanelet::AttributeValueString::Solid,
                         lanelet::AttributeValueString::Solid);
  ll_4.attributes()[lanelet::AttributeName::OneWay] = "no";

  // We will modify the map here to include 
  // opposite lanelet for id 10000
  lanelet::LineString3d left_ls_inv(lanelet::utils::getId(), { p2, p1 });
  lanelet::LineString3d right_ls_inv(lanelet::utils::getId(), { p3, p5 });
  auto ll_1_inv = carma_wm::getLanelet(10005, left_ls_inv, right_ls_inv, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);

  // 2 different direction lanelets -> |/ (directions)
  // both successors of id 10000
  auto p13 = carma_wm::getPoint(0, 2, 0);
  lanelet::LineString3d left_ls_5(lanelet::utils::getId(), { p2, p4 });
  lanelet::LineString3d right_ls_5(lanelet::utils::getId(), { p3, p8 });
  auto ll_6 = carma_wm::getLanelet(10006, left_ls_5, right_ls_5, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);
  lanelet::LineString3d left_ls_6(lanelet::utils::getId(), { p2, p13 });
  auto ll_5 = carma_wm::getLanelet(10007, left_ls_6, left_ls_3, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);

  // Add an area
  lanelet::LineString3d area_loop(lanelet::utils::getId(), { p9, p11, p12, p10 });

  area_loop.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
  area_loop.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Dashed;

  lanelet::Area area(10004, { area_loop });

  area.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::Multipolygon;
  area.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  area.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
  area.attributes()[lanelet::AttributeNamesString::ParticipantVehicle] = "yes";

  // Create basic map
  lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1, ll_2, ll_3, ll_4 }, { area });

  // create disjoint lanelet, created last to not affect other ids
  auto p1_unreg = getPoint(0, 1, 0); // overlapping lanelet pointing to right, perpendicular direction from others
  auto p2_unreg = getPoint(0, 2, 0); 
  auto p3_unreg = getPoint(2, 1, 0); // notice that this lanelet is long, not connecting to any of those in right lane
  auto p4_unreg = getPoint(2, 2, 0);
  lanelet::LineString3d right_ls_unreg(lanelet::utils::getId(), { p1_unreg, p3_unreg });
  lanelet::LineString3d left_ls_unreg(lanelet::utils::getId(), { p2_unreg, p4_unreg });
  auto ll_unreg = getLanelet(10008, left_ls_unreg, right_ls_unreg, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);

  map->add(ll_1_inv);
  map->add(ll_6);
  map->add(ll_5);
  map->add(ll_unreg);
  return map;
}

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
      },
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
        base_map_call_count++;
      },
      std::make_unique<TestTimerFactory>());

  //////
  // Get and convert map to binary message
  /////
  auto map = getTestMap();

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
  Geofence gf;
  gf.id_ = boost::uuids::random_generator()();
  gf.schedule = GeofenceSchedule(ros::Time(1),  // Schedule between 1 and 8
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
  wmb.geoReferenceCallback(sample_proj_string);

  // Verify adding geofence call
  wmb.geofenceCallback(gf_msg); 

  ros::Time::setNow(ros::Time(2.1));  // Set current time

  std::atomic<std::size_t> temp(0);
  carma_wm::waitForEqOrTimeout(3.0, 1, temp);

  ros::Time::setNow(ros::Time(3.1));  // Set current time

  carma_wm::waitForEqOrTimeout(3.0, 1, temp);
}

TEST(WMBroadcaster, addAndRemoveGeofence)
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
      std::make_unique<TestTimerFactory>());

  //////
  // Set up the map (add relevant regulatory elements)
  /////
  auto map = getTestMap();
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
  auto gf_ptr = std::make_shared<Geofence>(Geofence());
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
  // we can see that the gf_ptr->now would have the prev speed limit of 5_mph that affected llt 10000
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 1);
  ASSERT_EQ(gf_ptr->prev_regems_[0].first, 10000);
  ASSERT_EQ(gf_ptr->prev_regems_[0].second->id(), old_speed_limit->id());

  // now suppose the geofence is finished being used, we have to revert the changes
  wmb.removeGeofence(gf_ptr);
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 0);

  // we can check if the removeGeofence worked, by using addGeofence again and if the original is there again
  wmb.addGeofence(gf_ptr);
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 1);
  ASSERT_EQ(gf_ptr->prev_regems_[0].first, 10000);
  ASSERT_EQ(gf_ptr->prev_regems_[0].second->id(), old_speed_limit->id());

}
}  // namespace carma_wm_ctrl