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
#include <carma_wm_ctrl/Geofence.h>
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

  // flow for adding geofence to the map
  wmb.addGeofence(gf_ptr);
  // from broadcaster
  autoware_lanelet2_msgs::MapBin gf_obj_msg;
  carma_wm_ctrl::toGeofenceBinMsg(gf_ptr, &gf_obj_msg);
  // at map users
  auto gf_received = std::make_shared<Geofence>(Geofence());
  carma_wm_ctrl::fromGeofenceBinMsg(gf_obj_msg, gf_received);
  ASSERT_EQ(gf_received->id_, gf_ptr->id_);
  ASSERT_EQ(gf_ptr->remove_list_.size(), 1);
  ASSERT_EQ(gf_received->remove_list_.size(), 1); // old_speed_limit
  ASSERT_EQ(gf_received->remove_list_[0].second->attribute(lanelet::AttributeName::Subtype).value(), lanelet::DigitalSpeedLimit::RuleName );
  ASSERT_EQ(gf_received->update_list_.size(), 2); // geofence tags 2 lanelets
  ASSERT_EQ(gf_received->update_list_[1].first, 10000);

  // we can see that the gf_ptr->now would have the prev speed limit of 5_mph that affected llt 10000
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 1);
  ASSERT_EQ(gf_ptr->prev_regems_[0].first, 10000);
  ASSERT_EQ(gf_ptr->prev_regems_[0].second->id(), old_speed_limit->id());

  // now suppose the geofence is finished being used, we have to revert the changes
  wmb.removeGeofence(gf_ptr);
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 0); // should be reset
  // from broadcaster
  autoware_lanelet2_msgs::MapBin gf_msg_revert;
  carma_wm_ctrl::toGeofenceBinMsg(gf_ptr, &gf_msg_revert);
  // at map users
  auto gf_rec_revert = std::make_shared<Geofence>(Geofence());
  carma_wm_ctrl::fromGeofenceBinMsg(gf_msg_revert, gf_rec_revert);

  // previously added update_list_ should be tagged for removal, vice versa
  ASSERT_EQ(gf_rec_revert->remove_list_.size(), 2);
  ASSERT_EQ(gf_rec_revert->remove_list_.size(), gf_received->update_list_.size());
  ASSERT_EQ(gf_rec_revert->update_list_.size(), gf_received->remove_list_.size());
  ASSERT_EQ(gf_rec_revert->update_list_.size(), 1);
  ASSERT_EQ(gf_rec_revert->update_list_[0].first, 10000);
  ASSERT_EQ(gf_rec_revert->update_list_[0].second->id(), old_speed_limit->id());
  
}

}  // namespace carma_wm_ctrl