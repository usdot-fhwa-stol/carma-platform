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

#include <functional>
#include <mutex>
#include <carma_wm_ctrl/WMBroadcaster.h>
#include <carma_wm_ctrl/MapConformer.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <type_traits>

#include <lanelet2_core/primitives/Polygon.h>

// Misheel's additional include files
#include <proj.h>
#include <lanelet2_io/Projection.h>
#include <j2735_msgs/ControlType.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_extension/utility/utilities.h>


namespace carma_wm_ctrl
{
using std::placeholders::_1;

WMBroadcaster::WMBroadcaster(PublishMapCallback map_pub, std::unique_ptr<TimerFactory> timer_factory)
  : map_pub_(map_pub), scheduler_(std::move(timer_factory))
{
  scheduler_.onGeofenceActive(std::bind(&WMBroadcaster::addGeofence, this, _1));
  scheduler_.onGeofenceInactive(std::bind(&WMBroadcaster::removeGeofence, this, _1));
};

void WMBroadcaster::baseMapCallback(const autoware_lanelet2_msgs::MapBinConstPtr& map_msg)
{
  std::lock_guard<std::mutex> guard(map_mutex_);

  static bool firstCall = true;
  // This function should generally only ever be called one time so log a warning if it occurs multiple times
  if (firstCall)
  {
    firstCall = false;
    ROS_INFO("WMBroadcaster::baseMapCallback called for first time with new map message");
  }
  else
  {
    ROS_WARN("WMBroadcaster::baseMapCallback called multiple times in the same node");
  }

  lanelet::LaneletMapPtr new_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*map_msg, new_map);

  base_map_ = new_map;  // Store map
  //TODO dev
  // We dont have the georeference...
  lanelet::MapConformer::ensureCompliance(base_map_);  // Update map to ensure it complies with expectations

  // Publish map
  autoware_lanelet2_msgs::MapBin compliant_map_msg;
  lanelet::utils::conversion::toBinMsg(base_map_, &compliant_map_msg);
  map_pub_(compliant_map_msg);
};

void WMBroadcaster::geofenceCallback(const cav_msgs::ControlMessage& geofence_msg)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  Geofence gf;
  // Get ID
  std::copy(geofence_msg.id.begin(), geofence_msg.id.end(), gf.id_.begin());

  // TODO: logic to determine what type of geofence goes here
  // currently only converting portion of control message that is relevant to digital speed limit geofence
  // Convert from double to Velocity

  if (geofence_msg.control_type.control_type == j2735_msgs::ControlType::MAXSPEED) 
  {
    gf.max_speed_limit_->speed_limit_ = lanelet::Velocity(geofence_msg.control_value.value * lanelet::units::KmH());
    gf.max_speed_limit_->setId(base_map_->regulatoryElementLayer.uniqueId());
  }
  if (geofence_msg.control_type.control_type == j2735_msgs::ControlType::MINSPEED) 
  {
    gf.min_speed_limit_->speed_limit_ = lanelet::Velocity(geofence_msg.control_value.value * lanelet::units::KmH());
    gf.max_speed_limit_->setId(base_map_->regulatoryElementLayer.uniqueId());
  }

  // Get affected lanelet or areas by converting the georeference and querying the map using points in the geofence
  gf.affected_parts_ = getAffectedLaneletOrAreas(geofence_msg);

  // Get schedule
  // TODO dev DOW and UTC offset left
  gf.schedule = GeofenceSchedule(geofence_msg.schedule.start,  
                                 geofence_msg.schedule.end,
                                 geofence_msg.schedule.between.start,     
                                 geofence_msg.schedule.between.end, 
                                 geofence_msg.schedule.repeat.duration,   
                                 geofence_msg.schedule.repeat.interval);
  
  scheduler_.addGeofence(gf);  // Add the geofence to the scheduler
  ROS_INFO_STREAM("New geofence message received by WMBroadcaster with id" << gf.id_);
  
};

void WMBroadcaster::geoReferenceCallback(const std_msgs::String& geo_ref)
{
  base_map_georef_ = geo_ref.data;
}

// TODO dev
lanelet::Lanelets WMBroadcaster::getAffectedLaneletOrAreas(const cav_msgs::ControlMessage& geofence_msg)
{
  if (base_map_georef_ == "")
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map has empty proj string loaded as georeference. Therefore, WMBroadcaster failed to\n ") +
                                          std::string("get transformation between the geofence and the map"));

  if (!base_map_)
  {
    ROS_ERROR_STREAM("In function " << __FUNCTION__ << ": the lanelet map is not set in carma_wm_broadcaster");
    return {};
  }
  
  static PJ* geofence_in_map_proj = proj_create_crs_to_crs(PJ_DEFAULT_CTX, geofence_msg.proj.c_str(), base_map_georef_.c_str(), NULL);
  
  // convert all geofence points into our map's frame
  std::vector<lanelet::Point3d> gf_pts_in_base_map;
  for (auto pt : geofence_msg.points)
  {
    PJ_COORD c{{pt.x, pt.y, pt.z, 0}};
    PJ_COORD c_out;
    c_out = proj_trans(geofence_in_map_proj, PJ_FWD, c);
    gf_pts_in_base_map.push_back(lanelet::Point3d{base_map_->pointLayer.uniqueId(), c_out.xyz.x, c_out.xyz.y});
  }

  // Logic to detect which part is affected
  // REVIEW: dev Currently assuming that there would be at least 1 control message point on the lanelet it is affecting.
  int nearest_element_num = 5;
  // TODO improve the algorithm using the previous example
  std::unordered_set<lanelet::Lanelet> affected_lanelets;
  for (lanelet::Point3d pt : gf_pts_in_base_map)
  {
    // get nearest few lanelets as checking every single llt will be slower
    // and check if the point is within that llt
    for (lanelet::Lanelet llt : base_map_->laneletLayer.nearest(pt, nearest_element_num))
    {
      if (boost::geometry::within(pt.basicPoint2d(), llt.polygon2d()))
        affected_lanelets.insert(llt);
    }
  }
  // Currently only returning lanelet, but this could be expanded to LanelerOrArea compound object 
  // by implementing non-const version of that LaneletOrArea
  std::vector<lanelet::Lanelet> affected_parts;
  affected_parts.insert(affected_parts.end(), affected_lanelets.begin(), affected_lanelets.end());
  return affected_parts;
}

// TODO dev
void WMBroadcaster::addGeofence(Geofence& gf)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_INFO_STREAM("Adding active geofence to the map with geofence id: " << gf.id_);
  
  // TODO: Logic to determine what type of geofence goes here in the future
  // currently only speedchange is available, so it is assumed that

  // First, store the previous limits so that when we are iterating 
  // again to remove it we don't lose the relation between lanelets and limits
  for (auto el: gf.affected_parts_)
  {
    for (auto regem : el.regulatoryElements())
    {
      if (regem->RuleName == "digital_speed_limit") 
        gf.prev_regems_.push_back(std::make_pair(el.id(), regem));
    }
  }

  for (auto pair : gf.prev_regems_)
  {
    if (pair.second->RuleName == "digital_speed_limit") 
      base_map_->remove(pair.second);
    
    // update it with new regem
    if (gf.max_speed_limit_->id() != lanelet::InvalId)  
      base_map_->update(*(base_map_->laneletLayer.find(pair.first)), gf.max_speed_limit_);
    if (gf.min_speed_limit_->id() != lanelet::InvalId) 
      base_map_->update(*(base_map_->laneletLayer.find(pair.first)), gf.min_speed_limit_);
  }
    
};

// TODO dev
void WMBroadcaster::removeGeofence(Geofence& gf)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_INFO_STREAM("Removing inactive geofence to the map with geofence id: " << gf.id_);
  
  // as this gf received is the first gf that was sent in through addGeofence,
  // we have prev speed limit information inside it
  for (auto el: gf.affected_parts_)
  {
    for (auto regem : el.regulatoryElements())
    {
      // removing speed limit added by this geofence
      if (regem->RuleName == "digital_speed_limit") 
        base_map_->remove(regem);
    }
  }
  // put back old speed limits
  for (auto pair : gf.prev_regems_)
  {
    if (pair.second->RuleName == "digital_speed_limit") 
      base_map_->update(*(base_map_->laneletLayer.find(pair.first)), pair.second);
  }

};

}  // namespace carma_wm_ctrl
