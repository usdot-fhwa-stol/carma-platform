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
  lanelet::LaneletMapPtr new_map_to_change(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*map_msg, new_map);
  lanelet::utils::conversion::fromBinMsg(*map_msg, new_map_to_change);

  base_map_ = new_map;  // Store map
  current_map_ = new_map_to_change; // broadcaster makes changes to this
                                    // TODO publication of this modified map will be handled in the future

  lanelet::MapConformer::ensureCompliance(base_map_);  // Update map to ensure it complies with expectations
                                                       // base_map current_map_ are same here, so only 1 compliance needed

  // Publish map
  autoware_lanelet2_msgs::MapBin compliant_map_msg;
  lanelet::utils::conversion::toBinMsg(base_map_, &compliant_map_msg);
  map_pub_(compliant_map_msg);
};

const std::shared_ptr<Geofence> WMBroadcaster::geofenceFromMsg(const cav_msgs::ControlMessage& geofence_msg)
{
  auto gf_ptr = std::make_shared<Geofence>(Geofence());
  // Get ID
  std::copy(geofence_msg.id.begin(), geofence_msg.id.end(), gf_ptr->id_.begin());

  // TODO: logic to determine what type of geofence goes here
  // currently only converting portion of control message that is relevant to digital speed limit geofence
  // Convert from double to Velocity

  if (geofence_msg.control_type.control_type == j2735_msgs::ControlType::MAXSPEED) 
  {
    gf_ptr->max_speed_limit_->speed_limit_ = lanelet::Velocity(geofence_msg.control_value.value * lanelet::units::MPH());
    gf_ptr->max_speed_limit_->setId(current_map_->regulatoryElementLayer.uniqueId());
  }
  if (geofence_msg.control_type.control_type == j2735_msgs::ControlType::MINSPEED) 
  {
    gf_ptr->min_speed_limit_->speed_limit_ = lanelet::Velocity(geofence_msg.control_value.value * lanelet::units::MPH());
    gf_ptr->max_speed_limit_->setId(current_map_->regulatoryElementLayer.uniqueId());
  }

  // Get affected lanelet or areas by converting the georeference and querying the map using points in the geofence
  gf_ptr->affected_parts_ = getAffectedLaneletOrAreas(geofence_msg);

  // Get schedule (assuming everything is in UTC currently)
  gf_ptr->schedule = GeofenceSchedule(geofence_msg.schedule.start,  
                                 geofence_msg.schedule.end,
                                 geofence_msg.schedule.between.start,     
                                 geofence_msg.schedule.between.end, 
                                 geofence_msg.schedule.repeat.duration,   
                                 geofence_msg.schedule.repeat.interval);
  return gf_ptr;
}


void WMBroadcaster::geofenceCallback(const cav_msgs::ControlMessage& geofence_msg)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  auto gf_ptr = geofenceFromMsg(geofence_msg);
  scheduler_.addGeofence(gf_ptr);  // Add the geofence to the scheduler
  ROS_INFO_STREAM("New geofence message received by WMBroadcaster with id" << gf_ptr->id_);
  
};

void WMBroadcaster::geoReferenceCallback(const std_msgs::String& geo_ref)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  base_map_georef_ = geo_ref.data;
}

lanelet::ConstLaneletOrAreas WMBroadcaster::getAffectedLaneletOrAreas(const cav_msgs::ControlMessage& geofence_msg)
{
  if (!current_map_)
  {
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map is not loaded to the WMBroadcaster"));
  }
  if (base_map_georef_ == "")
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map has empty proj string loaded as georeference. Therefore, WMBroadcaster failed to\n ") +
                                          std::string("get transformation between the geofence and the map"));

  PJ* geofence_in_map_proj = proj_create_crs_to_crs(PJ_DEFAULT_CTX, geofence_msg.proj.c_str(), base_map_georef_.c_str(), NULL);
  
  // convert all geofence points into our map's frame
  std::vector<lanelet::Point3d> gf_pts;
  for (auto pt : geofence_msg.points)
  {
    PJ_COORD c{{pt.x, pt.y, pt.z, 0}};
    PJ_COORD c_out;
    c_out = proj_trans(geofence_in_map_proj, PJ_FWD, c);
    gf_pts.push_back(lanelet::Point3d{current_map_->pointLayer.uniqueId(), c_out.xyz.x, c_out.xyz.y});
  }

  // Logic to detect which part is affected
  double lane_max_width = 4;
  
  std::unordered_set<lanelet::Lanelet> affected_lanelets;
  for (int idx = 0; idx < gf_pts.size(); idx ++)
  {
    std::unordered_set<lanelet::Lanelet> possible_lanelets;
    // get nearest few nearest llts within lane_max_width
    // which actually house this geofence_point
    auto searchFunc = [&](const lanelet::BoundingBox2d& lltBox, const lanelet::Lanelet& llt) 
    {
      bool should_stop = boost::geometry::distance(gf_pts[idx].basicPoint2d(), llt.polygon2d()) > lane_max_width;
      if (!should_stop && boost::geometry::within(gf_pts[idx].basicPoint2d(), llt.polygon2d()))
      {
        possible_lanelets.insert(llt);
      }
      return should_stop;
    };
   
    current_map_->laneletLayer.nearestUntil(gf_pts[idx], searchFunc);

    // among these llts, filter the ones that are on same direction as the geofence
    if (idx + 1 == gf_pts.size()) // break if this is the last gf_pt after saving everything
    {
      for (auto llt: possible_lanelets)
      {
        affected_lanelets.insert(llt);
      }
      break;
    } 
  
    // check if each lines connecting end points of the llt is crossing with the line connecting current and next gf_pts
    for (auto llt: possible_lanelets)
    {
    
      lanelet::BasicLineString2d gf_dir_line({gf_pts[idx].basicPoint2d(), gf_pts[idx+1].basicPoint2d()});
      lanelet::BasicLineString2d llt_boundary({(llt.leftBound2d().end() -1)->basicPoint2d(), (llt.rightBound2d().end() - 1)->basicPoint2d()});
      if (boost::geometry::intersects(llt_boundary, gf_dir_line))
      {
        // record the llts that are on the same dir
        affected_lanelets.insert(llt);
      }
    }
  }
  
  // Currently only returning lanelet, but this could be expanded to LanelerOrArea compound object 
  // by implementing non-const version of that LaneletOrArea
  lanelet::ConstLaneletOrAreas affected_parts;
  affected_parts.insert(affected_parts.end(), affected_lanelets.begin(), affected_lanelets.end());
  return affected_parts;
}

void WMBroadcaster::addSpeedLimit(const std::shared_ptr<Geofence>& gf_ptr)
{
  // First loop is to save the relation between element and regulatory element
  // so that we can add back the old one after geofence deactivates
  for (auto el: gf_ptr->affected_parts_)
  {
    for (auto regem : el.regulatoryElements())
    {
      if (regem->attribute(lanelet::AttributeName::Subtype).value() == lanelet::DigitalSpeedLimit::RuleName)
        gf_ptr->prev_regems_.push_back(std::make_pair(el.id(), current_map_->regulatoryElementLayer.get(regem->id())));
    }
  }
  
  // this is loop is kept separately because removing while iterating 
  // can lose some relation where one regem affects multiple elements
  for (auto pair : gf_ptr->prev_regems_)
  {
    if (pair.second->attribute(lanelet::AttributeName::Subtype).value() == lanelet::DigitalSpeedLimit::RuleName)
    {
      // we have been using Const primitive so far, as remove requires non-const, we do following:
      current_map_->remove(current_map_->regulatoryElementLayer.get(pair.second->id()));
    }
  }

  // this loop is also kept separately because previously we assumed 
  // there was existing regem, but this handles changes to all of the elements
  for (auto el: gf_ptr->affected_parts_)
  {
    // update it with new regem
    if (gf_ptr->max_speed_limit_->id() != lanelet::InvalId)
      current_map_->update(current_map_->laneletLayer.get(el.id()), gf_ptr->max_speed_limit_); 
    if (gf_ptr->min_speed_limit_->id() != lanelet::InvalId)
      current_map_->update(current_map_->laneletLayer.get(el.id()), gf_ptr->min_speed_limit_);
  }
}

void WMBroadcaster::addGeofence(const std::shared_ptr<Geofence>& gf_ptr)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_INFO_STREAM("Adding active geofence to the map with geofence id: " << gf_ptr->id_);
  
  // TODO: Logic to determine what type of geofence goes here in the future
  // currently only speedchange is available, so it is assumed that
  addSpeedLimit(gf_ptr);

};

void WMBroadcaster::removeGeofence(const std::shared_ptr<Geofence>& gf_ptr)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_INFO_STREAM("Removing inactive geofence to the map with geofence id: " << gf_ptr->id_);
  
  // As this gf received is the first gf that was sent in through addGeofence,
  // we have prev speed limit information inside it
  for (auto el: gf_ptr->affected_parts_)
  {
    for (auto regem : el.regulatoryElements())
    {
      // removing speed limit added by this geofence
      if (regem->attribute(lanelet::AttributeName::Subtype).value() == lanelet::DigitalSpeedLimit::RuleName)
        current_map_->remove(current_map_->regulatoryElementLayer.get(regem->id()));
    }
  }
  // put back old speed limits
  for (auto pair : gf_ptr->prev_regems_)
  {
    if (pair.second->attribute(lanelet::AttributeName::Subtype).value() == lanelet::DigitalSpeedLimit::RuleName) 
      current_map_->update(current_map_->laneletLayer.get(pair.first), pair.second);
  }
  // as all changes are reverted back, we no longer need prev_regems
  gf_ptr->prev_regems_ = {};
};

}  // namespace carma_wm_ctrl
