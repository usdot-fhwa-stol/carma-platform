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


// Misheel's additional include files
#include <proj.h>
#include <lanelet2_io/Projection.h>
#include <j2735_msgs/ControlType.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_core/Forward.h>


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

  std::copy(geofence_msg.id.begin(), geofence_msg.id.end(), gf.id_.begin());

  // TODO: logic to determine what type of geofence goes here
  // currently only converting portion of control message that is relevant to digital speed limit geofence
  // Convert from double to Velocity
  if (geofence_msg.control_type.control_type == j2735_msgs::ControlType::MAXSPEED) 
  {
    gf.max_speed_limit_->speed_limit_ = lanelet::Velocity(geofence_msg.control_value.value * lanelet::units::KmH());
  }
  if (geofence_msg.control_type.control_type == j2735_msgs::ControlType::MINSPEED) 
  {
    gf.min_speed_limit_->speed_limit_ = lanelet::Velocity(geofence_msg.control_value.value * lanelet::units::KmH());
  }

  // Get affected lanelet or areas by converting the georeference and querying the map using points in the geofence
  gf.affected_parts_ = getAffectedLaneletOrAreas(geofence_msg);

  //TODO: Uncomment once gf is fully defined
  scheduler_.addGeofence(gf);  // Add the geofence to the scheduler
  ROS_INFO_STREAM("New geofence message received by WMBroadcaster with id" << gf.id_);
};

// TODO dev
lanelet::ConstLaneletOrAreas WMBroadcaster::getAffectedLaneletOrAreas(const cav_msgs::ControlMessage& geofence_msg)
{
  // Convert the points x,y,z to our local frame's x,y,z
  // 1. we might need to get tf between geofence_msg.proj to our map's georeference
  // 2. convert all geofence_msg.points

  // We need to query the affected lanelets or areas
  // - currently assuming only lanelet is affected (will change in the future)
  // - points fell within the boundaries are considered to point that lanelet
  // - if the points exactly match the the boundary, it is considered to not relate to that lanelet

  // input: map georeference, and gf georeference
  // coordinates in lat/lon as well as xyz
  // msg proj and datum should convert from lat/lon to xyz
  // need to convert form msj map's xyz to map's georeference xyz

  // REVIEW: current approach sort of codes it barebone, and not depend on lanelet2_extension/projection
  // get projector from geofence's local coordinates to base map's coordinates using their respective georeferences
  std::string base_map_georef = base_map_->getGeoreference();
  if (base_map_georef == "")
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map has empty proj string loaded as georeference. Therefore, WMBroadcaster failed to\n ") +
                                          std::string("get transformation between the geofence and the map"));

  static PJ* geofence_in_map_proj = proj_create_crs_to_crs(PJ_DEFAULT_CTX, geofence_msg.proj.c_str(), base_map_georef.c_str(), NULL);
  
  // convert all geofence points into our map's frame
  std::vector<lanelet::Point3d> gf_pts_base_map;
  for (auto pt : geofence_msg.points)
  {
    PJ_COORD c{{pt.x, pt.y, pt.z, 0}};
    PJ_COORD c_out;
    c_out = proj_trans(geofence_in_map_proj, PJ_FWD, c);
    gf_pts_base_map.push_back(lanelet::Point3d{c_out.xyz.x, c_out.xyz.y, c_out.xyz.z});
  }

  // Logic to detect which part is affected
  lanelet::ConstLaneletOrAreas affected_parts;

  return affected_parts;
}

// TODO dev
void WMBroadcaster::addGeofence(const Geofence& gf)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_INFO_STREAM("Adding active geofence to the map with geofence id: " << gf.id_);
  // this gf received is actually likely to be sent to removeGeofence again
  // if so, there is no need to save it somewhere
  
  // Logic to determine what type of geofence goes here
  // currently only speedchange is available


  // 1. Remove the prev speed limit and store it somewhere
  // 2. Add the new speed limit

};

// TODO dev
void WMBroadcaster::removeGeofence(const Geofence& gf)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_INFO_STREAM("Removing inactive geofence to the map with geofence id: " << gf.id_);
  // this gf received is actually likely to be the first gf that was sent in
  // if so, there is no need to save it somewhere
  
  // 1. Remove the current speed limit
  // 2. Add back the previous speed limit it was there

};

}  // namespace carma_wm_ctrl
