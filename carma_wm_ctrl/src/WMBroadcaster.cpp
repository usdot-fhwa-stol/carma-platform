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
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_extension/utility/utilities.h>
#include <algorithm>
#include <carma_wm/Geometry.h>
#include <math.h>

namespace carma_wm_ctrl
{
using std::placeholders::_1;


WMBroadcaster::WMBroadcaster(const PublishMapCallback& map_pub, const PublishMapUpdateCallback& map_update_pub, std::unique_ptr<TimerFactory> timer_factory)
  : map_pub_(map_pub), map_update_pub_(map_update_pub), scheduler_(std::move(timer_factory))
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

  lanelet::MapConformer::ensureCompliance(base_map_);     // Update map to ensure it complies with expectations
  lanelet::MapConformer::ensureCompliance(current_map_);

  // Publish map
  autoware_lanelet2_msgs::MapBin compliant_map_msg;
  lanelet::utils::conversion::toBinMsg(base_map_, &compliant_map_msg);
  map_pub_(compliant_map_msg);
};

std::shared_ptr<Geofence> WMBroadcaster::geofenceFromMsg(const cav_msgs::TrafficControlMessageV01& msg_v01)
{
  auto gf_ptr = std::make_shared<Geofence>(Geofence());
  // Get ID
  std::copy(msg_v01.id.id.begin(), msg_v01.id.id.end(), gf_ptr->id_.begin());

  // Get affected lanelet or areas by converting the georeference and querying the map using points in the geofence
  gf_ptr->affected_parts_ = getAffectedLaneletOrAreas(msg_v01);

  std::vector<lanelet::Lanelet> affected_llts;
  std::vector<lanelet::Area> affected_areas;

  // used for assigning them to the regem as parameters
  for (auto llt_or_area : gf_ptr->affected_parts_)
  {
    if (llt_or_area.isLanelet()) affected_llts.push_back(current_map_->laneletLayer.get(llt_or_area.lanelet()->id()));
    if (llt_or_area.isArea()) affected_areas.push_back(current_map_->areaLayer.get(llt_or_area.area()->id()));
  }

  // TODO: logic to determine what type of geofence goes here
  // currently only converting portion of control message that is relevant to:
  // - digital speed limit, passing control line

  cav_msgs::TrafficControlDetail msg_detail = msg_v01.params.detail;
  
  if (msg_detail.choice == cav_msgs::TrafficControlDetail::MAXSPEED_CHOICE) 
  {
    gf_ptr->regulatory_element_ = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::utils::getId(), 
                                        lanelet::Velocity(msg_detail.maxspeed * lanelet::units::MPH()),
                                        affected_llts, affected_areas, { lanelet::Participants::VehicleCar }));
  }
  
  if (msg_detail.choice == cav_msgs::TrafficControlDetail::MINSPEED_CHOICE) 
  {
    gf_ptr->regulatory_element_ = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::utils::getId(), 
                                        lanelet::Velocity(msg_detail.minspeed * lanelet::units::MPH()),
                                        affected_llts, affected_areas, { lanelet::Participants::VehicleCar }));
  }
  if (msg_detail.choice == cav_msgs::TrafficControlDetail::LATPERM_CHOICE || msg_detail.choice == cav_msgs::TrafficControlDetail::LATAFFINITY_CHOICE)
  {
    // Get affected bounds
    lanelet::LineStrings3d pcl_bounds;
    if (msg_detail.lataffinity == cav_msgs::TrafficControlDetail::LEFT)
    {
      for (auto llt : affected_llts) pcl_bounds.push_back(llt.leftBound());
      gf_ptr->pcl_affects_left_ = true;
    }
    else //right
    {
      for (auto llt : affected_llts) pcl_bounds.push_back(llt.rightBound());
      gf_ptr->pcl_affects_right_ = true;
    }

    // Get specified participants
    ros::V_string left_participants, right_participants, participants;
    for (j2735_msgs::TrafficControlVehClass participant : msg_v01.params.vclasses)
    {
      // Currently j2735_msgs::TrafficControlVehClass::RAIL is not supported
      if (participant.vehicle_class == j2735_msgs::TrafficControlVehClass::ANY)
      {
        participants = {lanelet::Participants::Vehicle, lanelet::Participants::Pedestrian, lanelet::Participants::Bicycle};
        break;
      }
      else if (participant.vehicle_class == j2735_msgs::TrafficControlVehClass::PEDESTRIAN)
      {
        participants.push_back(lanelet::Participants::Pedestrian);
      }
      else if (participant.vehicle_class == j2735_msgs::TrafficControlVehClass::BICYCLE)
      {
        participants.push_back(lanelet::Participants::Bicycle);
      }
      else if (participant.vehicle_class == j2735_msgs::TrafficControlVehClass::MICROMOBILE ||
               participant.vehicle_class == j2735_msgs::TrafficControlVehClass::MOTORCYCLE)
      {
        participants.push_back(lanelet::Participants::VehicleMotorcycle);
      }
      else if (participant.vehicle_class == j2735_msgs::TrafficControlVehClass::BUS)
      {
        participants.push_back(lanelet::Participants::VehicleBus);
      }
      else if (participant.vehicle_class == j2735_msgs::TrafficControlVehClass::LIGHT_TRUCK_VAN ||
              participant.vehicle_class == j2735_msgs::TrafficControlVehClass::PASSENGER_CAR)
      {
        participants.push_back(lanelet::Participants::VehicleCar);
      }
      else if (8<= participant.vehicle_class && participant.vehicle_class <= 16) // Truck enum definition range from 8-16 currently
      {
        participants.push_back(lanelet::Participants::VehicleTruck);
      }
    }

    // Create the pcl depending on the allowed passing control direction, left, right, or both
    if (msg_detail.latperm[0] == cav_msgs::TrafficControlDetail::PERMITTED ||
        msg_detail.latperm[0] == cav_msgs::TrafficControlDetail::PASSINGONLY)
    {
      left_participants = participants; 
    }
    else if (msg_detail.latperm[0] == cav_msgs::TrafficControlDetail::EMERGENCYONLY)
    {
      left_participants.push_back(lanelet::Participants::VehicleEmergency);
    }
    if (msg_detail.latperm[1] == cav_msgs::TrafficControlDetail::PERMITTED ||
        msg_detail.latperm[1] == cav_msgs::TrafficControlDetail::PASSINGONLY)
    {
      right_participants = participants; 
    }
    else if (msg_detail.latperm[1] == cav_msgs::TrafficControlDetail::EMERGENCYONLY)
    {
      right_participants.push_back(lanelet::Participants::VehicleEmergency);
    }

    gf_ptr->regulatory_element_ = std::make_shared<lanelet::PassingControlLine>(lanelet::PassingControlLine::buildData(
      lanelet::utils::getId(), pcl_bounds, left_participants, right_participants));
  }

  cav_msgs::TrafficControlSchedule msg_schedule = msg_v01.params.schedule;
  
  // Get schedule
  for (auto daily_schedule : msg_schedule.between)
  {
    gf_ptr->schedules.push_back(GeofenceSchedule(msg_schedule.start,  
                                 msg_schedule.end,
                                 daily_schedule.begin,     
                                 daily_schedule.duration,
                                 msg_schedule.repeat.offset,
                                 msg_schedule.repeat.span,   
                                 msg_schedule.repeat.period));
  }
  
  return gf_ptr;
}

// currently only supports geofence message version 1: TrafficControlMessageV01 
void WMBroadcaster::geofenceCallback(const cav_msgs::TrafficControlMessage& geofence_msg)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  // quickly check if the id has been added
  if (geofence_msg.choice != cav_msgs::TrafficControlMessage::TCMV01)
    return;

  boost::uuids::uuid id;
  std::copy(geofence_msg.tcmV01.id.id.begin(), geofence_msg.tcmV01.id.id.end(), id.begin());
  if (checked_geofence_ids_.find(boost::uuids::to_string(id)) != checked_geofence_ids_.end())
    return;
  
  checked_geofence_ids_.insert(boost::uuids::to_string(id));
  auto gf_ptr = geofenceFromMsg(geofence_msg.tcmV01);
  if (gf_ptr->affected_parts_.size() == 0)
  {
    ROS_WARN_STREAM("There is no applicable component in map for the new geofence message received by WMBroadcaster with id: " << gf_ptr->id_);
    return;
  }
  scheduler_.addGeofence(gf_ptr);  // Add the geofence to the scheduler
  ROS_INFO_STREAM("New geofence message received by WMBroadcaster with id: " << gf_ptr->id_);
  
};

void WMBroadcaster::geoReferenceCallback(const std_msgs::String& geo_ref)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  base_map_georef_ = geo_ref.data;
}

void WMBroadcaster::setMaxLaneWidth(double max_lane_width)
{
  max_lane_width_ = max_lane_width;
}

// currently only supports geofence message version 1: TrafficControlMessageV01 
lanelet::ConstLaneletOrAreas WMBroadcaster::getAffectedLaneletOrAreas(const cav_msgs::TrafficControlMessageV01& tcmV01)
{
  if (!current_map_)
  {
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map is not loaded to the WMBroadcaster"));
  }
  if (base_map_georef_ == "")
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map has empty proj string loaded as georeference. Therefore, WMBroadcaster failed to\n ") +
                                          std::string("get transformation between the geofence and the map"));

  PJ* geofence_in_map_proj = proj_create_crs_to_crs(PJ_DEFAULT_CTX, tcmV01.geometry.proj.c_str(), base_map_georef_.c_str(), NULL);
  
  // convert all geofence points into our map's frame
  std::vector<lanelet::Point3d> gf_pts;
  for (auto pt : tcmV01.geometry.nodes)
  {
    PJ_COORD c {{pt.x, pt.y, 0, 0}}; // z is not currently used
    PJ_COORD c_out;
    c_out = proj_trans(geofence_in_map_proj, PJ_FWD, c);
    gf_pts.push_back(lanelet::Point3d{current_map_->pointLayer.uniqueId(), c_out.xyz.x, c_out.xyz.y});
  }

  // Logic to detect which part is affected

  std::unordered_set<lanelet::Lanelet> affected_lanelets;
  for (int idx = 0; idx < gf_pts.size(); idx ++)
  {
    std::unordered_set<lanelet::Lanelet> possible_lanelets;
    // get nearest few nearest llts within max_lane_width_
    // which actually house this geofence_point
    auto searchFunc = [&](const lanelet::BoundingBox2d& lltBox, const lanelet::Lanelet& llt) 
    {
      bool should_stop_searching = boost::geometry::distance(gf_pts[idx].basicPoint2d(), llt.polygon2d()) > max_lane_width_;
      if (!should_stop_searching && boost::geometry::within(gf_pts[idx].basicPoint2d(), llt.polygon2d()))
      {
        possible_lanelets.insert(llt);
      }
      return should_stop_searching;
    };

    // this call updates possible_lanelets
    current_map_->laneletLayer.nearestUntil(gf_pts[idx], searchFunc);

    // among these llts, filter the ones that are on same direction as the geofence using routing
    if (idx + 1 == gf_pts.size()) // we only check this for the last gf_pt after saving everything
    {
      std::unordered_set<lanelet::Lanelet> filtered = filterSuccessorLanelets(possible_lanelets, affected_lanelets);
      affected_lanelets.insert(filtered.begin(), filtered.end());
      break;
    } 

    // check if each lines connecting end points of the llt is crossing with the line connecting current and next gf_pts
    for (auto llt: possible_lanelets)
    {
      lanelet::BasicLineString2d gf_dir_line({gf_pts[idx].basicPoint2d(), gf_pts[idx+1].basicPoint2d()});
      lanelet::BasicLineString2d llt_boundary({(llt.leftBound2d().end() -1)->basicPoint2d(), (llt.rightBound2d().end() - 1)->basicPoint2d()});
      
      // record the llts that are on the same dir
      if (boost::geometry::intersects(llt_boundary, gf_dir_line))
      {
        affected_lanelets.insert(llt);
      }
      // check condition if two geofence points are in one lanelet then check matching direction and record it also
      else if (boost::geometry::within(gf_pts[idx+1].basicPoint2d(), llt.polygon2d()) && 
              affected_lanelets.find(llt) == affected_lanelets.end())
      { 
        lanelet::BasicPoint2d median({((llt.leftBound2d().end() - 1)->basicPoint2d().x() + (llt.rightBound2d().end() - 1)->basicPoint2d().x())/2 , 
                                      ((llt.leftBound2d().end() - 1)->basicPoint2d().y() + (llt.rightBound2d().end() - 1)->basicPoint2d().y())/2});
        // turn into vectors
        Eigen::Vector2d vec_to_median(median);
        Eigen::Vector2d vec_to_gf_start(gf_pts[idx].basicPoint2d());
        Eigen::Vector2d vec_to_gf_end(gf_pts[idx + 1].basicPoint2d());

        // Get vector from start to external point
        Eigen::Vector2d start_to_median = vec_to_median - vec_to_gf_start;

        // Get vector from start to end point
        Eigen::Vector2d start_to_end = vec_to_gf_end - vec_to_gf_start;

        // Get angle between both vectors
        double interior_angle = carma_wm::geometry::getAngleBetweenVectors(start_to_median, start_to_end);
        // Save the lanelet if the direction of two points inside aligns with that of the lanelet
        if (interior_angle < M_PI_2 && interior_angle >= 0) affected_lanelets.insert(llt);
      }
    }
  }
  
  // Currently only returning lanelet, but this could be expanded to LanelerOrArea compound object 
  // by implementing non-const version of that LaneletOrArea
  lanelet::ConstLaneletOrAreas affected_parts;
  affected_parts.insert(affected_parts.end(), affected_lanelets.begin(), affected_lanelets.end());
  return affected_parts;
}

// helper function that filters successor lanelets of root_lanelets from possible_lanelets
std::unordered_set<lanelet::Lanelet> WMBroadcaster::filterSuccessorLanelets(const std::unordered_set<lanelet::Lanelet>& possible_lanelets, const std::unordered_set<lanelet::Lanelet>& root_lanelets)
{
  std::unordered_set<lanelet::Lanelet> filtered_lanelets;
  // we utilize routes to filter llts that are overlapping but not connected
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules_car = lanelet::traffic_rules::TrafficRulesFactory::create(
  lanelet::traffic_rules::CarmaUSTrafficRules::Location, lanelet::Participants::VehicleCar);
  lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*current_map_, *traffic_rules_car);
  
  // as this is the last lanelet 
  // we have to filter the llts that are only geometrically overlapping yet not connected to prev llts
  for (auto recorded_llt: root_lanelets)
  {
    for (auto following_llt: map_graph->following(recorded_llt, false))
    {
      auto mutable_llt = current_map_->laneletLayer.get(following_llt.id());
      auto it = possible_lanelets.find(mutable_llt);
      if (it != possible_lanelets.end())
      {
        filtered_lanelets.insert(mutable_llt);
      }
    }
  }
  return filtered_lanelets;
}

void WMBroadcaster::addRegulatoryComponent(std::shared_ptr<Geofence> gf_ptr)
{
  // First loop is to save the relation between element and regulatory element
  // so that we can add back the old one after geofence deactivates
  for (auto el: gf_ptr->affected_parts_)
  {
    for (auto regem : el.regulatoryElements())
    {
      ROS_WARN_STREAM("remove: looking at regem: " << regem->id() << " type: "<< regem->attribute(lanelet::AttributeName::Subtype).value() << ", el: " << el.id());
      // extra condition for PassingControlLine
      if (regem->attribute(lanelet::AttributeName::Subtype).value() == lanelet::PassingControlLine::RuleName && el.isLanelet())
      {
        lanelet::PassingControlLinePtr pcl =  std::dynamic_pointer_cast<lanelet::PassingControlLine>(current_map_->regulatoryElementLayer.get(regem->id()));
        // if this geofence's pcl doesn't match with the lanelets current bound side
        bool should_process_this_regem = false;
        for (auto control_line : pcl->controlLine())
        {
          if (control_line.id() == el.lanelet()->leftBound2d().id() && gf_ptr->pcl_affects_left_ ||
          control_line.id() == el.lanelet()->rightBound2d().id() && gf_ptr->pcl_affects_right_)
          {
            ROS_WARN_STREAM("Found a regem ought to process");
            should_process_this_regem = true;
            break;
          }
        }
        if (!should_process_this_regem) continue;
      }

      if (regem->attribute(lanelet::AttributeName::Subtype).value() == gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value())
      {
        lanelet::RegulatoryElementPtr nonconst_regem = current_map_->regulatoryElementLayer.get(regem->id());
        gf_ptr->prev_regems_.push_back(std::make_pair(el.id(), nonconst_regem));
        gf_ptr->remove_list_.push_back(std::make_pair(el.id(), nonconst_regem));
        current_map_->remove(current_map_->laneletLayer.get(el.lanelet()->id()), nonconst_regem);
        ROS_WARN_STREAM("remove: JUST REMOVED at regem: " << nonconst_regem->id() << " type: "<< nonconst_regem->attribute(lanelet::AttributeName::Subtype).value() << ", el: " << el.id());
        
      }
    }
  }
  
  // this loop is also kept separately because previously we assumed 
  // there was existing regem, but this handles changes to all of the elements
  for (auto el: gf_ptr->affected_parts_)
  {
    // update it with new regem
    if (gf_ptr->regulatory_element_->id() != lanelet::InvalId)
    {
      current_map_->update(current_map_->laneletLayer.get(el.id()), gf_ptr->regulatory_element_);
      gf_ptr->update_list_.push_back(std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>(el.id(), gf_ptr->regulatory_element_));
    }
  }
  
}

void WMBroadcaster::addBackRegulatoryComponent(std::shared_ptr<Geofence> gf_ptr)
{
  // First loop is to remove the relation between element and regulatory element that this geofence added initially
  
  for (auto el: gf_ptr->affected_parts_)
  {
    for (auto regem : el.regulatoryElements())
    {
      ROS_WARN_STREAM("remove: looking at regem: " << regem->id() << " type: "<< regem->attribute(lanelet::AttributeName::Subtype).value() << ", el: " << el.id());
      // extra condition for PassingControlLine
      if (regem->attribute(lanelet::AttributeName::Subtype).value() == lanelet::PassingControlLine::RuleName && el.isLanelet())
      {
        lanelet::PassingControlLinePtr pcl =  std::dynamic_pointer_cast<lanelet::PassingControlLine>(current_map_->regulatoryElementLayer.get(regem->id()));
        // if this geofence's pcl doesn't match with the lanelets current bound side
        bool should_process_this_regem = false;
        for (auto control_line : pcl->controlLine())
        {
          if (control_line.id() == el.lanelet()->leftBound2d().id() && gf_ptr->pcl_affects_left_ ||
          control_line.id() == el.lanelet()->rightBound2d().id() && gf_ptr->pcl_affects_right_)
          {
            ROS_WARN_STREAM("Found a regem ought to process");
            should_process_this_regem = true;
            break;
          }
        }
        if (!should_process_this_regem) continue;
      }

      if (regem->attribute(lanelet::AttributeName::Subtype).value() ==  gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value())
      {
        auto nonconst_regem = current_map_->regulatoryElementLayer.get(regem->id());
        gf_ptr->remove_list_.push_back(std::make_pair(el.id(), nonconst_regem));
        current_map_->remove(current_map_->laneletLayer.get(el.lanelet()->id()), nonconst_regem);
      }
    }
  }
  
  // As this gf received is the first gf that was sent in through addGeofence,
  // we have prev speed limit information inside it to put them back
  for (auto pair : gf_ptr->prev_regems_)
  {
    if (pair.second->attribute(lanelet::AttributeName::Subtype).value() ==  gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value())
    {
      current_map_->update(current_map_->laneletLayer.get(pair.first), pair.second);
      gf_ptr->update_list_.push_back(pair);
    } 
  }
  
}

void WMBroadcaster::addGeofence(std::shared_ptr<Geofence> gf_ptr)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_INFO_STREAM("Adding active geofence to the map with geofence id: " << gf_ptr->id_);
  
  // Process the geofence object to populate update remove lists
  addGeofenceHelper(gf_ptr);
  
  // publish
  autoware_lanelet2_msgs::MapBin gf_msg;
  auto send_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_));
  carma_wm::toBinMsg(send_data, &gf_msg);
  map_update_pub_(gf_msg);
  
};

void WMBroadcaster::removeGeofence(std::shared_ptr<Geofence> gf_ptr)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_INFO_STREAM("Removing inactive geofence from the map with geofence id: " << gf_ptr->id_);
  
  // Process the geofence object to populate update remove lists
  removeGeofenceHelper(gf_ptr);

  // publish
  autoware_lanelet2_msgs::MapBin gf_msg_revert;
  auto send_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_));
  
  carma_wm::toBinMsg(send_data, &gf_msg_revert);
  map_update_pub_(gf_msg_revert);
};

// helper function that detects the type of geofence and delegates
void WMBroadcaster::addGeofenceHelper(std::shared_ptr<Geofence> gf_ptr)
{
  // resetting the information inside geofence
  gf_ptr->remove_list_ = {};
  gf_ptr->update_list_ = {};

  // TODO: Logic to determine what type of geofence goes here in the future
  // currently only speedchange is available, so it is assumed that
  addRegulatoryComponent(gf_ptr);
}

// helper function that detects the type of geofence and delegates
void WMBroadcaster::removeGeofenceHelper(std::shared_ptr<Geofence> gf_ptr)
{
  // again, TODO: Logic to determine what type of geofence goes here in the future
  // reset the info inside geofence
  gf_ptr->remove_list_ = {};
  gf_ptr->update_list_ = {};
  addBackRegulatoryComponent(gf_ptr);
  // as all changes are reverted back, we no longer need prev_regems
  gf_ptr->prev_regems_ = {};
}

}  // namespace carma_wm_ctrl
