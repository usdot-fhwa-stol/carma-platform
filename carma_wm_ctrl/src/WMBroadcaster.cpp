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

#include <functional>
#include <mutex>
#include <carma_wm_ctrl/WMBroadcaster.hpp>
#include <carma_wm_ros2/Geometry.hpp>
#include <carma_wm_ros2/MapConformer.hpp>
#include <autoware_lanelet2_ros2_interface/utility/message_conversion.hpp>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <type_traits>
#include <carma_perception_msgs/msg/check_active_geofence.hpp>
#include <lanelet2_core/primitives/Polygon.h>
#include <proj.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_core/Forward.h>
#include <autoware_lanelet2_ros2_interface/utility/utilities.hpp>
#include <algorithm>
#include <limits>
#include <carma_wm_ros2/Geometry.hpp>
#include <math.h>
#include <boost/date_time/date_defs.hpp>
#include "RoutingGraphAccessor.hpp"

namespace carma_wm_ctrl
{
using std::placeholders::_1;

WMBroadcaster::WMBroadcaster(const PublishMapCallback& map_pub, const PublishMapUpdateCallback& map_update_pub, const PublishCtrlRequestCallback& control_msg_pub,
const PublishActiveGeofCallback& active_pub, std::shared_ptr<carma_ros2_utils::timers::TimerFactory> timer_factory, const PublishMobilityOperationCallback& tcm_ack_pub)
  : map_pub_(map_pub), map_update_pub_(map_update_pub), control_msg_pub_(control_msg_pub), active_pub_(active_pub), scheduler_(timer_factory), tcm_ack_pub_(tcm_ack_pub)
{
  scheduler_.onGeofenceActive(std::bind(&WMBroadcaster::addGeofence, this, _1));
  scheduler_.onGeofenceInactive(std::bind(&WMBroadcaster::removeGeofence, this, _1));
  std::bind(&WMBroadcaster::routeCallbackMessage, this, _1);
};

void WMBroadcaster::baseMapCallback(autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg)
{
  std::lock_guard<std::mutex> guard(map_mutex_);

  static bool firstCall = true;
  // This function should generally only ever be called one time so log a warning if it occurs multiple times
  if (firstCall)
  {
    firstCall = false;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "WMBroadcaster::baseMapCallback called for first time with new map message");
  }
  else
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "WMBroadcaster::baseMapCallback called multiple times in the same node");
  }

  lanelet::LaneletMapPtr new_map(new lanelet::LaneletMap);
  lanelet::LaneletMapPtr new_map_to_change(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*map_msg, new_map);
  lanelet::utils::conversion::fromBinMsg(*map_msg, new_map_to_change);

  base_map_ = new_map;  // Store map
  current_map_ = new_map_to_change; // broadcaster makes changes to this

  lanelet::MapConformer::ensureCompliance(base_map_, config_limit);     // Update map to ensure it complies with expectations
  lanelet::MapConformer::ensureCompliance(current_map_, config_limit);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Building routing graph for base map");

  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules_car = lanelet::traffic_rules::TrafficRulesFactory::create(
  lanelet::traffic_rules::CarmaUSTrafficRules::Location, participant_);
  current_routing_graph_ = lanelet::routing::RoutingGraph::build(*current_map_, *traffic_rules_car);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Done building routing graph for base map");

  // Publish map
  current_map_version_ += 1; // Increment the map version. It should always start from 1 for the first map
  
  autoware_lanelet2_msgs::msg::MapBin compliant_map_msg;

  // Populate the routing graph message
  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Creating routing graph message.");

  auto readable_graph = std::static_pointer_cast<RoutingGraphAccessor>(current_routing_graph_);

  compliant_map_msg.routing_graph = readable_graph->routingGraphToMsg(participant_);
  compliant_map_msg.has_routing_graph = true;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Done creating routing graph message.");

  lanelet::utils::conversion::toBinMsg(current_map_, &compliant_map_msg);
  compliant_map_msg.map_version = current_map_version_;
  map_pub_(compliant_map_msg);
};

/*!
  * \brief Populates the schedules member of the geofence object from given TrafficControlMessageV01 message
  * \param gf_ptr geofence pointer
  * \param msg_v01 TrafficControlMessageV01 (geofence msg)
  */
void WMBroadcaster::addScheduleFromMsg(std::shared_ptr<Geofence> gf_ptr, const carma_v2x_msgs::msg::TrafficControlMessageV01& msg_v01)
{
  // Handle schedule processing
  carma_v2x_msgs::msg::TrafficControlSchedule msg_schedule = msg_v01.params.schedule;
  double ns_to_sec = 1.0e9;
  auto clock_type = scheduler_.getClockType();
  
  rclcpp::Time end_time = {msg_schedule.end, clock_type};
  if (!msg_schedule.end_exists) {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "No end time for geofence, using rclcpp::Time::max()");
    end_time = {rclcpp::Time::max(), clock_type}; // If there is no end time use the max time
  }

  // If the days of the week are specified then convert them to the boost format. 
  GeofenceSchedule::DayOfTheWeekSet week_day_set = { 0, 1, 2, 3, 4, 5, 6 }; // Default to all days  0==Sun to 6==Sat
  if (msg_schedule.dow_exists) {
   // sun (6), mon (5), tue (4), wed (3), thu (2), fri (1), sat (0)
   week_day_set.clear();
   for (size_t i = 0; i < msg_schedule.dow.dow.size(); i++) {
     if (msg_schedule.dow.dow[i] == 1) {
      switch(i) {
          case 6: // sun
            week_day_set.emplace(boost::gregorian::greg_weekday::weekday_enum::Sunday);
            break;
          case 5: // mon
            week_day_set.emplace(boost::gregorian::greg_weekday::weekday_enum::Monday);
            break;
          case 4: // tue
            week_day_set.emplace(boost::gregorian::greg_weekday::weekday_enum::Tuesday);
            break;
          case 3: // wed
            week_day_set.emplace(boost::gregorian::greg_weekday::weekday_enum::Wednesday);
            break;
          case 2: // thur
            week_day_set.emplace(boost::gregorian::greg_weekday::weekday_enum::Thursday);
            break;
          case 1: // fri
            week_day_set.emplace(boost::gregorian::greg_weekday::weekday_enum::Friday);
            break;
          case 0: // sat
            week_day_set.emplace(boost::gregorian::greg_weekday::weekday_enum::Saturday);
            break;
          default:
            throw std::invalid_argument("Unrecognized weekday value: " + std::to_string(i));
      }
     }
   }
  }

  if (msg_schedule.between_exists) {

    for (auto daily_schedule : msg_schedule.between)
    {
      if (msg_schedule.repeat_exists) {
        gf_ptr->schedules.push_back(GeofenceSchedule({msg_schedule.start, clock_type},  
                                    end_time,
                                    rclcpp::Duration(daily_schedule.begin),     
                                    rclcpp::Duration(daily_schedule.duration),
                                    rclcpp::Duration(msg_schedule.repeat.offset),
                                    rclcpp::Duration(msg_schedule.repeat.span),   
                                    rclcpp::Duration(msg_schedule.repeat.period),
                                    week_day_set));
      } else {
        gf_ptr->schedules.push_back(GeofenceSchedule({msg_schedule.start, clock_type},  
                                  end_time,
                                  rclcpp::Duration(daily_schedule.begin),     
                                  rclcpp::Duration(daily_schedule.duration),
                                  rclcpp::Duration(0.0), // No offset
                                  rclcpp::Duration(daily_schedule.duration) - rclcpp::Duration(daily_schedule.begin),   // Compute schedule portion end time
                                  rclcpp::Duration(daily_schedule.duration) - rclcpp::Duration(daily_schedule.begin),   // No repetition so same as portion end time
                                  week_day_set));         
      }

    }
  }
  else {
    if (msg_schedule.repeat_exists) {
      gf_ptr->schedules.push_back(GeofenceSchedule({msg_schedule.start, clock_type},  
                                  end_time,
                                  rclcpp::Duration(0.0),     
                                  rclcpp::Duration(86400.0e9), // 24 hr daily application
                                  rclcpp::Duration(msg_schedule.repeat.offset),
                                  rclcpp::Duration(msg_schedule.repeat.span),   
                                  rclcpp::Duration(msg_schedule.repeat.period),
                                  week_day_set)); 
    } else {
      gf_ptr->schedules.push_back(GeofenceSchedule({msg_schedule.start, clock_type},  
                                  end_time,
                                  rclcpp::Duration(0.0),     
                                  rclcpp::Duration(86400.0e9), // 24 hr daily application
                                  rclcpp::Duration(0.0),     // No offset
                                  rclcpp::Duration(86400.0e9), // Applied for full lenth of 24 hrs
                                  rclcpp::Duration(86400.0e9), // No repetition
                                  week_day_set)); 
    }
   
  }
}


std::vector<std::shared_ptr<Geofence>> WMBroadcaster::geofenceFromMapMsg(std::shared_ptr<Geofence> gf_ptr, const carma_v2x_msgs::msg::MapData& map_msg)
{
  std::vector<std::shared_ptr<Geofence>> updates_to_send;
  std::vector<std::shared_ptr<lanelet::SignalizedIntersection>> intersections;
  std::vector<std::shared_ptr<lanelet::CarmaTrafficSignal>> traffic_signals;

  auto sim_copy = std::make_shared<carma_wm::SignalizedIntersectionManager>(*sim_);

  sim_->createIntersectionFromMapMsg(intersections, traffic_signals, map_msg, current_map_, current_routing_graph_);

  if (*sim_ == *sim_copy) // if no change
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), ">>> Detected no change from previous, ignoring duplicate message! with gf id: " << gf_ptr->id_);
    return {};
  }

  for (auto intersection : intersections)
  {
    auto update = std::make_shared<Geofence>();
    update->id_ = boost::uuids::random_generator()();
    update->label_ = carma_wm_ctrl::MAP_MSG_INTERSECTION;
    update->regulatory_element_ = intersection;
    for (auto llt : intersection->getEntryLanelets())
    {
      update->affected_parts_.push_back(llt);
    }
    updates_to_send.push_back(update);
  }

  for (auto signal : traffic_signals)
  {
    auto update = std::make_shared<Geofence>();
    update->id_ = boost::uuids::random_generator()();
    update->label_ = carma_wm_ctrl::MAP_MSG_TF_SIGNAL;
    update->regulatory_element_ = signal;
    for (auto llt : signal->getControlStartLanelets())
    {
      update->affected_parts_.push_back(llt);
    }
    updates_to_send.push_back(update);
  }
  
  return updates_to_send;
}

void WMBroadcaster::geofenceFromMsg(std::shared_ptr<Geofence> gf_ptr, const carma_v2x_msgs::msg::TrafficControlMessageV01& msg_v01)
{
  bool detected_workzone_signal = msg_v01.package.label_exists && msg_v01.package.label.find("SIG_WZ") != std::string::npos;
  carma_v2x_msgs::msg::TrafficControlDetail msg_detail = msg_v01.params.detail;

  // Get ID
  std::copy(msg_v01.id.id.begin(), msg_v01.id.id.end(), gf_ptr->id_.begin());
  
  gf_ptr->gf_pts = getPointsInLocalFrame(msg_v01);

  gf_ptr->affected_parts_ = getAffectedLaneletOrAreas(gf_ptr->gf_pts);

  if (gf_ptr->affected_parts_.size() == 0) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "There is no applicable component in map for the new geofence message received by WMBroadcaster with id: " << gf_ptr->id_);
    return; // Return empty geofence list
  }

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
  // - digital speed limit, passing control line, digital minimum gap, region access rule, and series of workzone related messages
  lanelet::Velocity sL;
  
  
  if (msg_detail.choice == carma_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE) 
  {  
    //Acquire speed limit information from TafficControlDetail msg
    sL = lanelet::Velocity(msg_detail.maxspeed * lanelet::units::MPS()); 
    std::string reason = "";
    if (msg_v01.package.label_exists)
      reason = msg_v01.package.label;

    if(config_limit > 0_mph && config_limit < 80_mph && config_limit < sL)//Accounting for the configured speed limit, input zero when not in use
        sL = config_limit;
    //Ensure Geofences do not provide invalid speed limit data (exceed predetermined maximum value)
    // @SONAR_STOP@
    if(sL > 80_mph )
    {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Digital maximum speed limit is invalid. Value capped at max speed limit."); //Output warning message
    sL = 80_mph; //Cap the speed limit to the predetermined maximum value

    }
    if(sL < 0_mph)
    {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Digital  speed limit is invalid. Value set to 0mph.");
      sL = 0_mph;
    }// @SONAR_START@
    
    gf_ptr->regulatory_element_ = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::utils::getId(), 
                                        sL, affected_llts, affected_areas, participantsChecker(msg_v01), reason));
  }
  if (msg_detail.choice == carma_v2x_msgs::msg::TrafficControlDetail::MINSPEED_CHOICE) 
  {
    //Acquire speed limit information from TafficControlDetail msg
    sL = lanelet::Velocity(msg_detail.minspeed * lanelet::units::MPS());
    if(config_limit > 0_mph && config_limit < 80_mph)//Accounting for the configured speed limit, input zero when not in use
        sL = config_limit;

    std::string reason = "";
    if (msg_v01.package.label_exists)
      reason = msg_v01.package.label;

    //Ensure Geofences do not provide invalid speed limit data 
    // @SONAR_STOP@
    if(sL > 80_mph )
    {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Digital speed limit is invalid. Value capped at max speed limit.");
    sL = 80_mph;
    }
    if(sL < 0_mph)
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Digital  speed limit is invalid. Value set to 0mph.");
      sL = 0_mph;
    }// @SONAR_START@
    gf_ptr->regulatory_element_ = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::utils::getId(), 
                                        sL, affected_llts, affected_areas, participantsChecker(msg_v01), reason));
  }
  if (msg_detail.choice == carma_v2x_msgs::msg::TrafficControlDetail::LATPERM_CHOICE || msg_detail.choice == carma_v2x_msgs::msg::TrafficControlDetail::LATAFFINITY_CHOICE)
  {
    addPassingControlLineFromMsg(gf_ptr, msg_v01, affected_llts);
  }
  if (msg_detail.choice == carma_v2x_msgs::msg::TrafficControlDetail::MINHDWY_CHOICE) 
  {

    double min_gap = (double)msg_detail.minhdwy;

    if(min_gap < 0)
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Digital min gap is invalid. Value set to 0 meter.");
      min_gap = 0;
    }
    addRegionMinimumGap(gf_ptr,msg_v01, min_gap, affected_llts, affected_areas);
  }

  if (detected_workzone_signal && msg_detail.choice != carma_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE) // if workzone message detected, save to cache to process later
  {
    gf_ptr->label_ = msg_v01.package.label; // to extract intersection, and signal group id
    if (msg_detail.choice == carma_v2x_msgs::msg::TrafficControlDetail::CLOSED_CHOICE && (msg_detail.closed == carma_v2x_msgs::msg::TrafficControlDetail::CLOSED ||
                                                                                msg_detail.closed == carma_v2x_msgs::msg::TrafficControlDetail::TAPERRIGHT ||
                                                                                msg_detail.closed == carma_v2x_msgs::msg::TrafficControlDetail::OPENRIGHT))
    {
      work_zone_geofence_cache_[msg_detail.closed] = gf_ptr;
    }
    else if (msg_detail.choice == carma_v2x_msgs::msg::TrafficControlDetail::DIRECTION_CHOICE && msg_detail.direction == carma_v2x_msgs::msg::TrafficControlDetail::REVERSE)
    {
      work_zone_geofence_cache_[WorkZoneSection::REVERSE] = gf_ptr;
    }
    return;
  }
  else if (msg_detail.choice == carma_v2x_msgs::msg::TrafficControlDetail::CLOSED_CHOICE && msg_detail.closed==carma_v2x_msgs::msg::TrafficControlDetail::CLOSED) // if stand-alone closed signal apart from Workzone
  {
    addRegionAccessRule(gf_ptr,msg_v01,affected_llts);
  }

  if (msg_detail.choice == carma_v2x_msgs::msg::TrafficControlDetail::RESTRICTED_CHOICE) {
    addRegionAccessRule(gf_ptr, msg_v01, affected_llts);
  }

  return;
}

std::shared_ptr<Geofence> WMBroadcaster::createWorkzoneGeofence(std::unordered_map<uint8_t, std::shared_ptr<Geofence>> work_zone_geofence_cache)
{
  std::shared_ptr<std::vector<lanelet::Lanelet>> parallel_llts = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());
  std::shared_ptr<std::vector<lanelet::Lanelet>> middle_opposite_lanelets = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());
  
  // Split existing lanelets and filter into parallel_llts and middle_opposite_lanelets
  preprocessWorkzoneGeometry(work_zone_geofence_cache, parallel_llts, middle_opposite_lanelets);

  // Create geofence and rest of the required lanelets along with traffic light for completing workzone area
  auto gf_ptr = createWorkzoneGeometry(work_zone_geofence_cache, parallel_llts->front(), parallel_llts->back(), middle_opposite_lanelets );

  // copy static info from the existing workzone
  gf_ptr->id_ = work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->id_; //using taperright's id as the whole geofence's id
  
  // schedule
  gf_ptr->schedules = work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->schedules; //using taperright's schedule as the whole geofence's schedule
  
  // erase cache now that it is processed
  for (auto pair : work_zone_geofence_cache)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Workzone geofence finished processing. Therefore following geofence id is being dropped from cache as it is processed as part of it: " << pair.second->id_);
  }
  work_zone_geofence_cache.clear();

  return gf_ptr;
}

std::shared_ptr<Geofence> WMBroadcaster::createWorkzoneGeometry(std::unordered_map<uint8_t, std::shared_ptr<Geofence>> work_zone_geofence_cache,  lanelet::Lanelet parallel_llt_front,  lanelet::Lanelet parallel_llt_back, 
                                                    std::shared_ptr<std::vector<lanelet::Lanelet>> middle_opposite_lanelets)
{
  auto gf_ptr = std::make_shared<Geofence>();

  //////////////////////////////
  //FRONT DIAGONAL LANELET
  //////////////////////////////

  lanelet::Lanelet front_llt_diag = createLinearInterpolatingLanelet(parallel_llt_front.leftBound3d().back(), parallel_llt_front.rightBound3d().back(), 
                                                                        middle_opposite_lanelets->back().rightBound3d().back(), middle_opposite_lanelets->back().leftBound3d().back());
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Created diag front_llt_diag id:" << front_llt_diag.id());
  for (auto regem : middle_opposite_lanelets->back().regulatoryElements()) //copy existing regem into the new llts
  {
    front_llt_diag.addRegulatoryElement(regem);
  }

  //////////////////////////////
  //BACK DIAGONAL LANELET
  //////////////////////////////

  lanelet::Lanelet back_llt_diag = createLinearInterpolatingLanelet(middle_opposite_lanelets->front().rightBound3d().front(),  middle_opposite_lanelets->front().leftBound3d().front(), 
                                                                     parallel_llt_back.leftBound3d().front(), parallel_llt_back.rightBound3d().front());
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Created back_llt_diag diag id:" << back_llt_diag.id());
  for (auto regem : parallel_llt_back.regulatoryElements()) //copy existing regem into the new llts
  {
    back_llt_diag.addRegulatoryElement(regem);
  }

  //////////////////////////////
  //MIDDLE LANELETS TO MATCH PARALLEL DIRECTION
  //////////////////////////////

  std::vector<lanelet::Lanelet> middle_llts;
  for (int i = middle_opposite_lanelets->size() - 1; i >= 0; i--) //do no use size_t as it might overflow
  {
    lanelet::Lanelet middle_llt (lanelet::utils::getId(), (*(middle_opposite_lanelets.get()))[i].rightBound3d().invert(), (*(middle_opposite_lanelets.get()))[i].leftBound3d().invert());
    for (auto regem : (*(middle_opposite_lanelets.get()))[i].regulatoryElements())
    {
      middle_llt.addRegulatoryElement(regem); //copy existing regem into the new llts
    }
    middle_llts.push_back(middle_llt);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Created matching direction of middle_llt id:" << middle_llt.id());
  }

  //////////////////////////////
  //ADD TF_LIGHT TO PARALLEL LANELET
  //////////////////////////////
  lanelet::LineString3d parallel_stop_ls(lanelet::utils::getId(), {parallel_llt_front.leftBound3d().back(), parallel_llt_front.rightBound3d().back()});
  // controlled lanelet:
  std::vector<lanelet::Lanelet> controlled_taper_right;

  controlled_taper_right.push_back(parallel_llt_front); //which has the light

  controlled_taper_right.push_back(front_llt_diag);

  controlled_taper_right.insert(controlled_taper_right.end(), middle_llts.begin(), middle_llts.end());

  controlled_taper_right.push_back(back_llt_diag);

  lanelet::CarmaTrafficSignalPtr tfl_parallel = std::make_shared<lanelet::CarmaTrafficSignal>(lanelet::CarmaTrafficSignal::buildData(lanelet::utils::getId(), {parallel_stop_ls}, {controlled_taper_right.front()}, {controlled_taper_right.back()})); 

  gf_ptr->traffic_light_id_lookup_.push_back({generate32BitId(work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->label_),tfl_parallel->id()});

  parallel_llt_front.addRegulatoryElement(tfl_parallel);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Created TF_LIGHT of Id: " << tfl_parallel->id() << ", to parallel_llt_front id:" << parallel_llt_front.id());

  //////////////////////////////
  //ADD TF_LIGHT TO OPPOSITE LANELET
  //////////////////////////////
  std::shared_ptr<std::vector<lanelet::Lanelet>> opposite_llts_with_stop_line = std::make_shared<std::vector<lanelet::Lanelet>>(std::vector<lanelet::Lanelet>());
  
  auto old_opposite_llts = splitOppositeLaneletWithPoint(opposite_llts_with_stop_line, work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->gf_pts.back().basicPoint2d(), 
                        current_map_->laneletLayer.get(work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->affected_parts_.back().lanelet().get().id()), error_distance_);
  
  lanelet::LineString3d opposite_stop_ls(lanelet::utils::getId(), {opposite_llts_with_stop_line->front().leftBound3d().back(), opposite_llts_with_stop_line->front().rightBound3d().back()});
  
  std::vector<lanelet::Lanelet> controlled_open_right;
  
  controlled_open_right.insert(controlled_open_right.end(), opposite_llts_with_stop_line->begin(), opposite_llts_with_stop_line->end());; //split lanelet one of which has light
  
  for (auto llt : work_zone_geofence_cache[WorkZoneSection::REVERSE]->affected_parts_)
  {
    controlled_open_right.push_back(current_map_->laneletLayer.get(llt.lanelet().get().id()));
  }

  lanelet::CarmaTrafficSignalPtr tfl_opposite = std::make_shared<lanelet::CarmaTrafficSignal>(lanelet::CarmaTrafficSignal::buildData(lanelet::utils::getId(), {opposite_stop_ls}, {controlled_open_right.front()}, {controlled_open_right.back()}));
  
  gf_ptr->traffic_light_id_lookup_.push_back({generate32BitId(work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->label_), tfl_opposite->id()});
  
  opposite_llts_with_stop_line->front().addRegulatoryElement(tfl_opposite);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Created TF_LIGHT of Id: " << tfl_opposite->id() << ", to opposite_llts_with_stop_line->front() id:" << opposite_llts_with_stop_line->front().id());

  //////////////////////////////
  //ADD ALL NEWLY CREATED LANELETS INTO GEOFENCE 
  //OBJECTS TO BE PROCESSED LATER BY SCHEDULER
  //////////////////////////////
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Added parallel_llt_front id:" << parallel_llt_front.id());
  gf_ptr->lanelet_additions_.push_back(parallel_llt_front);

  gf_ptr->lanelet_additions_.push_back(front_llt_diag);

  gf_ptr->lanelet_additions_.insert(gf_ptr->lanelet_additions_.end(), middle_llts.begin(), middle_llts.end());

  gf_ptr->lanelet_additions_.push_back(back_llt_diag);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Added parallel_llt_back id:" << parallel_llt_back.id());
  gf_ptr->lanelet_additions_.push_back(parallel_llt_back);

  gf_ptr->lanelet_additions_.insert(gf_ptr->lanelet_additions_.end(), opposite_llts_with_stop_line->begin(), opposite_llts_with_stop_line->end());;

  //////////////////////////////
  // ADD REGION_ACCESS_RULE REGEM TO THE OUTDATED LANELETS 
  // AS WELL AS LANELETS BEING BLOCKED BY WORKZONE GEOFENCE
  //////////////////////////////
    
  // fill information for paricipants and reason for blocking
  carma_v2x_msgs::msg::TrafficControlMessageV01 participants_and_reason_only;

  j2735_v2x_msgs::msg::TrafficControlVehClass participant; // sending all possible VEHICLE will be processed as they are not accessuble by regionAccessRule
  
  participant.vehicle_class =  j2735_v2x_msgs::msg::TrafficControlVehClass::MICROMOBILE;

  participants_and_reason_only.params.vclasses.push_back(participant);

  participant.vehicle_class =  j2735_v2x_msgs::msg::TrafficControlVehClass::BUS;

  participants_and_reason_only.params.vclasses.push_back(participant);

  participant.vehicle_class =  j2735_v2x_msgs::msg::TrafficControlVehClass::PASSENGER_CAR;

  participants_and_reason_only.params.vclasses.push_back(participant);

  participant.vehicle_class =  j2735_v2x_msgs::msg::TrafficControlVehClass::TWO_AXLE_SIX_TIRE_SINGLE_UNIT_TRUCK;

  participants_and_reason_only.params.vclasses.push_back(participant);

  participants_and_reason_only.package.label = "SIG_WZ";

  std::vector<lanelet::Lanelet> old_or_blocked_llts; // this is needed to addRegionAccessRule input signatures
  
  // from all affected parts, remove duplicate entries
  for (auto llt : work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->affected_parts_)
  {
    if (std::find(old_or_blocked_llts.begin(), old_or_blocked_llts.end(), llt) == old_or_blocked_llts.end())
    {
      gf_ptr->affected_parts_.push_back(current_map_->laneletLayer.get(llt.lanelet()->id()));
      old_or_blocked_llts.push_back(current_map_->laneletLayer.get(llt.lanelet()->id()));
    }
  }
  for (auto llt : work_zone_geofence_cache[WorkZoneSection::CLOSED]->affected_parts_)
  {
    if (std::find(old_or_blocked_llts.begin(), old_or_blocked_llts.end(), llt) == old_or_blocked_llts.end())
    {
      gf_ptr->affected_parts_.push_back(current_map_->laneletLayer.get(llt.lanelet()->id()));
      old_or_blocked_llts.push_back(current_map_->laneletLayer.get(llt.lanelet()->id()));
    }
  }
  for (auto llt : work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->affected_parts_)
  {
    if (std::find(old_or_blocked_llts.begin(), old_or_blocked_llts.end(), llt) == old_or_blocked_llts.end())
    {
      gf_ptr->affected_parts_.push_back(current_map_->laneletLayer.get(llt.lanelet()->id()));
      old_or_blocked_llts.push_back(current_map_->laneletLayer.get(llt.lanelet()->id()));
    }
  }
  
  gf_ptr->affected_parts_.push_back(old_opposite_llts[0]); // block old lanelet in the opposing lanelet that will be replaced with split lanelets that have trafficlight
  old_or_blocked_llts.push_back(old_opposite_llts[0]);

  // actual regulatory element adder
  addRegionAccessRule(gf_ptr, participants_and_reason_only, old_or_blocked_llts); 
  
  return gf_ptr;
}

void WMBroadcaster::setErrorDistance(double error_distance)
{
  error_distance_ = error_distance;
}

void WMBroadcaster::preprocessWorkzoneGeometry(std::unordered_map<uint8_t, std::shared_ptr<Geofence>> work_zone_geofence_cache, std::shared_ptr<std::vector<lanelet::Lanelet>> parallel_llts, std::shared_ptr<std::vector<lanelet::Lanelet>> opposite_llts)
{
  if (!current_map_ || current_map_->laneletLayer.size() == 0)
  {
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map is not loaded to the WMBroadcaster"));
  }
  ////////////////////////////////////
  /// PARALLEL FRONT (TAPERRIGHT side)
  ////////////////////////////////////
  std::vector <lanelet::Lanelet> new_taper_right_llts;
  auto taper_right_first_pt = work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->gf_pts.front().basicPoint2d();
  auto taper_right_first_llt = current_map_->laneletLayer.get(work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->affected_parts_.front().lanelet().get().id());

  new_taper_right_llts = splitLaneletWithPoint({taper_right_first_pt}, taper_right_first_llt, error_distance_);
  double check_dist_tpr = lanelet::geometry::distance2d(taper_right_first_llt.centerline2d().front().basicPoint2d(), taper_right_first_pt);

  // if no splitting happened and taperright's first point is close to starting boundary, we need to create duplicate of previous lanelet of TAPERRIGHT's first lanelet
  // to match the output expected of this function as if split happened
  if (new_taper_right_llts.size() == 1 && check_dist_tpr <= error_distance_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Creating duplicate lanelet of 'previous lanelet' due to TAPERRIGHT using entire lanelet...");
    auto previous_lanelets = current_routing_graph_->previous(work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->affected_parts_.front().lanelet().get());
    if (previous_lanelets.empty()) //error if bad match
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Workzone area starts from lanelet with no previous lanelet (Id : " << work_zone_geofence_cache[WorkZoneSection::TAPERRIGHT]->affected_parts_.front().lanelet().get().id()
                      << ". This case is rare and not supported at the moment.");
      return;
    }

    // get previous lanelet of affected part of TAPERRIGHT (doesn't matter which previous, as the new lanelet will only be duplicate anyways)
    auto prev_lanelet_to_copy  = current_map_->laneletLayer.get(previous_lanelets.front().id());

    // parallel_llts will have a copy of `prev_lanelet_to_copy` with new id to be used as part of workzone area
    new_taper_right_llts = splitLaneletWithPoint({prev_lanelet_to_copy.centerline2d().back()}, prev_lanelet_to_copy, error_distance_);
  }
  parallel_llts->insert(parallel_llts->end(), new_taper_right_llts.begin(), new_taper_right_llts.end());
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Finished TAPERRIGHT processing of size: " << new_taper_right_llts.size());

  //////////////////////////////////
  /// PARALLEL BACK (OPENRIGHT side)
  //////////////////////////////////
  std::vector <lanelet::Lanelet> new_open_right_llts;
  auto open_right_last_pt = work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->gf_pts.back().basicPoint2d();
  auto open_right_last_llt = current_map_->laneletLayer.get(work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->affected_parts_.back().lanelet().get().id());

  new_open_right_llts = splitLaneletWithPoint({open_right_last_pt}, open_right_last_llt, error_distance_);
  double check_dist_opr = lanelet::geometry::distance2d(open_right_last_llt.centerline2d().back().basicPoint2d(), open_right_last_pt);

  // if no splitting happened, we need to create duplicate of next lanelet of OPENRIGHT's last lanelet
  // to match the output expected of this function as if split happened
  if (new_open_right_llts.size() == 1 && check_dist_opr <= error_distance_) 
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Creating duplicate lanelet of 'next lanelet' due to OPENRIGHT using entire lanelet...");
    auto next_lanelets = current_routing_graph_->following(work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->affected_parts_.back().lanelet().get());
    if (next_lanelets.empty()) //error if bad match
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Workzone area ends at lanelet with no following lanelet (Id : " << work_zone_geofence_cache[WorkZoneSection::OPENRIGHT]->affected_parts_.back().lanelet().get().id()
                      << ". This case is rare and not supported at the moment.");
      return;
    }
    
    // get next lanelet of affected part of OPENRIGHT (doesn't matter which next, as the new lanelet will only be duplicate anyways)
    auto next_lanelet_to_copy  = current_map_->laneletLayer.get(next_lanelets.front().id());

    // parallel_llts will have a copy of `next_lanelet_to_copy` with new id to be used as part of workzone area
    new_open_right_llts = splitLaneletWithPoint({next_lanelet_to_copy.centerline2d().back()}, next_lanelet_to_copy, error_distance_);
  }
  parallel_llts->insert(parallel_llts->end(), new_open_right_llts.begin(), new_open_right_llts.end());
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Finished OPENRIGHT processing of size: " << new_open_right_llts.size());

  ////////////////////
  /// HANDLE MID HERE
  ////////////////////

  auto reverse_back_llts = carma_wm::query::getLaneletsFromPoint(current_map_,  work_zone_geofence_cache[WorkZoneSection::REVERSE]->gf_pts.back().basicPoint2d());
  auto reverse_back_llt = reverse_back_llts[0];
  auto reverse_front_llts = carma_wm::query::getLaneletsFromPoint(current_map_,  work_zone_geofence_cache[WorkZoneSection::REVERSE]->gf_pts.front().basicPoint2d());
  auto reverse_front_llt = reverse_front_llts[0];

  if (reverse_back_llt.id() == reverse_front_llt.id()) //means there is only 1 middle lanelet, which needs to be split into 3 lanelets
  {
    std::vector<lanelet::Lanelet> temp_llts;
    temp_llts = splitLaneletWithPoint({work_zone_geofence_cache[WorkZoneSection::REVERSE]->gf_pts.front().basicPoint2d(), 
                                          work_zone_geofence_cache[WorkZoneSection::REVERSE]->gf_pts.back().basicPoint2d()},
                                          current_map_->laneletLayer.get(work_zone_geofence_cache[WorkZoneSection::REVERSE]->affected_parts_.front().lanelet().get().id()), error_distance_);
    
    if (temp_llts.size() < 2) // if there is only 1 lanelet
    {
      // we found what we want, so return
      opposite_llts->insert(opposite_llts->end(), temp_llts.begin(), temp_llts.end());
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Ended preprocessWorkzoneGeometry with opposite_llts.size()" << opposite_llts->size() << ", and parallel_llts.size()" << parallel_llts->size());
      return;
    }
    else if (temp_llts.size() == 2) // determine which 
    {
      // back gap is bigger than front's, we should lose front lanelet from the two
      if (lanelet::geometry::distance2d(work_zone_geofence_cache[WorkZoneSection::REVERSE]->gf_pts.back().basicPoint2d(), reverse_front_llt.centerline2d().back().basicPoint2d()) >
        lanelet::geometry::distance2d(work_zone_geofence_cache[WorkZoneSection::REVERSE]->gf_pts.front().basicPoint2d(), reverse_front_llt.centerline2d().front().basicPoint2d()))
        {
          opposite_llts->push_back(temp_llts.back());
        }
        else
        {
          opposite_llts->push_back(temp_llts.front());
        }
    }
    else if (temp_llts.size() == 3) // leave only middle lanelets from 3
    {
      opposite_llts->insert(opposite_llts->end(), temp_llts.begin() + 1, temp_llts.end()- 1);
    }
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Finished REVERSE processing of size: " << opposite_llts->size() << " from original of 1 REVERSE lanelet size");
  }
  else //if there are two or more lanelets
  {
    /// OPPOSITE FRONT (OPENRIGHT side) 
    auto reverse_first_pt = work_zone_geofence_cache[WorkZoneSection::REVERSE]->gf_pts.front().basicPoint2d();
    auto reverse_first_llt = current_map_->laneletLayer.get(work_zone_geofence_cache[WorkZoneSection::REVERSE]->affected_parts_.front().lanelet().get().id());
    std::vector<lanelet::Lanelet> temp_opposite_front_llts;
    temp_opposite_front_llts = splitLaneletWithPoint( {reverse_first_pt}, reverse_first_llt, error_distance_);
    if (temp_opposite_front_llts.size() > 1)
    {
      opposite_llts->insert(opposite_llts->end(), temp_opposite_front_llts.begin() + 1, temp_opposite_front_llts.end());
    }
    else
    {
      opposite_llts->insert(opposite_llts->end(), temp_opposite_front_llts.begin(), temp_opposite_front_llts.end());
    }
    /// Fill in the middle part of middle lanelets
    if (work_zone_geofence_cache[WorkZoneSection::REVERSE]->affected_parts_.size() > 2)
    {
      for (int i = 1; i <  work_zone_geofence_cache[WorkZoneSection::REVERSE]->affected_parts_.size() - 1; i ++)
      {
        opposite_llts->push_back(current_map_->laneletLayer.get(work_zone_geofence_cache[WorkZoneSection::REVERSE]->affected_parts_[i].id()));
      }
    }
    /// OPPOSITE BACK (TAPERRIGHT side)
    
    auto reverse_last_pt = work_zone_geofence_cache[WorkZoneSection::REVERSE]->gf_pts.back().basicPoint2d();
    auto reverse_last_llt = current_map_->laneletLayer.get(work_zone_geofence_cache[WorkZoneSection::REVERSE]->affected_parts_.back().lanelet().get().id());
    std::vector<lanelet::Lanelet> temp_opposite_back_llts;
    temp_opposite_back_llts = splitLaneletWithPoint({reverse_last_pt}, reverse_last_llt, error_distance_);
    if (temp_opposite_back_llts.size() > 1)
    {
      opposite_llts->insert(opposite_llts->end(), temp_opposite_back_llts.begin(), temp_opposite_back_llts.end()- 1);
    }
    else
    {
      opposite_llts->insert(opposite_llts->end(), temp_opposite_back_llts.begin(), temp_opposite_back_llts.end());
    }
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Finished REVERSE processing of size: " << opposite_llts->size() << " from original of more than one REVERSE lanelet size");
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Ended preprocessWorkzoneGeometry with opposite_llts.size()" << opposite_llts->size() << ", and parallel_llts.size()" << parallel_llts->size());
}

std::vector<lanelet::Lanelet> WMBroadcaster::splitLaneletWithPoint(const std::vector<lanelet::BasicPoint2d>& input_pts, const lanelet::Lanelet& input_llt, double error_distance)
{
  // get ratio of this point and split
  std::vector<lanelet::Lanelet> parallel_llts;
  double llt_downtrack = carma_wm::geometry::trackPos(input_llt, input_llt.centerline().back().basicPoint2d()).downtrack;
  std::vector<double> ratios;

  for (auto pt : input_pts)
  {
    double point_downtrack = carma_wm::geometry::trackPos(input_llt, pt).downtrack;
    double point_downtrack_ratio = point_downtrack / llt_downtrack;
    ratios.push_back(point_downtrack_ratio);
  }
  
  auto new_parallel_llts = splitLaneletWithRatio(ratios, input_llt, error_distance);

  parallel_llts.insert(parallel_llts.end(),new_parallel_llts.begin(), new_parallel_llts.end());
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "splitLaneletWithPoint returning lanelets size: " << parallel_llts.size());
  return parallel_llts;
}

lanelet::Lanelets WMBroadcaster::splitOppositeLaneletWithPoint(std::shared_ptr<std::vector<lanelet::Lanelet>> opposite_llts, const lanelet::BasicPoint2d& input_pt, const lanelet::Lanelet& input_llt, double error_distance)
{
  // get ratio of this point and split
  auto point_downtrack = carma_wm::geometry::trackPos(input_llt, input_pt).downtrack;
  auto point_downtrack_ratio = point_downtrack / carma_wm::geometry::trackPos(input_llt, input_llt.centerline().back().basicPoint2d()).downtrack;
  
  // get opposing lanelets and split
  auto opposing_llts = carma_wm::query::nonConnectedAdjacentLeft(current_map_, input_pt);
  
  if (opposing_llts.empty())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "WMBroadcaster was not able to find opposing lane for given point in geofence related to Work Zone! Returning");
    return {};
  }

  auto new_llts_opposite = splitLaneletWithRatio({1 - point_downtrack_ratio}, opposing_llts[0], error_distance);
  opposite_llts->insert(opposite_llts->begin(),new_llts_opposite.begin(), new_llts_opposite.end());
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "splitOppositeLaneletWithPoint returning lanelets size: " << opposite_llts->size());
  return opposing_llts;
}

std::vector<lanelet::Lanelet> WMBroadcaster::splitLaneletWithRatio(std::vector<double> ratios, lanelet::Lanelet input_lanelet, double error_distance) const
{
  if (!current_map_ || current_map_->laneletLayer.size() == 0)
  {
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map is not loaded to the WMBroadcaster"));
  }
  if (ratios.empty())
  {
    throw lanelet::InvalidInputError(std::string("Ratios list is empty! Cannot split"));
  }

  std::vector<lanelet::Lanelet> created_llts;

  std::sort(ratios.begin(), ratios.end());
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "splitLaneletWithRatio evaluating input ratios of size: " << ratios.size());

  ratios.push_back(1.0); //needed to complete the loop

  int left_ls_size = input_lanelet.leftBound2d().size();
  int right_ls_size = input_lanelet.rightBound2d().size();

  int left_next_pt_idx = 0;
  int left_prev_pt_idx = 0;
  int right_next_pt_idx = 0;
  int right_prev_pt_idx = 0;
  for (int i = 0 ; i < ratios.size(); i ++)
  {
    left_next_pt_idx = std::round(ratios[i] * (left_ls_size - 1));
    right_next_pt_idx = std::round(ratios[i] * (right_ls_size - 1));
    // check if edge ratios are too close to any boundaries. if so, skip
    if (lanelet::geometry::distance2d(input_lanelet.leftBound2d().front().basicPoint2d(), input_lanelet.leftBound2d()[left_next_pt_idx].basicPoint2d()) <= error_distance)
    {
      // assuming both linestrings have roughly the same number of points and
      // assuming distance between 0th and index-th points are small enough we can approximate the curve between them as a line:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Ratio: " << ratios[i] << ", is too close to the lanelet's front boundary! Therefore, ignoring... Allowed error_distance: " << error_distance << ", Distance: " 
                        << lanelet::geometry::distance2d(input_lanelet.leftBound2d().front().basicPoint2d(), input_lanelet.leftBound2d()[left_next_pt_idx].basicPoint2d()));
      continue;
    } 
    if (lanelet::geometry::distance2d(input_lanelet.leftBound2d().back().basicPoint2d(), input_lanelet.leftBound2d()[left_next_pt_idx].basicPoint2d()) <= error_distance)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Ratio: " << ratios[i] << ", is too close to the lanelet's back boundary! Therefore, ignoring... Allowed error_distance: " << error_distance << ", Distance: " 
                  << lanelet::geometry::distance2d(input_lanelet.leftBound2d().back().basicPoint2d(), input_lanelet.leftBound2d()[left_next_pt_idx].basicPoint2d()));

      left_next_pt_idx = left_ls_size - 1;
      right_next_pt_idx = right_ls_size - 1;
    }
    // create lanelet
    std::vector<lanelet::Point3d> left_pts;
    left_pts.insert(left_pts.end(), current_map_->laneletLayer.get(input_lanelet.id()).leftBound3d().begin() + left_prev_pt_idx, current_map_->laneletLayer.get(input_lanelet.id()).leftBound3d().begin() + left_next_pt_idx + 1);

    lanelet::LineString3d left_ls(lanelet::utils::getId(), left_pts, current_map_->laneletLayer.get(input_lanelet.id()).leftBound3d().attributes());  

    std::vector<lanelet::Point3d> right_pts;
    right_pts.insert(right_pts.end(), current_map_->laneletLayer.get(input_lanelet.id()).rightBound3d().begin() + right_prev_pt_idx, current_map_->laneletLayer.get(input_lanelet.id()).rightBound3d().begin() + right_next_pt_idx + 1);
    
    lanelet::LineString3d right_ls(lanelet::utils::getId(), right_pts);  

    lanelet::Lanelet new_llt (lanelet::utils::getId(), left_ls, right_ls, current_map_->laneletLayer.get(input_lanelet.id()).rightBound3d().attributes());
    
    for (auto regem : current_map_->laneletLayer.get(input_lanelet.id()).regulatoryElements()) //copy existing regem into new llts
    {
      new_llt.addRegulatoryElement(current_map_->regulatoryElementLayer.get(regem->id()));
    }
    
    left_prev_pt_idx = left_next_pt_idx;
    right_prev_pt_idx = right_next_pt_idx;
    
    created_llts.push_back(new_llt);
    
    // Detected the end already. Exiting now
    if (left_prev_pt_idx == left_ls_size - 1 || right_prev_pt_idx == right_ls_size - 1)
    {
      break;
    }
    
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "splitLaneletWithRatio returning lanelets size: " << created_llts.size());

  return created_llts;
}


void WMBroadcaster::addPassingControlLineFromMsg(std::shared_ptr<Geofence> gf_ptr, const carma_v2x_msgs::msg::TrafficControlMessageV01& msg_v01, const std::vector<lanelet::Lanelet>& affected_llts) const
{
  carma_v2x_msgs::msg::TrafficControlDetail msg_detail;
  msg_detail = msg_v01.params.detail;
  // Get affected bounds
  lanelet::LineStrings3d pcl_bounds;
  if (msg_detail.lataffinity == carma_v2x_msgs::msg::TrafficControlDetail::LEFT)
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
  std::vector<std::string> left_participants;
  std::vector<std::string> right_participants;
  std::vector<std::string> participants=participantsChecker(msg_v01);

  // Create the pcl depending on the allowed passing control direction, left, right, or both
  if (msg_detail.latperm[0] == carma_v2x_msgs::msg::TrafficControlDetail::PERMITTED ||
      msg_detail.latperm[0] == carma_v2x_msgs::msg::TrafficControlDetail::PASSINGONLY)
  {
    left_participants = participants; 
  }
  else if (msg_detail.latperm[0] == carma_v2x_msgs::msg::TrafficControlDetail::EMERGENCYONLY)
  {
    left_participants.push_back(lanelet::Participants::VehicleEmergency);
  }
  if (msg_detail.latperm[1] == carma_v2x_msgs::msg::TrafficControlDetail::PERMITTED ||
      msg_detail.latperm[1] == carma_v2x_msgs::msg::TrafficControlDetail::PASSINGONLY)
  {
    right_participants = participants; 
  }
  else if (msg_detail.latperm[1] == carma_v2x_msgs::msg::TrafficControlDetail::EMERGENCYONLY)
  {
    right_participants.push_back(lanelet::Participants::VehicleEmergency);
  }

  gf_ptr->regulatory_element_ = std::make_shared<lanelet::PassingControlLine>(lanelet::PassingControlLine::buildData(
    lanelet::utils::getId(), pcl_bounds, left_participants, right_participants));
}

void WMBroadcaster::addRegionAccessRule(std::shared_ptr<Geofence> gf_ptr, const carma_v2x_msgs::msg::TrafficControlMessageV01& msg_v01, const std::vector<lanelet::Lanelet>& affected_llts) const
{
  const std::string& reason = msg_v01.package.label;
  gf_ptr->label_ = msg_v01.package.label;
  auto regulatory_element = std::make_shared<lanelet::RegionAccessRule>(lanelet::RegionAccessRule::buildData(lanelet::utils::getId(),affected_llts,{},invertParticipants(participantsChecker(msg_v01)), reason));

  if(!regulatory_element->accessable(lanelet::Participants::VehicleCar) || !regulatory_element->accessable(lanelet::Participants::VehicleTruck )) 
  {
    gf_ptr->invalidate_route_=true;
  }
  else
  {
    gf_ptr->invalidate_route_=false;
  }
  gf_ptr->regulatory_element_ = regulatory_element;
}

void WMBroadcaster::addRegionMinimumGap(std::shared_ptr<Geofence> gf_ptr,  const carma_v2x_msgs::msg::TrafficControlMessageV01& msg_v01, double min_gap, const std::vector<lanelet::Lanelet>& affected_llts, const std::vector<lanelet::Area>& affected_areas) const
{
  auto regulatory_element = std::make_shared<lanelet::DigitalMinimumGap>(lanelet::DigitalMinimumGap::buildData(lanelet::utils::getId(), 
                                        min_gap, affected_llts, affected_areas,participantsChecker(msg_v01) ));
  
  gf_ptr->regulatory_element_ = regulatory_element;
}

std::vector<std::string> WMBroadcaster::participantsChecker(const carma_v2x_msgs::msg::TrafficControlMessageV01& msg_v01) const
{
  std::vector<std::string> participants;
  for (j2735_v2x_msgs::msg::TrafficControlVehClass participant : msg_v01.params.vclasses)
  {
    // Currently j2735_v2x_msgs::msg::TrafficControlVehClass::RAIL is not supported
    if (participant.vehicle_class == j2735_v2x_msgs::msg::TrafficControlVehClass::ANY)
    {
      participants = {lanelet::Participants::Vehicle, lanelet::Participants::Pedestrian, lanelet::Participants::Bicycle};
      break;
    }
    else if (participant.vehicle_class == j2735_v2x_msgs::msg::TrafficControlVehClass::PEDESTRIAN)
    {
      participants.push_back(lanelet::Participants::Pedestrian);
    }
    else if (participant.vehicle_class == j2735_v2x_msgs::msg::TrafficControlVehClass::BICYCLE)
    {
      participants.push_back(lanelet::Participants::Bicycle);
    }
    else if (participant.vehicle_class == j2735_v2x_msgs::msg::TrafficControlVehClass::MICROMOBILE ||
              participant.vehicle_class == j2735_v2x_msgs::msg::TrafficControlVehClass::MOTORCYCLE)
    {
      participants.push_back(lanelet::Participants::VehicleMotorcycle);
    }
    else if (participant.vehicle_class == j2735_v2x_msgs::msg::TrafficControlVehClass::BUS)
    {
      participants.push_back(lanelet::Participants::VehicleBus);
    }
    else if (participant.vehicle_class == j2735_v2x_msgs::msg::TrafficControlVehClass::LIGHT_TRUCK_VAN ||
            participant.vehicle_class == j2735_v2x_msgs::msg::TrafficControlVehClass::PASSENGER_CAR)
    {
      participants.push_back(lanelet::Participants::VehicleCar);
    }
    else if (8<= participant.vehicle_class && participant.vehicle_class <= 16) // Truck enum definition range from 8-16 currently
    {
      participants.push_back(lanelet::Participants::VehicleTruck);
    }
  }
  // combine to single vehicle type if possible, otherwise pass through
  return  combineParticipantsToVehicle(participants);
}

std::vector<std::string> WMBroadcaster::invertParticipants(const std::vector<std::string>& input_participants) const
{
  std::vector<std::string> participants;

  if(std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::Pedestrian ) == input_participants.end()) participants.emplace_back(lanelet::Participants::Pedestrian);
  if(std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::Bicycle ) == input_participants.end()) participants.emplace_back(lanelet::Participants::Bicycle);
  if(std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::Vehicle ) == input_participants.end())
  {
    if(std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::VehicleMotorcycle)== input_participants.end()) participants.emplace_back(lanelet::Participants::VehicleMotorcycle);
    if(std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::VehicleBus)== input_participants.end()) participants.emplace_back(lanelet::Participants::VehicleBus);
    if(std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::VehicleCar)== input_participants.end()) participants.emplace_back(lanelet::Participants::VehicleCar);
    if(std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::VehicleTruck)== input_participants.end()) participants.emplace_back(lanelet::Participants::VehicleTruck);
  }
  return  participants;
}

std::vector<std::string> WMBroadcaster::combineParticipantsToVehicle(const std::vector<std::string>& input_participants) const
{
  std::vector<std::string> participants;

  if(std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::VehicleMotorcycle)!= input_participants.end() &&
      std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::VehicleBus) != input_participants.end() &&
      std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::VehicleCar) != input_participants.end() &&
      std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::VehicleTruck) != input_participants.end())
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Detected participants to cover all possible vehicle types");
    participants.emplace_back(lanelet::Participants::Vehicle);
  }
  else
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Not making any changes to the participants list");
    participants = input_participants;
  }

  return participants;
}

void WMBroadcaster::externalMapMsgCallback(carma_v2x_msgs::msg::MapData::UniquePtr map_msg)
{
  auto gf_ptr = std::make_shared<Geofence>();

  if (!current_map_ || current_map_->laneletLayer.size() == 0)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Map is not available yet. Skipping MAP msg");
    return;
  }

  // check if we have seen this message already
  bool up_to_date = false;
  if (sim_->intersection_id_to_regem_id_.size() == map_msg->intersections.size())
  {
    up_to_date = true;
    // check id of the intersection only
    for (auto intersection : map_msg->intersections)
    {
      if (sim_->intersection_id_to_regem_id_.find(intersection.id.id) == sim_->intersection_id_to_regem_id_.end())
      {
        up_to_date = false;
        break;
      }
    }
  }

  if(up_to_date)
  {
    return;
  }
    
  gf_ptr->map_msg_ = *map_msg;
  gf_ptr->msg_.package.label_exists = true;
  gf_ptr->msg_.package.label = "MAP_MSG";
  gf_ptr->id_ = boost::uuids::random_generator()(); 

  // create dummy traffic Control message to add instant activation schedule
  carma_v2x_msgs::msg::TrafficControlMessageV01 traffic_control_msg;

  // process schedule from message
  addScheduleFromMsg(gf_ptr, traffic_control_msg);
  
  scheduleGeofence(gf_ptr);

}

// currently only supports geofence message version 1: TrafficControlMessageV01 
void WMBroadcaster::geofenceCallback(carma_v2x_msgs::msg::TrafficControlMessage::UniquePtr geofence_msg)
{
  
  std::lock_guard<std::mutex> guard(map_mutex_);
  std::stringstream reason_ss;
  // quickly check if the id has been added
  if (geofence_msg->choice != carma_v2x_msgs::msg::TrafficControlMessage::TCMV01) {
    reason_ss << "Dropping received geofence for unsupported TrafficControl version: " << geofence_msg->choice;
    RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), reason_ss.str());
    pubTCMACK(geofence_msg->tcm_v01.reqid, geofence_msg->tcm_v01.msgnum, static_cast<int>(AcknowledgementStatus::REJECTED), reason_ss.str());
    return;
  }

  boost::uuids::uuid id;
  std::copy(geofence_msg->tcm_v01.id.id.begin(), geofence_msg->tcm_v01.id.id.end(), id.begin());
  if (checked_geofence_ids_.find(boost::uuids::to_string(id)) != checked_geofence_ids_.end()) { 
    reason_ss.str("");
    reason_ss << "Dropping received TrafficControl message with already handled id: " << boost::uuids::to_string(id);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), reason_ss.str());
    pubTCMACK(geofence_msg->tcm_v01.reqid, geofence_msg->tcm_v01.msgnum, static_cast<int>(AcknowledgementStatus::ACKNOWLEDGED), reason_ss.str());
    return;
  }

  // convert reqid to string check if it has been seen before
  boost::array<uint8_t, 16UL> req_id;
  for (auto i = 0; i < 8; i ++) req_id[i] = geofence_msg->tcm_v01.reqid.id[i];
  boost::uuids::uuid uuid_id;
  std::copy(req_id.begin(),req_id.end(), uuid_id.begin());
  std::string reqid = boost::uuids::to_string(uuid_id).substr(0, 8);
  // drop if the req has never been sent
  if (generated_geofence_reqids_.find(reqid) == generated_geofence_reqids_.end() && reqid.compare("00000000") != 0)
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "CARMA_WM_CTRL received a TrafficControlMessage with unknown TrafficControlRequest ID (reqid): " << reqid);
    return;
  }
    
  checked_geofence_ids_.insert(boost::uuids::to_string(id));

  auto gf_ptr = std::make_shared<Geofence>();

  gf_ptr->msg_ = geofence_msg->tcm_v01;

  try
  {
    // process schedule from message
    addScheduleFromMsg(gf_ptr, geofence_msg->tcm_v01);    
    scheduleGeofence(gf_ptr);
    reason_ss.str("");
    reason_ss << "Successfully processed TCM.";
    pubTCMACK(geofence_msg->tcm_v01.reqid, geofence_msg->tcm_v01.msgnum, static_cast<int>(AcknowledgementStatus::ACKNOWLEDGED), reason_ss.str());
  }
  catch(std::exception& ex)
  {
    reason_ss.str("");
    reason_ss << "Failed to process TCM. " << ex.what();
    pubTCMACK(geofence_msg->tcm_v01.reqid, geofence_msg->tcm_v01.msgnum, static_cast<int>(AcknowledgementStatus::REJECTED), reason_ss.str());
    throw; //rethrows the exception object
  }
};

void WMBroadcaster::scheduleGeofence(std::shared_ptr<carma_wm_ctrl::Geofence> gf_ptr)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Scheduling new geofence message received by WMBroadcaster with id: " << gf_ptr->id_);
  
  bool detected_workzone_signal = gf_ptr->msg_.package.label_exists && gf_ptr->msg_.package.label.find("SIG_WZ") != std::string::npos;
  
  carma_v2x_msgs::msg::TrafficControlDetail msg_detail = gf_ptr->msg_.params.detail;
  
  // create workzone specific extra speed geofence
  if (detected_workzone_signal && msg_detail.choice == carma_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE)
  {
    // duplicate the messages with inverted points to support new lanelets created from workzone
    // as carma-cloud currently does not support geofence points with direction opposite to that of the road

    auto gf_ptr_speed = std::make_shared<Geofence>();
    gf_ptr_speed->schedules = gf_ptr->schedules;

    carma_v2x_msgs::msg::TrafficControlMessageV01 duplicate_msg = gf_ptr->msg_;

    std::reverse(duplicate_msg.geometry.nodes.begin() + 1, duplicate_msg.geometry.nodes.end());
    double first_x = 0;
    double first_y = 0;

    // this fancy logic is needed as each node is expressed as an offset from the last one
    for (auto& pt: duplicate_msg.geometry.nodes)
    {
      first_x+= pt.x;
      first_y+= pt.y;
      pt.x = -1* pt.x;
      pt.y = -1* pt.y;
    }
    duplicate_msg.geometry.nodes[0].x = first_x;
    duplicate_msg.geometry.nodes[0].y = first_y;
    
    gf_ptr_speed->msg_ = duplicate_msg;
    scheduler_.addGeofence(gf_ptr_speed);
  }
  if (detected_workzone_signal && msg_detail.choice != carma_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE) // if workzone message detected, save to cache to process later
  {
    gf_ptr->label_ = gf_ptr->msg_.package.label; // to extract intersection, and signal group id
    if (msg_detail.choice == carma_v2x_msgs::msg::TrafficControlDetail::CLOSED_CHOICE && (msg_detail.closed == carma_v2x_msgs::msg::TrafficControlDetail::CLOSED ||
                                                                                msg_detail.closed == carma_v2x_msgs::msg::TrafficControlDetail::TAPERRIGHT ||
                                                                                msg_detail.closed == carma_v2x_msgs::msg::TrafficControlDetail::OPENRIGHT))
    {
      work_zone_geofence_cache_[msg_detail.closed] = gf_ptr;
    }
    else if (msg_detail.choice == carma_v2x_msgs::msg::TrafficControlDetail::DIRECTION_CHOICE && msg_detail.direction == carma_v2x_msgs::msg::TrafficControlDetail::REVERSE)
    {
      work_zone_geofence_cache_[WorkZoneSection::REVERSE] = gf_ptr;
    }
    if (work_zone_geofence_cache_.size() < WORKZONE_TCM_REQUIRED_SIZE)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Received 'SIG_WZ' signal. Waiting for the rest of the messages, returning for now...");
      return;
    }
  }

  scheduler_.addGeofence(gf_ptr);  // Add the geofence to the scheduler

}

void WMBroadcaster::geoReferenceCallback(std_msgs::msg::String::UniquePtr geo_ref)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  sim_->setTargetFrame(geo_ref->data);
  base_map_georef_ = geo_ref->data;
}

void WMBroadcaster::setMaxLaneWidth(double max_lane_width)
{
  sim_ = std::make_shared<carma_wm::SignalizedIntersectionManager>();
  
  max_lane_width_ = max_lane_width;
  sim_->setMaxLaneWidth(max_lane_width_);
}

void WMBroadcaster::setIntersectionCoordCorrection(const std::vector<int64_t>& intersection_ids_for_correction, const std::vector<double>& intersection_correction)
{
  if (intersection_correction.size() % 2 != 0 || intersection_ids_for_correction.size() != intersection_correction.size() / 2)
  {
    throw std::invalid_argument("Some of intersection coordinate correction parameters are not fully set!");
  }

  for (auto i = 0; i < intersection_correction.size(); i = i + 2)
  {
    sim_->intersection_coord_correction_[(uint16_t)intersection_ids_for_correction[i/2]].first =  intersection_correction[i]; //x
    sim_->intersection_coord_correction_[(uint16_t)intersection_ids_for_correction[i/2]].second = intersection_correction[i + 1]; //y
  }

}

void WMBroadcaster::setConfigSpeedLimit(double cL)
{
  /*Logic to change config_lim to Velocity value config_limit*/
  config_limit = lanelet::Velocity(cL * lanelet::units::MPH());
}

void WMBroadcaster::setConfigVehicleId(const std::string& vehicle_id){
  vehicle_id_ = vehicle_id;
}

void WMBroadcaster::setConfigACKPubTimes(int ack_pub_times){
  ack_pub_times_ = ack_pub_times;
}

void WMBroadcaster::setVehicleParticipationType(std::string participant)
{
  participant_ = participant;
}

std::string WMBroadcaster::getVehicleParticipationType()
{
  return participant_;
}

uint32_t WMBroadcaster::generate32BitId(const std::string& label)
{
  auto pos1 = label.find("INT_ID:") + 7;
  auto pos2 = label.find("SG_ID:") + 6;
  
  uint16_t intersection_id = std::stoi(label.substr(pos1 , 4));
  uint8_t signal_id = std::stoi(label.substr(pos2 , 3));

  return carma_wm::utils::get32BitId(intersection_id, signal_id);
}

// currently only supports geofence message version 1: TrafficControlMessageV01 
lanelet::Points3d WMBroadcaster::getPointsInLocalFrame(const carma_v2x_msgs::msg::TrafficControlMessageV01& tcm_v01)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Getting affected lanelets");
  if (!current_map_ || current_map_->laneletLayer.size() == 0)
  {
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map is not loaded to the WMBroadcaster"));
  }
  if (base_map_georef_ == "")
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map has empty proj string loaded as georeference. Therefore, WMBroadcaster failed to\n ") +
                                          std::string("get transformation between the geofence and the map"));

  // This next section handles the geofence projection conversion
  // The datum field is used to identify the frame for the provided referance lat/lon. 
  // This reference is then converted to the provided projection as a reference origin point
  // From the reference the message projection to map projection transformation is used to convert the nodes in the TrafficControlMessage
  std::string projection = tcm_v01.geometry.proj;
  std::string datum = tcm_v01.geometry.datum;
  if (datum.empty()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Datum field not populated. Attempting to use WGS84");
    datum = "WGS84";
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Projection field: " << projection);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Datum field: " << datum);
  
  std::string universal_frame = datum; //lat/long included in TCM is in this datum


  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Traffic Control heading provided: " << tcm_v01.geometry.heading << " System understanding is that this value will not affect the projection and is only provided for supporting derivative calculations.");
  
  // Create the resulting projection transformation
  PJ* universal_to_target = proj_create_crs_to_crs(PJ_DEFAULT_CTX, universal_frame.c_str(), projection.c_str(), nullptr);
  if (universal_to_target == nullptr) { // proj_create_crs_to_crs returns 0 when there is an error in the projection
    
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Failed to generate projection between geofence and map with error number: " <<  proj_context_errno(PJ_DEFAULT_CTX) 
      << " universal_frame: " << universal_frame << " projection: " << projection);

    return {}; // Ignore geofence if it could not be projected from universal to TCM frame
  }
  
  PJ* target_to_map = proj_create_crs_to_crs(PJ_DEFAULT_CTX, projection.c_str(), base_map_georef_.c_str(), nullptr);

  if (target_to_map == nullptr) { // proj_create_crs_to_crs returns 0 when there is an error in the projection
    
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Failed to generate projection between geofence and map with error number: " <<  proj_context_errno(PJ_DEFAULT_CTX) 
      << " target_to_map: " << target_to_map << " base_map_georef_: " << base_map_georef_);

    return {}; // Ignore geofence if it could not be projected into the map frame
  
  }
  
  // convert all geofence points into our map's frame
  std::vector<lanelet::Point3d> gf_pts;
  carma_v2x_msgs::msg::PathNode prev_pt;
  PJ_COORD c_init_latlong{{tcm_v01.geometry.reflat, tcm_v01.geometry.reflon, tcm_v01.geometry.refelv}};
  PJ_COORD c_init = proj_trans(universal_to_target, PJ_FWD, c_init_latlong);

  prev_pt.x = c_init.xyz.x;
  prev_pt.y =  c_init.xyz.y;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "In TCM's frame, initial Point X "<< prev_pt.x<<" Before conversion: Point Y "<< prev_pt.y );
  for (auto pt : tcm_v01.geometry.nodes)
  { 
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Before conversion in TCM frame: Point X "<< pt.x <<" Before conversion: Point Y "<< pt.y);

    PJ_COORD c {{prev_pt.x + pt.x, prev_pt.y + pt.y, 0, 0}}; // z is not currently used
    PJ_COORD c_out;
    c_out = proj_trans(target_to_map, PJ_FWD, c);

    gf_pts.push_back(lanelet::Point3d{current_map_->pointLayer.uniqueId(), c_out.xyz.x, c_out.xyz.y});
    prev_pt.x += pt.x;
    prev_pt.y += pt.y;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "After conversion in Map frame: Point X "<< gf_pts.back().x() <<" After conversion: Point Y "<< gf_pts.back().y());
  }
  
  // save the points converted to local map frame
  return gf_pts;
}

lanelet::ConstLaneletOrAreas WMBroadcaster::getAffectedLaneletOrAreas(const lanelet::Points3d& gf_pts)
{
  return carma_wm::query::getAffectedLaneletOrAreas(gf_pts, current_map_, current_routing_graph_, max_lane_width_);
}

/*!
  * \brief This is a helper function that returns true if the provided regem is marked to be changed by the geofence as there are
  *  usually multiple passing control lines are in the lanelet.
  * \param geofence_msg The ROS msg that contains geofence information
  * \param el The LaneletOrArea that houses the regem
  * \param regem The regulatoryElement that needs to be checked
  * NOTE: Currently this function only works on lanelets. It returns true if the regem is not passingControlLine or if the pcl should be changed.
  */
bool WMBroadcaster::shouldChangeControlLine(const lanelet::ConstLaneletOrArea& el,const lanelet::RegulatoryElementConstPtr& regem, std::shared_ptr<Geofence> gf_ptr) const
{
  // should change if if the regem is not a passing control line or area, which is not supported by this logic
  if (regem->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::PassingControlLine::RuleName) != 0 || !el.isLanelet())
  {
    return true;
  }
  
  lanelet::PassingControlLinePtr pcl =  std::dynamic_pointer_cast<lanelet::PassingControlLine>(current_map_->regulatoryElementLayer.get(regem->id()));
  // if this geofence's pcl doesn't match with the lanelets current bound side, return false as we shouldn't change
  bool should_change_pcl = false;
  for (auto control_line : pcl->controlLine())
  {
    if ((control_line.id() == el.lanelet()->leftBound2d().id() && gf_ptr->pcl_affects_left_) ||
        (control_line.id() == el.lanelet()->rightBound2d().id() && gf_ptr->pcl_affects_right_))
    {
      should_change_pcl = true;
      break;
    }
  }
  return should_change_pcl;
}

/*!
  * \brief This is a helper function that returns true if signal in the lanelet should be changed according to the records of signalizer intersection manager
           Used in managing multiple signal_groups in a single entry lanelet for example
  * \param el The LaneletOrArea that houses the regem
  * \param regem The regulatoryElement that needs to be checked
  * \param sim The signalized intersection manager that has records regems and corresponding lanelets
  * NOTE: Currently this function only works on lanelets. It returns true if the regem is not CarmaTrafficSignal or if the signal should be changed.
  */
bool WMBroadcaster::shouldChangeTrafficSignal(const lanelet::ConstLaneletOrArea& el,const lanelet::RegulatoryElementConstPtr& regem, std::shared_ptr<carma_wm::SignalizedIntersectionManager> sim) const
{
  // should change if the regem is not a CarmaTrafficSignal, which is not supported by this logic
  if (regem->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::CarmaTrafficSignal::RuleName) != 0 || !el.isLanelet() || !sim_)
  {
    return true;
  }
  
  lanelet::CarmaTrafficSignalPtr traffic_signal =  std::dynamic_pointer_cast<lanelet::CarmaTrafficSignal>(current_map_->regulatoryElementLayer.get(regem->id()));
  uint8_t signal_id = 0;
  
  for (auto it = sim->signal_group_to_traffic_light_id_.begin(); it != sim->signal_group_to_traffic_light_id_.end(); ++it) 
  {
    if (regem->id() == it->second)
    {
      signal_id = it->first;
    } 
  } 

  if (signal_id == 0) // doesn't exist in the record
    return true;
  
  if (sim->signal_group_to_entry_lanelet_ids_[signal_id].find(el.id()) != sim->signal_group_to_entry_lanelet_ids_[signal_id].end())
    return false; // signal group's entry lane is still part of the intersection, so don't change
  
  return true;
}

void WMBroadcaster::addRegulatoryComponent(std::shared_ptr<Geofence> gf_ptr) const
{
  // First loop is to save the relation between element and regulatory element
  // so that we can add back the old one after geofence deactivates
  for (auto el: gf_ptr->affected_parts_)
  {
    for (auto regem : el.regulatoryElements())
    {
      if (!shouldChangeControlLine(el, regem, gf_ptr) ||
        !shouldChangeTrafficSignal(el, regem, sim_))
        continue;

      if (regem->attribute(lanelet::AttributeName::Subtype).value() == gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value())
      {
        lanelet::RegulatoryElementPtr nonconst_regem = current_map_->regulatoryElementLayer.get(regem->id());
        gf_ptr->prev_regems_.push_back(std::make_pair(el.id(), nonconst_regem));
        gf_ptr->remove_list_.push_back(std::make_pair(el.id(), nonconst_regem));
        current_map_->remove(current_map_->laneletLayer.get(el.lanelet()->id()), nonconst_regem);
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
    } else {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Regulatory element with invalid id in geofence cannot be added to the map");
    }
  }
  


}

void WMBroadcaster::addBackRegulatoryComponent(std::shared_ptr<Geofence> gf_ptr) const
{
  // First loop is to remove the relation between element and regulatory element that this geofence added initially
  
  for (auto el: gf_ptr->affected_parts_)
  {
    for (auto regem : el.regulatoryElements())
    {
      if (!shouldChangeControlLine(el, regem, gf_ptr)) continue;

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
  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Adding active geofence to the map with geofence id: " << gf_ptr->id_);
  
  // if applying workzone geometry geofence, utilize workzone chache to create one 
  // also multiple map updates can be sent from one geofence object
  std::vector<std::shared_ptr<Geofence>> updates_to_send;

  bool detected_workzone_signal = gf_ptr->msg_.package.label_exists && gf_ptr->msg_.package.label.find("SIG_WZ") != std::string::npos;
  bool detected_map_msg_signal = gf_ptr->msg_.package.label_exists && gf_ptr->msg_.package.label.find("MAP_MSG") != std::string::npos;
  if (detected_workzone_signal && gf_ptr->msg_.params.detail.choice != carma_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE)
  {
    for (auto gf_cache_ptr : work_zone_geofence_cache_)
    {
      geofenceFromMsg(gf_cache_ptr.second, gf_cache_ptr.second->msg_);
    }
    updates_to_send.push_back(createWorkzoneGeofence(work_zone_geofence_cache_));
  }
  else if (detected_map_msg_signal)
  {
    updates_to_send = geofenceFromMapMsg(gf_ptr, gf_ptr->map_msg_);
  }
  else
  {
    geofenceFromMsg(gf_ptr, gf_ptr->msg_);
    updates_to_send.push_back(gf_ptr);
  }

  for (auto update : updates_to_send)
  {    
    // add marker to rviz
    tcm_marker_array_.markers.push_back(composeTCMMarkerVisualizer(update->gf_pts)); // create visualizer in rviz

    if (update->affected_parts_.empty())
      continue;

    // Process the geofence object to populate update remove lists
    addGeofenceHelper(update);
    
    if (!detected_map_msg_signal)
    {
      for (auto pair : update->update_list_) active_geofence_llt_ids_.insert(pair.first);
    }

    autoware_lanelet2_msgs::msg::MapBin gf_msg;
    
    // If the geofence invalidates the route graph then recompute the routing graph now that the map has been updated
    if (update->invalidate_route_) {

      RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Rebuilding routing graph after is was invalidated by geofence");

      lanelet::traffic_rules::TrafficRulesUPtr traffic_rules_car = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::traffic_rules::CarmaUSTrafficRules::Location, participant_);
      current_routing_graph_ = lanelet::routing::RoutingGraph::build(*current_map_, *traffic_rules_car);

      RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Done rebuilding routing graph after is was invalidated by geofence");

      // Populate routing graph structure
      RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Creating routing graph message");

      auto readable_graph = std::static_pointer_cast<RoutingGraphAccessor>(current_routing_graph_);

      gf_msg.routing_graph = readable_graph->routingGraphToMsg(participant_);
      gf_msg.has_routing_graph = true;

      RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Done creating routing graph message");
    }
    

    // Publish
    auto send_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(update->id_, update->update_list_, update->remove_list_, update->lanelet_additions_));
    send_data->traffic_light_id_lookup_ = update->traffic_light_id_lookup_;

    if (detected_map_msg_signal && updates_to_send.back() == update) // if last update
    {
      send_data->sim_ = *sim_;
    }

    carma_wm::toBinMsg(send_data, &gf_msg);
    update_count_++; // Update the sequence count for the geofence messages
    gf_msg.seq_id = update_count_;
    gf_msg.invalidates_route=update->invalidate_route_; 
    gf_msg.map_version = current_map_version_;
    map_update_pub_(gf_msg);
  }
  
}

void WMBroadcaster::removeGeofence(std::shared_ptr<Geofence> gf_ptr)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Removing inactive geofence from the map with geofence id: " << gf_ptr->id_);
  
  // Process the geofence object to populate update remove lists
  if (gf_ptr->affected_parts_.empty())
    return;

  removeGeofenceHelper(gf_ptr);

  for (auto pair : gf_ptr->remove_list_) active_geofence_llt_ids_.erase(pair.first);

  // publish
  autoware_lanelet2_msgs::msg::MapBin gf_msg_revert;
  auto send_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_, {}));

  if (gf_ptr->invalidate_route_) { // If a geofence initially invalidated the route it stands to reason its removal should as well

    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Rebuilding routing graph after is was invalidated by geofence removal");

    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules_car = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::traffic_rules::CarmaUSTrafficRules::Location, participant_
    );
    current_routing_graph_ = lanelet::routing::RoutingGraph::build(*current_map_, *traffic_rules_car);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Done rebuilding routing graph after is was invalidated by geofence removal");

    // Populate routing graph structure
    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Creating routing graph message for geofence removal");

    auto readable_graph = std::static_pointer_cast<RoutingGraphAccessor>(current_routing_graph_);

    gf_msg_revert.routing_graph = readable_graph->routingGraphToMsg(participant_);
    gf_msg_revert.has_routing_graph = true;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Done creating routing graph message for geofence removal");
  }
  
  carma_wm::toBinMsg(send_data, &gf_msg_revert);
  update_count_++; // Update the sequence count for geofence messages
  gf_msg_revert.seq_id = update_count_;
  gf_msg_revert.map_version = current_map_version_;

  map_update_pub_(gf_msg_revert);


}
  
carma_planning_msgs::msg::Route WMBroadcaster::getRoute()
{
  return current_route;
}

void  WMBroadcaster::routeCallbackMessage(carma_planning_msgs::msg::Route::UniquePtr route_msg)
{
 current_route = *route_msg;
 carma_v2x_msgs::msg::TrafficControlRequest cR; 
 cR =  controlRequestFromRoute(*route_msg);
 control_msg_pub_(cR);

}

carma_v2x_msgs::msg::TrafficControlRequest WMBroadcaster::controlRequestFromRoute(const carma_planning_msgs::msg::Route& route_msg, std::shared_ptr<j2735_v2x_msgs::msg::Id64b> req_id_for_testing)
{
  lanelet::ConstLanelets path; 

  if (!current_map_ || current_map_->laneletLayer.size() == 0)
  {
   // Return / log warning etc.
    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Value 'current_map_' does not exist.");
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map is not loaded to the WMBroadcaster"));

  }

  for(auto id : route_msg.route_path_lanelet_ids) 
  {
    auto laneLayer = current_map_->laneletLayer.get(id);
    path.push_back(laneLayer);
  }

  // update local copy
  route_path_ = path;
  
  if(path.size() == 0) throw lanelet::InvalidObjectStateError(std::string("No lanelets available in path."));

   /*logic to determine route bounds*/
  std::vector<lanelet::ConstLanelet> llt; 
  std::vector<lanelet::BoundingBox2d> pathBox; 
  double minX = std::numeric_limits<double>::max();
  double minY = std::numeric_limits<double>::max();
  double maxX = std::numeric_limits<double>::lowest();
  double maxY = std::numeric_limits<double>::lowest();

  while (path.size() != 0) //Continue until there are no more lanelet elements in path
  {
      llt.push_back(path.back()); //Add a lanelet to the vector
    

      pathBox.push_back(lanelet::geometry::boundingBox2d(llt.back())); //Create a bounding box of the added lanelet and add it to the vector


      if (pathBox.back().corner(lanelet::BoundingBox2d::BottomLeft).x() < minX)
        minX = pathBox.back().corner(lanelet::BoundingBox2d::BottomLeft).x(); //minimum x-value


      if (pathBox.back().corner(lanelet::BoundingBox2d::BottomLeft).y() < minY)
        minY = pathBox.back().corner(lanelet::BoundingBox2d::BottomLeft).y(); //minimum y-value



     if (pathBox.back().corner(lanelet::BoundingBox2d::TopRight).x() > maxX)
        maxX = pathBox.back().corner(lanelet::BoundingBox2d::TopRight).x(); //maximum x-value


     if (pathBox.back().corner(lanelet::BoundingBox2d::TopRight).y() > maxY)
        maxY = pathBox.back().corner(lanelet::BoundingBox2d::TopRight).y(); //maximum y-value


      path.pop_back(); //remove the added lanelet from path an reduce pack.size() by 1
  } //end of while loop
  

  std::string target_frame = base_map_georef_;
  if (target_frame.empty()) 
  {
   // Return / log warning etc.
    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Value 'target_frame' is empty.");
    throw lanelet::InvalidObjectStateError(std::string("Base georeference map may not be loaded to the WMBroadcaster"));

  }
  
  // Convert the minimum point to latlon
  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
  lanelet::BasicPoint3d localPoint;

  localPoint.x()= minX;
  localPoint.y()= minY;

  lanelet::GPSPoint gpsRoute = local_projector.reverse(localPoint); //If the appropriate library is included, the reverse() function can be used to convert from local xyz to lat/lon

  // Create a local transverse mercator frame at the minimum point to allow us to get east,north oriented bounds 
  std::string local_tmerc_enu_proj = "+proj=tmerc +datum=WGS84 +h_0=0 +lat_0=" + std::to_string(gpsRoute.lat) + " +lon_0=" + std::to_string(gpsRoute.lon);

  // Create transform from map frame to local transform mercator frame at bounds min point
  PJ* tmerc_proj = proj_create_crs_to_crs(PJ_DEFAULT_CTX, target_frame.c_str(), local_tmerc_enu_proj.c_str(), nullptr);
  
  if (tmerc_proj == nullptr) { // proj_create_crs_to_crs returns 0 when there is an error in the projection
    
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Failed to generate projection between request bounds frame and map with error number: " <<  proj_context_errno(PJ_DEFAULT_CTX) 
      << " MapProjection: " << target_frame << " Message Projection: " << local_tmerc_enu_proj);

    return {}; // Ignore geofence if it could not be projected into the map frame
  
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Before conversion: Top Left: ("<< minX <<", "<<maxY<<")");
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Before conversion: Top Right: ("<< maxX <<", "<<maxY<<")");
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Before conversion: Bottom Left: ("<< minX <<", "<<minY<<")");
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Before conversion: Bottom Right: ("<< maxX <<", "<<minY<<")");

  PJ_COORD pj_min {{minX, minY, 0, 0}}; // z is not currently used
  PJ_COORD pj_min_tmerc;
  PJ_COORD pj_max {{maxX, maxY, 0, 0}}; // z is not currently used
  PJ_COORD pj_max_tmerc;
  pj_min_tmerc = proj_trans(tmerc_proj, PJ_FWD, pj_min);
  pj_max_tmerc = proj_trans(tmerc_proj, PJ_FWD, pj_max);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "After conversion: MinPoint ( "<< pj_min_tmerc.xyz.x <<", " << pj_min_tmerc.xyz.y <<" )");
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "After conversion: MaxPoint ( "<< pj_max_tmerc.xyz.x <<", " << pj_max_tmerc.xyz.y <<" )");

  carma_v2x_msgs::msg::TrafficControlRequest cR; /*Fill the latitude value in message cB with the value of lat */
  carma_v2x_msgs::msg::TrafficControlBounds cB; /*Fill the longitude value in message cB with the value of lon*/
  
  cB.reflat = gpsRoute.lat;
  cB.reflon = gpsRoute.lon;

  cB.offsets[0].deltax = pj_max_tmerc.xyz.x - pj_min_tmerc.xyz.x; // Points in clockwise order min,min (lat,lon point) -> max,min -> max,max -> min,max
  cB.offsets[0].deltay = 0.0;                                     // calculating the offsets
  cB.offsets[1].deltax = pj_max_tmerc.xyz.x - pj_min_tmerc.xyz.x;
  cB.offsets[1].deltay = pj_max_tmerc.xyz.y - pj_min_tmerc.xyz.y;
  cB.offsets[2].deltax = 0.0;
  cB.offsets[2].deltay = pj_max_tmerc.xyz.y - pj_min_tmerc.xyz.y;

  tcr_polygon_ = composeTCRStatus(localPoint, cB, local_projector); // TCR polygon can be visualized in UI

  cB.oldest =rclcpp::Time(0.0,0.0, scheduler_.getClockType()); // TODO this needs to be set to 0 or an older value as otherwise this will filter out all controls
  
  cR.choice = carma_v2x_msgs::msg::TrafficControlRequest::TCRV01;
  
  // create 16 byte uuid
  boost::uuids::uuid uuid_id = boost::uuids::random_generator()(); 
  // take half as string
  std::string reqid = boost::uuids::to_string(uuid_id).substr(0, 8);
  std::string req_id_test = "12345678"; // TODO this is an extremely risky way of performing a unit test. This method needs to be refactored so that unit tests cannot side affect actual implementations
  generated_geofence_reqids_.insert(req_id_test);
  generated_geofence_reqids_.insert(reqid);


  // copy to reqid array
  boost::array<uint8_t, 16UL> req_id;
  std::copy(uuid_id.begin(),uuid_id.end(), req_id.begin());
  for (auto i = 0; i < 8; i ++)
  {
    cR.tcr_v01.reqid.id[i] = req_id[i];
    if (req_id_for_testing) req_id_for_testing->id[i] = req_id[i];
  }

  cR.tcr_v01.bounds.push_back(cB);
  
  return cR;

}

carma_v2x_msgs::msg::TrafficControlRequestPolygon WMBroadcaster::composeTCRStatus(const lanelet::BasicPoint3d& localPoint, const carma_v2x_msgs::msg::TrafficControlBounds& cB, const lanelet::projection::LocalFrameProjector& local_projector)
{
  carma_v2x_msgs::msg::TrafficControlRequestPolygon output;
  lanelet::BasicPoint3d local_point_tmp;

  int i = -1;
  while (i < 3) // three offsets; offsets.size() doesn't return accurate size
  {
    if (i == -1)
    {
      local_point_tmp.x() = localPoint.x();
      local_point_tmp.y() = localPoint.y();
    }
    else
    {
      local_point_tmp.x() = localPoint.x() + cB.offsets[i].deltax;;
      local_point_tmp.y() = localPoint.y() + cB.offsets[i].deltay;;
    }
    lanelet::GPSPoint gps_vertex = local_projector.reverse(local_point_tmp);
    
    carma_v2x_msgs::msg::Position3D gps_msg;
    gps_msg.elevation = gps_vertex.ele;
    gps_msg.latitude = gps_vertex.lat;
    gps_msg.longitude = gps_vertex.lon;
    output.polygon_list.push_back(gps_msg);

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "TCR Vertex Lat: "<< std::to_string(gps_vertex.lat));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "TCR Vertex Lon: "<<std::to_string(gps_vertex.lon));

    i++;
  }
  return output;
}

visualization_msgs::msg::Marker WMBroadcaster::composeTCMMarkerVisualizer(const std::vector<lanelet::Point3d>& input)
 {

         // create the marker msgs
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Time();
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = "route_visualizer";

        marker.scale.x = 0.65;
        marker.scale.y = 0.65;
        marker.scale.z = 0.65;
        marker.frame_locked = true;

        if (!tcm_marker_array_.markers.empty()) 
        {
        marker.id = tcm_marker_array_.markers.back().id + 1;
        }
        else
        {
        marker.id = 0;
        }
        marker.color.r = 0.0F;
        marker.color.g = 1.0F;
        marker.color.b = 0.0F;
        marker.color.a = 1.0F;

        for (int i = 0; i < input.size(); i++)
        {
            geometry_msgs::msg::Point temp_point;
            temp_point.x = input[i].x();
            temp_point.y = input[i].y();
            temp_point.z = 2; //to show up on top of the lanelet lines

            marker.points.push_back(temp_point);
        }

        return marker;
 }

double WMBroadcaster::distToNearestActiveGeofence(const lanelet::BasicPoint2d& curr_pos)
{
  std::lock_guard<std::mutex> guard(map_mutex_);

  if (!current_map_ || current_map_->laneletLayer.size() == 0) 
  {
    throw lanelet::InvalidObjectStateError(std::string("Lanelet map (current_map_) is not loaded to the WMBroadcaster"));
  }

  // filter only the lanelets in the route
  std::vector<lanelet::Id> active_geofence_on_route;
  for (auto llt : route_path_)
  {
    if (active_geofence_llt_ids_.find(llt.id()) != active_geofence_llt_ids_.end()) 
      active_geofence_on_route.push_back(llt.id());
  }
  // Get the lanelet of this point
  auto curr_lanelet = lanelet::geometry::findNearest(current_map_->laneletLayer, curr_pos, 1)[0].second;

  // Check if this point at least is actually within this lanelets
  if (!boost::geometry::within(curr_pos, curr_lanelet.polygon2d().basicPolygon()))
    throw std::invalid_argument("Given point is not within any lanelet");

  // get route distance (downtrack + cross_track) distances to every lanelets by their ids
  std::vector<double> route_distances;
  // and take abs of cross_track to add them to get route distance
  for (auto id: active_geofence_on_route)
  {
    carma_wm::TrackPos tp = carma_wm::geometry::trackPos(current_map_->laneletLayer.get(id), curr_pos);
    // downtrack needs to be negative for lanelet to be in front of the point, 
    // also we don't account for the lanelet that the vehicle is on
    if (tp.downtrack < 0 && id != curr_lanelet.id())
    {
      double dist = fabs(tp.downtrack) + fabs(tp.crosstrack);
      route_distances.push_back(dist);
    }
  }
  std::sort(route_distances.begin(), route_distances.end());

  if (route_distances.size() != 0 ) return route_distances[0];
  else return 0.0;

}
// helper function that detects the type of geofence and delegates
void WMBroadcaster::addGeofenceHelper(std::shared_ptr<Geofence> gf_ptr)
{
  // resetting the information inside geofence
  gf_ptr->remove_list_ = {};
  gf_ptr->update_list_ = {};

  // add additional lanelets that need to be added
  for (auto llt : gf_ptr->lanelet_additions_)
  {
    current_map_->add(llt);
  }

  // add trafficlight id mapping
  for (auto pair : gf_ptr->traffic_light_id_lookup_)
  {
    traffic_light_id_lookup_[pair.first] = pair.second;
  }

  // Logic to determine what type of geofence
  addRegulatoryComponent(gf_ptr);
}

// helper function that detects the type of geofence and delegates
void WMBroadcaster::removeGeofenceHelper(std::shared_ptr<Geofence> gf_ptr) const
{
  // Logic to determine what type of geofence
  // reset the info inside geofence
  gf_ptr->remove_list_ = {};
  gf_ptr->update_list_ = {};
  addBackRegulatoryComponent(gf_ptr);
  // as all changes are reverted back, we no longer need prev_regems
  gf_ptr->prev_regems_ = {};
}

void WMBroadcaster::currentLocationCallback(geometry_msgs::msg::PoseStamped::UniquePtr current_pos)
{
  if (current_map_ && current_map_->laneletLayer.size() != 0) {
    carma_perception_msgs::msg::CheckActiveGeofence check = checkActiveGeofenceLogic(*current_pos);
    active_pub_(check);//Publish
  } else {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Could not check active geofence logic because map was not loaded");
  }
}

bool WMBroadcaster::convertLightIdToInterGroupId(unsigned& intersection_id, unsigned& group_id, const lanelet::Id& lanelet_id)
{
  for (auto it = traffic_light_id_lookup_.begin(); it != traffic_light_id_lookup_.end(); ++it)
  {
    // Reverse of the logic for generating the lanelet_id. Reference function generate32BitId(const std::string& label)
    if (it -> second == lanelet_id)
    {
      group_id = (it -> first & 0xFF);
      intersection_id = (it -> first >> 8);
      return true;
    }
  }
  return false;
}

void WMBroadcaster::publishLightId()
{
  if (traffic_light_id_lookup_.empty())
  {
    return;
  }
  for(auto id : current_route.route_path_lanelet_ids) 
  {
    bool convert_success = false;
    unsigned intersection_id = 0;
    unsigned group_id = 0;
    auto route_lanelet= current_map_->laneletLayer.get(id);
    auto traffic_lights = route_lanelet.regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
    
    if (!traffic_lights.empty())
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Found Traffic Light Regulatory Element id: " << traffic_lights.front()->id());
      convert_success = convertLightIdToInterGroupId(intersection_id,group_id,  traffic_lights.front()->id());
    }

    if (!convert_success)
        continue; 

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Found Traffic Light with Intersection id: " << intersection_id << " Group id:" << group_id);
    bool id_exists = false;
    for (int idx = 0; idx < upcoming_intersection_ids_.data.size(); idx +2)
    {
      if (upcoming_intersection_ids_.data[idx] == intersection_id && upcoming_intersection_ids_.data[idx + 1] == group_id) //check if already there
      {
        id_exists = true;
        break;
      }
    }

    if (id_exists)
      continue;

    upcoming_intersection_ids_.data.push_back(static_cast<int>(intersection_id));
    upcoming_intersection_ids_.data.push_back(static_cast<int>(group_id));
  }
}
carma_perception_msgs::msg::CheckActiveGeofence WMBroadcaster::checkActiveGeofenceLogic(const geometry_msgs::msg::PoseStamped& current_pos)
{

  if (!current_map_ || current_map_->laneletLayer.size() == 0) 
  {
    throw lanelet::InvalidObjectStateError(std::string("Lanelet map 'current_map_' is not loaded to the WMBroadcaster"));
  }

  // Store current position values to be compared to geofence boundary values
  double current_pos_x = current_pos.pose.position.x;
  double current_pos_y = current_pos.pose.position.y;

  
  

  lanelet::BasicPoint2d curr_pos;
  curr_pos.x() = current_pos_x;
  curr_pos.y() = current_pos_y;

  carma_perception_msgs::msg::CheckActiveGeofence outgoing_geof; //message to publish
  double next_distance = 0 ; //Distance to next geofence

  if (active_geofence_llt_ids_.size() <= 0 ) 
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "No active geofence llt ids are loaded to the WMBroadcaster");
    return outgoing_geof;
  }

  // Obtain the closest lanelet to the vehicle's current position
  auto current_llt = lanelet::geometry::findNearest(current_map_->laneletLayer, curr_pos, 1)[0].second;


  /* determine whether or not the vehicle's current position is within an active geofence */
  if (boost::geometry::within(curr_pos, current_llt.polygon2d().basicPolygon()))
  {         
    next_distance = distToNearestActiveGeofence(curr_pos);
    outgoing_geof.distance_to_next_geofence = next_distance;

    for(auto id : active_geofence_llt_ids_) 
    {
      if (id == current_llt.id())
      {           
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Vehicle is on Lanelet " << current_llt.id() << ", which has an active geofence");
        outgoing_geof.is_on_active_geofence = true;
        for (auto regem: current_llt.regulatoryElements())
        {
          // Assign active geofence fields based on the speed limit associated with this lanelet
          if (regem->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0)
          {
            lanelet::DigitalSpeedLimitPtr speed =  std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>
            (current_map_->regulatoryElementLayer.get(regem->id()));
            outgoing_geof.value = speed->speed_limit_.value();
            outgoing_geof.advisory_speed = speed->speed_limit_.value();
            outgoing_geof.reason = speed->getReason(); 

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Active geofence has a speed limit of " << speed->speed_limit_.value());
                    
            // Cannot overrule outgoing_geof.type if it is already set to LANE_CLOSED
            if(outgoing_geof.type != carma_perception_msgs::msg::CheckActiveGeofence::LANE_CLOSED)
            {
              outgoing_geof.type = carma_perception_msgs::msg::CheckActiveGeofence::SPEED_LIMIT;
            }
          }

          // Assign active geofence fields based on the minimum gap associated with this lanelet (if it exists)
          if(regem->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalMinimumGap::RuleName) == 0)
          {
            lanelet::DigitalMinimumGapPtr min_gap =  std::dynamic_pointer_cast<lanelet::DigitalMinimumGap>
            (current_map_->regulatoryElementLayer.get(regem->id()));
            outgoing_geof.minimum_gap = min_gap->getMinimumGap();
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Active geofence has a minimum gap of " << min_gap->getMinimumGap());
          }
                 
          // Assign active geofence fields based on whether the current lane is closed or is immediately adjacent to a closed lane
          if(regem->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::RegionAccessRule::RuleName) == 0)
          {
            lanelet::RegionAccessRulePtr accessRuleReg =  std::dynamic_pointer_cast<lanelet::RegionAccessRule>
            (current_map_->regulatoryElementLayer.get(regem->id()));

            // Update the 'type' and 'reason' for this active geofence if the vehicle is in a closed lane
            if(!accessRuleReg->accessable(lanelet::Participants::VehicleCar) || !accessRuleReg->accessable(lanelet::Participants::VehicleTruck)) 
            {
              RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Active geofence is a closed lane.");
              RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Closed lane reason: " << accessRuleReg->getReason());
              outgoing_geof.reason = accessRuleReg->getReason();
              outgoing_geof.type = carma_perception_msgs::msg::CheckActiveGeofence::LANE_CLOSED;
            }
            // Otherwise, update the 'type' and 'reason' for this active geofence if the vehicle is in a lane immediately adjacent to a closed lane with the same travel direction
            else 
            {
              // Obtain all same-direction lanes sharing the right lane boundary (will include the current lanelet)
              auto right_boundary_lanelets = current_map_->laneletLayer.findUsages(current_llt.rightBound());

              // Check if the adjacent right lane is closed
              if(right_boundary_lanelets.size() > 1)
              {
                for(auto lanelet : right_boundary_lanelets)
                {
                  // Only check the adjacent right lanelet; ignore the current lanelet
                  if(lanelet.id() != current_llt.id())
                  {
                    for (auto rightRegem: lanelet.regulatoryElements())
                    {
                      if(rightRegem->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::RegionAccessRule::RuleName) == 0)
                      {
                        lanelet::RegionAccessRulePtr rightAccessRuleReg =  std::dynamic_pointer_cast<lanelet::RegionAccessRule>
                        (current_map_->regulatoryElementLayer.get(rightRegem->id()));
                        if(!rightAccessRuleReg->accessable(lanelet::Participants::VehicleCar) || !rightAccessRuleReg->accessable(lanelet::Participants::VehicleTruck))
                        {
                          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Right adjacent Lanelet " << lanelet.id() << " is CLOSED");
                          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Assigning LANE_CLOSED type to active geofence");
                          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Assigning reason " << rightAccessRuleReg->getReason());
                          outgoing_geof.reason = rightAccessRuleReg->getReason();
                          outgoing_geof.type = carma_perception_msgs::msg::CheckActiveGeofence::LANE_CLOSED;
                        }
                      }
                    }
                  }
                }
              }

              // Check if the adjacent left lane is closed
              auto left_boundary_lanelets = current_map_->laneletLayer.findUsages(current_llt.leftBound());
              if(left_boundary_lanelets.size() > 1)
              {
                for(auto lanelet : left_boundary_lanelets)
                {
                  // Only check the adjacent left lanelet; ignore the current lanelet
                  if(lanelet.id() != current_llt.id())
                  {
                    for (auto leftRegem: lanelet.regulatoryElements())
                    {
                      if(leftRegem->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::RegionAccessRule::RuleName) == 0)
                      {
                        lanelet::RegionAccessRulePtr leftAccessRuleReg =  std::dynamic_pointer_cast<lanelet::RegionAccessRule>
                        (current_map_->regulatoryElementLayer.get(leftRegem->id()));
                        if(!leftAccessRuleReg->accessable(lanelet::Participants::VehicleCar) || !leftAccessRuleReg->accessable(lanelet::Participants::VehicleTruck))
                        {
                          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Left adjacent Lanelet " << lanelet.id() << " is CLOSED");
                          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Assigning LANE_CLOSED type to active geofence");
                          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Assigning reason " << leftAccessRuleReg->getReason());
                          outgoing_geof.reason = leftAccessRuleReg->getReason();
                          outgoing_geof.type = carma_perception_msgs::msg::CheckActiveGeofence::LANE_CLOSED;
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  return outgoing_geof;
}

lanelet::LineString3d WMBroadcaster::createLinearInterpolatingLinestring(const lanelet::Point3d& front_pt, const lanelet::Point3d& back_pt, double increment_distance)
{
  double dx = back_pt.x() - front_pt.x();
  double dy = back_pt.y() - front_pt.y();

  std::vector<lanelet::Point3d> points;
  double distance = std::sqrt(pow(dx, 2) + pow(dy,2));
  double cos = dx / distance;
  double sin = dy / distance;
  points.push_back(front_pt);
  double sum = increment_distance;
  while ( sum < distance)
  {
    points.push_back(lanelet::Point3d(lanelet::utils::getId(),front_pt.x() + sum * cos, front_pt.y() + sum * sin, 0.0));
    sum += increment_distance;
  }
  points.push_back(back_pt);

  return lanelet::LineString3d(lanelet::utils::getId(), points);
}

lanelet::Lanelet  WMBroadcaster::createLinearInterpolatingLanelet(const lanelet::Point3d& left_front_pt, const lanelet::Point3d& right_front_pt, const lanelet::Point3d& left_back_pt, const lanelet::Point3d& right_back_pt, double increment_distance)
{
  return lanelet::Lanelet(lanelet::utils::getId(), createLinearInterpolatingLinestring(left_front_pt, left_back_pt, increment_distance), createLinearInterpolatingLinestring(right_front_pt, right_back_pt, increment_distance));
}

void WMBroadcaster::updateUpcomingSGIntersectionIds()
{
  uint16_t map_msg_intersection_id = 0;
  uint16_t cur_signal_group_id = 0;
  std::vector<lanelet::CarmaTrafficSignalPtr> traffic_lights;
  lanelet::Lanelet route_lanelet;
  lanelet::Ids cur_route_lanelet_ids = current_route.route_path_lanelet_ids;
  bool isLightFound = false;
  
  for(auto id : cur_route_lanelet_ids) 
  {    
    route_lanelet= current_map_->laneletLayer.get(id);
    traffic_lights = route_lanelet.regulatoryElementsAs<lanelet::CarmaTrafficSignal>();
    if(!traffic_lights.empty())
    {
      isLightFound  = true;
      break;
    }
  }

  if(isLightFound && !sim_)
  {
    for(auto itr = sim_->signal_group_to_traffic_light_id_.begin(); itr != sim_->signal_group_to_traffic_light_id_.end(); itr++)
    {     
      if(itr->second == traffic_lights.front()->id())
      {
        cur_signal_group_id = itr->first;
      }
    }
  }
  else{
     RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "NO matching Traffic lights along the route");
  }//END Traffic signals

  auto intersections = route_lanelet.regulatoryElementsAs<lanelet::SignalizedIntersection>();
  if (intersections.empty())
  {
    // no match if any of the entry lanelet is not part of any intersection.
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "NO matching intersection for current lanelet. lanelet id = " << route_lanelet.id());
  }
  else
  {
    //Currently, each lanelet has only one intersection
    lanelet::Id intersection_id = intersections.front()->id();    
    if(intersection_id != lanelet::InvalId)
    {
      for(auto itr = sim_->intersection_id_to_regem_id_.begin(); itr != sim_->intersection_id_to_regem_id_.end(); itr++)
      {
        if(itr->second == intersection_id)
        {
          map_msg_intersection_id = itr->first;
        }
      }
    }
  } //END intersections

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "MAP msg: Intersection ID = " <<  map_msg_intersection_id << ", Signal Group ID =" << cur_signal_group_id );
  if(map_msg_intersection_id != 0 && cur_signal_group_id != 0)
  { 
    upcoming_intersection_ids_.data.clear();
    upcoming_intersection_ids_.data.push_back(static_cast<int>(map_msg_intersection_id));
    upcoming_intersection_ids_.data.push_back(static_cast<int>(cur_signal_group_id));
  }
}

void WMBroadcaster::pubTCMACK(j2735_v2x_msgs::msg::Id64b tcm_req_id, uint16_t msgnum, int ack_status, const std::string& ack_reason)
{
  carma_v2x_msgs::msg::MobilityOperation mom_msg;
  mom_msg.m_header.timestamp = scheduler_.now().nanoseconds()/1000000;
  mom_msg.m_header.sender_id = vehicle_id_;
  mom_msg.strategy = geofence_ack_strategy_;
  std::stringstream ss;
  for(size_t i=0; i < tcm_req_id.id.size(); i++)
  {
    ss << std::setfill('0') << std::setw(2) << std::hex << (unsigned) tcm_req_id.id.at(i);
  }
	std::string tcmv01_req_id_hex = ss.str();	
  ss.str("");
  ss << "traffic_control_id:" << tcmv01_req_id_hex << ", msgnum:"<< msgnum << ", acknowledgement:" << ack_status << ", reason:" << ack_reason;
  mom_msg.strategy_params = ss.str();
  for(int i = 0; i < ack_pub_times_; i++)
  {
    tcm_ack_pub_(mom_msg);
  }
}

const uint8_t WorkZoneSection::OPEN = 0;
const uint8_t WorkZoneSection::CLOSED = 1;
const uint8_t WorkZoneSection::TAPERLEFT = 2;
const uint8_t WorkZoneSection::TAPERRIGHT = 3;
const uint8_t WorkZoneSection::OPENLEFT = 4;
const uint8_t WorkZoneSection::OPENRIGHT = 5;
const uint8_t WorkZoneSection::REVERSE = 6;


}  // namespace carma_wm_ctrl

