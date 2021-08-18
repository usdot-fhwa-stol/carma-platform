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
#include <carma_wm/Geometry.h>
#include <carma_wm/MapConformer.h>
#include <autoware_lanelet2_ros_interface/utility/message_conversion.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <type_traits>
#include <cav_msgs/CheckActiveGeofence.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <proj.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_core/Forward.h>
#include <autoware_lanelet2_ros_interface/utility/utilities.h>
#include <algorithm>
#include <limits>
#include <carma_wm/Geometry.h>
#include <math.h>
#include <boost/date_time/date_defs.hpp>

namespace carma_wm_ctrl
{
using std::placeholders::_1;


WMBroadcaster::WMBroadcaster(const PublishMapCallback& map_pub, const PublishMapUpdateCallback& map_update_pub, const PublishCtrlRequestCallback& control_msg_pub,
const PublishActiveGeofCallback& active_pub, std::unique_ptr<carma_utils::timers::TimerFactory> timer_factory)
  : map_pub_(map_pub), map_update_pub_(map_update_pub), control_msg_pub_(control_msg_pub), active_pub_(active_pub), scheduler_(std::move(timer_factory))
{
  scheduler_.onGeofenceActive(std::bind(&WMBroadcaster::addGeofence, this, _1));
  scheduler_.onGeofenceInactive(std::bind(&WMBroadcaster::removeGeofence, this, _1));
  std::bind(&WMBroadcaster::routeCallbackMessage, this, _1);
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

  lanelet::MapConformer::ensureCompliance(base_map_, config_limit);     // Update map to ensure it complies with expectations
  lanelet::MapConformer::ensureCompliance(current_map_, config_limit);

  ROS_INFO_STREAM("Building routing graph for base map");

  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules_car = lanelet::traffic_rules::TrafficRulesFactory::create(
  lanelet::traffic_rules::CarmaUSTrafficRules::Location, lanelet::Participants::VehicleCar);
  current_routing_graph_ = lanelet::routing::RoutingGraph::build(*current_map_, *traffic_rules_car);

  ROS_INFO_STREAM("Done building routing graph for base map");

  // Publish map
  current_map_version_ += 1; // Increment the map version. It should always start from 1 for the first map
  map_update_message_queue_.clear(); // Clear the update queue as the map version has changed
  autoware_lanelet2_msgs::MapBin compliant_map_msg;
  lanelet::utils::conversion::toBinMsg(current_map_, &compliant_map_msg);
  compliant_map_msg.map_version = current_map_version_;
  map_pub_(compliant_map_msg);
};

std::shared_ptr<Geofence> WMBroadcaster::geofenceFromMsg(const cav_msgs::TrafficControlMessageV01& msg_v01)
{
  auto gf_ptr = std::make_shared<Geofence>(Geofence());
  // Get ID
  std::copy(msg_v01.id.id.begin(), msg_v01.id.id.end(), gf_ptr->id_.begin());

  // Get affected lanelet or areas by converting the georeference and querying the map using points in the geofence
  gf_ptr->affected_parts_ = getAffectedLaneletOrAreas(msg_v01);

  if (gf_ptr->affected_parts_.size() == 0) {
    ROS_WARN_STREAM("There is no applicable component in map for the new geofence message received by WMBroadcaster with id: " << gf_ptr->id_);
    return nullptr; // Return null geofence
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
  // - digital speed limit, passing control line, digital minimum gap, region access rule
  lanelet::Velocity sL;
  cav_msgs::TrafficControlDetail msg_detail = msg_v01.params.detail;
 
  
  if (msg_detail.choice == cav_msgs::TrafficControlDetail::MAXSPEED_CHOICE) 
  {  
    //Acquire speed limit information from TafficControlDetail msg
    sL = lanelet::Velocity(msg_detail.maxspeed * lanelet::units::MPH()); 
    
    if(config_limit > 0_mph && config_limit < 80_mph && config_limit < sL)//Accounting for the configured speed limit, input zero when not in use
        sL = config_limit;
    //Ensure Geofences do not provide invalid speed limit data (exceed predetermined maximum value)
    // @SONAR_STOP@
    if(sL > 80_mph )
    {
     ROS_WARN_STREAM("Digital maximum speed limit is invalid. Value capped at max speed limit."); //Output warning message
     sL = 80_mph; //Cap the speed limit to the predetermined maximum value

    }
    if(sL < 0_mph)
    {
           ROS_WARN_STREAM("Digital  speed limit is invalid. Value set to 0mph.");
      sL = 0_mph;
    }// @SONAR_START@
    gf_ptr->regulatory_element_ = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::utils::getId(), 
                                        sL, affected_llts, affected_areas, participantsChecker(msg_v01) ));
  }
  
  if (msg_detail.choice == cav_msgs::TrafficControlDetail::MINSPEED_CHOICE) 
  {
    //Acquire speed limit information from TafficControlDetail msg
     sL = lanelet::Velocity(msg_detail.minspeed * lanelet::units::MPH());
     if(config_limit > 0_mph && config_limit < 80_mph)//Accounting for the configured speed limit, input zero when not in use
        sL = config_limit;
    //Ensure Geofences do not provide invalid speed limit data 
    // @SONAR_STOP@
    if(sL > 80_mph )
    {
     ROS_WARN_STREAM("Digital speed limit is invalid. Value capped at max speed limit.");
     sL = 80_mph;
    }
    if(sL < 0_mph)
    {
           ROS_WARN_STREAM("Digital  speed limit is invalid. Value set to 0mph.");
      sL = 0_mph;
    }// @SONAR_START@
    gf_ptr->regulatory_element_ = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::utils::getId(), 
                                        sL, affected_llts, affected_areas, participantsChecker(msg_v01) ));
  }
  if (msg_detail.choice == cav_msgs::TrafficControlDetail::LATPERM_CHOICE || msg_detail.choice == cav_msgs::TrafficControlDetail::LATAFFINITY_CHOICE)
  {
    addPassingControlLineFromMsg(gf_ptr, msg_v01, affected_llts);
  }

 if (msg_detail.choice == cav_msgs::TrafficControlDetail::CLOSED_CHOICE && msg_detail.closed==cav_msgs::TrafficControlDetail::CLOSED)
  {
    addRegionAccessRule(gf_ptr,msg_v01,affected_llts);
  }

  if (msg_detail.choice == cav_msgs::TrafficControlDetail::MINHDWY_CHOICE) 
  {

    double min_gap = (double)msg_detail.minhdwy;

    if(min_gap < 0)
    {
      ROS_WARN_STREAM("Digital min gap is invalid. Value set to 0 meter.");
      min_gap = 0;
    }
    addRegionMinimumGap(gf_ptr,msg_v01, min_gap, affected_llts, affected_areas);
  }

  // Handle schedule provessing
  cav_msgs::TrafficControlSchedule msg_schedule = msg_v01.params.schedule;
  
  ros::Time end_time = msg_schedule.end;
  if (!msg_schedule.end_exists) {
    ROS_DEBUG_STREAM("No end time for geofence, using ros::TIME_MAX");
    end_time = ros::TIME_MAX; // If there is no end time use the max time
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
        gf_ptr->schedules.push_back(GeofenceSchedule(msg_schedule.start,  
                                    end_time,
                                    daily_schedule.begin,     
                                    daily_schedule.duration,
                                    msg_schedule.repeat.offset,
                                    msg_schedule.repeat.span,   
                                    msg_schedule.repeat.period,
                                    week_day_set));
      } else {
        gf_ptr->schedules.push_back(GeofenceSchedule(msg_schedule.start,  
                                  end_time,
                                  daily_schedule.begin,     
                                  daily_schedule.duration,
                                  ros::Duration(0.0), // No offset
                                  daily_schedule.duration - daily_schedule.begin,   // Compute schedule portion end time
                                  daily_schedule.duration - daily_schedule.begin,   // No repetition so same as portion end time
                                  week_day_set));         
      }

    }
  }
  else {
    if (msg_schedule.repeat_exists) {
      gf_ptr->schedules.push_back(GeofenceSchedule(msg_schedule.start,  
                                  end_time,
                                  ros::Duration(0.0),     
                                  ros::Duration(86400.0), // 24 hr daily application
                                  msg_schedule.repeat.offset,
                                  msg_schedule.repeat.span,   
                                  msg_schedule.repeat.period,
                                  week_day_set)); 
    } else {
      gf_ptr->schedules.push_back(GeofenceSchedule(msg_schedule.start,  
                                  end_time,
                                  ros::Duration(0.0),     
                                  ros::Duration(86400.0), // 24 hr daily application
                                  ros::Duration(0.0),     // No offset
                                  ros::Duration(86400.0), // Applied for full lenth of 24 hrs
                                  ros::Duration(86400.0), // No repetition
                                  week_day_set)); 
    }
   
  }
    
  return gf_ptr;
}

void WMBroadcaster::addPassingControlLineFromMsg(std::shared_ptr<Geofence> gf_ptr, const cav_msgs::TrafficControlMessageV01& msg_v01, const std::vector<lanelet::Lanelet>& affected_llts) const
{
  cav_msgs::TrafficControlDetail msg_detail;
  msg_detail = msg_v01.params.detail;
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
  ros::V_string left_participants;
  ros::V_string right_participants;
  ros::V_string participants=participantsChecker(msg_v01);

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

void WMBroadcaster::addRegionAccessRule(std::shared_ptr<Geofence> gf_ptr, const cav_msgs::TrafficControlMessageV01& msg_v01, const std::vector<lanelet::Lanelet>& affected_llts) const
{
  const std::string& reason = msg_v01.package.label;
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

void WMBroadcaster::addRegionMinimumGap(std::shared_ptr<Geofence> gf_ptr,  const cav_msgs::TrafficControlMessageV01& msg_v01, double min_gap, const std::vector<lanelet::Lanelet>& affected_llts, const std::vector<lanelet::Area>& affected_areas) const
{
  auto regulatory_element = std::make_shared<lanelet::DigitalMinimumGap>(lanelet::DigitalMinimumGap::buildData(lanelet::utils::getId(), 
                                        min_gap, affected_llts, affected_areas,participantsChecker(msg_v01) ));
  
  gf_ptr->regulatory_element_ = regulatory_element;
}

ros::V_string WMBroadcaster::participantsChecker(const cav_msgs::TrafficControlMessageV01& msg_v01) const
{
  ros::V_string participants;
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
  // combine to single vehicle type if possible, otherwise pass through
  return  combineParticipantsToVehicle(participants);
}

ros::V_string WMBroadcaster::invertParticipants(const ros::V_string& input_participants) const
{
  ros::V_string participants;

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

ros::V_string WMBroadcaster::combineParticipantsToVehicle(const ros::V_string& input_participants) const
{
  ros::V_string participants;

  if(std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::VehicleMotorcycle)!= input_participants.end() &&
      std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::VehicleBus) != input_participants.end() &&
      std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::VehicleCar) != input_participants.end() &&
      std::find(input_participants.begin(),input_participants.end(),lanelet::Participants::VehicleTruck) != input_participants.end())
  {
    ROS_DEBUG_STREAM("Detected participants to cover all possible vehicle types");
    participants.emplace_back(lanelet::Participants::Vehicle);
  }
  else
  {
    ROS_DEBUG_STREAM("Not making any changes to the participants list");
    participants = input_participants;
  }

  return participants;
}

// currently only supports geofence message version 1: TrafficControlMessageV01 
void WMBroadcaster::geofenceCallback(const cav_msgs::TrafficControlMessage& geofence_msg)
{
  
  std::lock_guard<std::mutex> guard(map_mutex_);
  // quickly check if the id has been added
  if (geofence_msg.choice != cav_msgs::TrafficControlMessage::TCMV01) {
    ROS_WARN_STREAM("Dropping recieved geofence for unsupported TrafficControl version: " << geofence_msg.choice);
    return;
  }

  boost::uuids::uuid id;
  std::copy(geofence_msg.tcmV01.id.id.begin(), geofence_msg.tcmV01.id.id.end(), id.begin());
  if (checked_geofence_ids_.find(boost::uuids::to_string(id)) != checked_geofence_ids_.end()) { 
    ROS_DEBUG_STREAM("Dropping recieved TrafficControl message with already handled id: " <<  boost::uuids::to_string(id));
    return;
  }

  // convert reqid to string check if it has been seen before
  boost::array<uint8_t, 16UL> req_id;
  for (auto i = 0; i < 8; i ++) req_id[i] = geofence_msg.tcmV01.reqid.id[i];
  boost::uuids::uuid uuid_id;
  std::copy(req_id.begin(),req_id.end(), uuid_id.begin());
  std::string reqid = boost::uuids::to_string(uuid_id).substr(0, 8);
  // drop if the req has never been sent
  if (generated_geofence_reqids_.find(reqid) == generated_geofence_reqids_.end() && reqid.compare("00000000") != 0)
  {
    ROS_WARN_STREAM("CARMA_WM_CTRL received a TrafficControlMessage with unknown TrafficControlRequest ID (reqid): " << reqid);
    return;
  }
    
  checked_geofence_ids_.insert(boost::uuids::to_string(id));
  auto gf_ptr = geofenceFromMsg(geofence_msg.tcmV01);
  if (gf_ptr == nullptr || gf_ptr->affected_parts_.size() == 0)
  {
    ROS_WARN_STREAM("Geofence message could not be converted");
    tcm_marker_array_.markers.resize(tcm_marker_array_.markers.size() - 1); //truncate this geofence
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

void WMBroadcaster::setConfigSpeedLimit(double cL)
{
  /*Logic to change config_lim to Velocity value config_limit*/
  config_limit = lanelet::Velocity(cL * lanelet::units::MPH());
}

// currently only supports geofence message version 1: TrafficControlMessageV01 
lanelet::ConstLaneletOrAreas WMBroadcaster::getAffectedLaneletOrAreas(const cav_msgs::TrafficControlMessageV01& tcmV01)
{
  ROS_DEBUG_STREAM("Getting affected lanelets");
  if (!current_map_)
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
  std::string projection = tcmV01.geometry.proj;
  std::string datum = tcmV01.geometry.datum;
  if (datum.empty()) {
    ROS_WARN_STREAM("Datum field not populated. Attempting to use WGS84");
    datum = "WGS84";
  }

  
  ROS_DEBUG_STREAM("Projection field: " << projection);
  ROS_DEBUG_STREAM("Datum field: " << datum);
  
  std::string universal_frame = datum; //lat/long included in TCM is in this datum


  ROS_DEBUG_STREAM("Traffic Control heading provided: " << tcmV01.geometry.heading << " System understanding is that this value will not affect the projection and is only provided for supporting derivative calculations.");
  
  // Create the resulting projection transformation
  PJ* universal_to_target = proj_create_crs_to_crs(PJ_DEFAULT_CTX, universal_frame.c_str(), projection.c_str(), nullptr);
  if (universal_to_target == nullptr) { // proj_create_crs_to_crs returns 0 when there is an error in the projection
    
    ROS_ERROR_STREAM("Failed to generate projection between geofence and map with error number: " <<  proj_context_errno(PJ_DEFAULT_CTX) 
      << " universal_frame: " << universal_frame << " projection: " << projection);

    return {}; // Ignore geofence if it could not be projected from universal to TCM frame
  }
  
  PJ* target_to_map = proj_create_crs_to_crs(PJ_DEFAULT_CTX, projection.c_str(), base_map_georef_.c_str(), nullptr);

  if (target_to_map == nullptr) { // proj_create_crs_to_crs returns 0 when there is an error in the projection
    
    ROS_ERROR_STREAM("Failed to generate projection between geofence and map with error number: " <<  proj_context_errno(PJ_DEFAULT_CTX) 
      << " target_to_map: " << target_to_map << " base_map_georef_: " << base_map_georef_);

    return {}; // Ignore geofence if it could not be projected into the map frame
  
  }
  
  // convert all geofence points into our map's frame
  std::vector<lanelet::Point3d> gf_pts;
  cav_msgs::PathNode prev_pt;
  PJ_COORD c_init_latlong{{tcmV01.geometry.reflat, tcmV01.geometry.reflon, tcmV01.geometry.refelv}};
  PJ_COORD c_init = proj_trans(universal_to_target, PJ_FWD, c_init_latlong);

  prev_pt.x = c_init.xyz.x;
  prev_pt.y =  c_init.xyz.y;

  ROS_DEBUG_STREAM("In TCM's frame, initial Point X "<< prev_pt.x<<" Before conversion: Point Y "<< prev_pt.y );
  for (auto pt : tcmV01.geometry.nodes)
  { 
    ROS_DEBUG_STREAM("Before conversion in TCM frame: Point X "<< pt.x <<" Before conversion: Point Y "<< pt.y);

    PJ_COORD c {{prev_pt.x + pt.x, prev_pt.y + pt.y, 0, 0}}; // z is not currently used
    PJ_COORD c_out;
    c_out = proj_trans(target_to_map, PJ_FWD, c);

    gf_pts.push_back(lanelet::Point3d{current_map_->pointLayer.uniqueId(), c_out.xyz.x, c_out.xyz.y});
    prev_pt.x += pt.x;
    prev_pt.y += pt.y;

    ROS_DEBUG_STREAM("After conversion in Map frame: Point X "<< gf_pts.back().x() <<" After conversion: Point Y "<< gf_pts.back().y());
   }

  tcm_marker_array_.markers.push_back(composeTCMMarkerVisualizer(gf_pts));

  // Logic to detect which part is affected
  ROS_DEBUG_STREAM("Get affected lanelets loop");
  std::unordered_set<lanelet::Lanelet> affected_lanelets;
  for (size_t idx = 0; idx < gf_pts.size(); idx ++)
  {
    ROS_DEBUG_STREAM("Index: " << idx << " Point: " << gf_pts[idx].x() << ", " << gf_pts[idx].y());
    std::unordered_set<lanelet::Lanelet> possible_lanelets;

    // This loop identifes the lanelets which this point lies within that could be impacted by the geofence
    // This loop somewhat inefficiently calls the findNearest method iteratively until all the possible lanelets are identified. 
    // The reason findNearest is used instead of nearestUntil is because that method orders results by bounding box which
    // can give invalid sequences when dealing with large curved lanelets.  
    bool continue_search = true; 
    size_t nearest_count = 0;
    while (continue_search) {
      
      nearest_count += 10; // Increase the index search radius by 10 each loop until all nearby lanelets are found

      for (const auto& ll_pair : lanelet::geometry::findNearest(current_map_->laneletLayer, gf_pts[idx].basicPoint2d(), nearest_count)) { // Get the nearest lanelets and iterate over them
        auto ll = std::get<1>(ll_pair);

        if (possible_lanelets.find(ll) != possible_lanelets.end()) { // Skip if already found
          continue;
        }

        double dist = std::get<0>(ll_pair);
        ROS_DEBUG_STREAM("Distance to lanelet " << ll.id() << ": " << dist << " max_lane_width: " << max_lane_width_);
        
        if (dist > max_lane_width_) { // Only save values closer than max_lane_width. Since we are iterating in distance order when we reach this distance the search can stop
          continue_search = false;
          break;
        }

        // Check if the point is inside this lanelet
        if(dist == 0.0) { // boost geometry uses a distance of 0 to indicate a point is within a polygon
          possible_lanelets.insert(ll);
        }

      }

      if (nearest_count >= current_map_->laneletLayer.size()) { // if we are out of lanelets to evaluate then end the search
        continue_search = false;
      }
    }

    // among these llts, filter the ones that are on same direction as the geofence using routing
    if (idx + 1 == gf_pts.size()) // we only check this for the last gf_pt after saving everything
    {
      ROS_DEBUG_STREAM("Last point");
      std::unordered_set<lanelet::Lanelet> filtered = filterSuccessorLanelets(possible_lanelets, affected_lanelets);
      ROS_DEBUG_STREAM("Got successor lanelets of size: " << filtered.size());
      affected_lanelets.insert(filtered.begin(), filtered.end());
      break;
    } 

    ROS_DEBUG_STREAM("Checking possible lanelets");
    // check if each lines connecting end points of the llt is crossing with the line connecting current and next gf_pts
    for (auto llt: possible_lanelets)
    {
      ROS_DEBUG_STREAM("Evaluating lanelet: " << llt.id());
      lanelet::BasicLineString2d gf_dir_line({gf_pts[idx].basicPoint2d(), gf_pts[idx+1].basicPoint2d()});
      lanelet::BasicLineString2d llt_boundary({(llt.leftBound2d().end() -1)->basicPoint2d(), (llt.rightBound2d().end() - 1)->basicPoint2d()});
      
      // record the llts that are on the same dir
      if (boost::geometry::intersects(llt_boundary, gf_dir_line))
      {
        ROS_DEBUG_STREAM("Overlaps end line");
        affected_lanelets.insert(llt);
      }
      // check condition if two geofence points are in one lanelet then check matching direction and record it also
      else if (boost::geometry::within(gf_pts[idx+1].basicPoint2d(), llt.polygon2d()) && 
              affected_lanelets.find(llt) == affected_lanelets.end())
      { 
        ROS_DEBUG_STREAM("Within new lanelet");
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

        ROS_DEBUG_STREAM("vec_to_median: " << vec_to_median.x() << ", " << vec_to_median.y());
        ROS_DEBUG_STREAM("vec_to_gf_start: " << vec_to_gf_start.x() << ", " << vec_to_gf_start.y());
        ROS_DEBUG_STREAM("vec_to_gf_end: " << vec_to_gf_end.x() << ", " << vec_to_gf_end.y());
        ROS_DEBUG_STREAM("start_to_median: " << start_to_median.x() << ", " << start_to_median.y());
        ROS_DEBUG_STREAM("start_to_end: " << start_to_end.x() << ", " << start_to_end.y());
        ROS_DEBUG_STREAM("interior_angle: " << interior_angle);
        // Save the lanelet if the direction of two points inside aligns with that of the lanelet
        if (interior_angle < M_PI_2 && interior_angle >= 0) affected_lanelets.insert(llt); 
      }
    }
  }
  
  ROS_DEBUG_STREAM("affected_lanelets size: " << affected_lanelets.size());
  // Currently only returning lanelet, but this could be expanded to LanelerOrArea compound object 
  // by implementing non-const version of that LaneletOrArea
  lanelet::ConstLaneletOrAreas affected_parts;
  affected_parts.insert(affected_parts.end(), affected_lanelets.begin(), affected_lanelets.end());
  return affected_parts;
}

// helper function that filters successor lanelets of root_lanelets from possible_lanelets
std::unordered_set<lanelet::Lanelet> WMBroadcaster::filterSuccessorLanelets(const std::unordered_set<lanelet::Lanelet>& possible_lanelets, const std::unordered_set<lanelet::Lanelet>& root_lanelets)
{
  if (!current_routing_graph_) {
    throw std::invalid_argument("No routing graph available");
  }
  
  std::unordered_set<lanelet::Lanelet> filtered_lanelets;
  // we utilize routes to filter llts that are overlapping but not connected
  // as this is the last lanelet 
  // we have to filter the llts that are only geometrically overlapping yet not connected to prev llts
  for (auto recorded_llt: root_lanelets)
  {
    for (auto following_llt: current_routing_graph_->following(recorded_llt, false))
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

void WMBroadcaster::addRegulatoryComponent(std::shared_ptr<Geofence> gf_ptr) const
{

  // First loop is to save the relation between element and regulatory element
  // so that we can add back the old one after geofence deactivates
  for (auto el: gf_ptr->affected_parts_)
  {
    for (auto regem : el.regulatoryElements())
    {
      if (!shouldChangeControlLine(el, regem, gf_ptr)) continue;

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
      ROS_WARN_STREAM("Regulatory element with invalid id in geofence cannot be added to the map");
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
  ROS_INFO_STREAM("Adding active geofence to the map with geofence id: " << gf_ptr->id_);
  
  // Process the geofence object to populate update remove lists
  addGeofenceHelper(gf_ptr);
  
  for (auto pair : gf_ptr->update_list_) active_geofence_llt_ids_.insert(pair.first);

  // If the geofence invalidates the route graph then recompute the routing graph now that the map has been updated
  if (gf_ptr->invalidate_route_) {

    ROS_INFO_STREAM("Rebuilding routing graph after is was invalidated by geofence");

    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules_car = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::traffic_rules::CarmaUSTrafficRules::Location, lanelet::Participants::VehicleCar);
    current_routing_graph_ = lanelet::routing::RoutingGraph::build(*current_map_, *traffic_rules_car);

    ROS_INFO_STREAM("Done rebuilding routing graph after is was invalidated by geofence");

  }
  

  // Publish
  autoware_lanelet2_msgs::MapBin gf_msg;
  auto send_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_));
  carma_wm::toBinMsg(send_data, &gf_msg);
  update_count_++; // Update the sequence count for the geofence messages
  gf_msg.header.seq = update_count_;
  gf_msg.invalidates_route=gf_ptr->invalidate_route_; 
  gf_msg.map_version = current_map_version_;
  map_update_message_queue_.push_back(gf_msg); // Add diff to current map update queue
  map_update_pub_(gf_msg);
};

void WMBroadcaster::removeGeofence(std::shared_ptr<Geofence> gf_ptr)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_INFO_STREAM("Removing inactive geofence from the map with geofence id: " << gf_ptr->id_);
  
  // Process the geofence object to populate update remove lists
  removeGeofenceHelper(gf_ptr);

  for (auto pair : gf_ptr->remove_list_) active_geofence_llt_ids_.erase(pair.first);

  // publish
  autoware_lanelet2_msgs::MapBin gf_msg_revert;
  auto send_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_));
  
  carma_wm::toBinMsg(send_data, &gf_msg_revert);
  update_count_++; // Update the sequence count for geofence messages
  gf_msg_revert.header.seq = update_count_;
  gf_msg_revert.map_version = current_map_version_;
  map_update_message_queue_.push_back(gf_msg_revert); // Add diff to current map update queue
  map_update_pub_(gf_msg_revert);


};
  
cav_msgs::Route WMBroadcaster::getRoute()
{
  return current_route;
}

void  WMBroadcaster::routeCallbackMessage(const cav_msgs::Route& route_msg)
{

 current_route = route_msg;
 cav_msgs::TrafficControlRequest cR; 
 cR =  controlRequestFromRoute(route_msg);
 control_msg_pub_(cR);

}

cav_msgs::TrafficControlRequest WMBroadcaster::controlRequestFromRoute(const cav_msgs::Route& route_msg, std::shared_ptr<j2735_msgs::Id64b> req_id_for_testing)
{
  lanelet::ConstLanelets path; 

  if (!current_map_) 
  {
   // Return / log warning etc.
    ROS_INFO_STREAM("Value 'current_map_' does not exist.");
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
    ROS_INFO_STREAM("Value 'target_frame' is empty.");
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
    
    ROS_ERROR_STREAM("Failed to generate projection between request bounds frame and map with error number: " <<  proj_context_errno(PJ_DEFAULT_CTX) 
      << " MapProjection: " << target_frame << " Message Projection: " << local_tmerc_enu_proj);

    return {}; // Ignore geofence if it could not be projected into the map frame
  
  }

  ROS_DEBUG_STREAM("Before conversion: Top Left: ("<< minX <<", "<<maxY<<")");
  ROS_DEBUG_STREAM("Before conversion: Top Right: ("<< maxX <<", "<<maxY<<")");
  ROS_DEBUG_STREAM("Before conversion: Bottom Left: ("<< minX <<", "<<minY<<")");
  ROS_DEBUG_STREAM("Before conversion: Bottom Right: ("<< maxX <<", "<<minY<<")");

  PJ_COORD pj_min {{minX, minY, 0, 0}}; // z is not currently used
  PJ_COORD pj_min_tmerc;
  PJ_COORD pj_max {{maxX, maxY, 0, 0}}; // z is not currently used
  PJ_COORD pj_max_tmerc;
  pj_min_tmerc = proj_trans(tmerc_proj, PJ_FWD, pj_min);
  pj_max_tmerc = proj_trans(tmerc_proj, PJ_FWD, pj_max);

  ROS_DEBUG_STREAM("After conversion: MinPoint ( "<< pj_min_tmerc.xyz.x <<", " << pj_min_tmerc.xyz.y <<" )");
  ROS_DEBUG_STREAM("After conversion: MaxPoint ( "<< pj_max_tmerc.xyz.x <<", " << pj_max_tmerc.xyz.y <<" )");

  cav_msgs::TrafficControlRequest cR; /*Fill the latitude value in message cB with the value of lat */
  cav_msgs::TrafficControlBounds cB; /*Fill the longitude value in message cB with the value of lon*/
  
  cB.reflat = gpsRoute.lat;
  cB.reflon = gpsRoute.lon;

  cB.offsets[0].deltax = pj_max_tmerc.xyz.x - pj_min_tmerc.xyz.x; // Points in clockwise order min,min (lat,lon point) -> max,min -> max,max -> min,max
  cB.offsets[0].deltay = 0.0;                                     // calculating the offsets
  cB.offsets[1].deltax = pj_max_tmerc.xyz.x - pj_min_tmerc.xyz.x;
  cB.offsets[1].deltay = pj_max_tmerc.xyz.y - pj_min_tmerc.xyz.y;
  cB.offsets[2].deltax = 0.0;
  cB.offsets[2].deltay = pj_max_tmerc.xyz.y - pj_min_tmerc.xyz.y;

  tcr_polygon_ = composeTCRStatus(localPoint, cB, local_projector); // TCR polygon can be visualized in UI

  cB.oldest =ros::Time::now(); // TODO this needs to be set to 0 or an older value as otherwise this will filter out all controls
  
  cR.choice = cav_msgs::TrafficControlRequest::TCRV01;
  
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
    cR.tcrV01.reqid.id[i] = req_id[i];
    if (req_id_for_testing) req_id_for_testing->id[i] = req_id[i];
  }

  cR.tcrV01.bounds.push_back(cB);
  
  return cR;

}

cav_msgs::TrafficControlRequestPolygon WMBroadcaster::composeTCRStatus(const lanelet::BasicPoint3d& localPoint, const cav_msgs::TrafficControlBounds& cB, const lanelet::projection::LocalFrameProjector& local_projector)
{
  cav_msgs::TrafficControlRequestPolygon output;
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
    
    cav_msgs::Position3D gps_msg;
    gps_msg.elevation = gps_vertex.ele;
    gps_msg.latitude = gps_vertex.lat;
    gps_msg.longitude = gps_vertex.lon;
    output.polygon_list.push_back(gps_msg);

    ROS_DEBUG_STREAM("TCR Vertex Lat: "<< std::to_string(gps_vertex.lat));
    ROS_DEBUG_STREAM("TCR Vertex Lon: "<<std::to_string(gps_vertex.lon));

    i++;
  }
  return output;
}

visualization_msgs::Marker WMBroadcaster::composeTCMMarkerVisualizer(const std::vector<lanelet::Point3d>& input)
 {

         // create the marker msgs
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
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
            geometry_msgs::Point temp_point;
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
void WMBroadcaster::addGeofenceHelper(std::shared_ptr<Geofence> gf_ptr) const
{
  // resetting the information inside geofence
  gf_ptr->remove_list_ = {};
  gf_ptr->update_list_ = {};

  // TODO: Logic to determine what type of geofence goes here in the future
  // currently only speedchange is available, so it is assumed that
  addRegulatoryComponent(gf_ptr);
}

// helper function that detects the type of geofence and delegates
void WMBroadcaster::removeGeofenceHelper(std::shared_ptr<Geofence> gf_ptr) const
{
  // again, TODO: Logic to determine what type of geofence goes here in the future
  // reset the info inside geofence
  gf_ptr->remove_list_ = {};
  gf_ptr->update_list_ = {};
  addBackRegulatoryComponent(gf_ptr);
  // as all changes are reverted back, we no longer need prev_regems
  gf_ptr->prev_regems_ = {};
}

void WMBroadcaster::currentLocationCallback(const geometry_msgs::PoseStamped& current_pos)
{
  if (current_map_ && current_map_->laneletLayer.size() != 0) {
    cav_msgs::CheckActiveGeofence check = checkActiveGeofenceLogic(current_pos);
    active_pub_(check);//Publish
  } else {
    ROS_DEBUG_STREAM("Could not check active geofence logic because map was not loaded");
  }

}

cav_msgs::CheckActiveGeofence WMBroadcaster::checkActiveGeofenceLogic(const geometry_msgs::PoseStamped& current_pos)
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

  cav_msgs::CheckActiveGeofence outgoing_geof; //message to publish
  double next_distance = 0 ; //Distance to next geofence

  if (active_geofence_llt_ids_.size() <= 0 ) 
  {
    ROS_INFO_STREAM("No active geofence llt ids are loaded to the WMBroadcaster");
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
        ROS_DEBUG_STREAM("Vehicle is on Lanelet " << current_llt.id() << ", which has an active geofence");
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
            ROS_DEBUG_STREAM("Active geofence has a speed limit of " << speed->speed_limit_.value());
                    
            // Cannot overrule outgoing_geof.type if it is already set to LANE_CLOSED
            if(outgoing_geof.type != cav_msgs::CheckActiveGeofence::LANE_CLOSED)
            {
              outgoing_geof.type = cav_msgs::CheckActiveGeofence::SPEED_LIMIT;
            }
          }

          // Assign active geofence fields based on the minimum gap associated with this lanelet (if it exists)
          if(regem->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalMinimumGap::RuleName) == 0)
          {
            lanelet::DigitalMinimumGapPtr min_gap =  std::dynamic_pointer_cast<lanelet::DigitalMinimumGap>
            (current_map_->regulatoryElementLayer.get(regem->id()));
            outgoing_geof.minimum_gap = min_gap->getMinimumGap();
            ROS_DEBUG_STREAM("Active geofence has a minimum gap of " << min_gap->getMinimumGap());
          }
                 
          // Assign active geofence fields based on whether the current lane is closed or is immediately adjacent to a closed lane
          if(regem->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::RegionAccessRule::RuleName) == 0)
          {
            lanelet::RegionAccessRulePtr accessRuleReg =  std::dynamic_pointer_cast<lanelet::RegionAccessRule>
            (current_map_->regulatoryElementLayer.get(regem->id()));

            // Update the 'type' and 'reason' for this active geofence if the vehicle is in a closed lane
            if(!accessRuleReg->accessable(lanelet::Participants::VehicleCar) || !accessRuleReg->accessable(lanelet::Participants::VehicleTruck)) 
            {
              ROS_DEBUG_STREAM("Active geofence is a closed lane.");
              ROS_DEBUG_STREAM("Closed lane reason: " << accessRuleReg->getReason());
              outgoing_geof.reason = accessRuleReg->getReason();
              outgoing_geof.type = cav_msgs::CheckActiveGeofence::LANE_CLOSED;
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
                          ROS_DEBUG_STREAM("Right adjacent Lanelet " << lanelet.id() << " is CLOSED");
                          ROS_DEBUG_STREAM("Assigning LANE_CLOSED type to active geofence");
                          ROS_DEBUG_STREAM("Assigning reason " << rightAccessRuleReg->getReason());
                          outgoing_geof.reason = rightAccessRuleReg->getReason();
                          outgoing_geof.type = cav_msgs::CheckActiveGeofence::LANE_CLOSED;
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
                          ROS_DEBUG_STREAM("Left adjacent Lanelet " << lanelet.id() << " is CLOSED");
                          ROS_DEBUG_STREAM("Assigning LANE_CLOSED type to active geofence");
                          ROS_DEBUG_STREAM("Assigning reason " << leftAccessRuleReg->getReason());
                          outgoing_geof.reason = leftAccessRuleReg->getReason();
                          outgoing_geof.type = cav_msgs::CheckActiveGeofence::LANE_CLOSED;
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

void WMBroadcaster::newUpdateSubscriber(const ros::SingleSubscriberPublisher& single_sub_pub) const {

  for (const auto& msg : map_update_message_queue_) {
    single_sub_pub.publish(msg); // For each applied update for the current map version publish the update to the new subscriber
  }
}



}  // namespace carma_wm_ctrl


