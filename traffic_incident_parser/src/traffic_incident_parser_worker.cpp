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

#include "traffic_incident_parser_worker.h"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <carma_utils/containers/containers.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Traits.h>
#include <proj.h>

namespace traffic
{

  TrafficIncidentParserWorker::TrafficIncidentParserWorker(carma_wm::WorldModelConstPtr wm,const PublishTrafficControlCallback &traffic_control_pub) : traffic_control_pub_(traffic_control_pub),wm_(wm){}

  void TrafficIncidentParserWorker::mobilityOperationCallback(const cav_msgs::MobilityOperation &mobility_msg)
  {
   
      if((mobility_msg.strategy=="carma3/Incident_Use_Case"))
      { 
           bool valid_msg = mobilityMessageParser(mobility_msg.strategy_params);

           if(valid_msg && event_type=="CLOSED")
           {
            geofence_message_queue_.clear(); // Clear the previous geofence message queue since the geofence set will now be updated
            previous_strategy_params=mobility_msg.strategy_params;
            cav_msgs::TrafficControlMessage traffic_control_msg;
            traffic_control_msg.choice=cav_msgs::TrafficControlMessage::TCMV01;
            for(auto &traffic_msg:composeTrafficControlMesssages())
            {
              traffic_control_msg.tcm_v01=traffic_msg;
              geofence_message_queue_.push_back(traffic_control_msg); // Add the message to the geofence_message_queue for publication to new subscribers
              traffic_control_pub_(traffic_control_msg); // Publish the message to existing subscribers
            }
           }
           else
           {
             return;
           }
      }
   }

  void TrafficIncidentParserWorker::projectionCallback(const std_msgs::String &projection_msg)
  {
      projection_msg_=projection_msg.data;
   }


  bool TrafficIncidentParserWorker::mobilityMessageParser(std::string mobility_strategy_params)
  {

   std::vector<std::string> vec={};
   std::string delimiter = ",";
   size_t pos = 0;
   std::string token;
    while ((pos = mobility_strategy_params.find(delimiter)) != std::string::npos) 
    {
        token = mobility_strategy_params.substr(0, pos);
        vec.push_back(token);
        mobility_strategy_params.erase(0, pos + delimiter.length());
    }
    vec.push_back(mobility_strategy_params);

    if (vec.size() != 8) 
    {
      ROS_ERROR_STREAM("Given mobility strategy params are not correctly formatted.");
      return false;
    }  

    std::string lat_str=vec[0];
    std::string lon_str=vec[1];
    std::string downtrack_str=vec[2];
    std::string uptrack_str=vec[3];
    std::string min_gap_str=vec[4];
    std::string speed_advisory_str=vec[5];
    std::string event_reason_str=vec[6];
    std::string event_type_str=vec[7];

    // Evaluate if this message should be forwarded based on the gps point
    double temp_lat = stod(stringParserHelper(lat_str,lat_str.find_last_of("lat:")));
    double temp_lon = stod(stringParserHelper(lon_str,lon_str.find_last_of("lon:")));
    
    double constexpr APPROXIMATE_DEG_PER_5M = 0.00005;
    double delta_lat = temp_lat - latitude;
    double delta_lon = temp_lon - longitude;
    double approximate_degree_delta = sqrt(delta_lat*delta_lat + delta_lon*delta_lon);
    
    double temp_down_track=stod(stringParserHelper(downtrack_str,downtrack_str.find_last_of("down_track:")));
    double temp_up_track=stod(stringParserHelper(uptrack_str,uptrack_str.find_last_of("up_track:")));
    double temp_min_gap=stod(stringParserHelper(min_gap_str,min_gap_str.find_last_of("min_gap:")));
    double temp_speed_advisory=stod(stringParserHelper(speed_advisory_str,speed_advisory_str.find_last_of("advisory_speed:")));
    std::string temp_event_reason=stringParserHelper(event_reason_str,event_reason_str.find_last_of("event_reason:"));
    std::string temp_event_type=stringParserHelper(event_type_str,event_type_str.find_last_of("event_type:"));

    if ( approximate_degree_delta < APPROXIMATE_DEG_PER_5M // If the vehicle has not moved more than 5m and the parameters remain unchanged
      && temp_down_track == down_track 
      && temp_up_track == up_track 
      && temp_min_gap == min_gap 
      && temp_speed_advisory == speed_advisory 
      && temp_event_reason == event_reason 
      && temp_event_type == event_type) {
      
      ROS_DEBUG_STREAM("Strategy params are unchanged so ignoring new message: " << mobility_strategy_params
        << " degree_delta: " << approximate_degree_delta 
        << " prev_lat: " << latitude << " prev_lon: " << longitude);

      return false;

    }

    // Valid message contents so populate member variables
    latitude = temp_lat;
    longitude = temp_lon;
    down_track = temp_down_track;
    up_track = temp_up_track;
    min_gap = temp_min_gap;
    speed_advisory = temp_speed_advisory;
    event_reason = temp_event_reason;
    event_type = temp_event_type;
    
    return true;
  }

  std::string TrafficIncidentParserWorker::stringParserHelper(std::string str, unsigned long str_index) const
  {
    std::string str_temp="";
    for(int i=str_index+1;i<str.length();i++)
    {
        str_temp+=str[i];
    }
    return str_temp;
  }

 lanelet::BasicPoint2d TrafficIncidentParserWorker::getIncidentOriginPoint() const
  {
    lanelet::projection::LocalFrameProjector projector(projection_msg_.c_str());
    lanelet::GPSPoint gps_point;
    gps_point.lat = latitude;
    gps_point.lon = longitude;
    gps_point.ele = 0;
    auto local_point3d = projector.forward(gps_point);
    return {local_point3d.x(), local_point3d.y()};
  }

  /**
   * \brief Helper method to get the nearest point index of a point and a linestring
   */ 
  size_t getNearestPointIndex(const lanelet::ConstLineString3d& points,
                                                const lanelet::BasicPoint2d& point)
  {
    double min_distance = std::numeric_limits<double>::max();
    size_t i = 0;
    size_t best_index = 0;
    for (const auto& p : points)
    {
      double distance = lanelet::geometry::distance2d(p, point);
      if (distance < min_distance)
      {
        best_index = i;
        min_distance = distance;
      }
      i++;
    }

    return best_index;
  }

  void TrafficIncidentParserWorker::getAdjacentForwardCenterlines(const lanelet::ConstLanelets& adjacentSet,
    const lanelet::BasicPoint2d& start_point, double downtrack, std::vector<std::vector<lanelet::BasicPoint2d>>* forward_lanes)
  {
    ROS_DEBUG_STREAM("getAdjacentForwardCenterlines");
    for (auto ll : adjacentSet) {
      ROS_DEBUG_STREAM("Processing adjacent lanelet: " << ll.id());
      std::vector<lanelet::BasicPoint2d> following_lane;
      auto cur_ll = ll;
      double dist = 0;
      
      
      // Identify the point to start the accumulation from
      size_t p_idx = getNearestPointIndex(cur_ll.centerline(), start_point); // Get the index of the nearest point

      lanelet::BasicPoint2d prev_point = cur_ll.centerline()[p_idx].basicPoint2d();

      ROS_DEBUG_STREAM("nearest point" << cur_ll.centerline()[p_idx].id() << " : " << prev_point.x() << ", " << prev_point.y());
      
      ROS_DEBUG_STREAM("p_idx: " << p_idx);

      // Accumulate distance
      while (dist < downtrack) {
        ROS_DEBUG_STREAM("Accumulating lanelet: " << cur_ll.id());
        if (p_idx == cur_ll.centerline().size()) {
          auto next_lls = wm_->getMapRoutingGraph()->following(cur_ll, false);
          if (next_lls.size() == 0) {
            ROS_DEBUG_STREAM("No followers");
            break;
          }
          auto next = next_lls[0];
          ROS_DEBUG_STREAM("Getting next lanelet: " << next.id());
          cur_ll = next;
          p_idx = 0;
        }
        if (p_idx != 0 || dist == 0) {
          
          following_lane.push_back(lanelet::traits::to2D(cur_ll.centerline()[p_idx]));
          dist += lanelet::geometry::distance2d(prev_point, following_lane.back());
        }
        ROS_DEBUG_STREAM("distance " << dist);
        prev_point = lanelet::traits::to2D(cur_ll.centerline()[p_idx]);
        p_idx++;
      }
      ROS_DEBUG_STREAM("Adding lane with size: " << following_lane.size());
      forward_lanes->emplace_back(following_lane);
    }
  }

  void TrafficIncidentParserWorker::getAdjacentReverseCenterlines(const lanelet::ConstLanelets& adjacentSet,
    const lanelet::BasicPoint2d& start_point, double uptrack, std::vector<std::vector<lanelet::BasicPoint2d>>* reverse_lanes)
  {
    ROS_DEBUG_STREAM("getAdjacentReverseCenterlines");
    for (auto ll : adjacentSet) {
      ROS_DEBUG_STREAM("Processing adjacent lanelet: " << ll.id());
      std::vector<lanelet::BasicPoint2d> previous_lane;
      auto cur_ll = ll;
      double dist = 0;
      
      
      // Identify the point to start the accumulation from
      int p_idx = getNearestPointIndex(cur_ll.centerline(), start_point); // Get the index of the nearest point

      lanelet::BasicPoint2d prev_point = cur_ll.centerline()[p_idx].basicPoint2d();

      ROS_DEBUG_STREAM("nearest point" << cur_ll.centerline()[p_idx].id() << " : " << prev_point.x() << ", " << prev_point.y());
      
      ROS_DEBUG_STREAM("p_idx: " << p_idx);

      // Accumulate distance
      while (dist < uptrack) {
        ROS_DEBUG_STREAM("Accumulating lanelet: " << cur_ll.id());
        if (p_idx == 0) {
          auto next_lls = wm_->getMapRoutingGraph()->previous(cur_ll, false);
          if (next_lls.size() == 0) {
            ROS_DEBUG_STREAM("No previous lanelets");
            break;
          }
          auto next = next_lls[0];
          ROS_DEBUG_STREAM("Getting next lanelet: " << next.id());
          cur_ll = next;
          p_idx = cur_ll.centerline().size() - 1;
        }
        if (p_idx != cur_ll.centerline().size() - 1 || dist == 0) {
          
          previous_lane.push_back(lanelet::traits::to2D(cur_ll.centerline()[p_idx]));
          dist += lanelet::geometry::distance2d(prev_point, previous_lane.back());
        }
        ROS_DEBUG_STREAM("distance " << dist);
        prev_point = lanelet::traits::to2D(cur_ll.centerline()[p_idx]);
        p_idx--;
      }
      ROS_DEBUG_STREAM("Adding lane with size: " << previous_lane.size());
      reverse_lanes->emplace_back(previous_lane);
    }
  }

  std::vector<cav_msgs::TrafficControlMessageV01> TrafficIncidentParserWorker::composeTrafficControlMesssages()
  {
    ROS_DEBUG_STREAM("In composeTrafficControlMesssages");
    if(!wm_->getMap())
    {
      ROS_WARN_STREAM("Traffic Incident Parser received traffic control message, but it has not loaded the map yet. Returning empty list");
      return {};
    }
    local_point_=getIncidentOriginPoint();
    ROS_DEBUG_STREAM("Responder point in map frame: " << local_point_.x() << ", " << local_point_.y());
    auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, local_point_, 1); 
    if (current_lanelets.size() == 0) {
      ROS_ERROR_STREAM("No nearest lanelet to responder vehicle in map point: " << local_point_.x() << ", " << local_point_.y());
      return {};
    }
     
    lanelet::ConstLanelet current_lanelet = current_lanelets[0].second;

    ROS_DEBUG_STREAM("Nearest Lanelet: " << current_lanelet.id());

    lanelet::ConstLanelets lefts = { current_lanelet };
    for (const auto& l : wm_->getMapRoutingGraph()->lefts(current_lanelet)) {
      lefts.emplace_back(l);
      ROS_DEBUG_STREAM("Left lanelet: " << l.id());
    }
    
    lanelet::ConstLanelets rights = wm_->getMapRoutingGraph()->rights(current_lanelet);
    for (auto l : rights) {
      ROS_DEBUG_STREAM("Right lanelet: " << l.id());
    }

    // Assume that if there are more lanelets to the left than the right then the tahoe is on the left
    std::vector<std::vector<lanelet::BasicPoint2d>> forward_lanes, reverse_lanes;

    if (lefts.size() >=  rights.size()) {
      ROS_DEBUG_STREAM("Emergency vehicle on the right ");
      getAdjacentForwardCenterlines(lefts, local_point_, down_track, &forward_lanes);
      getAdjacentReverseCenterlines(lefts, local_point_, up_track, &reverse_lanes);     
    } else {
      ROS_DEBUG_STREAM("Emergency vehicle on the left ");
      getAdjacentForwardCenterlines(rights, local_point_, down_track, &forward_lanes);
      getAdjacentReverseCenterlines(rights, local_point_, up_track, &reverse_lanes);   
    }

    for (auto& lane : reverse_lanes) {
      std::reverse(lane.begin(), lane.end()); // Reverse the backward points
    }
  
    // Compine results
    for (size_t i = 0; i < reverse_lanes.size(); i++) {
      if (forward_lanes[i].size() > 1) {
        reverse_lanes[i].insert(reverse_lanes[i].end(), forward_lanes[i].begin() + 1, forward_lanes[i].end()); // Concat linestirngs but drop the shared point
      }
    }

    
   
    ROS_DEBUG_STREAM("Constructing message for lanes: " << reverse_lanes.size());
    std::vector<cav_msgs::TrafficControlMessageV01> output_msg;

    cav_msgs::TrafficControlMessageV01 traffic_mobility_msg;
  
    traffic_mobility_msg.geometry_exists=true;
    traffic_mobility_msg.params_exists=true;
    traffic_mobility_msg.package_exists=true;
    j2735_msgs::TrafficControlVehClass veh_type;
    veh_type.vehicle_class = j2735_msgs::TrafficControlVehClass::ANY; // TODO decide what vehicle is affected
    traffic_mobility_msg.params.vclasses.push_back(veh_type);
    traffic_mobility_msg.params.schedule.start=ros::Time::now();
    traffic_mobility_msg.params.schedule.end_exists=false;
    traffic_mobility_msg.params.schedule.dow_exists=false;
    traffic_mobility_msg.params.schedule.between_exists=false;
    traffic_mobility_msg.params.schedule.repeat_exists = false;
    
    ////
    // Begin handling of projection definition
    // This logic works by enforcing the ROS message specifications for TrafficControlMessage on the output data
    // First the latlon point in the provided data is identified, then a projection with a north east oriented tmerc frame is created to compute node locations
    ////
    std::string common_frame = "WGS84"; // Common frame to use for lat/lon definition. This will populate the datum field of the message. A more complex CRS should not be used here

    PJ* common_to_map_proj = proj_create_crs_to_crs(PJ_DEFAULT_CTX, common_frame.c_str(), projection_msg_.c_str() , NULL); // Create transformation between map frame and common frame. Reverse here takes map->latlon. Froward is latlon->map

    if (common_to_map_proj == nullptr) { // proj_create_crs_to_crs returns 0 when there is an error in the projection
    
      ROS_ERROR_STREAM("Failed to generate projection between map  georeference and common frame with error number: " <<  proj_context_errno(PJ_DEFAULT_CTX) 
        << " projection_msg_: " << projection_msg_ << " common_frame: " << common_frame);

      return {}; // Ignore geofence if it could not be projected into the map frame
    }

    PJ_COORD map_origin_map_frame{{0.0, 0.0, 0.0, 0.0}}; // Map origin to use as ref lat/lon
    PJ_COORD map_origin_in_common_frame;

    map_origin_in_common_frame = proj_trans(common_to_map_proj, PJ_INV, map_origin_map_frame);

    traffic_mobility_msg.geometry.datum = "WGS84";
    traffic_mobility_msg.geometry.reflat = map_origin_in_common_frame.lpz.lam;
    traffic_mobility_msg.geometry.reflon = map_origin_in_common_frame.lpz.phi;

    std::ostringstream lat_string;
    std::ostringstream lon_string;

    lat_string.precision(14);
    lat_string << std::fixed << traffic_mobility_msg.geometry.reflat;
    
    lon_string.precision(14);
    lon_string << std::fixed << traffic_mobility_msg.geometry.reflon;
   
    // Create a local transverse mercator frame at the reference point to allow us to get east,north oriented data reguardless of map projection orientation 
    // This is needed to match the TrafficControlMessage specification
    std::string local_tmerc_enu_proj = "+proj=tmerc +datum=WGS84 +h_0=0 +lat_0=" + lat_string.str() + " +lon_0=" + lon_string.str() + " +k=1 +x_0=0 +y_0=0 +units=m +vunits=m +no_defs";

    PJ* map_to_tmerc_proj = proj_create_crs_to_crs(PJ_DEFAULT_CTX, projection_msg_.c_str(), local_tmerc_enu_proj.c_str() , NULL); // Create transformation between the common frame and the local ENU oriented frame
    
    if (map_to_tmerc_proj == nullptr) { // proj_create_crs_to_crs returns 0 when there is an error in the projection
    
      ROS_ERROR_STREAM("Failed to generate projection between map  georeference and tmerc frame with error number: " <<  proj_context_errno(PJ_DEFAULT_CTX) 
        << " projection_msg_: " << projection_msg_ << " local_tmerc_enu_proj: " << local_tmerc_enu_proj);

      return {}; // Ignore geofence if it could not be projected into the map frame
    }

    traffic_mobility_msg.geometry.proj = local_tmerc_enu_proj;

    ROS_DEBUG_STREAM("Projection in message: " << traffic_mobility_msg.geometry.proj);

    ////
    // Projections setup. Next projections will be used for node computation
    ////    

    for (size_t i = 0; i < reverse_lanes.size(); i++) {

      traffic_mobility_msg.geometry.nodes.clear();
      if (reverse_lanes[i].size() == 0) {
        ROS_DEBUG_STREAM("Skipping empty lane");
        continue;
      }


      PJ_COORD map_pt{{reverse_lanes[i].front().x(), reverse_lanes[i].front().y(), 0.0, 0.0}}; // Map point to convert to tmerc frame

      PJ_COORD tmerc_pt = proj_trans(map_to_tmerc_proj, PJ_FWD, map_pt);

      cav_msgs::PathNode prev_point;
      prev_point.x = tmerc_pt.xyz.x;
      prev_point.y = tmerc_pt.xyz.y;
      bool first = true;
      for(const auto& p : carma_utils::containers::downsample_vector(reverse_lanes[i], 8))
      {
        cav_msgs::PathNode delta;

        map_pt = PJ_COORD({p.x(), p.y(), 0.0, 0.0}); // Map point to convert to tmerc frame
        tmerc_pt = proj_trans(map_to_tmerc_proj, PJ_FWD, map_pt); // Convert point to tmerc frame

        delta.x=tmerc_pt.xyz.x - prev_point.x;
        delta.y=tmerc_pt.xyz.y - prev_point.y;

        ROS_DEBUG_STREAM("prev_point x" << prev_point.x << ", prev_point y " << prev_point.y);
        ROS_DEBUG_STREAM("tmerc_pt.xyz.x" << tmerc_pt.xyz.x << ", tmerc_pt.xyz.y " << tmerc_pt.xyz.y);
        ROS_DEBUG_STREAM("map_pt x" << p.x() << ", map_pt y " << p.y());

        ROS_DEBUG_STREAM("calculated diff x" << delta.x << ", diff y" << delta.y);
        if (first)
        {
          traffic_mobility_msg.geometry.nodes.push_back(prev_point); 
          first = false;
        }
        else
        {
          traffic_mobility_msg.geometry.nodes.push_back(delta);
        }
        
        prev_point.x = p.x();
        prev_point.y = p.y();
      }

      if (i == 0) {
        boost::uuids::uuid closure_id = boost::uuids::random_generator()();
        std::copy(closure_id.begin(), closure_id.end(), traffic_mobility_msg.id.id.begin());
        traffic_mobility_msg.params.detail.choice=cav_msgs::TrafficControlDetail::CLOSED_CHOICE;
        traffic_mobility_msg.params.detail.closed=cav_msgs::TrafficControlDetail::CLOSED;
        traffic_mobility_msg.package.label=event_reason;
        output_msg.push_back(traffic_mobility_msg);
      }

      boost::uuids::uuid headway_id = boost::uuids::random_generator()();
      std::copy(headway_id.begin(), headway_id.end(), traffic_mobility_msg.id.id.begin());
      traffic_mobility_msg.params.detail.choice=cav_msgs::TrafficControlDetail::MINHDWY_CHOICE;
      traffic_mobility_msg.params.detail.minhdwy=min_gap;
      output_msg.push_back(traffic_mobility_msg);

      boost::uuids::uuid speed_id = boost::uuids::random_generator()();
      std::copy(speed_id.begin(), speed_id.end(), traffic_mobility_msg.id.id.begin());
      traffic_mobility_msg.params.detail.choice=cav_msgs::TrafficControlDetail::MAXSPEED_CHOICE;
      traffic_mobility_msg.params.detail.maxspeed=speed_advisory;
      output_msg.push_back(traffic_mobility_msg);

    }
    

    return output_msg;
   
    }
  
  void TrafficIncidentParserWorker::newGeofenceSubscriber(const ros::SingleSubscriberPublisher& single_sub_pub) const {

    for (const auto& msg : geofence_message_queue_) {
      single_sub_pub.publish(msg); // For each applied update for the current map version publish the update to the new subscriber
    }
  }

}//traffic