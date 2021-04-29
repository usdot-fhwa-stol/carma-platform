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

namespace traffic
{

  TrafficIncidentParserWorker::TrafficIncidentParserWorker(carma_wm::WorldModelConstPtr wm,const PublishTrafficControlCallback &traffic_control_pub) : traffic_control_pub_(traffic_control_pub),wm_(wm){};

  void TrafficIncidentParserWorker::mobilityOperationCallback(const cav_msgs::MobilityOperation &mobility_msg)
  {
   
        if((mobility_msg.strategy=="carma3/Incident_Use_Case") && (mobility_msg.strategy_params!=previous_strategy_params))
      { 
           previous_strategy_params=mobility_msg.strategy_params;
           mobilityMessageParser(mobility_msg.strategy_params);

           if(event_type=="CLOSED")
           {
            cav_msgs::TrafficControlMessage traffic_control_msg;
            traffic_control_msg.choice=cav_msgs::TrafficControlMessage::TCMV01;
            for(auto &traffic_msg:composeTrafficControlMesssages())
            {
            traffic_control_msg.tcmV01=traffic_msg;
            traffic_control_pub_(traffic_control_msg);
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


  void TrafficIncidentParserWorker::mobilityMessageParser(std::string mobility_strategy_params)
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
      return;
    }  

    std::string lat_str=vec[0];
    std::string lon_str=vec[1];
    std::string downtrack_str=vec[2];
    std::string uptrack_str=vec[3];
    std::string min_gap_str=vec[4];
    std::string speed_advisory_str=vec[5];
    std::string event_reason_str=vec[6];
    std::string event_type_str=vec[7];

    latitude=stod(stringParserHelper(lat_str,lat_str.find_last_of("lat:")));
    longitude=stod(stringParserHelper(lon_str,lon_str.find_last_of("lon:")));
    down_track=stod(stringParserHelper(downtrack_str,downtrack_str.find_last_of("down_track:")));
    up_track=stod(stringParserHelper(uptrack_str,uptrack_str.find_last_of("up_track:")));
    min_gap=stod(stringParserHelper(min_gap_str,min_gap_str.find_last_of("min_gap:")));
    speed_advisory=stod(stringParserHelper(speed_advisory_str,speed_advisory_str.find_last_of("advisory_speed:")));
    event_reason=stringParserHelper(event_reason_str,event_reason_str.find_last_of("event_reason:"));
    event_type=stringParserHelper(event_type_str,event_type_str.find_last_of("event_type:"));
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

  std::vector<cav_msgs::TrafficControlMessageV01> TrafficIncidentParserWorker::composeTrafficControlMesssages()
  {
    ROS_DEBUG_STREAM("In composeTrafficControlMesssages");
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
    for (const auto&& l : wm_->getMapRoutingGraph()->lefts(current_lanelet)) {
      lefts.emplace_back(l);
      ROS_DEBUG_STREAM("Left lanelet: " << l.id());
    }
    
    lefts.push_back(current_lanelet);
    lanelet::ConstLanelets rights = wm_->getMapRoutingGraph()->rights(current_lanelet);
    for (auto l : rights) {
      ROS_DEBUG_STREAM("Right lanelet: " << l.id());
    }

    std::vector<std::vector<lanelet::BasicPoint2d>> forward_lanes; // Ordered from right to left
    for (auto ll : lefts) {
      ROS_DEBUG_STREAM("Processing left lanelet: " << ll.id());
      std::vector<lanelet::BasicPoint2d> following_lane;
      auto cur_ll = ll;
      double dist = 0;
      size_t p_idx = 0;
      lanelet::BasicPoint2d prev_point = lanelet::traits::to2D(cur_ll.centerline().front());
      while (dist < down_track) {
        ROS_DEBUG_STREAM("Accumulating left lanelet: " << cur_ll.id());
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
      if (following_lane.size() != 0) {
        ROS_DEBUG_STREAM("Adding lane");
        forward_lanes.emplace_back(following_lane);
      }
    }

   // auto route_lanelets = wm_->getLaneletsBetween(route_trackpos_min, route_trackpos_max, false);
    ROS_DEBUG_STREAM("Constructing message for lanes: " << forward_lanes.size());
    std::vector<cav_msgs::TrafficControlMessageV01> output_msg;

    cav_msgs::TrafficControlMessageV01 traffic_mobility_msg;
  
    traffic_mobility_msg.geometry_exists=true;
    traffic_mobility_msg.params_exists=true;
    traffic_mobility_msg.package_exists=true;
    j2735_msgs::TrafficControlVehClass veh_type;
    veh_type.vehicle_class = j2735_msgs::TrafficControlVehClass::ANY; // TODO decide what vehicle is affected
    traffic_mobility_msg.params.vclasses.push_back(veh_type);
    traffic_mobility_msg.geometry.proj=projection_msg_;
    traffic_mobility_msg.params.schedule.start=ros::Time::now();
    traffic_mobility_msg.params.schedule.end_exists=false;
    traffic_mobility_msg.params.schedule.dow_exists=false;
    traffic_mobility_msg.params.schedule.between_exists=false;
    traffic_mobility_msg.params.schedule.repeat_exists = false;

    for (size_t i = 0; i < forward_lanes.size(); i++) {
      //forward_lanes[i] = carma_utils::containers::downsample_vector(forward_lanes[i], 8);

      traffic_mobility_msg.geometry.nodes.clear();

      for(const auto& p : carma_utils::containers::downsample_vector(forward_lanes[i], 8))
      {
        cav_msgs::PathNode path_point;
        path_point.x=p.x();
        path_point.y=p.y();
        traffic_mobility_msg.geometry.nodes.push_back(path_point);
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

}//traffic