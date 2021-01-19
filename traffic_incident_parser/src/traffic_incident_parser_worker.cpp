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

namespace traffic
{

  TrafficIncidentParserWorker::TrafficIncidentParserWorker(carma_wm::WorldModelConstPtr wm,const PublishTrafficControlCallback &traffic_control_pub) : traffic_control_pub_(traffic_control_pub),wm_(wm){};

  void TrafficIncidentParserWorker::mobilityOperationCallback(const cav_msgs::MobilityOperation &mobility_msg)
  {
   
        if((mobility_msg.strategy=="carma3/Incident_Use_Case") && (mobility_msg.strategy_params!=previous_strategy_params))
      { 
           previous_strategy_params=mobility_msg.strategy_params;
           mobilityMessageParser(mobility_msg.strategy_params);
           traffic_control_pub_(composeTrafficControlMesssage());
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

    if (vec.size() != 5) 
    {
      ROS_ERROR_STREAM("Given mobility strategy params are not correctly formatted.");
      return;
    }  

    std::string lat=vec[0];
    std::string lon=vec[1];
    std::string closed_lanes=vec[2];
    std::string downtrack=vec[3];
    std::string uptrack=vec[4];
    latitude=stod(stringParserHelper(lat,lat.find_last_of("lat:")));
    longitude=stod(stringParserHelper(lon,lon.find_last_of("lon:")));
    closed_lane=stoi((std::string)stringParserHelper(closed_lanes,closed_lanes.find_last_of("lat:")));
    down_track=stod((std::string)stringParserHelper(downtrack,downtrack.find_last_of("lon:")));
    up_track=stod((std::string)stringParserHelper(uptrack,uptrack.find_last_of("lon:")));
  }

  std::string TrafficIncidentParserWorker::stringParserHelper(std::string str,int str_index) const
  {
    std::string str_temp="";
    for(int i=str_index+1;i<=str.length();i++)
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

  cav_msgs::TrafficControlMessageV01 TrafficIncidentParserWorker::composeTrafficControlMesssage()
  {
    local_point_=getIncidentOriginPoint();
    auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, local_point_, 1);  
    lanelet::ConstLanelet current_lanelet = current_lanelets[0].second;
    double following_distance=0.0;
    double previous_distance=0.0;
    std::vector<lanelet::BasicPoint2d> center_line_points_right={};
    std::vector<lanelet::BasicPoint2d> center_line_points_left={};
 
    while(following_distance<down_track) //Extract the center line point towards the downtrack lanelets
    {
      auto next_lanelets = wm_->getMapRoutingGraph()->following(current_lanelet);
      if(!next_lanelets.empty())
      {
        carma_wm::TrackPos tp = carma_wm::geometry::trackPos(next_lanelets[0], local_point_);
        following_distance= std::fabs(tp.downtrack);
      }
      
      auto back_point = current_lanelet.centerline().back().basicPoint2d();
      auto front_point = current_lanelet.centerline().front().basicPoint2d();
      lanelet::BasicPoint2d middle = {(back_point.x() + front_point.x())/2, (back_point.y() + front_point.y())/2};
          
      center_line_points_right.push_back(middle);
      if(next_lanelets.empty())
      {
        break;
      }
      else
      {
      current_lanelet=next_lanelets[0];
      }
    }

    current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, local_point_, 1); 
    current_lanelet = current_lanelets[0].second;
    carma_wm::TrackPos tp0 = carma_wm::geometry::trackPos(current_lanelet, local_point_);
    previous_distance= std::fabs(tp0.downtrack);
    
   while(previous_distance<up_track) //Extract the center line point towards the uptrack lanelets
    {
      auto next_lanelets = wm_->getMapRoutingGraph()->previous(current_lanelet);
      if(!next_lanelets.empty())
      {
        carma_wm::TrackPos tp1 = carma_wm::geometry::trackPos(next_lanelets[0], local_point_);
        previous_distance= std::fabs(tp1.downtrack);
        auto back_point = next_lanelets[0].centerline().back().basicPoint2d();
        auto front_point = next_lanelets[0].centerline().front().basicPoint2d();
        lanelet::BasicPoint2d middle = {(back_point.x() + front_point.x())/2, (back_point.y() + front_point.y())/2};
        center_line_points_left.push_back(middle);
        current_lanelet=next_lanelets[0];
      }
       else
      {
        break;
      }
    }
   
    std::reverse(center_line_points_left.begin(), center_line_points_left.end());
   
    center_line_points_left.insert( center_line_points_left.end(), center_line_points_right.begin(), center_line_points_right.end() );
    cav_msgs::TrafficControlMessageV01 traffic_mobility_msg;
    
    traffic_mobility_msg.geometry_exists=true;
    traffic_mobility_msg.params.detail.choice=cav_msgs::TrafficControlDetail::CLOSED_CHOICE;
    traffic_mobility_msg.params.detail.closed=cav_msgs::TrafficControlDetail::CLOSED;
    
    for(auto &i:center_line_points_left)
    {
      cav_msgs::PathNode path_point;
      path_point.x=i.x();
      path_point.y=i.y();
      traffic_mobility_msg.geometry.nodes.push_back(path_point);
    }
      return traffic_mobility_msg;
    }

}//traffic