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

  TrafficIncidentParserWorker::TrafficIncidentParserWorker(carma_wm::WorldModelConstPtr wm,PublishTrafficControlCallback traffic_control_pub) : traffic_control_pub_(traffic_control_pub),wm_(wm){};

  void TrafficIncidentParserWorker::mobilityOperationCallback(const cav_msgs::MobilityOperation &mobility_msg)
  {

    if(mobility_msg.strategy=="carma3/Incident_Use_Case")
    {
          mobilityMessageParser(mobility_msg.strategy_params);
    }

  //cav_msgs::MobilityOperation traffic_mobility_msg=mobilityMessageGenerator(pinpoint_msg);
  traffic_control_pub_(traffic_mobility_msg);
  }

  void TrafficIncidentParserWorker::mobilityMessageParser(string mobility_strategy_params)
  {

   vector<string> vec={};
   std::string delimiter = ",";
   size_t pos = 0;
   std::string token;
    while ((pos = mobility_strategy_params.find(delimiter)) != std::string::npos) {
        token = mobility_strategy_params.substr(0, pos);
        vec.push_back(token);
        mobility_strategy_params.erase(0, pos + delimiter.length());
    }
   vec.push_back(mobility_strategy_params);

   string lat=vec[0];
   string lon=vec[1];
   string closed_lanes=vec[2];
   string downtrack=vec[3];
   string uptrack=vec[4];

   latitude_=stod(stringParserHelper(lat,lat.find_last_of("lat:")));
   longitude_=stod(stringParserHelper(lon,lon.find_last_of("lon:")));
   closed_lane_=stoi(stringParserHelper(closed_lanes,closed_lanes.find_last_of("lat:")));
   down_track_=stod(stringParserHelper(downtrack,downtrack.find_last_of("lon:")));
   up_track_=stod(stringParserHelper(uptrack,uptrack.find_last_of("lon:")));
   
  }

  string stringParserHelper(string str,int str_index)
  {
    string str_temp="";
    for(int i=str_index+1;i<=str.length();i++)
    {
        str_temp+=str[i];
    }
    return str_temp;
  }

  void TrafficIncidentParserWorker::projectionCallback(const std_msgs::String &projection_msg)
  {
      projection_msg_=projection_msg.data;
  }

  void TrafficIncidentParserWorker::earthToMapFrame()
  {
    lanelet::projection::LocalFrameProjector projector(projection_msg_.c_str());

    lanelet::GPSPoint gps_point;
    gps_point.lat = latitude_;
    gps_point.lon = longitude_;
    gps_point.ele = 0;
    local_point_ = projector.forward(gps_point);
  }

  void TrafficIncidentParserWorker::findNearByLanetlet()
  {
    //auto map = wm_->getMap();
    auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, local_point_, closed_lane_);  
    lanelet::ConstLanelet current_lanelet = current_lanelets[0].second;
    lanelet::LineString current_lanelet.centerline();
    current_lanelets.
    current_lanelets.

     carma_wm::TrackPos tp = carma_wm::geometry::trackPos(current_lanelet, local_point_);//Downtrack and cross track relative to lanelet starting position
  }

 /* cav_msgs::MobilityOperation TrafficIncidentWorker::mobilityMessageGenerator(const gps_common::GPSFix& pinpoint_msg)
  {
    cav_msgs::MobilityOperation traffic_mobility_msg;

    traffic_mobility_msg.header.timestamp=pinpoint_msg.header.stamp.sec*1000;
    traffic_mobility_msg.header.sender_id=sender_id_;

    traffic_mobility_msg.strategy="carma3/Incident_Use_Case";

    traffic_mobility_msg.strategy_params="lat:"+anytypeToString(pinpoint_msg.latitude)+","+"lon:"+anytypeToString(pinpoint_msg.longitude)+","+"closed_lanes:"+ closed_lane_ +","+"downtrack:"+anytypeToString(down_track_)+","+"uptrack:"+anytypeToString(up_track_);
  
    return traffic_mobility_msg;
  }*/
  
 /* void TrafficIncidentWorker::setSenderId(std::string sender_id)
  {
    sender_id_= sender_id;
  }

  void TrafficIncidentWorker::setClosedLane(std::string closed_lane)
  {
    closed_lane_= closed_lane;
  }

  void TrafficIncidentWorker::setDownTrack(double down_track)
  {
    down_track_= down_track;
  }

  void TrafficIncidentWorker::setUpTrack(double up_track)
  {
    up_track_= up_track;
  }*/

}//traffic