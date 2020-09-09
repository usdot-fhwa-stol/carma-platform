
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


#include <carma_wm/WMTestLibForGuidance.h>

namespace carma_wm
{
namespace test
{

void addObstacle(double x, double y, std::shared_ptr<carma_wm::CARMAWorldModel> cmw, std::vector<std::pair<double,double>> pred_coords , double width, double length)
{
    cav_msgs::RoadwayObstacle rwo;	

    tf2::Quaternion tf_orientation;	
    tf_orientation.setRPY(0, 0, 1.5708); // 90 deg

    rwo.object.pose.pose.position.x = x;	
    rwo.object.pose.pose.position.y = y;	
    rwo.object.pose.pose.position.z = 0;	

    rwo.object.pose.pose.orientation.x = tf_orientation.getX();	
    rwo.object.pose.pose.orientation.y = tf_orientation.getY();	
    rwo.object.pose.pose.orientation.z = tf_orientation.getZ();	
    rwo.object.pose.pose.orientation.w = tf_orientation.getW();	

    rwo.object.size.x = width;	
    rwo.object.size.y = length;	
    rwo.object.size.z = 1;	

    int time_stamp = 0;
    std::vector<cav_msgs::PredictedState> pred_states;
    for (auto pred : pred_coords)
    {
        cav_msgs::PredictedState ps;
        time_stamp += 1000;
        ps.header.stamp.nsec = time_stamp;

        ps.predicted_position.position.x = pred.first;	
        ps.predicted_position.position.y = pred.second;	
        ps.predicted_position.position.z = 0;	

        ps.predicted_position.orientation.x = tf_orientation.getX();	
        ps.predicted_position.orientation.y = tf_orientation.getY();	
        ps.predicted_position.orientation.z = tf_orientation.getZ();	
        ps.predicted_position.orientation.w = tf_orientation.getW();	
        pred_states.push_back(ps);
    }

    rwo.object.predictions = pred_states;

    // populate current and predicted lanelet_id, cross_track, downtracks 
    rwo = cmw->toRoadwayObstacle(rwo.object).get();

    std::vector<cav_msgs::RoadwayObstacle> rw_objs = cmw->getRoadwayObjects();	

    rw_objs.push_back(rwo);	

    cmw->setRoadwayObjects(rw_objs);	
}

// Assumes on the road, and assumes that it is the map returned by getGuidanceTestMap
void addObstacle(carma_wm::TrackPos tp, lanelet::Id lanelet_id, std::shared_ptr<carma_wm::CARMAWorldModel> cmw, std::vector<carma_wm::TrackPos> pred_trackpos_list, double width, double length)
{
    cav_msgs::RoadwayObstacle rwo;	

    if (!cmw->getMap() || cmw->getMap()->laneletLayer.size() == 0)
    {
      throw std::invalid_argument("Map is not set or does not contain lanelets");
    }
    rwo.connected_vehicle_type.type =
        cav_msgs::ConnectedVehicleType::NOT_CONNECTED;  // TODO No clear way to determine automation state at this time
    
    // get id of specified tp lanelet
    // it assumes the map from getGuidanceTestMap
    auto reference_llt = cmw->getMutableMap()->laneletLayer.get(lanelet_id);
    lanelet::BasicPoint2d object_center = {(reference_llt.leftBound()[0].x() + reference_llt.rightBound()[0].x())/2 + tp.crosstrack,
                                          (reference_llt.leftBound()[0].y() + reference_llt.rightBound()[0].y())/2 + tp.downtrack};    

    auto nearestLanelet = cmw->getMap()->laneletLayer.nearest(object_center, 1)[0]; 
    if (!boost::geometry::within(object_center, nearestLanelet.polygon2d()))
    {
      throw std::invalid_argument("Given trackpos from given lanelet id does land on any lanelet in the map");
    }
    rwo.lanelet_id = nearestLanelet.id();

    rwo.down_track = tp.downtrack;
    rwo.cross_track = tp.crosstrack;

    int time_stamp = 0;
    std::vector<cav_msgs::PredictedState> pred_states;
    for (auto pred_track_pos : pred_trackpos_list)
    {
      // record time intervals
      cav_msgs::PredictedState ps;
      time_stamp += 1000;
      ps.header.stamp.nsec = time_stamp;

      auto ref_llt_pred = cmw->getMutableMap()->laneletLayer.get(lanelet_id);
      lanelet::BasicPoint2d object_center_pred = {(ref_llt_pred.leftBound()[0].x() + ref_llt_pred.rightBound()[0].x())/2 + pred_track_pos.crosstrack,
                                            (ref_llt_pred.leftBound()[0].y() + ref_llt_pred.rightBound()[0].y())/2 + pred_track_pos.downtrack};    

      auto predNearestLanelet = cmw->getMap()->laneletLayer.nearest(object_center_pred, 1)[0]; 
      if (!boost::geometry::within(object_center_pred, predNearestLanelet.polygon2d()))
      {
        std::cerr << "Given pred trackpos from given lanelet id does land on any lanelet in the map" << std::endl;
      }
      rwo.predicted_lanelet_ids.emplace_back(predNearestLanelet.id());
      rwo.predicted_cross_tracks.emplace_back(pred_track_pos.crosstrack);
      rwo.predicted_down_tracks.emplace_back(pred_track_pos.downtrack);

      pred_states.push_back(ps);
    }
    // add time intervals
    rwo.object.predictions = pred_states;
    std::vector<cav_msgs::RoadwayObstacle> rw_objs = cmw->getRoadwayObjects();	

    rw_objs.push_back(rwo);	

    cmw->setRoadwayObjects(rw_objs);	
}

}
  //namespace test
} //namespace carma_wm