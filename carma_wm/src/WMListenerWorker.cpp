/*
 * Copyright (C) 2019 LEIDOS.
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

#include <lanelet2_extension/utility/message_conversion.h>
#include "WMListenerWorker.h"
#include <algorithm>

namespace carma_wm
{
WMListenerWorker::WMListenerWorker()
{
  world_model_.reset(new CARMAWorldModel);
}

WorldModelConstPtr WMListenerWorker::getWorldModel() const
{
  return std::static_pointer_cast<const WorldModel>(world_model_);  // Cast pointer to const variant
}

void WMListenerWorker::mapCallback(const autoware_lanelet2_msgs::MapBinConstPtr& map_msg)
{
  lanelet::LaneletMapPtr new_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*map_msg, new_map);

  world_model_->setMap(new_map);

  // Call user defined map callback
  if (map_callback_)
  {
    map_callback_();
  }
}

void WMListenerWorker::mapUpdateCallback(const autoware_lanelet2_msgs::MapBinConstPtr& geofence_msg)
{
  // convert ros msg to geofence object
  auto gf_ptr = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  carma_wm::fromGeofenceBinMsg(*geofence_msg, gf_ptr);

  ROS_WARN_STREAM("MAP REMOVE REQUEST RECEIVED AT WMLISTENER! OF SIZE " << gf_ptr->remove_list_.size());
  // update the map
  for (auto pair : gf_ptr->remove_list_)
  {
    ROS_WARN_STREAM("look at it regem size: " << world_model_->getMutableMap()->laneletLayer.get(pair.first).regulatoryElements().size());
    for (auto regem : world_model_->getMutableMap()->laneletLayer.get(pair.first).regulatoryElements())
    {
      ROS_WARN_STREAM("The llt with id: "<< pair.first << " for this pair has regem id: " << regem->id());
    }
    ROS_WARN_STREAM("Before Using findUsage");
    ROS_WARN_STREAM("lanelet layer ID:" << world_model_->getMutableMap()->laneletLayer.findUsages(pair.second).size());
    ROS_WARN_STREAM("regem layer if exists:" << world_model_->getMutableMap()->regulatoryElementLayer.exists(pair.second->id()));
  }

  for (auto pair : gf_ptr->remove_list_)
  {
    ROS_WARN_STREAM("REMOVE LLT ID: " << pair.first << " REMOVE REGEM ID: " << pair.second->id());
    ROS_WARN_STREAM("Before Using findUsage");
    auto parent_llt = world_model_->getMutableMap()->laneletLayer.get(pair.first);
    // we can only check by id, if the element is there
    // this is only for speed optimization, as world model here should blindly accept the map update received
    for (auto regem: parent_llt.regulatoryElements())
    {
      if (pair.second->id() == regem->id()) 
      {
        world_model_->getMutableMap()->remove(parent_llt, regem);
        break;
      }
    }
  }

  ROS_WARN_STREAM("MAP UPDATE REQUEST RECEIVED AT WMLISTENER! OF SIZE:" << gf_ptr->update_list_.size());
  for (auto pair : gf_ptr->update_list_)
  {
    ROS_WARN_STREAM("UPDATE LLT ID: " << pair.first << " UPDATE REGEM ID: " << pair.second->id());
    auto parent_llt = world_model_->getMutableMap()->laneletLayer.get(pair.first);
    
    for (auto regem: parent_llt.regulatoryElements())
    {
      //we first need to remove it to update if it exists
      //so that we can insert the regem with updated data
      if (pair.second->id() == regem->id()) 
      {
        world_model_->getMutableMap()->remove(parent_llt, regem);
        world_model_->getMutableMap()->update(parent_llt, pair.second);
        break;
      }
    }
    
  }
  
  // set the map to set a new routing
  //world_model_->setMap(world_model_->getMutableMap());
  ROS_WARN_STREAM("MAP SET SUCCESSFULLY");
  ROS_WARN_STREAM("OR DID IT?:");
  for (auto pair : gf_ptr->update_list_)
  {
    ROS_WARN_STREAM("look at its regem size: " << world_model_->getMutableMap()->laneletLayer.get(pair.first).regulatoryElements().size());
    ROS_WARN_STREAM("Before Using findUsage");
    if (world_model_->getMutableMap()->laneletLayer.findUsages(lanelet::utils::toConst(pair.second)).size() != 0)
      ROS_WARN_STREAM("lanelet layer ID:" << world_model_->getMutableMap()->laneletLayer.findUsages(pair.second)[0].id());
    else
      ROS_WARN_STREAM("No llt is using this shit...");
  }
}

void WMListenerWorker::roadwayObjectListCallback(const cav_msgs::RoadwayObstacleList& msg)
{
  // this topic publishes only the objects that are on the road
  world_model_->setRoadwayObjects(msg.roadway_obstacles);
}

void WMListenerWorker::routeCallback()
{
  // TODO Implement when route message has been defined
  // world_model_->setRoute(route_obj);
  // Call route_callback_;
  if (route_callback_)
  {
    route_callback_();
  }
}

void WMListenerWorker::setMapCallback(std::function<void()> callback)
{
  map_callback_ = callback;
}

void WMListenerWorker::setRouteCallback(std::function<void()> callback)
{
  route_callback_ = callback;
}
}  // namespace carma_wm