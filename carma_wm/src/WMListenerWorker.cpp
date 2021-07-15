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
#include <lanelet2_extension/regulatory_elements/DirectionOfTravel.h>
#include <lanelet2_extension/regulatory_elements/StopRule.h>
#include <lanelet2_extension/regulatory_elements/CarmaTrafficLight.h>
#include "WMListenerWorker.h"

namespace carma_wm
{
enum class GeofenceType{ INVALID, DIGITAL_SPEED_LIMIT, PASSING_CONTROL_LINE, REGION_ACCESS_RULE, DIGITAL_MINIMUM_GAP, DIRECTION_OF_TRAVEL, STOP_RULE, CARMA_TRAFFIC_LIGHT/* ... others */ };
// helper function that return geofence type as an enum, which makes it cleaner by allowing switch statement
GeofenceType resolveGeofenceType(const std::string& rule_name)
{
  if (rule_name.compare(lanelet::PassingControlLine::RuleName) == 0) return GeofenceType::PASSING_CONTROL_LINE;
  if (rule_name.compare(lanelet::DigitalSpeedLimit::RuleName) == 0) return GeofenceType::DIGITAL_SPEED_LIMIT;
  if (rule_name.compare(lanelet::RegionAccessRule::RuleName) == 0) return GeofenceType::REGION_ACCESS_RULE;
  if (rule_name.compare(lanelet::DigitalMinimumGap::RuleName) == 0) return GeofenceType::DIGITAL_MINIMUM_GAP;
  if (rule_name.compare(lanelet::DirectionOfTravel::RuleName) == 0) return GeofenceType::DIRECTION_OF_TRAVEL;
  if (rule_name.compare(lanelet::StopRule::RuleName) == 0) return GeofenceType::STOP_RULE;
  if (rule_name.compare(lanelet::CarmaTrafficLight::RuleName) == 0) return GeofenceType::CARMA_TRAFFIC_LIGHT;

  return GeofenceType::INVALID;
}

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
  current_map_version_ = map_msg->map_version;

  lanelet::LaneletMapPtr new_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*map_msg, new_map);

  world_model_->setMap(new_map, current_map_version_);

  // After setting map evaluate the current update queue to apply any updates that arrived before the map
  bool more_updates_to_apply = true;
  while(!map_update_queue_.empty() && more_updates_to_apply) {
    
    auto update = map_update_queue_.front(); // Get first update
    map_update_queue_.pop(); // Remove update from queue

    if (update->map_version < current_map_version_) { // Drop any so far unapplied updates for the previous map
      ROS_WARN_STREAM("There were unapplied updates in carma_wm when a new map was recieved.");
      continue;
    }
    if (update->map_version == current_map_version_) { // Current update goes with current map
      mapUpdateCallback(update); // Apply the update
    } else {
      ROS_INFO_STREAM("Done applying updates for new map. However, more updates are waiting for a future map.");
      more_updates_to_apply = false; // If there is more updates queued that are not for this map version assume they are for a future map version
    }

  }

  // Call user defined map callback
  if (map_callback_)
  {
    map_callback_();
  }

  if (delayed_route_msg_) {
    if (delayed_route_msg_.get()->map_version == current_map_version_) { // If there is a delayed route message to apply then do so
      routeCallback(delayed_route_msg_.get());
    } else if (delayed_route_msg_.get()->map_version < current_map_version_) {
      ROS_WARN_STREAM("Dropping delayed route message which was never applied as updated map was not recieved");
      delayed_route_msg_ = boost::none;
    } else {
      ROS_INFO_STREAM("There is a delayed route message still waiting to be applied in carma_wm");
    }
  }
}

void WMListenerWorker::incomingSpatCallback(const cav_msgs::SPAT& spat_msg)
{
  if (spat_msg.intersection_state_list.empty())
  {
    ROS_WARN_STREAM("No intersection_state_list in the newly received SPAT msg. Returning...");
    return;
  }
  for (auto curr_intersection : spat_msg.intersection_state_list)
  {
    for (auto current_movement_state : curr_intersection.movement_list) 
    {
      lanelet::Id curr_light_id = world_model_->getTrafficLightId(curr_intersection.id.id, current_movement_state.signal_group);
      if (curr_light_id == lanelet::InvalId)
      {
        ROS_WARN_STREAM("Received a SPAT message for traffic light that is not in the map with intersection_id: " << curr_intersection.id.id << 
                           ", and signal_group_id: " << current_movement_state.signal_group);
        continue;
      }
      auto general_regem = world_model_->getMutableMap()->regulatoryElementLayer.get(curr_light_id);
      
      // get ptr to the regem in the map
      lanelet::CarmaTrafficLightPtr curr_light = world_model_->getMutableMap()->laneletLayer.findUsages(general_regem)[0].regulatoryElementsAs<lanelet::CarmaTrafficLight>()[0];

      // check if we have processed this already or not
      if (curr_light->revision_ == curr_intersection.revision)
      {
        continue;
      }
      std::vector<std::pair<ros::Time, lanelet::CarmaTrafficLightState>> new_states;
      for (auto event : current_movement_state.movement_event_list)
      {
        new_states.push_back(std::make_pair(ros::Time(event.timing.min_end_time), static_cast<lanelet::CarmaTrafficLightState>(event.event_state.movement_phase_state)));
      }
      curr_light->setStates(new_states, curr_intersection.revision);
    }
  }
}

bool WMListenerWorker::checkIfReRoutingNeeded() const
{
  return rerouting_flag_;
}

void WMListenerWorker::enableUpdatesWithoutRoute()
{
  route_node_flag_=true;
}

void WMListenerWorker::mapUpdateCallback(const autoware_lanelet2_msgs::MapBinPtr& geofence_msg)
{
  ROS_INFO_STREAM("Map Update Being Evaluated. SeqNum: " << geofence_msg->header.seq);

  if (geofence_msg->header.seq <= most_recent_update_msg_seq_) {
    ROS_DEBUG_STREAM("Dropping map update which has already been processed. Received seq: " << geofence_msg->header.seq << " prev seq: " << most_recent_update_msg_seq_);
    return;
  } else if(!world_model_->getMap() || current_map_version_ < geofence_msg->map_version) { // If our current map version is older than the version target by this update
    ROS_DEBUG_STREAM("Update recieved for newer map version than available. Queueing update until map is available.");
    map_update_queue_.push(geofence_msg);
    return;
  } else if (current_map_version_ > geofence_msg->map_version) { // If this update is for an older map
    ROS_WARN_STREAM("Dropping old map update as newer map is already available.");
    return;
  }

  most_recent_update_msg_seq_ = geofence_msg->header.seq; // Update current sequence count

  if(geofence_msg->invalidates_route==true && world_model_->getRoute())
  {  
    rerouting_flag_=true;
    ROS_DEBUG_STREAM("Received notice that route has been invalidated in mapUpdateCallback");

    if(route_node_flag_!=true)
    {
     ROS_INFO_STREAM("Route is not yet available. Therefore queueing the update");
     map_update_queue_.push(geofence_msg);
     return;
    }
  }
  // convert ros msg to geofence object
  auto gf_ptr = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  carma_wm::fromBinMsg(*geofence_msg, gf_ptr);

  ROS_INFO_STREAM("Processing Map Update with Geofence Id:" << gf_ptr->id_);

  ROS_DEBUG_STREAM("Geofence id" << gf_ptr->id_ << " requests removal of size: " << gf_ptr->remove_list_.size());
  for (auto pair : gf_ptr->remove_list_)
  {
    auto parent_llt = world_model_->getMutableMap()->laneletLayer.get(pair.first);
    // we can only check by id, if the element is there
    // this is only for speed optimization, as world model here should blindly accept the map update received
    auto regems_copy_to_check = parent_llt.regulatoryElements(); // save local copy as the regem can be deleted during iteration
    ROS_DEBUG_STREAM("Regems found in lanelet: " << regems_copy_to_check.size());
    for (auto regem: regems_copy_to_check)
    {
      // we can't use the deserialized element as its data address conflicts the one in this node
      if (pair.second->id() == regem->id()) world_model_->getMutableMap()->remove(parent_llt, regem);
    }
    ROS_DEBUG_STREAM("Regems left in lanelet after removal: " << parent_llt.regulatoryElements().size());

  }

  ROS_INFO_STREAM("Geofence id" << gf_ptr->id_ << " requests update of size: " << gf_ptr->update_list_.size());
  
  // we should extract general regem to specific type of regem the geofence specifies
  
  for (auto pair : gf_ptr->update_list_)
  {

    auto parent_llt = world_model_->getMutableMap()->laneletLayer.get(pair.first);

    auto regemptr_it = world_model_->getMutableMap()->regulatoryElementLayer.find(pair.second->id());

    // if this regem is already in the map.
    // This section is expected to be called to add back regulations which were previously removed by expired geofences.
    if (regemptr_it != world_model_->getMutableMap()->regulatoryElementLayer.end())  
    {

      ROS_DEBUG_STREAM("Reapplying previously existing element");
      // again we should use the element with correct data address to be consistent
      world_model_->getMutableMap()->update(parent_llt, *regemptr_it);
    }
    else // Updates are treated as new regulations after the old value was removed. In both cases we enter this block. 
    {

      ROS_DEBUG_STREAM("New regulatory element " << pair.second->id());
      newRegemUpdateHelper(parent_llt, pair.second.get());

    }
  }
  
  // set the map to set a new routing
  world_model_->setMap(world_model_->getMutableMap(), current_map_version_);

  
  ROS_INFO_STREAM("Finished Applying the Map Update with Geofence Id:" << gf_ptr->id_); 
}

/*!
  * \brief This is a helper function updates the parent_llt with specified regem. This function is needed
  *        as we need to dynamic_cast from general regem to specific type of regem based on the geofence
  * \param parent_llt The Lanelet that need to register the regem
  * \param regem lanelet::RegulatoryElement* which is the type that the serializer decodes from binary
  * NOTE: Currently this function supports items in carma_wm::GeofenceType
  */
void WMListenerWorker::newRegemUpdateHelper(lanelet::Lanelet parent_llt, lanelet::RegulatoryElement* regem) const
{

  auto factory_pcl = lanelet::RegulatoryElementFactory::create(regem->attribute(lanelet::AttributeName::Subtype).value(),
                                                            std::const_pointer_cast<lanelet::RegulatoryElementData>(regem->constData()));

  // we should extract general regem to specific type of regem the geofence specifies
  switch(resolveGeofenceType(regem->attribute(lanelet::AttributeName::Subtype).value()))
  {
    case GeofenceType::PASSING_CONTROL_LINE:
    {

      lanelet::PassingControlLinePtr control_line = std::dynamic_pointer_cast<lanelet::PassingControlLine>(factory_pcl);
      world_model_->getMutableMap()->update(parent_llt, control_line);

      break;
    }
    case GeofenceType::DIGITAL_SPEED_LIMIT:
    {

      lanelet::DigitalSpeedLimitPtr speed = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(factory_pcl);
      world_model_->getMutableMap()->update(parent_llt, speed);
      ROS_DEBUG_STREAM("updateed llt id:" << parent_llt.id() << ", with digital speed limit of: " << speed->speed_limit_.value()<<"in ms");
      break;
    }
    case GeofenceType::REGION_ACCESS_RULE:
    {

      lanelet::RegionAccessRulePtr rar = std::dynamic_pointer_cast<lanelet::RegionAccessRule>(factory_pcl);
      world_model_->getMutableMap()->update(parent_llt, rar);

      break;
    }
    case GeofenceType::DIGITAL_MINIMUM_GAP:
    {

      lanelet::DigitalMinimumGapPtr min_gap = std::dynamic_pointer_cast<lanelet::DigitalMinimumGap>(factory_pcl);
      world_model_->getMutableMap()->update(parent_llt, min_gap);

      break;
    }
    case GeofenceType::DIRECTION_OF_TRAVEL:
    {

      lanelet::DirectionOfTravelPtr dot = std::dynamic_pointer_cast<lanelet::DirectionOfTravel>(factory_pcl);
      world_model_->getMutableMap()->update(parent_llt, dot);

      break;
    }
    case GeofenceType::STOP_RULE:
    {

      lanelet::StopRulePtr sr = std::dynamic_pointer_cast<lanelet::StopRule>(factory_pcl);
      world_model_->getMutableMap()->update(parent_llt, sr);

      break;
    }
    case GeofenceType::CARMA_TRAFFIC_LIGHT:
    {

      lanelet::CarmaTrafficLightPtr ctl = std::dynamic_pointer_cast<lanelet::CarmaTrafficLight>(factory_pcl);
      world_model_->getMutableMap()->update(parent_llt, ctl);

      break;
    }
    default:
      ROS_WARN_STREAM("World Model instance received an unsupported geofence type in its map update callback!");
      break;
  }
}

void WMListenerWorker::roadwayObjectListCallback(const cav_msgs::RoadwayObstacleList& msg)
{
  // this topic publishes only the objects that are on the road
  world_model_->setRoadwayObjects(msg.roadway_obstacles);
}

void WMListenerWorker::routeCallback(const cav_msgs::RouteConstPtr& route_msg)
{
  if (route_msg->map_version < current_map_version_) {
    ROS_WARN_STREAM("Route message rejected as it is for an older map");
    rerouting_flag_ = false; // Clear any blockers on map updates as the route we were waiting for is no longer valid
    return;
  }

  if (route_msg->map_version > current_map_version_) {
    ROS_WARN_STREAM("Route message received for future map. Delaying application until map is recieved");
    delayed_route_msg_ = route_msg;
    return;
  }

  if(rerouting_flag_==true && route_msg->is_rerouted && !route_node_flag_)
  {

    // After setting map evaluate the current update queue to apply any updates that arrived before the map
    bool more_updates_to_apply = true;
    while(!map_update_queue_.empty() && more_updates_to_apply) {
      
      auto update = map_update_queue_.front(); // Get first update
      map_update_queue_.pop(); // Remove update from queue
      update->invalidates_route = false;

      if (update->map_version < current_map_version_) { // Drop any so far unapplied updates for the current map
        ROS_WARN_STREAM("Apply from reroute: There were unapplied updates in carma_wm when a new map was recieved.");
        continue;
      }
      if (update->map_version == current_map_version_) { // Current update goes with current map which is also the map used by this route
        ROS_DEBUG_STREAM("Applying queued update after route was recieved. ");
        mapUpdateCallback(update); // Apply the update
      } else {
        ROS_INFO_STREAM("Apply from reroute: Done applying updates for new map. However, more updates are waiting for a future map.");
        more_updates_to_apply = false; // If there is more updates queued that are not for this map version assume they are for a future map version
      }

    }

  }

  rerouting_flag_ = false;

  if (!world_model_->getMap()) { // This check is a bit redundant but still useful from a debugging perspective as the alternative is a segfault
    ROS_ERROR_STREAM("WMListener received a route before a map was available. Dropping route message.");
    return;
  }

  auto path = lanelet::ConstLanelets();
  for(auto id : route_msg->shortest_path_lanelet_ids)
  {
    auto ll = world_model_->getMap()->laneletLayer.get(id);
    path.push_back(ll);
  }
  if(path.empty()) return;
  auto route_opt = path.size() == 1 ? world_model_->getMapRoutingGraph()->getRoute(path.front(), path.back())
                               : world_model_->getMapRoutingGraph()->getRouteVia(path.front(), lanelet::ConstLanelets(path.begin() + 1, path.end() - 1), path.back());
  if(route_opt.is_initialized()) {
    auto ptr = std::make_shared<lanelet::routing::Route>(std::move(route_opt.get()));
    world_model_->setRoute(ptr);
  }

  world_model_->setRouteEndPoint({route_msg->end_point.x,route_msg->end_point.y,route_msg->end_point.z});

  // Call route_callback_
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

void WMListenerWorker::setConfigSpeedLimit(double config_lim)
{
  config_speed_limit_ = config_lim;
  //Function to load config_limit into CarmaWorldModel
   world_model_->setConfigSpeedLimit(config_speed_limit_);
}

double WMListenerWorker::getConfigSpeedLimit() const
{
  return config_speed_limit_;
}



}  // namespace carma_wm
