/*
 * Copyright (C) 2022 LEIDOS.
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

#include <autoware_lanelet2_ros2_interface/utility/message_conversion.hpp>
#include <lanelet2_extension/regulatory_elements/DirectionOfTravel.h>
#include <lanelet2_extension/regulatory_elements/StopRule.h>
#include <lanelet2_extension/regulatory_elements/CarmaTrafficSignal.h>
#include <lanelet2_extension/regulatory_elements/SignalizedIntersection.h>
#include <lanelet2_routing/internal/Graph.h>
#include "WMListenerWorker.hpp"

namespace carma_wm
{

enum class GeofenceType{ INVALID, DIGITAL_SPEED_LIMIT, PASSING_CONTROL_LINE, REGION_ACCESS_RULE, DIGITAL_MINIMUM_GAP, 
                          DIRECTION_OF_TRAVEL, STOP_RULE, CARMA_TRAFFIC_LIGHT, SIGNALIZED_INTERSECTION/* ... others */ };
// helper function that return geofence type as an enum, which makes it cleaner by allowing switch statement
GeofenceType resolveGeofenceType(const std::string& rule_name)
{
  if (rule_name.compare(lanelet::PassingControlLine::RuleName) == 0) return GeofenceType::PASSING_CONTROL_LINE;
  if (rule_name.compare(lanelet::DigitalSpeedLimit::RuleName) == 0) return GeofenceType::DIGITAL_SPEED_LIMIT;
  if (rule_name.compare(lanelet::RegionAccessRule::RuleName) == 0) return GeofenceType::REGION_ACCESS_RULE;
  if (rule_name.compare(lanelet::DigitalMinimumGap::RuleName) == 0) return GeofenceType::DIGITAL_MINIMUM_GAP;
  if (rule_name.compare(lanelet::DirectionOfTravel::RuleName) == 0) return GeofenceType::DIRECTION_OF_TRAVEL;
  if (rule_name.compare(lanelet::StopRule::RuleName) == 0) return GeofenceType::STOP_RULE;
  if (rule_name.compare(lanelet::CarmaTrafficSignal::RuleName) == 0) return GeofenceType::CARMA_TRAFFIC_LIGHT;
  if (rule_name.compare(lanelet::SignalizedIntersection::RuleName) == 0) return GeofenceType::SIGNALIZED_INTERSECTION;

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


void WMListenerWorker::mapCallback(const autoware_lanelet2_msgs::msg::MapBin::UniquePtr map_msg)
{
  current_map_version_ = map_msg->map_version;

  lanelet::LaneletMapPtr new_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*map_msg, new_map);

  world_model_->setMap(new_map, current_map_version_);

  // After setting map evaluate the current update queue to apply any updates that arrived before the map
  bool more_updates_to_apply = true;
  while(!map_update_queue_.empty() && more_updates_to_apply) {
    
    auto update = std::move(map_update_queue_.front()); // Get first update
    map_update_queue_.pop(); // Remove update from queue

    if (update->map_version < current_map_version_) { // Drop any so far unapplied updates for the previous map
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "There were unapplied updates in carma_wm when a new map was received.");
      continue;
    }
    if (update->map_version == current_map_version_) { // Current update goes with current map
      mapUpdateCallback(std::move(update)); // Apply the update
    } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Done applying updates for new map. However, more updates are waiting for a future map.");
      more_updates_to_apply = false; // If there is more updates queued that are not for this map version assume they are for a future map version
    }

  }

  // Call user defined map callback
  if (map_callback_)
  {
    map_callback_();
  }

  if (delayed_route_msg_) {
    if (delayed_route_msg_.get().map_version == current_map_version_) { // If there is a delayed route message to apply then do so
      routeCallback(std::make_unique<carma_planning_msgs::msg::Route>(delayed_route_msg_.get()));
    } else if (delayed_route_msg_.get().map_version < current_map_version_) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Dropping delayed route message which was never applied as updated map was not recieved");
      delayed_route_msg_ = boost::none;
    } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "There is a delayed route message still waiting to be applied in carma_wm");
    }
  }
}

void WMListenerWorker::incomingSpatCallback(const carma_v2x_msgs::msg::SPAT::UniquePtr spat_msg)
{
  world_model_->processSpatFromMsg(*spat_msg, use_sim_time_);
}

bool WMListenerWorker::checkIfReRoutingNeeded() const
{
  return rerouting_flag_;
}

void WMListenerWorker::enableUpdatesWithoutRoute()
{
  route_node_flag_=true;
}

// helper function to log SignalizedIntersectionManager content
void logSignalizedIntersectionManager(const carma_wm::SignalizedIntersectionManager& sim)
{
  for (auto pair : sim.intersection_id_to_regem_id_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "inter id: " << (int)pair.first << ", regem id: " << pair.second);
  }
  for (auto pair : sim.signal_group_to_entry_lanelet_ids_)
  {
    for (auto iter = pair.second.begin(); iter != pair.second.end(); iter++)
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "signal id: " << (int)pair.first << ", entry llt id: " << *iter);
    }
  }
  for (auto pair : sim.signal_group_to_exit_lanelet_ids_)
  {
    for (auto iter = pair.second.begin(); iter != pair.second.end(); iter++)
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "signal id: " << (int)pair.first << ", exit llt id: " << *iter);
    }
  }
  for (auto pair : sim.signal_group_to_traffic_light_id_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "signal id: " << (int)pair.first << ", regem id: " << pair.second);
  }
}

void WMListenerWorker::mapUpdateCallback(autoware_lanelet2_msgs::msg::MapBin::UniquePtr geofence_msg)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Map Update Being Evaluated. SeqNum: " << geofence_msg->seq_id);
  if (rerouting_flag_) // no update should be applied if rerouting 
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Currently new route is being processed. Queueing this update. Received seq: " << geofence_msg->seq_id << " prev seq: " << most_recent_update_msg_seq_);
    map_update_queue_.emplace(std::move(geofence_msg));
    return;
  }
  if (geofence_msg->seq_id <= most_recent_update_msg_seq_) {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Dropping map update which has already been processed. Received seq: " << geofence_msg->seq_id << " prev seq: " << most_recent_update_msg_seq_);
    return;
  } else if(!world_model_->getMap() || current_map_version_ < geofence_msg->map_version) { // If our current map version is older than the version target by this update
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Update received for newer map version than available. Queueing update until map is available.");
    map_update_queue_.emplace(std::move(geofence_msg));
    return;
  } else if (current_map_version_ > geofence_msg->map_version) { // If this update is for an older map
    RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Dropping old map update as newer map is already available.");
    return;
  } else if (most_recent_update_msg_seq_ + 1 < geofence_msg->seq_id) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Queuing map update as we are waiting on an earlier update to be applied. most_recent_update_msg_seq_: " << most_recent_update_msg_seq_ << "geofence_msg->seq_id: " << geofence_msg->seq_id);
    map_update_queue_.emplace(std::move(geofence_msg));
    return;
  }


  if(geofence_msg->invalidates_route==true && world_model_->getRoute())
  {  
    rerouting_flag_=true;
    recompute_route_flag_ = true;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Received notice that route has been invalidated in mapUpdateCallback");

    if(route_node_flag_!=true)
    {
     RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Route is not yet available. Therefore queueing the update");
     map_update_queue_.emplace(std::move(geofence_msg));
     return;
    }
  }

  most_recent_update_msg_seq_ = geofence_msg->seq_id; // Update current sequence count

  auto gf_ptr = std::shared_ptr<carma_wm::TrafficControl>(new carma_wm::TrafficControl);
  
  // convert ros msg to geofence object
  carma_wm::fromBinMsg(*geofence_msg, gf_ptr, world_model_->getMutableMap());

  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Processing Map Update with Geofence Id:" << gf_ptr->id_);
  
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Geofence id" << gf_ptr->id_ << " requests addition of lanelets size: " << gf_ptr->lanelet_additions_.size());
  for (auto llt : gf_ptr->lanelet_additions_)
  {
    // world model here should blindly accept the map update received
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Adding new lanelet with id: " << llt.id());
    auto left = llt.leftBound3d(); //new lanelet coming in
    
    // updating incoming points' memory addresses with local ones of same ids
    // so that lanelet library can recognize they are same objects 
    for (size_t i = 0; i < left.size(); i ++)
    {
      if (world_model_->getMutableMap()->pointLayer.exists(left[i].id())) //rewrite the memory address of new pts with that of local
      {
        llt.leftBound3d()[i] = world_model_->getMutableMap()->pointLayer.get(left[i].id());
      }
    }
    auto right = llt.rightBound3d(); //new lanelet coming in
    for (size_t i = 0; i < right.size(); i ++)
    {
      if (world_model_->getMutableMap()->pointLayer.exists(right[i].id())) //rewrite the memory address of new pts with that of local
      {
        llt.rightBound3d()[i] = world_model_->getMutableMap()->pointLayer.get(right[i].id());
      }
    }

    world_model_->getMutableMap()->add(llt);
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Geofence id" << gf_ptr->id_ << " sends record of traffic_lights_id size: " << gf_ptr->traffic_light_id_lookup_.size());
  for (auto pair : gf_ptr->traffic_light_id_lookup_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Adding new pair for traffic light ids: " << pair.first << ", and lanelet::Id: " << pair.second);
    world_model_->setTrafficLightIds(pair.first, pair.second);
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Geofence id" << gf_ptr->id_ << " sends record of intersections size: " << gf_ptr->sim_.intersection_id_to_regem_id_.size());
  if (gf_ptr->sim_.intersection_id_to_regem_id_.size() > 0)
  {
    world_model_->sim_ = gf_ptr->sim_;
    logSignalizedIntersectionManager(world_model_->sim_);
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Geofence id" << gf_ptr->id_ << " requests removal of size: " << gf_ptr->remove_list_.size());
  for (auto pair : gf_ptr->remove_list_)
  {
    auto parent_llt = world_model_->getMutableMap()->laneletLayer.get(pair.first);
    // we can only check by id, if the element is there
    // this is only for speed optimization, as world model here should blindly accept the map update received
    auto regems_copy_to_check = parent_llt.regulatoryElements(); // save local copy as the regem can be deleted during iteration
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Regems found in lanelet: " << regems_copy_to_check.size());
    for (auto regem: regems_copy_to_check)
    {
      // we can't use the deserialized element as its data address conflicts the one in this node
      if (pair.second->id() == regem->id()) world_model_->getMutableMap()->remove(parent_llt, regem);
    }
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Regems left in lanelet after removal: " << parent_llt.regulatoryElements().size());

  }
  
  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Geofence id" << gf_ptr->id_ << " requests update of size: " << gf_ptr->update_list_.size());

  // we should extract general regem to specific type of regem the geofence specifies
  for (auto pair : gf_ptr->update_list_)
  {

    auto parent_llt = world_model_->getMutableMap()->laneletLayer.get(pair.first);

    auto regemptr_it = world_model_->getMutableMap()->regulatoryElementLayer.find(pair.second->id());

    // if this regem is already in the map.
    // This section is expected to be called to add back regulations which were previously removed by expired geofences.
    if (regemptr_it != world_model_->getMutableMap()->regulatoryElementLayer.end())  
    {

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Reapplying previously existing element for lanelet id:" << parent_llt.id() << ", and regem id: " << regemptr_it->get()->id());
      // again we should use the element with correct data address to be consistent
      world_model_->getMutableMap()->update(parent_llt, *regemptr_it);
    }
    else // Updates are treated as new regulations after the old value was removed. In both cases we enter this block. 
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "New regulatory element at lanelet: " << parent_llt.id() << ", and id: " << pair.second->id());
      newRegemUpdateHelper(parent_llt, pair.second.get());
    }
  }
  
  // set the Map to trigger a new route graph construction if rerouting was required by the updates and a new graph was not provided
  world_model_->setMap(world_model_->getMutableMap(), current_map_version_, recompute_route_flag_ && !geofence_msg->has_routing_graph );

  // If a new graph was provided then set that graph
  // recompute_route_flag_ not checked here to support the case of the first map or map version changing
  if (geofence_msg->has_routing_graph) {

    LaneletRoutingGraphPtr graph = routingGraphFromMsg(geofence_msg->routing_graph, world_model_->getMutableMap());

    if (!graph) {
      throw std::invalid_argument("Map updated provided routing graph which could not be applied to the current map.");
    }

    world_model_->setRoutingGraph(graph);

  }

  // no need to reroute again unless received invalidated msg again
  if (recompute_route_flag_)
    recompute_route_flag_ = false;
  

  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Finished Applying the Map Update with Geofence Id:" << gf_ptr->id_); 

  // Call user defined map callback
  if (map_callback_)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Calling user defined map update callback");
    map_callback_();
  }
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
  auto factory_regem = lanelet::RegulatoryElementFactory::create(regem->attribute(lanelet::AttributeName::Subtype).value(),
                                                            std::const_pointer_cast<lanelet::RegulatoryElementData>(regem->constData()));

  // we should extract general regem to specific type of regem the geofence specifies
  switch(resolveGeofenceType(regem->attribute(lanelet::AttributeName::Subtype).value()))
  {
    case GeofenceType::PASSING_CONTROL_LINE:
    {
      lanelet::PassingControlLinePtr control_line = std::dynamic_pointer_cast<lanelet::PassingControlLine>(factory_regem);
      if (control_line)
      {
        world_model_->getMutableMap()->update(parent_llt, control_line);
      }
      else
      {
        std::invalid_argument("Dynamic Pointer cast failed on getting valid control line");
      }

      break;
    }
    case GeofenceType::DIGITAL_SPEED_LIMIT:
    {
      lanelet::DigitalSpeedLimitPtr speed = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(factory_regem);
      if (speed)
      {
        world_model_->getMutableMap()->update(parent_llt, speed);
      }
      else
      {
        std::invalid_argument("Dynamic Pointer cast failed on getting valid speed limit");
      }
      break;
    }
    case GeofenceType::REGION_ACCESS_RULE:
    {

      lanelet::RegionAccessRulePtr rar = std::dynamic_pointer_cast<lanelet::RegionAccessRule>(factory_regem);
      if (rar)
      {
        world_model_->getMutableMap()->update(parent_llt, rar);
      }
      else
      {
        std::invalid_argument("Dynamic Pointer cast failed on getting valid region access rule");
      }
      
      break;
    }
    case GeofenceType::DIGITAL_MINIMUM_GAP:
    {

      lanelet::DigitalMinimumGapPtr min_gap = std::dynamic_pointer_cast<lanelet::DigitalMinimumGap>(factory_regem);
      if (min_gap)
      {
        world_model_->getMutableMap()->update(parent_llt, min_gap);
      }
      else
      {
        std::invalid_argument("Dynamic Pointer cast failed on getting valid minimum gap rule");
      }
      
      break;
    }
    case GeofenceType::DIRECTION_OF_TRAVEL:
    {

      lanelet::DirectionOfTravelPtr dot = std::dynamic_pointer_cast<lanelet::DirectionOfTravel>(factory_regem);
      if (dot)
      {
        world_model_->getMutableMap()->update(parent_llt, dot);
      }
      else
      {
        std::invalid_argument("Dynamic Pointer cast failed on getting valid direction of travel");
      }
      
      break;
    }
    case GeofenceType::STOP_RULE:
    {

      lanelet::StopRulePtr sr = std::dynamic_pointer_cast<lanelet::StopRule>(factory_regem);
      if (sr)
      {
        world_model_->getMutableMap()->update(parent_llt, sr);
      }
      else
      {
        std::invalid_argument("Dynamic Pointer cast failed on getting valid stop rule");
      }
      break;
    }
    case GeofenceType::CARMA_TRAFFIC_LIGHT:
    {
      lanelet::CarmaTrafficSignalPtr ctl = std::dynamic_pointer_cast<lanelet::CarmaTrafficSignal>(factory_regem);
      if (ctl)
      {
        world_model_->getMutableMap()->update(parent_llt, ctl);
      }
      else
      {
        std::invalid_argument("Dynamic Pointer cast failed on getting valid carma traffic signal");
      }
      break;
    }
    case GeofenceType::SIGNALIZED_INTERSECTION:
    {
      lanelet::SignalizedIntersectionPtr si = std::dynamic_pointer_cast<lanelet::SignalizedIntersection>(factory_regem);
      if (si)
      {
        world_model_->getMutableMap()->update(parent_llt, si);
      }
      else
      {
        std::invalid_argument("Dynamic Pointer cast failed on getting valid signalized intersection");
      }
      
      break;
    }
    default:
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "World Model instance received an unsupported geofence type in its map update callback!");
      break;
  }
}

LaneletRoutingGraphPtr WMListenerWorker::routingGraphFromMsg(const autoware_lanelet2_msgs::msg::RoutingGraph& msg, lanelet::LaneletMapPtr map) const {

  if (msg.participant_type.compare(getVehicleParticipationType()) != 0) {

    RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"),"Received routing graph does not have matching vehicle type for world model. WM Type: " 
      << getVehicleParticipationType() 
      << " graph type: " << msg.participant_type
    );

    return nullptr;
  }

  // Get the lists of passable lanelets and areas
  // Both these lists must be populated in the same order as the message to support later logic
  lanelet::ConstLanelets passable_lanelets;
  lanelet::ConstAreas passable_areas;

  passable_lanelets.reserve(msg.lanelet_vertices.size());
  passable_areas.reserve(msg.area_vertices.size());

  try {

    // All the passable lanelets and areas should be included as a vertext so just iterate over each and store
    for (auto vertex : msg.lanelet_vertices) {
      passable_lanelets.emplace_back(map->laneletLayer.get(vertex.lanelet_or_area));
    }

    for (auto vertex : msg.area_vertices) {
      passable_areas.emplace_back(map->areaLayer.get(vertex.lanelet_or_area));
    }

  } catch(const lanelet::NoSuchPrimitiveError& e) {

    RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Received routing graph specifies lanelets which do not match the current map version. Actual exception: " << e.what());

    return nullptr;
  }


  // Build the submap
  // This operation does increase in time as the number of lanelets and areas increase
  // however testing shows it to less than 1% of the total routing graph build time so this is a reasonable operation to keep
  auto passable_map = lanelet::utils::createConstSubmap(passable_lanelets, passable_areas);

  // This is the actual graph object which is used to initialize a RoutingGraph
  auto graph = std::make_unique<lanelet::routing::internal::RoutingGraphGraph>(msg.num_unique_routing_cost_ids);

  // Vertex must be added first then the edge can be added
  for (auto ll : passable_lanelets) {
    graph->addVertex(lanelet::routing::internal::VertexInfo{ll});
  }

  for (auto area : passable_areas) {
    graph->addVertex(lanelet::routing::internal::VertexInfo{area});
  }

  // Now we can add edges
  for (size_t i = 0; i < msg.lanelet_vertices.size(); ++i) {

    auto vertex = msg.lanelet_vertices[i];
    auto lanelet = passable_lanelets[i]; // passable_lanelets should be in the same order based on how its constructed

    for (size_t j = 0; j < vertex.lanelet_or_area_ids.size(); ++j) {

      lanelet::routing::RelationType relation;

      // Get relation
      switch (vertex.edge_relations[j])
      {
        case autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_SUCCESSOR:
          relation = lanelet::routing::RelationType::Successor; break;

        case autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_LEFT:
          relation = lanelet::routing::RelationType::Left; break;

        case autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_RIGHT:
          relation = lanelet::routing::RelationType::Right; break;

        case autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_ADJACENT_LEFT:
          relation = lanelet::routing::RelationType::AdjacentLeft; break;

        case autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_ADJACENT_RIGHT:
          relation = lanelet::routing::RelationType::AdjacentRight; break;

        case autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_CONFLICTING:
          relation = lanelet::routing::RelationType::Conflicting; break;

        case autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_AREA:
          relation = lanelet::routing::RelationType::Area; break;

        default: // Treat default as RELATION_NONE
          relation = lanelet::routing::RelationType::None; break;
      }

      try {

        // Create edge
        graph->addEdge(
          lanelet, 
          map->laneletLayer.get(vertex.lanelet_or_area_ids[j]), 
          lanelet::routing::internal::EdgeInfo{vertex.edge_routing_costs[j], vertex.edge_routing_cost_source_ids[j], relation}
        );

      }  catch(const lanelet::NoSuchPrimitiveError& e) {

        RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"),"Received routing graph specifies lanelets which do not match the current map version. Not found lanelet or area: " 
          << vertex.lanelet_or_area_ids[j] << " Actual exception: " << e.what());

        return nullptr;
      }

    }
  }

  // Build and return the final initialized routing graph
  return std::make_shared<lanelet::routing::RoutingGraph>(std::move(graph), std::move(passable_map));

}

std::string WMListenerWorker::getVehicleParticipationType() const
{
  return world_model_->getVehicleParticipationType();
}

void WMListenerWorker::roadwayObjectListCallback(const carma_perception_msgs::msg::RoadwayObstacleList::UniquePtr msg)
{
  // this topic publishes only the objects that are on the road
  world_model_->setRoadwayObjects(msg->roadway_obstacles);
}

void WMListenerWorker::routeCallback(const carma_planning_msgs::msg::Route::UniquePtr route_msg)
{
  if (route_msg->map_version < current_map_version_) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Route message rejected as it is for an older map");
    rerouting_flag_ = false; // Clear any blockers on map updates as the route we were waiting for is no longer valid
    return;
  }

  if (route_msg->map_version > current_map_version_) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Route message received for future map. Delaying application until map is recieved");
    delayed_route_msg_ = *route_msg;
    return;
  }

  bool route_invalidated_by_queued_map_update = false; // Flag to indicate whether this new route has been invalidated due to one of the applied queued map updates
  if(rerouting_flag_==true && route_msg->is_rerouted )

  {

    rerouting_flag_ = false; // Reset flag since the route node has finished re-routing

    // After setting map evaluate the current update queue to apply any updates that arrived before the map
    bool more_updates_to_apply = true;
    while(!map_update_queue_.empty() && more_updates_to_apply) {

      auto update = std::move(map_update_queue_.front()); // Get first update
      map_update_queue_.pop(); // Remove update from queue     

      if (update->map_version < current_map_version_) { // Drop any so far unapplied updates for the current map
        RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Apply from reroute: There were unapplied updates in carma_wm when a new map was recieved.");
        continue;
      }
      if (update->map_version == current_map_version_) { // Current update goes with current map which is also the map used by this route
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Applying queued update after route was recieved. ");

        if (update->invalidates_route == true) {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Applied queued map update has invalidated the route.");
          route_invalidated_by_queued_map_update = true;
        }

        update->invalidates_route=false; // Do not trigger recomputation of routing graph in mapUpdateCallback; recomputation of routing graph will occur outside of this loop

        mapUpdateCallback(std::move(update)); // Apply the update
      } else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Apply from reroute: Done applying updates for new map. However, more updates are waiting for a future map.");
        more_updates_to_apply = false; // If there is more updates queued that are not for this map version assume they are for a future map version
      }

    }

  }

  if (!world_model_->getMap()) { // This check is a bit redundant but still useful from a debugging perspective as the alternative is a segfault
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "WMListener received a route before a map was available. Dropping route message.");
    return;
  }

  // If one of the applied queued map updates invalidated the route, then the routing graph must be updated again for the route node
  if (route_invalidated_by_queued_map_update && route_node_flag_){
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"),"At least one applied queued map update has invalidated the route. Routing graph will be recomputed.");
    world_model_->setMap(world_model_->getMutableMap(), current_map_version_, route_invalidated_by_queued_map_update);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"),"Finished recomputing the routing graph for the applied queued map update(s)");

    rerouting_flag_ = true; // Set flag to trigger a route update by the route node due to the updated routing graph

    return;
  }
  else {
    rerouting_flag_ = false; // Reset flag since no applied queued map updates invalidated the route for the route node

    auto path = lanelet::ConstLanelets();
    for(auto id : route_msg->shortest_path_lanelet_ids)
    {
      auto ll = world_model_->getMap()->laneletLayer.get(id);
      path.push_back(ll);
    }

    auto route_opt = path.size() == 1 ? world_model_->getMapRoutingGraph()->getRoute(path.front(), path.back())
                                : world_model_->getMapRoutingGraph()->getRouteVia(path.front(), lanelet::ConstLanelets(path.begin() + 1, path.end() - 1), path.back());
    if(route_opt.is_initialized()) {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::WMListenerWorker"), "Setting route in world model");
      auto ptr = std::make_shared<lanelet::routing::Route>(std::move(route_opt.get()));
      world_model_->setRoute(ptr);
    }

    world_model_->setRouteEndPoint({route_msg->end_point.x,route_msg->end_point.y,route_msg->end_point.z});
    world_model_->setRouteName(route_msg->route_name);

    // Call route_callback_
    if (route_callback_)
    {
      route_callback_();
    }

    return;
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

void WMListenerWorker::isUsingSimTime(bool use_sim_time)
{
  use_sim_time_ = use_sim_time;
}

double WMListenerWorker::getConfigSpeedLimit() const
{
  return config_speed_limit_;
}

void WMListenerWorker::setVehicleParticipationType(std::string participant)
{  
  //Function to load participation type into CarmaWorldModel
  world_model_->setVehicleParticipationType(participant);
}


}  // namespace carma_wm
