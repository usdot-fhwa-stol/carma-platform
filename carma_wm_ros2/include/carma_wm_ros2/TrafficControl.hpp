#pragma once

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

#include <rclcpp/rclcpp.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <lanelet2_core/LaneletMap.h>

#include <autoware_lanelet2_msgs/msg/map_bin.hpp>

#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <lanelet2_extension/regulatory_elements/StopRule.h>
#include <lanelet2_extension/regulatory_elements/CarmaTrafficSignal.h>
#include <lanelet2_extension/regulatory_elements/PassingControlLine.h>
#include <lanelet2_extension/regulatory_elements/DigitalMinimumGap.h>
#include <carma_wm_ros2/SignalizedIntersectionManager.hpp>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <autoware_lanelet2_ros2_interface/utility/query.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace carma_wm
{
/*! \brief This class defines an update to traffic regulations received from carma_wm_broadcaster.
           This class can be sent/received over ROS network in a binary ROS msg format.
 */
using namespace lanelet::units::literals;

class TrafficControl
{
public:
  TrafficControl(){}
  TrafficControl(boost::uuids::uuid id,
                 std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> update_list, 
                 std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> remove_list,
                 std::vector<lanelet::Lanelet> lanelet_addition):
                 id_(id), update_list_(update_list), remove_list_(remove_list), lanelet_additions_(lanelet_addition){}  

  boost::uuids::uuid id_;  // Unique id of this geofence

  // elements needed for broadcasting to the rest of map users
  std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> update_list_;
  std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> remove_list_;

  // lanelets additions needed or broadcasting to the rest of map users
  std::vector<lanelet::Lanelet> lanelet_additions_;

  // traffic light id lookup
  std::vector<std::pair<uint32_t, lanelet::Id>> traffic_light_id_lookup_;

  // signalized intersection manager
  carma_wm::SignalizedIntersectionManager sim_;

};

/**
 * [Converts carma_wm::TrafficControl object to ROS message. Similar implementation to 
 * lanelet2_extension::utility::message_conversion::toBinMsg]
 * @param gf_ptr [Ptr to Geofence data]
 * @param msg [converted ROS message. Only "data" field is filled]
 * NOTE: When converting the geofence object, the converter fills its relevant map update
 * fields (update_list, remove_list) to be read once received at the user
 */
void toBinMsg(std::shared_ptr<carma_wm::TrafficControl> gf_ptr, autoware_lanelet2_msgs::msg::MapBin* msg);

/**
 * [Converts Geofence binary ROS message to carma_wm::TrafficControl object. Similar implementation to 
 * lanelet2_extension::utility::message_conversion::fromBinMsg]
 * @param msg         [ROS message for geofence]
 * @param gf_ptr      [Ptr to converted Geofence object]
 * @param lanelet_map [Ptr to lanelet map to match incoming objects' memory address with that of 
 *                    existing objects' with same lanelet id]
 * NOTE: When converting the geofence object, the converter only fills its relevant map update
 * fields (update_list, remove_list) as the ROS msg doesn't hold any other data field in the object.
 * NOTE: While main map update function needs to use lanelet_map, other utility use cases such as 
 *       unit test or map update logger does not currently use lanelet_map and can use nullptr as input
 */
void fromBinMsg(const autoware_lanelet2_msgs::msg::MapBin& msg, std::shared_ptr<carma_wm::TrafficControl> gf_ptr, lanelet::LaneletMapPtr lanelet_map = nullptr);

}  // namespace carma_wm


// used for converting geofence into ROS msg binary
namespace boost {
namespace serialization {

template <class Archive>
// NOLINTNEXTLINE
inline void save(Archive& ar, const carma_wm::TrafficControl& gf, unsigned int /*version*/) 
{
  std::string string_id = boost::uuids::to_string(gf.id_);
  ar << string_id;

  // convert the lanelet that need to be added
  size_t lanelet_additions_size = gf.lanelet_additions_.size();
  ar << lanelet_additions_size;
  for (auto llt : gf.lanelet_additions_) ar << llt;

  // convert the regems that need to be removed
  size_t remove_list_size = gf.remove_list_.size();
  ar << remove_list_size;
  for (auto pair : gf.remove_list_) ar << pair;

  // convert id, regem pairs that need to be updated
  size_t update_list_size = gf.update_list_.size();
  ar << update_list_size;
  for (auto pair : gf.update_list_) ar << pair;

  // convert traffic light id lookup
  size_t traffic_light_id_lookup_size = gf.traffic_light_id_lookup_.size();
  ar << traffic_light_id_lookup_size;
  for (auto pair : gf.traffic_light_id_lookup_) ar << pair;

  // convert signalized intersection manager
  ar << gf.sim_;
}

template <class Archive>
// NOLINTNEXTLINE
inline void load(Archive& ar, carma_wm::TrafficControl& gf, unsigned int /*version*/) 
{
  boost::uuids::string_generator gen;
  std::string id;
  ar >> id;
  gf.id_ = gen(id);

  // save llts to add
  size_t lanelet_additions_size;
  ar >> lanelet_additions_size;
  for (auto i = 0u; i < lanelet_additions_size; ++i) 
  {
    lanelet::Lanelet llt;
    ar >> llt;
    gf.lanelet_additions_.push_back(llt);
  }

  // save regems to remove
  size_t remove_list_size;
  ar >> remove_list_size;
  for (auto i = 0u; i < remove_list_size; ++i) 
  {
    std::pair<lanelet::Id, lanelet::RegulatoryElementPtr> remove_item;
    ar >> remove_item;
    gf.remove_list_.push_back(remove_item);
  }

  // save parts that need to be updated
  size_t update_list_size;
  ar >> update_list_size;
  
  for (auto i = 0u; i < update_list_size; ++i) 
  {
    std::pair<lanelet::Id, lanelet::RegulatoryElementPtr> update_item;
    ar >> update_item;
    gf.update_list_.push_back(update_item);
  }

  // save ids 
  size_t traffic_light_id_lookup_size;
  ar >> traffic_light_id_lookup_size;
  for (auto i = 0u; i < traffic_light_id_lookup_size; ++i) 
  {
    std::pair<uint32_t, lanelet::Id> traffic_light_id_pair;
    ar >> traffic_light_id_pair;
    gf.traffic_light_id_lookup_.push_back(traffic_light_id_pair);
  }

  // save signalized intersection manager
  ar >> gf.sim_;
}


template <class Archive>
// NOLINTNEXTLINE
inline void save(Archive& ar, const carma_wm::SignalizedIntersectionManager& sim, unsigned int /*version*/) 
{
  // convert traffic light id lookup
  size_t intersection_id_to_regem_id_size = sim.intersection_id_to_regem_id_.size();
  ar << intersection_id_to_regem_id_size;
  for (auto pair : sim.intersection_id_to_regem_id_)
  {
    ar << pair;
  } 

  // convert traffic light id lookup
  size_t signal_group_to_traffic_light_id_size = sim.signal_group_to_traffic_light_id_.size();
  ar << signal_group_to_traffic_light_id_size;
  for (auto pair : sim.signal_group_to_traffic_light_id_)
  {
    ar << pair;
  } 

  size_t signal_group_to_exit_lanelet_ids_size = sim.signal_group_to_exit_lanelet_ids_.size();
  ar << signal_group_to_exit_lanelet_ids_size;
  for (auto pair : sim.signal_group_to_exit_lanelet_ids_)
  {
    size_t set_size = pair.second.size();
    ar << set_size;
    ar << pair.first;
    for (auto iter = pair.second.begin(); iter != pair.second.end(); iter++)
    {
      ar << *iter;
    } 
  }

  size_t signal_group_to_entry_lanelet_ids_size = sim.signal_group_to_entry_lanelet_ids_.size();
  ar << signal_group_to_entry_lanelet_ids_size;
  for (auto pair : sim.signal_group_to_entry_lanelet_ids_)
  {
    size_t set_size = pair.second.size();
    ar << set_size;
    ar << pair.first;
    for (auto iter = pair.second.begin(); iter != pair.second.end(); iter++)
    {
      ar << *iter;
    } 
  }
}

template <class Archive>
// NOLINTNEXTLINE
inline void load(Archive& ar, carma_wm::SignalizedIntersectionManager& sim, unsigned int /*version*/) 
{
  // save traffic light id lookup
  size_t intersection_id_to_regem_id_size;
  ar >> intersection_id_to_regem_id_size;
  for (size_t i = 0; i < intersection_id_to_regem_id_size; i ++)
  {
    std::pair<uint16_t, lanelet::Id> pair;
    ar >> pair;
    sim.intersection_id_to_regem_id_.emplace(pair);
  } 

  // save traffic light id lookup
  size_t signal_group_to_traffic_light_id_size;
  ar >> signal_group_to_traffic_light_id_size;
  for (size_t i = 0; i < signal_group_to_traffic_light_id_size; i ++)
  {
    std::pair<uint8_t, lanelet::Id> pair;
    ar >> pair;
    sim.signal_group_to_traffic_light_id_.emplace(pair);
  }

  size_t signal_group_to_exit_lanelet_ids_size;
  ar >> signal_group_to_exit_lanelet_ids_size;
  for (size_t i = 0; i < signal_group_to_exit_lanelet_ids_size; i ++)
  {
    size_t set_size;
    ar >> set_size;
    uint8_t signal_grp;
    ar >> signal_grp;
    for (size_t j = 0; j <set_size; j++)
    {
      lanelet::Id id;
      ar >>id;
      sim.signal_group_to_exit_lanelet_ids_[signal_grp].insert(id);
    }
  }

  size_t signal_group_to_entry_lanelet_ids_size;
  ar >> signal_group_to_entry_lanelet_ids_size;
  for (size_t i = 0; i < signal_group_to_entry_lanelet_ids_size; i ++)
  {
    size_t set_size;
    ar >> set_size;
    uint8_t signal_grp;
    ar >> signal_grp;
    for (size_t j = 0; j <set_size; j++)
    {
      lanelet::Id id;
      ar >>id;
      sim.signal_group_to_entry_lanelet_ids_[signal_grp].insert(id);
    }
  }

}

template <typename Archive>
void serialize(Archive& ar, std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>& p, unsigned int /*version*/) 
{
  ar& p.first;
  ar& p.second;
}

template <typename Archive>
void serialize(Archive& ar, std::pair<uint8_t, lanelet::Id>& p, unsigned int /*version*/) 
{
  ar& p.first;
  ar& p.second;
}

template <typename Archive>
void serialize(Archive& ar, std::pair<uint16_t, lanelet::Id>& p, unsigned int /*version*/) 
{
  ar& p.first;
  ar& p.second;
}

template <typename Archive>
void serialize(Archive& ar, std::pair<uint32_t, lanelet::Id>& p, unsigned int /*version*/) 
{
  ar& p.first;
  ar& p.second;
}

} // namespace serialization
} // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(carma_wm::TrafficControl)
BOOST_SERIALIZATION_SPLIT_FREE(carma_wm::SignalizedIntersectionManager)
