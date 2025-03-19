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

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <carma_wm/SignalizedIntersectionManager.hpp>
#include <algorithm>
#include <j2735_v2x_msgs/msg/lane_type_attributes.hpp>

namespace carma_wm
{
  namespace
  {
    // Helper function to check if a signal state is green (permissive or protected)
    inline bool isGreenSignalState(const lanelet::CarmaTrafficSignalState& state)
    {
        return (state == lanelet::CarmaTrafficSignalState::PERMISSIVE_MOVEMENT_ALLOWED ||
                state == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED);
    }
  }
  void SignalizedIntersectionManager::setTargetFrame(const std::string& target_frame)
  {
    target_frame_ = target_frame;
  }

  void SignalizedIntersectionManager::setMaxLaneWidth(double max_lane_width)
  {
    max_lane_width_ = max_lane_width;
  }

  bool SignalizedIntersectionManager::operator==(const SignalizedIntersectionManager& rhs)
  {
    bool is_same = true;
    if (rhs.intersection_id_to_regem_id_ != this->intersection_id_to_regem_id_)
    {
      is_same = false;
    }
    if (rhs.signal_group_to_entry_lanelet_ids_ != this->signal_group_to_entry_lanelet_ids_)
    {
      is_same = false;
    }
    if (rhs.signal_group_to_exit_lanelet_ids_ != this->signal_group_to_exit_lanelet_ids_)
    {
      is_same = false;
    }
    if (rhs.signal_group_to_traffic_light_id_ != this->signal_group_to_traffic_light_id_)
    {
      is_same = false;
    }

    return is_same;
  }

  void SignalizedIntersectionManager::convertLaneToLaneletId(std::unordered_map<uint8_t, lanelet::Id>& entry, std::unordered_map<uint8_t, lanelet::Id>& exit, const carma_v2x_msgs::msg::IntersectionGeometry& intersection,
                                                              const std::shared_ptr<lanelet::LaneletMap>& map, std::shared_ptr<const lanelet::routing::RoutingGraph> current_routing_graph)
  {
    std::unordered_map<uint8_t, std::unordered_set<uint16_t>> signal_group_to_exit_lanes;
    std::unordered_map<uint8_t, std::unordered_set<uint16_t>> signal_group_to_entry_lanes;

    if (target_frame_ == "")
    {
      throw std::invalid_argument("Map is not initialized yet as the georeference was not found...");
    }

    lanelet::projection::LocalFrameProjector local_projector(target_frame_.c_str());

    lanelet::GPSPoint gps_point;
    gps_point.lat = intersection.ref_point.latitude;
    gps_point.lon = intersection.ref_point.longitude;
    gps_point.ele = intersection.ref_point.elevation;

    auto ref_node  = local_projector.forward(gps_point);

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Reference node in map frame x: " << ref_node.x() << ", y: " << ref_node.y());


    for (auto lane : intersection.lane_list)
    {
      if (lane.lane_attributes.lane_type.choice != j2735_v2x_msgs::msg::LaneTypeAttributes::VEHICLE)
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Lane id: " << (int)lane.lane_id << ", is not a lane for vehicle. Only vehicle road is currently supported. Skipping..." );
        continue;
      }
      std::vector<lanelet::Point3d> node_list;

      double curr_x = ref_node.x();
      double curr_y = ref_node.y();

      if (intersection_coord_correction_.find(intersection.id.id) != intersection_coord_correction_.end())
      {
        curr_x += intersection_coord_correction_[intersection.id.id].first;
        curr_y += intersection_coord_correction_[intersection.id.id].second;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Applied reference point correction, delta_x: " <<  intersection_coord_correction_[intersection.id.id].first <<
                          ", delta_y: " << intersection_coord_correction_[intersection.id.id].second << ", to intersection id: " << (int)lane.lane_id);
      }

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Processing Lane id: " << (int)lane.lane_id);

      size_t min_number_of_points = 2; // two points minimum are required

      size_t size_of_available_points = lane.node_list.nodes.node_set_xy.size();

      if (size_of_available_points < min_number_of_points)
      {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Not enough points are provided to match a lane. Skipping... ");
        continue;
      }

      for (size_t i = 0; i < size_of_available_points; i ++ )
      {
        curr_x = lane.node_list.nodes.node_set_xy[i].delta.x + curr_x;
        curr_y = lane.node_list.nodes.node_set_xy[i].delta.y + curr_y;
        lanelet::Point3d curr_node{map->pointLayer.uniqueId(), curr_x, curr_y, 0};

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Current node x: " << curr_x << ", y: " << curr_y);

        node_list.push_back(curr_node);
      }

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Lane directions: " << (int)lane.lane_attributes.directional_use.lane_direction);

      if (lane.lane_attributes.directional_use.lane_direction == LANE_DIRECTION::INGRESS)
      {
        // flip direction if ingress to pick up correct lanelets
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Reversed the node list!");
        std::reverse(node_list.begin(), node_list.end());
      }

      for (auto node : node_list)
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "intersection: " << intersection.id.id << ", " << node.x() << ", " << node.y());
      }
      intersection_nodes_[intersection.id.id].insert(intersection_nodes_[intersection.id.id].end(), node_list.begin(), node_list.end());

      // save which signal group connect to which exit lanes
      for (auto connection : lane.connect_to_list)
      {
        signal_group_to_exit_lanes[connection.signal_group].emplace(connection.connecting_lane.lane);

        if (lane.lane_attributes.directional_use.lane_direction == LANE_DIRECTION::INGRESS)
          signal_group_to_entry_lanes[connection.signal_group].emplace(lane.lane_id);
      }

      // query corresponding lanelet lane from local map
      auto affected_llts = carma_wm::query::getAffectedLaneletOrAreas(node_list, map, current_routing_graph, max_lane_width_);

      if (affected_llts.empty())
      {
        // https://github.com/usdot-fhwa-stol/carma-platform/issues/1593
        // Open issue TODO on how this error is handled
        RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Given offset points are not inside the map for lane: " << (int)lane.lane_id);
        continue;
      }

      lanelet::Id corresponding_lanelet_id;
      if (lane.lane_attributes.directional_use.lane_direction == LANE_DIRECTION::EGRESS)
      {
        corresponding_lanelet_id = affected_llts.front().id();
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Default corresponding_lanelet_id: " << corresponding_lanelet_id <<", in EGRESS");

      }
      else //ingress
      {
        corresponding_lanelet_id = affected_llts.back().id();
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Default corresponding_lanelet_id: " << corresponding_lanelet_id <<", in INGRESS");
      }

      for (auto llt : affected_llts) // filter out intersection lanelets
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Checking if we can get entry/exit from lanelet " << llt.id());
        //TODO direction of affected_llts may play role, but it should be good
        if (llt.lanelet().get().hasAttribute("turn_direction") &&
            (llt.lanelet().get().attribute("turn_direction").value().compare("left") == 0 ||
            llt.lanelet().get().attribute("turn_direction").value().compare("right") == 0))
        {
          std::vector<lanelet::ConstLanelet> connecting_llts;
          if (lane.lane_attributes.directional_use.lane_direction == LANE_DIRECTION::EGRESS)
          {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "lanelet " << llt.id() << " is actually part of the intersecion. Trying to detect EGRESS...");
            connecting_llts = current_routing_graph->following(llt.lanelet().get());
          }
          else
          {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "lanelet " << llt.id() << " is actually part of the intersecion. Trying to detect INGRESS...");
            connecting_llts = current_routing_graph->previous(llt.lanelet().get());
          }

          if (!connecting_llts.empty())
          {
            corresponding_lanelet_id = connecting_llts[0].id();
          }
          else
          {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Interestingly, did not detect here");
            continue;
          }

          break;
        }
      }

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Found existing lanelet id: " << corresponding_lanelet_id);

      if (lane.lane_attributes.directional_use.lane_direction == LANE_DIRECTION::INGRESS)
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Detected INGRESS, " << (int)lane.lane_id);
        entry[lane.lane_id] = corresponding_lanelet_id;
      }
      else if (lane.lane_attributes.directional_use.lane_direction == LANE_DIRECTION::EGRESS)
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Detected EGRESS, " << (int)lane.lane_id);
        exit[lane.lane_id] = corresponding_lanelet_id;
      }
      // ignoring types that are neither ingress nor egress
    }

    // convert and save exit lane ids into lanelet ids with their corresponding signal group ids
    for (auto iter = signal_group_to_exit_lanes.begin(); iter != signal_group_to_exit_lanes.end(); iter++)
    {
      for (auto exit_lane : iter->second)
      {
        if (exit.find(exit_lane) != exit.end())
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Adding exit_lane id: " << exit_lane);
          signal_group_to_exit_lanelet_ids_[iter->first].insert(exit[exit_lane]);
        }
        else
        {
          // https://github.com/usdot-fhwa-stol/carma-platform/issues/1593
          // Open issue TODO on how this error is handled
          RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Unable to convert exit lane Id: "  + std::to_string((int)exit_lane) + ", to lanelet id using the given MAP.msg!");
        }
      }
    }

    // convert and save entry lane ids into lanelet ids with their corresponding signal group ids
    for (auto iter = signal_group_to_entry_lanes.begin(); iter != signal_group_to_entry_lanes.end(); iter++)
    {
      for (auto entry_lane : iter->second)
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Adding entry_lane id: " << entry_lane);
        if (entry.find(entry_lane) != entry.end())
        {
          signal_group_to_entry_lanelet_ids_[iter->first].insert(entry[entry_lane]);
        }
        else
        {
          // https://github.com/usdot-fhwa-stol/carma-platform/issues/1593
          // Open issue TODO on how this error is handled
          RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Unable to convert entry lane Id: "  + std::to_string((int)entry_lane) + ", to lanelet id using the given MAP.msg!");
        }
      }
    }
  }

  lanelet::Id SignalizedIntersectionManager::matchSignalizedIntersection(const lanelet::Lanelets& entry_llts, const lanelet::Lanelets& exit_llts)
  {
    lanelet::Id matching_id = lanelet::InvalId;

    std::vector<lanelet::SignalizedIntersectionPtr> existing_si;

    for (auto llt : entry_llts)
    {
      auto intersections = llt.regulatoryElementsAs<lanelet::SignalizedIntersection>();

      if (intersections.empty())
      {
        // no match if any of the entry lanelet is not part of any intersection.
        break;
      }
      existing_si.insert(existing_si.end(), intersections.begin(),intersections.end());
    }

    for (auto intersection : existing_si)
    {
      auto existing_entry_llts = intersection->getEntryLanelets();
      auto existing_exit_llts = intersection->getExitLanelets();

      if (existing_exit_llts.size() != exit_llts.size() || existing_entry_llts.size() != entry_llts.size())
        continue;

      bool is_different = false;

      for (auto llt: existing_entry_llts)
      {
        if (std::find(entry_llts.begin(), entry_llts.end(), llt) == entry_llts.end())
        {
          is_different = true;

          break;
        }
      }

      for (auto llt: existing_exit_llts)
      {
        if (std::find(exit_llts.begin(), exit_llts.end(), llt) == exit_llts.end())
        {
          is_different = true;

          break;
        }
      }

      if (!is_different)
      {
        // found a match
        matching_id = intersection->id();

        break;
      }

    }

    return matching_id;
  }

  std::shared_ptr<lanelet::CarmaTrafficSignal> SignalizedIntersectionManager::createTrafficSignalUsingSGID(uint8_t signal_group_id, const lanelet::Lanelets& entry_lanelets, const lanelet::Lanelets& exit_lanelets)
  {
    std::vector<lanelet::LineString3d> stop_lines;
    for (auto llt : entry_lanelets)
    {
      std::vector<lanelet::Point3d> points;
      points.push_back(lanelet::Point3d(lanelet::utils::getId(), llt.leftBound2d().back().x(), llt.leftBound2d().back().y(), 0));
      points.push_back(lanelet::Point3d(lanelet::utils::getId(), llt.rightBound().back().x(), llt.rightBound().back().y(), 0));

      lanelet::LineString3d stop_line(lanelet::utils::getId(), points);
      stop_lines.push_back(stop_line);
    }

    lanelet::Id traffic_light_id = lanelet::utils::getId();
    std::shared_ptr<lanelet::CarmaTrafficSignal> traffic_light(new lanelet::CarmaTrafficSignal(lanelet::CarmaTrafficSignal::buildData(traffic_light_id, stop_lines, entry_lanelets, exit_lanelets)));
    signal_group_to_traffic_light_id_[signal_group_id] =  traffic_light_id;

    for (auto llt : exit_lanelets)
    {
      signal_group_to_exit_lanelet_ids_[signal_group_id].insert(llt.id());
    }
    for (auto llt : entry_lanelets)
    {
      signal_group_to_entry_lanelet_ids_[signal_group_id].insert(llt.id());
    }
    return traffic_light;
  }

  void SignalizedIntersectionManager::createIntersectionFromMapMsg(std::vector<lanelet::SignalizedIntersectionPtr>& sig_intersections, std::vector<lanelet::CarmaTrafficSignalPtr>& traffic_signals, const carma_v2x_msgs::msg::MapData& map_msg,
                                                                      const std::shared_ptr<lanelet::LaneletMap>& map, std::shared_ptr<const lanelet::routing::RoutingGraph> routing_graph)
  {

    if (target_frame_ == "")
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Georeference is not loaded yet. Returning without processing this MAP msg.");
      return;
    }

    for (auto const& intersection : map_msg.intersections)
    {
      std::unordered_map<uint8_t, lanelet::Id> entry;
      std::unordered_map<uint8_t, lanelet::Id> exit;

      convertLaneToLaneletId(entry, exit, intersection, map, routing_graph);

      std::vector<lanelet::Lanelet> entry_llts;
      std::vector<lanelet::Lanelet> exit_llts;

      for (auto iter = entry.begin(); iter != entry.end(); iter++)
      {
        entry_llts.push_back(map->laneletLayer.get(iter->second));
      }
      for (auto iter = exit.begin(); iter != exit.end(); iter++)
      {
        exit_llts.push_back(map->laneletLayer.get(iter->second));
      }

      lanelet::Id intersection_id = matchSignalizedIntersection(entry_llts, exit_llts);

      if (intersection_id == lanelet::InvalId)
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "No existing intersection found. Creating a new one...");
        intersection_id = lanelet::utils::getId();

        std::vector<lanelet::Lanelet> interior_llts = identifyInteriorLanelets(entry_llts, map);

        std::shared_ptr<lanelet::SignalizedIntersection> sig_inter(new lanelet::SignalizedIntersection
                                                          (lanelet::SignalizedIntersection::buildData(intersection_id, entry_llts, exit_llts, interior_llts)));
        intersection_id_to_regem_id_[intersection.id.id] = intersection_id;
        regem_id_to_intersection_id_[intersection_id] = intersection.id.id;
        sig_intersections.push_back(sig_inter);
      }
    }


    // create signal group for each from the message
    // check if it already exists
    for (auto sig_grp_pair : signal_group_to_exit_lanelet_ids_)
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"), "Creating signal for: " << (int)sig_grp_pair.first);
      // ignore the traffic signals already inside
      if (signal_group_to_traffic_light_id_.find(sig_grp_pair.first) != signal_group_to_traffic_light_id_.end() &&
           map->regulatoryElementLayer.exists(signal_group_to_traffic_light_id_[sig_grp_pair.first]))
      {
        continue;
      }

      std::vector<lanelet::Lanelet> exit_lanelets;
      for (auto iter = sig_grp_pair.second.begin(); iter != sig_grp_pair.second.end(); iter++)
      {
        exit_lanelets.push_back(map->laneletLayer.get(*iter));
      }
      std::vector<lanelet::Lanelet> entry_lanelets;
      for (auto iter = signal_group_to_entry_lanelet_ids_[sig_grp_pair.first].begin(); iter != signal_group_to_entry_lanelet_ids_[sig_grp_pair.first].end(); iter++)
      {
        entry_lanelets.push_back(map->laneletLayer.get(*iter));
      }

      traffic_signals.push_back(createTrafficSignalUsingSGID(sig_grp_pair.first, entry_lanelets, exit_lanelets));
    }
  }

  lanelet::Lanelets SignalizedIntersectionManager::identifyInteriorLanelets(const lanelet::Lanelets& entry_llts, const std::shared_ptr<lanelet::LaneletMap>& map)
  {
    lanelet::BasicLineString2d polygon_corners;

    if (entry_llts.size() < 2) //at least two lanes (1 ingress and 1 egress) needed to form intersection
    {
      return {};
    }

    for (auto llt : entry_llts)
    {
      lanelet::BasicPoint2d pt(llt.centerline2d().back().x(), llt.centerline2d().back().y());
      polygon_corners.push_back(pt);
    }
    lanelet::BasicPolygon2d polygon(polygon_corners);
    auto interior_llt_pairs = lanelet::geometry::findWithin2d(map->laneletLayer, polygon);
    lanelet::Lanelets interior_llts;

    for (auto pair : interior_llt_pairs)
    {
      if (std::find(entry_llts.begin(),entry_llts.end(), pair.second) == entry_llts.end())
        interior_llts.push_back(pair.second);
    }
    return interior_llts;
  }

  SignalizedIntersectionManager& SignalizedIntersectionManager::operator=(SignalizedIntersectionManager other)
  {
    this->signal_group_to_entry_lanelet_ids_ = other.signal_group_to_entry_lanelet_ids_;
    this->signal_group_to_exit_lanelet_ids_ = other.signal_group_to_exit_lanelet_ids_;
    this->intersection_id_to_regem_id_ = other.intersection_id_to_regem_id_;
    this->signal_group_to_traffic_light_id_ = other.signal_group_to_traffic_light_id_;
    this->intersection_coord_correction_ = other.intersection_coord_correction_;

    return *this;
  }


  SignalizedIntersectionManager::SignalizedIntersectionManager(const SignalizedIntersectionManager& other)
  {
    this->signal_group_to_entry_lanelet_ids_ = other.signal_group_to_entry_lanelet_ids_;
    this->signal_group_to_exit_lanelet_ids_ = other.signal_group_to_exit_lanelet_ids_;
    this->intersection_id_to_regem_id_ = other.intersection_id_to_regem_id_;
    this->signal_group_to_traffic_light_id_ = other.signal_group_to_traffic_light_id_;
    this->intersection_coord_correction_ = other.intersection_coord_correction_;
  }

  void SignalizedIntersectionManager::updateSignalAsFixedSignal(
    uint16_t intersection_id, const std::shared_ptr<lanelet::LaneletMap>& semantic_map)
  {
    // Check if the intersection exists in our map
    auto it_intersection = traffic_signal_states_.find(intersection_id);
    if (it_intersection == traffic_signal_states_.end())
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"),
        "Intersection ID " << (int)intersection_id << " not found in traffic signal states map");
      return;
    }

    // Find signal groups with red to red transition
    // ALLRED (AR) -> RED will have the full RED = Y + G + AR
    uint8_t red_to_red_signal_group = 99;
    uint8_t opposing_lane_green_signal_group = 98;

    // Save transition indices for later use
    size_t full_red_index = 0;
    size_t green_signal_index = 0;
    boost::posix_time::ptime end_time_of_full_red;
    boost::posix_time::ptime end_time_of_green;
    boost::posix_time::ptime end_time_of_yellow;
    boost::posix_time::ptime start_time_of_full_red;

    // Search in given intersection_id
    for (const auto& signal_group_pair : it_intersection->second)
    {
      uint8_t signal_group_id = signal_group_pair.first;
      const auto& state_transitions = signal_group_pair.second;

      // Need at least 2 transitions to have a red-to-red or valid green
      if (state_transitions.size() < 2) {
        continue;
      }

      if (isGreenSignalState(state_transitions.back().second))
      {
        // When one signal transitions from AR to RED,
        // One group should be GREEN
        opposing_lane_green_signal_group = signal_group_id;
        end_time_of_green = state_transitions.back().first;
        green_signal_index = state_transitions.size() - 1;
        continue;
      }

      // Find red to red transitions
      for (size_t i = 0; i < state_transitions.size() - 1; ++i)
      {
        const auto& current_state = state_transitions[i];
        const auto& next_state = state_transitions[i + 1];

        // Check if both current and next states are RED
        if (current_state.second == lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN &&
            next_state.second == lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN)
        {
          red_to_red_signal_group = signal_group_id;
          full_red_index = i + 1;
          end_time_of_full_red = next_state.first;
          start_time_of_full_red = current_state.first;
          // By ignoring all red, we can approximate end time of yellow is
          // same as start time of full red
          end_time_of_yellow = start_time_of_full_red;
          continue;
        }
      }
    }

    if (red_to_red_signal_group == 99 || opposing_lane_green_signal_group == 98)
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm::SignalizedIntersectionManager"),
        "Intersection ID " << (int)intersection_id << " doesn't have a full cycle of signals yet");
      return;
    }

    // Calculate durations
    auto full_red_duration = (end_time_of_full_red - start_time_of_full_red);
    auto full_cycle = full_red_duration + full_red_duration;
    auto green_duration = end_time_of_green - start_time_of_full_red;
    auto yellow_duration = end_time_of_full_red - end_time_of_green;

    // For each signal group in the intersection, update the traffic signals
    for (const auto& [signal_group_id, signal_states] : it_intersection->second)
    {
      // If no new update is recorded, it shouldn't update anything and skip
      if (traffic_signal_states_[intersection_id][signal_group_id].empty() &&
        traffic_signal_start_times_[intersection_id][signal_group_id].empty())
      {
        continue;
      }

      lanelet::Id curr_light_id = getTrafficSignalId(intersection_id, signal_group_id);
      lanelet::CarmaTrafficSignalPtr curr_light = getTrafficSignal(curr_light_id, semantic_map);

      std::vector<std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>>
        full_cycle_time_steps;

      // Fill in the complete signal cycle based on whether this is the green group or red group
      if (signal_group_id == opposing_lane_green_signal_group) {
          // For the green signal group
          full_cycle_time_steps.push_back(std::make_pair(
            end_time_of_green, lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED));
          full_cycle_time_steps.push_back(std::make_pair(
            end_time_of_yellow,lanelet::CarmaTrafficSignalState::PROTECTED_CLEARANCE));
          full_cycle_time_steps.push_back(std::make_pair(
            end_time_of_full_red, lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN));
          full_cycle_time_steps.push_back(std::make_pair(
            end_time_of_green + full_cycle,
            lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED));
      } else {
          // For other signal groups (red)
          full_cycle_time_steps.push_back(std::make_pair(
            end_time_of_full_red, lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN));
          full_cycle_time_steps.push_back(std::make_pair(
            end_time_of_full_red + green_duration,
            lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED));
          full_cycle_time_steps.push_back(std::make_pair(
            end_time_of_full_red + green_duration + yellow_duration,
            lanelet::CarmaTrafficSignalState::PROTECTED_CLEARANCE));
          full_cycle_time_steps.push_back(std::make_pair(
            end_time_of_full_red + full_cycle,
            lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN));
      }

      // Set the states with revision number 0
      curr_light->setStates(full_cycle_time_steps, 0);

      // Clear the recorded states for this signal group
      traffic_signal_states_[intersection_id][signal_group_id] = {};
      traffic_signal_start_times_[intersection_id][signal_group_id] = {};
    }
  }

  void SignalizedIntersectionManager::processSpatFromMsg(const carma_v2x_msgs::msg::SPAT &spat_msg, const std::shared_ptr<lanelet::LaneletMap>& semantic_map)
  {
    if (phase_type_ == signalized_intersection_manager::PHASE_TYPE::OFF)
    {
      return;
    }

    if (!semantic_map)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm"), "Map is not set yet.");
      return;
    }

    if (spat_msg.intersection_state_list.empty())
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm"), "No intersection_state_list in the newly received SPAT msg. Returning...");
      return;
    }

    for (const auto& curr_intersection : spat_msg.intersection_state_list)
    {
      for (const auto& current_movement_state : curr_intersection.movement_list)
      {
        lanelet::Id curr_light_id = getTrafficSignalId(curr_intersection.id.id, current_movement_state.signal_group);

        if (curr_light_id == lanelet::InvalId)
        {
          continue;
        }

        lanelet::CarmaTrafficSignalPtr curr_light =
          getTrafficSignal(curr_light_id, semantic_map);

        if (curr_light == nullptr)
        {
          continue;
        }

        // reset states if the intersection's geometry changed
        if (curr_light->revision_ != curr_intersection.revision)
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm"), "Received a new intersection geometry. intersection_id: " << (int)curr_intersection.id.id << ", and signal_group_id: " << (int)current_movement_state.signal_group);
          traffic_signal_start_times_[curr_intersection.id.id][current_movement_state.signal_group].clear();
          traffic_signal_states_[curr_intersection.id.id][current_movement_state.signal_group].clear();
        }

        // all maneuver types in same signal group is currently expected to share signal timing, so only 0th index is used when setting states
        if (current_movement_state.movement_event_list.empty())
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm"), "Movement_event_list is empty . intersection_id: " << (int)curr_intersection.id.id << ", and signal_group_id: " << (int)current_movement_state.signal_group);
          continue;
        }
        else
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm"), "Movement_event_list size: " << current_movement_state.movement_event_list.size() << " . intersection_id: " << (int)curr_intersection.id.id << ", and signal_group_id: " << (int)current_movement_state.signal_group);
        }

        curr_light->revision_ = curr_intersection.revision; // valid SPAT msg

        auto extracted_signal_states =
          extract_signal_states_from_movement_state(curr_intersection, current_movement_state);

        traffic_signal_states_[curr_intersection.id.id][current_movement_state.signal_group] =
          std::get<0>(extracted_signal_states);
        traffic_signal_start_times_[curr_intersection.id.id][current_movement_state.signal_group] =
          std::get<1>(extracted_signal_states);
      }


      // After all signal groups are recorded, update regulatory element objects in the map for
      // this intersection based on whether if it is dynamic or fixed signal intersection
      if (phase_type_ == signalized_intersection_manager::PHASE_TYPE::FIXED)
      {
        // TSMO UC2
        updateSignalAsFixedSignal(curr_intersection.id.id, semantic_map);
      }
      else
      {
        // TSMO UC3
        updateSignalAsDynamicSignal(curr_intersection.id.id, semantic_map);
      }
    }
  }

  lanelet::Id SignalizedIntersectionManager::getTrafficSignalId(uint16_t intersection_id, uint8_t signal_group_id)
  {
    lanelet::Id inter_id = lanelet::InvalId;
    lanelet::Id signal_id = lanelet::InvalId;

    if (intersection_id_to_regem_id_.find(intersection_id) != intersection_id_to_regem_id_.end())
    {
      inter_id = intersection_id_to_regem_id_[intersection_id];
    }

    if (inter_id != lanelet::InvalId && signal_group_to_traffic_light_id_.find(signal_group_id) != signal_group_to_traffic_light_id_.end())
    {
      signal_id = signal_group_to_traffic_light_id_[signal_group_id];
    }

    return signal_id;
  }

  void SignalizedIntersectionManager::updateSignalAsDynamicSignal(
    uint16_t intersection_id, const std::shared_ptr<lanelet::LaneletMap>& semantic_map)
  {

    const auto& signal_groups_map = traffic_signal_states_[intersection_id];
    // Iterate over all signal groups for this intersection
    // and directly apply the recorded signal states list to each traffic signal objects
    for (const auto& [signal_group_id, signal_states] : signal_groups_map) {

      // If no new update is recorded, it shouldn't update anything and skip
      if (traffic_signal_states_[intersection_id][signal_group_id].empty() &&
        traffic_signal_start_times_[intersection_id][signal_group_id].empty())
      {
        continue;
      }

      lanelet::Id curr_light_id = getTrafficSignalId(intersection_id, signal_group_id);
      lanelet::CarmaTrafficSignalPtr curr_light = getTrafficSignal(curr_light_id, semantic_map);

      curr_light->recorded_time_stamps =
        traffic_signal_states_[intersection_id][signal_group_id];
      curr_light->recorded_start_time_stamps  =
        traffic_signal_start_times_[intersection_id][signal_group_id];

      traffic_signal_states_[intersection_id][signal_group_id]={};
      traffic_signal_start_times_[intersection_id][signal_group_id]={};
    }
  }

  lanelet::CarmaTrafficSignalPtr SignalizedIntersectionManager::getTrafficSignal(const
    lanelet::Id& id, const std::shared_ptr<lanelet::LaneletMap>& semantic_map) const
  {
    auto general_regem = semantic_map->regulatoryElementLayer.get(id);

    auto lanelets_general = semantic_map->laneletLayer.findUsages(general_regem);
    if (lanelets_general.empty())
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm"), "There was an error querying lanelet for traffic light with id: " << id);
    }

    auto curr_light_list = lanelets_general[0].regulatoryElementsAs<lanelet::CarmaTrafficSignal>();

    if (curr_light_list.empty())
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm"), "There was an error querying traffic light with id: " << id);
      return nullptr;
    }

    lanelet::CarmaTrafficSignalPtr curr_light;

    for (auto signal : lanelets_general[0].regulatoryElementsAs<lanelet::CarmaTrafficSignal>())
    {
      if (signal->id() == id)
      {
        curr_light = signal;
        break;
      }
    }

    if (!curr_light)
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm"), "Was not able to find traffic signal with id: " << id << ", ignoring...");
      return nullptr;
    }

    return curr_light;
  }

  std::tuple<std::vector<std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>>,
    std::vector<boost::posix_time::ptime>>
    SignalizedIntersectionManager::extract_signal_states_from_movement_state(
      const carma_v2x_msgs::msg::IntersectionState& curr_intersection,
      const carma_v2x_msgs::msg::MovementState& current_movement_state)
  {
    std::vector<std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>>
      min_end_t_and_states;

    std::vector<boost::posix_time::ptime> start_time_and_states;

    for(auto current_movement_event:current_movement_state.movement_event_list)
    {
      // raw min_end_time in seconds measured from the most recent full hour
      boost::posix_time::ptime min_end_time = lanelet::time::timeFromSec(
        current_movement_event.timing.min_end_time);
      boost::posix_time::ptime start_time = lanelet::time::timeFromSec(
        current_movement_event.timing.start_time);

      min_end_time = min_end_time_converter_minute_of_year(min_end_time,
        curr_intersection.moy_exists,curr_intersection.moy,
        use_sim_time_, use_real_time_spat_in_sim_); // Accounting minute of the year

      start_time = min_end_time_converter_minute_of_year(start_time,
        curr_intersection.moy_exists,curr_intersection.moy,
        use_sim_time_, use_real_time_spat_in_sim_); // Accounting minute of the year

      auto received_state = static_cast<lanelet::CarmaTrafficSignalState>(
        current_movement_event.event_state.movement_phase_state);

      min_end_t_and_states.push_back(std::make_pair(min_end_time, received_state));

      start_time_and_states.push_back(start_time);

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm"), "intersection id: "
        << (int)curr_intersection.id.id << ", signal: " << (int)current_movement_state.signal_group
        << ", start_time: " << std::to_string(lanelet::time::toSec(start_time))
        << ", end_time: " << std::to_string(lanelet::time::toSec(min_end_time))
        << ", state: " << received_state);
    }

    return std::make_tuple(min_end_t_and_states, start_time_and_states);
  }

  boost::posix_time::ptime SignalizedIntersectionManager::min_end_time_converter_minute_of_year(
    boost::posix_time::ptime min_end_time,bool moy_exists,uint32_t moy, bool is_simulation,
    bool use_real_time_spat_in_sim)
  {
    double simulation_time_difference_in_seconds = 0.0;
    double wall_time_offset_in_seconds = 0.0;
    // NOTE: In simulation, ROS1 clock (often coming from CARLA) can have a large time ahead.
    // the timing calculated here is in Simulation time, which is behind. Therefore, the world model adds the offset to make it meaningful to carma-platform:
    // https://github.com/usdot-fhwa-stol/carma-platform/issues/2217
    if (ros1_clock_ && simulation_clock_)
    {
      simulation_time_difference_in_seconds = ros1_clock_.value().seconds() - simulation_clock_.value().seconds();
    }
    else if (ros1_clock_ && use_real_time_spat_in_sim)
    {
      // NOTE: If carma-platform is running in simulation with clock synced to sim time but the incoming spat information is based on wall clock
      // the spat signal phase time must be offset to sim time.
      wall_time_offset_in_seconds = (std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()) - ros1_clock_.value().seconds();
    }

    min_end_time += lanelet::time::durationFromSec(simulation_time_difference_in_seconds);
    min_end_time -= lanelet::time::durationFromSec(wall_time_offset_in_seconds);

    if (moy_exists) //account for minute of the year
    {
      auto inception_boost(boost::posix_time::time_from_string("1970-01-01 00:00:00.000")); // inception of epoch
      auto duration_since_inception(lanelet::time::durationFromSec(std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()));
      auto curr_time_boost = inception_boost + duration_since_inception;

      int curr_year = curr_time_boost.date().year();

      // Force the current year to start of epoch if it is simulation
      if (is_simulation && !use_real_time_spat_in_sim)
        curr_year = 1970;

      auto curr_year_start_boost(boost::posix_time::time_from_string(std::to_string(curr_year)+ "-01-01 00:00:00.000"));

      auto curr_minute_stamp_boost = curr_year_start_boost + boost::posix_time::minutes((int)moy);

      int hours_of_day = curr_minute_stamp_boost.time_of_day().hours();
      int curr_month = curr_minute_stamp_boost.date().month();
      int curr_day = curr_minute_stamp_boost.date().day();

      auto curr_day_boost(boost::posix_time::time_from_string(std::to_string(curr_year) + "/" + std::to_string(curr_month) + "/" + std::to_string(curr_day) +" 00:00:00.000")); // GMT is the standard
      auto curr_hour_boost = curr_day_boost + boost::posix_time::hours(hours_of_day);

      min_end_time += lanelet::time::durationFromSec(lanelet::time::toSec(curr_hour_boost));
      return min_end_time;
    }
    else
    {
      return min_end_time; // return unchanged
    }
  }

}  // namespace carma_wm
