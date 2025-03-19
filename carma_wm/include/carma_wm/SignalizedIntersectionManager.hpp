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
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_extension/regulatory_elements/StopRule.h>
#include <lanelet2_extension/regulatory_elements/CarmaTrafficSignal.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <carma_wm/WorldModelUtils.hpp>
#include <carma_v2x_msgs/msg/map_data.hpp>
#include <carma_v2x_msgs/msg/spat.hpp>
#include <lanelet2_core/Forward.h>
#include <lanelet2_extension/regulatory_elements/SignalizedIntersection.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>


namespace carma_wm
{

struct LANE_DIRECTION
{
  static const uint8_t INGRESS = 2;
  static const uint8_t EGRESS = 1;
};

namespace signalized_intersection_manager
{
enum class PHASE_TYPE : uint8_t
{
  OFF = 0,
  DYNAMIC = 1,
  FIXED = 2
};
} // namespace signalized_intersection_manager

using namespace lanelet::units::literals;

/*! \brief This class manages and keeps track of all signalized intersections in the map.
           All of the SPAT and MAP standard's lane ids to lanelet id mapping is recorded here.
           NOTE: This class functions do not update the map given.
 */
class SignalizedIntersectionManager
{
public:

  SignalizedIntersectionManager(){}

  /*!
  *
  *  \brief Copy constructor that copies everything except the traffic signal states.
  *         This is to keep the states although the map is updated or a similar event happened
  *         NOTE: The function does not update the map with new elements
  *  \param[out] other manager
  */
  SignalizedIntersectionManager(const SignalizedIntersectionManager& other);

  /*!
  *  \brief Assignment operator that copies everything except the traffic signal states.
  *         This is to keep the states although the map is updated or a similar event happened
  *         NOTE: The function does not update the map with new elements
  *  \param[out] other manager
  */
  SignalizedIntersectionManager& operator=(SignalizedIntersectionManager other);

  /*!
  *  \brief Equality operator that checks if every mapping are same except the traffic signal states.
  *         This is to keep the states although the map is updated or a similar event happened
  *         NOTE: The function does not update the map with new elements
  *  \param[out] rhs manager
  */
  bool operator==(const SignalizedIntersectionManager& rhs);

  /*!
  *  \brief Create relevant signalized intersection and carma traffic signals based on the MAP.msg and the lanelet_map
  *         NOTE: The function does not update the map with new elements
  *  \param[out] intersections to return
  *  \param[out] traffic_signals to return
  *  \param map_msg MAP.msg that consists all static data portion of the intersection
  *  \param map lanelet_map to check
  *  \param routing_graph of the lanelet map to accurately detect lanes
  */
  void createIntersectionFromMapMsg(std::vector<lanelet::SignalizedIntersectionPtr>& intersections, std::vector<lanelet::CarmaTrafficSignalPtr>& traffic_signals, const carma_v2x_msgs::msg::MapData& map_msg,
                                    const std::shared_ptr<lanelet::LaneletMap>& map, std::shared_ptr<const lanelet::routing::RoutingGraph> routing_graph);

  /*!
  *  \brief Returns mapping of MAP lane id to lanelet id for the given map and intersection.msg in the MAP.msg.
            This function also records signal_group_id_to its own lanelet id, and also signal group to entry and exit lanelets id mappings
  *  \param[out] entry lane id to lanelet id mappings to return
  *  \param[out] exit  lane id to lanelet id mappings to return
  *  \param intersection MAP.msg that consists all static data portion of the intersection
  *  \param map lanelet_map to check
  *  \param routing_graph of the lanelet map to accurately detect lanes
  *  \throw invalid_argument if given coordinates in the msg doesn't exist in the map
  *         TODO: Need to think about error buffer in the future. Map msgs are made from google maps or open streets maps normally so this function might run into some errors from that.
  */
  void convertLaneToLaneletId(std::unordered_map<uint8_t, lanelet::Id>& entry, std::unordered_map<uint8_t, lanelet::Id>& exit, const carma_v2x_msgs::msg::IntersectionGeometry& intersection,
                              const std::shared_ptr<lanelet::LaneletMap>& map, std::shared_ptr<const lanelet::routing::RoutingGraph> current_routing_graph);

  /*!
   * \brief Sets the max lane width in meters. Map msg points are associated to a lanelet if they are
   *        within this distance to a lanelet as map msg points are guaranteed to apply to a single lane
   */
  void setMaxLaneWidth(double max_lane_width);

  /*!
  *  \brief Returns existing signalized intersection with same entry and exit llts if exists.
  *  \param entry_llts of the intersection
  *  \param exit_llts of the intersection
  *  \return id of the matching intersection in the map, or lanelet::InvalId if none exists
  */
  lanelet::Id matchSignalizedIntersection(const lanelet::Lanelets& entry_llts, const lanelet::Lanelets& exit_llts);

  /*!
  *  \brief Saves the georeference string to be used for converting MAP.msg coordinates
  *  \param target_frame PROJ string of the map
  */
  void setTargetFrame(const std::string& target_frame);

  /*!
  *  \brief Returns carma traffic signal and saves all id relevant id mappings (signal group to lanelet id) internally
  *  \param signal_group_id of the traffic signal
  *  \param exit_llts of the signal_group
  *  \param entry_llts of the signal_group
  *  \return traffic signal corresponding to the signal group
  */
  std::shared_ptr<lanelet::CarmaTrafficSignal> createTrafficSignalUsingSGID(uint8_t signal_group_id, const lanelet::Lanelets& entry_lanelets, const lanelet::Lanelets& exit_lanelets);

  /*!
  *  \brief Returns carma traffic signal and saves all id relevant id mappings (signal group to lanelet id) internally
  *  \param signal_group_id of the traffic signal
  *  \param exit_llts of the signal_group
  *  \param entry_llts of the signal_group
  *  \return traffic signal corresponding to the signal group
  */
  lanelet::Lanelets identifyInteriorLanelets(const lanelet::Lanelets& entry_llts, const std::shared_ptr<lanelet::LaneletMap>& map);

  /**
   * @brief processSpatFromMsg update map's traffic light states with SPAT msg
   *
   * @param spat_msg Msg to update with
   */
  void processSpatFromMsg(const carma_v2x_msgs::msg::SPAT& spat_msg,
    const std::shared_ptr<lanelet::LaneletMap>& semantic_map);

  /*!
  *  \brief Update the recorded traffic signal objects in the intersection as fixed signal,
  *         determining the phase cycles from the available signal durations.
  *         Do nothing if intersection_id is not found.
  *  \param intersection_id of the intersection TODO
  */
  void updateSignalAsFixedSignal(uint16_t intersection_id,
    const std::shared_ptr<lanelet::LaneletMap>& semantic_map);

  /*!
  *  \brief Update the recorded traffic signal objects in the intersection as dynamic signal,
  *         directly overwriting any previous states
  *  \param intersection_id of the intersection TODO
  */
  void updateSignalAsDynamicSignal(uint16_t intersection_id,
    const std::shared_ptr<lanelet::LaneletMap>& semantic_map);

  /*!
  *  \brief TODO
  *  \param intersection_id of the intersection TODO
  */
  std::tuple<std::vector<std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>>,
    std::vector<boost::posix_time::ptime>>
    extract_signal_states_from_movement_state(
      const carma_v2x_msgs::msg::IntersectionState& curr_intersection,
      const carma_v2x_msgs::msg::MovementState& current_movement_state);

    /*! \brief update minimum end time to account for minute of the year
    * \param min_end_time minimum end time of the spat movement event list
    * \param moy_exists tells weather minute of the year exist or not
    * \param moy value of the minute of the year
    * \param use_real_time_spat_in_sim Boolean to indicate if the incoming spat is based on
    *   wall clock. Required in edge cases where deployment in simulation is receiving
    *   SPaT messages based on wall clock.
    * Defaults to false.
   */
  boost::posix_time::ptime min_end_time_converter_minute_of_year(
    boost::posix_time::ptime min_end_time,bool moy_exists,uint32_t moy=0,
    bool is_simulation = true, bool use_real_time_spat_in_sim=false);

  /*! \brief helper for traffic signal Id
   */
  lanelet::Id getTrafficSignalId(uint16_t intersection_id,uint8_t signal_id);

  /*! \brief helper for getting traffic signal with given lanelet::Id
   */
  lanelet::CarmaTrafficSignalPtr getTrafficSignal(const lanelet::Id& id,
    const std::shared_ptr<lanelet::LaneletMap>& semantic_map) const;

  // SignalizedIntersection's geometry points from MAP Msg
  std::unordered_map<uint16_t, std::vector<lanelet::Point3d>> intersection_nodes_;

  // SignalizedIntersection's reference point correction pair of (x, y) for each intersection_id
  std::unordered_map<uint16_t, std::pair<double, double>> intersection_coord_correction_;

  // SignalizedIntersection quick id lookup
  std::unordered_map<uint16_t, lanelet::Id> intersection_id_to_regem_id_;

  // SignalizedIntersection reverse quick id lookup
  std::unordered_map<lanelet::Id, uint16_t> regem_id_to_intersection_id_;

  // CarmaTrafficSignal quick id lookup
  std::unordered_map<uint8_t, lanelet::Id> signal_group_to_traffic_light_id_;

  // CarmaTrafficSignal exit lanelets ids quick lookup
  std::unordered_map<uint8_t, std::unordered_set<lanelet::Id>> signal_group_to_exit_lanelet_ids_;

  // CarmaTrafficSignal entry lanelets ids quick lookup
  std::unordered_map<uint8_t, std::unordered_set<lanelet::Id>> signal_group_to_entry_lanelet_ids_;

  // Traffic signal states and their end_time mappings.
  std::unordered_map<uint16_t, std::unordered_map<uint8_t,std::vector<std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>>>> traffic_signal_states_; //[intersection_id][signal_group_id]

  // Traffic signal's start_time mappings (must be same size as traffic_signal_states_)
  std::unordered_map<uint16_t, std::unordered_map<uint8_t,std::vector<boost::posix_time::ptime>>> traffic_signal_start_times_; //[intersection_id][signal_group_id]

  // Last received signal state from SPAT
  std::unordered_map<uint16_t, std::unordered_map<uint8_t,std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>>> last_seen_state_; //[intersection_id][signal_group_id]

  // traffic signal state counter
  std::unordered_map<uint16_t, std::unordered_map<uint8_t,int>> signal_state_counter_; //[intersection_id][signal_group_id]

  std::optional<rclcpp::Time> ros1_clock_ = std::nullopt;
  std::optional<rclcpp::Time> simulation_clock_ = std::nullopt;

  bool use_sim_time_;
  bool use_real_time_spat_in_sim_;
  signalized_intersection_manager::PHASE_TYPE phase_type_ =
    signalized_intersection_manager::PHASE_TYPE::OFF;

private:
  // PROJ string of current map
  std::string target_frame_ = "";

  // Max width of lane in meters
  double max_lane_width_ = 4;

};

}  // namespace carma_wm
