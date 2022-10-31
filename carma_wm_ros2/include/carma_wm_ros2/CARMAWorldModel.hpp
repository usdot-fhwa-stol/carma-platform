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

#include "carma_wm_ros2/WorldModel.hpp"
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>
#include "carma_wm_ros2/IndexedDistanceMap.hpp"
#include <carma_perception_msgs/msg/roadway_obstacle.hpp>
#include <carma_perception_msgs/msg/roadway_obstacle_list.hpp>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <carma_v2x_msgs/msg/spat.hpp>
#include "carma_wm_ros2/TrackPos.hpp"
#include "carma_wm_ros2/WorldModelUtils.hpp"
#include <lanelet2_extension/time/TimeConversion.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "carma_wm_ros2/SignalizedIntersectionManager.hpp"


namespace carma_wm
{
/*! \brief Class which implements the WorldModel interface. In addition this class provides write access to the world
 *         model. Write access is achieved through setters for the Map and Route and getMutableMap().
 *         NOTE: This class should NOT be used in runtime code by users and is exposed solely for use in unit tests where the WMListener class cannot be instantiated. 
 *
 *  Proper usage of this class dictates that the Map and Route object be kept in sync. For this reason normal WorldModel users should not try to construct this class directly unless in unit tests.
 *
 * NOTE: This class uses the CarmaUSTrafficRules class internally to interpret routes.
 *       So routes which are set on this model should use the getTrafficRules() method to build using the correct rule
 * set
 */
class CARMAWorldModel : public WorldModel
{
public:
  /**
   * @brief Constructor
   *
   */
  CARMAWorldModel() = default;

  /**
   * @brief Destructor as required by interface
   *
   */
  ~CARMAWorldModel() = default;

  /*! \brief Set the current map
   *
   *  \param map A shared pointer to the map which will share ownership to this object
   *  \param map_version Optional field to set the map version. While this is technically optional its uses is highly advised to manage synchronization.
   *  \param recompute_routing_graph Optional field which if true will result in the routing graph being recomputed. NOTE: If this map is the first map set the graph will always be recomputed
   */
  void setMap(lanelet::LaneletMapPtr map, size_t map_version = 0, bool recompute_routing_graph = true);

  /*!
   * \brief Set the routing graph for the participant type.
   *        This function may serve as an optimization to recomputing the routing graph when it is already available
   * 
   * NOTE: The set graph will be overwritten if setMap(recompute_routing_graph=true) is called. 
   *       It will not, be overwritten if the map is set with recompute_routing_graph=false
   *
   * \param graph The graph to set. 
   *              NOTE: This graph must be for the participant type specified getVehicleParticipationType().
   *              There is no way to validate this from the object so the user must ensure consistency. 
   * 
   */ 
  void setRoutingGraph(LaneletRoutingGraphPtr graph);

  /*! \brief Set the current route. This route must match the current map for this class to function properly
   *
   *  \param route A shared pointer to the route which will share ownership to this object
   */
  void setRoute(LaneletRoutePtr route);

  /*! \brief Sets the id mapping between intersection/signal group and lanelet::Id for traffic lights in the map.
   *  \param id intersection_id (16bit) and signal_group_id (8bit) concatenated in that order and saved in 32bit
   *  \param lanelet_id lanelet_id
   */
  void setTrafficLightIds(uint32_t id, lanelet::Id lanelet_id);

  /*! \brief Get a mutable version of the current map
   * 
   *  NOTE: the user must make sure to setMap() after any edit to the map and to set a valid route
   */
  lanelet::LaneletMapPtr getMutableMap() const;

  /*! \brief Update internal records of roadway objects. These objects MUST be guaranteed to be on the road. 
   * 
   * These are detected by the sensor fusion node and are passed as objects compatible with lanelet 
   */
  void setRoadwayObjects(const std::vector<carma_perception_msgs::msg::RoadwayObstacle>& rw_objs);

  /**
   * @brief processSpatFromMsg update map's traffic light states with SPAT msg
   *
   * @param spat_msg Msg to update with
   */
  void processSpatFromMsg(const carma_v2x_msgs::msg::SPAT& spat_msg);

  /**
   * \brief This function is called by distanceToObjectBehindInLane or distanceToObjectAheadInLane. 
   * Gets Downtrack distance to AND copy of the closest object on the same lane as the given point. Also returns crosstrack
   * distance relative to that object. Plus downtrack if the object is ahead along the lane, and also plus crosstrack
   * if the object is to the right relative to the reference line that crosses given object_center and is parallel to the
   * centerline of the lane. 
   *
   * \param object_center the point to measure the distance from
   * \param section either of LANE_AHEAD, LANE_BEHIND, LANE_FULL each including the current lanelet
   *
   * \throw std::invalid_argument if the map is not set, contains no lanelets, or the given point
   * is not on the current semantic map
   *
   * \return An optional tuple of <TrackPos, carma_perception_msgs::msg::RoadwayObstacle> to the closest in lane object. Return empty if there is no objects on current lane or the road
   */
  lanelet::Optional<std::tuple<TrackPos,carma_perception_msgs::msg::RoadwayObstacle>> getNearestObjInLane(const lanelet::BasicPoint2d& object_center, const LaneSection& section = LANE_AHEAD) const;

  /*! \brief update minimum end time to account for minute of the year
    * \param min_end_time minimum end time of the spat movement event list
    * \param moy_exists tells weather minute of the year exist or not
    * \param moy value of the minute of the year
   */
  boost::posix_time::ptime min_end_time_converter_minute_of_year(boost::posix_time::ptime min_end_time,bool moy_exists,uint32_t moy=0);

  /*! \brief for cheking previous rate to avoid repetation.
    * \param min_end_time_dynamic dynamic spat processing minimum end time
    * \param received_state_dynamic phase rate of the movement event list event state
    * \param mov_id id of the traffic signal states
    * \param mov_signal_group signal group of the traffic signal states
   */
  bool check_if_seen_before_movement_state(boost::posix_time::ptime min_end_time_dynamic,lanelet::CarmaTrafficSignalState received_state_dynamic,uint16_t mov_id, uint8_t mov_signal_group);

/** \param config_lim the configurable speed limit value populated from WMListener using the config_speed_limit parameter
 * in VehicleConfigParams.yaml
*
*/
  void setConfigSpeedLimit(double config_lim);

  /*! \brief Set vehicle participation type
   */
  void setVehicleParticipationType(const std::string& participant);
  
  /*! \brief Get vehicle participation type
   */
  std::string getVehicleParticipationType();

  /*! \brief Set endpoint of the route
   */
  void setRouteEndPoint(const lanelet::BasicPoint3d& end_point);

  /*! \brief Set the name of the route
   */
  void setRouteName(const std::string& route_name);
 
  /*! \brief helper for traffic signal Id
   */
  lanelet::Id getTrafficSignalId(uint16_t intersection_id,uint8_t signal_id);

  /*! \brief helper for getting traffic signal with given lanelet::Id
   */
  lanelet::CarmaTrafficSignalPtr getTrafficSignal(const lanelet::Id& id) const;

  /**
   * \brief (non-const version) Gets the underlying lanelet, given the cartesian point on the map 
   *
   * \param point         Cartesian point to check the corressponding lanelet
   * \param n             Number of lanelets to return. Default is 10. As there could be many lanelets overlapping.
   * \throw std::invalid_argument if the map is not set, contains no lanelets
   *
   * \return vector of underlying lanelet, empty vector if it is not part of any lanelet
   */
  std::vector<lanelet::Lanelet> getLaneletsFromPoint(const lanelet::BasicPoint2d& point, const unsigned int n);

  /**
   * \brief (non-const version) Given the cartesian point on the map, tries to get the opposite direction lanelet on the left
   *        This function is intended to find "adjacentLeft lanelets" that doesn't share points between lanelets
   *        where adjacentLeft of lanelet library fails
   *
   * \param point         Cartesian point to check the corressponding lanelet
   * \param n             Number of lanelets to return. Default is 10. As there could be many lanelets overlapping.
   * 
   * \throw std::invalid_argument if the map is not set, contains no lanelets, or if adjacent lanelet is not opposite direction
   * NOTE:  Only to be used on 2 lane, opposite direction road. Number of points in all linestrings are assumed to be roughly the same.
   *        The point is assumed to be on roughly similar shape of overlapping lanelets if any
   * \return vector of underlying lanelet, empty vector if it is not part of any lanelet
   */
  std::vector<lanelet::Lanelet> nonConnectedAdjacentLeft(const lanelet::BasicPoint2d& input_point, const unsigned int n = 10);

  ////
  // Overrides
  ////
  std::pair<TrackPos, TrackPos> routeTrackPos(const lanelet::ConstArea& area) const override;

  TrackPos routeTrackPos(const lanelet::ConstLanelet& lanelet) const override;

  TrackPos routeTrackPos(const lanelet::BasicPoint2d& point) const override;

  std::vector<lanelet::ConstLanelet> getLaneletsBetween(double start, double end, bool shortest_path_only = false,  bool bounds_inclusive = true) const override;

  std::vector<lanelet::BasicPoint2d> sampleRoutePoints(double start_downtrack, double end_downtrack, double step_size) const override;

  boost::optional<lanelet::BasicPoint2d> pointFromRouteTrackPos(const TrackPos& route_pos) const override;

  lanelet::LaneletMapConstPtr getMap() const override;

  LaneletRouteConstPtr getRoute() const override;

  std::string getRouteName() const override;

  TrackPos getRouteEndTrackPos() const override;

  LaneletRoutingGraphConstPtr getMapRoutingGraph() const override;
  
  lanelet::Optional<TrafficRulesConstPtr>
  getTrafficRules(const std::string& participant) const override;

  lanelet::Optional<TrafficRulesConstPtr>
  getTrafficRules() const override;

  std::vector<carma_perception_msgs::msg::RoadwayObstacle> getRoadwayObjects() const override;

  std::vector<carma_perception_msgs::msg::RoadwayObstacle> getInLaneObjects(const lanelet::ConstLanelet& lanelet, const LaneSection& section = LANE_AHEAD) const override;

  lanelet::Optional<lanelet::Lanelet> getIntersectingLanelet (const carma_perception_msgs::msg::ExternalObject& object) const override;

  lanelet::Optional<carma_perception_msgs::msg::RoadwayObstacle> toRoadwayObstacle(const carma_perception_msgs::msg::ExternalObject& object) const override;

  lanelet::Optional<double> distToNearestObjInLane(const lanelet::BasicPoint2d& object_center) const override;

  lanelet::Optional<std::tuple<TrackPos,carma_perception_msgs::msg::RoadwayObstacle>> nearestObjectAheadInLane(const lanelet::BasicPoint2d& object_center) const override;

  lanelet::Optional<std::tuple<TrackPos,carma_perception_msgs::msg::RoadwayObstacle>> nearestObjectBehindInLane(const lanelet::BasicPoint2d& object_center) const override;

  std::vector<lanelet::ConstLanelet> getLane(const lanelet::ConstLanelet& lanelet, const LaneSection& section = LANE_AHEAD) const override;

  size_t getMapVersion() const override;

  std::vector<lanelet::ConstLanelet> getLaneletsFromPoint(const lanelet::BasicPoint2d& point, const unsigned int n = 10) const override;

  std::vector<lanelet::ConstLanelet> nonConnectedAdjacentLeft(const lanelet::BasicPoint2d& input_point, const unsigned int n = 10) const override;

  std::vector<lanelet::CarmaTrafficSignalPtr> getSignalsAlongRoute(const lanelet::BasicPoint2d& loc) const override;

  boost::optional<std::pair<lanelet::ConstLanelet, lanelet::ConstLanelet>> getEntryExitOfSignalAlongRoute(const lanelet::CarmaTrafficSignalPtr& traffic_signal) const override;
  
  std::vector<std::shared_ptr<lanelet::AllWayStop>> getIntersectionsAlongRoute(const lanelet::BasicPoint2d& loc) const override;

  std::vector<lanelet::SignalizedIntersectionPtr> getSignalizedIntersectionsAlongRoute(const lanelet::BasicPoint2d &loc) const;

  std::unordered_map<uint32_t, lanelet::Id> traffic_light_ids_;

  carma_wm::SignalizedIntersectionManager sim_; // records SPAT/MAP lane ids to lanelet ids

private:

  double config_speed_limit_;

  std::string participant_type_ = lanelet::Participants::Vehicle;
  
  /*! \brief Helper function to compute the geometry of the route downtrack/crosstrack reference line
   *         This function should generally only be called from inside the setRoute function as it uses member variables
   * set in that function
   *
   *  Sets the shortest_path_centerlines_, shortest_path_centerlines_lengths_, and
   * shortest_path_filtered_centerline_view_ member variables
   */
  void computeDowntrackReferenceLine();

  /*! \brief Helper function to perform a deep copy of a LineString and assign new ids to all the elements. Used during
   * route centerline construction
   *
   *  \param line The linestring to be coppied
   *
   *  \return A new linestring containing unique ids for all points and the linestring itself
   */
  lanelet::LineString3d copyConstructLineString(const lanelet::ConstLineString3d& line) const;

  std::shared_ptr<lanelet::LaneletMap> semantic_map_;
  LaneletRoutePtr route_;
  LaneletRoutingGraphPtr map_routing_graph_;
  double route_length_ = 0;
  lanelet::LaneletSubmapConstUPtr shortest_path_view_;  // Map containing only lanelets along the shortest path of the
                                                     // route
  std::vector<lanelet::LineString3d> shortest_path_centerlines_;  // List of disjoint centerlines seperated by lane
                                                                  // changes along the shortest path
  IndexedDistanceMap shortest_path_distance_map_;
  lanelet::LaneletMapUPtr shortest_path_filtered_centerline_view_;  // Lanelet map view of shortest path center lines
                                                                    // only
  std::vector<carma_perception_msgs::msg::RoadwayObstacle> roadway_objects_; // 

  size_t map_version_ = 0; // The current map version. This is cached from calls to setMap();

  std::string route_name_; // The current route name. This is set from calls to setRouteName();
  
  // The following constants are default timining plans for recieved traffic lights. 
  // The light is assumed to use these values until otherwise known
  // TODO can these be optional parameters?
  static constexpr double RED_LIGHT_DURATION = 20.0; //in sec
  static constexpr double YELLOW_LIGHT_DURATION = 3.0; //in sec
  static constexpr double GREEN_LIGHT_DURATION = 20.0; //in sec
  
};
}  // namespace carma_wm
