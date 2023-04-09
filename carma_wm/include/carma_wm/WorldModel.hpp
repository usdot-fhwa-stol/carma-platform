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

#include <exception>
#include <memory>
#include <tuple>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_core/utility/Optional.h>
#include <carma_perception_msgs/msg/roadway_obstacle.hpp>
#include <carma_perception_msgs/msg/roadway_obstacle_list.hpp>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <lanelet2_extension/regulatory_elements/CarmaTrafficSignal.h>
#include <lanelet2_extension/regulatory_elements/SignalizedIntersection.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include "carma_wm/TrackPos.hpp"
#include <lanelet2_extension/regulatory_elements/BusStopRule.h>


namespace carma_wm
{

// Helpful using declarations which are not defined by lanelet::routing module
using LaneletRoutePtr = std::shared_ptr<lanelet::routing::Route>;
using LaneletRouteConstPtr = std::shared_ptr<const lanelet::routing::Route>;
using LaneletRouteUPtr = std::unique_ptr<lanelet::routing::Route>;
using LaneletRouteUConstPtr = std::unique_ptr<const lanelet::routing::Route>;

using LaneletRoutingGraphPtr = std::shared_ptr<lanelet::routing::RoutingGraph>;
using LaneletRoutingGraphConstPtr = std::shared_ptr<const lanelet::routing::RoutingGraph>;
using LaneletRoutingGraphUPtr = std::unique_ptr<lanelet::routing::RoutingGraph>;
using LaneletRoutingGraphConstUPtr = std::unique_ptr<const lanelet::routing::RoutingGraph>;

using TrafficRulesConstPtr = std::shared_ptr<const lanelet::traffic_rules::TrafficRules>;
using TrafficRulesUConstPtr = std::unique_ptr<const lanelet::traffic_rules::TrafficRules>;

// Helpful enums for dividing lane into sections of interest.
enum LaneSection
{
  LANE_AHEAD,
  LANE_BEHIND,
  LANE_FULL
};

/*! \brief An interface which provides read access to the semantic map and route.
 *         This class is not thread safe. All units of distance are in meters
 *
 *  Utility functions are provided by this interface for functionality not present in the lanelet2 library such as
 *  computing downtrack and crosstrack distances or road curvatures.
 */
class WorldModel
{
  public:
    /**
     * @brief Virtual destructor to ensure delete safety for pointers to implementing classes
     *
     */
    virtual ~WorldModel() {};

    /*! \brief Returns a pair of TrackPos, computed in 2d, of the provided area relative to the current route.
    *        The distance is based on the Area vertex with the smallest and largest downtrack distance
    *        This method overload is the most expensive of the routeTrackPos methods
    *
    * NOTE: The route definition used in this class contains discontinuities in the reference line at lane changes. It is
    * important to consider that when using route related functions.
    *
    * \param area The lanelet2 area which will have its distance computed
    *
    * \throws std::invalid_argument If the route is not yet loaded or the area does not contain vertices
    *
    * \return A pair where the first element contains the TrackPos area's earliest downtrack vertex and the second
    * element contains the area's latest downtrack point
    */
    virtual std::pair<TrackPos, TrackPos> routeTrackPos(const lanelet::ConstArea& area) const = 0;


    /*! \brief Returns the TrackPos, computed in 2d, of the provided lanelet relative to the current route.
    *        The distance is based on the first point in the lanelet centerline
    *
    * NOTE: The route definition used in this class contains discontinuities in the reference line at lane changes. It is
    * important to consider that when using route related functions.
    *
    * \param lanelet The lanelet2 lanelet which will have its distance computed
    *
    * \throws std::invalid_argument If the route is not yet loaded or the lanelet centerline cannot be found
    *
    * \return The TrackPos containing the downtrack and crosstrack location of the lanelet's first centerline point
    */
    virtual TrackPos routeTrackPos(const lanelet::ConstLanelet& lanelet) const = 0;

    /*! \brief Returns the TrackPos, computed in 2d, of the provided point relative to the current route
    *
    * NOTE: The route definition used in this class contains discontinuities in the reference line at lane changes. It is
    * important to consider that when using route related functions.
    *
    * \param point The lanelet2 point which will have its distance computed
    *
    * \throws std::invalid_argument If the route is not yet loaded
    *
    * \return The TrackPos of the point
    */
    virtual TrackPos routeTrackPos(const lanelet::BasicPoint2d& point) const = 0;

    /*! \brief Returns a list of lanelets which are part of the route and whose downtrack bounds exist within the provided
    * start and end distances. 
    *
    * \param start The starting downtrack for the query
    * \param end The ending downtrack for the query.
    * \param shortest_path_only If true the lanelets returned will be part of the route shortest path
    * param bounds_inclusive If true, the bounds are included so areas which end exactly at start or start exactly at end are
    * included. NOTE: Non-inclusive behavior toggled by !bounds_inclusive is not equivalent to a != check as it merely shrinks bounds
    * by 0.00001 to get new start and end distances. 
    *
    * \throws std::invalid_argument If the route is not yet loaded or if start > end
    *
    * \return A list of lanelets which contain regions that lie between start and end along the route. This function will
    * not return lanelets which are not part of the route
    */
    virtual std::vector<lanelet::ConstLanelet> getLaneletsBetween(double start, double end, bool shortest_path_only = false,
                                                                bool bounds_inclusive = true) const = 0;

    /*! \brief Samples the route centerline between the provided downtracks with the provided step size. 
    *         At lane changes the points should exhibit a discontinuous jump at the end of the initial lanelet
    *         to the end of the lane change lanelet as expected by the route definition. 
    *         Returned points are always inclusive of provided bounds, so the last step might be less than step_size. 
    * 
    *  NOTE: If start_downtrack == end_downtrack a single point is returned. 
    *        If the route is not set or the bounds lie outside the route an empty vector is returned.
    *        
    *        In the default implementation, this method has m * O(log n) complexity where n is the number of lane changes in the route + 1
    *        and m is the number of sampled points which is nominally 1 + ((start_downtrack - end_downtrack) / step_size). 
    *        Since the route is fixed for the duration of method execution this can generally be thought of as a linear complexity call dominated by m.
    * 
    *  \param start_downtrack The starting route downtrack to sample from in meters
    *  \param end_downtrack The ending downtrack to stop sampling at in meters
    *  \param step_size The sampling step size in meters.
    * 
    *  \return The sampled x,y points
    * 
    * NOTE: This method really needs clarification of behavior for lane changes.
    */
    virtual std::vector<lanelet::BasicPoint2d> sampleRoutePoints(double start_downtrack, double end_downtrack,
                                                               double step_size) const = 0;        


    /*! \brief Converts a route track position into a map frame cartesian point.
    *
    *  \param route_pos The TrackPos to convert to and x,y point. This position should be relative to the route
    * 
    *  \return If the provided downtrack position is not within the route bounds or the route is not set then boost::none
    *  is returned. Otherwise the point is converted to x,y. 
    * 
    *  NOTE: This method will run faster if the crosstrack is unset or set to 0. 
    *        The default implementation lookup has complexity O(log n) where n is the number of lane changes is a route + 1
    *
    */
    virtual boost::optional<lanelet::BasicPoint2d> pointFromRouteTrackPos(const TrackPos& route_pos) const = 0;     

    /*! \brief Get a pointer to the current map. If the underlying map has changed the pointer will also need to be
    * reacquired
    *
    * \return Shared pointer to underlying lanelet map. Pointer will return false on boolean check if no map is loaded
    */
    virtual lanelet::LaneletMapConstPtr getMap() const = 0;    

    /*! \brief Get a pointer to the current route. If the underlying route has changed the pointer will also need to be
    * reacquired
    *
    * \return Shared pointer to underlying lanelet route. Pointer will return false on boolean check if no route or map
    * is loaded
    */
    virtual LaneletRouteConstPtr getRoute() const = 0;    

    /*! \brief Get the current route name. 
    *
    * \return A string that matches the name of the current route.
    */
    virtual std::string getRouteName() const = 0;    

    /*! \brief Get trackpos of the end of route point relative to the route
    *
    * \return Trackpos
    */
    virtual TrackPos getRouteEndTrackPos() const = 0;  

    /*! \brief Get a pointer to the routing graph for the current map. If the underlying map has changed the pointer will
    * also need to be reacquired
    *
    * \return Shared pointer to underlying lanelet route graph. Pointer will return false on boolean check if no map is
    * loaded
    */
    virtual LaneletRoutingGraphConstPtr getMapRoutingGraph() const = 0;

    /*! \brief Get most recent roadway objects - all objects on the road detected by perception stack.
    *
    * \return Vector list of RoadwayObstacle which are lanelet compatible. Empty vector if no object found.
    */
    virtual std::vector<carma_perception_msgs::msg::RoadwayObstacle> getRoadwayObjects() const = 0;

    /*! \brief Get a pointer to the traffic rules object used internally by the world model and considered the carma
    * system default
    *
    * \param participant The lanelet participant to return the traffic rules object for. Defaults to a generic vehicle
    *
    * \return Optional Shared pointer to an intialized traffic rules object which is used by carma. Optional is false if
    * no rule set is available for the requested participant.
    */
    virtual lanelet::Optional<TrafficRulesConstPtr>
    getTrafficRules(const std::string& participant) const = 0;

    /**
     * @brief Get the Traffic Rules object 
     * @return  Optional Shared pointer to an intialized traffic rules object which is used by carma. A default participant value will be used
     * in case setVehicleParticipationType is not called. Acceptable participants are Vehicle, VehicleCar, and VehicleTruck
     */
    virtual lanelet::Optional<TrafficRulesConstPtr>
    getTrafficRules() const = 0;

    /**
     * \brief Converts an ExternalObject in a RoadwayObstacle by mapping its position onto the semantic map. Can also be
     * used to determine if the object is on the roadway
     *
     * \param object the external object to convert
     *
     * \throw std::invalid_argument if the map is not set or contains no lanelets
     *
     * \return An optional RoadwayObstacle message created from the provided object. If the external object is not on the
     * roadway then the optional will be empty.
    */
    virtual lanelet::Optional<carma_perception_msgs::msg::RoadwayObstacle>
    toRoadwayObstacle(const carma_perception_msgs::msg::ExternalObject& object) const = 0;

    /**
     * \brief Gets the a lanelet the object is currently on determined by its position on the semantic map. If it's
     * across multiple lanelets, get the closest one
     *
     * \param object the external object to get the lanelet of
     *
     * \throw std::invalid_argument if the map is not set or contains no lanelets
     *
     * \return An optional lanelet primitive that is on the semantic map. If the external object is not on the
     * roadway then the optional will be empty.
    */

    virtual lanelet::Optional<lanelet::Lanelet> getIntersectingLanelet(const carma_perception_msgs::msg::ExternalObject& object) const = 0;

     /**
     * \brief  Return a list of bus stop along the current route.  
     * The bus stop along a route and the next bus stop ahead of us on the route specifically, 
     * so a sorted list (by downtrack distance) of bus stop on the route ahead of us thus eliminating those behind the vehicle.
     *
     * \param loc location
     * \throw std::invalid_argument if the map is not set, contains no lanelets, or route is not set
     *
     * \return list of bus stop along the current route
   */
    virtual std::vector<lanelet::BusStopRulePtr> getBusStopsAlongRoute(const lanelet::BasicPoint2d& loc) const = 0;
    
    /**
     * \brief Gets all roadway objects currently in the same lane as the given lanelet
     *
     * \param lanelet the lanelet that is part of the continuous lane
     * \param section either of LANE_AHEAD, LANE_BEHIND, LANE_FULL each including the current lanelet
     *
     * \throw std::invalid_argument if the map is not set, contains no lanelets, or if the given
     * lanelet is not on the current semantic map, or lane section input is not of the three
     *
     * \return A vector of RoadwayObstacle objects that is on the current lane.
     * Return empty vector if there is no objects on current lane or the road
     */

    virtual std::vector<carma_perception_msgs::msg::RoadwayObstacle> getInLaneObjects(const lanelet::ConstLanelet& lanelet,
                                                                  const LaneSection& section = LANE_AHEAD) const = 0;

    /**
     * \brief Gets Cartesian distance to the closest object on the same lane as the given point
     *
     * \param object_center the point to measure the distance from
     *
     * \throw std::invalid_argument if the map is not set, contains no lanelets, or the given point
     * is not on the current semantic map
     *
     * \return An optional Cartesian distance in double to the closest in lane object. Return empty if there is no objects
     * on current lane or the road
     */
    virtual lanelet::Optional<double> distToNearestObjInLane(const lanelet::BasicPoint2d& object_center) const = 0;

    /**
     * \brief Gets Downtrack distance to AND copy of the closest object AHEAD on the same lane as the given point. Also
     * returns crosstrack distance relative to that object. Plus downtrack if the object is ahead along the lane, and also
     * plus crosstrack if the object is to the right relative to the reference line that crosses given object_center and
     * is parallel to the centerline of the lane.
     *
     * \param object_center the point to measure the distance from
     *
     * \throw std::invalid_argument if the map is not set, contains no lanelets, or the given point
     * is not on the current semantic map
     *
     * \return An optional tuple of <TrackPos, carma_perception_msgs::msg::RoadwayObstacle> to the closest in lane object AHEAD. Return
     * empty if there is no objects on current lane or the road
     */
    virtual lanelet::Optional<std::tuple<TrackPos, carma_perception_msgs::msg::RoadwayObstacle>>
    nearestObjectAheadInLane(const lanelet::BasicPoint2d& object_center) const = 0;

    /**
     * \brief Gets Downtrack distance to AND copy of the closest object BEHIND on the same lane as the given point. Also
     * returns crosstrack distance relative to that object. Plus downtrack if the object is ahead along the lane, and also
     * plus crosstrack if the object is to the right relative to the reference line that crosses given object_center and
     * is parallel to the centerline of the lane.
     *
     * \param object_center the point to measure the distance from
     *
     * \throw std::invalid_argument if the map is not set, contains no lanelets, or the given point
     * is not on the current semantic map
     *
     * \return An optional tuple of <TrackPos, carma_perception_msgs::msg::RoadwayObstacle> to the closest in lane object BEHIND. Return
     * empty if there is no objects on current lane or the road
     */
    virtual lanelet::Optional<std::tuple<TrackPos, carma_perception_msgs::msg::RoadwayObstacle>>
    nearestObjectBehindInLane(const lanelet::BasicPoint2d& object_center) const = 0;

    /**
     * \brief Gets the specified lane section achievable without lane change, sorted from the start, that includes the
     * given lanelet
     *
     * \param lanelet the lanelet to get the full lane of
     * \param section either of LANE_AHEAD, LANE_BEHIND, LANE_FULL each including the current lanelet
     *
     * \throw std::invalid_argument if the map is not set, contains no lanelets, or the given lanelet
     * is not on the current semantic map, or lane section input is not of the three
     *
     * \return An optional vector of ConstLanalet. Returns at least the vector of given lanelet if no other is found
     */
    virtual std::vector<lanelet::ConstLanelet> getLane(const lanelet::ConstLanelet& lanelet,
                                                     const LaneSection& section = LANE_AHEAD) const = 0;

    /**
     * \brief Returns a monotonically increasing version number which represents the version stamp of the map geometry data
     *        It is possible for the non-geometric aspects of the map to change without this value increasing.
     * 
     * \return map version
     */ 
    virtual size_t getMapVersion() const = 0;

    /**
     * \brief Gets the underlying lanelet, given the cartesian point on the map
     *
     * \param point         Cartesian point to check the corressponding lanelet
     * \param n             Number of lanelets to return. Default is 10. As there could be many lanelets overlapping.
     * \throw std::invalid_argument if the map is not set, contains no lanelets
     *
     * \return vector of underlying lanelet, empty vector if it is not part of any lanelet
     */
    virtual std::vector<lanelet::ConstLanelet> getLaneletsFromPoint(const lanelet::BasicPoint2d& point, const unsigned int n = 10) const = 0;

    /**
     * \brief  Return a list of traffic lights/intersections along the current route.  
     * The traffic lights along a route and the next traffic light ahead of us on the route specifically, 
     * so a sorted list (by downtrack distance) of traffic lights on the route ahead of us thus eliminating those behind the vehicle.
     *
     * \param loc location
     * \throw std::invalid_argument if the map is not set, contains no lanelets, or route is not set
     *
     * \return list of traffic lights along the current route
   */
    virtual std::vector<lanelet::CarmaTrafficSignalPtr> getSignalsAlongRoute(const lanelet::BasicPoint2d& loc) const = 0;

    /**
   * \brief Returns the entry and exit lanelet of the signal along the SHORTEST PATH of route. This is useful if traffic_signal controls
   *        both directions in an intersection for example. 
   *
   * \param traffic_signal traffic signal of interest
   * \throw std::invalid_argument if nullptr is passed in traffic_signal
   *
   * \return pair of entry and exit lanelet of the signal along the route. boost::none if the given signal doesn't have both entry/exit pair along the shortest path route.
   */
  virtual boost::optional<std::pair<lanelet::ConstLanelet, lanelet::ConstLanelet>> getEntryExitOfSignalAlongRoute(const lanelet::CarmaTrafficSignalPtr& traffic_signal) const = 0;

    /**
     * \brief  Return a list of all way stop intersections along the current route.  
     * The tall way stop intersections along a route and the next all way stop intersections ahead of us on the route specifically, 
     * so a sorted list (by downtrack distance) of all way stop intersections on the route ahead of us thus eliminating those behind the vehicle.
     *
     * \param loc location
     * \throw std::invalid_argument if the map is not set, contains no lanelets, or route is not set
     *
     * \return list of all way stop intersections along the current route
     */
    virtual std::vector<std::shared_ptr<lanelet::AllWayStop>> getIntersectionsAlongRoute(const lanelet::BasicPoint2d& loc) const = 0;

    /**
     * \brief  Return a list of signalized intersections along the current route.  
     * The signalized intersections along a route and the next signalized intersections ahead of us on the route specifically, 
     * so a sorted list (by downtrack distance) of signalized intersections on the route ahead of us thus eliminating those behind the vehicle.
     *
     * \param loc location
     * \throw std::invalid_argument if the map is not set, contains no lanelets, or route is not set
     *
     * \return list of signalized intersections along the current route
     */
    virtual std::vector<lanelet::SignalizedIntersectionPtr> getSignalizedIntersectionsAlongRoute(const lanelet::BasicPoint2d &loc) const = 0;

    /**
     * \brief Given the cartesian point on the map, tries to get the opposite direction lanelet on the left
     *        This function is intended to find "adjacentLeft lanelets" that doesn't share points between lanelets
     *        where adjacentLeft of lanelet library fails
     *
     * \param semantic_map  Lanelet Map Ptr
     * \param point         Cartesian point to check the corressponding lanelet
     * \param n             Number of lanelets to return. Default is 10. As there could be many lanelets overlapping.
     * 
     * \throw std::invalid_argument if the map is not set, contains no lanelets, or if adjacent lanelet is not opposite direction
     * NOTE:  Only to be used on 2 lane, opposite direction road. Number of points in all linestrings are assumed to be roughly the same.
     *        The point is assumed to be on roughly similar shape of overlapping lanelets if any
     * \return vector of underlying lanelet, empty vector if it is not part of any lanelet
     */
    virtual std::vector<lanelet::ConstLanelet> nonConnectedAdjacentLeft(const lanelet::BasicPoint2d& input_point, const unsigned int n = 10) const = 0;
                                                                                             
};
// Helpful using declarations for carma_wm classes
using WorldModelConstPtr = std::shared_ptr<const WorldModel>;
}  // namespace carma_wm