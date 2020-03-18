#pragma once

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

/*! \brief Position in a track based coordinate system where the axis are downtrack and crosstrack.
 *         Positive crosstrack is to the left of the reference line
 *
 * The position of a point relative to a reference line. The perpendicular distance from the point to the reference line
 * is the crosstrack The distance to the intersection of the perpendicular from the start of the line is the downtrack
 * distance If an x,y point would be located to the right of the reference line then the crosstrack is positive. It is
 * negative if on the left side of the line. NOTE: The positive/negative crosstrack side is flipped relative to a
 * lanelet::ArcCoordinate Most utility functions will assume that the downtrack/crosstrack values are specified in
 * meters
 */
class TrackPos
{
public:
  double downtrack = 0;
  double crosstrack = 0;

  /*! \brief Constructor
   *
   * \param down_track The downtrack distance
   * \param cross_track The crosstrack distance
   */
  TrackPos(double down_track, double cross_track) : downtrack(down_track), crosstrack(cross_track)
  {
  }
  /*! \brief Constructor from lanelet ArcCoordinates
   *         which are converted to TrackPos where downtrack = ArcCoordinates.length and crosstrack =
   * -ArcCoordinates.distance
   *
   * \param arc_coord ArcCoordinate to copy
   */
  TrackPos(const lanelet::ArcCoordinates& arc_coord) : downtrack(arc_coord.length), crosstrack(-arc_coord.distance)
  {
  }

  /*! \brief Returns a lanelet ArcCoordinate built from this TrackPos
   *        where downtrack = ArcCoordinates.length and crosstrack = -ArcCoordinates.distance
   *
   * \return lanelet::ArcCoordinates with values copied from this object
   */
  inline lanelet::ArcCoordinates toArcCoordinates()
  {
    return lanelet::ArcCoordinates{ downtrack, -crosstrack };
  }

  // Overload == and != operators
  bool operator==(const TrackPos& other) const
  {
    return this == &other || (this->downtrack == other.downtrack && this->crosstrack == other.crosstrack);
  }

  bool operator!=(const TrackPos& other) const
  {
    return !(*this == other);
  }
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
  virtual ~WorldModel(){};

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

  /*! \brief Returns the TrackPos, computed in 2d, of the provided point relative to the centerline of the provided
   * lanelet. Positive crosstrack will be to the right. Points occuring before the segment will have negative downtrack.
   *        See the matchSegment function for a description of the edge cases associated with the max_crosstrack
   * parameter
   *
   * \param lanelet The lanelet whose centerline will serve as the TrackPos reference line
   * \param point The lanelet2 point which will have its distance computed
   * \param max_crosstrack The maximum crosstrack distance allowed for a perfect centerline segment match. See the
   * matchSegment function for details
   *
   * \throws std::invalid_argument If the lanelet centerline cannot be found
   *
   * \return The TrackPos of the point relative to the lanelet centerline
   */
  virtual TrackPos trackPos(const lanelet::ConstLanelet& lanelet, const lanelet::BasicPoint2d& point) const = 0;

  /*! \brief Return the track position of a point relative to a line segment defined by two other points.
   *         Positive crosstrack will be to the right. Points occuring before the segment will have negative downtrack
   *
   *  \param p The point to find the TrackPos of
   *  \param seg_start The starting point of a line segment
   *  \param seg_end The ending point of a line segment
   *
   *  \return The TrackPos of point p relative to the line defined by seg_start and seg_end
   */
  virtual TrackPos trackPos(const lanelet::BasicPoint2d& p, const lanelet::BasicPoint2d& seg_start,
                            const lanelet::BasicPoint2d& seg_end) const = 0;

  /**
   * \brief Get the TrackPos of the provided point along the linestring and the segment of the line string which the
   * provided point should be considered in.
   *
   * This function iterates over the provided linestring from front to back.
   * First the nearest vertex point on the linestring to the external point is found
   * Once the nearest point has been found next step is to determine which segment it should go with using the following
   * rules. If the minimum point is the first point then use the first segment If the minimum point is the last point
   * then use the last segment If the minimum point is within the downtrack bounds of one segment but not the other then
   * use the one it is within If the minimum point is within the downtrack bounds of both segments then use the one with
   * the smallest crosstrack distance If the minimum point is within the downtrack bounds of both segments and has
   * exactly equal crosstrack bounds with each segment then use the first one
   *
   * \param point The 2d point to match with a segment
   * \param line_string The line_string to match against
   *
   * \throws std::invalid_argument if line string contains only one point
   *
   * \return An std::tuple where the first element is the TrackPos of the point and the second element is the matched
   * segment
   */
  virtual std::tuple<TrackPos, lanelet::BasicSegment2d>
  matchSegment(const lanelet::BasicPoint2d& p, const lanelet::BasicLineString2d& line_string) const = 0;

  /*! \brief Returns a list of lanelets which are part of the route and whose downtrack bounds exist within the provided
   * start and end distances The bounds are included so areas which end exactly at start or start exactly at end are
   * included
   *
   * \param start The starting downtrack for the query
   * \param end The ending downtrack for the query
   *
   * \throws std::invalid_argument If the route is not yet loaded or if start >= end
   *
   * \return A list of lanelets which contain regions that lie between start and end along the route. This function will
   * not return lanelets which are not part of the route
   */
  virtual std::vector<lanelet::ConstLanelet> getLaneletsBetween(double start, double end) const = 0;

  /*! \brief Returns a list of lists of local (3-point) curvatures, computed in 2d. Each continuous segment of the
   * lanelets' centerlines is one elemtent in the first list. Where each lane change occurs along the list of lanelets a
   * new list of curvatures is started.
   *
   * Each elemnent in the first list contains a tuple where the first element is the index of the lanelet which is the
   * starting point of that segment. The second element of the tuple contains the list of local curvatures for that
   * segment. These map to the points on the lanlet centerlines excluding first and last point of the continuous
   * centerline segments
   *
   * \param lanelets The list of lanelets to compute curvatures for
   *
   * \throws std::invalid_argument If one of the provided lanelets cannot have its centerline computed
   *
   * \return A list of continuous centerline segments and their respective curvatures
   */
  virtual std::vector<std::tuple<size_t, std::vector<double>>>
  getLocalCurvatures(const std::vector<lanelet::ConstLanelet>& lanelets) const = 0;

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

  /*! \brief Get a pointer to the routing graph for the current map. If the underlying map has changed the pointer will
   * also need to be reacquired
   *
   * \return Shared pointer to underlying lanelet route graph. Pointer will return false on boolean check if no map is
   * loaded
   */
  virtual LaneletRoutingGraphConstPtr getMapRoutingGraph() const = 0;

  /*! \brief Get a pointer to the traffic rules object used internally by the world model and considered the carma
   * system default
   *
   * \param participant The lanelet participant to return the traffic rules object for. Defaults to a generic vehicle
   *
   * \return Optional Shared pointer to an intialized traffic rules object which is used by carma. Optional is false if
   * no rule set is available for the requested participant.
   */
  virtual lanelet::Optional<TrafficRulesConstPtr>
  getTrafficRules(const std::string& participant = lanelet::Participants::Vehicle) const = 0;

  /*! \brief Function for computing curvature from 3 points.
   *
   * This function is a direct copy of the function by the same name found in the lanelet2_validation package which was
   * not exposed for use The origianal function can be found here
   * https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_validation/src/validators/CurvatureTooBig.cpp
   * The source function is copyrighted under BSD 3-Clause "New" or "Revised" License a copy of that notice has been
   * included with this package
   *
   * \param p1 The first point
   * \param p2 The second point
   * \param p3 The third point
   *
   * \return The computed curvature in 1/m units.
   */
  virtual double computeCurvature(const lanelet::BasicPoint2d& p1, const lanelet::BasicPoint2d& p2,
                                  const lanelet::BasicPoint2d& p3) const = 0;

  /**
   * \brief Calculates the angle between two vectors.
   *
   * \param vec1 the first vector
   * \param vec2 the second vector
   *
   * \return The angle in rad between the two vectors
   */
  virtual double getAngleBetweenVectors(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) const = 0;
};

// Helpful using declarations for carma_wm classes
using WorldModelConstPtr = std::shared_ptr<const WorldModel>;
}  // namespace carma_wm
