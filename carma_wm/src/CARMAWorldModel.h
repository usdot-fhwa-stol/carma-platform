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

#include <carma_wm/WorldModel.h>
#include <lanelet2_core/primitives/LineString.h>
#include "IndexedDistanceMap.h"

namespace carma_wm
{
/*! \brief Class which implements the WorldModel interface. In addition this class provides write access to the world
 * model. Write access is achieved through setters for the Map and Route.
 *
 *  Proper usage of this class dictates that the Map and Route object be kept in sync
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
  CARMAWorldModel();

  /**
   * @brief Destructor as required by interface
   *
   */
  ~CARMAWorldModel();

  /*! \brief Set the current map
   *
   *  \param map A shared pointer to the map which will share ownership to this object
   */
  void setMap(lanelet::LaneletMapPtr map);

  /*! \brief Set the current route. This route must match the current map for this class to function properly
   *
   *  \param route A shared pointer to the route which will share ownership to this object
   */
  void setRoute(LaneletRoutePtr route);

  ////
  // Overrides
  ////
  std::pair<TrackPos, TrackPos> routeTrackPos(const lanelet::ConstArea& area) const override;

  TrackPos routeTrackPos(const lanelet::ConstLanelet& lanelet) const override;

  TrackPos routeTrackPos(const lanelet::BasicPoint2d& point) const override;

  TrackPos trackPos(const lanelet::ConstLanelet& lanelet, const lanelet::BasicPoint2d& point) const override;

  TrackPos trackPos(const lanelet::BasicPoint2d& p, const lanelet::BasicPoint2d& seg_start,
                    const lanelet::BasicPoint2d& seg_end) const override;

  std::tuple<TrackPos, lanelet::BasicSegment2d>
  matchSegment(const lanelet::BasicPoint2d& p, const lanelet::BasicLineString2d& line_string) const override;

  std::vector<lanelet::ConstLanelet> getLaneletsBetween(double start, double end) const override;

  std::vector<std::tuple<size_t, std::vector<double>>>
  getLocalCurvatures(const std::vector<lanelet::ConstLanelet>& lanelets) const override;

  lanelet::LaneletMapConstPtr getMap() const override;

  LaneletRouteConstPtr getRoute() const override;

  LaneletRoutingGraphConstPtr getMapRoutingGraph() const override;

  double computeCurvature(const lanelet::BasicPoint2d& p1, const lanelet::BasicPoint2d& p2,
                          const lanelet::BasicPoint2d& p3) const override;

  double getAngleBetweenVectors(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) const override;

  lanelet::Optional<TrafficRulesConstPtr>
  getTrafficRules(const std::string& participant = lanelet::Participants::Vehicle) const override;

private:
  /*! \brief Helper function to compute the geometry of the route downtrack/crosstrack reference line
   *         This function should generally only be called from inside the setRoute function as it uses member variables
   * set in that function
   *
   *  Sets the shortest_path_centerlines_, shortest_path_centerlines_lengths_, and
   * shortest_path_filtered_centerline_view_ member variables
   */
  void computeDowntrackReferenceLine();

  /*! \brief Helper function to identify whether to select the preceeding or succeeding segment of a linstring point
   * that is nearest an external point when trying to find the TrackPos of the external point. Of the 3 points
   * comprising the 2 segment linestirng the external point must be closest to the midpoint for this function to be
   * valid
   *
   * ASSUMPTION: Of the 3 points comprising the 2 segment linestirng the external point is closest to the midpoint
   *
   * Meant to be called in the matchSegment method. Logic is as follows
   * If the external point is within the downtrack bounds of one segment but not the other then use the one it is within
   * If the external point is within the downtrack bounds of both segments then use the one with the smallest crosstrack
   * distance If the external point is within the downtrack bounds of both segments and has exactly equal crosstrack
   * bounds with each segment then use the preceeding segment If the external point is outside the downtrack bounds of
   * both segments then assign to the first segment as it will always have positive downtrack in this case while the
   * second segment will always have negative downtrack
   *
   * \param first_seg_trackPos The TrackPos of the external point relative to the preceeding segment
   * \param second_seg_trackPos The TrackPos of the succeeding segment
   * \param first_seg_length The length of the preceeding segment
   * \param second_seg_length The length of the succeeding segment
   *
   * \return True if the preceeding segment is the better choice. False if the succeeding segment is the better choice.
   */
  bool selectFirstSegment(const TrackPos& first_seg_trackPos, const TrackPos& second_seg_trackPos,
                          double first_seg_length, double second_seg_length) const;

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

  lanelet::LaneletMapConstUPtr shortest_path_view_;  // Map containing only lanelets along the shortest path of the
                                                     // route
  std::vector<lanelet::LineString3d> shortest_path_centerlines_;  // List of disjoint centerlines seperated by lane
                                                                  // changes along the shortest path
  IndexedDistanceMap shortest_path_distance_map_;
  lanelet::LaneletMapUPtr shortest_path_filtered_centerline_view_;  // Lanelet map view of shortest path center lines
                                                                    // only
};
}  // namespace carma_wm