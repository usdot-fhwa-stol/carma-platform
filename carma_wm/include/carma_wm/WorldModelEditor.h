#pragma once

/*
 * Copyright (C) 2021 LEIDOS.
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
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/utility/Optional.h>
#include "TrackPos.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <ros/ros.h>
#include <carma_wm/Geometry.h>

namespace carma_wm
{
/**
 * \brief editor namespace contains implementations for utility read or write functions for stand-alone lanelet map without rest of the CARMAWorldModel features.
 *        NOTE: Functions allowed for normal users are exposed in WorldModel.h interface, so normal users should NOT use this file.
 *        Currently only WMBroadcaster (carma_wm_ctrl) is using this to manipulate map without instance of carma_wm
 */
namespace editor
{

/**
 * \brief Gets the underlying lanelet, given the cartesian point on the map
 *
 * \param semantic_map  Lanelet Map Ptr
 * \param point         Cartesian point to check the corressponding lanelet
 * \param n             Number of lanelets to return. Default is 10. As there could be many lanelets overlapping.
 * \throw std::invalid_argument if the map is not set, contains no lanelets
 *
 * \return vector of underlying lanelet, empty vector if it is not part of any lanelet
 */
std::vector<lanelet::ConstLanelet> getLaneletsFromPoint(const lanelet::LaneletMapConstPtr& semantic_map, const lanelet::BasicPoint2d& point,
                                                          const unsigned int n = 10);

/**
 * \brief (non-const version) Gets the underlying lanelet, given the cartesian point on the map 
 *
 * \param semantic_map  Lanelet Map Ptr
 * \param point         Cartesian point to check the corressponding lanelet
 * \param n             Number of lanelets to return. Default is 10. As there could be many lanelets overlapping.
 * \throw std::invalid_argument if the map is not set, contains no lanelets
 *
 * \return vector of underlying lanelet, empty vector if it is not part of any lanelet
 */
std::vector<lanelet::Lanelet> getLaneletsFromPoint(const lanelet::LaneletMapPtr& semantic_map, const lanelet::BasicPoint2d& point,
                                                                    const unsigned int n = 10);
/**
 * \brief Given the cartesian point on the map, tries to get the opposite direction lanelet on the left
 *        This function is intended to find "adjacentLeft lanelets" that doesn't share points between lanelets
 *        where adjacentLeft of lanelet library fails
 *
 * \param semantic_map  Lanelet Map Ptr
 * \param point         Cartesian point to check the corressponding lanelet
 * \param n             Number of lanelets to return. Default is 10. As there could be many lanelets overlapping.
 * 
 * \throw std::invalid_argument if the map is not set, contains no lanelets
 * NOTE:  Only to be used on 2 lane, opposite direction road. Number of points in all linestrings are assumed to be roughly the same.
 *        The point is assumed to be on roughly similar shape of overlapping lanelets if any
 * \return vector of underlying lanelet, empty vector if it is not part of any lanelet
 */
std::vector<lanelet::ConstLanelet> nonConnectedAdjacentLeft(const lanelet::LaneletMapConstPtr& semantic_map, const lanelet::BasicPoint2d& input_point,
                                                          const unsigned int n = 10);

/**
 * \brief (non-const version) Given the cartesian point on the map, tries to get the opposite direction lanelet on the left
 *        This function is intended to find "adjacentLeft lanelets" that doesn't share points between lanelets
 *        where adjacentLeft of lanelet library fails
 *
 * \param semantic_map  Lanelet Map Ptr
 * \param point         Cartesian point to check the corressponding lanelet
 * \param n             Number of lanelets to return. Default is 10. As there could be many lanelets overlapping.
 * 
 * \throw std::invalid_argument if the map is not set, contains no lanelets
 * NOTE:  Only to be used on 2 lane, opposite direction road. Number of points in all linestrings are assumed to be roughly the same.
 *        The point is assumed to be on roughly similar shape of overlapping lanelets if any
 * \return vector of underlying lanelet, empty vector if it is not part of any lanelet
 */
std::vector<lanelet::Lanelet> nonConnectedAdjacentLeft(const lanelet::LaneletMapPtr& semantic_map, const lanelet::BasicPoint2d& input_point,
                                                          const unsigned int n = 10);

}  // namespace editor

}  // namespace carma_wm