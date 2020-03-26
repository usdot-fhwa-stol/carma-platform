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
#include <tuple>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/utility/Optional.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include "TrackPos.h"

namespace carma_wm
{
/**
 * \brief geometry namespace contains utility geometry functions which do not require the map or route provided by an
 * active world model
 */
namespace geometry
{
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
TrackPos trackPos(const lanelet::ConstLanelet& lanelet, const lanelet::BasicPoint2d& point);

/*! \brief Return the track position of a point relative to a line segment defined by two other points.
 *         Positive crosstrack will be to the right. Points occuring before the segment will have negative downtrack
 *
 *  \param p The point to find the TrackPos of
 *  \param seg_start The starting point of a line segment
 *  \param seg_end The ending point of a line segment
 *
 *  \return The TrackPos of point p relative to the line defined by seg_start and seg_end
 */
TrackPos trackPos(const lanelet::BasicPoint2d& p, const lanelet::BasicPoint2d& seg_start,
                  const lanelet::BasicPoint2d& seg_end);
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
 * \throw std::invalid_argument if line string contains only one point
 *
 * \return An std::tuple where the first element is the TrackPos of the point and the second element is the matched
 * segment
 */
std::tuple<TrackPos, lanelet::BasicSegment2d> matchSegment(const lanelet::BasicPoint2d& p,
                                                           const lanelet::BasicLineString2d& line_string);

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
 * \throw std::invalid_argument If one of the provided lanelets cannot have its centerline computed
 *
 * \return A list of continuous centerline segments and their respective curvatures
 */
std::vector<std::tuple<size_t, std::vector<double>>>
getLocalCurvatures(const std::vector<lanelet::ConstLanelet>& lanelets);

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
double computeCurvature(const lanelet::BasicPoint2d& p1, const lanelet::BasicPoint2d& p2,
                        const lanelet::BasicPoint2d& p3);

/**
 * \brief Calculates the angle between two vectors.
 *
 * \param vec1 the first vector
 * \param vec2 the second vector
 *
 * \return The angle in rad between the two vectors
 */
double getAngleBetweenVectors(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2);

/**
 * \brief Uses the provided pose and size vector of an ExternalObject to compute what the polygon would be of that
 * object would be if viewed from the same frame as the pose is defined relative to.
 *
 * \param pose the pose of an external object
 * \param size The size vector of an external object
 *
 * \return A polygon of 4 points describing the object aligned bounds starting with the upper left point in the object
 * frame and moving clockwise
 */
lanelet::BasicPolygon2d objectToMapPolygon(const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& size);

}  // namespace geometry

}  // namespace carma_wm