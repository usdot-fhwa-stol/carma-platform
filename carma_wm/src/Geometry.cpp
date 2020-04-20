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

#include <carma_wm/Geometry.h>
#include <lanelet2_core/geometry/Point.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace carma_wm
{
namespace geometry
{

double getAngleBetweenVectors(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2)
{
  double vec1Mag = vec1.norm();
  double vec2Mag = vec2.norm();
  if (vec1Mag == 0 || vec2Mag == 0)
  {
    return 0;
  }
  return std::acos(vec1.dot(vec2) / (vec1Mag * vec2Mag));
}

TrackPos trackPos(const lanelet::BasicPoint2d& p, const lanelet::BasicPoint2d& seg_start,
                  const lanelet::BasicPoint2d& seg_end)
{
  Eigen::Vector2d vec_to_p(p);
  Eigen::Vector2d vec_to_start(seg_start);
  Eigen::Vector2d vec_to_end(seg_end);

  // Get vector from start to external point
  Eigen::Vector2d start_to_p = vec_to_p - vec_to_start;

  // Get vector from start to end point
  Eigen::Vector2d start_to_end = vec_to_end - vec_to_start;

  // Get angle between both vectors
  double interior_angle = getAngleBetweenVectors(start_to_p, start_to_end);

  // Calculate downtrack distance
  double start_to_p_mag = start_to_p.norm();
  double downtrack_dist = start_to_p_mag * std::cos(interior_angle);

  /**
   * Calculate the sign of the crosstrack distance by projecting the points to 2d
   * d = (p_x - s_x)(e_y - s_y) - (p_y - s_y)(e_x - s_x)
   * Equivalent to d = (start_to_p.x * start_to_end.y) - (start_to_p.y * start_to_end.x)
   *
   * Code below based on math equation described at
   * https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
   * Question asked by user
   * Ritvars (https://math.stackexchange.com/users/56723/ritvars)
   * and answered by user
   * Shard (https://math.stackexchange.com/users/55608/shard)
   * Attribution here is in line with Stack Overflow's Attribution policy cc-by-sa found here:
   * https://stackoverflow.blog/2009/06/25/attribution-required/
   */

  double d = (start_to_p[0] * start_to_end[1]) - (start_to_p[1] * start_to_end[0]);
  double sign = d >= 0 ? 1.0 : -1.0;  // If d is positive then the point is to the right if it is negative the point is
                                      // to the left

  double crosstrack = start_to_p_mag * std::sin(interior_angle) * sign;

  return TrackPos(downtrack_dist, crosstrack);
}

TrackPos trackPos(const lanelet::ConstLanelet& lanelet, const lanelet::BasicPoint2d& point)
{
  auto center_line = lanelet::utils::to2D(lanelet.centerline());

  if (center_line.numSegments() < 1)
  {
    throw std::invalid_argument("Provided lanelet has invalid centerline containing no points");
  }
  auto tuple = matchSegment(point, center_line.basicLineString());

  return std::get<0>(tuple);
}

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
                        double first_seg_length, double second_seg_length)
{
  const bool first_seg_in_downtrack =
      (0 <= first_seg_trackPos.downtrack && first_seg_trackPos.downtrack < first_seg_length);
  const bool second_seg_in_downtrack =
      (0 <= second_seg_trackPos.downtrack && second_seg_trackPos.downtrack < second_seg_length);

  if (first_seg_in_downtrack && !second_seg_in_downtrack)
  {  // If in the first segment but not the last segment

    return true;  // First segment is better
  }
  else if (second_seg_in_downtrack && !first_seg_in_downtrack)
  {
    return false;  // Second segment is better
  }
  else if (first_seg_in_downtrack && second_seg_in_downtrack)
  {
    return first_seg_trackPos.crosstrack <=
           second_seg_trackPos.crosstrack;  // Pick the first segment if the crosstrack values are equal or the first
                                            // segment is closer
  }
  else
  {  // Point lies outside the downtrack bounds of both segments (Remember According to function contract the nearest
     // point to external point is the midpoint of the two segments)
    return true;  // Choose first segment as it will always have positive downtrack in this case while the second
                  // segment will always have negative downtrack
  }
}

std::tuple<TrackPos, lanelet::BasicSegment2d> matchSegment(const lanelet::BasicPoint2d& p,
                                                           const lanelet::BasicLineString2d& line_string)
{
  if (line_string.size() < 2)
  {
    throw std::invalid_argument("Provided with linestring containing fewer than 2 points");
  }

  lanelet::BasicSegment2d best_segment =
      std::make_pair(line_string[0], line_string[1]);  // Default to starting segment if no match is found
  auto point_2d = lanelet::utils::to2D(p);

  double min_distance = lanelet::geometry::distance2d(point_2d, line_string[0]);
  size_t best_point_index = 0;
  double best_accumulated_length = 0;
  double best_last_accumulated_length = 0;
  double best_seg_length = 0;
  double best_last_seg_length = 0;
  double last_seg_length = 0;

  double accumulated_length = 0;
  double last_accumulated_length = 0;
  for (size_t i = 0; i < line_string.size(); i++)
  {  // Iterate over line string to find nearest point
    double seg_length = 0;
    if (i < line_string.size() - 1)
    {
      seg_length = lanelet::geometry::distance2d(line_string[i], line_string[i + 1]);  // Compute segment length
    }

    double distance = lanelet::geometry::distance2d(p, line_string[i]);  // Compute from current point to external point
    if (distance < min_distance)
    {  // If this distance is below minimum discovered so far then update minimum
      min_distance = distance;
      best_point_index = i;
      best_accumulated_length = accumulated_length;
      best_last_accumulated_length = last_accumulated_length;  // Record accumulated lengths to each segment
      best_seg_length = seg_length;
      best_last_seg_length = last_seg_length;
    }

    last_accumulated_length = accumulated_length;  // Update accumulated lenths
    accumulated_length += seg_length;
    last_seg_length = seg_length;
  }

  // Minimum point has been found next step is to determine which segment it should go with using the following rules.
  // If the minimum point is the first point then use the first segment
  // If the minimum point is the last point then use the last segment
  // If the minimum point is within the downtrack bounds of one segment but not the other then use the one it is within
  // If the minimum point is within the downtrack bounds of both segments then use the one with the smallest crosstrack
  // distance If the minimum point is within the downtrack bounds of both segments and has exactly equal crosstrack
  // bounds with each segment then use the first one
  TrackPos best_pos(0, 0);
  if (best_point_index == 0)
  {
    best_pos = trackPos(p, line_string[0], line_string[1]);
    best_segment = std::make_pair(line_string[0], line_string[1]);
  }
  else if (best_point_index == line_string.size() - 1)
  {
    best_pos = trackPos(p, line_string[line_string.size() - 2], line_string[line_string.size() - 1]);
    best_pos.downtrack += best_last_accumulated_length;
    best_segment = std::make_pair(line_string[line_string.size() - 2], line_string[line_string.size() - 1]);
  }
  else
  {
    TrackPos first_seg_trackPos = trackPos(p, line_string[best_point_index - 1], line_string[best_point_index]);
    TrackPos second_seg_trackPos = trackPos(p, line_string[best_point_index], line_string[best_point_index + 1]);
    if (selectFirstSegment(first_seg_trackPos, second_seg_trackPos, best_last_seg_length, best_seg_length))
    {
      best_pos = first_seg_trackPos;
      best_pos.downtrack += best_last_accumulated_length;
      best_segment = std::make_pair(line_string[best_point_index - 1], line_string[best_point_index]);
    }
    else
    {
      best_pos = second_seg_trackPos;
      best_pos.downtrack += best_accumulated_length;
      best_segment = std::make_pair(line_string[best_point_index], line_string[best_point_index + 1]);
    }
  }

  // couldn't find a matching segment, so use the first segment within the downtrack range of.
  // Or the starting segment assuming we are before the route
  return std::make_tuple(best_pos, best_segment);
}

// NOTE: See Geometry.h header file for details on source of logic in this function
double computeCurvature(const lanelet::BasicPoint2d& p1, const lanelet::BasicPoint2d& p2,
                        const lanelet::BasicPoint2d& p3)
{
  auto dp = 0.5 * (p3 - p1);
  auto ddp = p3 - 2.0 * p2 + p1;
  auto denom = std::pow(dp.x() * dp.x() + dp.y() * dp.y(), 3.0 / 2.0);
  if (std::fabs(denom) < 1e-20)
  {
    denom = 1e-20;
  }
  return static_cast<double>((ddp.y() * dp.x() - dp.y() * ddp.x()) / denom);
}

std::vector<std::tuple<size_t, std::vector<double>>>
getLocalCurvatures(const std::vector<lanelet::ConstLanelet>& lanelets)
{
  std::vector<std::tuple<size_t, std::vector<double>>> vec;

  lanelet::BasicPoint2d prevEndPoint;
  lanelet::BasicPoint2d first_2d, second_2d, third_2d;

  for (size_t n = 0; n < lanelets.size(); n++)
  {
    lanelet::ConstLanelet ll = lanelets[n];
    auto mutableCenterLine = lanelet::utils::to2D(ll.centerline()).basicLineString();

    if (mutableCenterLine.empty())
    {
      throw std::invalid_argument("Provided lanelet contains no centerline");
    }

    for (size_t i = 0; i < mutableCenterLine.size() - 1; i++)
    {
      bool new_segment_needed = false;
      // If the first lanelet initialize the 3 points needed for computing curvature
      if (n == 0 && i <= 1)
      {
        new_segment_needed = true;
      }
      else if (n != 0 && i <= 1)
      {
        new_segment_needed = lanelet::geometry::distance2d(prevEndPoint, mutableCenterLine[i]) > 0.1;
      }

      if (new_segment_needed && i == 0)
      {
        std::vector<double> curvatures;
        vec.push_back(std::make_tuple(n, curvatures));
        continue;
      }
      else if (new_segment_needed && i == 1)
      {
        first_2d = mutableCenterLine[i - 1];
        second_2d = mutableCenterLine[i];
        third_2d = mutableCenterLine[i + 1];
        std::get<1>(vec.back()).push_back(computeCurvature(first_2d, second_2d, third_2d));
        continue;
      }

      first_2d = second_2d;
      second_2d = third_2d;
      third_2d = mutableCenterLine[i + 1];
      std::get<1>(vec.back()).push_back(computeCurvature(first_2d, second_2d, third_2d));
    }
    
    prevEndPoint = mutableCenterLine.back();
  }

  return vec;
}

lanelet::BasicPolygon2d objectToMapPolygon(const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& size)
{
  tf2::Transform object_tf;
  tf2::fromMsg(pose, object_tf);

  double half_x_bound = size.x / 2;
  double half_y_bound = size.y / 2;

  // 4 corners of the object starting with upper left and moving in clockwise direction in pose frame
  tf2::Vector3 obj_p1(half_x_bound, half_y_bound, 0);
  tf2::Vector3 obj_p2(half_x_bound, -half_y_bound, 0);
  tf2::Vector3 obj_p3(-half_x_bound, -half_y_bound, 0);
  tf2::Vector3 obj_p4(-half_x_bound, half_y_bound, 0);

  tf2::Vector3 obj_p1_map = object_tf * obj_p1;
  tf2::Vector3 obj_p2_map = object_tf * obj_p2;
  tf2::Vector3 obj_p3_map = object_tf * obj_p3;
  tf2::Vector3 obj_p4_map = object_tf * obj_p4;

  lanelet::BasicPoint2d p1(obj_p1_map.getX(), obj_p1_map.getY());
  lanelet::BasicPoint2d p2(obj_p2_map.getX(), obj_p2_map.getY());
  lanelet::BasicPoint2d p3(obj_p3_map.getX(), obj_p3_map.getY());
  lanelet::BasicPoint2d p4(obj_p4_map.getX(), obj_p4_map.getY());

  return { p1, p2, p3, p4 };
}

}  // namespace geometry

}  // namespace carma_wm