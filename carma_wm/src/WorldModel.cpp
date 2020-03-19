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

namespace carma_wm
{
std::vector<double>
WorldModel::getLocalCurvatures(const std::vector<lanelet::ConstLanelet>& lanelets)
{
  if (lanelets.empty()) {
    throw std::invalid_argument("Attempted to call getLocalCurvatures on empty lanelet list");
  }

  lanelet::BasicLineString2d centerline_points = concatenate_lanelets(lanelets);

  if (centerline_points.empty()) {
    throw std::invalid_argument("No points in lanelet centerline for curvature calculation");
  }

  std::vector<Eigen::Vector2d> spatial_derivative = compute_finite_differences(centerline_points);
  std::vector<Eigen::Vector2d> normalized_tangent_vectors = normalize_vectors(spatial_derivative);

  std::vector<double> arc_lengths = compute_arc_lengths(centerline_points);

  std::vector<Eigen::Vector2d> tangent_derivative = 
    compute_finite_differences(
      normalized_tangent_vectors, 
      arc_lengths);

  std::vector<double> curvature = compute_magnitude_of_vectors(tangent_derivative);

  return curvature;
}

lanelet::BasicLineString2d WorldModel::concatenate_lanelets(const std::vector<lanelet::ConstLanelet>& lanelets)
{
  lanelet::BasicLineString2d centerline_points = lanelets[0].centerline2d().basicLineString();
  lanelet::BasicLineString2d new_points;
  for (int i = 1; i < lanelets.size(); i++) {
    new_points = lanelets[i].centerline2d().basicLineString();
    centerline_points = concatenate_line_strings(centerline_points, new_points);
  }

  return centerline_points;
}

lanelet::BasicLineString2d 
WorldModel::concatenate_line_strings(const lanelet::BasicLineString2d& a, 
                                          const lanelet::BasicLineString2d& b)
{
  lanelet::BasicLineString2d out;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());

  return out;
}

std::vector<Eigen::Vector2d> 
WorldModel::compute_finite_differences(const lanelet::BasicLineString2d& data)
{
  std::vector<Eigen::Vector2d> out;
  Eigen::Vector2d diff;
  Eigen::Vector2d vec_i_plus_1;
  Eigen::Vector2d vec_i_minus_1;

  for (int i = 0; i < data.size(); i++) {
    if (i == 0) {
      // Compute forward derivative
      diff = (data[i + 1] - data[i]);
    } else if (i == data.size() - 1) {
      // Compute backward derivative
      diff = (data[i] - data[i - 1]);
    } else {
      // Else, compute centered derivative
      vec_i_plus_1 = data[i + 1];
      vec_i_minus_1 = data[i - 1];
      diff = (vec_i_plus_1 - vec_i_minus_1)/2.0;
    }

    out.push_back(diff);
  }

  return out;
}

std::vector<double> 
WorldModel::compute_finite_differences(const std::vector<double>& data)
{
  std::vector<double> out;
  double diff;
  for (int i = 0; i < data.size(); i++) {

    if (i == 0) {
      diff = data[i + 1] - data[i];
    } else if (i == data.size() - 1) {
      diff = data[i] - data[i - 1];
    } else {
      diff = (data[i + 1] - data[i - 1])/2.0;
    }
    
    out.push_back(diff);
  }

  return out;
}

std::vector<Eigen::Vector2d> 
WorldModel::compute_finite_differences(const std::vector<Eigen::Vector2d>& x,const std::vector<double>& y)
{
  if (x.size() != y.size()) {
    throw std::invalid_argument("Attempting to differentiate two unequal sized lists!");
  }

  std::vector<Eigen::Vector2d> out;
  Eigen::Vector2d diff;
  for (int i = 0; i < x.size(); i++) {
    if (i == 0) {
      diff = (x[i + 1] - x[i])/(y[i + 1] - y[i]);
    } else if (i == x.size() - 1) {
      diff = (x[i] - x[i - 1])/(y[i] - y[i - 1]);
    } else {
      diff = (x[i + 1] - x[i - 1])/(y[i + 1] - y[i - 1]);
    }
    
    out.push_back(diff);
  }

  return out;
}

std::vector<double>
WorldModel::compute_arc_lengths(const lanelet::BasicLineString2d& data)
{
  std::vector<double> out;
  double total = 0;
  double diff = 0;
  for (int i =  0; i < data.size(); i++) {
    if (i == 0) {
      out.push_back(0);
    } else {
      diff = compute_euclidean_distance(data[i - 1], data[i]);
      total += diff;
      out.push_back(total);
    }
  }

  return out;
}

double 
WorldModel::compute_euclidean_distance(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
  return std::sqrt(std::pow(b[0] - a[0], 2) + std::pow(b[1] - a[1], 2));
  //return (b - a).norm();
}

std::vector<Eigen::Vector2d>
WorldModel::normalize_vectors(const std::vector<Eigen::Vector2d>& vectors)
{
  std::vector<Eigen::Vector2d> out;
  for (auto vec : vectors) {
    out.push_back(vec.normalized());
  }
  return out;
}

std::vector<double> 
WorldModel::compute_magnitude_of_vectors(const std::vector<Eigen::Vector2d>& vectors)
{
  std::vector<double> out;
  for (auto vec : vectors) {
    out.push_back(vec.norm());
  }

  return out;
}
};