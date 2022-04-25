
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


#include <basic_autonomy/smoothing/BSpline.h>

namespace basic_autonomy
{
namespace smoothing
{
void BSpline::setPoints(std::vector<lanelet::BasicPoint2d> points)
{
  Eigen::MatrixXd matrix_points(2, points.size());
  int row_index = 0;
  for(auto const point : points){
      matrix_points.col(row_index) << point.x(), point.y();
      row_index++;
  }
  spline_ = Eigen::SplineFitting<Spline2d>::Interpolate(matrix_points,3 );
}
lanelet::BasicPoint2d BSpline::operator()(double t) const
{
  Eigen::VectorXd values = spline_(t);
  lanelet::BasicPoint2d pt = {values.x(), values.y()};
  return pt;
}

lanelet::BasicPoint2d  BSpline::first_deriv(double t) const {
  Eigen::Array2Xd v = spline_.derivatives(t, 1);
  lanelet::BasicPoint2d  output = {v(2), v(3)};
  return output;
}

lanelet::BasicPoint2d  BSpline::second_deriv(double t) const {
  Eigen::Array2Xd v = spline_.derivatives(t, 2);
  lanelet::BasicPoint2d  output = {v(4), v(5)};
  return output;
}

}  // namespace smoothing
}  // namespace basic_autonomy