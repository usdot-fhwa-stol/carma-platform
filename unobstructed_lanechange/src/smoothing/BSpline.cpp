/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include <smoothing/BSpline.h>

namespace unobstructed_lanechange
{
    namespace smoothing
    {
        void BSpline::setPoints(std::vector<lanelet::BasicPoint2d> points)
        {
            Eigen::MatrixXf matrix_points(2, points.size());
            int row_index = 0;
            for(auto const point : points){
                matrix_points.col(row_index) << point.x(), point.y();
                row_index++;
            }
            spline_ = Eigen::SplineFitting<Spline2d>::Interpolate(matrix_points, 2);
        }

        lanelet::BasicPoint2d BSpline::operator()(double t) const
        {
            Eigen::VectorXf values = spline_(t);
            lanelet::BasicPoint2d pt = {(double)values.x(), (double)values.y()};
            return pt;
        }
    };  //namespace smoothing
};      //namespace unobstructed_lanechange_plugin