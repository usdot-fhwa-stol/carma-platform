#pragma once

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

#include <vector>
#include <carma_wm/Geometry.h>
#include "smoothing/SplineI.h"
#include <unsupported/Eigen/Splines>

namespace cooperative_lanechange
{
namespace smoothing
{

    typedef Eigen::Spline<double, 2> Spline2d;
/**
 * \brief Realization of SplineI that uses the Eigen::Splines library for interpolation 
 */ 
class BSpline : public SplineI
{
public:
 ~BSpline(){};
  void setPoints(std::vector<lanelet::BasicPoint2d> points) override;
  lanelet::BasicPoint2d operator()(double t) const override;
  lanelet::BasicPoint2d first_deriv(double t) const override;
  lanelet::BasicPoint2d second_deriv(double t) const override;

private:
  Spline2d spline_;
};
};  // namespace smoothing
};  // namespace cooperative_lanechange