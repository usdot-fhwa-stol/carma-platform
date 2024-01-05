#pragma once
/*
 * Copyright (C) 2018-2022 LEIDOS.
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

#include <stdio.h>
#include <vector>
#include <tuple>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

namespace quintic_coefficient_calculator {
    
    /*! 
    \brief  quintic_coefficient_calculator is used to solve quintic_coefficient for object avoidance .
    \param x0 position at time t0
    \param xt position at time t
    \param v0 velocity at time t0
    \param vt velocity at time t
    \param a0 acceleration at time t0
    \param at acceleration at time t
    \param t0 initial time
    \param tt goal time
    */

    std::vector<double> quintic_coefficient_calculator(double x0, double xt, double v0, double vt, double a0, double at, __uint64_t t0, __uint64_t tt);

}  // namespace quintic_coefficient_calculator