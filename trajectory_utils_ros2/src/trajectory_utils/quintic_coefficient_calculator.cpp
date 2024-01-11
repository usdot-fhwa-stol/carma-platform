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

#include <string>
#include <vector>
#include <tuple>
#include <math.h>
#include <exception>
#include <trajectory_utils/quintic_coefficient_calculator.hpp>

namespace quintic_coefficient_calculator {

    std::vector<double> quintic_coefficient_calculator(double x0, double xt, double v0, double vt, double a0, double at, __uint64_t t0, __uint64_t tt ) {

        Eigen::VectorXd state_values(6);
        state_values << x0, xt, v0, vt, a0, at;

        Eigen::Matrix<double, 6, 6> mat;

        mat(0,0) = pow(t0, 5);
        mat(0,1) = pow(t0, 4);
        mat(0,2) = pow(t0, 3); 
        mat(0,3) = pow(t0, 2);
        mat(0,4) = t0;
        mat(0,5) = 1;

        mat(1,0) = pow(tt, 5);
        mat(1,1) = pow(tt, 4);
        mat(1,2) = pow(tt, 3); 
        mat(1,3) = pow(tt, 2);
        mat(1,4) = tt;
        mat(1,5) = 1;

        mat(2,0) = 5 * pow(t0, 4);
        mat(2,1) = 4 * pow(t0, 3);
        mat(2,2) = 3 * pow(t0, 2);
        mat(2,3) = 2 * t0;
        mat(2,4) = 1;
        mat(2,5) = 0;

        mat(3,0) = 5 * pow(tt, 4);
        mat(3,1) = 4 * pow(tt, 3);
        mat(3,2) = 3 * pow(tt, 2);
        mat(3,3) = 2 * tt;
        mat(3,4) = 1;
        mat(3,5) = 0;

        mat(4,0) = 20 * pow(t0, 3);
        mat(4,1) = 12 * pow(t0, 2);
        mat(4,2) = 6 * t0;
        mat(4,3) = 2;
        mat(4,4) = 0;
        mat(4,5) = 0;

        mat(5,0) = 20 * pow(tt, 3);
        mat(5,1) = 12 * pow(tt, 2);
        mat(5,2) = 6 * tt;
        mat(5,3) = 2;
        mat(5,4) = 0;
        mat(5,5) = 0;

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mat_ = mat.inverse();
        Eigen::VectorXd result;

        result = mat_*state_values;

        std::vector<double> vector_result;
        for (size_t i = 0; i < result.size(); i++) {
            vector_result.push_back(result[i]);
        }

        return vector_result;
    }

}   // namespace quintic_coefficient_calculator