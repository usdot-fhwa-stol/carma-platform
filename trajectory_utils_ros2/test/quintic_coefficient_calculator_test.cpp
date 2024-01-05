/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include <gtest/gtest.h>
#include <trajectory_utils/quintic_coefficient_calculator.hpp>

TEST(trajectory_utils_test, TestGetCoefficient)
{
    double x0 = 0;
    double xt = 20;
    double v0 = 30;
    double vt = 15;
    double a0 = 0;
    double at = 0;

    __uint64_t t0 = 0;
    __uint64_t tt = 7;

    std::vector<double> vec = quintic_coefficient_calculator::quintic_coefficient_calculator(x0, xt, v0, vt, a0, at,t0,tt);
    std::vector<double> vec1{ -0.0490867, 0.880883, -4.31487, 0, 30, 0};

    for (size_t i = 0; i < vec.size(); i++) {
        ASSERT_NEAR(vec[i],vec1[i],  0.1);
    }

}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
} 