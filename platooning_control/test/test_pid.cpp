/*
 * Copyright (C) 2024 LEIDOS.
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
#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include "platooning_control/platooning_control.hpp"


TEST(PIDControllerTest, test1)
{
    platooning_control::PIDController pid;
    double res = pid.calculate(40, 38);
    EXPECT_EQ(-9, res);;
}

TEST(PIDControllerTest, test2)
{
    platooning_control::PIDController pid;
    double res = pid.calculate(20, 300);
    EXPECT_EQ(2, res);
}

TEST(PIDControllerTest, test3)
{
    platooning_control::PIDController pid;
    double res = pid.calculate(300, 20);
    EXPECT_EQ(-10, res);
}

TEST(PIDControllerTest, test4)
{
    platooning_control::PIDController pid;
    pid.reset();
    double res = pid.calculate(200, 20);
    EXPECT_EQ(-10, res);
    double res2 = pid.calculate(500,25);
    EXPECT_EQ(-10, res2);
    double res3 = pid.calculate(25,500);
    EXPECT_EQ(2, res3);
}