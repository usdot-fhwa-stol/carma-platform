
/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
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

------------------------------------------------------------------------------*/

#include "pid_controller.h"
#include <gtest/gtest.h>
#include <ros/ros.h>


TEST(PIDControllerTest, test1) //unlimited integral & outputs with no i & d terms
{
    platoon_control_pid0::PIDController pid(4.0, 10.0, 1.0, 2.0, 0.0, 0.0, 0.5, -20.0, 20.0, -50.0, 50.0);
    double res;
    res = pid.calculate(40.0, 28.0); //exercise k2 with positive error
    EXPECT_EQ(10.0, res);
    res = pid.calculate(40.0, 33.0); //exercise k1 with positive error
    EXPECT_EQ(3.0, res);
    res = pid.calculate(40.0, 39.1); //exercise deadband with positive error
    EXPECT_EQ(0.0, res);
    res = pid.calculate(40.0, 44.0); //exercise deadband with negative error
    EXPECT_EQ(0.0, res);
    res = pid.calculate(40.0, 46.6); //exercise k1 with negative error
    EXPECT_EQ(-2.6, res);
    res = pid.calculate(40.0, 55.9); //exercise k2 with negative error
    EXPECT_EQ(-17.8, res);
}

TEST(PIDControllerTest, test2) //testing output limiter
{
    platoon_control_pid0::PIDController pid(4.0, 10.0, 1.0, 2.0, 0.0, 0.0, 0.5, -20.0, 20.0, -20.0, 20.0);
    double res;
    res = pid.calculate(40.0, 55.9); //negative not limited
    EXPECT_EQ(-17.8, res);
    res = pid.calculate(40.0, 80.0); //negative limited
    EXPECT_EQ(-20.0, res);
    res = pid.calculate(40.0, 26.2); //positive not limited
    EXPECT_EQ(13.6, res);
    res = pid.calculate(40.0, 22.9); //positive limited
    EXPECT_EQ(20.0, res);
}

TEST(PIDControllerTest, test3) //testing integral term & limiters
{
    platoon_control_pid0::PIDController pid(4.0, 10.0, 1.0, 2.0, 0.1, 0.0, 0.5, -5.0, 12.0, -50.0, 50.0);
    double res;
    res = pid.calculate(40.0, 20.0); //kp2 positive error, integral no history
    EXPECT_EQ(27.0, res);
    res = pid.calculate(40.0, 31.0); //kp1 positive error, integral positive limited
    EXPECT_EQ(6.2, res);
    res = pid.calculate(40.0, 39.0); //deadband, integral limited
    EXPECT_EQ(1.2, res);
    res = pid.calculate(40.0, 43.0); //deadband
    EXPECT_EQ(1.05, res);
    res = pid.calculate(40.0, 45.0); //kp1 negative error
    EXPECT_EQ(-0.2, res);
    res = pid.calculate(40.0, 50.0); //kp1 negative error
    EXPECT_EQ(-5.7, res);
    res = pid.calculate(40.0, 60.0); //kp2 negative error
    EXPECT_EQ(-26.2, res);
    res = pid.calculate(40.0, 60.0); //kp2 negatvie error, integral negative limited
    EXPECT_EQ(-26.5, res);
}

TEST(PIDControllerTest, test4) //testing derivative term
{
    platoon_control_pid0::PIDController pid(4.0, 10.0, 1.0, 2.0, 0.0, 0.2, 0.5, -50.0, 50.0, -50.0, 50.0);
    double res;
    res = pid.calculate(40.0, 45.0); //kp1 negative error, prev error initialized to zero
    EXPECT_EQ(-3.0, res);
    res = pid.calculate(40.0, 48.0); //kp1 negative error
    EXPECT_EQ(-5.2, res);
    res = pid.calculate(40.0, 48.0); //kp1 negative error, zero derivative
    EXPECT_EQ(-4.0, res);
    res = pid.calculate(40.0, 42.0); //deadband, positive derivative
    EXPECT_EQ(2.4, res);
}

TEST(PIDControllerTest, test5) //both derivative & integral terms active
{
    platoon_control_pid0::PIDController pid(4.0, 10.0, 1.0, 2.0, 0.1, 0.2, 0.5, -50.0, 50.0, -50.0, 50.0);
    double res;
    res = pid.calculate(40.0, 45.0); //kp1 negative error, integral negative, deriv negative
    EXPECT_EQ(-3.25, res);
    res = pid.calculate(40.0, 39.0); //deadband, integral still negative, deriv positive
    EXPECT_EQ(2.2, res);
    res = pid.calculate(40.0, 32.0); //kp1 positive error, integral positive, deriv positive
    EXPECT_EQ(7.0, res);
}
