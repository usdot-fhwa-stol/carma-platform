#include "pid_controller.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>


TEST(PIDControllerTest, test1)
{
    platoon_control::PIDController pid;
    double res = pid.calculate(40, 38);
    EXPECT_EQ(-8, res);

    
}
