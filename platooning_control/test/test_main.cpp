#include "pid_controller.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>


// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
