/**
 * Test fixture for TrajectoryExecutor testing
 * Maintains publishers, subscribers, and message tracking for all tests.
 * State is reset between tests to ensure clean results.
 */

#ifndef __TEST_UTILS_HPP__
#define __TEST_UTILS_HPP__

#include <gtest/gtest.h>
#include "arbitrator_state_machine.hpp"

class ArbitratorStateMachineTest : public ::testing::Test 
{
    public:
        ArbitratorStateMachineTest() {};
        arbitrator::ArbitratorStateMachine sm_;
};

#endif //__TEST_UTILS_HPP__
