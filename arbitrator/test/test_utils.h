/**
 * Test fixture for TrajectoryExecutor testing
 * Maintains publishers, subscribers, and message tracking for all tests.
 * State is reset between tests to ensure clean results.
 */

#ifndef __TEST_UTILS_HPP__
#define __TEST_UTILS_HPP__

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "arbitrator_state_machine.hpp"
#include "fixed_priority_cost_function.hpp"
#include "beam_search_strategy.hpp"
#include "capabilities_interface.hpp"

class ArbitratorStateMachineTest : public ::testing::Test 
{
    public:
        ArbitratorStateMachineTest() {};
        arbitrator::ArbitratorStateMachine sm_;
};

class FixedPriorityCostFunctionTest : public ::testing::Test 
{
    public:
        FixedPriorityCostFunctionTest():
            fpcf{{
                {"plugin_a", 10.0}, 
                {"plugin_b", 5.0}, 
                {"plugin_c", 15.0}}} {};
        arbitrator::FixedPriorityCostFunction fpcf;
};

class BeamSearchStrategyTest : public ::testing::Test 
{
    public:
        BeamSearchStrategyTest() {};
        arbitrator::BeamSearchStrategy bss{3};
};

#endif //__TEST_UTILS_HPP__
