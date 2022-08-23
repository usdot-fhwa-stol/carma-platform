/*
 * Copyright (C) 2022 LEIDOS.
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
#include <rclcpp/rclcpp.hpp>
#include <boost/optional.hpp>
#include "localization_manager/LocalizationTransitionTable.hpp"

namespace localization_manager
{
    /**
 * \brief Helper function to assert that the provided list of signals do not change the current state of the provided
 * transition table
 */
void testNonTransitionSignals(LocalizationTransitionTable& table, const std::vector<LocalizationSignal>& signals)
{
  LocalizationState initial_state = table.getState();
  for (LocalizationSignal signal : signals)
  {
    table.signal(signal);
    ASSERT_EQ(initial_state, table.getState());
  }
}

TEST(LocalizationTransitionTable, testGetState)
{
  LocalizationTransitionTable ltt(LocalizerMode::AUTO_WITH_TIMEOUT);
  ASSERT_EQ(LocalizationState::UNINITIALIZED, ltt.getState());

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());
}

TEST(LocalizationTransitionTable, testTransitionCallback)
{
  LocalizationTransitionTable ltt(LocalizerMode::AUTO_WITH_TIMEOUT);

  boost::optional<LocalizationState> prev, current;
  boost::optional<LocalizationSignal> sig;
  ltt.setTransitionCallback([&](LocalizationState prev_state, LocalizationState new_state, LocalizationSignal signal) {
    prev = prev_state;
    current = new_state;
    sig = signal;
  });

  ASSERT_FALSE(!!prev);
  ASSERT_FALSE(!!current);
  ASSERT_FALSE(!!sig);

  ASSERT_EQ(LocalizationState::UNINITIALIZED, ltt.getState());

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE,
                                    LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::TIMEOUT,
                                    LocalizationSignal::LIDAR_SENSOR_FAILURE,
                                });
  
  ASSERT_FALSE(!!prev);
  ASSERT_FALSE(!!current);
  ASSERT_FALSE(!!sig);

  ltt.signal(LocalizationSignal::INITIAL_POSE); // Trigger state transition
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  ASSERT_TRUE(!!prev);
  ASSERT_TRUE(!!current);
  ASSERT_TRUE(!!sig);

  ASSERT_EQ(LocalizationState::UNINITIALIZED, prev.get());
  ASSERT_EQ(LocalizationState::INITIALIZING, current.get());
  ASSERT_EQ(LocalizationSignal::INITIAL_POSE, sig.get());
}

TEST(LocalizationTransitionTable, testTransitionsGNSSMode)
{
  // Test GNSS Mode
  LocalizationTransitionTable ltt(LocalizerMode::GNSS);

  // Evaluate UNINITIALIZED state
  ASSERT_EQ(LocalizationState::UNINITIALIZED, ltt.getState());

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE,
                                    LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::TIMEOUT,
                                    LocalizationSignal::LIDAR_SENSOR_FAILURE,
                                });

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, ltt.getState());

  // Evaluate DEGRADED_NO_LIDAR_FIX state

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::TIMEOUT,
                                    LocalizationSignal::LIDAR_SENSOR_FAILURE,
                                    LocalizationSignal::INITIAL_POSE,
                                });
}


TEST(LocalizationTransitionTable, testTransitionsNDTMode)
{
  // Test NDT Mode
  LocalizationTransitionTable ltt(LocalizerMode::NDT);

  // Evaluate UNINITIALIZED State
  ASSERT_EQ(LocalizationState::UNINITIALIZED, ltt.getState());

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE,
                                    LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::TIMEOUT,
                                    LocalizationSignal::LIDAR_SENSOR_FAILURE,
                                });

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::INITIAL_POSE,  
                                });

  ltt.signal(LocalizationSignal::TIMEOUT);
  ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, ltt.getState());

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE,
                                    LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::TIMEOUT,
                                    LocalizationSignal::LIDAR_SENSOR_FAILURE,
                                });

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  // Evaluate DEGRADED state
  ltt.signal(LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::DEGRADED, ltt.getState());

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::TIMEOUT,
                                });

  ltt.signal(LocalizationSignal::LIDAR_SENSOR_FAILURE);
  ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, ltt.getState());

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  ltt.signal(LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::DEGRADED, ltt.getState());

  ltt.signal(LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, ltt.getState());

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  ltt.signal(LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::DEGRADED, ltt.getState());

  // Evaluate OPERATIONAL State
  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  ltt.signal(LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::OPERATIONAL, ltt.getState());

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::TIMEOUT,
                                });

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  ltt.signal(LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::OPERATIONAL, ltt.getState());

  ltt.signal(LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::DEGRADED, ltt.getState());

  ltt.signal(LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::OPERATIONAL, ltt.getState());

  ltt.signal(LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, ltt.getState());

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  ltt.signal(LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::OPERATIONAL, ltt.getState());

  ltt.signal(LocalizationSignal::LIDAR_SENSOR_FAILURE);
  ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, ltt.getState());

  // Lidar sensor failure during initialization
  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  ltt.signal(LocalizationSignal::LIDAR_SENSOR_FAILURE);
  ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, ltt.getState());
}

TEST(LocalizationTransitionTable, testTransitionsAUTOMode)
{
  // Test NDT Mode
  LocalizationTransitionTable ltt(LocalizerMode::AUTO_WITH_TIMEOUT);

  // Evaluate UNINITIALIZED State
  ASSERT_EQ(LocalizationState::UNINITIALIZED, ltt.getState());

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE,
                                    LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::TIMEOUT,
                                    LocalizationSignal::LIDAR_SENSOR_FAILURE,
                                });

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::INITIAL_POSE,  
                                });

  ltt.signal(LocalizationSignal::TIMEOUT);
  ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, ltt.getState());

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE,
                                    LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::TIMEOUT,
                                    LocalizationSignal::LIDAR_SENSOR_FAILURE,
                                });

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  // Evaluate DEGRADED state
  ltt.signal(LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::DEGRADED, ltt.getState());

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE,
                                    LocalizationSignal::TIMEOUT,
                                });

  ltt.signal(LocalizationSignal::LIDAR_SENSOR_FAILURE);
  ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, ltt.getState());

  testNonTransitionSignals(ltt, {
                                  LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE,
                                  LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE,
                                  LocalizationSignal::LIDAR_SENSOR_FAILURE,
                              });

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  ltt.signal(LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::DEGRADED, ltt.getState());

  ltt.signal(LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, ltt.getState());

  // Evaluate OPERATIONAL State
  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  ltt.signal(LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::OPERATIONAL, ltt.getState());

  testNonTransitionSignals(ltt, {
                                    LocalizationSignal::TIMEOUT,
                                });

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  ltt.signal(LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::OPERATIONAL, ltt.getState());

  ltt.signal(LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::DEGRADED, ltt.getState());

  ltt.signal(LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::OPERATIONAL, ltt.getState());

  ltt.signal(LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, ltt.getState());

  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  ltt.signal(LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE);
  ASSERT_EQ(LocalizationState::OPERATIONAL, ltt.getState());

  ltt.signal(LocalizationSignal::LIDAR_SENSOR_FAILURE);
  ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, ltt.getState());

  // Evaluate DEGRADED_NO_LIDAR_FIX
  ltt.signal(LocalizationSignal::TIMEOUT);
  ASSERT_EQ(LocalizationState::AWAIT_MANUAL_INITIALIZATION, ltt.getState());

  // Lidar sensor failure during initialization
  ltt.signal(LocalizationSignal::INITIAL_POSE);
  ASSERT_EQ(LocalizationState::INITIALIZING, ltt.getState());

  ltt.signal(LocalizationSignal::LIDAR_SENSOR_FAILURE);
  ASSERT_EQ(LocalizationState::DEGRADED_NO_LIDAR_FIX, ltt.getState());
}

}
