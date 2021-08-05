/*
 * Copyright (C) 2021 LEIDOS.
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
#include <ros/console.h>
#include "test_fixture.h"
#include "wz_strategic_plugin/wz_state_transition_table.h"

// Unit tests for transition table
namespace wz_strategic_plugin
{
/**
 * \brief Helper function to assert that the provided list of signals do not change the current state of the provided
 * transition table
 */
void testNonTransitionSignals(WorkZoneStateTransitionTable& table, const std::vector<TransitEvent>& signals)
{
  TransitState initial_state = table.getState();
  for (TransitEvent signal : signals)
  {
    table.signal(signal);
    ASSERT_EQ(initial_state, table.getState());
  }
}

TEST_F(WorkZoneTestFixture, getState)
{
  WorkZoneStateTransitionTable table;
  ASSERT_EQ(TransitState::UNAVAILABLE, table.getState());

  table.signal(TransitEvent::IN_STOPPING_RANGE);
  ASSERT_EQ(TransitState::APPROACHING, table.getState());
}

// Unit test to evaluate all state transitions
TEST_F(WorkZoneTestFixture, signal)
{
  WorkZoneStateTransitionTable table;
  ASSERT_EQ(TransitState::UNAVAILABLE, table.getState());

  testNonTransitionSignals(table, { TransitEvent::STOPPED, TransitEvent::RED_TO_GREEN_LIGHT,
                                    TransitEvent::CROSSED_STOP_BAR, TransitEvent::INTERSECTION_EXIT });

  table.signal(TransitEvent::IN_STOPPING_RANGE);
  ASSERT_EQ(TransitState::APPROACHING, table.getState());

  testNonTransitionSignals(
      table, { TransitEvent::IN_STOPPING_RANGE, TransitEvent::RED_TO_GREEN_LIGHT, TransitEvent::INTERSECTION_EXIT });

  table.signal(TransitEvent::STOPPED);

  ASSERT_EQ(TransitState::WAITING, table.getState());

  testNonTransitionSignals(table, { TransitEvent::IN_STOPPING_RANGE, TransitEvent::STOPPED,
                                    TransitEvent::CROSSED_STOP_BAR, TransitEvent::INTERSECTION_EXIT });

  table.signal(TransitEvent::RED_TO_GREEN_LIGHT);

  ASSERT_EQ(TransitState::DEPARTING, table.getState());

  testNonTransitionSignals(table, { TransitEvent::IN_STOPPING_RANGE, TransitEvent::STOPPED,
                                    TransitEvent::RED_TO_GREEN_LIGHT, TransitEvent::CROSSED_STOP_BAR });

  table.signal(TransitEvent::INTERSECTION_EXIT);

  ASSERT_EQ(TransitState::UNAVAILABLE, table.getState());

  // Reset table to test non-stopping case
  table = WorkZoneStateTransitionTable();

  table.signal(TransitEvent::IN_STOPPING_RANGE);

  ASSERT_EQ(TransitState::APPROACHING, table.getState());

  table.signal(TransitEvent::CROSSED_STOP_BAR);

  ASSERT_EQ(TransitState::DEPARTING, table.getState());
}

TEST_F(WorkZoneTestFixture, setTransitionCallback)
{
  WorkZoneStateTransitionTable table;

  boost::optional<TransitState> prev, current;
  boost::optional<TransitEvent> sig;
  table.setTransitionCallback([&](TransitState prev_state, TransitState new_state, TransitEvent signal) {
    prev = prev_state;
    current = new_state;
    sig = signal;
  });

  ASSERT_FALSE(!!prev);
  ASSERT_FALSE(!!current);
  ASSERT_FALSE(!!sig);

  ASSERT_EQ(TransitState::UNAVAILABLE, table.getState());

  table.signal(TransitEvent::IN_STOPPING_RANGE);

  ASSERT_EQ(TransitState::APPROACHING, table.getState());
  ASSERT_TRUE(!!prev);
  ASSERT_TRUE(!!current);
  ASSERT_TRUE(!!sig);

  ASSERT_EQ(TransitState::UNAVAILABLE, prev.get());
  ASSERT_EQ(TransitState::APPROACHING, current.get());
  ASSERT_EQ(TransitEvent::IN_STOPPING_RANGE, sig.get());
}
}  // namespace wz_strategic_plugin