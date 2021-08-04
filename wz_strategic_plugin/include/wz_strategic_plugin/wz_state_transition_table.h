#pragma once
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
#include "wz_states.h"
#include <functional>

namespace wz_strategic_plugin
{
/**
 * \brief Class defining the state transition table behavior for the WorkZone Strategic Plugin
 */
class WorkZoneStateTransitionTable
{
public:
  using TransitionCallback = std::function<void(TransitState prev_state, TransitState new_state, TransitEvent signal)>;

  /**
   * \brief Default Constructor
   */
  WorkZoneStateTransitionTable() = default;

  /**
   * \brief Returns the current state
   * \return Current state
   */
  TransitState getState() const;

  /**
   * \brief Trigger signal for the transition table.
   *
   * \param signal The signal for the transition table to evaluate
   */
  void signal(TransitEvent signal);

  /**
   * \brief Callback setting function. The provided callback will be triggered any time the current state changes to a
   * new state.
   *
   * \param cb The callback function which will be provided with the previous state, new current state, and the signal
   * which caused the transition.
   */
  void setTransitionCallback(TransitionCallback cb);

private:
  //! Current state. This state should only ever be set using the setAndLogState() function.
  TransitState state_ = TransitState::UNAVAILABLE;

  TransitionCallback transition_callback_;

  // Helper functions for processing each provided signal based on the current state
  void signalWhenUNAVAILABLE(TransitEvent signal);
  void signalWhenAPPROACHING(TransitEvent signal);
  void signalWhenWAITING(TransitEvent signal);
  void signalWhenDEPARTING(TransitEvent signal);

  /**
   * \brief Helper function for logging the provide signal
   * \param signal The signal to be logged
   */
  void logDebugSignal(TransitEvent signal) const;

  /**
   * \brief Function to change the current state and log the details of the transition.
   *
   * \param new_state The state to set.
   * \param source_signal The signal which caused the new_state to be set
   */
  void setAndLogState(TransitState new_state, TransitEvent source_signal);
};
}  // namespace wz_strategic_plugin