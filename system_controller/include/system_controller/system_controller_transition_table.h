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
#include "system_controller_states.h"
#include <functional>

namespace system_controller
{
/**
 * \brief Class defining the state transition table behavior for the SystemController Strategic Plugin
 */
class SystemControllerStateTransitionTable
{
public:
  using TransitionCallback = std::function<void(SystemState prev_state, SystemState new_state, SystemEvent signal)>;

  /**
   * \brief Default Constructor
   */
  SystemControllerStateTransitionTable() = default;

  /**
   * \brief Returns the current state
   * \return Current state
   */
  SystemState getState() const;

  /**
   * \brief Trigger signal for the transition table.
   *
   * \param signal The signal for the transition table to evaluate
   */
  void signal(SystemEvent signal);

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
  SystemState state_ = SystemState::STARTING_UP;

  TransitionCallback transition_callback_;

  // Helper functions for processing each provided signal based on the current state
  void signalWhenSTARTINGUP(SystemEvent signal);
  void signalWhenINACTIVE(SystemEvent signal);
  void signalWhenACTIVE(SystemEvent signal);
  void signalWhenFINALIZED(SystemEvent signal);

  /**
   * \brief Helper function for logging the provide signal
   * \param signal The signal to be logged
   */
  void logDebugSignal(SystemEvent signal) const;

  /**
   * \brief Function to change the current state and log the details of the transition.
   *
   * \param new_state The state to set.
   * \param source_signal The signal which caused the new_state to be set
   */
  void setAndLogState(SystemState new_state, SystemEvent source_signal);
};
}  // namespace wz_strategic_plugin