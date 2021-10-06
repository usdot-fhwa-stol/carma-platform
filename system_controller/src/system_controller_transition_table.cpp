
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
#include <ros/console.h>
#include "system_controller/system_controller_transition_table.h"

namespace system_controller
{


/*

enum class SystemState
{
    STARTING_UP,
    INACTIVE,
    ACTIVE,
    FINALIZED
};

std::ostream& operator<<(std::ostream& os, SystemState s);


//! @brief Enum describing the possible signals to change the current SystemState
enum class SystemEvent
{
  STARTUP_DELAY_EXCEEDED,
  INTERNAL_FAULT,
  SUBSYSTEM_FAULT,
  EXTERNAL_SHUTDOWN
};

*/

SystemState SystemControllerStateTransitionTable::getState() const
{
  return state_;
}

void SystemControllerStateTransitionTable::signal(SystemEvent signal)
{
  switch (state_)
  {
    case SystemState::STARTING_UP:
      signalWhenSTARTINGUP(signal);
      break;

    case SystemState::ACTIVE:
      signalWhenACTIVE(signal);
      break;

    case SystemState::ERROR_PROCESSING:
      signalWhenERROR_PROCESSING(signal);
      break;

    case SystemState::SHUTTING_DOWN:
      signalWhenSHUTTING_DOWN(signal);
      break;

    case SystemState::FINALIZED:
      signalWhenFINALIZED(signal);
      break;

    default:
      throw std::invalid_argument("Transition table in unsupported state");
  }
}

void SystemControllerStateTransitionTable::setTransitionCallback(TransitionCallback cb)
{
  transition_callback_ = cb;
}

void SystemControllerStateTransitionTable::signalWhenSTARTINGUP(SystemEvent signal)
{
  if (signal == SystemEvent::STARTUP_DELAY_EXCEEDED)
  {
    setAndLogState(SystemState::ACTIVE, signal);
  }
  else
  {
    logDebugSignal(signal);
  }
}

void SystemControllerStateTransitionTable::signalWhenACTIVE(SystemEvent signal)
{
  if ( signal == SystemEvent::INTERNAL_FAULT || signal == SystemEvent::SUBSYSTEM_FAULT )
  {
    setAndLogState(SystemState::ERROR_PROCESSING, signal);
  }
  else if(signal == SystemEvent::EXTERNAL_SHUTDOWN) {
    setAndLogState(SystemState::SHUTTING_DOWN, signal);
  }
  else
  {
    logDebugSignal(signal);
  }
}

void SystemControllerStateTransitionTable::signalWhenERROR_PROCESSING(SystemEvent signal)
{
  if ( signal == SystemEvent::INTERNAL_FAULT || signal == SystemEvent::SUBSYSTEM_FAULT )
  {
    setAndLogState(SystemState::FINALIZED, signal);
  }
  else
  {
    logDebugSignal(signal);
  }
}

void SystemControllerStateTransitionTable::signalWhenSHUTTING_DOWN(SystemEvent signal)
{
  if ( signal == SystemEvent::SHUTDOWN_COMPLETED || signal == SystemEvent::SHUTDOWN_ERROR )
  {
    setAndLogState(SystemState::FINALIZED, signal);
  }
  else
  {
    logDebugSignal(signal);
  }
}

void SystemControllerStateTransitionTable::signalWhenFINALIZED(SystemEvent signal)
{
  logDebugSignal(signal);
}

void SystemControllerStateTransitionTable::logDebugSignal(SystemEvent signal) const
{
  ROS_DEBUG_STREAM("SystemControllerStateTransitionTable received unsupported signal of " << signal << " while in state "
                                                                                  << state_);
}

void SystemControllerStateTransitionTable::setAndLogState(SystemState new_state, SystemEvent source_signal)
{
  if (new_state == state_)
  {
    return;  // State was unchanged no need to log or trigger callbacks
  }

  ROS_INFO_STREAM("SystemControllerStateTransitionTable changed SystemController Strategic Plugin state from "
                  << state_ << " to " << new_state << " because of signal " << source_signal);
 
  SystemState prev_state = state_;
  state_ = new_state; // Set new state
  
  if (transition_callback_) // Trigger callback if available
  {
    transition_callback_(prev_state, state_, source_signal);
  }
}

}  // namespace system_controller