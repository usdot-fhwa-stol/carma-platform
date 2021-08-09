
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
#include "wz_strategic_plugin/wz_state_transition_table.h"

namespace wz_strategic_plugin
{
  // TODO clean up
// THERE SHOULD BE FOUR OPERATIONAL MODES FOR THIS PLUGIN
// UNAVAILABLE { On In Intersection Range Event -> APPROACH }
// APPROACHING { On Stopped Event -> WAITING, On Crossed Stop Bar -> DEPARTURE}
// WAITING { On Green Light Event -> DEPARTURE}
// DEPARTING { On exit intersection -> UNAVAILABLE} // TODO remove

TransitState WorkZoneStateTransitionTable::getState() const
{
  return state_;
}

void WorkZoneStateTransitionTable::signal(TransitEvent signal)
{
  switch (state_)
  {
    case TransitState::UNAVAILABLE:
      signalWhenUNAVAILABLE(signal);
      break;

    case TransitState::APPROACHING:
      signalWhenAPPROACHING(signal);
      break;

    case TransitState::WAITING:
      signalWhenWAITING(signal);
      break;

    case TransitState::DEPARTING:
      signalWhenDEPARTING(signal);
      break;

    default:
      throw std::invalid_argument("Transition table in unsupported state");
  }
}

void WorkZoneStateTransitionTable::setTransitionCallback(TransitionCallback cb)
{
  transition_callback_ = cb;
}

void WorkZoneStateTransitionTable::signalWhenUNAVAILABLE(TransitEvent signal)
{
  if (signal == TransitEvent::IN_STOPPING_RANGE)
  {
    setAndLogState(TransitState::APPROACHING, signal);
  }
  else
  {
    logDebugSignal(signal);
  }
}

void WorkZoneStateTransitionTable::signalWhenAPPROACHING(TransitEvent signal)
{
  switch (signal)
  {
    case TransitEvent::STOPPED:
      setAndLogState(TransitState::WAITING, signal);
      break;

    case TransitEvent::CROSSED_STOP_BAR:
      setAndLogState(TransitState::DEPARTING, signal);
      break;

    default:
      logDebugSignal(signal);
      break;
  }
}

void WorkZoneStateTransitionTable::signalWhenWAITING(TransitEvent signal)
{
  if (signal == TransitEvent::RED_TO_GREEN_LIGHT)
  {
    setAndLogState(TransitState::DEPARTING, signal);
  }
  else
  {
    logDebugSignal(signal);
  }
}

void WorkZoneStateTransitionTable::signalWhenDEPARTING(TransitEvent signal)
{
  if (signal == TransitEvent::INTERSECTION_EXIT)
  {
    setAndLogState(TransitState::UNAVAILABLE, signal);
  }
  else
  {
    logDebugSignal(signal);
  }
}

void WorkZoneStateTransitionTable::logDebugSignal(TransitEvent signal) const
{
  ROS_DEBUG_STREAM("WorkZoneStateTransitionTable received unsupported signal of " << signal << " while in state "
                                                                                  << state_);
}

void WorkZoneStateTransitionTable::setAndLogState(TransitState new_state, TransitEvent source_signal)
{
  if (new_state == state_)
  {
    return;  // State was unchanged no need to log or trigger callbacks
  }

  ROS_INFO_STREAM("WorkZoneStateTransitionTable changed WorkZone Strategic Plugin state from "
                  << state_ << " to " << new_state << " because of signal " << source_signal);
 
  TransitState prev_state = state_;
  state_ = new_state; // Set new state
  
  if (transition_callback_) // Trigger callback if available
  {
    transition_callback_(prev_state, state_, source_signal);
  }
}

}  // namespace wz_strategic_plugin