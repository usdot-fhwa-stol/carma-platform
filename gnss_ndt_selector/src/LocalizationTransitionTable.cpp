/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include <ros/ros.h>
#include "LocalizationTransitionTable.h"

namespace localizer
{
LocalizationTransitionTable(LocalizerMode mode) : mode_(mode)
{
}
LocalizationState LocalizationTransitionTable::getState()
{
  return state_;
}

void LocalizationTransitionTable::logDebugSignal(LocalizationSignal signal)
{
  ROS_DEBUG_LOG_STREAM("LocalizationTransitionTable received unsupported signal of " << signal << " while in state "
                                                                                     << state_);
}

void LocalizationTransitionTable::setAndLogState(LocalizationState new_state, LocalizationSignal source_signal)
{
  if (new_state == state_) {
    return; // State was unchanged no need to log or trigger callbacks
  }
  ROS_INFO_STREAM("LocalizationTransitionTable changed localization state from "
                  << state_ << " to " << new_state << " because of signal " << source_signal " while in mode " << mode_);
  LocalizationState prev_state = state_;
  state_ = new_state;
  if (transition_callback_) {
    transition_callback_(prev_state, state_, source_signal);
  }
}

void LocalizationTransitionTable::signalWhenUNINITIALIZED(LocalizationSignal signal)
{
  switch (signal)
  {
    case INITIAL_POSE:
      if (mode_ == LocalizerMode::GNSS) {
        setAndLogState(DEGRADED_NO_LIDAR_FIX, signal);
      } else {
        setAndLogState(INITIALIZING, signal);
      }
      break;
    default:
      logDebugSignal(signal);
      break;
  }
}

void LocalizationTransitionTable::signalWhenINITIALIZING(LocalizationSignal signal)
{
  switch (signal)
  {
    // How to handle the combined conditions?
    case GOOD_NDT_FREQ_AND_FITNESS_SCORE:
      setAndLogState(OPERATIONAL, signal);
      break;
    case POOR_NDT_FREQ_OR_FITNESS_SCORE:
      setAndLogState(DEGRADED, signal);
      break;
    case LIDAR_SENSOR_FAILURE:  // TODO should we support this in the initialization phase?
      setAndLogState(DEGRADED_NO_LIDAR_FIX, signal);
      break;
    case TIMEOUT:
      setAndLogState(AWAIT_MANUAL_INITIALIZATION, signal);
      break;
    default:
      logDebugSignal(signal);
      break;
  }
}
void LocalizationTransitionTable::signalWhenOPERATIONAL(LocalizationSignal signal)
{
  switch (signal)
  {
    case INITIAL_POSE:
      setAndLogState(INITIALIZING, signal);
      break;
    case POOR_NDT_FREQ_OR_FITNESS_SCORE:
      setAndLogState(DEGRADED, signal);
      break;
    case LIDAR_SENSOR_FAILURE:
      setAndLogState(DEGRADED_NO_LIDAR_FIX, signal);
      break;
    case UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE:
      setAndLogState(DEGRADED_NO_LIDAR_FIX, signal);
      break;
    default:
      logDebugSignal(signal);
      break;
  }
}
void LocalizationTransitionTable::signalWhenDEGRADED(LocalizationSignal signal)
{
  switch (signal)
  {
    case INITIAL_POSE:
      setAndLogState(INITIALIZING, signal);
      break;
    case GOOD_NDT_FREQ_AND_FITNESS_SCORE:
      setAndLogState(OPERATIONAL, signal);
      break;
    case LIDAR_SENSOR_FAILURE:
      if (mode_ == LocalizerMode::AUTO) {
        setAndLogState(DEGRADED_NO_LIDAR_FIX, signal);
      } else {
        setAndLogState(AWAIT_MANUAL_INITIALIZATION, signal);
      }
      break;
    case UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE:
      setAndLogState(DEGRADED_NO_LIDAR_FIX, signal);
      break;
    default:
      logDebugSignal(signal);
      break;
  }
}
void LocalizationTransitionTable::signalWhenDEGRADED_NO_LIDAR_FIX(LocalizationSignal signal)
{
  switch (signal)
  {
    case INITIAL_POSE:
      setAndLogState(INITIALIZING, signal);
      break;
    case TIMEOUT:
      if (mode_ != LocalizerMode::GNSS) {
        setAndLogState(AWAIT_MANUAL_INITIALIZATION, signal);
      }
      break;
    default:
      logDebugSignal(signal);
      break;
  }
}
void LocalizationTransitionTable::signalWhenAWAIT_MANUAL_INITIALIZATION(LocalizationSignal signal)
{
  switch (signal)
  {
    case INITIAL_POSE:
      setAndLogState(INITIALIZING, signal);
      break;
    default:
      logDebugSignal(signal);
      break;
  }
}

void LocalizationTransitionTable::signal(LocalizationSignal signal)
{
  switch (state_)
  {
    case UNINITIALIZED:
      signalWhenUNINITIALIZED(signal);
      break;
    case INITIALIZING:
      signalWhenINITIALIZING(signal);
      break;
    case OPERATIONAL:
      signalWhenOPERATIONAL(signal);
      break;
    case DEGRADED:
      signalWhenDEGRADED(signal);
      break;
    case DEGRADED_NO_LIDAR_FIX:
      signalWhenDEGRADED_NO_LIDAR_FIX(signal);
      break;
    case AWAIT_MANUAL_INITIALIZATION:
      signalWhenAWAIT_MANUAL_INITIALIZATION(signal);
      break;
    default:
      // TODO throw error
      break;
  }
}
}  // namespace localizer