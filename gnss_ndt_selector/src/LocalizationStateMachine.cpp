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
#include "LocalizationStateMachine.h"

namespace localizer
{
LocalizationState LocalizationStateMachine::getState()
{
  return state_;
}

void LocalizationStateMachine::logDebugSignal(LocalizationSignal signal)
{
  ROS_DEBUG_LOG_STREAM("LocalizationStateMachine received unsupported signal of " << signal << " while in state "
                                                                                  << state_);
}

void LocalizationStateMachine::setAndLogState(LocalizationState new_state, LocalizationSignal source_signal)
{
  ROS_INFO_STREAM("LocalizationStateMachine changed localization state from "
                  << state_ << " to " << new_state << " because of signal " << source_signal);
  state_ = new_state;
}

void LocalizationStateMachine::signalWhenUNINITIALIZED(LocalizationSignal signal)
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

void LocalizationStateMachine::signalWhenINITIALIZING(LocalizationSignal signal)
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
void LocalizationStateMachine::signalWhenOPERATIONAL(LocalizationSignal signal)
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
void LocalizationStateMachine::signalWhenDEGRADED(LocalizationSignal signal)
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
void LocalizationStateMachine::signalWhenDEGRADED_NO_LIDAR_FIX(LocalizationSignal signal)
{
  switch (signal)
  {
    case INITIAL_POSE:
      setAndLogState(INITIALIZING, signal);
      break;
    case TIMEOUT:
      setAndLogState(AWAIT_MANUAL_INITIALIZATION, signal);
      break;
    default:
      logDebugSignal(signal);
      break;
  }
}
void LocalizationStateMachine::signalWhenAWAIT_MANUAL_INITIALIZATION(LocalizationSignal signal)
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

void LocalizationStateMachine::signal(LocalizationSignal signal)
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