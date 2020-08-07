#pragma once
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
#include "LocalizerMode.h"

namespace localizer
{
//! @brief Enum describing the possible states of the localization system
enum LocalizationState
{
  UNINITIALIZED,
  INITIALIZING,
  OPERATIONAL,
  DEGRADED,
  DEGRADED_NO_LIDAR_FIX,
  AWAIT_MANUAL_INITIALIZATION,
};

//! @brief Enum describing the possible signals to change the current LocalizationState
enum LocalizationSignal
{
  INITIAL_POSE,
  GOOD_NDT_FREQ_AND_FITNESS_SCORE,
  POOR_NDT_FREQ_OR_FITNESS_SCORE,
  UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE,
  TIMEOUT,
  LIDAR_SENSOR_FAILURE,
};

class LocalizationStateMachine
{
private:
  LocalizationState state_ = LocalizationState::UNINITIALIZED;

  void signalWhenUNINITIALIZED(LocalizationSignal signal);
  void signalWhenINITIALIZING(LocalizationSignal signal);
  void signalWhenOPERATIONAL(LocalizationSignal signal);
  void signalWhenDEGRADED(LocalizationSignal signal);
  void signalWhenDEGRADED_NO_LIDAR_FIX(LocalizationSignal signal);
  void signalWhenAWAIT_MANUAL_INITIALIZATION(LocalizationSignal signal);

  void logDebugSignal(LocalizationSignal signal);

  void setAndLogState(LocalizationState new_state, LocalizationSignal source_signal);

public:
  LocalizationState getState();
  void signal(LocalizationSignal signal);
};
}  // namespace localizer