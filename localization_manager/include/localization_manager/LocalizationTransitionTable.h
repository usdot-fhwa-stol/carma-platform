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
#include "LocalizationTypes.h"

namespace localizer
{

class LocalizationTransitionTable
{
public:
  using TransitionCallback =
      std::function<void(LocalizationState prev_state, LocalizationState new_state, LocalizationSignal signal)>;
  LocalizationTransitionTable(LocalizerMode mode);
  LocalizationState getState();
  void signal(LocalizationSignal signal);
  void setTransitionCallback(TransitionCallback cb);

private:
  LocalizationState state_ = LocalizationState::UNINITIALIZED;
  LocalizerMode mode_ = LocalizerMode::AUTO;

  TransitionCallback transition_callback_;

  void signalWhenUNINITIALIZED(LocalizationSignal signal);
  void signalWhenINITIALIZING(LocalizationSignal signal);
  void signalWhenOPERATIONAL(LocalizationSignal signal);
  void signalWhenDEGRADED(LocalizationSignal signal);
  void signalWhenDEGRADED_NO_LIDAR_FIX(LocalizationSignal signal);
  void signalWhenAWAIT_MANUAL_INITIALIZATION(LocalizationSignal signal);

  void logDebugSignal(LocalizationSignal signal);

  void setAndLogState(LocalizationState new_state, LocalizationSignal source_signal);
};
}  // namespace localizer