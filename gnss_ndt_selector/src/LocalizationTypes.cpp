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
std::ostream& operator<<(std::ostream& os, LocalizerMode m)
{
  switch (m)
  {  // clang-format off
    case LocalizerMode::NDT   : os << "NDT"; break;
    case LocalizerMode::GNSS: os << "GNSS"; break;
    case LocalizerMode::AUTO : os << "AUTO"; break;
    default: os.setstate(std::ios_base::failbit);
  }  // clang-format on
  return os;
}

std::ostream& operator<<(std::ostream& os, LocalizationState s)
{
  switch (s)
  {  // clang-format off
    case LocalizationState::UNINITIALIZED   : os << "UNINITIALIZED"; break;
    case LocalizationState::INITIALIZING: os << "INITIALIZING"; break;
    case LocalizationState::OPERATIONAL : os << "OPERATIONAL"; break;
    case LocalizationState::DEGRADED  : os << "DEGRADED"; break;
    case LocalizationState::DEGRADED_NO_LIDAR_FIX  : os << "DEGRADED_NO_LIDAR_FIX"; break;
    case LocalizationState::AWAIT_MANUAL_INITIALIZATION  : os << "AWAIT_MANUAL_INITIALIZATION"; break;
    default: os.setstate(std::ios_base::failbit);
  }  // clang-format on
  return os;
}

std::ostream& operator<<(std::ostream& os, LocalizationSignal s)
{
  switch (s)
  {  // clang-format off
    case LocalizationSignal::INITIAL_POSE   : os << "INITIAL_POSE"; break;
    case LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE: os << "GOOD_NDT_FREQ_AND_FITNESS_SCORE"; break;
    case LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE : os << "POOR_NDT_FREQ_OR_FITNESS_SCORE"; break;
    case LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE  : os << "UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE"; break;
    case LocalizationSignal::TIMEOUT  : os << "TIMEOUT"; break;
    case LocalizationSignal::LIDAR_SENSOR_FAILURE  : os << "LIDAR_SENSOR_FAILURE"; break;
    default: os.setstate(std::ios_base::failbit);
  }  // clang-format on
  return os;
}

}  // namespace localizer