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
#include "LocalizationTransitionTable.h"
#include "LocalizationManagerConfig.h"
#include <boost/optional.hpp>

namespace localizer
{
//! @brief Enum describing the possible states of the localization system
struct LocalizationStatusData
{
  double ndt_frequency = 0;
  double ndt_fitness_score = 0;
  bool lidar_failure = false;
};
// TODO this class seems a bit unnecessary
class SignalGenerator
{
private:
  LocalizationManagerConfig config_;

public:
  LocalizationSignal computeSignal(LocalizationStatusData data)
  {
     if (data.lidar_failure)
    {
      return LocalizationSignal::LIDAR_SENSOR_FAILURE;
    }
    else if (data.ndt_fitness_score > config_.fitness_score_fault_threshold ||
             data.ndt_frequency < config_.ndt_frequency_fault_threshold)
    {
      return LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE;
    }
    else if (data.ndt_fitness_score > config_.fitness_score_degraded_threshold ||
             data.ndt_frequency < config_.ndt_frequency_degraded_threshold)
    {
      return LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE;
    }
    else if ()
    {
      return LocalizationSignal::
    }
  }
  void setConfig(LocalizationManagerConfig config)
  {
    config_ = config;
  }
};
}  // namespace localizer