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
//! @brief Struct to store the configuration settings for the LocalizationManager class
struct LocalizationManagerConfig
{
  //! NDT Fitness score above which the localization is considered in a degraded state
  double fitness_score_degraded_threshold = 1.0;
  //! NDT Fitness score above which the localization is considered in a fault state and NDT matching can no longer be
  //! used.
  double fitness_score_fault_threshold = 2.0;
  //! NDT solution frequency below which the localization is considered in a degraded state
  double ndt_frequency_degraded_threshold = 9;
  //! NDT solution frequency below which the localization is considered in a fault state and NDT matching can no longer
  //! be used.
  double ndt_frequency_fault_threshold = 5;
  //! Timeout in ms for auto initialization.
  //! If initialization cannot be completed in this time user action will be requested.
  int auto_initialization_timeout = 30000;
  //! Timeout in ms for GNSS only operation. Ignored when in GNSS mode.
  int gnss_initialization_timeout = 6000;
  //! Localization mode to use
  LocalizerMode localization_mode = LocalizerMode::NDT;
};
}  // namespace localizer