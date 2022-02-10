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

#include <ostream>

namespace wz_strategic_plugin
{

//! @brief Enum describing the possible states of the WorkZone Strategic Plugin
enum class TransitState
{
  UNAVAILABLE, // State representing that there are no applicable intersections in range, so the plugin cannot plan
  APPROACHING, // State representing that the vehicle is approaching an intersection
  WAITING, // State representing that the vehicle is stopped and waiting at a light
  DEPARTING // State representing that the vehicle is traversing the intersection
};

/**
 * \brief Stream operator for TransitStates enum.
 */
std::ostream& operator<<(std::ostream& os, TransitState s);

//! @brief Enum describing the possible signals to change the current TransitState
enum class TransitEvent
{
  IN_STOPPING_RANGE, // Transition event representing that the vehicle is in the stopping range of a light
  STOPPED, // Transition event representing that the vehicle has come to a full stop
  CROSSED_STOP_BAR, // Transition event representing that the vehicle has crossed the stop bar
  RED_TO_GREEN_LIGHT, // Transition event representing that the current light of interest has changed from red to green
  INTERSECTION_EXIT // Transition event representing that the end of the current intersection was crossed
};
/**
 * \brief Stream operator for TransitEvent enum.
 */
std::ostream& operator<<(std::ostream& os, TransitEvent s);

}  // namespace localizer