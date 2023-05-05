#pragma once
/*
 * Copyright (C) 2022 LEIDOS.
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

namespace approaching_emergency_vehicle_plugin
{

//! @brief Enum describing the possible states of the ApproachingEmergencyVehiclePlugin (Strategic Plugin)
enum class ApproachingEmergencyVehicleState
{
    NO_APPROACHING_ERV, // State representing that an ERV is not passing the ego vehicle within the duration defined by 
                        // the "approaching_threshold" configurable parameter.
    MOVING_OVER_FOR_APPROACHING_ERV, // State representing that the ego vehicle must change lanes in response to an approaching ERV.

    SLOWING_DOWN_FOR_ERV  // State representing that the ERV is actively passing the ego vehicle, so the ego vehicle should remain in its 
                          // current lane and slow down if necessary.
};

/**
 * \brief Stream operator for ApproachingEmergencyVehicleState enum.
 */
std::ostream& operator<<(std::ostream& os, ApproachingEmergencyVehicleState state);


//! @brief Enum describing the possible signals to change the current ApproachingEmergencyVehicleState
enum class ApproachingEmergencyVehicleEvent
{
    ERV_UPDATE_TIMEOUT, // Event representing that no BSMs have been received from the currently-tracked ERV within the duration
                        // defined by the "timeout_duration" configurable parameters.

    NO_APPROACHING_ERV, // Event representing that an ERV is not passing the ego vehicle within the duration defined by 
                        // the "approaching_threshold" configurable parameter.

    APPROACHING_ERV_IN_PATH, // Event representing that an ERV is approaching the ego vehicle and the ego vehicle is either
                             // in the same lane as the ERV or the ego vehicle is not in the rightmost lane.
    APPROACHING_ERV_NOT_IN_PATH, // Event representing that an ERV is approaching the ego vehicle and the ego vehicle is either in the
                                 // rightmost lane, or is in a lane adjacent to the rightmost lane if the ERV is in the rightmost lane.

    ERV_PASSING_IN_PATH, // Event representing that the ERV is actively passing the ego vehicle, and the ego vehicle is either
                         // in the same lane as the ERV or the ego vehicle is not in the rightmost lane.

    ERV_PASSED  // Event representing that the ERV has fully passed the ego vehicle.
};

/**
 * \brief Stream operator for ApproachingEmergencyVehicleEvent enum.
 */
std::ostream& operator<<(std::ostream& os, ApproachingEmergencyVehicleEvent event);

}  // namespace approaching_emergency_vehicle_plugin