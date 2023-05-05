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

#include "approaching_emergency_vehicle_plugin/approaching_emergency_vehicle_states.hpp"

namespace approaching_emergency_vehicle_plugin
{
std::ostream& operator<<(std::ostream& os, ApproachingEmergencyVehicleState state)
{
  os << "ApproachingEmergencyVehicleState::";
  switch (state)
  {  // clang-format off
    case ApproachingEmergencyVehicleState::NO_APPROACHING_ERV   : os << "NO_APPROACHING_ERV"; break;
    case ApproachingEmergencyVehicleState::MOVING_OVER_FOR_APPROACHING_ERV: os << "MOVING_OVER_FOR_APPROACHING_ERV"; break;
    case ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV  : os << "SLOWING_DOWN_FOR_ERV"; break;
    default: os.setstate(std::ios_base::failbit);
  }  // clang-format on
  return os;
}

std::ostream& operator<<(std::ostream& os, ApproachingEmergencyVehicleEvent event)
{
  os << "ApproachingEmergencyVehicleEvent::";
  switch (event)
  {  // clang-format off
    case ApproachingEmergencyVehicleEvent::ERV_UPDATE_TIMEOUT   : os << "ERV_UPDATE_TIMEOUT"; break;
    case ApproachingEmergencyVehicleEvent::NO_APPROACHING_ERV: os << "NO_APPROACHING_ERV"; break;
    case ApproachingEmergencyVehicleEvent::APPROACHING_ERV_IN_PATH : os << "APPROACHING_ERV_IN_PATH"; break;
    case ApproachingEmergencyVehicleEvent::APPROACHING_ERV_NOT_IN_PATH  : os << "APPROACHING_ERV_NOT_IN_PATH"; break;
    case ApproachingEmergencyVehicleEvent::ERV_PASSING_IN_PATH  : os << "ERV_PASSING_IN_PATH"; break;
    case ApproachingEmergencyVehicleEvent::ERV_PASSED  : os << "ERV_PASSED"; break;

    default: os.setstate(std::ios_base::failbit);
  }  // clang-format on
  return os;
}

}  // namespace approaching_emergency_vehicle_plugin