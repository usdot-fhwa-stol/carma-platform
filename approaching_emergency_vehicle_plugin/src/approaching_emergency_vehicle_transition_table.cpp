/*
 * Copyright (C) 2022-2023 LEIDOS.
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

#include "approaching_emergency_vehicle_plugin/approaching_emergency_vehicle_transition_table.hpp"

namespace approaching_emergency_vehicle_plugin
{

ApproachingEmergencyVehicleState ApproachingEmergencyVehicleTransitionTable::getState() const
{
  return state_;
}

void ApproachingEmergencyVehicleTransitionTable::event(ApproachingEmergencyVehicleEvent event)
{
  switch (state_)
  {
    case ApproachingEmergencyVehicleState::NO_APPROACHING_ERV:
      eventWhenNO_APPROACHING_ERV(event);
      break;

    case ApproachingEmergencyVehicleState::MOVING_OVER_FOR_APPROACHING_ERV:
      eventWhenMOVING_OVER_FOR_APPROACHING_ERV(event);
      break;

    case ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV:
      eventWhenSLOWING_DOWN_FOR_ERV(event);
      break;

    default:
      throw std::invalid_argument("Transition table in unsupported state");
  }
}

void ApproachingEmergencyVehicleTransitionTable::setTransitionCallback(TransitionCallback cb)
{
  transition_callback_ = cb;
}

void ApproachingEmergencyVehicleTransitionTable::eventWhenNO_APPROACHING_ERV(ApproachingEmergencyVehicleEvent event)
{
  switch (event)
  {
    case ApproachingEmergencyVehicleEvent::NO_APPROACHING_ERV:
      setAndLogState(ApproachingEmergencyVehicleState::NO_APPROACHING_ERV, event);
      break;

    case ApproachingEmergencyVehicleEvent::APPROACHING_ERV_IN_PATH:
      setAndLogState(ApproachingEmergencyVehicleState::MOVING_OVER_FOR_APPROACHING_ERV, event);
      break;

    case ApproachingEmergencyVehicleEvent::APPROACHING_ERV_NOT_IN_PATH:
      setAndLogState(ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV, event);
      break;

    case ApproachingEmergencyVehicleEvent::ERV_PASSING_IN_PATH:
      setAndLogState(ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV, event);
      break;

    default:
      logDebugEvent(event);
      break;
  }
}

void ApproachingEmergencyVehicleTransitionTable::eventWhenMOVING_OVER_FOR_APPROACHING_ERV(ApproachingEmergencyVehicleEvent event)
{
  switch (event)
  {
    case ApproachingEmergencyVehicleEvent::ERV_UPDATE_TIMEOUT:
      setAndLogState(ApproachingEmergencyVehicleState::NO_APPROACHING_ERV, event);
      break;

    case ApproachingEmergencyVehicleEvent::NO_APPROACHING_ERV:
      setAndLogState(ApproachingEmergencyVehicleState::NO_APPROACHING_ERV, event);
      break;

    case ApproachingEmergencyVehicleEvent::APPROACHING_ERV_IN_PATH:
      setAndLogState(ApproachingEmergencyVehicleState::MOVING_OVER_FOR_APPROACHING_ERV, event);
      break;

    case ApproachingEmergencyVehicleEvent::APPROACHING_ERV_NOT_IN_PATH:
      setAndLogState(ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV, event);
      break;

    case ApproachingEmergencyVehicleEvent::ERV_PASSING_IN_PATH:
      setAndLogState(ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV, event);
      break;

    default:
      logDebugEvent(event);
      break;
  }
}

void ApproachingEmergencyVehicleTransitionTable::eventWhenSLOWING_DOWN_FOR_ERV(ApproachingEmergencyVehicleEvent event)
{
  switch (event)
  {
    case ApproachingEmergencyVehicleEvent::ERV_UPDATE_TIMEOUT:
      setAndLogState(ApproachingEmergencyVehicleState::NO_APPROACHING_ERV, event);
      break;

    case ApproachingEmergencyVehicleEvent::NO_APPROACHING_ERV:
      setAndLogState(ApproachingEmergencyVehicleState::NO_APPROACHING_ERV, event);
      break;

    case ApproachingEmergencyVehicleEvent::APPROACHING_ERV_IN_PATH:
      setAndLogState(ApproachingEmergencyVehicleState::MOVING_OVER_FOR_APPROACHING_ERV, event);
      break;

    case ApproachingEmergencyVehicleEvent::APPROACHING_ERV_NOT_IN_PATH:
      setAndLogState(ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV, event);
      break;

    case ApproachingEmergencyVehicleEvent::ERV_PASSING_IN_PATH:
      setAndLogState(ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV, event);
      break;

    case ApproachingEmergencyVehicleEvent::ERV_PASSED:
      setAndLogState(ApproachingEmergencyVehicleState::NO_APPROACHING_ERV, event);
      break;

    default:
      logDebugEvent(event);
      break;
  }
}

void ApproachingEmergencyVehicleTransitionTable::logDebugEvent(ApproachingEmergencyVehicleEvent event) const
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("approaching_emergency_vehicle_plugin"), "ApproachingEmergencyVehicleTransitionTable received unsupported event of " << event << " while in state "
                                                                                  << state_);
}

void ApproachingEmergencyVehicleTransitionTable::setAndLogState(ApproachingEmergencyVehicleState new_state, ApproachingEmergencyVehicleEvent source_event)
{
  if (new_state == state_)
  {
    return;  // State was unchanged no need to log or trigger callbacks
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("approaching_emergency_vehicle_plugin"), "ApproachingEmergencyVehicleTransitionTable changed ApproachingEmergencyVehicle Strategic Plugin state from "
                  << state_ << " to " << new_state << " because of event " << source_event);
 
  ApproachingEmergencyVehicleState prev_state = state_;
  state_ = new_state; // Set new state
  
  if (transition_callback_) // Trigger callback if available
  {
    transition_callback_(prev_state, state_, source_event);
  }
}

}  // namespace approaching_emergency_vehicle_plugin