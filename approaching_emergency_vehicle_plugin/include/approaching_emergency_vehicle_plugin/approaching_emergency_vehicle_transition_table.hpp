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

#pragma once

#include "approaching_emergency_vehicle_states.hpp"
#include <rclcpp/rclcpp.hpp>
#include <functional>

namespace approaching_emergency_vehicle_plugin
{
    /**
     * \brief Class defining the state transition table behavior for the ApproachingEmergencyVehiclePlugin
     */
    class ApproachingEmergencyVehicleTransitionTable
    {
    public:
        using TransitionCallback =
            std::function<void(ApproachingEmergencyVehicleState prev_state, ApproachingEmergencyVehicleState new_state, ApproachingEmergencyVehicleEvent event)>;

        /**
         * \brief ApproachingEmergencyVehicleTransitionTable Constructor
         *
         * \param mode Defines the operational mode of the state machine which modifies some of the state transitions
         */
        ApproachingEmergencyVehicleTransitionTable() = default;

        /**
         * \brief Returns the current state
         * \return Current state
         */
        ApproachingEmergencyVehicleState getState() const;

        /**
         * \brief Trigger event for the transition table.
         *
         * \param event The event for the transition table to evaluate
         */
        void event(ApproachingEmergencyVehicleEvent event);

        /**
         * \brief Callback setting function. The provided callback will be triggered any time the current state changes to a
         * new state.
         *
         * \param cb The callback function which will be provided with the previous state, new current state, and the event
         * which caused the transition.
         */
        void setTransitionCallback(TransitionCallback cb);

    private:
        //! Current state. This state should only ever be set using the setAndLogState() function.
        ApproachingEmergencyVehicleState state_ = ApproachingEmergencyVehicleState::NO_APPROACHING_ERV;

        TransitionCallback transition_callback_;

        // Helper functions for processing each the provided events based on the current state
        void eventWhenNO_APPROACHING_ERV(ApproachingEmergencyVehicleEvent event);
        void eventWhenMOVING_OVER_FOR_APPROACHING_ERV(ApproachingEmergencyVehicleEvent event);
        void eventWhenWAITING_FOR_APPROACHING_ERV(ApproachingEmergencyVehicleEvent event);
        void eventWhenSLOWING_DOWN_FOR_ERV(ApproachingEmergencyVehicleEvent event);

        /**
         * \brief Helper function for logging the provide event
         * \param event The event to be logged
         */
        void logDebugEvent(ApproachingEmergencyVehicleEvent event) const;

        /**
         * \brief Function to change the current state and log the details of the transition.
         *
         * \param new_state The state to set.
         * \param source_event The event which caused the new_state to be set
         */
        void setAndLogState(ApproachingEmergencyVehicleState new_state, ApproachingEmergencyVehicleEvent source_event);
    };
} // namespace approaching_emergency_vehicle_plugin