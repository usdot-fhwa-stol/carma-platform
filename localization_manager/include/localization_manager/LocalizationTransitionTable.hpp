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

#include "LocalizationTypes.hpp"

namespace localization_manager
{
    /**
     * \brief Class defining the state transition table behavior for the LocalizationManager
     */
    class LocalizationTransitionTable
    {
    public:
        using TransitionCallback =
            std::function<void(LocalizationState prev_state, LocalizationState new_state, LocalizationSignal signal)>;

        /**
         * \brief Constructor
         *
         * \param mode Defines the operational mode of the state machine which modifies some of the state transitions
         */
        LocalizationTransitionTable(LocalizerMode mode);

        /**
         * \brief Returns the current state
         * \return Current state
         */
        LocalizationState getState() const;

        /**
         * \brief Trigger signal for the transition table.
         *
         * \param signal The signal for the transition table to evaluate
         */
        void signal(LocalizationSignal signal);

        /**
         * \brief Callback setting function. The provided callback will be triggered any time the current state changes to a
         * new state.
         *
         * \param cb The callback function which will be provided with the previous state, new current state, and the signal
         * which caused the transition.
         */
        void setTransitionCallback(TransitionCallback cb);

    private:
        //! Current state. This state should only ever be set using the setAndLogState() function.
        LocalizationState state_ = LocalizationState::UNINITIALIZED;

        LocalizerMode mode_ = LocalizerMode::AUTO_WITHOUT_TIMEOUT;

        TransitionCallback transition_callback_;

        // Helper functions for processing each the provided signal based on the current state
        void signalWhenUNINITIALIZED(LocalizationSignal signal);
        void signalWhenINITIALIZING(LocalizationSignal signal);
        void signalWhenOPERATIONAL(LocalizationSignal signal);
        void signalWhenDEGRADED(LocalizationSignal signal);
        void signalWhenDEGRADED_NO_LIDAR_FIX(LocalizationSignal signal);
        void signalWhenAWAIT_MANUAL_INITIALIZATION(LocalizationSignal signal);

        /**
         * \brief Helper function for logging the provide signal
         * \param signal The signal to be logged
         */
        void logDebugSignal(LocalizationSignal signal) const;

        /**
         * \brief Function to change the current state and log the details of the transition.
         *
         * \param new_state The state to set.
         * \param source_signal The signal which caused the new_state to be set
         */
        void setAndLogState(LocalizationState new_state, LocalizationSignal source_signal);
    };
} // namespace localization_manager