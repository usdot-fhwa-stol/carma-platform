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

#include "localization_manager/LocalizationTransitionTable.hpp"

namespace localization_manager
{

    LocalizationTransitionTable::LocalizationTransitionTable(LocalizerMode mode) : mode_(mode) {}

    LocalizationState LocalizationTransitionTable::getState() const
    {
        return state_;
    }

    void LocalizationTransitionTable::logDebugSignal(LocalizationSignal signal) const
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("localization_manager"), "LocalizationTransitionTable received unsupported signal of " << signal << "while in state" << state_);
    }

    void LocalizationTransitionTable::setAndLogState(LocalizationState new_state, LocalizationSignal source_signal)
    {
        if (new_state == state_)
        {
            return; // State was unchanged no need to log or trigger callbacks
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("localization_manager"), "LocalizationTransitionTable changed localization state from " << state_ << " to " << new_state << " because of signal " << source_signal << " while in mode " << mode_);

        LocalizationState prev_state = state_;
        state_ = new_state;
        if (transition_callback_)
        {
            transition_callback_(prev_state, state_, source_signal);
        }
    }

    void LocalizationTransitionTable::signalWhenUNINITIALIZED(LocalizationSignal signal)
    {
        switch (signal)
        {
        case LocalizationSignal::INITIAL_POSE:
            if (mode_ == LocalizerMode::GNSS || mode_ == LocalizerMode::GNSS_WITH_FIXED_OFFSET)
            {
                setAndLogState(LocalizationState::DEGRADED_NO_LIDAR_FIX, signal);
            }
            else
            {
                setAndLogState(LocalizationState::INITIALIZING, signal);
            }
            break;
        default:
            logDebugSignal(signal);
            break;
        }
    }

    void LocalizationTransitionTable::signalWhenINITIALIZING(LocalizationSignal signal)
    {
        switch (signal)
        {
        // How to handle the combined conditions?
        case LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE:
            setAndLogState(LocalizationState::OPERATIONAL, signal);
            break;
        case LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE:
            setAndLogState(LocalizationState::DEGRADED, signal);
            break;
        case LocalizationSignal::LIDAR_SENSOR_FAILURE:
            if (mode_ == LocalizerMode::NDT)
            {
                setAndLogState(LocalizationState::AWAIT_MANUAL_INITIALIZATION, signal);
            }
            else
            {
                setAndLogState(LocalizationState::DEGRADED_NO_LIDAR_FIX, signal);
            }
            break;
        case LocalizationSignal::TIMEOUT:
            setAndLogState(LocalizationState::AWAIT_MANUAL_INITIALIZATION, signal);
            break;
        default:
            logDebugSignal(signal);
            break;
        }
    }

    void LocalizationTransitionTable::signalWhenOPERATIONAL(LocalizationSignal signal)
    {
        switch (signal)
        {
        case LocalizationSignal::INITIAL_POSE:
            setAndLogState(LocalizationState::INITIALIZING, signal);
            break;
        case LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE:
            setAndLogState(LocalizationState::DEGRADED, signal);
            break;
        case LocalizationSignal::LIDAR_SENSOR_FAILURE: // Allowing fallthrough for duplicated behavior
        case LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE:
            if (mode_ == LocalizerMode::AUTO_WITH_TIMEOUT || mode_ == LocalizerMode::AUTO_WITHOUT_TIMEOUT)
            {
                setAndLogState(LocalizationState::DEGRADED_NO_LIDAR_FIX, signal);
            }
            else
            {
                setAndLogState(LocalizationState::AWAIT_MANUAL_INITIALIZATION, signal);
            }
            break;
        case LocalizationSignal::LIDAR_INITIALIZED_SWITCH_TO_GPS:
            if (mode_ == LocalizerMode::GNSS_WITH_NDT_INIT)
            {
                setAndLogState(LocalizationState::DEGRADED_NO_LIDAR_FIX, signal);
            }
            break;
        default:
            logDebugSignal(signal);
            break;
        }
    }

    void LocalizationTransitionTable::signalWhenDEGRADED(LocalizationSignal signal)
    {
        switch (signal)
        {
        case LocalizationSignal::INITIAL_POSE:
            setAndLogState(LocalizationState::INITIALIZING, signal);
            break;
        case LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE:
            setAndLogState(LocalizationState::OPERATIONAL, signal);
            break;
        case LocalizationSignal::LIDAR_SENSOR_FAILURE:
            if (mode_ == LocalizerMode::AUTO_WITH_TIMEOUT || mode_ == LocalizerMode::AUTO_WITHOUT_TIMEOUT)
            {
                setAndLogState(LocalizationState::DEGRADED_NO_LIDAR_FIX, signal);
            }
            else
            {
                setAndLogState(LocalizationState::AWAIT_MANUAL_INITIALIZATION, signal);
            }
            break;
        case LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE:
            if (mode_ == LocalizerMode::AUTO_WITH_TIMEOUT || mode_ == LocalizerMode::AUTO_WITHOUT_TIMEOUT)
            {
                setAndLogState(LocalizationState::DEGRADED_NO_LIDAR_FIX, signal);
            }
            else
            {
                setAndLogState(LocalizationState::AWAIT_MANUAL_INITIALIZATION, signal);
            }
            break;
        default:
            logDebugSignal(signal);
            break;
        }
    }

    void LocalizationTransitionTable::signalWhenDEGRADED_NO_LIDAR_FIX(LocalizationSignal signal)
    {
        switch (signal)
        {
        case LocalizationSignal::INITIAL_POSE:
            if (mode_ != LocalizerMode::GNSS && mode_ != LocalizerMode::GNSS_WITH_FIXED_OFFSET)
            {
                setAndLogState(LocalizationState::INITIALIZING, signal);
            }
            break;
        case LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE:
            if (mode_ != LocalizerMode::GNSS && mode_ != LocalizerMode::GNSS_WITH_FIXED_OFFSET)
            {
                setAndLogState(LocalizationState::OPERATIONAL, signal);
            }
            break;
        case LocalizationSignal::TIMEOUT:
            if (mode_ != LocalizerMode::GNSS && mode_ != LocalizerMode::GNSS_WITH_FIXED_OFFSET && mode_ != LocalizerMode::AUTO_WITHOUT_TIMEOUT && mode_ != LocalizerMode::GNSS_WITH_NDT_INIT)
            {
                setAndLogState(LocalizationState::AWAIT_MANUAL_INITIALIZATION, signal);
            }
            break;
        case LocalizationSignal::GNSS_DATA_TIMEOUT:
            throw std::runtime_error("GNSS_DATA_TIMEOUT occurred while in DEGRADED_NO_LIDAR_FIX state. Localization cannot recover");
        default:
            logDebugSignal(signal);
            break;
        }
    }

    void LocalizationTransitionTable::signalWhenAWAIT_MANUAL_INITIALIZATION(LocalizationSignal signal)
    {
        switch (signal)
        {
        case LocalizationSignal::INITIAL_POSE:
            if (mode_ == LocalizerMode::GNSS || mode_ == LocalizerMode::GNSS_WITH_FIXED_OFFSET)
            {
                setAndLogState(LocalizationState::DEGRADED_NO_LIDAR_FIX, signal);
            }
            else
            {
                setAndLogState(LocalizationState::INITIALIZING, signal);
            }
            break;
        default:
            logDebugSignal(signal);
            break;
        }
    }

    void LocalizationTransitionTable::signal(LocalizationSignal signal)
    {
        switch (state_)
        {
        case LocalizationState::UNINITIALIZED:
            signalWhenUNINITIALIZED(signal);
            break;
        case LocalizationState::INITIALIZING:
            signalWhenINITIALIZING(signal);
            break;
        case LocalizationState::OPERATIONAL:
            signalWhenOPERATIONAL(signal);
            break;
        case LocalizationState::DEGRADED:
            signalWhenDEGRADED(signal);
            break;
        case LocalizationState::DEGRADED_NO_LIDAR_FIX:
            signalWhenDEGRADED_NO_LIDAR_FIX(signal);
            break;
        case LocalizationState::AWAIT_MANUAL_INITIALIZATION:
            signalWhenAWAIT_MANUAL_INITIALIZATION(signal);
            break;
        default:
            throw std::invalid_argument("Invalid signal passed to LocalizationTransitionTable::signal");
            break;
        }
    }

    void LocalizationTransitionTable::setTransitionCallback(TransitionCallback cb)
    {
        transition_callback_ = cb;
    }

} // namespace localization_manager