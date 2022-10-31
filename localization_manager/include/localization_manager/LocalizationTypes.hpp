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

#include <iostream>
#include <carma_localization_msgs/msg/localization_status_report.hpp>
#include <rclcpp/rclcpp.hpp>

namespace localization_manager
{
    //! @brief Enum describing the possible operational modes of the LocalizationManager
    enum LocalizerMode
    {
        NDT = 0,                  // NDT only operation
        GNSS = 1,                 // GNSS only operation
        AUTO_WITH_TIMEOUT = 2,    // NDT operation with support for GPS fallback that will timeout
        AUTO_WITHOUT_TIMEOUT = 3, // NDT operation with support for GPS fallback that will not timeout
        GNSS_WITH_NDT_INIT = 4,   // GNSS only operation with NDT initialization, switching to GNSS after 5 sequential timesteps of OPERATIONAL NDT
        GNSS_WITH_FIXED_OFFSET = 5 // GNSS only operation with fixed offset
    };

    /**
     * \brief Stream operator for LocalizerMode enum.
     */
    std::ostream &operator<<(std::ostream &os, LocalizerMode m);

    //! @brief Enum describing the possible states of the localization system
    enum class LocalizationState
    {
        UNINITIALIZED,
        INITIALIZING,
        OPERATIONAL,
        DEGRADED,
        DEGRADED_NO_LIDAR_FIX,
        AWAIT_MANUAL_INITIALIZATION,
    };

    /**
     * \brief Stream operator for LocalizationState enum.
     */
    std::ostream &operator<<(std::ostream &os, LocalizationState s);

    /**
     * \brief Helper function to convert LocalizationState objects into LocalizationStatusReport ROS messages
     *
     * \param state The state to convert
     * \param stamp The timestamp to set for the message header
     *
     * \return The initialized report message
     */
    carma_localization_msgs::msg::LocalizationStatusReport stateToMsg(LocalizationState state, const rclcpp::Time &stamp);

    //! @brief Enum describing the possible signals to change the current LocalizationState
    enum class LocalizationSignal
    {
        INITIAL_POSE,
        GOOD_NDT_FREQ_AND_FITNESS_SCORE,
        POOR_NDT_FREQ_OR_FITNESS_SCORE,
        UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE,
        TIMEOUT,
        LIDAR_SENSOR_FAILURE,
        LIDAR_INITIALIZED_SWITCH_TO_GPS,
        GNSS_DATA_TIMEOUT
    };

    /**
     * \brief Stream operator for LocalizationSignal enum.
     */
    std::ostream &operator<<(std::ostream &os, LocalizationSignal s);

} // namespace localization_manager