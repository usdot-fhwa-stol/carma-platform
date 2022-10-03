/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <vector>
#include <climits>
#include <random>

namespace bsm_generator
{

    /**
     * \class BSMGeneratorWorker
     * \brief The class containing the primary business logic for the BSM Generator Package
     */
    class BSMGeneratorWorker
    {
        public:

            /**
             * \brief Default Constructor for BSMGeneratorWorker
             */
            BSMGeneratorWorker();

            /**
             * \brief Function to increment the BSM message counter and obtain the new counter value.
             * Counter restarts at 0 once it reaches a value of 128.
             * \return The updated BSM message counter value
             */
            uint8_t getNextMsgCount();

            /**
             * \brief Function to obtain the current BSM message ID. The ID is updated to a new random BSM 
             * message ID every 5 minutes.
             * \param now The current time
             * \param secs Id change period in sec
             * \return The current BSM message ID
             */
            std::vector<uint8_t> getMsgId(const rclcpp::Time now, double secs);

            /**
             * \brief Function to obtain the 'milliseconds' mark of the provided time within the last minute
             * \param now The current time
             * \return The 'milliseconds' mark of the provided time within the last minute
             */
            uint16_t getSecMark(const rclcpp::Time now);

            /**
             * \brief Function to apply minimum and maximum limits to a speed value. Minimum limit is 0.0, and
             * maximum limit is 163.8.
             * \param speed The current vehicle speed
             * \return Speed value (minimum limit is 0.0, maximum limit is 163.8)
             */
            float getSpeedInRange(const double speed);

            /**
             * \brief Function to apply minimum and maximum limits to a steering wheel angle value. Minimum limit
             * is -189.0, maximum limit is 189.0.
             * \param angle The current steering wheel angle
             * \return Steering wheel angle value (minimum limit is -189.0, maximum limit is 189.0)
             */
            float getSteerWheelAngleInRange(const double angle);

            /**
             * \brief Function to apply minimum and maximum limits to a longitudinal acceleration value. Minimum limit
             * is -20.0, maximum limit is 20.0.
             * \param accel The current longitudinal acceleration value
             * \return Longitudinal acceleration value (minimum limit is -20.0, maximum limit is 20.0)
             */
            float getLongAccelInRange(const float accel);

            /**
             * \brief Function to apply minimum and maximum limits to a yaw rate value. Minimum limit
             * is -327.67, maximum limit is 327.67.
             * \param yaw_rate The current yaw rate value
             * \return Yaw Rate value (minimum limit is -327.67, maximum limit is 327.67)
             */
            float getYawRateInRange(const double yaw_rate);

            /**
             * \brief Function to convert the current applied brake status to a value used within the BSM message.
             * \param brake The current applied brake status
             * \return Converted brake status value
             */
            uint8_t getBrakeAppliedStatus(const double brake);

            /**
             * \brief Function to apply minimum and maximum limits to a vehicle heading value. Minimum limit
             * is 0.0, maximum limit is 359.9875.
             * \param heading The current vehicle heading value
             * \return Vehicle heading value (minimum limit is 0.0, maximum limit is 359.9875)
             */
            float getHeadingInRange(const float heading);

        private:
            // Random number generator for BSM id
            std::default_random_engine generator_;

            // BSM message counter value
            uint8_t msg_count_ {0};

            // Random ID used to generate a new random BSM Message ID
            int random_id_ {0};

            // Variable to track the time that the last randomized BSM Message ID was generated
            rclcpp::Time last_id_generation_time_;

            // Variable to track whether this object's getMsgId() method is being called for the first time, 
            // so that 'last_id_generation_time_' can initialized with the proper time source
            bool first_msg_id_ = true;
    };
} // namespace bsm_generator