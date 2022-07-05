/*
 * Copyright (C) 2018-2022 LEIDOS.
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

#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <carma_driver_msgs/msg/robot_enabled.hpp>
#include <carma_planning_msgs/msg/route_event.hpp>
#include <autoware_msgs/msg/vehicle_status.hpp>

#include <rclcpp/rclcpp.hpp>

namespace guidance
{
    class GuidanceStateMachine
    {
        public:
        
            enum Signal
            {
                INITIALIZED = 0,
                ACTIVATED = 1,
                ENGAGE = 2,
                DISENGAGED = 3,
                SHUTDOWN = 4,
                OVERRIDE = 5,
                PARK = 6,
            };

            enum State
            {
                OFF = 0,
                STARTUP = 1,
                DRIVERS_READY = 2,
                ACTIVE = 3,
                ENGAGED = 4,
                INACTIVE = 5,
                ENTER_PARK = 6,
            };

            /*!
             * \brief constructor for GuidanceStateMachine
             * \param logger The logger interface that will be used by this object
             */
            GuidanceStateMachine(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger);

            /*!
             * \brief Handle vehicle status message from ROS network.
             */
            void onVehicleStatus(autoware_msgs::msg::VehicleStatus::UniquePtr msg);

            /*!
             * \brief Updates Guidance State Machine with INITIALIZED signal
             */
            void onGuidanceInitialized();

            /*!
             * \brief Updates Guidance State Machine with SHUTDOWN signal
             */
            void onGuidanceShutdown();

            /*!
             * \brief Handle set_guidance_active service call from ROS network.
             */
            void onSetGuidanceActive(bool msg);

            /*!
             * \brief Handle robotic_status message from ROS network.
             */
            void onRoboticStatus(carma_driver_msgs::msg::RobotEnabled::UniquePtr msg);

            /*!
             * \brief Handle route event message.
             */
            void onRouteEvent(carma_planning_msgs::msg::RouteEvent::UniquePtr msg);

            /*!
             * \brief Indicate if SetEnableRobotic needs to be called in ACTIVE state.
             */
            bool shouldCallSetEnableRobotic();

            /*!
             * \brief Get current state machine status.
             */
            uint8_t getCurrentState();

        private:

            /*!
             * \brief Actual state machine logic driven by signal enum.
             */
            void onGuidanceSignal(Signal signal);

            // a local variable keeps the current state machine state
            State current_guidance_state_ {State::STARTUP};

            // Previous robotic active status
            bool robotic_active_status_{false};

            // make one service call in ACTIVE state to engage
            bool called_robotic_engage_in_active_{false};

            // Current vehicle speed in m/s. Used to handle end of route state transition.
            double current_velocity_ = 0.0;

            // Logger interface
            rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

    };

} // guidance