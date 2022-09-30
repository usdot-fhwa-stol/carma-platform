#pragma once

/*
 * Copyright (C) 2020-2022 LEIDOS.
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

#include <rclcpp/rclcpp.hpp>
#include <functional>

namespace port_drayage_plugin
{
    /**
     * \brief Enum containing the possible state values for the PortDrayagePlugin
     * 
     * INACTIVE = PortDrayagePlugin is not currently operating in an active
     *  capacity
     * EN_ROUTE_TO_INITIAL_DESTINATION = The vehicle is currently enroute  to its initial
     *   destination, the type of which is based on the configuration parameters for this node
     * EN_ROUTE_TO_RECEIVED_DESTINATION = The vehicle is currently enroute to a destination
     *   that was received from infrastructure
     * AWAITING_DIRECTION = The vehicle is currently stopped and is awaiting further guidance from the 
     *  port infrastructure to tell it the next destination.
     */ 
    enum PortDrayageState
    {
        INACTIVE,
        EN_ROUTE_TO_INITIAL_DESTINATION,
        EN_ROUTE_TO_RECEIVED_DESTINATION,
        AWAITING_DIRECTION
    };

    /**
     * \brief Enum containing the event values that cause changes in the PortDrayagePlugin
     * state machine.
     * 
     * DRAYAGE_START = Drayage operations have been commenced by the vehicle
     * RECEIVED_NEW_DESTINATION = The port has commanded the vehicle to travel
     *   to a new destination.
     * ARRIVED_AT_DESTINATION = The vehicle has arrived at it's destination.
     * DRAYAGE_COMPLETED = The drayage operation is complete and the vehicle has
     *  exited the port area.
     */
    enum PortDrayageEvent
    {
        DRAYAGE_START,
        RECEIVED_NEW_DESTINATION,
        ARRIVED_AT_DESTINATION,
        DRAYAGE_COMPLETED
    };

    /**
     * \Brief An implementation of the state machine for the PortDrayagePlugin.
     * 
     * The state machine accepts events as inputs which drive the internal state
     * changes. This state can be queried at any given point in time, but also
     * each transition into a state has an associated callback which will be 
     * invoked when that state is entered.
     */
    class PortDrayageStateMachine
    {
        private:
            PortDrayageState state_;
            std::function<void()> on_system_startup_;
            std::function<void()> on_received_new_destination_;
            std::function<void()> on_arrived_at_destination_;
            std::function<void()> on_drayage_completed_;

            // Logger interface for this object
            rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

        public:
            /**
             * \brief Constructor the PortDrayageStateMachine.
             * 
             * The system initially starts in INACTIVE state.
             */
            PortDrayageStateMachine(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger) :
                logger_(logger),
                state_(PortDrayageState::INACTIVE) {};
        
            /**
             * \Brief Inform the state machine that an event has transpired.
             * 
             * \param event The event that took place.
             */
            void processEvent(PortDrayageEvent event);

            /**
             * \brief Get the current state of the state machine
             * \return The current state value
             */
            PortDrayageState getState() const;

            /**
             * \brief Set the callback to be invoked upon transitioning out of the
             * inactive state.
             * \param cb The std::function callback object
             */
            void setOnSystemStartupCallback(const std::function<void()> &cb);

            /**
             * \brief Set the callback to be invoked upon transitioning into the 
             * EN_ROUTE state
             * \param cb The std::function callback object
             */
            void setOnReceivedNewDestinationCallback(const std::function<void()> &cb);

            /**
             * \brief Set the callback to be invoked upon transitioning into the 
             * AWAITING_DESTINATION state
             * \param cb The std::function callback object
             */
            void setOnArrivedAtDestinationCallback(const std::function<void()> &cb);

            /**
             * \brief Set the callback to be invoked upon transitioning into the
             * inactive state.
             * \param cb The std::function callback object
             */
            void setOnDrayageCompletedCallback(const std::function<void()> &cb);
    };
} // namespace port_drayage_plugin