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

#include "port_drayage_plugin/port_drayage_state_machine.hpp"

namespace port_drayage_plugin
{
    void PortDrayageStateMachine::processEvent(PortDrayageEvent event) {
        switch (state_)
        {
            case PortDrayageState::INACTIVE:
                if (event == PortDrayageEvent::DRAYAGE_START) {
                    state_ = PortDrayageState::EN_ROUTE_TO_INITIAL_DESTINATION;
                }
                break;
            case PortDrayageState::EN_ROUTE_TO_INITIAL_DESTINATION:
                if (event == PortDrayageEvent::ARRIVED_AT_DESTINATION) {
                    if (on_arrived_at_destination_) {
                        on_arrived_at_destination_();
                    }
                    state_ = PortDrayageState::AWAITING_DIRECTION;
                }
                break;
            case PortDrayageState::EN_ROUTE_TO_RECEIVED_DESTINATION:
                if (event == PortDrayageEvent::ARRIVED_AT_DESTINATION) {
                    if (on_arrived_at_destination_) {
                        on_arrived_at_destination_();
                    }
                    state_ = PortDrayageState::AWAITING_DIRECTION;
                }
                break;
            case PortDrayageState::AWAITING_DIRECTION:
                if (event == PortDrayageEvent::RECEIVED_NEW_DESTINATION) {
                    state_ = PortDrayageState::EN_ROUTE_TO_RECEIVED_DESTINATION;
                    if (on_received_new_destination_) {
                        on_received_new_destination_();
                    }
                }
                break;
            default:
                RCLCPP_ERROR_STREAM(logger_->get_logger(), "Unhandled port drayage state: " << state_ << "!");
                throw std::invalid_argument("Unhandled port drayage state");
        }
    }

    PortDrayageState PortDrayageStateMachine::getState() const {
        return state_;
    }

    void PortDrayageStateMachine::setOnSystemStartupCallback(const std::function<void()> &cb) {
        on_system_startup_ = cb;
    }

    void PortDrayageStateMachine::setOnReceivedNewDestinationCallback(const std::function<void()> &cb) {
        on_received_new_destination_ = cb;
    }
    
    void PortDrayageStateMachine::setOnArrivedAtDestinationCallback(const std::function<void()> &cb) {
        on_arrived_at_destination_ = cb;
    }

    void PortDrayageStateMachine::setOnDrayageCompletedCallback(const std::function<void()> &cb) {
        on_drayage_completed_ = cb;
    }
}