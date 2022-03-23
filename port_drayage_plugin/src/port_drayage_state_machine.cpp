/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include <ros/ros.h>
#include "port_drayage_plugin/port_drayage_state_machine.h"
#include <exception>

namespace port_drayage_plugin
{
    void PortDrayageStateMachine::process_event(PortDrayageEvent event) {
        switch (_state)
        {
            case PortDrayageState::INACTIVE:
                if (event == PortDrayageEvent::DRAYAGE_START) {
                    _state = PortDrayageState::EN_ROUTE_TO_INITIAL_DESTINATION;
                }
                break;
            case PortDrayageState::EN_ROUTE_TO_INITIAL_DESTINATION:
                if (event == PortDrayageEvent::ARRIVED_AT_DESTINATION) {
                    if (_on_arrived_at_destination) {
                        _on_arrived_at_destination();
                    }
                    _state = PortDrayageState::AWAITING_DIRECTION;
                }
                break;
            case PortDrayageState::EN_ROUTE_TO_RECEIVED_DESTINATION:
                if (event == PortDrayageEvent::ARRIVED_AT_DESTINATION) {
                    if (_on_arrived_at_destination) {
                        _on_arrived_at_destination();
                    }
                    _state = PortDrayageState::AWAITING_DIRECTION;
                }
                break;
            case PortDrayageState::AWAITING_DIRECTION:
                if (event == PortDrayageEvent::RECEIVED_NEW_DESTINATION) {
                    _state = PortDrayageState::EN_ROUTE_TO_RECEIVED_DESTINATION;
                    if (_on_received_new_destination) {
                        _on_received_new_destination();
                    }
                }
                break;
            default:
                ROS_ERROR_STREAM("Unhandled port drayage state: " << _state << "!");
                throw std::invalid_argument("Unhandled port drayage state");
        }
    }

    PortDrayageState PortDrayageStateMachine::get_state() const {
        return _state;
    }

    void PortDrayageStateMachine::set_on_system_startup_callback(const std::function<void()> &cb) {
        _on_system_startup = cb;
    }

    void PortDrayageStateMachine::set_on_received_new_destination_callback(const std::function<void()> &cb) {
        _on_received_new_destination = cb;
    }
    
    void PortDrayageStateMachine::set_on_arrived_at_destination_callback(const std::function<void()> &cb) {
        _on_arrived_at_destination = cb;
    }

    void PortDrayageStateMachine::set_on_drayage_completed_callback(const std::function<void()> &cb) {
        _on_drayage_completed = cb;
    }
}
