/*
 * Copyright (C) 2018-2019 LEIDOS.
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

#include "guidance/guidance_state_machine.hpp"

namespace guidance
{
    void GuidanceStateMachine::onGuidanceSignal(Signal signal)
    {
        switch(currentGuidanceState)
        {
            case State::STARTUP:
                StartUpState(signal);
                break;
            case State::DRIVERS_READY:
                DriversReadyState(signal);
                break;
            case State::ACTIVE:
                ActiveState(signal);
                break;
            case State::ENGAGED:
                EngagedState(signal);
                break;
            case State::INACTIVE:
                InactiveState(signal);
                break;
            case State::OFF:
                OffState(signal);
                break;
        }
    }

    void GuidanceStateMachine::onSystemAlert(const cav_msgs::SystemAlertConstPtr& msg)
    {
        if(msg->type == msg->DRIVERS_READY)
        {
            onGuidanceSignal(Signal::INITIALIZED);
        } else if(msg->type == msg->SHUTDOWN || msg->type == msg->FATAL)
        {
            onGuidanceSignal(Signal::SHUTDOWN);
        }
    }

    void GuidanceStateMachine::onSetGuidanceActive(bool msg)
    {
        if(msg)
        {
            onGuidanceSignal(Signal::ACTIVATED);
        } else
        {
            onGuidanceSignal(Signal::DISENGAGED);
        }
        
    }

    void GuidanceStateMachine::onRoboticStatus(const cav_msgs::RobotEnabledConstPtr& msg)
    {
        if(msg->robot_enabled && msg->robot_active)
        {
            onGuidanceSignal(Signal::ENGAGE);
        } else if(msg->robot_enabled && !msg->robot_active)
        {
            onGuidanceSignal(Signal::OVERRIDE);
        }
    }

    uint8_t GuidanceStateMachine::getCurrentState()
    {
        return static_cast<uint8_t>(currentGuidanceState);
    }

    GuidanceStateMachine::GuidanceStateMachine() : currentGuidanceState(State::STARTUP) {}

}
