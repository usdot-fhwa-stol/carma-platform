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
    void SoftwareOnlyEngagedStateMachine::StartUpState(Signal signal)
    {
        if(signal == Signal::INITIALIZED)
        {
            current_guidance_state = State::DRIVERS_READY;
        } else if(signal == Signal::SHUTDOWN)
        {
            current_guidance_state = State::OFF;
        }
    }

    void SoftwareOnlyEngagedStateMachine::DriversReadyState(Signal signal){
        if(signal == Signal::ENGAGE)
        {
            current_guidance_state = State::ENGAGED;
        } else if(signal == Signal::SHUTDOWN)
        {
            current_guidance_state = State::OFF;
        }
    }

    // This state is not reachable. No need to have any implementations
    void SoftwareOnlyEngagedStateMachine::ActiveState(Signal signal){ }

    void SoftwareOnlyEngagedStateMachine::EngagedState(Signal signal){
        if(signal == Signal::DISENGAGED)
        {
            current_guidance_state = State::DRIVERS_READY;
        } else if(signal == Signal::OVERRIDE)
        {
            current_guidance_state = State::INACTIVE;
        } else if(signal == Signal::SHUTDOWN)
        {
            current_guidance_state = State::OFF;
        }
    }

    void SoftwareOnlyEngagedStateMachine::InactiveState(Signal signal){
        if(signal == Signal::DISENGAGED)
        {
            current_guidance_state = State::DRIVERS_READY;
        } else if(signal == Signal::ENGAGE)
        {
            current_guidance_state = State::ENGAGED;
        } else if(signal == Signal::SHUTDOWN)
        {
            current_guidance_state = State::OFF;
        }
    }

    void SoftwareOnlyEngagedStateMachine::OffState(Signal signal){ }

}
