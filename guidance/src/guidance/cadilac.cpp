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
    void Cadilac::StartUpState(Signal signal)
    {
        if(signal == Signal::INITIALIZED)
        {
            currentGuidanceState = State::DRIVERS_READY;
        } else if(signal == Signal::SHUTDOWN)
        {
            currentGuidanceState = State::OFF;
        }
    }

    void Cadilac::DriversReadyState(Signal signal){
        if(signal == Signal::ACTIVATED)
        {
            currentGuidanceState = State::ACTIVE;
        } else if(signal == Signal::SHUTDOWN)
        {
            currentGuidanceState = State::OFF;
        }
    }

    void Cadilac::ActiveState(Signal signal){
        if(signal == Signal::ENGAGE)
        {
            currentGuidanceState = State::ENGAGED;
        } else if(signal == Signal::DISENGAGED)
        {
            currentGuidanceState = State::DRIVERS_READY;
        } else if(signal == Signal::SHUTDOWN)
        {
            currentGuidanceState = State::OFF;
        }
    }

    void Cadilac::EngagedState(Signal signal){
        if(signal == Signal::DISENGAGED)
        {
            currentGuidanceState = State::DRIVERS_READY;
        } else if(signal == Signal::OVERRIDE)
        {
            currentGuidanceState = State::INACTIVE;
        } else if(signal == Signal::SHUTDOWN)
        {
            currentGuidanceState = State::OFF;
        }
    }

    void Cadilac::InactiveState(Signal signal){
        if(signal == Signal::DISENGAGED)
        {
            currentGuidanceState = State::DRIVERS_READY;
        } else if(signal == Signal::ENGAGE)
        {
            currentGuidanceState = State::ENGAGED;
        } else if(signal == Signal::SHUTDOWN)
        {
            currentGuidanceState = State::OFF;
        }
    }

    void Cadilac::OffState(Signal signal){
    }

}
