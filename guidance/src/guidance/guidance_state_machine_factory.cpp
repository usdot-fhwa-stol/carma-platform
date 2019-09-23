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

#pragma once

#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/RobotEnabled.h>
#include <cav_msgs/GuidanceState.h>
#include "guidance/guidance_state_machine.hpp"
#include "guidance/guidance_state_machine_factory.hpp"

namespace guidance
{
    std::unique_ptr<GuidanceStateMachine> GuidanceStateMachineFactory::createCadilacInstance() {
        std::unique_ptr<GuidanceStateMachine> GuidanceStateMachinePtr(new Cadilac);
        return GuidanceStateMachinePtr;
    }

    std::unique_ptr<GuidanceStateMachine> GuidanceStateMachineFactory::createLexusInstance() {
        std::unique_ptr<GuidanceStateMachine> GuidanceStateMachinePtr(new Lexus);
        return GuidanceStateMachinePtr;
    }

}