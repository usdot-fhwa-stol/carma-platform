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

#include "arbitrator_state_machine.hpp"
#include <iostream>

namespace arbitrator
{

    ArbitratorState ArbitratorStateMachine::get_state()
    {
        return current_state;        
    }


    ArbitratorState ArbitratorStateMachine::submit_event(ArbitratorEvent event)
    {
        if (event == SYSTEM_STARTUP_COMPLETE)
        {
            std::cerr <<"GOT EVENT SYSTEM_STARTUP_COMPLETE " << std::endl;
        }
        else
        {
            std::cerr <<"GOOT UNKNOWN EVENT" << std::endl;
        }
        for (auto iter = ARBITRATOR_TRANSITIONS.begin(); iter != ARBITRATOR_TRANSITIONS.end(); iter++) 
        {
            if (current_state == iter->current_state && event == iter->input_event) {
                // In the event of multiple legal transistions this will take the last transition,
                // But ultimately this condition should never arise.
                current_state = iter->final_state;
                if (iter->final_state == PLANNING)
                {
                    std::cerr <<"CHANGED TO PLANNING state" << std::endl;
                }
                else if (iter->final_state == PAUSED)
                {
                    std::cerr <<"CHANGED TO PAUSED state" << std::endl;
                }
                else if (iter->final_state == WAITING)
                {
                    std::cerr <<"CHANGED TO WAITING state" << std::endl;
                }
                else if (iter->final_state == SHUTDOWN)
                {
                    std::cerr <<"CHANGED TO SHUTDOWN state" << std::endl;
                }
                else if (iter->final_state == INITIAL)
                {
                    std::cerr <<"CHANGED TO INITIAL state" << std::endl;
                }
                break;
            }
        }

        return current_state;
    }

};