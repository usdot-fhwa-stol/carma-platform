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
        std:: cerr << "SIZE: " << ARBITRATOR_TRANSITIONS.size() << std::endl;
        for (auto pair : ARBITRATOR_TRANSITIONS) 
        {
            if (current_state == INITIAL)
            {
                std::cerr << "1. INITIAL!!!" << std::endl;
            }
            else if (current_state == PLANNING)
            {
                std::cerr << "1. PLANNING!!!" << std::endl;
            }
            else if (current_state == WAITING)
            {
                std::cerr << "1. WAITING!!!" << std::endl;
            }
            else if (current_state == PAUSED)
            {
                std::cerr << "1. PAUSED!!!" << std::endl;
            }
            else if (current_state == SHUTDOWN)
            {
                std::cerr << "1. SHUTDOWN!!!" << std::endl;
            }
            else
            {
                std::cerr << "1. STH ELSEE!!!!!!!" << std::endl;
            }

            if (event == SYSTEM_STARTUP_COMPLETE)
            {
                std::cerr << "1. SYSTEM_STARTUP_COMPLETE!!!" << std::endl;
            }
            else if (event == PLANNING_COMPLETE)
            {
                std::cerr << "1. PLANNING_COMPLETE!!!" << std::endl;
            }
            else if (event == PLANNING_TIMER_TRIGGER)
            {
                std::cerr << "1. PLANNING_TIMER_TRIGGER!!!" << std::endl;
            }
            else if (event == ARBITRATOR_PAUSED)
            {
                std::cerr << "1. ARBITRATOR_PAUSED!!!" << std::endl;
            }
            else if (event == ARBITRATOR_RESUMED)
            {
                std::cerr << "1. ARBITRATOR_RESUMED!!!" << std::endl;
            }
            else if (event == SYSTEM_SHUTDOWN_INITIATED)
            {
                std::cerr << "1. SYSTEM_SHUTDOWN_INITIATED!!!" << std::endl;
            }
            else
            {
                std::cerr << "1. STH ELSE" << std::endl;
            }
            if (pair.current_state == ArbitratorState::INITIAL)
            {
                std::cerr << "1.a. INITIAL!!!" << std::endl;
            }
            else if (pair.current_state == PLANNING)
            {
                std::cerr << "1.a. PLANNING!!!" << std::endl;
            }
            else if (pair.current_state == WAITING)
            {
                std::cerr << "1.a. WAITING!!!" << std::endl;
            }
            else if (pair.current_state == PAUSED)
            {
                std::cerr << "1.a. PAUSED!!!" << std::endl;
            }
            else if (pair.current_state == SHUTDOWN)
            {
                std::cerr << "1.a. SHUTDOWN!!!" << std::endl;
            }
            else
            {
                std::cerr << "1.a. STH ELSEE!!!!!!!" << std::endl;
            }

            if (pair.input_event == ArbitratorEvent::SYSTEM_STARTUP_COMPLETE)
            {
                std::cerr << "1.a. SYSTEM_STARTUP_COMPLETE!!!" << std::endl;
            }
            else if (pair.input_event == PLANNING_COMPLETE)
            {
                std::cerr << "1.a. PLANNING_COMPLETE!!!" << std::endl;
            }
            else if (pair.input_event == PLANNING_TIMER_TRIGGER)
            {
                std::cerr << "1.a. PLANNING_TIMER_TRIGGER!!!" << std::endl;
            }
            else if (pair.input_event == ARBITRATOR_PAUSED)
            {
                std::cerr << "1.a. ARBITRATOR_PAUSED!!!" << std::endl;
            }
            else if (pair.input_event == ARBITRATOR_RESUMED)
            {
                std::cerr << "1.a. ARBITRATOR_RESUMED!!!" << std::endl;
            }
            else if (pair.input_event == SYSTEM_SHUTDOWN_INITIATED)
            {
                std::cerr << "1.a. SYSTEM_SHUTDOWN_INITIATED!!!" << std::endl;
            }
            else
            {
                std::cerr << "1.a. STH ELSE" << std::endl;
            }

            if (current_state == pair.current_state && event == pair.input_event) 
            {
                // In the event of multiple legal transistions this will take the last transition,
                // But ultimately this condition should never arise.
                current_state = pair.final_state;

                if (current_state == INITIAL)
                {
                    std::cerr << "2. INITIAL!!!" << std::endl;
                }
                else if (current_state == PLANNING)
                {
                    std::cerr << "2. PLANNING!!!" << std::endl;
                }
                else if (current_state == WAITING)
                {
                    std::cerr << "2. WAITING!!!" << std::endl;
                }
                else if (current_state == PAUSED)
                {
                    std::cerr << "2. PAUSED!!!" << std::endl;
                }
                else if (current_state == SHUTDOWN)
                {
                    std::cerr << "2. SHUTDOWN!!!" << std::endl;
                }
                else
                {
                    std::cerr << "2. STH ELSEE!!!!!!!" << std::endl;
                }
                break;
            }
        }

        return current_state;
    }

};