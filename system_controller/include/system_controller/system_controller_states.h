#pragma once
/*
 * Copyright (C) 2021 LEIDOS.
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

#include <ostream>

namespace system_controller
{

//! @brief Enum describing the possible states of the WorkZone Strategic Plugin
enum class SystemState
{
    STARTING_UP,
    ACTIVE,
    ERROR_PROCESSING,
    SHUTTING_DOWN,
    FINALIZED
};
/**
 * \brief Stream operator for SystemStates enum.
 */
std::ostream& operator<<(std::ostream& os, SystemState s);


//! @brief Enum describing the possible signals to change the current SystemState
enum class SystemEvent
{
  STARTUP_DELAY_EXCEEDED,
  INTERNAL_FAULT,
  SUBSYSTEM_FAULT,
  EXTERNAL_SHUTDOWN,
  SHUTDOWN_COMPLETED,
  SHUTDOWN_ERROR
};

/**
 * \brief Stream operator for SystemEvent enum.
 */
std::ostream& operator<<(std::ostream& os, SystemEvent s);

}  // namespace localizer