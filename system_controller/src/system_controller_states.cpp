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

#include "system_controller/system_controller_states.h"

namespace system_controller
{
  std::ostream &operator<<(std::ostream &os, SystemState s)
  {
    os << "SystemState::";
    switch (s)
    { // clang-format off
    case SystemState::STARTING_UP   : os << "STARTING_UP"; break;
    case SystemState::INACTIVE: os << "INACTIVE"; break;
    case SystemState::ACTIVE : os << "ACTIVE"; break;
    case SystemState::FINALIZED  : os << "FINALIZED"; break;
    default: os.setstate(std::ios_base::failbit);
  } // clang-format on
    return os;
  }

  std::ostream &operator<<(std::ostream &os, SystemEvent s)
  {
    os << "SystemEvent::";
    switch (s)
    { // clang-format off
    case SystemEvent::STARTUP_DELAY_EXCEEDED   : os << "STARTUP_DELAY_EXCEEDED"; break;
    case SystemEvent::INTERNAL_FAULT: os << "INTERNAL_FAULT"; break;
    case SystemEvent::SUBSYSTEM_FAULT : os << "SUBSYSTEM_FAULT"; break;
    case SystemEvent::EXTERNAL_SHUTDOWN  : os << "EXTERNAL_SHUTDOWN"; break;
    default: os.setstate(std::ios_base::failbit);
  } // clang-format on
    return os;
  }

} // namespace system_controller