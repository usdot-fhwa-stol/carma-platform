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

#include "wz_strategic_plugin/wz_states.h"

namespace wz_strategic_plugin
{
std::ostream& operator<<(std::ostream& os, TransitState s)
{
  os << "TransitState::";
  switch (s)
  {  // clang-format off
    case TransitState::UNAVAILABLE   : os << "UNAVAILABLE"; break;
    case TransitState::APPROACHING: os << "APPROACHING"; break;
    case TransitState::WAITING : os << "WAITING"; break;
    case TransitState::DEPARTING  : os << "DEPARTING"; break;
    default: os.setstate(std::ios_base::failbit);
  }  // clang-format on
  return os;
}

std::ostream& operator<<(std::ostream& os, TransitEvent s)
{
  os << "TransitEvent::";
  switch (s)
  {  // clang-format off
    case TransitEvent::IN_STOPPING_RANGE   : os << "IN_STOPPING_RANGE"; break;
    case TransitEvent::STOPPED: os << "STOPPED"; break;
    case TransitEvent::CROSSED_STOP_BAR : os << "CROSSED_STOP_BAR"; break;
    case TransitEvent::RED_TO_GREEN_LIGHT  : os << "RED_TO_GREEN_LIGHT"; break;
    case TransitEvent::INTERSECTION_EXIT  : os << "INTERSECTION_EXIT"; break;
    default: os.setstate(std::ios_base::failbit);
  }  // clang-format on
  return os;
}

}  // namespace wz_strategic_plugin