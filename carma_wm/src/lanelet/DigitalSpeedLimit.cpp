/*
 * Copyright (C) 2020 LEIDOS.
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
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <boost/algorithm/string.hpp>
#include <carma_wm/lanelet/DigitalSpeedLimit.h>
#include "RegulatoryHelpers.h"

namespace lanelet
{
// C++ 14 vs 17 constant defintion
#if __cplusplus < 201703L
// Forward declare static constexpr
constexpr char DigitalSpeedLimit::RuleName[];  // instantiate string in cpp file
constexpr char DigitalSpeedLimit::Limit[];
#endif

ConstLanelets DigitalSpeedLimit::getLanelets() const
{
  return getParameters<ConstLanelet>(RoleName::Refers);
}

ConstAreas DigitalSpeedLimit::getAreas() const
{
  return getParameters<ConstArea>(RoleName::Refers);
}

Velocity DigitalSpeedLimit::getSpeedLimit() const
{
  return speed_limit_;
}

bool DigitalSpeedLimit::appliesTo(const std::string& participant) const
{
  return setContainsParticipant(participants_, participant);
}

DigitalSpeedLimit::DigitalSpeedLimit(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data)
{
  // Read participants
  addParticipantsToSetFromMap(participants_, attributes());

  // Read speed limit
  auto optional_speed_limit = attribute(Limit).asVelocity();

  if (!optional_speed_limit)
  {
    throw std::invalid_argument("Limit attribute of DigitalSpeedLimit regulatory element is not set or cannot be "
                                "read ");
  }

  speed_limit_ = *optional_speed_limit;
}

lanelet::RegulatoryElementDataPtr DigitalSpeedLimit::buildData(Id id, Velocity speed_limit, Lanelets lanelets,
                                                               Areas areas, std::vector<std::string> participants)
{
  // Add parameters
  RuleParameterMap rules;
  rules[lanelet::RoleNameString::Refers].insert(rules[lanelet::RoleNameString::Refers].end(), lanelets.begin(),
                                                lanelets.end());
  rules[lanelet::RoleNameString::Refers].insert(rules[lanelet::RoleNameString::Refers].end(), areas.begin(),
                                                areas.end());

  // Add attributes
  AttributeMap attribute_map({ { AttributeNamesString::Type, AttributeValueString::RegulatoryElement },
                               { AttributeNamesString::Subtype, RuleName },
                               { Limit, Attribute(speed_limit).value() } });

  for (auto participant : participants)
  {
    const std::string key = std::string(AttributeNamesString::Participant) + ":" + participant;
    attribute_map[key] = "yes";
  }

  return std::make_shared<RegulatoryElementData>(id, rules, attribute_map);
}

namespace
{
// this object actually does the registration work for us
lanelet::RegisterRegulatoryElement<lanelet::DigitalSpeedLimit> reg;
}  // namespace

}  // namespace lanelet