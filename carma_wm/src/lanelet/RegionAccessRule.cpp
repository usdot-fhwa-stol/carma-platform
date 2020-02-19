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
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Area.h>
#include <boost/algorithm/string.hpp>
#include <lanelet2_core/Forward.h>
#include <carma_wm/lanelet/RegionAccessRule.h>
#include "RegulatoryHelpers.h"

namespace lanelet
{
// C++ 14 vs 17 constent definition
#if __cplusplus < 201703L
constexpr char RegionAccessRule::RuleName[];  // instantiate string in cpp file
#endif

ConstLanelets RegionAccessRule::getLanelets() const
{
  return getParameters<ConstLanelet>(RoleName::Refers);
}

ConstAreas RegionAccessRule::getAreas() const
{
  return getParameters<ConstArea>(RoleName::Refers);
}

bool RegionAccessRule::accessable(const std::string& participant) const
{
  return setContainsParticipant(participants_, participant);
}

RegionAccessRule::RegionAccessRule(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data)
{
  // Read participants
  addParticipantsToSetFromMap(participants_, attributes());
}

lanelet::RegulatoryElementDataPtr RegionAccessRule::buildData(Id id, Lanelets lanelets, Areas areas,
                                                              std::vector<std::string> participants)
{
  // Add parameters
  RuleParameterMap rules;

  rules[lanelet::RoleNameString::Refers].insert(rules[lanelet::RoleNameString::Refers].end(), lanelets.begin(),
                                                lanelets.end());
  rules[lanelet::RoleNameString::Refers].insert(rules[lanelet::RoleNameString::Refers].end(), areas.begin(),
                                                areas.end());

  // Add attributes
  AttributeMap attribute_map({ { AttributeNamesString::Type, AttributeValueString::RegulatoryElement },
                               { AttributeNamesString::Subtype, RuleName } });

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
lanelet::RegisterRegulatoryElement<lanelet::RegionAccessRule> reg;
}  // namespace
}  // namespace lanelet
